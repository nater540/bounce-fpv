//! truck-diag :: servo — gimbal-servo sweep, in ISOLATION from the rest of the truck node (Phase C: nRF52840).
//!
//! Builds two 50 Hz `SimplePwm` outputs (PWM0 for pan, PWM1 for tilt — one independent PWM peripheral per servo, as
//! `nrf_adapters::PwmServoChannel` requires), wraps each in `PwmServoChannel` -> `servo::Servo`, and runs a slow,
//! visible sweep on BOTH servos: center (1500 us) -> min (1000 us) -> max (2000 us) -> center, dwelling at each
//! waypoint and stepping smoothly between them. Each step prints the commanded pulse width over USB-CDC (applog) so
//! the serial log mirrors what the gimbal should be doing.
//!
//! GOOD: both servos sweep smoothly through their full travel and re-center, the printed pulse widths advance
//! monotonically through each leg, and there is no buzzing/stalling at the endpoints. FAILURE / wiring signature:
//! a servo that twitches, hums, or does not move usually means the signal pin is on the wrong GPIO, the servo is on
//! the 3.3 V pin (too weak — see the banner warning) instead of a 5 V rail, or grounds are not common.
//!
//! POWER WARNING (per CLAUDE.md / the migration doc): power the MG90S servos from a DEDICATED 5 V rail that shares
//! ground with the nRF — NEVER the 3.3 V pin. The 3.3 V regulator cannot source the stall current and the brownout
//! will reset the board.

#![no_std]
#![no_main]

// Pull in the shared #[panic_handler] from applog (replaces the old truck-diag lib panic handler + esp app_desc).
use applog as _;

use embassy_executor::Spawner;
use embassy_nrf::pwm::SimplePwm;
use embassy_time::{Duration, Timer};
use nrf_adapters::PwmServoChannel;
use servo::Servo;

// Servo frame rate (Hz), for the banner. The actual 50 Hz timing is fixed by PwmServoChannel::servo_config (Div16
// prescaler + 20_000-tick countertop = 20 ms), so the duty math (1 tick = 1 us) lines up with the servo crate.
const SERVO_FREQ_HZ: u32 = 50;

// Sweep waypoints (microseconds) and stepping. TODO: tune — widen toward the MG90S 400-2400 us mechanical limits
// only after confirming the gimbal does not bind, and shrink STEP_US / lengthen STEP_DELAY if the motion is jerky.
const MIN_US: u16 = 1_000;
const CENTER_US: u16 = 1_500;
const MAX_US: u16 = 2_000;
const STEP_US: u16 = 25; //  TODO: tune — smaller = smoother but slower visible sweep.
const STEP_DELAY: Duration = Duration::from_millis(20); //  TODO: tune — pacing between micro-steps.
const DWELL: Duration = Duration::from_millis(800); //  TODO: tune — pause at each waypoint so motion is obvious.

#[embassy_executor::main]
async fn main(spawner: Spawner) {
  // embassy-nrf init at SoftDevice-safe interrupt priorities (GPIOTE + time-driver at P2; the SD reserves P0/P1/P4).
  let p = applog::init_embassy_nrf();
  // board_pins! partial-moves only the GPIO pin fields, leaving PWM0 / PWM1 / USBD on `p`.
  let pins = board::board_pins!(p);

  applog::init(
    spawner,
    p.USBD,
    applog::UsbIdentity::new(0x1209, 0x0001, "fabulous-fpv", "truck-diag-servo", "phase-c"),
  );

  applog::log_println!("");
  applog::log_println!("=== truck-diag/servo: gimbal-servo sweep (nRF52840) ===");
  applog::log_println!(
    "pan servo on P{}.{:02} (PWM0), tilt servo on P{}.{:02} (PWM1), {} Hz",
    board::SERVO_PAN_PORT, board::SERVO_PAN_PIN, board::SERVO_TILT_PORT, board::SERVO_TILT_PIN, SERVO_FREQ_HZ
  );
  applog::log_println!("POWER: drive the MG90S from a DEDICATED 5 V rail with COMMON GROUND to the nRF — NOT 3.3 V.");
  applog::log_println!(
    "sweep: center {}us -> min {}us -> max {}us -> center, step {}us",
    CENTER_US, MIN_US, MAX_US, STEP_US
  );
  applog::log_println!("");

  // SD COEXISTENCE: SimplePwm is fire-and-forget (each duty is a register/DMA write; it never awaits a PWM IRQ), so
  // PWM0/PWM1 raise no interrupt and need no priority lowering — unlike the SPIM/TWIM/UARTE bins. One PWM peripheral
  // per servo because PwmServoChannel owns the whole SimplePwm and targets channel 0 of it.
  let cfg = PwmServoChannel::servo_config();
  let pan_pwm = SimplePwm::new_1ch(p.PWM0, pins.servo_pan, &cfg);
  let tilt_pwm = SimplePwm::new_1ch(p.PWM1, pins.servo_tilt, &cfg);

  let mut pan = Servo::new(PwmServoChannel::new(pan_pwm, 0));
  let mut tilt = Servo::new(PwmServoChannel::new(tilt_pwm, 0));
  applog::log_println!("PWM ready: max_duty {} (1 tick = 1 us). Beginning sweep.", pan.max_duty());
  applog::log_println!("");

  // The sweep legs, in order: home to center, down to min, up across to max, then back to center. Each leg ramps in
  // STEP_US increments with a short pace, then dwells so the eye can register the endpoint.
  loop {
    sweep_leg(&mut pan, &mut tilt, CENTER_US, CENTER_US).await;
    Timer::after(DWELL).await;
    sweep_leg(&mut pan, &mut tilt, CENTER_US, MIN_US).await;
    Timer::after(DWELL).await;
    sweep_leg(&mut pan, &mut tilt, MIN_US, MAX_US).await;
    Timer::after(DWELL).await;
    sweep_leg(&mut pan, &mut tilt, MAX_US, CENTER_US).await;
    Timer::after(DWELL).await;
  }
}

/// Ramps both servos from `from_us` to `to_us` in STEP_US steps (direction-aware), driving pan and tilt to the same
/// width each step and printing the commanded pulse so the serial log tracks the visible motion. Generic over the
/// concrete PWM channel type so a single helper drives both Servo instances.
async fn sweep_leg<C>(pan: &mut Servo<C>, tilt: &mut Servo<C>, from_us: u16, to_us: u16)
where
  C: embedded_hal::pwm::SetDutyCycle,
{
  // Step toward the target in either direction, inclusive of the endpoint, clamping the final partial step so we land
  // exactly on `to_us` rather than overshooting by a fraction of STEP_US.
  if to_us >= from_us {
    let mut us = from_us;
    loop {
      drive(pan, tilt, us);
      if us == to_us {
        break;
      }
      us = (us + STEP_US).min(to_us);
      Timer::after(STEP_DELAY).await;
    }
  } else {
    let mut us = from_us;
    loop {
      drive(pan, tilt, us);
      if us == to_us {
        break;
      }
      us = us.saturating_sub(STEP_US).max(to_us);
      Timer::after(STEP_DELAY).await;
    }
  }
}

/// Commands both servos to the same pulse width and logs it. set_pulse_us drives a raw pulse (the sweep stays within
/// 1000-2000 us by construction); a duty error is logged rather than panicked so the sweep keeps running.
fn drive<C>(pan: &mut Servo<C>, tilt: &mut Servo<C>, us: u16)
where
  C: embedded_hal::pwm::SetDutyCycle,
{
  applog::log_println!("pulse {} us", us);
  if pan.set_pulse_us(us).is_err() {
    applog::log_println!("pan servo duty error");
  }
  if tilt.set_pulse_us(us).is_err() {
    applog::log_println!("tilt servo duty error");
  }
}
