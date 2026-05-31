//! truck-diag :: servo — LEDC gimbal-servo sweep, in ISOLATION from the rest of the truck node.
//!
//! Builds the same 50 Hz LowSpeed LEDC timer + two channels on `servo_pan` / `servo_tilt` that truck-node uses,
//! wraps each in `servo::Servo`, and runs a slow, visible sweep on BOTH servos: center (1500 us) -> min
//! (1000 us) -> max (2000 us) -> center, dwelling at each waypoint and stepping smoothly between them. Each step
//! prints the commanded pulse width over USB Serial/JTAG so the serial log mirrors what the gimbal should be doing.
//!
//! GOOD: both servos sweep smoothly through their full travel and re-center, the printed pulse widths advance
//! monotonically through each leg, and there is no buzzing/stalling at the endpoints. FAILURE / wiring signature:
//! a servo that twitches, hums, or does not move usually means the signal pin is on the wrong GPIO, the servo is
//! on the 3.3 V pin (too weak — see the banner warning) instead of a 5 V rail, or grounds are not common.
//!
//! POWER WARNING (per CLAUDE.md / the overview): power the MG90S servos from a DEDICATED 5 V rail that shares
//! ground with the C6 — NEVER the 3.3 V pin. The 3.3 V regulator cannot source the stall current and the brownout
//! will reset the C6.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::DriveMode;
use esp_hal::ledc::channel::config::Config as ChannelConfig;
use esp_hal::ledc::channel::{self, ChannelIFace};
use esp_hal::ledc::timer::config::Config as LedcTimerConfig;
use esp_hal::ledc::timer::{self, TimerIFace};
use esp_hal::ledc::{LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use servo::Servo;

// Pull in the shared #[panic_handler] + esp_app_desc!() (defined in the crate lib so all four bins reuse them).
use truck_diag as _;

// LEDC servo timer: 50 Hz, 14-bit duty — same as truck-node so the duty math (servo::pulse_us_to_duty) matches.
const SERVO_FREQ_HZ: u32 = 50;

// Sweep waypoints (microseconds) and stepping. TODO: tune — widen toward the MG90S 400-2400 us mechanical limits
// only after confirming the gimbal does not bind, and shrink STEP_US / lengthen STEP_DELAY if the motion is jerky.
const MIN_US: u16 = 1_000;
const CENTER_US: u16 = 1_500;
const MAX_US: u16 = 2_000;
const STEP_US: u16 = 25; //  TODO: tune — smaller = smoother but slower visible sweep.
const STEP_DELAY: Duration = Duration::from_millis(20); //  TODO: tune — pacing between micro-steps.
const DWELL: Duration = Duration::from_millis(800); //  TODO: tune — pause at each waypoint so motion is obvious.

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
  let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
  let peripherals = esp_hal::init(config);

  // board_pins! partial-moves only the GPIO pin fields, leaving LEDC / TIMG0 / SW_INTERRUPT on `peripherals`.
  let pins = board::board_pins!(peripherals);

  let timg0 = TimerGroup::new(peripherals.TIMG0);
  let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
  esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

  println!();
  println!("=== truck-diag/servo: LEDC gimbal-servo sweep ===");
  println!("pan servo on GPIO{}, tilt servo on GPIO{}, {} Hz", board::SERVO_PAN, board::SERVO_TILT, SERVO_FREQ_HZ);
  println!("POWER: drive the MG90S from a DEDICATED 5 V rail with COMMON GROUND to the C6 — NOT the 3.3 V pin.");
  println!("sweep: center {}us -> min {}us -> max {}us -> center, step {}us", CENTER_US, MIN_US, MAX_US, STEP_US);
  println!();

  // Build the shared 50 Hz LowSpeed LEDC timer + two channels exactly as truck-node's servo task does, then wrap
  // each configured channel in servo::Servo (which snapshots max_duty and converts us -> duty for us).
  let mut ledc = Ledc::new(peripherals.LEDC);
  ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

  let mut lstimer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
  lstimer
    .configure(LedcTimerConfig {
      duty: timer::config::Duty::Duty14Bit,
      clock_source: timer::LSClockSource::APBClk,
      frequency: Rate::from_hz(SERVO_FREQ_HZ),
    })
    .expect("LEDC timer configure");

  let mut pan_ch = ledc.channel::<LowSpeed>(channel::Number::Channel0, pins.servo_pan);
  pan_ch
    .configure(ChannelConfig { timer: &lstimer, duty_pct: 0, drive_mode: DriveMode::PushPull })
    .expect("LEDC pan channel configure");
  let mut tilt_ch = ledc.channel::<LowSpeed>(channel::Number::Channel1, pins.servo_tilt);
  tilt_ch
    .configure(ChannelConfig { timer: &lstimer, duty_pct: 0, drive_mode: DriveMode::PushPull })
    .expect("LEDC tilt channel configure");

  let mut pan = Servo::new(pan_ch);
  let mut tilt = Servo::new(tilt_ch);
  println!("LEDC ready: max_duty {} (14-bit). Beginning sweep.", pan.max_duty());
  println!();

  // The sweep legs, in order: home to center, down to min, up across to max, then back to center. Each leg ramps
  // in STEP_US increments with a short pace, then dwells so the eye can register the endpoint.
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
/// concrete LEDC channel type so a single helper drives both Servo instances.
async fn sweep_leg<C>(pan: &mut Servo<C>, tilt: &mut Servo<C>, from_us: u16, to_us: u16)
where
  C: embedded_hal::pwm::SetDutyCycle,
{
  // Step toward the target in either direction, inclusive of the endpoint, clamping the final partial step so we
  // land exactly on `to_us` rather than overshooting by a fraction of STEP_US.
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

/// Commands both servos to the same pulse width and logs it. set_pulse_us drives a raw pulse (the sweep stays
/// within 1000-2000 us by construction); a duty error is logged rather than panicked so the sweep keeps running.
fn drive<C>(pan: &mut Servo<C>, tilt: &mut Servo<C>, us: u16)
where
  C: embedded_hal::pwm::SetDutyCycle,
{
  println!("pulse {} us", us);
  if pan.set_pulse_us(us).is_err() {
    println!("pan servo duty error");
  }
  if tilt.set_pulse_us(us).is_err() {
    println!("tilt servo duty error");
  }
}
