//! `SimplePwm` -> `embedded_hal::pwm::SetDutyCycle` adapter for the `servo` crate.
//!
//! `servo::Servo<C: SetDutyCycle>` drives an MG90S by writing a duty value where `duty / max_duty` is the
//! fraction of the 20 ms frame the pin is held high (it computes `duty = pulse_us * max_duty / 20000`).
//! `embassy_nrf::pwm::SimplePwm` does not implement `SetDutyCycle`; it exposes inherent `max_duty()` /
//! `set_duty(channel, DutyCycle)` / `set_max_duty()` / `set_prescaler()` instead. [`PwmServoChannel`] bridges
//! the two and pins down the 50 Hz timing so the servo crate's mapping arrives as a clean microsecond count.
//!
//! ## 50 Hz frame + duty math
//!
//! The nRF PWM counter clock is `PWM_CLK_HZ = 16 MHz`. With [`Prescaler::Div16`] the counter ticks at
//! `16 MHz / 16 = 1 MHz`, i.e. **1 tick = 1 µs**. Setting the countertop (`set_max_duty`) to `20_000` makes
//! the period `20_000 µs = 20 ms = 50 Hz` exactly, and the duty value is then a pulse width *in microseconds*.
//! (`20_000` is well under the nRF's 15-bit `32_767` countertop limit.)
//!
//! Because `max_duty_cycle()` returns `20_000`, the servo crate's `pulse_us * max_duty / 20000` reduces to
//! `pulse_us` — the duty value handed to `set_duty_cycle` IS the pulse width in µs (e.g. 1500 -> 1.5 ms center).
//!
//! ## Polarity
//!
//! On the nRF PWM, `DutyCycle::normal(v)` drives the output high while the counter is **at or above** `v`
//! (high for `max_duty - v` ticks), whereas `DutyCycle::inverted(v)` drives it high while the counter is
//! **below** `v` (high for `v` ticks). A servo wants a high pulse whose width equals the duty value, so this
//! adapter writes `DutyCycle::inverted(duty)`: a `set_duty_cycle(1500)` yields a 1500 µs high pulse per frame.

use core::convert::Infallible;
use embassy_nrf::pwm::{CounterMode, DutyCycle, Prescaler, SimpleConfig, SimplePwm};
use embedded_hal::pwm::{ErrorType, SetDutyCycle};

/// Countertop (max duty) for a 50 Hz / 20 ms servo frame at a 1 MHz counter tick: 20_000 ticks = 20_000 µs.
/// Also the value `max_duty_cycle()` reports, so the servo crate's pulse-to-duty scaling is the identity.
pub const SERVO_MAX_DUTY: u16 = 20_000;

/// The prescaler giving a 1 MHz counter (1 tick = 1 µs) from the 16 MHz PWM clock: `16 MHz / 16 = 1 MHz`.
const SERVO_PRESCALER: Prescaler = Prescaler::Div16;

/// A single PWM output configured for a 50 Hz servo frame, adapting `SimplePwm` to `SetDutyCycle`.
///
/// It owns the whole `SimplePwm` peripheral and targets one of its channels. `SetDutyCycle::set_duty_cycle`
/// needs `&mut self`, and a `SimplePwm` channel cannot be split off independently, so the simplest sound
/// arrangement is **one PWM peripheral per servo**: drive pan from a `PwmServoChannel` over `PWM0` and tilt
/// from another over `PWM1` (the nRF52840 has `PWM0..PWM3`, all independent). Each is fed to its own
/// `servo::Servo::new(..)`.
pub struct PwmServoChannel<'d> {
  pwm: SimplePwm<'d>,
  channel: usize,
}

impl<'d> PwmServoChannel<'d> {
  /// A [`SimpleConfig`] preset for the 50 Hz servo frame: up-counter, `Div16` prescaler (1 µs tick), and a
  /// `20_000`-tick countertop (20 ms period). Pass it to `SimplePwm::new_1ch`/`new_2ch` when constructing the
  /// peripheral; the channels start idle-low. Provided so the binary configures timing in one place.
  pub fn servo_config() -> SimpleConfig {
    // `SimpleConfig` is `#[non_exhaustive]`, so start from its `Default` and override the timing fields.
    let mut config = SimpleConfig::default();
    config.counter_mode = CounterMode::Up;
    config.prescaler = SERVO_PRESCALER;
    config.max_duty = SERVO_MAX_DUTY;
    config
  }

  /// Wraps an already-constructed `SimplePwm` and forces the 50 Hz servo timing onto it (prescaler +
  /// countertop), regardless of how it was originally configured, then targets `channel` (0..=3). Use this
  /// when you built the `SimplePwm` yourself; if you built it from [`servo_config`](Self::servo_config) the
  /// timing is already correct and this just re-asserts it. `channel` must be a channel the `SimplePwm` was
  /// created with (e.g. `0` for a `new_1ch`).
  pub fn new(pwm: SimplePwm<'d>, channel: usize) -> Self {
    pwm.set_prescaler(SERVO_PRESCALER);
    pwm.set_max_duty(SERVO_MAX_DUTY);
    Self { pwm, channel }
  }

  /// Releases the underlying `SimplePwm` so the binary can reclaim or reconfigure the peripheral.
  pub fn release(self) -> SimplePwm<'d> {
    self.pwm
  }
}

impl ErrorType for PwmServoChannel<'_> {
  // Setting a duty on the nRF PWM cannot fail (it is a register write into a DMA'd duty array), so the
  // adapter is infallible; `servo::Servo`'s `C::Error` therefore becomes `Infallible`.
  type Error = Infallible;
}

impl SetDutyCycle for PwmServoChannel<'_> {
  fn max_duty_cycle(&self) -> u16 {
    // Returns the countertop (20_000), so the servo crate's `pulse_us * max_duty / 20000` is the identity and
    // the duty value passed to `set_duty_cycle` is exactly the pulse width in microseconds.
    self.pwm.max_duty()
  }

  fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
    // Clamp to the countertop first: the nRF `DutyCycle` is 15-bit, so a value above `max_duty` (20_000) would
    // otherwise alias mod 0x8000 instead of saturating. The `SetDutyCycle` contract is "saturate at full-scale".
    let duty = duty.min(self.max_duty_cycle());
    // `inverted` so the high time equals `duty` ticks (= `duty` µs at the 1 MHz tick), i.e. the servo pulse.
    self.pwm.set_duty(self.channel, DutyCycle::inverted(duty));
    Ok(())
  }
}
