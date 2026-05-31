//! MG90S servo control via the ESP32-C6 LEDC peripheral, expressed as pure integer math plus a thin generic
//! wrapper.
//!
//! The C6 has no MCPWM; servos are driven by LEDC low-speed channels off one shared 50 Hz timer. At 50 Hz
//! the period is 20 000 us, and the duty for a given pulse is `duty = pulse_us * max_duty / 20000`. All math
//! here is integer (no floats), using u32 intermediates so `pulse_us * max_duty` never overflows for the
//! ~16-bit `max_duty` LEDC reports. The hardware binding is generic over `embedded_hal::pwm::SetDutyCycle`,
//! so this crate builds and unit-tests standalone while the binary hands it a concrete LEDC channel.
//!
//! Electrical: MG90S wants 4.8-6 V on a SEPARATE 5 V rail sharing ground with the C6 — never the 3.3 V pin.

#![no_std]

use embedded_hal::pwm::SetDutyCycle;

/// LEDC period in microseconds at the 50 Hz servo frame rate (20 ms).
pub const PERIOD_US: u32 = 20_000;

/// Conservative pulse-width endpoints (microseconds). The MG90S datasheet spans 400-2400 us for the full
/// ~180 degrees, but RC convention and the goggle PPM use ~1000-2000 us with 1500 = center, which keeps the
/// gimbal off its mechanical stops. Map incoming PPM straight onto this band.
pub const MIN_PULSE_US: u16 = 1_000;
pub const CENTER_PULSE_US: u16 = 1_500;
pub const MAX_PULSE_US: u16 = 2_000;

/// Converts a pulse width (microseconds) to an LEDC duty value for the given `max_duty` (what
/// `SetDutyCycle::max_duty_cycle()` returns). Integer-only: `duty = pulse_us * max_duty / 20000`, computed in
/// u32 then clamped to `max_duty`. A `pulse_us` past the 20 ms period saturates at full on rather than wrapping.
pub fn pulse_us_to_duty(pulse_us: u16, max_duty: u16) -> u16 {
  let duty = (pulse_us as u32 * max_duty as u32) / PERIOD_US;
  if duty > max_duty as u32 { max_duty } else { duty as u16 }
}

/// Maps a raw PPM channel width (typically ~1000-2000 us) to a servo pulse width, clamped into
/// [`MIN_PULSE_US`, `MAX_PULSE_US`]. Out-of-band PPM (noise, extended-range goggles) is clamped, not wrapped,
/// so the gimbal never slams a stop. Use this to turn a decoded PPM channel directly into a servo command.
pub fn ppm_to_pulse_us(ppm_us: u16) -> u16 {
  ppm_us.clamp(MIN_PULSE_US, MAX_PULSE_US)
}

/// Maps a normalized position to a servo pulse width using integer interpolation over [`MIN_PULSE_US`,
/// `MAX_PULSE_US`]. `value` is interpreted as a fraction `value / scale` (e.g. `value=0..=1000`, `scale=1000`),
/// avoiding floats; `value` is clamped to `0..=scale` first so the result stays within the pulse band.
pub fn normalized_to_pulse_us(value: u16, scale: u16) -> u16 {
  let scale = scale.max(1);
  let v = value.min(scale) as u32;
  let span = (MAX_PULSE_US - MIN_PULSE_US) as u32;
  MIN_PULSE_US + (v * span / scale as u32) as u16
}

/// Thin servo wrapper over any `embedded_hal::pwm::SetDutyCycle` channel (e.g. an esp-hal LEDC channel). It
/// caches `max_duty_cycle()` at construction and converts pulse widths / normalized values to duty for you.
pub struct Servo<C> {
  channel: C,
  max_duty: u16,
}

impl<C: SetDutyCycle> Servo<C> {
  /// Wraps a configured PWM channel, snapshotting its `max_duty_cycle()`. The channel must already be bound
  /// to a 50 Hz LEDC timer; this type only sets duty, it does not configure timer frequency.
  pub fn new(channel: C) -> Self {
    let max_duty = channel.max_duty_cycle();
    Self { channel, max_duty }
  }

  /// The cached full-scale duty (`max_duty_cycle()`), exposed for diagnostics.
  pub fn max_duty(&self) -> u16 {
    self.max_duty
  }

  /// Drives the servo to a raw pulse width in microseconds (clamped to the 0..=period duty range).
  pub fn set_pulse_us(&mut self, pulse_us: u16) -> Result<(), C::Error> {
    let duty = pulse_us_to_duty(pulse_us, self.max_duty);
    self.channel.set_duty_cycle(duty)
  }

  /// Drives the servo from a raw PPM channel width (clamped to the servo pulse band first).
  pub fn set_ppm_us(&mut self, ppm_us: u16) -> Result<(), C::Error> {
    self.set_pulse_us(ppm_to_pulse_us(ppm_us))
  }

  /// Drives the servo from a normalized `value / scale` fraction (integer interpolation across the band).
  pub fn set_normalized(&mut self, value: u16, scale: u16) -> Result<(), C::Error> {
    self.set_pulse_us(normalized_to_pulse_us(value, scale))
  }

  /// Commands the mechanical center (1500 us). Use this for the gimbal's boot/home position.
  pub fn center(&mut self) -> Result<(), C::Error> {
    self.set_pulse_us(CENTER_PULSE_US)
  }

  /// Releases the underlying PWM channel.
  pub fn release(self) -> C {
    self.channel
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn duty_for_center_pulse_is_about_7_5_percent() {
    // At 1500 us / 20000 us = 7.5%. With a 14-bit LEDC (max_duty 16383), that is ~1228.
    let d = pulse_us_to_duty(1500, 16383);
    assert_eq!(d, (1500u32 * 16383 / 20000) as u16);
    assert!(d > 1200 && d < 1260);
  }

  #[test]
  fn duty_saturates_not_wraps() {
    assert_eq!(pulse_us_to_duty(25_000, 1000), 1000); //  past the 20 ms period clamps to full on.
  }

  #[test]
  fn ppm_clamps_into_band() {
    assert_eq!(ppm_to_pulse_us(800), MIN_PULSE_US);
    assert_eq!(ppm_to_pulse_us(1500), 1500);
    assert_eq!(ppm_to_pulse_us(2400), MAX_PULSE_US);
  }

  #[test]
  fn normalized_endpoints_and_center() {
    assert_eq!(normalized_to_pulse_us(0, 1000), MIN_PULSE_US);
    assert_eq!(normalized_to_pulse_us(1000, 1000), MAX_PULSE_US);
    assert_eq!(normalized_to_pulse_us(500, 1000), CENTER_PULSE_US);
    assert_eq!(normalized_to_pulse_us(5000, 1000), MAX_PULSE_US); //  over-range clamps, not wraps.
  }
}
