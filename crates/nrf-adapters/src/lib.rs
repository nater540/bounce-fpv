//! embassy-nrf -> embedded-hal adapters shared by the frozen driver crates on the nRF52840.
//!
//! Most driver crates were written against generic embedded-hal 1.0 / embedded-io-async traits, and
//! embassy-nrf 0.10 already implements those traits on the peripherals they bound on — so almost no glue is
//! needed. The one exception is the servo PWM path:
//!
//! - [`pwm::PwmServoChannel`] wraps an `embassy_nrf::pwm::SimplePwm` configured for a 50 Hz servo frame and
//!   implements `embedded_hal::pwm::SetDutyCycle`, so `servo::Servo::new(channel)` type-checks unchanged.
//!
//! The GPS UART RX path needs NO adapter: embassy-nrf 0.10's `BufferedUarteRx` natively implements
//! `embedded_io_async::Read` (the same 0.7 the workspace pins), so `gps::GpsReader<R: Read>` consumes a
//! `BufferedUarteRx` directly. The old `UarteReadAdapter` (which wrapped the blocking, whole-buffer
//! `UarteRx::read` and shimmed an obsolete embedded-io-async 0.6 error) has been removed — see the GPS
//! bring-up notes for the `BufferedUarte::new` / `.split()` construction the diagnostic and truck node use.

#![no_std]

pub mod pwm;

pub use pwm::PwmServoChannel;
