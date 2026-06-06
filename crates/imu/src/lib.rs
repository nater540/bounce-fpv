//! MPU-6050 boot-time home/center detection over the async `mpu6050-dmp` driver.
//!
//! On the truck the gimbal "home" is the orientation the IMU sees at power-on. Per the doc's startup pattern
//! this crate wakes the device, lets it settle, takes N accelerometer samples, averages the accel-derived
//! roll/pitch, and stores them as the [`Home`] reference the servo loop centers against. It is generic over
//! an `embedded_hal_async::i2c::I2c` (so it rides the shared async I2C bus) and an async `DelayNs` for the
//! settle/inter-sample waits; the node supplies both concrete types.
//!
//! Roll/pitch come from the gravity vector: with raw accel counts (ax, ay, az) the scale cancels in the
//! ratios, so `roll = atan2(ay, az)` and `pitch = atan2(-ax, hypot(ay, az))`, both in degrees. `libm` provides
//! the `no_std` trig — this is the only floating-point in the driver crates, confined to one-shot boot
//! calibration (not the hot path), which is why it is acceptable here.

#![no_std]

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;
use mpu6050_dmp::accel::Accel;
use mpu6050_dmp::address::Address;
use mpu6050_dmp::sensor_async::Mpu6050;

pub use mpu6050_dmp::accel::AccelFullScale as ImuAccelFullScale;
pub use mpu6050_dmp::address::Address as ImuAddress;

/// I2C address with AD0 low (the common default for the bare MPU-6050 breakout).
pub const ADDR_AD0_LOW: u8 = 0x68;
/// I2C address with AD0 high.
pub const ADDR_AD0_HIGH: u8 = 0x69;

/// Settle time before sampling, in milliseconds. The accel output needs ~1-2 s after wake to stabilize.
pub const DEFAULT_SETTLE_MS: u32 = 1_500;
/// Default number of samples averaged into the home reference.
pub const DEFAULT_SAMPLES: u16 = 100;
/// Spacing between samples, in milliseconds (100 samples * 5 ms = ~0.5 s of averaging).
pub const SAMPLE_INTERVAL_MS: u32 = 5;

/// The boot home reference: the average resting roll and pitch in degrees.
/// The servo loop treats this as the gimbal center, so a tilted mounting is calibrated out automatically.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Home {
  pub roll_deg: f32,
  pub pitch_deg: f32,
}

/// Errors from home detection: either driver init failed, or a sample read failed.
/// The driver's own error type is generic over the I2C error, so we keep this enum minimal and `Debug`-only.
#[derive(Debug)]
pub enum HomeError {
  /// `Mpu6050::new` could not bring the device out of sleep (bad wiring / wrong address).
  Init,
  /// An accelerometer read failed mid-calibration.
  Read,
}

/// Computes roll and pitch (degrees) from a raw accel sample.
/// Scale cancels in the ratios, so raw counts are fine. `roll = atan2(ay, az)`, `pitch = atan2(-ax, hypot(ay, az))`.
pub fn roll_pitch_from_accel(ax: i16, ay: i16, az: i16) -> (f32, f32) {
  let (ax, ay, az) = (ax as f32, ay as f32, az as f32);
  let roll = libm::atan2f(ay, az);
  let pitch = libm::atan2f(-ax, libm::sqrtf(ay * ay + az * az));
  (roll.to_degrees(), pitch.to_degrees())
}

/// Initializes the MPU-6050 on the given shared I2C bus and runs the boot home/center routine: wake, settle,
/// average `samples` accel-derived roll/pitch readings, and return the [`Home`] reference. `delay` provides
/// the settle and inter-sample waits. The accelerometer is left at its +-2 g full-scale default.
///
/// Call this once at startup, before the servo loop takes over, with the device held still.
pub async fn detect_home<I, D>(i2c: I, address: Address, samples: u16, delay: &mut D) -> Result<Home, HomeError>
where
  I: I2c,
  D: DelayNs,
{
  let mut mpu = Mpu6050::new(i2c, address).await.map_err(|_| HomeError::Init)?;

  // Let the accel output settle after wake before we trust any sample.
  delay.delay_ms(DEFAULT_SETTLE_MS).await;

  let n = samples.max(1);
  let mut roll_sum = 0.0f32;
  let mut pitch_sum = 0.0f32;
  for _ in 0..n {
    let a: Accel = mpu.accel().await.map_err(|_| HomeError::Read)?;
    let (roll, pitch) = roll_pitch_from_accel(a.x(), a.y(), a.z());
    roll_sum += roll;
    pitch_sum += pitch;
    delay.delay_ms(SAMPLE_INTERVAL_MS).await;
  }

  let inv = 1.0 / n as f32;
  Ok(Home { roll_deg: roll_sum * inv, pitch_deg: pitch_sum * inv })
}

/// Convenience over [`detect_home`] using [`DEFAULT_SAMPLES`] and the AD0-low address (0x68).
pub async fn detect_home_default<I, D>(i2c: I, delay: &mut D) -> Result<Home, HomeError>
where
  I: I2c,
  D: DelayNs,
{
  detect_home(i2c, Address(ADDR_AD0_LOW), DEFAULT_SAMPLES, delay).await
}
