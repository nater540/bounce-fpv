//! truck-diag :: imu — MPU-6050 over the async I2C bus, in ISOLATION from the rest of the truck node.
//!
//! Runs the SAME boot home/center routine the truck node uses (`imu::detect_home_default`: settle + average N
//! accel-derived roll/pitch samples) and prints the home reference, then enters a live loop that streams accel
//! samples straight off the mpu6050-dmp async driver, converts each with `imu::roll_pitch_from_accel`, and prints
//! roll/pitch a few times a second so the user can tilt the board and watch the numbers track.
//!
//! GOOD: the boot home prints a sane resting roll/pitch (near 0 on a level board) and the live roll/pitch follow
//! the board as it is tilted — roll changes when rolled left/right, pitch when pitched fore/aft, returning near the
//! home values when level. FAILURE / wiring signature: init fails or readings are frozen/garbage -> check SDA/SCL
//! wiring, the AD0 address strap (0x68 with AD0 low, 0x69 with AD0 high), and 3.3 V power. The bin logs that hint
//! and parks rather than panicking, so a miswired board gives a clear message instead of a silent reset loop.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use imu::roll_pitch_from_accel;
use mpu6050_dmp::address::Address;
use mpu6050_dmp::sensor_async::Mpu6050;

// Pull in the shared #[panic_handler] + esp_app_desc!() (defined in the crate lib so all four bins reuse them).
use truck_diag as _;

// MPU-6050 default address (AD0 low). The breakout straps to 0x69 if AD0 is pulled high; surface both in the banner.
const IMU_ADDR: u8 = 0x68;

// Live-loop cadence: a few updates a second is fast enough to watch the numbers track a hand tilt without flooding
// the console. TODO: tune if you want a snappier feel while tilting.
const LIVE_PERIOD: Duration = Duration::from_millis(250);

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
  let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
  let peripherals = esp_hal::init(config);

  // board_pins! partial-moves only the GPIO pin fields, leaving I2C0 / TIMG0 / SW_INTERRUPT on `peripherals`.
  let pins = board::board_pins!(peripherals);

  let timg0 = TimerGroup::new(peripherals.TIMG0);
  let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
  esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

  println!();
  println!("=== truck-diag/imu: MPU-6050 home + live roll/pitch ===");
  println!("I2C SDA GPIO{}, SCL GPIO{}, addr 0x{:02X} (0x69 if AD0 high)", board::I2C_SDA, board::I2C_SCL, IMU_ADDR);
  println!("Hold the board still for the boot home calibration (~2 s), then tilt it and watch roll/pitch track.");
  println!();

  // Single-device diagnostic: no shared-bus Mutex needed. esp-hal's I2c<Async> implements embedded_hal_async::i2c::I2c
  // directly, so it satisfies both the imu crate's bound and the raw mpu6050-dmp driver below.
  let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
    .expect("I2C0 config")
    .with_sda(pins.i2c_sda)
    .with_scl(pins.i2c_scl)
    .into_async();

  // Boot home: settle + average N accel-derived roll/pitch samples — the exact routine truck-node runs before its
  // servo loop. detect_home_default consumes its I2C, so reuse needs a borrow; we pass &mut to keep the bus for the
  // live loop. The imu API takes the bus by value, so we hand it a mutable reference (I2c also impls the trait for
  // &mut Self via embedded-hal-async's blanket impl).
  let mut i2c = i2c;
  let mut delay = Delay;
  match imu::detect_home_default(&mut i2c, &mut delay).await {
    Ok(home) => println!("IMU home: roll {} deg, pitch {} deg", home.roll_deg, home.pitch_deg),
    Err(e) => {
      println!("IMU home detection FAILED ({:?})", e);
      println!("check SDA/SCL wiring + AD0 address (0x68/0x69) + 3.3 V power, then re-flash. Parking.");
      park().await;
    }
  }

  // Live loop: drive the async driver directly so each sample is immediate (no per-sample 1.5 s settle like the
  // boot routine), convert raw accel counts with the imu crate's pure helper, and print roll/pitch.
  let mut mpu = match Mpu6050::new(i2c, Address(IMU_ADDR)).await {
    Ok(mpu) => mpu,
    Err(_) => {
      println!("MPU-6050 re-init for live loop FAILED — check SDA/SCL + AD0 (0x68/0x69) + 3.3 V power. Parking.");
      park().await;
    }
  };

  println!();
  println!("live roll/pitch (tilt the board):");
  loop {
    match mpu.accel().await {
      Ok(a) => {
        let (roll, pitch) = roll_pitch_from_accel(a.x(), a.y(), a.z());
        println!("roll {} deg, pitch {} deg", roll, pitch);
      }
      // A transient read error should not kill the live view; log and keep polling at the same cadence.
      Err(_) => println!("accel read error — check SDA/SCL + power"),
    }
    Timer::after(LIVE_PERIOD).await;
  }
}

/// Parks forever after a fatal wiring/init error so the failure message stays on screen instead of a reset loop.
async fn park() -> ! {
  loop {
    Timer::after(Duration::from_secs(3600)).await;
  }
}
