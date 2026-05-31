//! truck-diag :: imu — MPU-6050 over the async I2C bus, in ISOLATION from the rest of the truck node (Phase C: nRF).
//!
//! Runs the SAME boot home/center routine the truck node uses (`imu::detect_home_default`: settle + average N
//! accel-derived roll/pitch samples) and prints the home reference, then enters a live loop that streams accel
//! samples straight off the mpu6050-dmp async driver, converts each with `imu::roll_pitch_from_accel`, and prints
//! roll/pitch a few times a second so the user can tilt the board and watch the numbers track.
//!
//! GOOD: the boot home prints a sane resting roll/pitch (near 0 on a level board) and the live roll/pitch follow the
//! board as it is tilted — roll changes when rolled left/right, pitch when pitched fore/aft, returning near the home
//! values when level. FAILURE / wiring signature: init fails or readings are frozen/garbage -> check SDA/SCL wiring,
//! the AD0 address strap (0x68 with AD0 low, 0x69 with AD0 high), and 3.3 V power. The bin logs that hint and parks
//! rather than panicking, so a miswired board gives a clear message instead of a silent reset loop.

#![no_std]
#![no_main]

// Pull in the shared #[panic_handler] from applog (replaces the old truck-diag lib panic handler + esp app_desc).
use applog as _;

use embassy_executor::Spawner;
use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{bind_interrupts, interrupt, peripherals};
use embassy_time::{Delay, Duration, Timer};
use imu::roll_pitch_from_accel;
use mpu6050_dmp::address::Address;
use mpu6050_dmp::sensor_async::Mpu6050;
use static_cell::StaticCell;

// Bind ONLY the TWISPI0 interrupt this binary uses (the shared I2C bus on TWISPI0). USBD is bound by applog — do NOT
// bind it here.
bind_interrupts!(struct Irqs {
  TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

// MPU-6050 default address (AD0 low). The breakout straps to 0x69 if AD0 is pulled high; surface both in the banner.
const IMU_ADDR: u8 = 0x68;

// Live-loop cadence: a few updates a second is fast enough to watch the numbers track a hand tilt without flooding the
// console. TODO: tune if you want a snappier feel while tilting.
const LIVE_PERIOD: Duration = Duration::from_millis(250);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
  // embassy-nrf init at SoftDevice-safe interrupt priorities (GPIOTE + time-driver at P2; the SD reserves P0/P1/P4).
  let p = applog::init_embassy_nrf();
  // board_pins! partial-moves only the GPIO pin fields, leaving TWISPI0 / USBD on `p`.
  let pins = board::board_pins!(p);

  // SD COEXISTENCE: Twim::new enables the TWISPI0 interrupt but does NOT set its NVIC priority (it defaults to P0,
  // which the SoftDevice reserves and would fault on). Lower it to P2 BEFORE building the Twim. CONFIRM ON-TARGET.
  interrupt::TWISPI0.set_priority(Priority::P2);

  applog::init(
    spawner,
    p.USBD,
    applog::UsbIdentity::new(0x1209, 0x0001, "fabulous-fpv", "truck-diag-imu", "phase-c"),
  );

  applog::log_println!("");
  applog::log_println!("=== truck-diag/imu: MPU-6050 home + live roll/pitch (nRF52840) ===");
  applog::log_println!(
    "I2C SDA P{}.{:02}, SCL P{}.{:02}, addr 0x{:02X} (0x69 if AD0 high)",
    board::I2C_SDA_PORT, board::I2C_SDA_PIN, board::I2C_SCL_PORT, board::I2C_SCL_PIN, IMU_ADDR
  );
  applog::log_println!("Hold the board still for the boot home calibration (~2 s), then tilt it and watch roll/pitch.");
  applog::log_println!("");

  // Single-device diagnostic: no shared-bus Mutex needed. embassy-nrf's Twim implements embedded_hal_async::i2c::I2c
  // directly, so it satisfies both the imu crate's bound and the raw mpu6050-dmp driver below. The TWIM tx_ram_buffer
  // must be 'static (it outlives the 'static peripheral); a StaticCell gives it that. It is only used if a TX slice is
  // in flash — small command bytes — so 16 bytes is ample (frame data is not sent here).
  static TWIM_TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
  let tx_buf = TWIM_TX_BUF.init([0; 16]);
  let i2c = Twim::new(p.TWISPI0, Irqs, pins.i2c_sda, pins.i2c_scl, twim::Config::default(), tx_buf);

  // Boot home: settle + average N accel-derived roll/pitch samples — the exact routine truck-node runs before its
  // servo loop. detect_home_default takes the bus by value, so we hand it a &mut (Twim impls the trait for &mut Self
  // via embedded-hal-async's blanket impl) to keep the bus for the live loop.
  let mut i2c = i2c;
  let mut delay = Delay;
  match imu::detect_home_default(&mut i2c, &mut delay).await {
    Ok(home) => applog::log_println!("IMU home: roll {} deg, pitch {} deg", home.roll_deg, home.pitch_deg),
    Err(e) => {
      applog::log_println!("IMU home detection FAILED ({:?})", e);
      applog::log_println!("check SDA/SCL wiring + AD0 address (0x68/0x69) + 3.3 V power, then re-flash. Parking.");
      park().await;
    }
  }

  // Live loop: drive the async driver directly so each sample is immediate (no per-sample 1.5 s settle like the boot
  // routine), convert raw accel counts with the imu crate's pure helper, and print roll/pitch.
  let mut mpu = match Mpu6050::new(i2c, Address(IMU_ADDR)).await {
    Ok(mpu) => mpu,
    Err(_) => {
      applog::log_println!("MPU-6050 re-init for live loop FAILED — check SDA/SCL + AD0 (0x68/0x69) + power. Parked.");
      park().await;
    }
  };

  applog::log_println!("");
  applog::log_println!("live roll/pitch (tilt the board):");
  loop {
    match mpu.accel().await {
      Ok(a) => {
        let (roll, pitch) = roll_pitch_from_accel(a.x(), a.y(), a.z());
        applog::log_println!("roll {} deg, pitch {} deg", roll, pitch);
      }
      // A transient read error should not kill the live view; log and keep polling at the same cadence.
      Err(_) => applog::log_println!("accel read error — check SDA/SCL + power"),
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
