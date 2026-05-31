//! truck-diag :: oled — SSD1306 status panel over the async I2C bus, in ISOLATION from the rest of the truck node.
//!
//! Builds `display::StatusDisplay` (0x3C) on the same I2C pins the truck node uses and loops `render` with
//! deliberately changing values: a counting speed_cm_s and toggling link/fix flags, refreshed a few times a second
//! so the user sees the panel update live. This proves the panel, its address, and the async I2C path end to end
//! without needing LoRa or GPS data.
//!
//! GOOD: the panel lights up with the "FPV HEAD TRACK" title, the LINK/FIX flags toggle Y/N, and the speed value
//! counts up and wraps — all visibly animating. FAILURE / wiring signature: `new()` errors or the panel stays dark
//! -> check SDA/SCL wiring, the panel address strap (0x3C vs 0x3D), and power. The bin logs that hint and parks
//! rather than panicking, so a miswired or wrong-address panel gives a clear message.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use esp_hal::clock::CpuClock;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;

// Pull in the shared #[panic_handler] + esp_app_desc!() (defined in the crate lib so all four bins reuse them).
use truck_diag as _;

// SSD1306 default address. Some panels strap to 0x3D; the display crate drives the common 0x3C. Surface both.
const OLED_ADDR: u8 = 0x3C;

// Refresh cadence for the animated demo values. A few Hz makes the toggles and counter clearly visible.
const REFRESH: Duration = Duration::from_millis(400);

// How far the demo speed counts before wrapping (cm/s). Picked so the derived km/h line also visibly changes.
const SPEED_WRAP_CM_S: u32 = 500;
const SPEED_STEP_CM_S: u32 = 25;

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
  println!("=== truck-diag/oled: SSD1306 status panel ===");
  println!("I2C SDA GPIO{}, SCL GPIO{}, addr 0x{:02X} (0x3D on some)", board::I2C_SDA, board::I2C_SCL, OLED_ADDR);
  println!("Watch the panel: title, toggling LINK/FIX flags, and a counting speed value.");
  println!();

  // Single-device diagnostic: no shared-bus Mutex needed — esp-hal's I2c<Async> implements the embedded-hal-async I2c
  // trait directly, which is exactly the bound display::StatusDisplay::new requires.
  let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
    .expect("I2C0 config")
    .with_sda(pins.i2c_sda)
    .with_scl(pins.i2c_scl)
    .into_async();

  let mut oled = match display::StatusDisplay::new(i2c).await {
    Ok(d) => d,
    Err(e) => {
      println!("OLED init FAILED ({:?})", e);
      println!("check SDA/SCL wiring + panel address (0x3C/0x3D) + power, then re-flash. Parking.");
      loop {
        embassy_time::Timer::after(Duration::from_secs(3600)).await;
      }
    }
  };

  println!("OLED init OK — animating status. If the panel stays dark, recheck address/power.");
  println!();

  // Cycle the demo values so every field visibly changes: speed counts up and wraps, and the two flags toggle on
  // alternating frames so LINK and FIX are never in lockstep (easy to confirm each independently renders).
  let mut ticker = Ticker::every(REFRESH);
  let mut speed_cm_s: u32 = 0;
  let mut frame: u32 = 0;
  loop {
    ticker.next().await;
    let status = display::Status {
      speed_cm_s,
      link_up: frame % 2 == 0,
      gps_fix: frame % 3 == 0,
    };
    if let Err(e) = oled.render(status).await {
      println!("OLED render error: {:?} — check SDA/SCL + power", e);
    }

    speed_cm_s += SPEED_STEP_CM_S;
    if speed_cm_s >= SPEED_WRAP_CM_S {
      speed_cm_s = 0;
    }
    frame = frame.wrapping_add(1);
  }
}
