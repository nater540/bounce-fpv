//! truck-diag :: oled — SSD1306 status panel over the async I2C bus, in ISOLATION from the truck node (Phase C: nRF).
//!
//! Builds `display::StatusDisplay` (0x3C) on the same I2C pins the truck node uses and loops `render` with
//! deliberately changing values: a counting speed_cm_s and toggling link/fix flags, refreshed a few times a second
//! so the user sees the panel update live. This proves the panel, its address, and the async I2C path end to end
//! without needing LoRa or GPS data.
//!
//! GOOD: the panel lights up with the "FPV HEAD TRACK" title, the LINK/FIX flags toggle Y/N, and the speed value
//! counts up and wraps — all visibly animating. FAILURE / wiring signature: `new()` errors or the panel stays dark ->
//! check SDA/SCL wiring, the panel address strap (0x3C vs 0x3D), and power. The bin logs that hint and parks rather
//! than panicking, so a miswired or wrong-address panel gives a clear message.

#![no_std]
#![no_main]

// Pull in the shared #[panic_handler] from applog (replaces the old truck-diag lib panic handler + esp app_desc).
use applog as _;

use embassy_executor::Spawner;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{bind_interrupts, peripherals};
use embassy_time::{Duration, Ticker, Timer};
use static_cell::StaticCell;

// Bind ONLY the TWISPI0 interrupt this binary uses (the shared I2C bus on TWISPI0). USBD is bound by applog — do NOT
// bind it here.
bind_interrupts!(struct Irqs {
  TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

// SSD1306 default address. Some panels strap to 0x3D; the display crate drives the common 0x3C. Surface both.
const OLED_ADDR: u8 = 0x3C;

// Refresh cadence for the animated demo values. A few Hz makes the toggles and counter clearly visible.
const REFRESH: Duration = Duration::from_millis(400);

// How far the demo speed counts before wrapping (cm/s). Picked so the derived km/h line also visibly changes.
const SPEED_WRAP_CM_S: u32 = 500;
const SPEED_STEP_CM_S: u32 = 25;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
  // embassy-nrf init at SoftDevice-safe interrupt priorities (GPIOTE + time-driver at P2; the SD reserves P0/P1/P4).
  let p = applog::init_embassy_nrf();
  // board_pins! partial-moves only the GPIO pin fields, leaving TWISPI0 / USBD on `p`.
  let pins = board::board_pins!(p);

  applog::init(
    spawner,
    p.USBD,
    applog::UsbIdentity::new(0x1209, 0x0001, "fabulous-fpv", "truck-diag-oled", "phase-c"),
  );

  applog::log_println!("");
  applog::log_println!("=== truck-diag/oled: SSD1306 status panel (nRF52840) ===");
  applog::log_println!(
    "I2C SDA P{}.{:02}, SCL P{}.{:02}, addr 0x{:02X} (0x3D on some)",
    board::I2C_SDA_PORT, board::I2C_SDA_PIN, board::I2C_SCL_PORT, board::I2C_SCL_PIN, OLED_ADDR
  );
  applog::log_println!("Watch the panel: title, toggling LINK/FIX flags, and a counting speed value.");
  applog::log_println!("");

  // Single-device diagnostic: no shared-bus Mutex needed — embassy-nrf's Twim implements the embedded-hal-async I2c
  // trait directly, which is exactly the bound display::StatusDisplay::new requires. The TWIM tx_ram_buffer must be
  // 'static; a StaticCell gives it that. It is only used for flash-resident TX slices (small command bytes), not the
  // framebuffer (ssd1306 flushes from its own RAM buffer), so 16 bytes is ample.
  static TWIM_TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
  let tx_buf = TWIM_TX_BUF.init([0; 16]);
  // Enable the nRF internal SDA/SCL pull-ups so a panel without its own (or long bench wires) still drives the
  // open-drain bus instead of leaving it floating and NAKing every transfer. Both flags set (embassy-nrf gates both
  // lines off sda_pullup in one path); weak ~13 kOhm, harmless if the module already pulls.
  let mut i2c_cfg = twim::Config::default();
  i2c_cfg.sda_pullup = true;
  i2c_cfg.scl_pullup = true;
  let mut i2c = Twim::new(p.TWISPI0, Irqs, pins.i2c_sda, pins.i2c_scl, i2c_cfg, tx_buf);

  // Probe 0x3C/0x3D first so a dead bus (wrong/swapped pins, no pull-ups, no panel) is reported distinctly from a
  // panel that answers but fails init. This is the no-serial isolation test: a GOOD bus then animates below.
  let addr = match display::probe_address(&mut i2c).await {
    Some(a) => {
      applog::log_println!("OLED ACK at 0x{:02X} — initializing.", a);
      a
    }
    None => {
      applog::log_println!("OLED NOT FOUND (no ACK at 0x3C/0x3D). Check SDA->P{}.{:02}, SCL->P{}.{:02} (not swapped),",
        board::I2C_SDA_PORT, board::I2C_SDA_PIN, board::I2C_SCL_PORT, board::I2C_SCL_PIN);
      applog::log_println!("common GND, and pull-ups. Parking.");
      loop {
        Timer::after(Duration::from_secs(3600)).await;
      }
    }
  };
  let mut oled = match display::StatusDisplay::new_with_addr(i2c, addr).await {
    Ok(d) => d,
    Err(e) => {
      applog::log_println!("OLED at 0x{:02X} answered but init FAILED ({:?}) — likely an SH1106, not an SSD1306.", addr, e);
      applog::log_println!("Parking.");
      loop {
        Timer::after(Duration::from_secs(3600)).await;
      }
    }
  };

  applog::log_println!("OLED init OK — animating status. If the panel stays dark, recheck address/power.");
  applog::log_println!("");

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
      // Animate the lap line too so the new field is exercised on the panel: a counter that advances every frame so
      // the LAP value visibly ticks up alongside the speed/flag demo.
      lap_secs: frame,
    };
    if let Err(e) = oled.render(status).await {
      applog::log_println!("OLED render error: {:?} — check SDA/SCL + power", e);
    }

    speed_cm_s += SPEED_STEP_CM_S;
    if speed_cm_s >= SPEED_WRAP_CM_S {
      speed_cm_s = 0;
    }
    frame = frame.wrapping_add(1);
  }
}
