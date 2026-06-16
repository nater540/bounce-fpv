//! truck-diag :: demo — full truck-node "movie mode" with NO LoRa / GPS / IMU: drives the production T1 OLED
//! layout and both gimbal servos from synthetic data so you can eyeball motion AND screen rendering speed on the
//! bench. Two independent Embassy tasks mirror the real node's structure:
//!
//!   - `servo_task` — a steady 50 Hz loop sweeping pan and tilt as two independent triangle waves (different
//!     periods, so they look like live head tracking, not lockstep) across the `servo` crate's pulse band, via the
//!     same `PwmServoChannel` -> `Servo` path the truck node uses (PWM0 pan, PWM1 tilt).
//!   - `oled_task` — renders `display::render_truck` (the committed "Arc / Center" speedo layout) back-to-back as
//!     fast as the I2C bus allows, animating a synthetic ground-speed ramp, a sweeping RSSI (so the signal bars
//!     step 1..4), and a periodic link drop (so the "--" / zero-bars down-state shows). It TIMES each full-frame
//!     flush and logs the achieved frame rate + last/max flush time once a second — that flush time IS the panel's
//!     real rendering speed (a 1 KB SSD1306 framebuffer over I2C is the bottleneck, not the draw).
//!
//! GOOD: the panel animates the speedo arc + big MPH number smoothly, the header RSSI/bars sweep and briefly drop
//! to "--", and BOTH servos sweep through their travel out of phase. The serial log prints a per-second render line
//! like `render: ~30 fps | flush last 31ms max 34ms | 42 mph | rssi -68 bars 4 link up`. FAILURE: a dark panel ->
//! check SDA/SCL + address + power (the bin logs the hint and runs servos-only rather than parking); a servo that
//! twitches/stalls -> it is on the 3.3 V pin, not a 5 V rail (see the POWER warning below), or a wrong signal pin.
//!
//! POWER WARNING (per CLAUDE.md / the migration doc): power the MG90S servos from a DEDICATED 5 V rail that shares
//! ground with the nRF — NEVER the 3.3 V pin, which browns out under stall current and resets the board.

#![no_std]
#![no_main]

// Pull in the shared #[panic_handler] from applog (replaces the old truck-diag lib panic handler + esp app_desc).
use applog as _;

use embassy_executor::Spawner;
use embassy_nrf::pwm::SimplePwm;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{bind_interrupts, peripherals};
use embassy_time::{Duration, Instant, Ticker};
use nrf_adapters::PwmServoChannel;
use servo::Servo;
use static_cell::StaticCell;

// Bind ONLY the TWISPI0 interrupt this binary uses (the OLED's I2C bus). USBD is bound by applog — do NOT bind it
// here. The PWM peripherals are fire-and-forget (register/DMA writes, no awaited IRQ), so they need no binding.
bind_interrupts!(struct Irqs {
  TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

// Servo refresh: 50 Hz, the MG90S frame rate — the same cadence the production servo_task runs at.
const SERVO_PERIOD: Duration = Duration::from_millis(20);
// Sweep periods in 50 Hz ticks. Pan and tilt are deliberately coprime-ish (3.0 s vs 2.0 s) so the gimbal traces a
// slowly-drifting Lissajous figure rather than a back-and-forth line — it reads as "live head tracking" on the bench.
const PAN_PERIOD_TICKS: u32 = 150;
const TILT_PERIOD_TICKS: u32 = 100;
// Tilt rides a narrower band than pan (a nod, not a full swing) so the two axes look distinct: center +/- 350 us.
const TILT_MIN_US: u16 = servo::CENTER_PULSE_US - 350;
const TILT_MAX_US: u16 = servo::CENTER_PULSE_US + 350;

// Synthetic ground-speed sweep for the OLED arc, driven by WALL-CLOCK time (not the frame counter) so it animates at
// a fixed, render-rate-independent pace: ~1 mph climbing to ~60 mph (filling the T1 arc, SPEED_MAX_MPH = 60) over
// SPEED_RAMP_MS, then back down over the same span. The Telemetry unit is cm/s: 60 mph = 2682 cm/s, 1 mph ~= 45 cm/s.
// Tying this to the clock is the fix for "the number barely moves" — a full-frame I2C flush is slow, so a
// frame-counter sweep crawls at the panel's fps; the clock keeps the 1->60 ramp a steady ~10 s no matter the fps.
const SPEED_MIN_CM_S: u16 = 45;
const SPEED_MAX_CM_S: u16 = 2682;
const SPEED_RAMP_MS: u32 = 10_000;
// Synthetic RSSI sweep magnitude (dBm, negated below): -45 dBm (4 bars) down to -115 dBm (1 bar) and back, so the
// header bars step through every level. Period chosen so the sweep is watchable at the loop's ~30 fps.
const RSSI_STRONG_MAG: u16 = 45;
const RSSI_WEAK_MAG: u16 = 115;
const RSSI_PERIOD_FRAMES: u32 = 140;
// Link-liveness cycle: up for most of each LINK_CYCLE_FRAMES window, then a brief drop so the "--" RSSI + zero-bars
// down-state renders. ~260/300 frames up at ~30 fps ~= 8.5 s up, ~1.3 s down.
const LINK_CYCLE_FRAMES: u32 = 300;
const LINK_UP_FRAMES: u32 = 260;

// Render-stats log cadence: summarize the achieved frame rate + flush timing once per second.
const STATS_PERIOD: Duration = Duration::from_secs(1);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
  // embassy-nrf init at SoftDevice-safe interrupt priorities (GPIOTE + time-driver at P2; the SD reserves P0/P1/P4).
  let p = applog::init_embassy_nrf();
  // board_pins! partial-moves only the GPIO pin fields, leaving TWISPI0 / PWM0 / PWM1 / USBD on `p`.
  let pins = board::board_pins!(p);

  applog::init(
    spawner,
    p.USBD,
    applog::UsbIdentity::new(0x1209, 0x0001, "fabulous-fpv", "truck-diag-demo", "phase-c"),
  );

  applog::log_println!("");
  applog::log_println!("=== truck-diag/demo: simulated movement + OLED render-speed (nRF52840) ===");
  applog::log_println!(
    "OLED I2C SDA P{}.{:02}, SCL P{}.{:02} @ 400 kHz | servos pan P{}.{:02} (PWM0), tilt P{}.{:02} (PWM1)",
    board::I2C_SDA_PORT, board::I2C_SDA_PIN, board::I2C_SCL_PORT, board::I2C_SCL_PIN,
    board::SERVO_PAN_PORT, board::SERVO_PAN_PIN, board::SERVO_TILT_PORT, board::SERVO_TILT_PIN
  );
  applog::log_println!("POWER: drive the MG90S from a DEDICATED 5 V rail with COMMON GROUND — NOT the 3.3 V pin.");
  applog::log_println!("No LoRa/GPS/IMU — all values are synthetic. Watch the panel animate; serial logs render speed.");
  applog::log_println!("");

  // Servos: one PWM peripheral per channel (PWM0 pan, PWM1 tilt), built here so the SimplePwm borrows move cleanly
  // into the task; servo_config fixes Div16 + a 20_000-tick countertop so 1 tick = 1 us. Spawned unconditionally so
  // the motion demo still runs even if the OLED is absent.
  let servo_cfg = PwmServoChannel::servo_config();
  let pan_pwm = SimplePwm::new_1ch(p.PWM0, pins.servo_pan, &servo_cfg);
  let tilt_pwm = SimplePwm::new_1ch(p.PWM1, pins.servo_tilt, &servo_cfg);
  spawner.spawn(servo_task(pan_pwm, tilt_pwm).expect("servo_task token"));

  // OLED on TWISPI0 at 400 kHz (the bin bumps the bus from the 100 kHz default so the full-frame flush — and thus the
  // animation — is as snappy as the panel allows; the measured flush time below reports the real cost). Pull-ups on
  // so a panel without its own ACKs on bare wiring. The TWIM tx_ram_buffer must be 'static; a StaticCell gives it that.
  static TWIM_TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
  let tx_buf = TWIM_TX_BUF.init([0; 16]);
  let mut i2c_cfg = twim::Config::default();
  i2c_cfg.frequency = twim::Frequency::K400;
  i2c_cfg.sda_pullup = true;
  i2c_cfg.scl_pullup = true;
  let mut i2c = Twim::new(p.TWISPI0, Irqs, pins.i2c_sda, pins.i2c_scl, i2c_cfg, tx_buf);

  // Probe 0x3C/0x3D so a dead bus is reported distinctly from a panel that answers but fails init. A missing panel is
  // NOT fatal here — the servo motion demo is still worth running, so log the hint and let servo_task carry on alone.
  match display::probe_address(&mut i2c).await {
    Some(addr) => match display::StatusDisplay::new_with_addr(i2c, addr).await {
      Ok(oled) => {
        applog::log_println!("OLED init OK at 0x{:02X} — animating T1 speedo + timing each flush.", addr);
        spawner.spawn(oled_task(oled).expect("oled_task token"));
      }
      Err(e) => applog::log_println!("OLED at 0x{:02X} init FAILED ({:?}) — running servos only.", addr, e),
    },
    None => applog::log_println!("OLED not found (no ACK at 0x3C/0x3D) — running servos only, check SDA/SCL + power."),
  }
}

/// Maps a phase counter to a triangle wave in `[lo, hi]`: rises `lo -> hi` over the first half of `period`, falls
/// back over the second half. Integer-only (no libm) and float-free, so it adds nothing to the binary's math.
/// `period` is assumed even and >= 2; `hi >= lo`.
fn triangle(phase: u32, period: u32, lo: u16, hi: u16) -> u16 {
  let half = period / 2;
  let p = phase % period;
  let up = if p < half { p } else { period - p }; // 0..=half, peaking at the midpoint.
  lo + ((hi - lo) as u32 * up / half) as u16
}

/// Servo task. Sweeps pan and tilt as two out-of-phase triangle waves at a steady 50 Hz, driving the real
/// `Servo`/`PwmServoChannel` path so the bench motion matches what the truck node would command. No radio, no
/// blocking — pure synthetic motion.
#[embassy_executor::task]
async fn servo_task(pan_pwm: SimplePwm<'static>, tilt_pwm: SimplePwm<'static>) {
  let mut pan = Servo::new(PwmServoChannel::new(pan_pwm, 0));
  let mut tilt = Servo::new(PwmServoChannel::new(tilt_pwm, 0));

  let mut ticker = Ticker::every(SERVO_PERIOD);
  let mut phase: u32 = 0;
  // Offset tilt's phase so the two axes don't both start at an endpoint — reads more like live tracking.
  let tilt_offset = TILT_PERIOD_TICKS / 4;
  // Log the commanded pose a few times a minute so the serial trace shows motion without flooding the OLED stats.
  let mut log_at: u32 = 0;

  loop {
    ticker.next().await;
    let pan_us = triangle(phase, PAN_PERIOD_TICKS, servo::MIN_PULSE_US, servo::MAX_PULSE_US);
    let tilt_us = triangle(phase + tilt_offset, TILT_PERIOD_TICKS, TILT_MIN_US, TILT_MAX_US);
    if pan.set_pulse_us(pan_us).is_err() {
      applog::log_println!("pan servo duty error");
    }
    if tilt.set_pulse_us(tilt_us).is_err() {
      applog::log_println!("tilt servo duty error");
    }

    if phase >= log_at {
      applog::log_println!("servo: pan {} us, tilt {} us", pan_us, tilt_us);
      log_at = phase + 100; // ~ every 2 s at 50 Hz.
    }
    phase = phase.wrapping_add(1);
  }
}

/// OLED task. Renders the production T1 "Arc / Center" layout back-to-back from synthetic speed/RSSI/link data and
/// times each full-frame I2C flush, logging the achieved frame rate + last/max flush time once a second. The data
/// advances every frame so the render dirty-check never short-circuits — every measured frame is a real flush, so
/// the reported timing is the panel's true rendering speed.
#[embassy_executor::task]
async fn oled_task(mut oled: display::StatusDisplay<Twim<'static>>) {
  let mut frame: u32 = 0;
  // Animation epoch for the wall-clock speed sweep, so it advances on real time, not on the frame counter.
  let start = Instant::now();
  // Per-second window: count frames and track the worst flush so the log shows sustained fps + a max-latency figure.
  let mut window_start = Instant::now();
  let mut window_frames: u32 = 0;
  let mut window_max_us: u64 = 0;

  loop {
    // Speed sweeps on wall-clock time (steady ~10 s 1->60->1 ramp regardless of render rate). RSSI stays
    // frame-driven and steps every frame, so each render differs from the last — the dirty-check never short-
    // circuits, every measured frame is a true flush, and the timing log below reflects real rendering speed.
    let period_ms = 2 * SPEED_RAMP_MS;
    let phase_ms = (start.elapsed().as_millis() % period_ms as u64) as u32;
    let speed_cm_s = triangle(phase_ms, period_ms, SPEED_MIN_CM_S, SPEED_MAX_CM_S) as u32;
    let rssi = -(triangle(frame, RSSI_PERIOD_FRAMES, RSSI_STRONG_MAG, RSSI_WEAK_MAG) as i16);
    let linked = frame % LINK_CYCLE_FRAMES < LINK_UP_FRAMES;
    let status = display::TruckStatus {
      mph: display::cm_s_to_mph(speed_cm_s),
      kmh: display::cm_s_to_kmh(speed_cm_s),
      bars: display::rssi_to_bars(rssi, linked),
      rssi,
      linked,
    };

    // Time the render: render_truck clears + draws + flushes the 1 KB framebuffer over I2C, the dominant cost.
    let t0 = Instant::now();
    let res = oled.render_truck(status).await;
    let last_flush_us = t0.elapsed().as_micros();
    if let Err(e) = res {
      applog::log_println!("OLED render error: {:?} — check SDA/SCL + power", e);
    }

    window_frames += 1;
    if last_flush_us > window_max_us {
      window_max_us = last_flush_us;
    }

    // Once a second, summarize: achieved fps over the window, the last + worst flush time, and the current sim state.
    if window_start.elapsed() >= STATS_PERIOD {
      applog::log_println!(
        "render: ~{} fps | flush last {}ms max {}ms | {} mph | rssi {} bars {} link {}",
        window_frames, last_flush_us / 1000, window_max_us / 1000, status.mph, status.rssi, status.bars,
        if status.linked { "up" } else { "down" }
      );
      window_start = Instant::now();
      window_frames = 0;
      window_max_us = 0;
    }

    frame = frame.wrapping_add(1);
  }
}
