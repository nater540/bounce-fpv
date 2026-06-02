//! Phase 0 PPM diagnostic for the Skyzone goggle head-tracking stream (Phase C: ported to the nRF52840).
//!
//! Configures `pins.ppm` as a GPIO input, decodes the multi-channel pulse train with a GPIOTE edge-await +
//! `embassy_time::Instant` timestamping, and prints each channel's pulse width (us) plus the detected channel count
//! over the Nice!Nano's USB Serial/JTAG (the applog USB-CDC backend). Run this FIRST to confirm frame structure,
//! channel count, sync-gap length, and which indices carry pan/tilt before building the full system.
//!
//! Capture approach: `Input::wait_for_rising_edge().await` + `Instant::now()` deltas, with the inter-edge deltas
//! handed to the shared `ppm_decoder::PpmDecoder` (the same decoder `goggle-node` uses, so this diagnostic exercises
//! the exact production decode path rather than a private copy). A gap longer than the decoder's sync threshold
//! closes the frame; PPM is rising-edge-positioned, so the rising-to-rising interval IS the channel value
//! (typically ~1000-2000 us). NOTE: if the printed widths are jittery or unstable across head motion, the GPIOTE
//! edge-await may be the cause — switch to a GPIOTE channel + free-running timer capture and re-measure.
//!
//! HT-OUT 3.5 mm jack pinout — CORRECTED on the nRF for the SKY04X (2026-06-01). The jack carries only PPM OUT + GND
//! (no separate power pin), and the earlier "ring = signal, tip = +5 V, idle-low" note (from a different probe) was
//! WRONG and cost a long bring-up:
//!   - Tip    = the PPM signal → `pins.ppm`. It RESTS at ~5 V (measured ~3.78 V before the series R / ~2.98 V at the
//!     pin) and dips LOW for each marker — i.e. idle-HIGH, FALLING-edge-positioned. This is the default decoder
//!     polarity below (`Pull::Down` + `wait_for_falling_edge()`). The nRF is NOT 5 V tolerant, so the series
//!     protection resistor is REQUIRED (it + the pin clamp diode cap the 5 V idle).
//!   - Ring   = unused / not connected on the SKY04X.
//!   - Sleeve = GND (common ground with the nRF52840).
//!
//! Input protection: the PPM peak amplitude is UNCONFIRMED (3.3 V or 5 V, since a 5 V rail is present on the tip).
//! Put a ~4.7 kohm (1 kohm is also fine) series resistor from the ring into the PPM pin. If the line swings to 5 V the
//! nRF's internal input-clamp diode caps it while the series resistor holds the clamp current to a fraction of a mA;
//! if it is only 3.3 V the resistor is harmless. Upgrade path if edges look rough: a proper 3.3 V buffer
//! (e.g. 74LVC1G125) in place of the bare resistor.

#![no_std]
#![no_main]

// Pull in the shared #[panic_handler] (defined once in applog and linked via this `use`). Replaces the old esp
// manual panic handler + esp_bootloader_esp_idf::esp_app_desc!() macro.
use applog as _;

use core::fmt::Write as _;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_nrf::gpio::{Input, Pull};
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker, Timer};
use heapless::String;
use ppm_decoder::{Config as PpmConfig, Frame, PpmDecoder};
use static_cell::StaticCell;

// Frame sync threshold (us). Surfaced from the shared decoder's default so the banner prints the same value the
// decoder actually keys on. A real PPM sync gap is typically > ~3 ms.
const SYNC_GAP_US: u32 = ppm_decoder::DEFAULT_SYNC_GAP_US;

// Pan/tilt channel indices (0-based): menu ch5 = index 4, ch6 = index 5 (matches goggle-node + the Phase 0 finding).
const PAN_INDEX: usize = 4;
const TILT_INDEX: usize = 5;

// Bind the TWISPI0 interrupt for the status OLED's I2C bus. USBD is bound by applog; GPIOTE (the PPM edge await) is
// bound by embassy-nrf's init at the SD-safe P2 priority. No SPIM here — ppm-diag has no radio.
bind_interrupts!(struct Irqs {
  TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

/// Snapshot of one decoded frame for the OLED task. Copy + small so it rides a latest-value Signal. `pan_us`/`tilt_us`
/// are the ch5/ch6 widths (0 if the frame was too short to carry them); `frames` is a running count so a climbing
/// number on the panel confirms frames are actually arriving.
#[derive(Copy, Clone, Default)]
struct PpmStats {
  channels: u8,
  sync_gap_us: u32,
  corrupt: bool,
  pan_us: u16,
  tilt_us: u16,
  frames: u32,
}

// Latest decoded-frame snapshot, published by the edge loop and rendered by the OLED task. CriticalSectionRawMutex
// because the Signal is a `static` shared across the main edge loop and the spawned task.
static PPM_STATS: Signal<CriticalSectionRawMutex, PpmStats> = Signal::new();

// Idle line level sampled when NO edge has arrived for ~0.5 s — the OLED shows it so a silent line is diagnosable:
// `true` (HIGH) points at the +5 V tip or an idle-high/inverted signal; `false` (LOW) means no signal reaches the pin.
static LINE_LEVEL: Signal<CriticalSectionRawMutex, bool> = Signal::new();

// Running count of detected edges (of the configured polarity), published per edge. A CLIMBING count means the pin is
// actually transitioning — the signal IS present and pulsing, so a no-frames state is a decode/polarity/timing issue,
// not a dead line. Stuck at 0 means no edges reach the pin: a static line (goggle not pulsing) or the wrong polarity.
static EDGE_COUNT: Signal<CriticalSectionRawMutex, u32> = Signal::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
  // embassy-nrf init at SoftDevice-safe interrupt priorities (GPIOTE + time-driver at P2; the SD reserves P0/P1/P4).
  let p = applog::init_embassy_nrf();
  // board_pins! partial-moves only the GPIO pin fields out of `p`, leaving the controller singletons (USBD, ...) on
  // `p` for the rest of main.
  let pins = board::board_pins!(p);

  // Bring up the SoftDevice + USB-CDC logging stack. After this, applog::log_println!/the `log` macros emit over the
  // CDC port. A per-binary product string ("ppm-diag") makes this board's CDC port identifiable on the host.
  applog::init(
    spawner,
    p.USBD,
    applog::UsbIdentity::new(0x1209, 0x0001, "fabulous-fpv", "ppm-diag", "phase-c"),
  );

  // Mirror the decode to the SSD1306 (TWISPI0) so the PPM can be read while moving the head, no serial tether. The
  // slow I2C flush runs in its OWN task fed by PPM_STATS — it must NEVER sit in the edge-timing loop below, or the
  // flush would distort the inter-edge measurements. Internal pull-ups + a 0x3C/0x3D probe, as in the other bins.
  static TWIM_TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
  let tx_buf = TWIM_TX_BUF.init([0; 16]);
  let mut i2c_cfg = twim::Config::default();
  i2c_cfg.sda_pullup = true;
  i2c_cfg.scl_pullup = true;
  let i2c = Twim::new(p.TWISPI0, Irqs, pins.i2c_sda, pins.i2c_scl, i2c_cfg, tx_buf);
  spawner.spawn(oled_task(i2c).expect("oled_task token"));

  // PPM line pull + polarity. BOTH modes use Pull::Down; only the edge (set in the loop) differs:
  //  - default: idle-HIGH signal, brief LOW markers, FALLING-edge positioned — the SKY04X HT output natively (tip =
  //    PPM, rests ~5 V, dips low per marker; the jack is PPM+GND only, no power pin). CONFIRMED on hardware.
  //  - `idle-low-ppm`: idle-LOW signal, brief positive HIGH markers, RISING-edge positioned.
  // Pull::Down (NOT Up) is deliberate: it gives a DEFINED LOW when the source is disconnected, so the panel reads
  // "idle LOW" if the wire is off versus "idle HIGH" only when the source is ACTUALLY driving the line high — removing
  // the ambiguity a pull-up creates — and it assists the brief LOW markers across the threshold; the strong ~5 V
  // source still wins at idle. The series resistor clamps the 5 V swing (the nRF is NOT 5 V tolerant).
  let mut ppm = Input::new(pins.ppm, Pull::Down);

  applog::log_println!("");
  applog::log_println!("=== ppm-diag: Phase 0 PPM diagnostic (nRF52840) ===");
  #[cfg(not(feature = "idle-low-ppm"))]
  applog::log_println!("polarity: idle-HIGH, falling-edge (default — SKY04X HT output on the tip)");
  #[cfg(feature = "idle-low-ppm")]
  applog::log_println!("polarity: idle-LOW, rising-edge (--features idle-low-ppm)");
  applog::log_println!(
    "PPM input on P{}.{:02}, sync gap threshold {} us",
    board::PPM_PORT, board::PPM_PIN, SYNC_GAP_US
  );
  applog::log_println!("Wire HT-OUT ring (PPM) via ~4.7k series R, sleeve to GND, tip (+5 V) UNCONNECTED. Waiting...");
  applog::log_println!("");

  // The shared decode state machine: feed it rising-edge-to-rising-edge deltas; it returns a completed Frame each
  // time a sync gap closes the current frame. Same thresholds goggle-node runs, but with `emit_corrupt_frames` ON so
  // a jittery/noisy link still surfaces frames (flagged corrupt) instead of leaving a dead console — the node keeps
  // the default and drops them. The constructed frames carry the REAL measured sync gap, which we print per frame.
  let mut decoder = PpmDecoder::new(PpmConfig { emit_corrupt_frames: true, ..PpmConfig::default() });
  let mut last_edge: Option<Instant> = None;

  // Throttle printing so the serial console stays readable: PPM frames arrive at ~50 Hz, but we only need a snapshot
  // every so often to judge stability across head motion. Count frames between prints to show the live frame rate.
  let mut frames_seen: u32 = 0;
  let mut frame_total: u32 = 0;
  let mut edge_total: u32 = 0;
  let mut last_print = Instant::now();

  loop {
    // Await the channel-marker edge, but BOUNDED by a 0.5 s timeout: when the line is silent the timeout fires and we
    // sample the idle level so the panel can distinguish a dead-but-high line (likely the +5 V tip / inverted) from a
    // dead-but-low one (no signal). The edge direction is the only polarity-dependent part; the timing/decode below is
    // shared across both wiring modes. With a live signal an edge arrives every <20 ms, so the timeout never fires.
    #[cfg(not(feature = "idle-low-ppm"))]
    let outcome = select(ppm.wait_for_falling_edge(), Timer::after(Duration::from_millis(500))).await;
    #[cfg(feature = "idle-low-ppm")]
    let outcome = select(ppm.wait_for_rising_edge(), Timer::after(Duration::from_millis(500))).await;
    if matches!(outcome, Either::Second(())) {
      // No edge in the window: report the idle level, and drop any partial frame so a resumed signal starts clean.
      LINE_LEVEL.signal(ppm.is_high());
      last_edge = None;
      decoder.reset();
      continue;
    }
    let now = Instant::now();
    // Count every edge regardless of whether it decodes — a climbing count proves the pin is transitioning.
    edge_total = edge_total.wrapping_add(1);
    EDGE_COUNT.signal(edge_total);

    let Some(prev) = last_edge else {
      // First edge after boot: establish a reference, nothing to measure yet.
      last_edge = Some(now);
      continue;
    };
    last_edge = Some(now);

    let delta_us = now.duration_since(prev).as_micros() as u32;
    let Some(frame) = decoder.feed(delta_us) else {
      // Mid-frame interval, or a sync gap closing an empty/corrupt frame: nothing to report yet.
      continue;
    };
    frames_seen += 1;
    frame_total = frame_total.wrapping_add(1);
    // Hand the latest frame to the OLED task — a cheap Signal store (no flush here), so the edge timing stays clean.
    PPM_STATS.signal(PpmStats {
      channels: frame.count as u8,
      sync_gap_us: frame.sync_gap_us,
      corrupt: frame.corrupt,
      pan_us: frame.channel(PAN_INDEX).unwrap_or(0),
      tilt_us: frame.channel(TILT_INDEX).unwrap_or(0),
      frames: frame_total,
    });

    // A clean frame just completed. Print a snapshot at most ~once a second so head motion stays observable without
    // flooding the CDC console; report how many frames arrived since the last print as a live frame-rate sanity check.
    if now.duration_since(last_print).as_millis() >= 1_000 {
      print_frame(&frame, frames_seen);
      last_print = now;
      frames_seen = 0;
    }
  }
}

/// Prints one decoded frame: channel count, the REAL measured sync gap that closed it (`frame.sync_gap_us`, not the
/// threshold constant), and each channel width in us. Channels are 1-indexed in the output to match how goggle menus
/// number them (the Skyzone default is channel 5 = pan, channel 6 = tilt — confirm against this readout). A frame
/// the decoder flagged `corrupt` (an out-of-band interval or channel overflow, surfaced because `emit_corrupt_frames`
/// is on) is marked "(CORRUPT)" so the operator sees a noisy/partial frame distinctly from a clean one;
/// `frames_since_last` is how many frames arrived since the previous print.
fn print_frame(frame: &Frame, frames_since_last: u32) {
  let corrupt = if frame.corrupt { " (CORRUPT)" } else { "" };
  applog::log_println!(
    "frame: {} channels | sync gap {} us | {} frames since last print{}",
    frame.count,
    frame.sync_gap_us,
    frames_since_last,
    corrupt
  );
  for (i, w) in frame.channels[..frame.count].iter().enumerate() {
    applog::log_println!("  ch{:>2}: {:>4} us", i + 1, w);
  }
  applog::log_println!("");
}

/// Status OLED task: renders the latest [`PpmStats`] at a few Hz, OFF the edge-timing loop so its slow I2C flush never
/// distorts the inter-edge measurements. Shows the channel count + a climbing frame counter (confirms frames are
/// arriving at all), the sync gap + a `COR` marker for a corrupt frame, and the ch5/ch6 (pan/tilt) widths so head
/// motion is visible live. A missing/dead panel just exits the task — the serial decode keeps running regardless.
#[embassy_executor::task]
async fn oled_task(mut i2c: Twim<'static>) {
  let addr = match display::probe_address(&mut i2c).await {
    Some(a) => a,
    None => {
      applog::log_println!("ppm-diag OLED not found (no ACK at 0x3C/0x3D) — serial-only");
      return;
    }
  };
  let mut oled = match display::StatusDisplay::new_with_addr(i2c, addr).await {
    Ok(d) => d,
    Err(e) => {
      applog::log_println!("ppm-diag OLED init failed at 0x{:02X}: {:?}", addr, e);
      return;
    }
  };
  // Until the first frame, hold a waiting banner — a panel STUCK here means NO edges are arriving (signal/wiring).
  let _ = oled.render_lines(&["PPM diag", "waiting for edges", "(no signal?)"]).await;

  let mut ticker = Ticker::every(Duration::from_millis(250));
  let mut stats = PpmStats::default();
  let mut got_frame = false;
  let mut level: Option<bool> = None;
  let mut edges: u32 = 0;
  loop {
    ticker.next().await;
    if let Some(s) = PPM_STATS.try_take() {
      stats = s;
      got_frame = true;
    }
    if let Some(l) = LINE_LEVEL.try_take() {
      level = Some(l);
    }
    if let Some(e) = EDGE_COUNT.try_take() {
      edges = e;
    }

    if !got_frame {
      // No DECODED frames yet — the EDGE count is the decisive tell. CLIMBING => the pin IS transitioning (signal is
      // present and pulsing, so it's a decode/polarity/timing issue, NOT a dead line). Stuck at 0 => no edges reach the
      // pin: a static line (goggle not pulsing) or the wrong polarity. The idle level distinguishes a driven-high line
      // (HIGH) from a disconnected one (LOW, via Pull::Down).
      let lvl = match level {
        Some(true) => "HIGH",
        Some(false) => "LOW",
        None => "?",
      };
      let mut l_e: String<24> = String::new();
      let _ = write!(l_e, "edges {}", edges);
      let mut l_l: String<24> = String::new();
      let _ = write!(l_l, "idle {}", lvl);
      let _ = oled.render_lines(&["PPM: no frames", &l_e, &l_l, "climb=signal present", "0=static / no pulses"]).await;
      continue;
    }

    let mut l_ch: String<24> = String::new();
    let _ = write!(l_ch, "ch {}  #{}", stats.channels, stats.frames);
    let mut l_sync: String<24> = String::new();
    let _ = write!(l_sync, "sync {}us{}", stats.sync_gap_us, if stats.corrupt { " COR" } else { "" });
    let mut l_pan: String<24> = String::new();
    let _ = write!(l_pan, "ch5 pan  {}", stats.pan_us);
    let mut l_tilt: String<24> = String::new();
    let _ = write!(l_tilt, "ch6 tilt {}", stats.tilt_us);
    if let Err(e) = oled.render_lines(&["PPM diag", &l_ch, &l_sync, &l_pan, &l_tilt]).await {
      applog::log_println!("ppm-diag OLED render error: {:?}", e);
    }
  }
}
