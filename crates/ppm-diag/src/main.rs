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
//! HT-OUT 3.5 mm jack pinout (Skyzone, probed on the ESP32-C6 build, carried over unchanged — the goggles are the
//! same hardware regardless of which MCU decodes them):
//!   - Tip    = steady ~4.7 V → the +5 V supply rail, NOT the signal. Leave it UNCONNECTED. The nRF52840 is NOT 5 V
//!     tolerant, so never wire the tip to a GPIO.
//!   - Ring   = the PPM signal → `pins.ppm`. The line idles LOW and emits brief positive-going marker pulses, i.e.
//!     non-inverted, idle-low, rising-edge-positioned. This confirms the default decoder polarity below
//!     (`Pull::Down` + `wait_for_rising_edge()`) is correct for this unit.
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

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Input, Pull};
use embassy_time::Instant;
use ppm_decoder::{Config as PpmConfig, Frame, PpmDecoder};

// Frame sync threshold (us). Surfaced from the shared decoder's default so the banner prints the same value the
// decoder actually keys on. A real PPM sync gap is typically > ~3 ms.
const SYNC_GAP_US: u32 = ppm_decoder::DEFAULT_SYNC_GAP_US;

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

  // PPM line polarity. Default (feature `inverted-ppm` OFF): the measured Skyzone ring idles LOW and pulses HIGH, so
  // pull-down keeps the line defined when unplugged and the rising edge marks each channel position. Enable the
  // `inverted-ppm` feature only if you later insert an INVERTING buffer (e.g. 74LVC1G14), which flips the line to
  // idle-high with falling-edge-positioned pulses; that path uses pull-up + falling-edge await instead.
  #[cfg(not(feature = "inverted-ppm"))]
  let mut ppm = Input::new(pins.ppm, Pull::Down);
  #[cfg(feature = "inverted-ppm")]
  let mut ppm = Input::new(pins.ppm, Pull::Up);

  applog::log_println!("");
  applog::log_println!("=== ppm-diag: Phase 0 PPM diagnostic (nRF52840) ===");
  #[cfg(not(feature = "inverted-ppm"))]
  applog::log_println!("polarity: NON-inverted (idle-low, rising-edge) — default bare series-resistor wiring");
  #[cfg(feature = "inverted-ppm")]
  applog::log_println!("polarity: INVERTED (idle-high, falling-edge) — for an inverting buffer ahead of the PPM pin");
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
  let mut last_print = Instant::now();

  loop {
    // Await the channel-marker edge. The edge direction is the only polarity-dependent part of the loop; everything
    // below (timing, sync detection, decode, reporting) is shared across both wiring modes.
    #[cfg(not(feature = "inverted-ppm"))]
    ppm.wait_for_rising_edge().await;
    #[cfg(feature = "inverted-ppm")]
    ppm.wait_for_falling_edge().await;
    let now = Instant::now();

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
