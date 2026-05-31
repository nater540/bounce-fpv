//! Phase 0 PPM diagnostic for the Skyzone goggle head-tracking stream.
//!
//! Configures one GPIO as a PPM input, decodes the multi-channel pulse train with GPIO edge-await +
//! `embassy_time::Instant` timestamping, and prints each channel's pulse width (us) plus the detected channel count
//! over the C6's built-in USB Serial/JTAG. Run this FIRST to confirm frame structure, channel count, sync-gap length,
//! and which indices carry pan/tilt before building the full system.
//!
//! Capture approach: `Input::wait_for_rising_edge().await` + `Instant::now()` deltas. A gap longer than `SYNC_GAP_US`
//! is treated as the frame sync and resets the channel index. PPM is rising-edge-positioned, so the rising-to-rising
//! interval IS the channel value (typically ~1000-2000 us). NOTE: esp-hal `wait_for_*` had historical C-series
//! reliability/jitter bugs (esp-hal issue #657) — if the printed widths are jittery or unstable across head
//! motion, switch to a GPIO interrupt + free-running timer capture.
//!
//! MEASURED 3.5 mm HT-OUT jack pinout (this unit, probed with a multimeter, black probe on sleeve):
//!   - Tip    = steady 4.7 V → this is the +5 V supply rail, NOT the signal. Leave it UNCONNECTED. The ESP32-C6 is
//!     NOT 5 V tolerant, so never wire the tip to a GPIO.
//!   - Ring   = the PPM signal → GPIO2. Multimeter average (0.1-0.7 V) is consistent with a line that idles LOW and
//!     emits brief positive-going marker pulses, i.e. non-inverted, idle-low, rising-edge-positioned. This confirms
//!     the default decoder polarity below (`Pull::Down` + `wait_for_rising_edge()`) is correct for this unit.
//!   - Sleeve = GND (common ground with the C6).
//!
//! Input protection (interim, no scope and no level shifter on hand): the PPM peak amplitude is UNCONFIRMED — it
//! could be 3.3 V or 5 V, since a 5 V rail is present on the tip. We put a ~4.7 kohm (1 kohm is also fine) series
//! resistor from the ring into GPIO2. If the line swings to 5 V the C6's internal input-clamp diode caps it at
//! ~3.6 V while the series resistor holds the clamp current to a fraction of a mA; if it is only 3.3 V the resistor
//! is harmless. This is amplitude-agnostic. Upgrade path if edges look rough, or if 5 V is later confirmed and more
//! margin is wanted: a proper buffer (e.g. 74LVC1G125 powered at 3.3 V) in place of the bare resistor.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Instant;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Pull};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;

// Manual panic handler: log the panic over USB Serial/JTAG then park. esp-rtos owns the runtime here, so we keep
// the handler minimal rather than pulling in esp-backtrace.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
  println!("PANIC: {}", info);
  loop {}
}

// App descriptor required by the esp-idf second-stage bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

// PPM input pin. GPIO2 is a safe general-purpose pin on both XIAO and DevKitC-1 (XIAO reserves GPIO3/GPIO14 for the
// RF switch, so those are deliberately avoided). Wire the goggles' 3.5 mm HT-OUT RING (the PPM signal) here through a
// ~4.7 kohm series resistor; sleeve to a common ground; leave the tip (+5 V rail) UNCONNECTED. See the module-level
// doc for the measured pinout and the input-protection rationale.
const PPM_GPIO: u8 = 2;

// Frame sync threshold. A real PPM sync gap is typically > ~3 ms; anything longer than this resets the channel index to
// 0 and marks the start of a fresh frame.
const SYNC_GAP_US: u64 = 3_000;

// Plausibility window for a real channel pulse interval. Intervals outside this band are rejected (glitch/noise) so
// a single spurious edge does not corrupt the frame.
const MIN_PULSE_US: u64 = 500;
const MAX_PULSE_US: u64 = 2_500;

// Upper bound on channels we will record per frame. A PPM frame is usually 4-12 channels; 16 leaves headroom.
// Extra pulses beyond this in one frame are ignored until the next sync.
const MAX_CHANNELS: usize = 16;

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
  let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
  let peripherals = esp_hal::init(config);

  // esp-rtos drives Embassy off a TIMG timer + a software interrupt; this runs before any timing primitive is awaited.
  let timg0 = TimerGroup::new(peripherals.TIMG0);
  let sw_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
  esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

  // PPM line polarity. Default (feature `inverted-ppm` OFF): the measured Skyzone ring idles LOW and pulses HIGH, so
  // pull-down keeps the line defined when unplugged and the rising edge marks each channel position. Enable the
  // `inverted-ppm` feature only if you later insert an INVERTING buffer (e.g. 74LVC1G14), which flips the line to
  // idle-high with falling-edge-positioned pulses; that path uses pull-up + falling-edge await instead.
  #[cfg(not(feature = "inverted-ppm"))]
  let input_config = InputConfig::default().with_pull(Pull::Down);
  #[cfg(feature = "inverted-ppm")]
  let input_config = InputConfig::default().with_pull(Pull::Up);
  let mut ppm = Input::new(peripherals.GPIO2, input_config);

  println!();
  println!("=== ppm-diag: Phase 0 PPM diagnostic ===");
  #[cfg(not(feature = "inverted-ppm"))]
  println!("polarity: NON-inverted (idle-low, rising-edge) — default bare series-resistor wiring");
  #[cfg(feature = "inverted-ppm")]
  println!("polarity: INVERTED (idle-high, falling-edge) — for an inverting buffer ahead of GPIO2");
  println!("PPM input on GPIO{}, sync gap threshold {} us", PPM_GPIO, SYNC_GAP_US);
  println!("Wire HT-OUT ring (PPM) via ~4.7k series R, sleeve to GND, tip (+5 V) UNCONNECTED. Waiting for frames...");
  println!();

  // Decoded channel widths for the frame currently being assembled, plus the timestamp of the previous rising edge.
  let mut widths: [u64; MAX_CHANNELS] = [0; MAX_CHANNELS];
  let mut index: usize = 0;
  let mut last_edge: Option<Instant> = None;

  // Throttle printing so the serial console stays readable: PPM frames arrive at ~50 Hz, but we only need a
  // snapshot every so often to judge stability across head motion.
  let mut frames_seen: u32 = 0;
  let mut last_print = Instant::now();

  loop {
    // Await the channel-marker edge. The edge direction is the only polarity-dependent part of the loop; everything
    // below (timing, sync detection, plausibility, reporting) is shared across both wiring modes.
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

    let delta_us = now.duration_since(prev).as_micros();

    if delta_us >= SYNC_GAP_US {
      // Sync gap: the frame that just ended is complete. Report it (throttled), then reset to start accumulating
      // the next frame.
      let count = index;
      frames_seen += 1;

      if now.duration_since(last_print).as_millis() >= 1_000 && count > 0 {
        print_frame(&widths[..count], delta_us, frames_seen);
        last_print = now;
        frames_seen = 0;
      }

      index = 0;
      continue;
    }

    // Mid-frame pulse interval = one channel value. Record it if plausible, otherwise drop it as noise rather than
    // letting a glitch shift every later channel.
    if (MIN_PULSE_US..=MAX_PULSE_US).contains(&delta_us) && index < MAX_CHANNELS {
      widths[index] = delta_us;
      index += 1;
    }
  }
}

/// Prints one decoded frame: channel count, sync-gap length, and each channel width in us. Channels are 1-indexed
/// in the output to match how goggle menus number them (the Skyzone default is channel 5 = pan, channel 6 =
/// tilt — confirm against this readout).
fn print_frame(widths: &[u64], sync_gap_us: u64, frames_since_last: u32) {
  println!(
    "frame: {} channels | sync gap {} us | {} frames since last print",
    widths.len(),
    sync_gap_us,
    frames_since_last
  );
  for (i, w) in widths.iter().enumerate() {
    println!("  ch{:>2}: {:>4} us", i + 1, w);
  }
  println!();
}
