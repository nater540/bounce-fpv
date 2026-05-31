//! Standalone LoRa link bring-up prototype (Phase-0-style diagnostic) for the RFM95W/SX1276 (Phase C: nRF52840).
//!
//! This validates the radio wiring (SPI SCK/MOSI/MISO + NSS/RESET/DIO0) and measures the ping->pong round-trip
//! latency at the link's chosen modulation (SF7 / BW 500 kHz / CR 4/5 @ 915 MHz) BEFORE the full goggle<->truck path
//! is integrated. It builds the same embassy-nrf async SPIM + `ExclusiveDevice` + `build_sx1276` radio the node
//! binaries will use, so a clean run here de-risks every downstream LoRa assumption.
//!
//! One binary, two roles selected by a Cargo feature (mirrors ppm-diag's `inverted-ppm` cfg style):
//!   - feature OFF (default) -> PINGER: send a small seq-numbered packet, await the echo with a bounded timeout,
//!     compute RTT, and print RTT + RSSI/SNR plus running min/avg/max RTT and a lost-packet count.
//!   - feature `ponger` ON   -> PONGER: receive a packet, immediately echo it back (with the magic byte bumped so
//!     the pong is distinguishable on the wire), and print the seq + RSSI/SNR it heard.
//! Flash ONE board with the default build and the OTHER with `--features ponger`; power both, watch the pinger's
//! console for stable low RTT and matching seq numbers.
//!
//! Payload format (5 bytes, deliberately independent of the `proto` crate — this is a link test, not a protocol
//! test): bytes 0..4 = the u32 sequence counter, little-endian; byte 4 = a 1-byte magic tag (PING_MAGIC on the
//! ping, PONG_MAGIC on the echo). The pinger validates both the echoed seq and the pong magic before counting an
//! RTT, so a stray packet from anything else on the band cannot pollute the statistics.
//!
//! ## lora-phy / defmt link note (mirrored verbatim by the Phase D nodes)
//!
//! `lora-phy` depends on `defmt` UNCONDITIONALLY, so it emits `_defmt_*` symbols and an undefined `#[global_logger]`
//! that the linker must resolve even though our real logging is the USB-CDC `applog` path. That linkage is now
//! CENTRALIZED, so this crate carries no per-binary defmt bits: `applog` (a dep, pulled in via `use applog as _;`)
//! provides the `#[global_logger]` through its own `use defmt_rtt as _;`, and `.cargo/config.toml` adds `-Tdefmt.x`
//! globally so the `.defmt` linker section is always defined. The defmt/RTT output is never read (no probe is
//! attached; everything human-facing goes over USB-CDC) — this purely makes the image link.

#![no_std]
#![no_main]

// Pull in the shared #[panic_handler] (defined once in applog). applog ALSO provides defmt-rtt's #[global_logger]
// (it does `use defmt_rtt as _;`), which is what lets lora-phy's unconditional defmt dependency link here without any
// per-binary defmt setup. Replaces the old esp manual panic handler + esp_bootloader_esp_idf::esp_app_desc!() macro.
use applog as _;

use embassy_executor::Spawner;
#[cfg(not(feature = "ponger"))]
use embassy_futures::select::{select, Either};
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::spim::{self, Spim};
use embassy_nrf::{bind_interrupts, interrupt, peripherals};
use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_time::Delay;
#[cfg(not(feature = "ponger"))]
use embassy_time::Instant;
use embassy_time::{Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use lora_link::{LoraLink, Sx1276Radio};

// Bind ONLY the SPIM3 interrupt this binary uses (the LoRa SPI bus on SPI3). USBD is bound by applog — do NOT bind it
// here. GPIOTE (DIO0 edge waits) is bound by embassy-nrf's init at the SD-safe P2 priority via init_embassy_nrf.
bind_interrupts!(struct Irqs {
  SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
});

// Operating frequency. 915 MHz is the US ISM band and matches lora-link's default. TODO: tune on hardware per your
// region (e.g. 868 MHz in EU) — both ends must agree.
const FREQUENCY_HZ: u32 = 915_000_000;

// Output power in dBm passed to lora-phy's prepare_for_tx. 14 dBm is a conservative bring-up level that keeps the PA
// well inside spec while still giving a usable link on the bench. TODO: tune on hardware / per regional limits.
const OUTPUT_POWER_DBM: i32 = 14;

// PA output routing. The bare RFM95W bonds ONLY its PA_BOOST pin to the antenna (the RFO pin is left unconnected), so
// tx_boost MUST be true or you get near-zero radiated power and the link silently fails to come up — a classic
// bring-up gotcha. Leave this on unless you are on a board that explicitly routes RFO.
const TX_BOOST: bool = true;

// 1-byte tags so the pinger can tell its own echo apart from any other traffic on the band. The ponger bumps the
// magic from PING_MAGIC to PONG_MAGIC so a captured packet's direction is visible on the wire.
const PING_MAGIC: u8 = 0xA5;
const PONG_MAGIC: u8 = 0x5A;

// Fixed payload size: 4 LE bytes of sequence number + 1 magic byte.
const PAYLOAD_LEN: usize = 5;

// Pinger loop pacing: one ping per this interval so the console stays readable. The actual RTT is far shorter; this
// is just the gap between attempts. TODO: tune on hardware once the measured RTT is known.
#[cfg(not(feature = "ponger"))]
const PING_PERIOD: Duration = Duration::from_millis(200);

// How long the pinger waits for the echo before declaring the ping lost. Generous relative to the expected SF7/BW500
// round trip so a momentary miss is still attributed correctly. TODO: tighten once real RTT is measured.
#[cfg(not(feature = "ponger"))]
const ECHO_TIMEOUT: Duration = Duration::from_millis(150);

// Print a rolling statistics line every this many pings so the serial log summarizes link health at a glance.
#[cfg(not(feature = "ponger"))]
const STATS_EVERY: u32 = 10;

// Concrete radio type for this board: the SX1276 over an ExclusiveDevice (SPIM3 async bus + NSS Output + a
// blocking/async `Delay`), with the RESET Output and DIO0 Input as the lora-phy interface-variant pins. Matches the
// node binaries' aliases; this prototype runs everything in `main`, but the alias keeps the types readable.
type RadioSpi = ExclusiveDevice<Spim<'static>, Output<'static>, Delay>;
type Radio = Sx1276Radio<RadioSpi, Output<'static>, Input<'static>>;
type Link = LoraLink<Radio, Delay>;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
  // embassy-nrf init at SoftDevice-safe interrupt priorities (GPIOTE + time-driver at P2; the SD reserves P0/P1/P4).
  let p = applog::init_embassy_nrf();
  // board_pins! partial-moves only the GPIO pin fields out of `p`, leaving the controller singletons (SPI3, USBD, ...)
  // on `p` for the rest of main.
  let pins = board::board_pins!(p);

  // SD COEXISTENCE: Spim::new enables the SPIM3 interrupt but does NOT set its NVIC priority, which defaults to P0 —
  // a priority the SoftDevice reserves, so a SPIM IRQ there would fault the SD. Lower it to P2 (same band as GPIOTE /
  // the time driver) BEFORE building the Spim. CONFIRM ON-TARGET that the radio IRQ behaves with the SD enabled.
  interrupt::SPIM3.set_priority(Priority::P2);

  applog::init(
    spawner,
    p.USBD,
    applog::UsbIdentity::new(0x1209, 0x0001, "fabulous-fpv", "lora-ping", "phase-c"),
  );

  // Boot banner: role, radio params, and the LoRa pin assignments (surfaced from the board crate's pub consts) so the
  // serial log is self-documenting and a wiring fault is easy to cross-check against the schematic.
  applog::log_println!("");
  #[cfg(not(feature = "ponger"))]
  applog::log_println!("=== lora-ping: PINGER (default build) — measures ping->pong RTT (nRF52840) ===");
  #[cfg(feature = "ponger")]
  applog::log_println!("=== lora-ping: PONGER (--features ponger) — echoes packets back (nRF52840) ===");
  applog::log_println!(
    "radio: {} Hz | SF7 | BW500kHz | CR4/5 | power {} dBm | tx_boost {}",
    FREQUENCY_HZ,
    OUTPUT_POWER_DBM,
    TX_BOOST
  );
  applog::log_println!("note: RFM95W routes PA_BOOST only — tx_boost MUST be true or output is near-zero.");
  applog::log_println!(
    "LoRa pins: SCK=P{}.{:02} MOSI=P{}.{:02} MISO=P{}.{:02} NSS=P{}.{:02} RESET=P{}.{:02} DIO0=P{}.{:02}",
    board::LORA_SCK_PORT, board::LORA_SCK_PIN, board::LORA_MOSI_PORT, board::LORA_MOSI_PIN,
    board::LORA_MISO_PORT, board::LORA_MISO_PIN, board::LORA_NSS_PORT, board::LORA_NSS_PIN,
    board::LORA_RESET_PORT, board::LORA_RESET_PIN, board::LORA_DIO0_PORT, board::LORA_DIO0_PIN
  );

  // Build the SX1276 SPI bus on SPI3. spim::Config defaults to 1 MHz, mode 0 (the SX1276 default), which is exactly
  // what the radio wants. NSS/RESET are push-pull Outputs (idle high); DIO0 is the IRQ Input the RX/TX futures await.
  // ExclusiveDevice owns the bus + NSS and toggles CS around each transfer.
  let spi_bus = Spim::new(p.SPI3, Irqs, pins.lora_sck, pins.lora_miso, pins.lora_mosi, spim::Config::default());

  let nss = Output::new(pins.lora_nss, Level::High, OutputDrive::Standard);
  let reset = Output::new(pins.lora_reset, Level::High, OutputDrive::Standard);
  let dio0 = Input::new(pins.lora_dio0, Pull::None);
  let spi_dev = ExclusiveDevice::new(spi_bus, nss, Delay).expect("ExclusiveDevice");

  // Bring up the radio. build_sx1276 fixes SF7/BW500/CR4-5 @ 915 MHz (lora-link's defaults). On failure DO NOT panic
  // silently — log the error and park, because an error here almost always means a wiring fault (NSS/RESET/DIO0 or a
  // SPI line), which is exactly what this prototype exists to catch.
  applog::log_println!("radio init: building SX1276 ...");
  let link: Link = match lora_link::build_sx1276(spi_dev, reset, dio0, Delay, TX_BOOST).await {
    Ok(link) => {
      applog::log_println!("radio init: OK");
      link
    }
    Err(e) => {
      applog::log_println!("radio init: FAILED ({:?})", e);
      applog::log_println!("check SPI wiring (SCK/MOSI/MISO), NSS, RESET, DIO0, and 3.3 V power to the radio. Parked.");
      loop {
        Timer::after(Duration::from_secs(3600)).await;
      }
    }
  };

  #[cfg(not(feature = "ponger"))]
  pinger(link).await;
  #[cfg(feature = "ponger")]
  ponger(link).await;
}

/// PINGER role. Maintains a u32 sequence counter; for each ping it records t0, transmits the seq + PING_MAGIC, then
/// awaits the echo bounded by ECHO_TIMEOUT. On a valid echo (matching seq, PONG_MAGIC) it computes RTT = now - t0 and
/// prints it with the echo's RSSI/SNR; it also tracks running min/avg/max RTT and a lost count, printed every
/// STATS_EVERY pings. A timeout prints an explicit wiring/power hint rather than deadlocking. Never returns.
#[cfg(not(feature = "ponger"))]
async fn pinger(mut link: Link) -> ! {
  applog::log_println!(
    "pinger: sending one ping every {} ms, echo timeout {} ms",
    PING_PERIOD.as_millis(),
    ECHO_TIMEOUT.as_millis()
  );
  applog::log_println!("");

  let mut rx_buf = [0u8; lora_link::MAX_PAYLOAD as usize];
  let mut seq: u32 = 0;

  // Rolling RTT stats in microseconds. `min`/`max` start unset; `sum`/`recv` accumulate for the average, `lost`
  // counts pings with no valid echo before the timeout.
  let mut min_us: u64 = u64::MAX;
  let mut max_us: u64 = 0;
  let mut sum_us: u64 = 0;
  let mut recv: u32 = 0;
  let mut lost: u32 = 0;

  loop {
    let mut payload = [0u8; PAYLOAD_LEN];
    payload[0..4].copy_from_slice(&seq.to_le_bytes());
    payload[4] = PING_MAGIC;

    let t0 = Instant::now();
    if let Err(e) = link.send(&payload, OUTPUT_POWER_DBM).await {
      // A TX error is a radio/SPI problem, not a missing peer — surface it distinctly from a lost echo.
      applog::log_println!("seq {}: TX error: {:?}", seq, e);
      Timer::after(PING_PERIOD).await;
      seq = seq.wrapping_add(1);
      continue;
    }

    // Await the echo, bounded so a lost pong never deadlocks the loop.
    match select(link.receive_with_status(&mut rx_buf), Timer::after(ECHO_TIMEOUT)).await {
      Either::First(Ok((len, status))) => {
        let rtt_us = Instant::now().duration_since(t0).as_micros();
        if valid_echo(&rx_buf[..len], seq) {
          recv += 1;
          sum_us += rtt_us;
          if rtt_us < min_us {
            min_us = rtt_us;
          }
          if rtt_us > max_us {
            max_us = rtt_us;
          }
          applog::log_println!("seq {}: RTT {} us | RSSI {} dBm | SNR {} dB", seq, rtt_us, status.rssi, status.snr);
        } else {
          // A packet arrived but it was not our echo (wrong seq or magic): count it as lost for this seq and note it.
          lost += 1;
          applog::log_println!("seq {}: stray/mismatched packet ({} bytes) — not our echo", seq, len);
        }
      }
      Either::First(Err(e)) => {
        lost += 1;
        applog::log_println!("seq {}: RX error: {:?}", seq, e);
      }
      Either::Second(()) => {
        lost += 1;
        applog::log_println!(
          "seq {}: no echo for {} ms — check ponger powered / antenna / NSS-RESET-DIO0 wiring",
          seq,
          ECHO_TIMEOUT.as_millis()
        );
      }
    }

    // Periodic rolling summary so link health is visible without scanning every line.
    if (seq + 1) % STATS_EVERY == 0 {
      if recv > 0 {
        applog::log_println!(
          "stats: pings {} | echoes {} | lost {} | RTT min {} / avg {} / max {} us",
          seq + 1, recv, lost, min_us, sum_us / recv as u64, max_us
        );
      } else {
        applog::log_println!("stats: pings {} | echoes 0 | lost {} | no RTT yet (no valid echoes)", seq + 1, lost);
      }
      applog::log_println!("");
    }

    Timer::after(PING_PERIOD).await;
    seq = seq.wrapping_add(1);
  }
}

/// Validates a received echo against the expected sequence: it must be exactly PAYLOAD_LEN bytes, carry PONG_MAGIC,
/// and decode to the seq we just sent. This rejects stray band traffic and stale echoes so the RTT stats stay clean.
#[cfg(not(feature = "ponger"))]
fn valid_echo(buf: &[u8], expected_seq: u32) -> bool {
  if buf.len() != PAYLOAD_LEN || buf[4] != PONG_MAGIC {
    return false;
  }
  let seq = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
  seq == expected_seq
}

/// PONGER role. Parks in continuous RX; on each received packet it echoes the same payload straight back (with the
/// magic bumped to PONG_MAGIC so the direction is visible on the wire) and prints the seq + RSSI/SNR it heard. This
/// exercises the ponger's own RX->TX turnaround and the full SPI/NSS/RESET/DIO0 path on its board. Never returns.
#[cfg(feature = "ponger")]
async fn ponger(mut link: Link) -> ! {
  applog::log_println!("ponger: listening, will echo every packet back with magic 0x{:02X}", PONG_MAGIC);
  applog::log_println!("");

  let mut rx_buf = [0u8; lora_link::MAX_PAYLOAD as usize];

  loop {
    let (len, status) = match link.receive_with_status(&mut rx_buf).await {
      Ok(result) => result,
      Err(e) => {
        applog::log_println!("RX error: {:?}", e);
        continue;
      }
    };

    // Decode the seq for the log if the packet is the expected shape; otherwise still echo whatever we got so the
    // pinger's mismatch path is exercised too, but flag it.
    if len == PAYLOAD_LEN && rx_buf[4] == PING_MAGIC {
      let seq = u32::from_le_bytes([rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]]);
      applog::log_println!("heard seq {}: RSSI {} dBm | SNR {} dB — echoing", seq, status.rssi, status.snr);
    } else {
      applog::log_println!(
        "heard {} bytes (not a ping payload): RSSI {} dBm | SNR {} dB — echoing",
        len, status.rssi, status.snr
      );
    }

    // Echo the same payload back, bumping the magic byte so the pinger can confirm it is a pong, not its own TX
    // leaking back. Only the magic is touched; the seq is preserved so the pinger can match it to its t0.
    let mut echo = [0u8; PAYLOAD_LEN];
    let copy = len.min(PAYLOAD_LEN);
    echo[..copy].copy_from_slice(&rx_buf[..copy]);
    echo[4] = PONG_MAGIC;
    if let Err(e) = link.send(&echo, OUTPUT_POWER_DBM).await {
      applog::log_println!("echo TX error: {:?}", e);
    }
  }
}
