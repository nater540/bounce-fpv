//! Standalone LoRa link bring-up prototype (Phase-0-style diagnostic) for the RFM95W/SX1276.
//!
//! This validates the radio wiring (SPI SCK/MOSI/MISO + NSS/RESET/DIO0) and measures the ping->pong round-trip
//! latency at the link's chosen modulation (SF7 / BW 500 kHz / CR 4/5 @ 915 MHz) BEFORE the full goggle<->truck path
//! is integrated — per the overview's Recommendation #3. It builds the exact same esp-hal async SPI + ExclusiveDevice
//! + `build_sx1276` radio the node binaries use, so a clean run here de-risks every downstream LoRa assumption.
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

#![no_std]
#![no_main]

use embassy_executor::Spawner;
#[cfg(not(feature = "ponger"))]
use embassy_futures::select::{select, Either};
#[cfg(not(feature = "ponger"))]
use embassy_time::Instant;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::Async;
use esp_println::println;
use lora_link::{LoraLink, Sx1276Radio};

// Manual panic handler: log over USB Serial/JTAG then park. esp-rtos owns the runtime here, so we keep this minimal
// rather than pulling in esp-backtrace (matches ppm-diag / the node binaries).
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
  println!("PANIC: {}", info);
  loop {}
}

// App descriptor required by the esp-idf second-stage bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

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

// Concrete radio type for this board: the SX1276 over an ExclusiveDevice (SPI2 async bus + NSS Output + a
// blocking/async `Delay`), with the RESET Output and DIO0 Input as the lora-phy interface-variant pins. Matches the
// node binaries' aliases; this prototype runs everything in `main`, but the alias keeps the types readable.
type RadioSpi = ExclusiveDevice<Spi<'static, Async>, Output<'static>, Delay>;
type Radio = Sx1276Radio<RadioSpi, Output<'static>, Input<'static>>;
type Link = LoraLink<Radio, Delay>;

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
  let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
  let peripherals = esp_hal::init(config);

  // `board_pins!` partial-moves only the GPIO pin fields out of `peripherals` into BoardPins, so the binding retains
  // the controller singletons (SPI2/TIMG0/SW_INTERRUPT/...) and we use them directly by field access.
  let pins = board::board_pins!(peripherals);

  // esp-rtos drives Embassy off a TIMG timer + a software interrupt; this runs before any timing primitive is awaited.
  let timg0 = TimerGroup::new(peripherals.TIMG0);
  let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
  esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

  // Boot banner: role, radio params, and the LoRa pin assignments (surfaced from the board crate's pub consts) so the
  // serial log is self-documenting and a wiring fault is easy to cross-check against the schematic.
  println!();
  #[cfg(not(feature = "ponger"))]
  println!("=== lora-ping: PINGER (default build) — measures ping->pong RTT ===");
  #[cfg(feature = "ponger")]
  println!("=== lora-ping: PONGER (--features ponger) — echoes packets back ===");
  println!(
    "radio: {} Hz | SF7 | BW500kHz | CR4/5 | power {} dBm | tx_boost {}",
    FREQUENCY_HZ, OUTPUT_POWER_DBM, TX_BOOST
  );
  println!("note: RFM95W routes PA_BOOST only — tx_boost MUST be true or output is near-zero.");
  println!(
    "LoRa pins: SCK=GPIO{} MOSI=GPIO{} MISO=GPIO{} NSS=GPIO{} RESET=GPIO{} DIO0=GPIO{}",
    board::LORA_SCK, board::LORA_MOSI, board::LORA_MISO, board::LORA_NSS, board::LORA_RESET, board::LORA_DIO0
  );

  // Build the SX1276 SPI bus on SPI2 at 1 MHz, mode 0 (the SX1276 default), created blocking then converted to async.
  // NSS/RESET are push-pull Outputs (idle high); DIO0 is the IRQ Input the RX/TX futures await. ExclusiveDevice owns
  // the bus + NSS and toggles CS around each transfer. Identical to the node binaries' radio construction.
  let spi_bus = Spi::new(
    peripherals.SPI2,
    SpiConfig::default().with_frequency(Rate::from_mhz(1)).with_mode(Mode::_0),
  )
  .expect("SPI2 config")
  .with_sck(pins.lora_sck)
  .with_mosi(pins.lora_mosi)
  .with_miso(pins.lora_miso)
  .into_async();

  let nss = Output::new(pins.lora_nss, Level::High, OutputConfig::default());
  let reset = Output::new(pins.lora_reset, Level::High, OutputConfig::default());
  let dio0 = Input::new(pins.lora_dio0, InputConfig::default().with_pull(Pull::None));
  let spi_dev = ExclusiveDevice::new(spi_bus, nss, Delay).expect("ExclusiveDevice");

  // Bring up the radio. build_sx1276 fixes SF7/BW500/CR4-5 @ 915 MHz (lora-link's defaults). On failure DO NOT panic
  // silently — log the error and park, because an error here almost always means a wiring fault (NSS/RESET/DIO0 or a
  // SPI line), which is exactly what this prototype exists to catch.
  println!("radio init: building SX1276 ...");
  let link: Link = match lora_link::build_sx1276(spi_dev, reset, dio0, Delay, TX_BOOST).await {
    Ok(link) => {
      println!("radio init: OK");
      link
    }
    Err(e) => {
      println!("radio init: FAILED ({:?})", e);
      println!("check SPI wiring (SCK/MOSI/MISO), NSS, RESET, DIO0, and 3.3 V power to the module. Parking.");
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
/// STATS_EVERY pings. A timeout prints an explicit wiring/power hint rather than deadlocking.
#[cfg(not(feature = "ponger"))]
async fn pinger(mut link: Link) -> ! {
  println!(
    "pinger: sending one ping every {} ms, echo timeout {} ms",
    PING_PERIOD.as_millis(),
    ECHO_TIMEOUT.as_millis()
  );
  println!();

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
      println!("seq {}: TX error: {:?}", seq, e);
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
          println!("seq {}: RTT {} us | RSSI {} dBm | SNR {} dB", seq, rtt_us, status.rssi, status.snr);
        } else {
          // A packet arrived but it was not our echo (wrong seq or magic): count it as lost for this seq and note it.
          lost += 1;
          println!("seq {}: stray/mismatched packet ({} bytes) — not our echo", seq, len);
        }
      }
      Either::First(Err(e)) => {
        lost += 1;
        println!("seq {}: RX error: {:?}", seq, e);
      }
      Either::Second(()) => {
        lost += 1;
        println!(
          "seq {}: no echo for {} ms — check ponger powered / antenna / NSS-RESET-DIO0 wiring",
          seq, ECHO_TIMEOUT.as_millis()
        );
      }
    }

    // Periodic rolling summary so link health is visible without scanning every line.
    if (seq + 1) % STATS_EVERY == 0 {
      if recv > 0 {
        println!(
          "stats: pings {} | echoes {} | lost {} | RTT min {} / avg {} / max {} us",
          seq + 1, recv, lost, min_us, sum_us / recv as u64, max_us
        );
      } else {
        println!("stats: pings {} | echoes 0 | lost {} | no RTT yet (no valid echoes)", seq + 1, lost);
      }
      println!();
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
/// exercises the ponger's own RX->TX turnaround and the full SPI/NSS/RESET/DIO0 path on its board.
#[cfg(feature = "ponger")]
async fn ponger(mut link: Link) -> ! {
  println!("ponger: listening, will echo every packet back with magic 0x{:02X}", PONG_MAGIC);
  println!();

  let mut rx_buf = [0u8; lora_link::MAX_PAYLOAD as usize];

  loop {
    let (len, status) = match link.receive_with_status(&mut rx_buf).await {
      Ok(result) => result,
      Err(e) => {
        println!("RX error: {:?}", e);
        continue;
      }
    };

    // Decode the seq for the log if the packet is the expected shape; otherwise still echo whatever we got so the
    // pinger's mismatch path is exercised too, but flag it.
    if len == PAYLOAD_LEN && rx_buf[4] == PING_MAGIC {
      let seq = u32::from_le_bytes([rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]]);
      println!("heard seq {}: RSSI {} dBm | SNR {} dB — echoing", seq, status.rssi, status.snr);
    } else {
      println!(
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
      println!("echo TX error: {:?}", e);
    }
  }
}
