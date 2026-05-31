//! truck-diag :: gps — UART NMEA reader, in ISOLATION from the rest of the truck node.
//!
//! Default (parsed) mode: opens UART1 at 9600 baud on `gps_tx` / `gps_rx`, wraps it in `gps::GpsReader`, loops
//! `next_fix`, and prints ground speed (cm/s), the derived km/h, satellite count, and fix quality. This validates
//! the hand-rolled NMEA parser against a real module end to end.
//!
//! Raw mode (`--features gps-raw`, default OFF): skips the parser entirely and echoes the incoming UART bytes to
//! the console. This is the CRITICAL distinguisher when no fixes appear — it shows whether bytes are even arriving
//! and whether they look like NMEA sentences, separating a wiring/baud fault (no bytes, or garbage) from a parser
//! fault (clean `$G...` sentences but no parsed fix).
//!
//! GOOD (parsed): once the module has a fix, lines print rising satellite counts, fix quality 1+, and a plausible
//! speed (near 0 at rest). GOOD (raw): readable `$GPGGA,... / $GPRMC,...` sentences scroll by. FAILURE / wiring
//! signature: nothing prints at all, or raw mode shows garbage -> wrong RX/TX pins (RX must see the module's TX),
//! wrong baud, or no power/antenna. NOTE: a GPS needs OPEN SKY for a fix, and speed is only meaningful WITH a fix.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use esp_hal::clock::CpuClock;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_println::println;

// Pull in the shared #[panic_handler] + esp_app_desc!() (defined in the crate lib so all four bins reuse them).
use truck_diag as _;

// Most NMEA modules default to 9600 baud. TODO: confirm against the specific receiver (some default to 38400/115200).
const GPS_BAUD: u32 = 9_600;

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
  let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
  let peripherals = esp_hal::init(config);

  // board_pins! partial-moves only the GPIO pin fields, leaving UART1 / TIMG0 / SW_INTERRUPT on `peripherals`.
  let pins = board::board_pins!(peripherals);

  let timg0 = TimerGroup::new(peripherals.TIMG0);
  let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
  esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

  println!();
  println!("=== truck-diag/gps: UART NMEA reader ===");
  println!("UART TX GPIO{} (config only), RX GPIO{} (module's NMEA), {} baud", board::GPS_TX, board::GPS_RX, GPS_BAUD);
  println!("NOTE: a GPS needs OPEN SKY for a fix; speed is only meaningful with a fix.");
  #[cfg(feature = "gps-raw")]
  println!("MODE: RAW byte echo (gps-raw) — printing incoming UART bytes, NOT parsing. Look for readable $G... lines.");
  #[cfg(not(feature = "gps-raw"))]
  println!("MODE: parsed — printing speed/sats/fix. If nothing prints, re-flash with `--features gps-raw`.");
  println!();

  // GPS UART on UART1 (UART0 backs the console). RX carries the NMEA stream into the C6; TX is config-only and
  // unused by the reader. Built blocking then converted to async, exactly as the truck node does.
  let uart = Uart::new(peripherals.UART1, UartConfig::default().with_baudrate(GPS_BAUD))
    .expect("UART1 config")
    .with_rx(pins.gps_rx)
    .with_tx(pins.gps_tx)
    .into_async();

  #[cfg(feature = "gps-raw")]
  raw_loop(uart).await;
  #[cfg(not(feature = "gps-raw"))]
  parsed_loop(uart).await;
}

/// Parsed mode: feed the UART into gps::GpsReader and print each completed speed fix with its satellite/fix info.
/// This is the real validation of the NMEA parser against live data.
#[cfg(not(feature = "gps-raw"))]
async fn parsed_loop(uart: Uart<'static, esp_hal::Async>) -> ! {
  use gps::GpsReader;

  let mut reader = GpsReader::new(uart);
  loop {
    match reader.next_fix().await {
      Ok(fix) => {
        let kmh = display::cm_s_to_kmh(fix.speed_cm_s);
        let sats = fix.satellites.map(|s| s as u32).unwrap_or(0);
        let quality = fix.fix.map(|f| f.raw as u32).unwrap_or(0);
        println!("speed {} cm/s ({} km/h) | sats {} | fix quality {}", fix.speed_cm_s, kmh, sats, quality);
      }
      // A UART read error (framing/overrun) is transient; pace retries so a wiring fault does not spin the console hot.
      Err(_) => {
        println!("UART read error — check RX wiring + baud (try `--features gps-raw` to see raw bytes)");
        embassy_time::Timer::after(embassy_time::Duration::from_millis(200)).await;
      }
    }
  }
}

/// Raw mode: read straight off the async UART and echo bytes so a wiring/baud problem (no bytes / garbage) is
/// distinguishable from a parser problem (clean sentences, no parsed fix). Prints each chunk as it arrives.
#[cfg(feature = "gps-raw")]
async fn raw_loop(mut uart: Uart<'static, esp_hal::Async>) -> ! {
  use embedded_io_async::Read;

  let mut buf = [0u8; 64];
  loop {
    // Call the async-trait read explicitly: esp-hal's Uart has an inherent blocking `read` that would otherwise
    // shadow the embedded_io_async::Read method (which is what GpsReader uses in parsed mode).
    match Read::read(&mut uart, &mut buf).await {
      Ok(0) => {} //  no data yet — keep awaiting.
      Ok(n) => {
        // Echo the raw bytes as a lossy UTF-8 view so NMEA sentences are readable while non-ASCII garbage (wrong
        // baud) still shows up as replacement characters rather than being hidden.
        let text = core::str::from_utf8(&buf[..n]).unwrap_or("<non-utf8 bytes — wrong baud?>");
        esp_println::print!("{}", text);
      }
      Err(_) => {
        println!();
        println!("UART read error — check RX wiring + baud");
        embassy_time::Timer::after(embassy_time::Duration::from_millis(200)).await;
      }
    }
  }
}
