//! truck-diag :: gps — UART NMEA reader, in ISOLATION from the rest of the truck node (Phase C: nRF52840).
//!
//! Default (parsed) mode: opens UARTE1 RX-only at 9600 baud on `gps_rx` via embassy-nrf's interrupt-driven
//! `BufferedUarteRx`, feeds it straight into `gps::GpsReader` (it natively impls `embedded_io_async::Read`), loops
//! `next_fix`, and prints ground speed (cm/s), the derived km/h, satellite count, and fix quality. This validates
//! the hand-rolled NMEA parser against a real module end to end.
//!
//! Raw mode (`--features gps-raw`, default OFF): skips the parser entirely and echoes the incoming UART bytes to the
//! console. This is the CRITICAL distinguisher when no fixes appear — it shows whether bytes are even arriving and
//! whether they look like NMEA sentences, separating a wiring/baud fault (no bytes, or garbage) from a parser fault
//! (clean `$G...` sentences but no parsed fix). `BufferedUarteRx` honors short reads — it returns the bytes already
//! ring-buffered instead of blocking for a full buffer — so raw mode now streams bytes as they arrive rather than
//! stalling until a whole chunk lands (the bug that defeated the old whole-buffer `UarteReadAdapter`).
//!
//! GOOD (parsed): once the module has a fix, lines print rising satellite counts, fix quality 1+, and a plausible
//! speed (near 0 at rest). GOOD (raw): readable `$GPGGA,... / $GPRMC,...` sentences scroll by. FAILURE / wiring
//! signature: nothing prints at all, or raw mode shows garbage -> wrong RX/TX pins (RX must see the module's TX),
//! wrong baud, or no power/antenna. NOTE: a GPS needs OPEN SKY for a fix, and speed is only meaningful WITH a fix.

#![no_std]
#![no_main]

// Pull in the shared #[panic_handler] from applog (replaces the old truck-diag lib panic handler + esp app_desc).
use applog as _;

use embassy_executor::Spawner;
use embassy_nrf::buffered_uarte::{self, BufferedUarteRx};
use embassy_nrf::uarte::{self, Baudrate};
use embassy_nrf::{bind_interrupts, peripherals};
use static_cell::StaticCell;

// Bind ONLY the UARTE1 interrupt this binary uses (the GPS UART on UARTE1). It MUST be the buffered-UARTE handler,
// not the plain `uarte::InterruptHandler` — `BufferedUarteRx` services its ring buffer from this ISR. USBD is bound
// by applog — do NOT bind it here.
bind_interrupts!(struct Irqs {
  UARTE1 => buffered_uarte::InterruptHandler<peripherals::UARTE1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
  // embassy-nrf init at SoftDevice-safe interrupt priorities (GPIOTE + time-driver at P2; the SD reserves P0/P1/P4).
  let p = applog::init_embassy_nrf();
  // board_pins! partial-moves only the GPIO pin fields, leaving the controller singletons (UARTE1, TIMER1, the PPI
  // channels/group BufferedUarteRx needs, USBD) on `p`.
  let pins = board::board_pins!(p);

  applog::init(
    spawner,
    p.USBD,
    applog::UsbIdentity::new(0x1209, 0x0001, "fabulous-fpv", "truck-diag-gps", "phase-c"),
  );

  applog::log_println!("");
  applog::log_println!("=== truck-diag/gps: UART NMEA reader (nRF52840) ===");
  applog::log_println!(
    "UARTE1 TX P{}.{:02} (config only), RX P{}.{:02} (module's NMEA), 9600 baud",
    board::GPS_TX_PORT, board::GPS_TX_PIN, board::GPS_RX_PORT, board::GPS_RX_PIN
  );
  applog::log_println!("NOTE: a GPS needs OPEN SKY for a fix; speed is only meaningful with a fix.");
  #[cfg(feature = "gps-raw")]
  applog::log_println!("MODE: RAW byte echo (gps-raw) — printing UART bytes, NOT parsing. Look for $G... lines.");
  #[cfg(not(feature = "gps-raw"))]
  applog::log_println!("MODE: parsed — printing speed/sats/fix. If nothing prints, re-flash with --features gps-raw.");
  applog::log_println!("");

  // GPS UART on UARTE1, RX-only. Most NMEA modules default to 9600 baud — TODO: confirm against the specific receiver
  // (some default to 38400/115200). `BufferedUarteRx` is interrupt-driven and ring-buffers RX bytes between reads, so
  // it never drops bytes mid-parse and honors short reads — exactly what `gps::GpsReader` (an `embedded_io_async::Read`
  // consumer) wants. It needs a TIMER + two PPI channels + a PPI group for its DMA/EasyDMA hand-off; we use TIMER1
  // (NOT TIMER0, which the SoftDevice reserves) and PPI_CH0/PPI_CH1/PPI_GROUP0, all SD-safe and otherwise unused by
  // this isolated diagnostic. The RX ring buffer must be a `'static mut [u8]` of EVEN length; a StaticCell gives it a
  // 'static home. The module's TX pin (`pins.gps_tx`) is intentionally unused — the reader never transmits.
  let mut config = uarte::Config::default();
  config.baudrate = Baudrate::BAUD9600;

  // 512-byte (EVEN) RX ring: comfortably larger than a single ~82-byte NMEA sentence, sized for scheduler-stall slack
  // so a burst of sentences is absorbed while the parser/USB-CDC path is busy. Overrun is NOT recoverable here — a
  // UARTE overrun with the ring full PANICS inside embassy-nrf's ISR — so the buffer must be generous, not retried.
  static RX_RING: StaticCell<[u8; 512]> = StaticCell::new();
  let rx_ring = RX_RING.init([0u8; 512]);

  let rx = BufferedUarteRx::new(
    p.UARTE1, p.TIMER1, p.PPI_CH0, p.PPI_CH1, p.PPI_GROUP0, Irqs, pins.gps_rx, config, rx_ring,
  );

  #[cfg(feature = "gps-raw")]
  raw_loop(rx).await;
  #[cfg(not(feature = "gps-raw"))]
  parsed_loop(rx).await;
}

/// Parsed mode: feed the UART RX into gps::GpsReader and print each completed speed fix with its satellite/fix info.
/// This is the real validation of the NMEA parser against live data. Never returns.
#[cfg(not(feature = "gps-raw"))]
async fn parsed_loop(rx: BufferedUarteRx<'static>) -> ! {
  use gps::GpsReader;

  let mut reader = GpsReader::new(rx);
  loop {
    match reader.next_fix().await {
      Ok(fix) => {
        let kmh = display::cm_s_to_kmh(fix.speed_cm_s);
        let sats = fix.satellites.map(|s| s as u32).unwrap_or(0);
        let quality = fix.fix.map(|f| f.raw as u32).unwrap_or(0);
        applog::log_println!("speed {} cm/s ({} km/h) | sats {} | fix quality {}", fix.speed_cm_s, kmh, sats, quality);
      }
      // NOT a retryable/"transient" error: a UARTE overrun (RX overrun + ring full) PANICS inside embassy-nrf's ISR,
      // never surfacing here, and BufferedUarteRx's `Error` enum is empty, so this arm is effectively unreachable. If
      // it ever fires, pace it so a wiring fault does not spin the console hot, then keep polling.
      Err(_) => {
        applog::log_println!("UART read error — check RX wiring + baud (try `--features gps-raw` to see raw bytes)");
        embassy_time::Timer::after(embassy_time::Duration::from_millis(200)).await;
      }
    }
  }
}

/// Raw mode: read straight off the UART RX and echo bytes so a wiring/baud problem (no bytes / garbage) is
/// distinguishable from a parser problem (clean sentences, no parsed fix). `BufferedUarteRx::read` honors short
/// reads — it returns whatever bytes are already ring-buffered rather than blocking for a full buffer — so each
/// chunk prints as it arrives, streaming the live NMEA feed instead of stalling until 32 bytes land. Never returns.
#[cfg(feature = "gps-raw")]
async fn raw_loop(mut rx: BufferedUarteRx<'static>) -> ! {
  // `BufferedUarteRx::read` is an inherent method, so no `embedded_io_async::Read` import is needed in raw mode
  // (the trait is what `gps::GpsReader` consumes internally in the parsed path).
  let mut buf = [0u8; 32];
  loop {
    match rx.read(&mut buf).await {
      Ok(0) => {} //  empty buffer fast-path — never hit here (buf is non-empty); keep awaiting.
      Ok(n) => {
        // Echo the raw bytes as a lossy UTF-8 view so NMEA sentences are readable while non-ASCII garbage (wrong
        // baud) still shows up as a marker rather than being hidden.
        let text = core::str::from_utf8(&buf[..n]).unwrap_or("<non-utf8 bytes — wrong baud?>");
        applog::log_println!("{}", text);
      }
      Err(_) => {
        applog::log_println!("UART read error — check RX wiring + baud");
        embassy_time::Timer::after(embassy_time::Duration::from_millis(200)).await;
      }
    }
  }
}
