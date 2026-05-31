//! Goggle node: decode the Skyzone goggle PPM head-tracking stream and drive the goggle->truck side of the
//! half-duplex LoRa link.
//!
//! Two Embassy tasks share a single latest-value `Signal` (lossy — only the freshest head pose matters, stale
//! samples are dropped, never queued):
//!   - `ppm_task`  — `Input` on `pins.ppm`, `wait_for_rising_edge().await` + `Instant` deltas fed to a
//!     `ppm_decoder::PpmDecoder`; on each decoded `Frame` it pulls the pan/tilt channels and signals a
//!     `proto::Control`.
//!   - `lora_task` — builds the SX1276 radio from `pins.lora_*`, then on a fixed ~50 Hz cadence takes the
//!     latest `Control`, encodes it with micropb, transmits, and listens briefly for the truck's `Telemetry`
//!     reply, logging the ground speed (km/h) over the C6's built-in USB Serial/JTAG (the goggle has no OLED).
//!
//! Half-duplex turnaround (scaffold scheme, tunable on hardware): the goggle is the link master. It TXes a
//! Control, then immediately listens for one Telemetry reply with a short bounded timeout before the next
//! cycle. The truck (the slave) only TXes its Telemetry right after it RXes a Control, so a single radio per
//! node stays collision-free. See `truck-node` for the matching slave half.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};
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
use micropb::{MessageDecode, MessageEncode, PbEncoder};
use ppm_decoder::{Config as PpmConfig, PpmDecoder};
use proto::{Control, Telemetry};

// Manual panic handler: log over USB Serial/JTAG then park. esp-rtos owns the runtime, so we keep this minimal
// rather than pulling in esp-backtrace (matches ppm-diag).
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
  println!("PANIC: {}", info);
  loop {}
}

// App descriptor required by the esp-idf second-stage bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

// Skyzone head-tracker channel indices, 0-based (menu "channel 5/6" are indices 4/5). CONFIRMED on hardware via
// `ppm-diag` (2026-05-30): ch5 = pan = index 4, ch6 = tilt = index 5; these are the documented SKY01/SKY02
// defaults but remain user-reconfigurable per goggle.
const PAN_CHANNEL: usize = 4;
const TILT_CHANNEL: usize = 5;

// Default pulse width signalled until a real PPM frame arrives, so the truck centers rather than slamming a stop.
const CENTER_US: u32 = 1_500;

// LoRa TX cadence: ~50 Hz head-tracking update. One Control goes out per tick, then we listen for the reply.
const TX_PERIOD: Duration = Duration::from_millis(20);

// How long to wait for the truck's Telemetry reply before giving up this cycle and moving on. Kept short so a
// missed reply never stalls the head-tracking TX rate. TODO: tune on hardware against measured round-trip time.
const REPLY_TIMEOUT: Duration = Duration::from_millis(12);

// TX power in dBm passed to lora-phy's prepare_for_tx. 17 dBm is a safe RFM95W PA_BOOST default (tx_boost is
// enabled in the radio build below). TODO: tune on hardware / per regional limits.
const TX_POWER_DBM: i32 = 17;

// Latest pan/tilt command, published by the PPM reader and consumed by the LoRa task. CriticalSectionRawMutex
// because the Signal is a `static` shared across tasks (and potentially executors).
static CONTROL: Signal<CriticalSectionRawMutex, Control> = Signal::new();

// Concrete radio type for this board: the SX1276 over an ExclusiveDevice (SPI2 async bus + NSS Output + a
// blocking/async `Delay`), with the RESET Output and DIO0 Input as the lora-phy interface-variant pins. Named
// so the spawned `lora_task` has a non-generic signature (an Embassy task cannot be generic).
type RadioSpi = ExclusiveDevice<Spi<'static, Async>, Output<'static>, Delay>;
type Radio = Sx1276Radio<RadioSpi, Output<'static>, Input<'static>>;
type Link = LoraLink<Radio, Delay>;

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
  let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
  let peripherals = esp_hal::init(config);

  // `board_pins!` partial-moves only the GPIO pin fields out of `peripherals` into BoardPins, so the binding
  // retains the controller singletons (SPI2/TIMG0/SW_INTERRUPT/...) and we use them directly by field access.
  let pins = board::board_pins!(peripherals);

  // esp-rtos drives Embassy off a TIMG timer + a software interrupt; this runs before any timing primitive is
  // awaited or any task is spawned.
  let timg0 = TimerGroup::new(peripherals.TIMG0);
  let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
  esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

  println!();
  println!("=== goggle-node: PPM -> LoRa TX, Telemetry RX ===");

  // PPM line idles low and pulses high (matches ppm-diag): pull-down keeps it defined when unplugged.
  let ppm = Input::new(pins.ppm, InputConfig::default().with_pull(Pull::Down));

  // Build the SX1276 SPI bus on SPI2 at 1 MHz, mode 0 (the SX1276 default). The bus is created blocking then
  // converted to async; NSS/RESET are push-pull Outputs (idle high), DIO0 is the IRQ Input the RX/TX futures
  // await. ExclusiveDevice owns the bus + NSS and toggles CS around each transfer.
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

  // Bare RFM95W: enable tx_boost for the PA_BOOST output pin. build_sx1276 fixes SF7/BW500/CR4-5 @ 915 MHz.
  let link: Link = lora_link::build_sx1276(spi_dev, reset, dio0, Delay, true)
    .await
    .expect("SX1276 radio init");

  // The task macro returns a Result<SpawnToken, SpawnError> (the pool-full case); unwrap the token then spawn.
  spawner.spawn(ppm_task(ppm).expect("ppm_task token"));
  spawner.spawn(lora_task(link).expect("lora_task token"));

  // Nothing else for the entry task to do; the executor keeps the spawned tasks running.
  loop {
    Timer::after(Duration::from_secs(3600)).await;
  }
}

/// PPM reader task. Times rising edges with `Instant`, feeds inter-edge deltas to the decoder, and on each
/// completed frame publishes the latest pan/tilt as a `Control` into the shared `Signal`. The Signal is lossy
/// by design — if the LoRa task is mid-transmit, intervening frames are simply overwritten.
#[embassy_executor::task]
async fn ppm_task(mut ppm: Input<'static>) {
  let mut decoder = PpmDecoder::new(PpmConfig::default());
  let mut last_edge: Option<Instant> = None;

  loop {
    ppm.wait_for_rising_edge().await;
    let now = Instant::now();
    let Some(prev) = last_edge else {
      last_edge = Some(now);
      continue;
    };
    last_edge = Some(now);

    let delta_us = now.duration_since(prev).as_micros() as u32;
    if let Some(frame) = decoder.feed(delta_us) {
      // Fall back to center for an axis the frame did not carry, so a short frame never sends a stale/garbage us.
      let pan_us = frame.channel(PAN_CHANNEL).map(u32::from).unwrap_or(CENTER_US);
      let tilt_us = frame.channel(TILT_CHANNEL).map(u32::from).unwrap_or(CENTER_US);
      CONTROL.signal(Control { pan_us, tilt_us });
    }
  }
}

/// LoRa task (link master). On each ~50 Hz tick: take the freshest `Control`, encode it, transmit, then listen
/// for one `Telemetry` reply within `REPLY_TIMEOUT`; decode and log the ground speed (km/h). A missed reply is
/// logged and the loop moves on so the head-tracking TX rate is never held hostage by the truck's reply.
#[embassy_executor::task]
async fn lora_task(mut link: Link) {
  let mut ticker = Ticker::every(TX_PERIOD);
  // Until the first PPM frame, send center so the truck has a defined pose.
  let mut latest = Control { pan_us: CENTER_US, tilt_us: CENTER_US };
  let mut rx_buf = [0u8; lora_link::MAX_PAYLOAD as usize];

  loop {
    ticker.next().await;

    // Latest-value hand-off: if a fresher Control is waiting, take it; otherwise resend the last one so a
    // momentary PPM gap still keeps the link alive at a steady rate.
    if let Some(control) = CONTROL.try_take() {
      latest = control;
    }

    // Encode the Control into a heapless Vec<u8, 12> (covers any two-uint32 message), then transmit. micropb's
    // container-heapless-0-9 implements PbWrite for exactly this heapless 0.9 Vec.
    let mut enc = PbEncoder::new(heapless::Vec::<u8, 12>::new());
    if latest.encode(&mut enc).is_err() {
      println!("control encode failed");
      continue;
    }
    let payload = enc.into_writer();
    if let Err(e) = link.send(&payload, TX_POWER_DBM).await {
      println!("LoRa TX error: {:?}", e);
      continue;
    }

    // Listen for the truck's Telemetry reply, bounded by REPLY_TIMEOUT so a lost reply cannot stall the loop.
    match select(link.receive(&mut rx_buf), Timer::after(REPLY_TIMEOUT)).await {
      Either::First(Ok(len)) => {
        let mut telem = Telemetry::default();
        if telem.decode_from_bytes(&rx_buf[..len]).is_ok() {
          // km/h = cm_s * 36 / 1000, integer rounded — the goggle's "display" is this serial line. speed_cm_s is
          // a freshly-decoded (externally controllable) uint32, so widen to u64 first: the product can never wrap.
          let kmh = ((telem.speed_cm_s as u64 * 36 + 500) / 1000) as u32;
          println!("telemetry: {} km/h | sats {} | fix {}", kmh, telem.sats, telem.fix_quality);
        } else {
          println!("telemetry decode failed ({} bytes)", len);
        }
      }
      Either::First(Err(e)) => println!("LoRa RX error: {:?}", e),
      Either::Second(()) => { /* no reply this cycle — expected occasionally, just move on. */ }
    }
  }
}
