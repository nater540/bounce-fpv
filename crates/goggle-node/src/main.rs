//! Goggle node (Phase D: nRF52840): decode the Skyzone goggle PPM head-tracking stream and drive the
//! goggle->truck side of the half-duplex LoRa link, now with the status OLED + a lap-reset button (both moved
//! onto the goggle in this phase).
//!
//! Four Embassy tasks, wired by latest-value `Signal`s (lossy — only the freshest value matters, stale samples
//! are dropped, never queued):
//!   - `ppm_task`    — `Input` on `pins.ppm`, `wait_for_rising_edge().await` + `Instant` deltas fed to a
//!     `ppm_decoder::PpmDecoder`; on each decoded `Frame` it pulls the pan/tilt channels and signals `CONTROL`.
//!   - `lora_task`   — builds the SX1276 radio, then on a fixed ~50 Hz cadence takes the latest `Control`,
//!     encodes it with micropb, transmits, and listens briefly for the truck's `Telemetry` reply; on a decode
//!     it signals the ground speed into `SPEED` for the OLED.
//!   - `oled_task`   — `display::StatusDisplay` on a direct `Twim` (the SSD1306 is the only I2C device here, so
//!     no shared bus). Renders the truck's speed (from `SPEED`) + link/fix flags + a lap-timer line (whole
//!     seconds since the last reset).
//!   - `button_task` — `Input` on `pins.button` (active-low, internal pull-up); on press it signals `LAP_RESET`
//!     so the OLED re-bases its lap-timer origin (a simple elapsed-since-reset stopwatch for now; richer lap
//!     detection is a future TODO).
//!
//! Half-duplex turnaround (scaffold scheme, tunable on hardware): the goggle is the link master. It TXes a
//! Control, then immediately listens for one Telemetry reply with a short bounded timeout before the next
//! cycle. The truck (the slave) only TXes its Telemetry right after it RXes a Control, so a single radio per
//! node stays collision-free. See `truck-node` for the matching slave half.

#![no_std]
#![no_main]

// Pull in the shared #[panic_handler] (defined once in applog). applog ALSO provides defmt-rtt's #[global_logger]
// (it does `use defmt_rtt as _;`), which is what lets lora-phy's unconditional defmt dependency link here without
// any per-binary defmt setup.
use applog as _;

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_nrf::gpio::{Input, Pull};
use embassy_nrf::spim;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker, Timer};
use micropb::{MessageDecode, MessageEncode, PbEncoder};
use ppm_decoder::{Config as PpmConfig, PpmDecoder};
use proto::{Control, Telemetry};
use static_cell::StaticCell;

// Bind the SERIAL interrupts this node uses: SPIM3 (LoRa SPI bus) and TWISPI0 (OLED I2C). USBD is bound by applog
// — do NOT bind it here. GPIOTE (PPM + DIO0 + button edge waits) is bound by embassy-nrf's init at the SD-safe P2
// priority via init_embassy_nrf.
bind_interrupts!(struct Irqs {
  SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
  TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

// Skyzone head-tracker channel indices, 0-based (menu "channel 5/6" are indices 4/5). CONFIRMED on hardware via
// `ppm-diag` (2026-05-30): ch5 = pan = index 4, ch6 = tilt = index 5; these are the documented SKY01/SKY02
// defaults but remain user-reconfigurable per goggle.
const PAN_CHANNEL: usize = 4;
const TILT_CHANNEL: usize = 5;

// Default pulse width signalled until a real PPM frame arrives, so the truck centers rather than slamming a stop.
const CENTER_US: u32 = 1_500;

// LoRa TX cadence: one Control out per tick, then listen for the reply. Sized to the measured SF7/BW500 airtime:
// a ~12-byte packet is ~10 ms on air, so a full Control-out + Telemetry-back cycle is send (~10 ms) + reply wait
// (REPLY_TIMEOUT). 40 ms (~25 Hz) covers that with headroom — still ample for head tracking, and the radio's air
// time caps the practical round-trip rate near here anyway. TODO: tighten once the on-hardware RTT is measured.
const TX_PERIOD: Duration = Duration::from_millis(40);

// Upper bound on a single transmit. `send()` blocks on the DIO0 TX-done IRQ; if that IRQ is ever missed (e.g. GPIOTE
// sense contention between the high-rate PPM edge waits and DIO0 on this node), the unbounded await would hang the
// whole TX loop permanently — the "link dies after ~70 s, only a goggle reboot recovers" symptom. Bounding it lets the
// loop re-prepare the radio next cycle and self-heal. 30 ms is ~3x the ~10 ms SF7/BW500 airtime, so a normal TX never
// trips it. TODO: the deeper fix is isolating PPM from DIO0 (a dedicated GPIOTE channel or an InterruptExecutor).
const TX_TIMEOUT: Duration = Duration::from_millis(30);

// How long to wait for the truck's Telemetry reply before giving up this cycle. This MUST exceed one reply's flight:
// the truck only finishes receiving our Control when our TX ends, then turns around and sends ~10+ ms of telemetry,
// so the reply lands ~15-17 ms into this window. The old 12 ms was shorter than a single packet's airtime and missed
// every reply (the LINK:N symptom). 30 ms gives ~2x margin. TODO: tune down once the real RTT is measured.
const REPLY_TIMEOUT: Duration = Duration::from_millis(30);

// TX power in dBm passed to lora-phy's prepare_for_tx. Dropped from 17 to 10 dBm: at 17 dBm on the RFM95W's PA_BOOST,
// sustained ~25 %-duty transmitting trips the PA's over-current/thermal protection after ~a minute — TX-done still
// fires but NO RF radiates, so the truck goes silent and the link dies until a reboot (the observed failure). The
// bench link sits at ~-41 dBm RSSI (~30 dB of margin), so 10 dBm is plenty and keeps the PA well under its limit.
// TODO: raise toward 14-17 dBm only with a proper antenna + heatsinking and once real range needs it.
const TX_POWER_DBM: i32 = 10;

// PA output routing. The bare RFM95W bonds ONLY its PA_BOOST pin to the antenna, so tx_boost MUST be true or the
// radiated power is near-zero and the link silently fails to come up (see lora-ping for the full rationale).
const TX_BOOST: bool = true;

// OLED refresh cadence. The panel only shows a human-readable glance, so a few Hz is plenty.
const OLED_PERIOD: Duration = Duration::from_millis(250);

// Link liveness window: if no Telemetry reply has been decoded within this span, the OLED shows LINK:N. A couple
// of TX cycles' worth of slack so a single dropped reply does not flap the flag. TODO: tune on hardware.
const LINK_TIMEOUT: Duration = Duration::from_millis(500);

// Latest pan/tilt command, published by the PPM reader and consumed by the LoRa task. CriticalSectionRawMutex
// because the Signal is a `static` shared across tasks.
static CONTROL: Signal<CriticalSectionRawMutex, Control> = Signal::new();
// Latest truck telemetry decoded by the LoRa task, consumed by the OLED task to render speed + fix flags.
static SPEED: Signal<CriticalSectionRawMutex, Telemetry> = Signal::new();
// Lap-reset edge from the button, consumed by the OLED task to reset its placeholder lap timer. The unit payload
// is just an event marker — the OLED re-bases its elapsed-time origin when it fires.
static LAP_RESET: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// The radio type lives in `nrf_adapters::lora` now (shared by both nodes + lora-ping so it can never drift); the
// spawned LoRa task takes `nrf_adapters::lora::Link` directly for its non-generic signature.
// The OLED's direct (non-shared) I2C device: embassy-nrf's Twim implements embedded-hal-async I2c, exactly the
// bound display::StatusDisplay::new requires.
type OledI2c = Twim<'static>;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
  // embassy-nrf init at SoftDevice-safe interrupt priorities (GPIOTE + time-driver at P2; the SD reserves P0/P1/P4).
  let p = applog::init_embassy_nrf();
  // board_pins! partial-moves only the GPIO pin fields out of `p`, leaving the controller singletons (SPI3,
  // TWISPI0, USBD, ...) on `p` for the rest of main.
  let pins = board::board_pins!(p);

  // SD COEXISTENCE: init_embassy_nrf centrally lowers all SERIAL/SPIM/TWIM/UARTE IRQs to P2 (the SD-safe band with
  // GPIOTE + the time driver), so the per-binary set_priority calls are gone — the peripheral constructors below
  // inherit P2. CONFIRM ON-TARGET that SPIM3 + TWISPI0 + GPIOTE coexist with the SD enabled.

  applog::init(
    spawner,
    p.USBD,
    applog::UsbIdentity::new(0x1209, 0x0001, "fabulous-fpv", "goggle", "d"),
  );

  applog::log_println!("");
  applog::log_println!("=== goggle-node: PPM -> LoRa TX, Telemetry RX -> OLED (nRF52840) ===");

  // PPM line from the SKY04X HT jack: it idles HIGH (~5 V on the tip; the jack is PPM+GND only, NO power pin) with
  // brief LOW markers, so the FALLING edge marks each channel position. Pull::Down keeps the line defined LOW when the
  // goggle is unplugged and helps the markers cross the threshold; the series protection resistor clamps the 5 V swing
  // (the nRF is NOT 5 V tolerant). Confirmed on hardware via `ppm-diag` (the idle-high/falling build).
  let ppm = Input::new(pins.ppm, Pull::Down);

  // Lap-reset button: active-low to GND with an internal pull-up, so the line idles high and a press drives it low
  // (a falling edge). GPIOTE backs the edge wait.
  let button = Input::new(pins.button, Pull::Up);

  // Build the SX1276/RFM95W link via the shared nrf-adapters helper (Spim on SPI3 + NSS/RESET Outputs + DIO0 Input
  // + ExclusiveDevice, all centralized). Pins go in sck/mosi/miso then nss/reset/dio0 order; tx_boost drives the
  // bare module's PA_BOOST output. On an init error (e.g. a wiring fault) the deployed node must NOT panic into
  // DFU — log and park forever instead, matching the lora-ping diagnostic.
  let link = match nrf_adapters::lora::build_lora_link(
    p.SPI3, Irqs, pins.lora_sck, pins.lora_mosi, pins.lora_miso, pins.lora_nss, pins.lora_reset, pins.lora_dio0,
    TX_BOOST,
  )
  .await
  {
    Ok((link, version)) => {
      // 0x12 confirms SPI reaches the SX1276; any other value flags a dead SCK/MOSI/MISO/NSS path at boot.
      applog::log_println!("radio init OK | SX127x version 0x{:02X}", version);
      link
    }
    Err(e) => {
      applog::log_println!("radio init FAILED ({:?})", e);
      loop {
        Timer::after_secs(60).await;
      }
    }
  };

  // OLED I2C on TWISPI0. The SSD1306 is the only I2C device here, so it gets a direct Twim (no shared bus). The
  // TWIM tx_ram_buffer must be 'static (it outlives the 'static peripheral); a StaticCell gives it that, and 16
  // bytes is ample for the small command bytes the driver sends (ssd1306 flushes its framebuffer from its own RAM).
  static TWIM_TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
  let tx_buf = TWIM_TX_BUF.init([0; 16]);
  // Enable the internal SDA/SCL pull-ups (Config::default enables NEITHER): the OLED bus needs them on bare wiring,
  // the same fix that brought the panel up in lora-ping. Both flags set — embassy-nrf gates both lines off sda_pullup.
  let mut oled_i2c_cfg = twim::Config::default();
  oled_i2c_cfg.sda_pullup = true;
  oled_i2c_cfg.scl_pullup = true;
  let oled_i2c = Twim::new(p.TWISPI0, Irqs, pins.i2c_sda, pins.i2c_scl, oled_i2c_cfg, tx_buf);

  // The task macro returns a Result<SpawnToken, SpawnError> (the pool-full case); unwrap the token then spawn.
  spawner.spawn(ppm_task(ppm).expect("ppm_task token"));
  spawner.spawn(lora_task(link).expect("lora_task token"));
  spawner.spawn(oled_task(oled_i2c).expect("oled_task token"));
  spawner.spawn(button_task(button).expect("button_task token"));
}

/// PPM reader task. Times FALLING edges with `Instant` (the SKY04X PPM idles high and dips low for each marker),
/// feeds inter-edge deltas to the decoder, and on each completed frame publishes the latest pan/tilt as a `Control`
/// into the shared `Signal`. The Signal is lossy by design — if the LoRa task is mid-transmit, intervening frames
/// are simply overwritten.
#[embassy_executor::task]
async fn ppm_task(mut ppm: Input<'static>) {
  // Default Config suppresses corrupt frames (unlike ppm-diag, which surfaces them) — the production node only
  // wants clean frames, so a noisy/partial frame never reaches the servos through the link.
  let mut decoder = PpmDecoder::new(PpmConfig::default());
  let mut last_edge: Option<Instant> = None;

  loop {
    ppm.wait_for_falling_edge().await;
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
/// for one `Telemetry` reply within `REPLY_TIMEOUT`; on a decode it signals the telemetry into `SPEED` for the
/// OLED. A missed reply is logged and the loop moves on so the head-tracking TX rate is never held hostage by
/// the truck's reply.
#[embassy_executor::task]
async fn lora_task(mut link: nrf_adapters::lora::Link) {
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
      applog::log_println!("control encode failed");
      continue;
    }
    let payload = enc.into_writer();
    // Bounded TX: a never-completing send (missed DIO0 TX-done, likely GPIOTE contention with the PPM edge waits)
    // would otherwise hang the whole transmit loop forever. On a timeout, drop this cycle and re-prepare the radio
    // next tick so the link self-heals instead of dying until a reboot.
    match select(link.send(&payload, TX_POWER_DBM), Timer::after(TX_TIMEOUT)).await {
      Either::First(Ok(())) => {}
      Either::First(Err(e)) => {
        applog::log_println!("LoRa TX error: {:?}", e);
        continue;
      }
      Either::Second(()) => {
        applog::log_println!("LoRa TX timed out (no TX-done) — recovering");
        continue;
      }
    }

    // Listen for the truck's Telemetry reply, bounded by REPLY_TIMEOUT so a lost reply cannot stall the loop.
    match select(link.receive(&mut rx_buf), Timer::after(REPLY_TIMEOUT)).await {
      Either::First(Ok(len)) => {
        let mut telem = Telemetry::default();
        if telem.decode_from_bytes(&rx_buf[..len]).is_ok() {
          // km/h = cm_s * 36 / 1000, integer rounded — logged here for the serial console; the freshest telemetry
          // is also handed to the OLED. speed_cm_s is an externally-controllable uint32, so widen to u64 first so
          // the product can never wrap (review fix #3).
          let kmh = ((telem.speed_cm_s as u64 * 36 + 500) / 1000) as u32;
          applog::log_println!("telemetry: {} km/h | sats {} | fix {}", kmh, telem.sats, telem.fix_quality);
          SPEED.signal(telem);
        } else {
          applog::log_println!("telemetry decode failed ({} bytes)", len);
        }
      }
      Either::First(Err(e)) => applog::log_println!("LoRa RX error: {:?}", e),
      Either::Second(()) => { /* no reply this cycle — expected occasionally, just move on. */ }
    }
  }
}

/// OLED status task. Renders the truck's ground speed (km/h, from `SPEED`) + link/fix flags + a lap-timer line
/// (whole seconds since the last reset) on a direct (non-shared) I2C bus. "Link up" is a real liveness check: a
/// Telemetry reply decoded within `LINK_TIMEOUT`. The lap timer is an elapsed-since-reset stopwatch — richer lap
/// detection is a future TODO; the button task's `LAP_RESET` re-bases the elapsed-time origin.
#[embassy_executor::task]
async fn oled_task(mut i2c: OledI2c) {
  // Probe 0x3C/0x3D first so a panel strapped to either address comes up (mirrors the lora-ping bring-up), and a
  // dead bus (no ACK) is reported distinctly. A missing/miswired panel must not brick the goggle, so either failure
  // logs and exits the task cleanly — the PPM->LoRa head-tracking path keeps running headless.
  let addr = match display::probe_address(&mut i2c).await {
    Some(a) => a,
    None => {
      applog::log_println!("OLED not found on I2C (no ACK at 0x3C/0x3D) — OLED task exiting, link still runs");
      return;
    }
  };
  let mut oled = match display::StatusDisplay::new_with_addr(i2c, addr).await {
    Ok(d) => d,
    Err(e) => {
      // A panel that ACKs but won't init (wrong driver, e.g. SH1106) also must not brick the goggle; log and exit.
      applog::log_println!("OLED init failed at 0x{:02X}: {:?}", addr, e);
      return;
    }
  };

  let mut ticker = Ticker::every(OLED_PERIOD);
  let mut last = Telemetry::default();
  // Timestamp of the last decoded Telemetry reply. Initialized far enough in the past that the link reads down
  // until a real reply arrives (Instant is monotonic, so duration_since never underflows). saturating_sub guards
  // the boot case where now < LINK_TIMEOUT.
  let mut last_rx = Instant::now().saturating_sub(LINK_TIMEOUT);
  // Lap stopwatch origin: re-based to now on each LAP_RESET (or boot). The OLED shows elapsed-since-reset in
  // whole seconds; richer lap detection is a future TODO.
  let mut lap_origin = Instant::now();

  loop {
    ticker.next().await;
    if let Some(telem) = SPEED.try_take() {
      last = telem;
      last_rx = Instant::now();
    }
    if LAP_RESET.try_take().is_some() {
      lap_origin = Instant::now();
    }

    // Real liveness, not a one-way latch: the link is up only while a reply has been seen within LINK_TIMEOUT, so
    // the OLED falls back to LINK:N when the truck goes away instead of showing LINK:Y forever.
    let link_up = Instant::now().duration_since(last_rx) < LINK_TIMEOUT;
    let gps_fix = last.fix_quality != 0;
    // Whole seconds since the last LAP_RESET (or boot). The display dirty-check skips the flush until this ticks
    // over, so the panel only re-renders the lap line ~1 Hz while everything else is steady.
    let lap_secs = Instant::now().duration_since(lap_origin).as_secs() as u32;
    let status = display::Status { speed_cm_s: last.speed_cm_s, link_up, gps_fix, lap_secs };
    if let Err(e) = oled.render(status).await {
      applog::log_println!("OLED render error: {:?}", e);
    }
  }
}

/// Lap-reset button task. The button is active-low with an internal pull-up, so a press is a falling edge; on each
/// press it signals `LAP_RESET` for the OLED to re-base its placeholder lap timer. Full lap logic is a future TODO.
#[embassy_executor::task]
async fn button_task(mut button: Input<'static>) {
  loop {
    // Wait for the press edge (active-low -> falling), then sleep the debounce window and RE-CHECK the line is
    // still low. A real press holds the line low past the window; a bounce/glitch has released by then and fires
    // no reset. Only on a confirmed press do we signal LAP_RESET once, then wait for the rising edge (release)
    // before re-arming — so one physical press = exactly one reset. TODO: tune the debounce window on hardware.
    button.wait_for_falling_edge().await;
    Timer::after(Duration::from_millis(50)).await;
    if button.is_low() {
      LAP_RESET.signal(());
      button.wait_for_rising_edge().await;
    }
  }
}
