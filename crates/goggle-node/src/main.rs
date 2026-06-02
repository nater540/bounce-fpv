//! Goggle node (Phase D: nRF52840): decode the Skyzone goggle PPM head-tracking stream and drive the
//! goggle->truck side of the half-duplex LoRa link, with the committed G1 "Stopwatch" status OLED + a lap
//! button (both on the goggle in this phase).
//!
//! Four Embassy tasks, wired by latest-value `Signal`s (lossy — only the freshest value matters, stale samples
//! are dropped, never queued):
//!   - `ppm_task`    — `Input` on `pins.ppm`, `wait_for_rising_edge().await` + `Instant` deltas fed to a
//!     `ppm_decoder::PpmDecoder`; on each decoded `Frame` it pulls the pan/tilt channels and signals both
//!     `CONTROL` (for the LoRa TX) and `HEAD` (the OLED's own copy of the pose).
//!   - `lora_task`   — builds the SX1276 radio, then on a fixed ~50 Hz cadence takes the latest `Control`,
//!     encodes it with micropb, transmits, and listens briefly for the truck's `Telemetry` reply; on a decode
//!     it signals the telemetry into `SPEED` (GPS sats for the OLED) and the reply RSSI into `LINK_RSSI`.
//!   - `oled_task`   — `display::StatusDisplay` on a direct `Twim` (the SSD1306 is the only I2C device here, so
//!     no shared bus). Owns the screen state and cycles between two layouts: the G1 Stopwatch (lap clock +
//!     pan/tilt + sats + bars) and the Nav "find my truck" screen (the truck's distance + bearing to its home,
//!     relayed in the telemetry). A long press resets the lap on Stopwatch or requests a re-home on Nav.
//!   - `button_task` — `Input` on `pins.button` (active-low, internal pull-up); classifies each press as
//!     `Press::Short` (cycle screens) or `Press::Long` (the current screen's action) and signals it; the OLED
//!     task interprets the gesture per the screen it is showing.
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

// Self-heal threshold: after this many consecutive cycles with no valid Telemetry reply, hardware-re-init the radio
// (RESET pulse + cold start) instead of looping forever. This is the actual cure for the PA-death failure where the
// goggle's TX-done IRQ keeps firing but the power stage has latched off — a reset re-arms it without a reboot. At the
// 40 ms cadence, 8 misses is ~320 ms of silence, well below any human-noticeable head-tracking stall. TODO: tune.
const REINIT_AFTER_MISSES: u32 = 8;

// Back-off threshold once Disconnected: after the first re-init fails to bring the truck back, we can't tell "my PA
// is dead" from "the truck is simply off/out of range", so retry slowly (75 * 40 ms = ~3 s) instead of RESET-pulsing
// every ~320 ms forever. Drops to the fast REINIT_AFTER_MISSES again the moment a real reply re-marks us Connected.
const REINIT_BACKOFF_MISSES: u32 = 75;

// LoRa binding: phrase -> UID -> link-id + CRC initializer, derived at compile time from BINDING_PHRASE (defaulted in
// build.rs). Both nodes must build with the same phrase; every TX frame is tagged + CRC'd so the truck drops frames
// from a differently bound pair, and we drop theirs — ExpressLRS-style anti-collision on a shared frequency.
const BINDING: link_id::Binding = link_id::derive(env!("BINDING_PHRASE"));

// OLED refresh cadence. The panel only shows a human-readable glance, so a few Hz is plenty.
const OLED_PERIOD: Duration = Duration::from_millis(250);

// Button hold time that separates a short press (LAP) from a long press (RESET). 800 ms is comfortably past a
// deliberate tap but short enough to feel responsive when intentionally resetting. TODO: tune on hardware.
const LONG_PRESS: Duration = Duration::from_millis(800);

// Link liveness window: if no Telemetry reply has been decoded within this span, the OLED shows LINK:N. A couple
// of TX cycles' worth of slack so a single dropped reply does not flap the flag. TODO: tune on hardware.
const LINK_TIMEOUT: Duration = Duration::from_millis(500);

// How many consecutive Control transmits carry the re-home flag after a Nav-screen long press. The link is lossy
// half-duplex, so a single flagged Control can be dropped; bursting it over ~10 * 40 ms = 0.4 s makes the truck
// hear it reliably without latching the flag on permanently (which would re-home every fix).
const REHOME_BURST: u8 = 10;

// Latest pan/tilt command, published by the PPM reader and consumed by the LoRa task. CriticalSectionRawMutex
// because the Signal is a `static` shared across tasks.
static CONTROL: Signal<CriticalSectionRawMutex, Control> = Signal::new();
// Latest truck telemetry decoded by the LoRa task, consumed by the OLED task to render the GPS sat count (the
// goggle has no local GPS — "SV nn" on G1 is the truck's fix relayed over the half-duplex telemetry).
static SPEED: Signal<CriticalSectionRawMutex, Telemetry> = Signal::new();
// Latest head pan/tilt (us) published by the PPM reader ALONGSIDE CONTROL, consumed by the OLED to show the live
// degree readouts. A Signal hands each value to ONE taker, so the OLED needs its own copy separate from CONTROL
// (which the LoRa task consumes) — the same pattern the truck uses for its DISPLAY_CONTROL.
static HEAD: Signal<CriticalSectionRawMutex, (u32, u32)> = Signal::new();
// RSSI (dBm) of the most recent binding-valid Telemetry reply, published by the LoRa task, consumed by the OLED to
// derive the header signal bars. Published only on a decoded reply so noise/foreign frames never move the bars.
static LINK_RSSI: Signal<CriticalSectionRawMutex, i16> = Signal::new();
// Raw button press from the button task: a short or a long press. The MEANING is decided by the OLED task per the
// current screen (short = cycle screens; long = reset the lap on Stopwatch, or request a re-home on Nav), so the
// button task stays screen-agnostic and only classifies the gesture.
static BUTTON: Signal<CriticalSectionRawMutex, Press> = Signal::new();
// Re-home request from the OLED task (a long press on the Nav screen) to the LoRa task, which then sets the Control
// re-home flag on its next few transmits so the truck re-captures its home point. Unit payload — only the edge matters.
static SET_HOME_REQ: Signal<CriticalSectionRawMutex, ()> = Signal::new();
// Latest LoRa link state (Connected/Tentative/Disconnected) published by the LoRa task on each change. Surfaced for
// the status display; the OLED derives liveness from the reply timeout instead, but this stays published for logs.
static LINK_STATE: Signal<CriticalSectionRawMutex, lora_link::LinkState> = Signal::new();

/// A classified button gesture. `Copy`/`Send`, so it rides a `Signal` to the OLED task, which interprets it per
/// the screen currently shown.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Press {
  Short, // quick tap: cycle to the next screen
  Long,  // held past LONG_PRESS: the current screen's action (reset lap / re-home)
}

/// Which goggle screen is currently shown. The short press cycles between them; each owns its own render path.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Screen {
  Stopwatch, // G1: lap clock + pan/tilt (render_goggle)
  Nav,       // "find my truck": distance + bearing to the truck's home (render_nav)
}

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
      // flags is 0 here — the re-home command bit is injected by the LoRa task, not the PPM path.
      CONTROL.signal(Control { pan_us, tilt_us, flags: 0 });
      // Publish the OLED's own copy of the head pose (independent Signal, lossy — latest frame only).
      HEAD.signal((pan_us, tilt_us));
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
  let mut latest = Control { pan_us: CENTER_US, tilt_us: CENTER_US, flags: 0 };
  let mut rx_buf = [0u8; lora_link::MAX_PAYLOAD as usize];
  // Countdown of remaining re-home-flagged transmits; armed to REHOME_BURST by a Nav-screen long press.
  let mut rehome_pending: u8 = 0;
  // Owns the self-heal: counts reply-misses, re-inits the radio at the threshold, then backs off while Disconnected.
  let mut health = lora_link::LinkHealth::new(REINIT_AFTER_MISSES, REINIT_BACKOFF_MISSES);
  // Last published link state, so we only signal LINK_STATE on a change rather than every 40 ms tick.
  let mut last_state = health.state();
  LINK_STATE.signal(last_state);

  loop {
    ticker.next().await;

    // Latest-value hand-off: if a fresher Control is waiting, take it; otherwise resend the last one so a
    // momentary PPM gap still keeps the link alive at a steady rate.
    if let Some(control) = CONTROL.try_take() {
      latest = control;
    }

    // A Nav-screen long press arms a short burst of re-home-flagged transmits, so the truck re-captures its home
    // even if some flagged frames are lost over the air. The burst counts down by SUCCESSFUL transmits (in the TX
    // Ok branch below), not by loop ticks, so a run of TX errors/timeouts can't silently drain it without the
    // truck ever hearing a flagged frame.
    if SET_HOME_REQ.try_take().is_some() {
      rehome_pending = REHOME_BURST;
    }
    let mut out = latest; // out.flags is 0 — both the PPM path and the initial value set flags: 0.
    if rehome_pending > 0 {
      out.flags = 1; // request re-home (bit0); counted against the burst only once this frame actually goes out.
    }

    // Encode the Control into a heapless Vec<u8, 24> (pan/tilt/flags worst-case to 18 B, plus headroom), then wrap
    // it with the binding frame ([link_id][proto][crc16], +4 bytes) so the truck only accepts frames from our pair.
    // micropb's container-heapless-0-9 implements PbWrite for exactly this heapless 0.9 Vec.
    let mut enc = PbEncoder::new(heapless::Vec::<u8, 24>::new());
    if out.encode(&mut enc).is_err() {
      applog::log_println!("control encode failed");
      continue;
    }
    let payload = enc.into_writer();
    let mut framed = heapless::Vec::<u8, 28>::new();
    if link_id::frame(&BINDING, &payload, &mut framed).is_err() {
      applog::log_println!("control frame overflow");
      continue;
    }

    // One TX + reply round-trip. `got_reply` is set only when a valid Telemetry frame from our pair decodes; any
    // other outcome (TX error/timeout, RX error/timeout, foreign frame) counts as a miss toward the self-heal.
    let mut got_reply = false;
    // Bounded TX: a never-completing send (missed DIO0 TX-done, likely GPIOTE contention with the PPM edge waits)
    // would otherwise hang the transmit loop forever. On error/timeout we fall through to the miss accounting below.
    match select(link.send(&framed, TX_POWER_DBM), Timer::after(TX_TIMEOUT)).await {
      Either::First(Ok(())) => {
        // The Control (flagged or not) actually left the radio, so count a re-home frame against the burst HERE —
        // never on a TX error/timeout — so the burst survives a link blip and is only spent by frames on the air.
        rehome_pending = rehome_pending.saturating_sub(1);
        // Listen for the truck's Telemetry reply, bounded by REPLY_TIMEOUT so a lost reply cannot stall the loop.
        match select(link.receive_with_status(&mut rx_buf), Timer::after(REPLY_TIMEOUT)).await {
          Either::First(Ok((len, status))) => {
            if let Some(bytes) = link_id::deframe(&BINDING, &rx_buf[..len]) {
              let mut telem = Telemetry::default();
              if telem.decode_from_bytes(bytes).is_ok() {
                // Log km/h for the serial console (the freshest telemetry is also handed to the OLED). Use the
                // display crate's shared converter so the goggle log and the truck OLED can't drift apart — it does
                // the same u64-widened integer rounding (the speed_cm_s overflow fix lives in one place now).
                applog::log_println!(
                  "telemetry: {} km/h | sats {} | fix {}",
                  display::cm_s_to_kmh(telem.speed_cm_s),
                  telem.sats,
                  telem.fix_quality
                );
                SPEED.signal(telem);
                // Publish the reply's RSSI for the OLED signal bars — only here, inside the decoded-reply branch, so
                // foreign/corrupt frames never move the bars. Does not affect the self-heal accounting below.
                LINK_RSSI.signal(status.rssi);
                got_reply = true;
              } else {
                applog::log_println!("telemetry decode failed ({} bytes)", bytes.len());
              }
            }
            // A frame that fails deframe is another pair's traffic (or noise) — not our reply, so leave got_reply false.
          }
          Either::First(Err(e)) => applog::log_println!("LoRa RX error: {:?}", e),
          Either::Second(()) => { /* no reply this cycle — expected occasionally. */ }
        }
      }
      Either::First(Err(e)) => applog::log_println!("LoRa TX error: {:?}", e),
      Either::Second(()) => applog::log_println!("LoRa TX timed out (no TX-done) — recovering"),
    }

    // Self-heal: feed this cycle's result to LinkHealth, which clears the streak on a good reply or, after enough
    // misses, re-inits the radio to clear a latched PA fault — the fix for "link dies, only a reboot recovers".
    match health.service(Some(got_reply), &mut link).await {
      lora_link::Serviced::Reinited(Ok(())) => applog::log_println!("radio re-init OK"),
      lora_link::Serviced::Reinited(Err(e)) => applog::log_println!("radio re-init failed: {:?}", e),
      lora_link::Serviced::Idle => {}
    }
    // Publish link state for the status display only when it changes (the OLED does not consume it yet).
    let state = health.state();
    if state != last_state {
      last_state = state;
      LINK_STATE.signal(state);
    }
  }
}

/// OLED status task. Owns the screen state and cycles two layouts with the button: the G1 "Stopwatch" (a big lap
/// clock SS.s, live pan/tilt degrees, the relayed GPS sat count, LoRa bars) and the "Nav" find-my-truck screen
/// (distance + bearing to the truck's home). A SHORT press (`Press::Short`) cycles screens; a LONG press
/// (`Press::Long`) is the current screen's action — re-base the Stopwatch clock, or request a re-home on Nav.
/// "Link up" is a real liveness check: a Telemetry reply decoded within `LINK_TIMEOUT`. Manual lap-advance was
/// dropped when the button took over screen cycling, so the lap number is a fixed "01" placeholder for now.
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
  // Last reply RSSI, floored so the bars read empty until a real reply lands (rssi_to_bars also zeroes when down).
  let mut last_rssi: i16 = -127;
  // Latest head pose (us), seeded to center so the degree readouts show 0/0 before the first PPM frame.
  let mut pan_us: u32 = CENTER_US;
  let mut tilt_us: u32 = CENTER_US;
  // Stopwatch state owned here: the button task only signals raw presses. With manual lap-advance dropped (the
  // button now cycles screens), only the running clock is live — a long press re-bases its origin to now. The lap
  // NUMBER has no advance trigger, so it is a fixed "01" placeholder passed straight into the render below.
  let mut lap_origin = Instant::now();
  // Which screen the button is currently showing; the short press toggles it.
  let mut screen = Screen::Stopwatch;

  loop {
    ticker.next().await;
    if let Some(telem) = SPEED.try_take() {
      last = telem;
      last_rx = Instant::now();
    }
    if let Some((pan, tilt)) = HEAD.try_take() {
      pan_us = pan;
      tilt_us = tilt;
    }
    if let Some(rssi) = LINK_RSSI.try_take() {
      last_rssi = rssi;
    }
    // Short press cycles screens; long press is the current screen's action — re-base the stopwatch on Stopwatch,
    // request a re-home on Nav (the truck owns the home point, so we ask it over the link rather than acting locally).
    match BUTTON.try_take() {
      Some(Press::Short) => {
        screen = match screen {
          Screen::Stopwatch => Screen::Nav,
          Screen::Nav => Screen::Stopwatch,
        };
      }
      Some(Press::Long) => match screen {
        Screen::Stopwatch => lap_origin = Instant::now(),
        Screen::Nav => {
          SET_HOME_REQ.signal(());
          applog::log_println!("re-home requested");
        }
      },
      None => {}
    }

    // Real liveness, not a one-way latch: the link is up only while a reply has been seen within LINK_TIMEOUT, so
    // the bars fall to empty when the truck goes away instead of holding their last value forever.
    let linked = Instant::now().duration_since(last_rx) < LINK_TIMEOUT;
    let bars = display::rssi_to_bars(last_rssi, linked);

    // Render the active screen. The Nav screen intentionally keeps showing the LAST known distance/bearing when the
    // link drops (bars fall to empty) — for "find my truck", the final fix is exactly what you want.
    let result = match screen {
      Screen::Stopwatch => {
        // Lap time in tenths since the origin. Instant is monotonic and the origin is always <= now, but use the
        // saturating form to stay underflow-proof regardless.
        let lap_tenths = (Instant::now().saturating_duration_since(lap_origin).as_millis() / 100) as u32;
        oled
          .render_goggle(display::GoggleStatus {
            lap: 1, // fixed placeholder — no lap-advance trigger remains (see the task doc).
            lap_tenths,
            pan_deg: display::pan_us_to_deg(pan_us as i32),
            tilt_deg: display::tilt_us_to_deg(tilt_us as i32),
            sats: last.sats,
            bars,
          })
          .await
      }
      Screen::Nav => {
        // "At home" when the truck sits within GPS noise of home — there the bearing would spin, so the screen
        // shows "AT HOME" instead. render_nav already gates the whole readout on nav_valid, so this need not
        // re-check it. AT_HOME_M is shared with the truck's nav math. bearing_deg is folded mod 360 so an
        // out-of-range value from a corrupt decode lands in a real sector instead of truncating via `as u16`.
        let at_home = last.dist_m < geo::AT_HOME_M;
        oled
          .render_nav(display::NavStatus {
            dist_m: last.dist_m,
            bearing_deg: (last.bearing_deg % 360) as u16,
            nav_valid: last.nav_valid,
            at_home,
            sats: last.sats,
            bars,
          })
          .await
      }
    };
    if let Err(e) = result {
      applog::log_println!("OLED render error: {:?}", e);
    }
  }
}

/// Button task. The button is active-low with an internal pull-up, so a press is a falling edge. It only
/// classifies the gesture — a short press signals `Press::Short`, a long press (held past `LONG_PRESS`) signals
/// `Press::Long` — and the OLED task decides what each means for the current screen.
#[embassy_executor::task]
async fn button_task(mut button: Input<'static>) {
  loop {
    // Wait for the press edge (active-low -> falling), then sleep the debounce window and RE-CHECK the line is still
    // low. A real press holds the line low past the window; a bounce/glitch has released by then and is ignored.
    button.wait_for_falling_edge().await;
    Timer::after(Duration::from_millis(50)).await;
    if button.is_high() {
      continue; // glitch, not a real press — re-arm.
    }
    // Confirmed press held low. Race the release (rising edge) against the long-press timer: released first =>
    // short press; the timer wins => still held at the threshold => long press.
    let press = match select(button.wait_for_rising_edge(), Timer::after(LONG_PRESS)).await {
      Either::First(()) => Press::Short,
      Either::Second(()) => Press::Long,
    };
    BUTTON.signal(press);
    // For a long press the release has NOT happened yet (the timer won the race), so drain it before re-arming so
    // one physical press is exactly one event. A short press already consumed its release edge in the race above.
    if press == Press::Long {
      button.wait_for_rising_edge().await;
    }
  }
}
