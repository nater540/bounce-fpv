//! Truck node (Phase D: nRF52840, headless): receive pan/tilt over LoRa, drive two gimbal servos at a fixed
//! rate, read a GPS for ground speed, and reply with Telemetry. The matching slave half of the goggle's
//! half-duplex link. No OLED — the status panel moved to the goggle, so the only I2C device here (the MPU-6050)
//! uses a direct `Twim`, not a shared bus.
//!
//! Tasks and latest-value `Signal` hand-offs (all lossy — control loops act on the freshest sample, never a
//! queue):
//!   - startup one-shot (in `main`, behind the default-on `imu` feature) — builds the MPU-6050 `Twim` and runs
//!     `imu::detect_home_default` to establish the gimbal center trim BEFORE the servo loop starts, passing it as
//!     pan/tilt trims into `servo_task`. With the `imu` feature off the read is skipped and the trims are zero.
//!   - `lora_task`  — RX a `Control`, decode; on success `CONTROL.signal(..)`, on decode failure `continue`
//!     (never replies to garbage); then read the latest GPS `Telemetry` and TX it as the turnaround reply.
//!   - `servo_task` — builds the two 50 Hz `SimplePwm` outputs (PWM0 pan, PWM1 tilt), wraps each in
//!     `servo::Servo`, and at a fixed ~50 Hz reads the latest `Control`, applies the boot IMU home trim, clamps
//!     into the servo band, and drives the PWM duty. Never touches the radio, so RX jitter never reaches servos.
//!   - `gps_task`   — `BufferedUarteRx` -> `gps::GpsReader`; latches a home point at the first valid fix, computes
//!     distance + bearing to it via `geo::nav`, and publishes each fix as a `Telemetry` (speed + nav) for the LoRa
//!     reply. A `SET_HOME` signal (raised when a received Control carries the re-home flag) re-latches home.
//!
//! Half-duplex turnaround (scaffold scheme, tunable later): the goggle is master, the truck is slave. The truck
//! parks in continuous RX; on each received Control it applies it and immediately TXes the latest Telemetry,
//! then returns to RX. One radio per node, no collisions. See `goggle-node` for the master half.

#![no_std]
#![no_main]

// Pull in the shared #[panic_handler] (defined once in applog). applog ALSO provides defmt-rtt's #[global_logger]
// (it does `use defmt_rtt as _;`), which is what lets lora-phy's unconditional defmt dependency link here without
// any per-binary defmt setup.
use applog as _;

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_nrf::buffered_uarte::{self, BufferedUarteRx};
use embassy_nrf::pwm::SimplePwm;
use embassy_nrf::spim;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::uarte::{self, Baudrate};
use embassy_nrf::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker, Timer};
use gps::GpsReader;
use micropb::{MessageDecode, MessageEncode, PbEncoder};
use nrf_adapters::PwmServoChannel;
use proto::{Control, Telemetry};
use servo::Servo;
use static_cell::StaticCell;

// Bind the SERIAL/UART interrupts this node uses: SPIM3 (LoRa SPI bus), TWISPI0 (MPU-6050 I2C), and UARTE1 (GPS,
// via the BUFFERED handler — `BufferedUarteRx` services its ring buffer from this ISR). USBD is bound by applog —
// do NOT bind it here. GPIOTE (DIO0 edge waits) is bound by embassy-nrf's init at the SD-safe P2 priority.
bind_interrupts!(struct Irqs {
  SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
  TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
  UARTE1 => buffered_uarte::InterruptHandler<peripherals::UARTE1>;
});

// Servo refresh rate. The MG90S frame is 50 Hz; refreshing duty at the same rate keeps the loop simple and well
// within the 50-100 Hz band the doc calls for. TODO: tune on hardware if the gimbal needs faster updates.
const SERVO_PERIOD: Duration = Duration::from_millis(20);

// Pulse-width change per degree of IMU home tilt, used to trim the gimbal center for a tilted mount. Derived
// from the MG90S ~400-2400 us / ~180 deg span ((2400-400)/180 ~= 11 us/deg). TODO: calibrate on hardware.
#[cfg(feature = "imu")]
const US_PER_DEGREE: i32 = 11;

// Loss-of-signal failsafe window: if no fresh Control arrives within this span the servo loop recenters the
// gimbal (with the home trim still applied, so it points "forward") rather than holding the last pose forever.
// TODO: tune on hardware.
const CONTROL_TIMEOUT: Duration = Duration::from_millis(1000);

// Plausible mount-tilt bound for the boot IMU home angle. A present-but-garbage MPU-6050 that ACKs returns bogus
// accel, which would otherwise bake a large permanent center bias into the trim; an angle beyond this is treated
// as zero trim (calibration suspect). TODO: tune on hardware.
#[cfg(feature = "imu")]
const MAX_HOME_DEG: f32 = 30.0;

// GPS UART baud. Most modules default to 9600. TODO: confirm against the specific receiver.
const GPS_BAUD: Baudrate = Baudrate::BAUD9600;

// Minimum satellites before the home point is latched (and before distance/bearing are reported valid). A 1-2
// satellite "fix" is positionally garbage and would anchor home tens of meters off; 4 is the floor for a usable
// 3D fix. The fix-quality field must also be non-zero. TODO: tune against the receiver's reported sat counts.
const MIN_SATS_FOR_HOME: u32 = 4;

// TX power in dBm for the Telemetry reply (RFM95W PA_BOOST). Dropped from 17 to 10 dBm to match the goggle: sustained
// 17 dBm PA_BOOST transmitting trips the PA's over-current/thermal protection after ~a minute (TX-done fires but no RF
// radiates). The bench link has ~30 dB of margin, so 10 dBm is ample. TODO: raise only with antenna + heatsink + range.
const TX_POWER_DBM: i32 = 10;

// PA output routing. The bare RFM95W bonds ONLY its PA_BOOST pin to the antenna, so tx_boost MUST be true or the
// radiated power is near-zero and the link silently fails to come up (see lora-ping for the full rationale).
const TX_BOOST: bool = true;

// Bound on the otherwise-unbounded continuous RX. Without it the task parks on the DIO0 IRQ forever when the goggle
// goes silent and never re-arms. Must comfortably exceed the goggle's 40 ms TX_PERIOD so a normal inter-packet gap
// never trips it; 200 ms ~= 5 missed controls. On a timeout we count a miss toward the radio self-heal. TODO: tune.
const RX_TIMEOUT: Duration = Duration::from_millis(200);

// Upper bound on the telemetry turnaround transmit. `send()` blocks on the DIO0 TX-done IRQ; if that IRQ is missed
// (the same PA-latch / GPIOTE-contention class the goggle guards against) the unbounded await would park this whole
// task forever — so a stuck truck-side TX could never reach the self-heal below. 30 ms is ~3x the ~10 ms SF7/BW500
// airtime, so a healthy reply never trips it; a timeout instead feeds the radio re-init as a likely TX-leg fault.
const TX_TIMEOUT: Duration = Duration::from_millis(30);

// Self-heal threshold: after this many consecutive radio-fault windows — RX silence, an RX error, or our own reply
// TX stalling — hardware-re-init the radio to clear a stuck RX/TX stage. Foreign frames from another pair do NOT
// count (the RX path clearly works), so a busy neighbor can't drive spurious re-inits. By wall-clock the truck
// re-inits at ~5*200 ms = 1 s while the goggle (master) leads at ~8*40 ms = 320 ms, so the two ends don't reset in
// lockstep and miss each other's first post-reset packets.
const REINIT_AFTER_MISSES: u32 = 5;

// Back-off threshold once Disconnected: after the first re-init fails to recover the link, a missing goggle and a
// dead local radio look identical, so retry slowly (25 * 200 ms = ~5 s) instead of RESET-pulsing every ~1 s forever.
// Drops back to the fast REINIT_AFTER_MISSES the moment a valid Control re-marks us Connected.
const REINIT_BACKOFF_MISSES: u32 = 25;

// LoRa binding: phrase -> UID -> link-id + CRC initializer, derived at compile time from BINDING_PHRASE (defaulted in
// build.rs). MUST match the goggle's phrase. Every received frame is checked against it (link-id + CRC) so traffic
// from a differently bound pair is dropped, and our replies carry the same tag — ExpressLRS-style anti-collision.
const BINDING: link_id::Binding = link_id::derive(env!("BINDING_PHRASE"));

// Status OLED refresh cadence and link-liveness window. The panel only needs a human glance, so a few Hz; the link
// reads down if no Control has been received within LINK_TIMEOUT (a couple of TX cycles of slack against one drop).
const OLED_PERIOD: Duration = Duration::from_millis(250);
const LINK_TIMEOUT: Duration = Duration::from_millis(500);

// Latest pan/tilt command from the radio, consumed by the servo loop (lossy: only the freshest pose matters).
// CriticalSectionRawMutex because the Signal is a `static` shared across tasks.
static CONTROL: Signal<CriticalSectionRawMutex, Control> = Signal::new();
// Latest GPS-derived telemetry from the UART, consumed by the LoRa reply. The reader re-takes it each turnaround.
static GPS_TELEM: Signal<CriticalSectionRawMutex, Telemetry> = Signal::new();
// Latest GPS telemetry for the status OLED, published by gps_task ALONGSIDE GPS_TELEM (which the LoRa reply
// consumes) — the OLED needs its own copy to render ground speed. Lossy: latest fix only.
static SPEED_DISPLAY: Signal<CriticalSectionRawMutex, Telemetry> = Signal::new();
// RSSI (dBm) of the most recent binding-valid Control frame, published by the LoRa task, consumed by the OLED for
// the header signal bars + numeric readout. Published only on a decoded Control so foreign frames never move it.
static LINK_RSSI: Signal<CriticalSectionRawMutex, i16> = Signal::new();
// Latest LoRa link state (Connected/Tentative/Disconnected) published by the LoRa task on each change. Surfaced for
// the status display; the OLED derives liveness from the Control-RX timeout instead, but this stays published for logs.
static LINK_STATE: Signal<CriticalSectionRawMutex, lora_link::LinkState> = Signal::new();
// Manual re-home request: the LoRa task signals this when a received Control carries the re-home flag (bit0), and
// gps_task drops its captured home so the next valid fix re-latches it. Unit payload — only the edge matters.
static SET_HOME: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// The radio type lives in `nrf_adapters::lora` now (shared by both nodes + lora-ping so it can never drift); the
// spawned LoRa task takes `nrf_adapters::lora::Link` directly for its non-generic signature. The status OLED rides a
// direct (non-shared) Twim — it takes the bus after the one-shot boot IMU read, so no shared_bus Mutex is needed.
type OledI2c = Twim<'static>;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
  // embassy-nrf init at SoftDevice-safe interrupt priorities (GPIOTE + time-driver at P2; the SD reserves P0/P1/P4).
  let p = applog::init_embassy_nrf();
  // board_pins! partial-moves only the GPIO pin fields out of `p`, leaving the controller singletons (SPI3,
  // TWISPI0, UARTE1, TIMER1, the PPI channels/group, PWM0/PWM1, USBD, ...) on `p` for the rest of main.
  let pins = board::board_pins!(p);

  // SD COEXISTENCE: init_embassy_nrf centrally lowers all SERIAL/SPIM/TWIM/UARTE IRQs to P2 (the SD-safe band with
  // GPIOTE + the time driver), so the per-binary set_priority calls are gone — the peripheral constructors below
  // inherit P2. SimplePwm (PWM0/PWM1) is fire-and-forget (register/DMA writes, no awaited IRQ), so it raises no
  // interrupt anyway. CONFIRM ON-TARGET that SPIM3 + TWISPI0 + UARTE1 + GPIOTE coexist with the SD enabled.

  applog::init(
    spawner,
    p.USBD,
    applog::UsbIdentity::new(0x1209, 0x0001, "fabulous-fpv", "truck", "d"),
  );

  applog::log_println!("");
  applog::log_println!("=== truck-node: LoRa RX -> servos + IMU + GPS (nRF52840, headless) ===");

  // Shared I2C bus (TWISPI0). The status OLED ALWAYS needs it; the MPU-6050 (optional `imu` feature) is the only
  // other device, so a direct Twim suffices — no shared_bus Mutex. The TWIM tx_ram_buffer must be 'static (it
  // outlives the 'static peripheral); a StaticCell gives it that, and 16 bytes is ample for the small command
  // bytes either driver sends. Enable the internal SDA/SCL pull-ups (Config::default enables NEITHER) so the bus
  // ACKs on bare wiring — the same fix that brought I2C up in lora-ping (both flags set; embassy-nrf gates both
  // lines off sda_pullup). The OLED takes ownership at the bottom of main; with `imu` OFF nothing mutates the bus
  // in main before then, hence the conditional allow(unused_mut).
  static TWIM_TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
  let tx_buf = TWIM_TX_BUF.init([0; 16]);
  let mut i2c_cfg = twim::Config::default();
  i2c_cfg.sda_pullup = true;
  i2c_cfg.scl_pullup = true;
  #[cfg_attr(not(feature = "imu"), allow(unused_mut))]
  let mut i2c = Twim::new(p.TWISPI0, Irqs, pins.i2c_sda, pins.i2c_scl, i2c_cfg, tx_buf);

  // Boot IMU home -> gimbal center trim (pan from roll, tilt from pitch), passed into servo_task. Behind the
  // default-on `imu` feature: with it ON the MPU-6050 is read once BEFORE the servo loop starts so a tilted mount is
  // calibrated out; the truck must be held still during the ~2 s settle+average. With it OFF the read is skipped
  // entirely — no still-at-power-on requirement — and the trims are zero, so the gimbal centers on the pure servo
  // neutral. Either branch yields the same (pan_trim, tilt_trim) so the spawn below is feature-agnostic.
  #[cfg(feature = "imu")]
  let (pan_trim, tilt_trim) = {
    let mut delay = embassy_time::Delay;
    let home = match imu::detect_home_default(&mut i2c, &mut delay).await {
      Ok(home) => {
        applog::log_println!("IMU home: roll {} deg, pitch {} deg", home.roll_deg, home.pitch_deg);
        home
      }
      // A missing/miswired IMU must not brick the truck; log and fall back to a zero trim (fixed center).
      Err(e) => {
        applog::log_println!("IMU home detection failed ({:?}) — using fixed center", e);
        imu::Home::default()
      }
    };
    (bounded_home_trim(home.roll_deg, "roll"), bounded_home_trim(home.pitch_deg, "pitch"))
  };
  #[cfg(not(feature = "imu"))]
  let (pan_trim, tilt_trim): (i32, i32) = {
    applog::log_println!("IMU disabled (build without the `imu` feature) — fixed gimbal center, no boot home read");
    (0, 0)
  };

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

  // GPS UART on UARTE1, RX-only via the interrupt-driven BufferedUarteRx (it ring-buffers RX bytes between reads
  // and honors short reads — exactly what gps::GpsReader, an embedded_io_async::Read consumer, wants). It needs a
  // TIMER + two PPI channels + a PPI group for its EasyDMA hand-off; TIMER1 (NOT TIMER0, the SD reserves it) and
  // PPI_CH0/PPI_CH1/PPI_GROUP0, all SD-safe. The RX ring must be a 'static mut [u8] of EVEN length; a StaticCell
  // gives it a 'static home. The module's TX pin is intentionally unused — the reader never transmits.
  let mut uarte_config = uarte::Config::default();
  uarte_config.baudrate = GPS_BAUD;
  // 512-byte (EVEN) RX ring: ~0.5 s of bytes at 9600 baud of slack against scheduler stalls. An overrun or a
  // ring-buffer-full is NOT recoverable on embassy-nrf 0.10 — both PANIC in the ISR — so the only mitigation is
  // a large ring plus gps_task draining promptly; this sizes the ring well past a burst of ~82-byte NMEA
  // sentences. TODO: confirm size on hardware.
  static RX_RING: StaticCell<[u8; 512]> = StaticCell::new();
  let rx_ring = RX_RING.init([0u8; 512]);
  let gps_rx = BufferedUarteRx::new(
    p.UARTE1, p.TIMER1, p.PPI_CH0, p.PPI_CH1, p.PPI_GROUP0, Irqs, pins.gps_rx, uarte_config, rx_ring,
  );

  // Build the two 50 Hz servo PWM outputs here so the SimplePwm borrows move cleanly into the servo task. One PWM
  // peripheral per servo (PWM0 pan, PWM1 tilt) because PwmServoChannel owns the whole SimplePwm and targets its
  // channel 0; servo_config fixes the Div16 prescaler + 20_000-tick countertop so 1 tick = 1 us.
  let servo_cfg = PwmServoChannel::servo_config();
  let pan_pwm = SimplePwm::new_1ch(p.PWM0, pins.servo_pan, &servo_cfg);
  let tilt_pwm = SimplePwm::new_1ch(p.PWM1, pins.servo_tilt, &servo_cfg);

  // The task macro returns a Result<SpawnToken, SpawnError> (the pool-full case); unwrap the token then spawn.
  spawner.spawn(lora_task(link).expect("lora_task token"));
  spawner.spawn(servo_task(pan_pwm, tilt_pwm, pan_trim, tilt_trim).expect("servo_task token"));
  spawner.spawn(gps_task(GpsReader::new(gps_rx)).expect("gps_task token"));
  // The Twim is free now — with `imu` on, the one-shot boot read above already released its &mut borrow; with it off
  // nothing touched the bus. Hand it to the status OLED task, which shows the received pan/tilt (no shared_bus
  // needed; the IMU, when present, is not read at runtime).
  spawner.spawn(oled_task(i2c).expect("oled_task token"));
}

/// LoRa task (link slave). Parks in RX; on each received `Control` it decodes and publishes the latest pose for
/// the servo loop, then immediately TXes the freshest `Telemetry` reply (the half-duplex turnaround) before
/// returning to RX. Servos are driven elsewhere — this task only hands off the pose via the Signal.
#[embassy_executor::task]
async fn lora_task(mut link: nrf_adapters::lora::Link) {
  let mut rx_buf = [0u8; lora_link::MAX_PAYLOAD as usize];
  // Most recent telemetry to reply with; updated from the GPS Signal each loop, defaults to "no fix".
  let mut telem = Telemetry::default();
  // Owns the self-heal: counts radio-fault windows, re-inits at the threshold, then backs off while Disconnected.
  let mut health = lora_link::LinkHealth::new(REINIT_AFTER_MISSES, REINIT_BACKOFF_MISSES);
  // Running count of self-heal radio re-inits, logged with uptime so an overnight soak shows how often the link
  // drops and that each one recovered (normal telemetry logging resuming after is the proof of recovery).
  let mut reinit_count: u32 = 0;
  // Last published link state, so we only signal LINK_STATE on a change rather than every RX window.
  let mut last_state = health.state();
  LINK_STATE.signal(last_state);

  loop {
    // Each window classifies the radio's health for the self-heal at the bottom:
    //   Some(true)  — a faultless round-trip (heard our goggle, and any reply went out): clear the streak.
    //   Some(false) — a fault a radio reset could fix (RX silence, RX error, or our reply TX stalling): count it.
    //   None        — a foreign/corrupt frame: the RX path demonstrably works, it just wasn't for us, so leave the
    //                  streak untouched. Counting these as misses let a nearby second pair's traffic trigger spurious
    //                  re-inits (each foreign frame returns from receive() immediately, racing to the threshold).
    let outcome: Option<bool>;

    // Bounded RX: without the timeout this awaits the DIO0 IRQ forever, so a silent goggle would park us until a
    // packet happens to land.
    match select(link.receive_with_status(&mut rx_buf), Timer::after(RX_TIMEOUT)).await {
      Either::First(Ok((len, status))) => {
        if let Some(bytes) = link_id::deframe(&BINDING, &rx_buf[..len]) {
          // A frame that passes the binding (link-id + CRC) is ours. Decode the Control, publish it, then turn around
          // the Telemetry reply. The round-trip is healthy unless our own reply transmit stalls (handled below).
          let mut control = Control::default();
          if control.decode_from_bytes(bytes).is_ok() {
            // Publish to the servo loop (CONTROL) and the status OLED (LINK_RSSI — the header bars/number, which
            // also serves as the OLED's link-liveness heartbeat). Published only here, in the decoded-Control
            // branch, so foreign frames never move the bars or falsely mark the link live.
            LINK_RSSI.signal(status.rssi);
            // A re-home command (flags bit0) asks gps_task to drop its home so the next valid fix re-latches it.
            // Checked before publishing so the request rides the same Control the operator pressed it on.
            if control.flags & 1 != 0 {
              SET_HOME.signal(());
            }
            CONTROL.signal(control);

            // Refresh the reply payload from the latest GPS fix (if any), then frame + transmit the turnaround.
            if let Some(latest) = GPS_TELEM.try_take() {
              telem = latest;
            }
            // Telemetry now carries dist_m/bearing_deg/nav_valid on top of speed/sats/fix. Worst case is exactly
            // 32 B (five uint32 at 6 B each + a 2 B bool); 40/44 give headroom over that exact fit, both well
            // under MAX_PAYLOAD (64). Encode failure is still handled gracefully below, not via a panic.
            let mut enc = PbEncoder::new(heapless::Vec::<u8, 40>::new());
            if telem.encode(&mut enc).is_err() {
              // An encode/frame failure is a data bug, not a radio fault — the RX leg is proven alive, so don't re-init.
              applog::log_println!("telemetry encode failed");
              outcome = Some(true);
            } else {
              // Framing prepends link-id + appends CRC, so even an all-zero (no-GPS) Telemetry that proto3 encodes to
              // ZERO bytes still produces a non-empty 4-byte LoRa payload — which incidentally retires the old
              // 0-length-payload workaround (a 0-byte SX1276 transmit never completed and was the LINK:N symptom).
              let payload = enc.into_writer();
              let mut framed = heapless::Vec::<u8, 44>::new();
              if link_id::frame(&BINDING, &payload, &mut framed).is_err() {
                applog::log_println!("telemetry frame overflow");
                outcome = Some(true);
              } else {
                // Bound the turnaround TX so a missed TX-done IRQ can't park the task forever (the original
                // "dies until reboot" failure, on the truck's TX leg). A timeout/error means our TX stage may be
                // dead, so mark it a radio fault → it feeds the self-heal; a clean send is a healthy round-trip.
                match select(link.send(&framed, TX_POWER_DBM), Timer::after(TX_TIMEOUT)).await {
                  Either::First(Ok(())) => outcome = Some(true),
                  Either::First(Err(e)) => {
                    applog::log_println!("LoRa TX (telemetry) error: {:?}", e);
                    outcome = Some(false);
                  }
                  Either::Second(()) => {
                    applog::log_println!("telemetry TX timed out (no TX-done) — recovering");
                    outcome = Some(false);
                  }
                }
              }
            }
          } else {
            // Binding-valid but garbage Control: RX leg alive, so skip the reply but don't re-init the radio.
            applog::log_println!("control decode failed ({} bytes)", bytes.len());
            outcome = Some(true);
          }
        } else {
          // Foreign/corrupt frame: the radio received fine, it just isn't ours — neutral for the self-heal.
          outcome = None;
        }
      }
      Either::First(Err(e)) => {
        applog::log_println!("LoRa RX error: {:?}", e);
        outcome = Some(false);
      }
      Either::Second(()) => outcome = Some(false), // RX silence this window — a genuine "heard nothing" fault.
    }

    // Self-heal: LinkHealth folds the outcome (Some(true)=clean, Some(false)=radio fault, None=foreign/neutral),
    // re-inits the radio past the threshold to clear a stuck RX/TX stage, and backs off once Disconnected.
    match health.service(outcome, &mut link).await {
      lora_link::Serviced::Reinited(result) => {
        reinit_count += 1;
        let up = Instant::now().as_secs();
        let (h, m, s) = (up / 3600, (up % 3600) / 60, up % 60);
        match result {
          Ok(()) => applog::log_println!("radio re-init OK (#{}, up {}h{:02}m{:02}s)", reinit_count, h, m, s),
          Err(e) => applog::log_println!("radio re-init failed: {:?} (#{}, up {}h{:02}m{:02}s)", e, reinit_count, h, m, s),
        }
      }
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

/// Rounds an f32 to the nearest i32 using only `*`/`+` and the truncating `as i32` cast — no libm, so this is
/// usable in no_std without pulling a float-math dependency into the binary. Bias toward the value's own sign so
/// negative trims round symmetrically (e.g. -1.5 -> -2, 1.5 -> 2). IMU-home trim only, hence feature-gated.
#[cfg(feature = "imu")]
fn round_to_i32(v: f32) -> i32 {
  if v >= 0.0 { (v + 0.5) as i32 } else { (v - 0.5) as i32 }
}

/// Servo task. Wraps the two pre-built 50 Hz `SimplePwm` outputs in `servo::Servo` (via the nrf-adapters channel
/// adapter), and at a fixed ~50 Hz maps the latest `Control` pan/tilt us to PWM duty — but only writes when the
/// pose actually changed (new Control, failsafe recenter, or the first tick), since the PWM holds its duty. On a
/// loss of signal (no fresh Control within `CONTROL_TIMEOUT`) it recenters the gimbal. Never blocks on the radio.
#[embassy_executor::task]
async fn servo_task(pan_pwm: SimplePwm<'static>, tilt_pwm: SimplePwm<'static>, pan_trim: i32, tilt_trim: i32) {
  // PwmServoChannel adapts each SimplePwm channel 0 to embedded_hal::pwm::SetDutyCycle; Servo caches its max_duty
  // (20_000, so 1 tick = 1 us) and converts us -> duty for us.
  let mut pan_servo = Servo::new(PwmServoChannel::new(pan_pwm, 0));
  let mut tilt_servo = Servo::new(PwmServoChannel::new(tilt_pwm, 0));

  // Center trim from the boot IMU home (pan from roll, tilt from pitch), already bounded + converted to us in main
  // — or zero when the `imu` feature is off. A tilted mount shifts the gimbal center so head-tracking pivots about
  // level; with no IMU these are zero and the gimbal centers on the pure servo neutral.

  let mut ticker = Ticker::every(SERVO_PERIOD);
  let center =
    Control { pan_us: servo::CENTER_PULSE_US as u32, tilt_us: servo::CENTER_PULSE_US as u32, flags: 0 };
  let mut latest = center;
  // Timestamp of the last accepted Control, for the loss-of-signal failsafe.
  let mut last_control = Instant::now();
  // Force a write on the first iteration so the gimbal homes immediately; thereafter only write when something
  // actually changed (the PWM peripheral holds its duty between writes).
  let mut dirty = true;

  // Servo pulse band as i32 so the trim offset (which can be negative) is applied and clamped before narrowing.
  let min = servo::MIN_PULSE_US as i32;
  let max = servo::MAX_PULSE_US as i32;

  loop {
    ticker.next().await;

    if let Some(control) = CONTROL.try_take() {
      latest = control;
      last_control = Instant::now();
      dirty = true;
    } else if latest != center && Instant::now().duration_since(last_control) >= CONTROL_TIMEOUT {
      // Loss-of-signal failsafe: no fresh Control for CONTROL_TIMEOUT, so recenter (the home trim below still
      // applies, pointing "forward") instead of holding the last pose forever. Re-arm by treating center as the
      // current pose so the failsafe fires once, not every tick.
      latest = center;
      dirty = true;
    }

    // Nothing changed and the PWM holds its last duty, so skip the register writes on an idle tick.
    if !dirty {
      continue;
    }
    dirty = false;

    // Apply the home trim then clamp into the servo band in i32 BEFORE narrowing to u16, so a large/garbage
    // decoded us can never wrap past the clamp and slam a mechanical stop. set_pulse_us re-clamps to the duty
    // range; set_duty_cycle on the PWM adapter is Infallible, but log if it ever reports an error.
    let pan = ((latest.pan_us as i32) + pan_trim).clamp(min, max) as u16;
    let tilt = ((latest.tilt_us as i32) + tilt_trim).clamp(min, max) as u16;
    if pan_servo.set_pulse_us(pan).is_err() {
      applog::log_println!("pan servo duty error");
    }
    if tilt_servo.set_pulse_us(tilt).is_err() {
      applog::log_println!("tilt servo duty error");
    }
  }
}

/// Converts a boot IMU home angle into a center-trim in us, rejecting an implausible angle. A present-but-garbage
/// MPU-6050 can ACK yet return bogus accel; an angle beyond `MAX_HOME_DEG` is treated as zero trim (calibration
/// suspect) and logged, rather than baking a large permanent center bias into the gimbal. f32 `as i32` truncates
/// toward zero (no libm), so `round_to_i32` biases by +/-0.5 first. TODO: calibrate sign/scale on hardware.
#[cfg(feature = "imu")]
fn bounded_home_trim(deg: f32, axis: &str) -> i32 {
  if deg.abs() > MAX_HOME_DEG {
    applog::log_println!("IMU home {} {} deg exceeds +/-{} — ignoring trim", axis, deg, MAX_HOME_DEG);
    0
  } else {
    round_to_i32(deg * US_PER_DEGREE as f32)
  }
}

/// GPS task. Reads NMEA from the UART and on each speed fix publishes a `Telemetry` for the LoRa reply (and a
/// copy for the OLED). It also owns the home point: the FIRST fix with a real position (fix quality != 0,
/// >= `MIN_SATS_FOR_HOME` satellites, coordinates present) latches home, and from then on each fix carries the
/// distance + bearing from home to the truck (computed here via `geo::nav`, so the goggle only displays them).
/// A `SET_HOME` signal (raised by the LoRa task on a re-home command) drops home so the next valid fix re-latches
/// it. Lossy Signals: only the latest fix matters to the reply.
#[embassy_executor::task]
async fn gps_task(mut reader: GpsReader<BufferedUarteRx<'static>>) {
  // The captured launch point; `None` until the first valid fix (or after a re-home request clears it).
  let mut home: Option<geo::CoordE7> = None;

  loop {
    match reader.next_fix().await {
      Ok(fix) => {
        // A re-home request clears the latched home so the current fix below re-establishes it.
        if SET_HOME.try_take().is_some() {
          home = None;
          applog::log_println!("GPS re-home requested — home cleared, will re-latch at next valid fix");
        }

        let sats = fix.satellites.unwrap_or(0) as u32;
        let fix_quality = fix.fix.map(|f| f.raw as u32).unwrap_or(0);

        // The gps crate only surfaces coordinates for a usable fix (RMC status 'A' or GGA quality != 0), so a
        // present position is already trustworthy. Gate on the coordinate's PRESENCE, never on its value (0,0 is a
        // valid point). Critically, do NOT also require sats/fix_quality here: those ride GGA only, so an RMC-only
        // receiver (which carries position but no sat count) would otherwise never produce a usable position.
        let position = match (fix.lat_e7, fix.lon_e7) {
          (Some(lat_e7), Some(lon_e7)) => Some(geo::CoordE7 { lat_e7, lon_e7 }),
          _ => None,
        };
        // Home is anchored on a STRONG fix: a position plus a decent satellite count WHEN one is reported. The
        // strict sat floor is a one-time guard so a weak first fix can't anchor home tens of meters off; when sats
        // are unknown (RMC-only), the 'A'-validated position is the best signal available, so accept it.
        let strong_fix = fix.satellites.is_none_or(|s| s as u32 >= MIN_SATS_FOR_HOME);

        // Once home is latched, ANY later valid position keeps reporting nav — a brief satellite dip still yields a
        // usable position, and blanking the "find my truck" readout back to ACQUIRING there would be worse than a
        // slightly noisier distance. nav_valid is false only until home is first anchored (or right after a re-home).
        let (dist_m, bearing_deg, nav_valid) = match position {
          Some(cur) => {
            if home.is_none() && strong_fix {
              home = Some(cur);
              applog::log_println!("GPS home latched: lat_e7 {} lon_e7 {} (sats {})", cur.lat_e7, cur.lon_e7, sats);
            }
            match home {
              Some(h) => {
                let nav = geo::nav(h, cur);
                (nav.distance_m, nav.bearing_deg as u32, true)
              }
              None => (0, 0, false), // have a position, but no strong-enough fix yet to anchor home.
            }
          }
          None => (0, 0, false),
        };

        // Publish to BOTH the LoRa reply (GPS_TELEM) and the OLED (SPEED_DISPLAY) via separate Signals — a Signal
        // hands each value to one taker. Build the small all-scalar struct twice rather than relying on Telemetry: Clone.
        let telem = Telemetry { speed_cm_s: fix.speed_cm_s, sats, fix_quality, dist_m, bearing_deg, nav_valid };
        GPS_TELEM.signal(telem);
        SPEED_DISPLAY.signal(telem);
      }
      Err(_) => {
        // embassy-nrf 0.10's BufferedUarteRx Error enum is empty and a UART overrun or ring-buffer-full PANICS
        // in the ISR (verified) — it is NOT recoverable here, so this arm cannot actually see an overrun. The
        // mitigation is upstream: a large RX ring (512 B) plus draining promptly in the tight next_fix loop. A
        // panic is caught by panic-persist and reported on the next boot. Pace any retry so we never spin hot.
        Timer::after(Duration::from_millis(100)).await;
      }
    }
  }
}

/// Status OLED task (truck) — the committed T1 "Arc / Center" layout: a segmented speedo arc wrapping a big MPH
/// readout, KM/H on the baseline, and RSSI + LoRa signal bars in the header. Speed comes from the local GPS
/// (`SPEED_DISPLAY`); the bars and link liveness both come from `LINK_RSSI` (published on each decoded Control).
/// Probes 0x3C/0x3D; a missing or dead panel just exits the task and the node runs headless. Renders at
/// OLED_PERIOD, kept off the LoRa/servo hot paths.
#[embassy_executor::task]
async fn oled_task(mut i2c: OledI2c) {
  let addr = match display::probe_address(&mut i2c).await {
    Some(a) => a,
    None => {
      applog::log_println!("truck OLED not found (no ACK at 0x3C/0x3D) — OLED task exiting, node runs headless");
      return;
    }
  };
  let mut oled = match display::StatusDisplay::new_with_addr(i2c, addr).await {
    Ok(d) => d,
    Err(e) => {
      applog::log_println!("truck OLED init failed at 0x{:02X}: {:?}", addr, e);
      return;
    }
  };
  let _ = oled.render_lines(&["TRUCK", "waiting for", "control..."]).await;

  let mut ticker = Ticker::every(OLED_PERIOD);
  // Latest GPS fix; defaults to zero speed/no-fix so the panel reads 0 mph with a live link before any GPS sentence.
  let mut last = Telemetry::default();
  // Last Control RSSI, floored so the bars read empty until the first frame (rssi_to_bars also zeroes when down).
  let mut last_rssi: i16 = -127;
  // Last Control-RX time for link liveness, seeded in the past so the link reads down until the first packet lands.
  let mut last_rx = Instant::now().saturating_sub(LINK_TIMEOUT);

  loop {
    ticker.next().await;
    if let Some(telem) = SPEED_DISPLAY.try_take() {
      last = telem;
    }
    // A decoded Control publishes LINK_RSSI; its arrival is also the link-liveness heartbeat, so stamp last_rx here.
    if let Some(rssi) = LINK_RSSI.try_take() {
      last_rssi = rssi;
      last_rx = Instant::now();
    }

    let linked = Instant::now().duration_since(last_rx) < LINK_TIMEOUT;
    let status = display::TruckStatus {
      mph: display::cm_s_to_mph(last.speed_cm_s),
      kmh: display::cm_s_to_kmh(last.speed_cm_s),
      bars: display::rssi_to_bars(last_rssi, linked),
      rssi: last_rssi,
      linked,
    };
    if let Err(e) = oled.render_truck(status).await {
      applog::log_println!("truck OLED render error: {:?}", e);
    }
  }
}
