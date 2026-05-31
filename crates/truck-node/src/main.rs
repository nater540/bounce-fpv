//! Truck node (Phase D: nRF52840, headless): receive pan/tilt over LoRa, drive two gimbal servos at a fixed
//! rate, read a GPS for ground speed, and reply with Telemetry. The matching slave half of the goggle's
//! half-duplex link. No OLED — the status panel moved to the goggle, so the only I2C device here (the MPU-6050)
//! uses a direct `Twim`, not a shared bus.
//!
//! Tasks and latest-value `Signal` hand-offs (all lossy — control loops act on the freshest sample, never a
//! queue):
//!   - startup one-shot (in `main`) — builds the MPU-6050 `Twim` and runs `imu::detect_home_default` to
//!     establish the gimbal center reference BEFORE the servo loop starts, then passes it into `servo_task`.
//!   - `lora_task`  — RX a `Control`, decode; on success `CONTROL.signal(..)`, on decode failure `continue`
//!     (never replies to garbage); then read the latest GPS `Telemetry` and TX it as the turnaround reply.
//!   - `servo_task` — builds the two 50 Hz `SimplePwm` outputs (PWM0 pan, PWM1 tilt), wraps each in
//!     `servo::Servo`, and at a fixed ~50 Hz reads the latest `Control`, applies the boot IMU home trim, clamps
//!     into the servo band, and drives the PWM duty. Never touches the radio, so RX jitter never reaches servos.
//!   - `gps_task`   — `BufferedUarteRx` -> `gps::GpsReader`; publishes each speed fix as a `Telemetry` for the
//!     LoRa reply.
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
use embassy_nrf::buffered_uarte::{self, BufferedUarteRx};
use embassy_nrf::pwm::SimplePwm;
use embassy_nrf::spim;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::uarte::{self, Baudrate};
use embassy_nrf::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};
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
const US_PER_DEGREE: i32 = 11;

// Loss-of-signal failsafe window: if no fresh Control arrives within this span the servo loop recenters the
// gimbal (with the home trim still applied, so it points "forward") rather than holding the last pose forever.
// TODO: tune on hardware.
const CONTROL_TIMEOUT: Duration = Duration::from_millis(1000);

// Plausible mount-tilt bound for the boot IMU home angle. A present-but-garbage MPU-6050 that ACKs returns bogus
// accel, which would otherwise bake a large permanent center bias into the trim; an angle beyond this is treated
// as zero trim (calibration suspect). TODO: tune on hardware.
const MAX_HOME_DEG: f32 = 30.0;

// GPS UART baud. Most modules default to 9600. TODO: confirm against the specific receiver.
const GPS_BAUD: Baudrate = Baudrate::BAUD9600;

// TX power in dBm for the Telemetry reply (RFM95W PA_BOOST). TODO: tune on hardware / per regional limits.
const TX_POWER_DBM: i32 = 17;

// PA output routing. The bare RFM95W bonds ONLY its PA_BOOST pin to the antenna, so tx_boost MUST be true or the
// radiated power is near-zero and the link silently fails to come up (see lora-ping for the full rationale).
const TX_BOOST: bool = true;

// Latest pan/tilt command from the radio, consumed by the servo loop (lossy: only the freshest pose matters).
// CriticalSectionRawMutex because the Signal is a `static` shared across tasks.
static CONTROL: Signal<CriticalSectionRawMutex, Control> = Signal::new();
// Latest GPS-derived telemetry from the UART, consumed by the LoRa reply. The reader re-takes it each turnaround.
static GPS_TELEM: Signal<CriticalSectionRawMutex, Telemetry> = Signal::new();

// The radio type lives in `nrf_adapters::lora` now (shared by both nodes + lora-ping so it can never drift); the
// spawned LoRa task takes `nrf_adapters::lora::Link` directly for its non-generic signature.

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

  // Startup one-shot: establish the gimbal home from the IMU's resting orientation BEFORE the servo loop runs.
  // The MPU-6050 is the only I2C device on this node, so it gets a direct Twim (no shared bus). The TWIM
  // tx_ram_buffer must be 'static (it outlives the 'static peripheral); a StaticCell gives it that, and 16 bytes
  // is ample for the small command bytes the driver sends.
  static TWIM_TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
  let tx_buf = TWIM_TX_BUF.init([0; 16]);
  let mut i2c = Twim::new(p.TWISPI0, Irqs, pins.i2c_sda, pins.i2c_scl, twim::Config::default(), tx_buf);
  let mut delay = Delay;
  // Resolve the boot home so the servo loop can trim the gimbal center for a tilted mount. Home is Copy, so it is
  // passed by value into servo_task below.
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
    Ok(link) => link,
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
  spawner.spawn(servo_task(pan_pwm, tilt_pwm, home).expect("servo_task token"));
  spawner.spawn(gps_task(GpsReader::new(gps_rx)).expect("gps_task token"));
}

/// LoRa task (link slave). Parks in RX; on each received `Control` it decodes and publishes the latest pose for
/// the servo loop, then immediately TXes the freshest `Telemetry` reply (the half-duplex turnaround) before
/// returning to RX. Servos are driven elsewhere — this task only hands off the pose via the Signal.
#[embassy_executor::task]
async fn lora_task(mut link: nrf_adapters::lora::Link) {
  let mut rx_buf = [0u8; lora_link::MAX_PAYLOAD as usize];
  // Most recent telemetry to reply with; updated from the GPS Signal each loop, defaults to "no fix".
  let mut telem = Telemetry::default();

  loop {
    let len = match link.receive(&mut rx_buf).await {
      Ok(len) => len,
      Err(e) => {
        applog::log_println!("LoRa RX error: {:?}", e);
        continue;
      }
    };

    let mut control = Control::default();
    if control.decode_from_bytes(&rx_buf[..len]).is_ok() {
      CONTROL.signal(control);
    } else {
      // CRC-valid but garbage Control: skip the turnaround so we do not waste half-duplex air-time on a reply.
      applog::log_println!("control decode failed ({} bytes)", len);
      continue;
    }

    // Refresh the reply payload from the latest GPS fix (if any), then transmit it as the turnaround response.
    if let Some(latest) = GPS_TELEM.try_take() {
      telem = latest;
    }
    let mut enc = PbEncoder::new(heapless::Vec::<u8, 18>::new());
    if telem.encode(&mut enc).is_err() {
      applog::log_println!("telemetry encode failed");
      continue;
    }
    let payload = enc.into_writer();
    if let Err(e) = link.send(&payload, TX_POWER_DBM).await {
      applog::log_println!("LoRa TX (telemetry) error: {:?}", e);
    }
  }
}

/// Rounds an f32 to the nearest i32 using only `*`/`+` and the truncating `as i32` cast — no libm, so this is
/// usable in no_std without pulling a float-math dependency into the binary. Bias toward the value's own sign so
/// negative trims round symmetrically (e.g. -1.5 -> -2, 1.5 -> 2).
fn round_to_i32(v: f32) -> i32 {
  if v >= 0.0 { (v + 0.5) as i32 } else { (v - 0.5) as i32 }
}

/// Servo task. Wraps the two pre-built 50 Hz `SimplePwm` outputs in `servo::Servo` (via the nrf-adapters channel
/// adapter), and at a fixed ~50 Hz maps the latest `Control` pan/tilt us to PWM duty — but only writes when the
/// pose actually changed (new Control, failsafe recenter, or the first tick), since the PWM holds its duty. On a
/// loss of signal (no fresh Control within `CONTROL_TIMEOUT`) it recenters the gimbal. Never blocks on the radio.
#[embassy_executor::task]
async fn servo_task(pan_pwm: SimplePwm<'static>, tilt_pwm: SimplePwm<'static>, home: imu::Home) {
  // PwmServoChannel adapts each SimplePwm channel 0 to embedded_hal::pwm::SetDutyCycle; Servo caches its max_duty
  // (20_000, so 1 tick = 1 us) and converts us -> duty for us.
  let mut pan_servo = Servo::new(PwmServoChannel::new(pan_pwm, 0));
  let mut tilt_servo = Servo::new(PwmServoChannel::new(tilt_pwm, 0));

  // Center trim from the IMU boot home: a tilted mount shifts the gimbal center so head-tracking pivots about
  // level. roll trims pan, pitch trims tilt. Bound each angle first: a present-but-garbage MPU-6050 that ACKs
  // returns bogus accel, which would bake a large permanent center bias into the trim; an angle beyond
  // MAX_HOME_DEG is treated as zero trim (boot calibration suspect) and logged. f32 `as i32` truncates toward
  // zero (and needs no libm), so round_to_i32 biases by +/-0.5 first. TODO: calibrate sign/scale on hardware.
  let pan_trim = bounded_home_trim(home.roll_deg, "roll");
  let tilt_trim = bounded_home_trim(home.pitch_deg, "pitch");

  let mut ticker = Ticker::every(SERVO_PERIOD);
  let center = Control { pan_us: servo::CENTER_PULSE_US as u32, tilt_us: servo::CENTER_PULSE_US as u32 };
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
/// suspect) and logged, rather than baking a large permanent center bias into the gimbal.
fn bounded_home_trim(deg: f32, axis: &str) -> i32 {
  if deg.abs() > MAX_HOME_DEG {
    applog::log_println!("IMU home {} {} deg exceeds +/-{} — ignoring trim", axis, deg, MAX_HOME_DEG);
    0
  } else {
    round_to_i32(deg * US_PER_DEGREE as f32)
  }
}

/// GPS task. Reads NMEA from the UART and on each speed fix publishes a `Telemetry` for the LoRa reply. Lossy
/// Signal: only the latest fix matters to the reply.
#[embassy_executor::task]
async fn gps_task(mut reader: GpsReader<BufferedUarteRx<'static>>) {
  loop {
    match reader.next_fix().await {
      Ok(fix) => {
        let sats = fix.satellites.unwrap_or(0) as u32;
        let fix_quality = fix.fix.map(|f| f.raw as u32).unwrap_or(0);
        GPS_TELEM.signal(Telemetry { speed_cm_s: fix.speed_cm_s, sats, fix_quality });
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
