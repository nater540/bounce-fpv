//! Truck node: receive pan/tilt over LoRa, drive two gimbal servos at a fixed rate, read a GPS for ground
//! speed, and show link/fix/speed on a shared-I2C OLED. The matching slave half of the goggle's half-duplex
//! link.
//!
//! Tasks and latest-value `Signal` hand-offs (all lossy — control loops act on the freshest sample, never a
//! queue):
//!   - startup one-shot (in `main`) — builds the shared async I2C bus, runs `imu::detect_home` to establish
//!     the gimbal center reference BEFORE the servo loop starts, and logs it.
//!   - `lora_task`  — RX a `Control`, decode, `CONTROL.signal(..)`; then read the latest `GpsFix` and TX a
//!     `Telemetry` reply (the slave side of the turnaround: it transmits only right after receiving).
//!   - `servo_task` — builds the LEDC 50 Hz timer + two channels, wraps each in `servo::Servo`, and at a fixed
//!     ~50 Hz reads the latest `Control` Signal and maps pan/tilt us straight to LEDC duty. Never touches the
//!     radio, so RX jitter can never reach the servos.
//!   - `gps_task`   — async UART -> `gps::GpsReader`; publishes each `GpsFix` for the LoRa reply and the OLED.
//!   - `oled_task`  — `display::StatusDisplay` on the shared I2C bus, renders speed (km/h) + link/fix flags.
//!
//! Half-duplex turnaround (scaffold scheme, tunable later): the goggle is master, the truck is slave. The truck
//! parks in continuous RX; on each received Control it applies it and immediately TXes the latest Telemetry,
//! then returns to RX. One radio per node, no collisions. See `goggle-node` for the master half.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Ticker, Timer};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::ledc::channel::config::Config as ChannelConfig;
use esp_hal::ledc::channel::{self, ChannelIFace};
use esp_hal::ledc::timer::config::Config as LedcTimerConfig;
use esp_hal::ledc::timer::{self, TimerIFace};
use esp_hal::ledc::{LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::gpio::DriveMode;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_hal::Async;
use esp_println::println;
use gps::{GpsFix, GpsReader};
use lora_link::{LoraLink, Sx1276Radio};
use micropb::{MessageDecode, MessageEncode, PbEncoder};
use proto::{Control, Telemetry};
use servo::Servo;
use static_cell::StaticCell;

// Manual panic handler + app descriptor (matches ppm-diag / goggle-node).
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
  println!("PANIC: {}", info);
  loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();

// Servo refresh rate. The MG90S frame is 50 Hz; refreshing duty at the same rate keeps the loop simple and well
// within the 50-100 Hz band the doc calls for. TODO: tune on hardware if the gimbal needs faster updates.
const SERVO_PERIOD: Duration = Duration::from_millis(20);

// LEDC servo timer: 50 Hz, 14-bit duty (servo crate's pulse_us_to_duty assumes the LEDC max_duty it reports).
const SERVO_FREQ_HZ: u32 = 50;

// Pulse-width change per degree of IMU home tilt, used to trim the gimbal center for a tilted mount. Derived
// from the MG90S ~400-2400 us / ~180 deg span ((2400-400)/180 ~= 11 us/deg). TODO: calibrate on hardware.
const US_PER_DEGREE: i32 = 11;

// GPS UART baud. Most modules default to 9600. TODO: confirm against the specific receiver.
const GPS_BAUD: u32 = 9_600;

// OLED refresh cadence. The panel only shows a human-readable glance, so a few Hz is plenty.
const OLED_PERIOD: Duration = Duration::from_millis(250);

// TX power in dBm for the Telemetry reply (RFM95W PA_BOOST). TODO: tune on hardware / per regional limits.
const TX_POWER_DBM: i32 = 17;

// Latest pan/tilt command from the radio, consumed by the servo loop (lossy: only the freshest pose matters).
static CONTROL: Signal<CriticalSectionRawMutex, Control> = Signal::new();
// Latest GPS fix from the UART, consumed by the LoRa reply and the OLED. Both readers re-take it independently.
static GPS_TELEM: Signal<CriticalSectionRawMutex, Telemetry> = Signal::new();
static GPS_STATUS: Signal<CriticalSectionRawMutex, GpsFix> = Signal::new();

// Shared async I2C bus behind a NoopRawMutex (single executor): the MPU-6050 (0x68) and SSD1306 (0x3C) each get
// their own I2cDevice borrowing this StaticCell-held Mutex for the program's lifetime.
static I2C_BUS: StaticCell<Mutex<NoopRawMutex, I2c<'static, Async>>> = StaticCell::new();

// Concrete radio + I2C-device types so the spawned tasks have non-generic signatures.
type RadioSpi = ExclusiveDevice<Spi<'static, Async>, Output<'static>, Delay>;
type Radio = Sx1276Radio<RadioSpi, Output<'static>, Input<'static>>;
type Link = LoraLink<Radio, Delay>;
type SharedI2c = I2cDevice<'static, NoopRawMutex, I2c<'static, Async>>;

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
  let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
  let peripherals = esp_hal::init(config);

  // `board_pins!` partial-moves only the GPIO pin fields out of `peripherals` into BoardPins, so the binding
  // retains the controller singletons (SPI2/I2C0/UART1/LEDC/TIMG0/SW_INTERRUPT) and we use them directly below.
  let pins = board::board_pins!(peripherals);

  let timg0 = TimerGroup::new(peripherals.TIMG0);
  let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
  esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

  println!();
  println!("=== truck-node: LoRa RX -> servos + IMU + GPS + OLED ===");

  // Shared async I2C bus on I2C0 at 400 kHz, created blocking then converted to async and parked in the
  // StaticCell so I2cDevice handles can borrow it for 'static.
  let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
    .expect("I2C0 config")
    .with_sda(pins.i2c_sda)
    .with_scl(pins.i2c_scl)
    .into_async();
  let i2c_bus: &'static Mutex<NoopRawMutex, I2c<'static, Async>> = I2C_BUS.init(Mutex::new(i2c));

  // Startup one-shot: establish the gimbal home from the IMU's resting orientation BEFORE the servo loop runs.
  // detect_home consumes its I2cDevice; the OLED gets a separate device on the same bus afterward.
  let mut delay = Delay;
  // Resolve the boot home so the servo loop can trim the gimbal center for a tilted mount. Home is Copy, so it
  // is passed by value into servo_task below.
  let home = match imu::detect_home_default(I2cDevice::new(i2c_bus), &mut delay).await {
    Ok(home) => {
      println!("IMU home: roll {} deg, pitch {} deg", home.roll_deg, home.pitch_deg);
      home
    }
    // A missing/miswired IMU must not brick the truck; log and fall back to a zero trim (fixed center). TODO:
    // surface on OLED.
    Err(e) => {
      println!("IMU home detection failed ({:?}) — using fixed center", e);
      imu::Home::default()
    }
  };

  // Build the SX1276 SPI bus on SPI2 (same wiring scheme as the goggle node).
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
  let link: Link = lora_link::build_sx1276(spi_dev, reset, dio0, Delay, true)
    .await
    .expect("SX1276 radio init");

  // GPS UART on UART1 (UART0 is left for the console path); RX carries the NMEA stream, TX is config-only.
  let uart = Uart::new(peripherals.UART1, UartConfig::default().with_baudrate(GPS_BAUD))
    .expect("UART1 config")
    .with_rx(pins.gps_rx)
    .with_tx(pins.gps_tx)
    .into_async();

  // OLED device shares the same I2C bus as the (now-released) IMU device.
  let oled_i2c: SharedI2c = I2cDevice::new(i2c_bus);

  // Hand the LEDC peripheral and the two servo pins to the servo task, which builds the timer + channels there
  // so all the LEDC borrows stay task-local.
  let ledc = Ledc::new(peripherals.LEDC);

  // The task macro returns a Result<SpawnToken, SpawnError> (the pool-full case); unwrap the token then spawn.
  spawner.spawn(lora_task(link).expect("lora_task token"));
  spawner.spawn(servo_task(ledc, pins.servo_pan, pins.servo_tilt, home).expect("servo_task token"));
  spawner.spawn(gps_task(GpsReader::new(uart)).expect("gps_task token"));
  spawner.spawn(oled_task(oled_i2c).expect("oled_task token"));

  loop {
    Timer::after(Duration::from_secs(3600)).await;
  }
}

/// LoRa task (link slave). Parks in RX; on each received `Control` it decodes and publishes the latest pose for
/// the servo loop, then immediately TXes the freshest `Telemetry` reply (the half-duplex turnaround) before
/// returning to RX. Servos are driven elsewhere — this task only hands off the pose via the Signal.
#[embassy_executor::task]
async fn lora_task(mut link: Link) {
  let mut rx_buf = [0u8; lora_link::MAX_PAYLOAD as usize];
  // Most recent telemetry to reply with; updated from the GPS Signal each loop, defaults to "no fix".
  let mut telem = Telemetry::default();

  loop {
    let len = match link.receive(&mut rx_buf).await {
      Ok(len) => len,
      Err(e) => {
        println!("LoRa RX error: {:?}", e);
        continue;
      }
    };

    let mut control = Control::default();
    if control.decode_from_bytes(&rx_buf[..len]).is_ok() {
      CONTROL.signal(control);
    } else {
      // CRC-valid but garbage Control: skip the turnaround so we do not waste half-duplex air-time on a reply.
      println!("control decode failed ({} bytes)", len);
      continue;
    }

    // Refresh the reply payload from the latest GPS fix (if any), then transmit it as the turnaround response.
    if let Some(latest) = GPS_TELEM.try_take() {
      telem = latest;
    }
    let mut enc = PbEncoder::new(heapless::Vec::<u8, 18>::new());
    if telem.encode(&mut enc).is_err() {
      println!("telemetry encode failed");
      continue;
    }
    let payload = enc.into_writer();
    if let Err(e) = link.send(&payload, TX_POWER_DBM).await {
      println!("LoRa TX (telemetry) error: {:?}", e);
    }
  }
}

/// Rounds an f32 to the nearest i32 using only `*`/`+` and the truncating `as i32` cast — no libm, so this is
/// usable in no_std without pulling a float-math dependency into the binary. Bias toward the value's own sign so
/// negative trims round symmetrically (e.g. -1.5 -> -2, 1.5 -> 2).
fn round_to_i32(v: f32) -> i32 {
  if v >= 0.0 { (v + 0.5) as i32 } else { (v - 0.5) as i32 }
}

/// Servo task. Builds the shared 50 Hz LEDC timer and the two pan/tilt channels, wraps each in `servo::Servo`,
/// and at a fixed ~50 Hz maps the latest `Control` pan/tilt us straight to LEDC duty. The first command is the
/// center, so the gimbal homes until real head-tracking data arrives. Never blocks on the radio.
#[embassy_executor::task]
async fn servo_task(
  ledc: Ledc<'static>,
  pan_pin: board::ServoPanPin,
  tilt_pin: board::ServoTiltPin,
  home: imu::Home,
) {
  let mut ledc = ledc;
  ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

  // One shared low-speed timer at 50 Hz, 14-bit duty. The C6 has only low-speed LEDC and all timers share one
  // clock; both servo channels hang off this single timer.
  let mut lstimer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
  lstimer
    .configure(LedcTimerConfig {
      duty: timer::config::Duty::Duty14Bit,
      clock_source: timer::LSClockSource::APBClk,
      frequency: Rate::from_hz(SERVO_FREQ_HZ),
    })
    .expect("LEDC timer configure");

  let mut pan = ledc.channel::<LowSpeed>(channel::Number::Channel0, pan_pin);
  pan
    .configure(ChannelConfig { timer: &lstimer, duty_pct: 0, drive_mode: DriveMode::PushPull })
    .expect("LEDC pan channel configure");
  let mut tilt = ledc.channel::<LowSpeed>(channel::Number::Channel1, tilt_pin);
  tilt
    .configure(ChannelConfig { timer: &lstimer, duty_pct: 0, drive_mode: DriveMode::PushPull })
    .expect("LEDC tilt channel configure");

  // Wrap the configured channels: Servo caches each channel's max_duty and converts us -> duty for us.
  let mut pan_servo = Servo::new(pan);
  let mut tilt_servo = Servo::new(tilt);

  // Center trim from the IMU boot home: a tilted mount shifts the gimbal center so head-tracking pivots about
  // level. roll trims pan, pitch trims tilt. f32 `as i32` truncates toward zero (and needs no libm), so bias by
  // +/-0.5 first to round to the nearest us. TODO: calibrate sign/scale on hardware.
  let pan_trim = round_to_i32(home.roll_deg * US_PER_DEGREE as f32);
  let tilt_trim = round_to_i32(home.pitch_deg * US_PER_DEGREE as f32);

  let mut ticker = Ticker::every(SERVO_PERIOD);
  let mut latest = Control { pan_us: servo::CENTER_PULSE_US as u32, tilt_us: servo::CENTER_PULSE_US as u32 };

  // Servo pulse band as i32 so the trim offset (which can be negative) is applied and clamped before narrowing.
  let min = servo::MIN_PULSE_US as i32;
  let max = servo::MAX_PULSE_US as i32;

  loop {
    ticker.next().await;
    if let Some(control) = CONTROL.try_take() {
      latest = control;
    }
    // Apply the home trim then clamp into the servo band in i32 BEFORE narrowing to u16, so a large/garbage
    // decoded us can never wrap past the clamp and slam a mechanical stop. set_pulse_us re-clamps to the duty
    // range; set_duty_cycle on an LEDC channel is infallible in practice, but log if it ever reports an error.
    let pan = ((latest.pan_us as i32) + pan_trim).clamp(min, max) as u16;
    let tilt = ((latest.tilt_us as i32) + tilt_trim).clamp(min, max) as u16;
    if pan_servo.set_pulse_us(pan).is_err() {
      println!("pan servo duty error");
    }
    if tilt_servo.set_pulse_us(tilt).is_err() {
      println!("tilt servo duty error");
    }
  }
}

/// GPS task. Reads NMEA from the UART and on each speed fix publishes both a `Telemetry` (for the LoRa reply)
/// and the raw `GpsFix` (for the OLED). Lossy Signals: only the latest fix matters to either consumer.
#[embassy_executor::task]
async fn gps_task(mut reader: GpsReader<Uart<'static, Async>>) {
  loop {
    match reader.next_fix().await {
      Ok(fix) => {
        let sats = fix.satellites.unwrap_or(0) as u32;
        let fix_quality = fix.fix.map(|f| f.raw as u32).unwrap_or(0);
        GPS_TELEM.signal(Telemetry { speed_cm_s: fix.speed_cm_s, sats, fix_quality });
        GPS_STATUS.signal(fix);
      }
      Err(_) => {
        // A UART read error (framing/overrun) is transient; pace retries so a wiring fault does not spin hot.
        Timer::after(Duration::from_millis(100)).await;
      }
    }
  }
}

/// OLED status task. Renders ground speed (km/h) plus link/fix flags on the shared I2C bus. "Link up" here is a
/// simple heuristic: a GPS fix has been seen. TODO: drive link_up from actual LoRa RX liveness once measured.
#[embassy_executor::task]
async fn oled_task(i2c: SharedI2c) {
  let mut oled = match display::StatusDisplay::new(i2c).await {
    Ok(d) => d,
    Err(e) => {
      // A missing/miswired panel must not brick the truck; log and let the task exit cleanly.
      println!("OLED init failed: {:?}", e);
      return;
    }
  };

  let mut ticker = Ticker::every(OLED_PERIOD);
  let mut last = GpsFix::default();

  loop {
    ticker.next().await;
    if let Some(fix) = GPS_STATUS.try_take() {
      last = fix;
    }
    let gps_fix = last.fix.map(|f| f.has_fix()).unwrap_or(false);
    let status = display::Status { speed_cm_s: last.speed_cm_s, link_up: gps_fix, gps_fix };
    if let Err(e) = oled.render(status).await {
      println!("OLED render error: {:?}", e);
    }
  }
}
