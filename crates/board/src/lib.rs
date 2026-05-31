//! Board pin-map for the FPV head-tracking system. This crate owns the single source of truth for
//! every GPIO assignment, so the node binaries never hard-code pin numbers. It is the one driver-side
//! crate that deliberately depends on esp-hal concrete types: the `board_pins!` macro destructures the
//! chip `Peripherals` into a `BoardPins` struct of named, typed pins each subsystem turns into its driver.
//!
//! Two layouts are selected by Cargo feature, never by separate crate: `board-xiao` (default, Seeed XIAO
//! ESP32-C6) and `board-devkit` (Espressif ESP32-C6-DevKitC-1). They differ only in pin numbers. On the
//! XIAO, GPIO3 (RF_SWITCH_EN) and GPIO14 (RF_ANT_SELECT) drive the FM8625H RF switch and are RESERVED —
//! this pin-map never assigns them to PPM, servos, SPI, I2C, or UART.

#![no_std]

// Exactly one board feature must be active. Guard the empty / double-selected cases at compile time so a
// misconfigured binary fails the build instead of silently picking the wrong pins.
#[cfg(all(feature = "board-xiao", feature = "board-devkit"))]
compile_error!("enable exactly one board feature: `board-xiao` or `board-devkit`, not both");
#[cfg(not(any(feature = "board-xiao", feature = "board-devkit")))]
compile_error!("enable one board feature: `board-xiao` (default) or `board-devkit`");

// Per-board role -> GPIO-number table. Each board module binds the same set of role identifiers to a
// concrete esp-hal `peripherals::GPIOn` type and its matching `Peripherals` field, so the rest of the
// crate (and the binaries) speak in roles. The numeric choices and rationale are documented inline below.
//
// The macro emits, for each board, the `BoardPins` field types and the `board_pins!` field moves. We keep
// it as one `define_board!` invocation per feature so adding a board is a single localized edit.

/// Builds `BoardPins` by partial-moving the role pins out of the caller's own `Peripherals` binding.
/// Invoke once at boot, after `esp_hal::init`: `let pins = board::board_pins!(peripherals);`. The macro
/// reads `peripherals.GPIOn` fields in place — it does NOT take ownership of the whole struct — so the
/// remaining peripheral blocks (SPI2/I2C0/UART1/LEDC/TIMG0/SW_INTERRUPT/...) stay owned by and accessible
/// through the caller's `peripherals` variable afterward. Only the named pin fields are moved out, which a
/// plain struct like esp-hal's `Peripherals` permits. This mirrors esp-hal's own pin-splitting, lets the
/// node binaries keep using the controller singletons directly, and needs no `unsafe`/`steal()`.
///
/// The argument must be an addressable place expression (a `let` binding, not a temporary): each field is
/// moved out of it by value, so the rest of that same binding remains usable after the call.
#[macro_export]
macro_rules! board_pins {
  ($peripherals:expr) => {
    $crate::BoardPins {
      ppm: $peripherals.GPIO2,
      servo_pan: $crate::__servo_pan!($peripherals),
      servo_tilt: $crate::__servo_tilt!($peripherals),
      lora_sck: $crate::__lora_sck!($peripherals),
      lora_mosi: $crate::__lora_mosi!($peripherals),
      lora_miso: $crate::__lora_miso!($peripherals),
      lora_nss: $crate::__lora_nss!($peripherals),
      lora_reset: $crate::__lora_reset!($peripherals),
      lora_dio0: $crate::__lora_dio0!($peripherals),
      i2c_sda: $crate::__i2c_sda!($peripherals),
      i2c_scl: $crate::__i2c_scl!($peripherals),
      gps_tx: $crate::__gps_tx!($peripherals),
      gps_rx: $crate::__gps_rx!($peripherals),
    }
  };
}

// One macro per role so the board feature picks the exact `Peripherals` field by name. These are crate
// internals (the leading `__`); only `board_pins!` and `BoardPins` are meant to be used directly.
macro_rules! role_accessors {
  (
    $servo_pan:ident, $servo_tilt:ident,
    $lora_sck:ident, $lora_mosi:ident, $lora_miso:ident, $lora_nss:ident, $lora_reset:ident, $lora_dio0:ident,
    $i2c_sda:ident, $i2c_scl:ident,
    $gps_tx:ident, $gps_rx:ident $(,)?
  ) => {
    #[doc(hidden)] #[macro_export] macro_rules! __servo_pan   { ($p:expr) => { $p.$servo_pan }; }
    #[doc(hidden)] #[macro_export] macro_rules! __servo_tilt  { ($p:expr) => { $p.$servo_tilt }; }
    #[doc(hidden)] #[macro_export] macro_rules! __lora_sck    { ($p:expr) => { $p.$lora_sck }; }
    #[doc(hidden)] #[macro_export] macro_rules! __lora_mosi   { ($p:expr) => { $p.$lora_mosi }; }
    #[doc(hidden)] #[macro_export] macro_rules! __lora_miso   { ($p:expr) => { $p.$lora_miso }; }
    #[doc(hidden)] #[macro_export] macro_rules! __lora_nss    { ($p:expr) => { $p.$lora_nss }; }
    #[doc(hidden)] #[macro_export] macro_rules! __lora_reset  { ($p:expr) => { $p.$lora_reset }; }
    #[doc(hidden)] #[macro_export] macro_rules! __lora_dio0   { ($p:expr) => { $p.$lora_dio0 }; }
    #[doc(hidden)] #[macro_export] macro_rules! __i2c_sda     { ($p:expr) => { $p.$i2c_sda }; }
    #[doc(hidden)] #[macro_export] macro_rules! __i2c_scl     { ($p:expr) => { $p.$i2c_scl }; }
    #[doc(hidden)] #[macro_export] macro_rules! __gps_tx      { ($p:expr) => { $p.$gps_tx }; }
    #[doc(hidden)] #[macro_export] macro_rules! __gps_rx      { ($p:expr) => { $p.$gps_rx }; }
  };
}

/// The complete set of pins this firmware uses, handed out once at boot by `board_pins!`. Fields are
/// grouped by subsystem; each is a concrete esp-hal pin type the binary passes straight into `Input::new`,
/// `Output::new`, an SPI/I2C/UART builder, or an LEDC channel. The peripheral blocks (SPI2/I2C0/UART/LEDC)
/// stay with the binary — only the pin routing lives here.
pub struct BoardPins {
  /// PPM input from the goggles' HT OUT jack (goggle node). GPIO2 on both boards, matching ppm-diag.
  pub ppm: esp_hal::peripherals::GPIO2<'static>,
  /// Servo PWM outputs (truck node): pan / tilt, one LEDC channel each off the shared 50 Hz timer. Drive
  /// the servos from a separate 5 V rail with common ground, never the 3.3 V pin.
  pub servo_pan: ServoPanPin,
  pub servo_tilt: ServoTiltPin,
  /// LoRa SX1276/RFM95W SPI bus + control lines. SCK/MOSI/MISO go to an SPI2 instance; NSS is an Output
  /// wrapped with `embedded-hal-bus` `ExclusiveDevice`; RESET is an Output; DIO0 is the IRQ Input the RX/TX
  /// futures await on.
  pub lora_sck: LoraSckPin,
  pub lora_mosi: LoraMosiPin,
  pub lora_miso: LoraMisoPin,
  pub lora_nss: LoraNssPin,
  pub lora_reset: LoraResetPin,
  pub lora_dio0: LoraDio0Pin,
  /// Shared async I2C bus (truck node): MPU-6050 (0x68) + SSD1306 (0x3C) behind one `shared_bus` Mutex.
  pub i2c_sda: I2cSdaPin,
  pub i2c_scl: I2cSclPin,
  /// GPS UART (bonus): TX to the module (config only), RX carrying the module's NMEA stream into the C6.
  pub gps_tx: GpsTxPin,
  pub gps_rx: GpsRxPin,
}

#[cfg(feature = "board-xiao")]
mod board {
  // Seeed XIAO ESP32-C6. Chosen to keep the RF-reserved pins clear: GPIO3/GPIO14 are the FM8625H RF switch
  // and are never assigned here. PPM stays on GPIO2 to match the ppm-diag default the bring-up agent uses.
  use esp_hal::peripherals;
  pub type ServoPanPin = peripherals::GPIO16<'static>; //  pan servo PWM (LEDC ch0), 5 V-rail driven, common GND.
  pub type ServoTiltPin = peripherals::GPIO17<'static>; //  tilt servo PWM (LEDC ch1), same 50 Hz LEDC timer.
  pub type LoraSckPin = peripherals::GPIO19<'static>; //  SX1276 SPI clock.
  pub type LoraMosiPin = peripherals::GPIO18<'static>; //  SX1276 SPI MOSI (C6 -> radio).
  pub type LoraMisoPin = peripherals::GPIO20<'static>; //  SX1276 SPI MISO (radio -> C6).
  pub type LoraNssPin = peripherals::GPIO21<'static>; //  SX1276 chip-select (Output, ExclusiveDevice).
  pub type LoraResetPin = peripherals::GPIO22<'static>; //  SX1276 RESET (Output, active-low reset sequence).
  pub type LoraDio0Pin = peripherals::GPIO23<'static>; //  SX1276 DIO0 IRQ (Input) — drives RX/TX await futures.
  pub type I2cSdaPin = peripherals::GPIO6<'static>; //  shared I2C SDA: MPU-6050 (0x68) + SSD1306 (0x3C).
  pub type I2cSclPin = peripherals::GPIO7<'static>; //  shared I2C SCL.
  pub type GpsTxPin = peripherals::GPIO0<'static>; //  UART TX to the GPS module (config only; reader needs RX).
  pub type GpsRxPin = peripherals::GPIO1<'static>; //  UART RX: the GPS module's NMEA stream into the C6.

  // GPIO-number constants for diagnostics/logging (the typed pins above carry the real binding).
  pub const PPM: u8 = 2;
  pub const SERVO_PAN: u8 = 16;
  pub const SERVO_TILT: u8 = 17;
  pub const LORA_SCK: u8 = 19;
  pub const LORA_MOSI: u8 = 18;
  pub const LORA_MISO: u8 = 20;
  pub const LORA_NSS: u8 = 21;
  pub const LORA_RESET: u8 = 22;
  pub const LORA_DIO0: u8 = 23;
  pub const I2C_SDA: u8 = 6;
  pub const I2C_SCL: u8 = 7;
  pub const GPS_TX: u8 = 0;
  pub const GPS_RX: u8 = 1;
}

#[cfg(feature = "board-xiao")]
role_accessors!(GPIO16, GPIO17, GPIO19, GPIO18, GPIO20, GPIO21, GPIO22, GPIO23, GPIO6, GPIO7, GPIO0, GPIO1);

#[cfg(feature = "board-devkit")]
mod board {
  // Espressif ESP32-C6-DevKitC-1. The full header is broken out, so we pick a clean non-overlapping spread
  // and stay away from USB-JTAG (GPIO12/GPIO13) and the SPI-flash pins. No RF-switch reservation applies.
  use esp_hal::peripherals;
  pub type ServoPanPin = peripherals::GPIO0<'static>; //  pan servo PWM (LEDC ch0).
  pub type ServoTiltPin = peripherals::GPIO1<'static>; //  tilt servo PWM (LEDC ch1).
  pub type LoraSckPin = peripherals::GPIO6<'static>; //  SX1276 SPI clock.
  pub type LoraMosiPin = peripherals::GPIO7<'static>; //  SX1276 SPI MOSI.
  pub type LoraMisoPin = peripherals::GPIO5<'static>; //  SX1276 SPI MISO.
  pub type LoraNssPin = peripherals::GPIO18<'static>; //  SX1276 chip-select (Output).
  pub type LoraResetPin = peripherals::GPIO19<'static>; //  SX1276 RESET (Output).
  pub type LoraDio0Pin = peripherals::GPIO20<'static>; //  SX1276 DIO0 IRQ (Input).
  pub type I2cSdaPin = peripherals::GPIO22<'static>; //  shared I2C SDA.
  pub type I2cSclPin = peripherals::GPIO23<'static>; //  shared I2C SCL.
  pub type GpsTxPin = peripherals::GPIO16<'static>; //  UART TX to the GPS module.
  pub type GpsRxPin = peripherals::GPIO17<'static>; //  UART RX from the GPS module.

  pub const PPM: u8 = 2;
  pub const SERVO_PAN: u8 = 0;
  pub const SERVO_TILT: u8 = 1;
  pub const LORA_SCK: u8 = 6;
  pub const LORA_MOSI: u8 = 7;
  pub const LORA_MISO: u8 = 5;
  pub const LORA_NSS: u8 = 18;
  pub const LORA_RESET: u8 = 19;
  pub const LORA_DIO0: u8 = 20;
  pub const I2C_SDA: u8 = 22;
  pub const I2C_SCL: u8 = 23;
  pub const GPS_TX: u8 = 16;
  pub const GPS_RX: u8 = 17;
}

#[cfg(feature = "board-devkit")]
role_accessors!(GPIO0, GPIO1, GPIO6, GPIO7, GPIO5, GPIO18, GPIO19, GPIO20, GPIO22, GPIO23, GPIO16, GPIO17);

pub use board::*;
