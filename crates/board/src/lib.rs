//! Board pin-map for the FPV head-tracking system on the **Nice!Nano v2** (nRF52840). This crate owns the
//! single source of truth for every GPIO assignment, so the node binaries never hard-code pin numbers. It is
//! the one driver-side crate that deliberately touches `embassy-nrf` concrete pin types: the `board_pins!`
//! macro partial-moves the named pins out of the caller's own `embassy_nrf::Peripherals` into a `BoardPins`
//! struct, leaving the controller singletons (SPIM/TWIM/UARTE/PWM instances, GPIOTE, ...) owned by and usable
//! through the caller afterward — exactly as the prior esp-hal version split pins out of its `Peripherals`.
//!
//! Today there is one variant (Nice!Nano v2), kept as the default `mod board`. The role-accessor macro
//! pattern is preserved so a second board can later be added behind a Cargo feature picking a different
//! `mod board` + `role_accessors!` invocation, without changing `BoardPins` or `board_pins!`.
//!
//! ## nRF SERIAL/peripheral instance guidance for the binaries
//!
//! This crate hands out *pins* only; the binaries pick the controller singletons. On the nRF52840 the
//! SPIM/TWIM/UARTE controllers alias shared SERIAL hardware blocks, so distinct subsystems MUST pick distinct
//! instances or they silently collide:
//! - `UARTE0`/`TWISPI0` are SERIAL0; `UARTE1`/`TWISPI1` are SERIAL1; `SPI2`/`SPI3` are SPI-only SERIAL2/3.
//! - Suggested non-colliding assignment for the truck node (the busiest): LoRa SPI -> `SPI3`, shared I2C ->
//!   `TWISPI0` (a TWIM instance), GPS UART -> `UARTE1`. That leaves `UARTE0`/`TWISPI1`/`SPI2` free.
//! - Servos use the PWM peripheral, which is independent of the SERIAL blocks: pan/tilt as two channels of one
//!   `PWM0` `SimplePwm` (a single 50 Hz frame drives both), so `PWM1`/`PWM2`/`PWM3` stay free.
//! - PPM input and the lap-reset button are plain GPIO edge waits driven by `GPIOTE`.
//!
//! ## Reserved pins (never assigned here)
//!
//! Per `docs/01-nrf52840-migration.md` and the Nice!Nano v2 pinout: **P0.15** on-board LED (active-high),
//! **P0.00/P0.01** the 32.768 kHz LFXO crystal, **P0.04** battery-voltage divider, **P0.13** VCC power
//! control, and the on-board QSPI flash pins (**P0.19/P0.21/P0.23** plus its other IO lines, not broken out).
//! `RESET` is dedicated. This pin-map only assigns freely-exposed castellated GPIO outside that set.

#![no_std]

/// Builds `BoardPins` by partial-moving the role pins out of the caller's own `Peripherals` binding. Invoke
/// once at boot, after `embassy_nrf::init`: `let pins = board::board_pins!(p);`. The macro reads `p.P0_xx`
/// fields in place — it does NOT take ownership of the whole struct — so the remaining peripheral singletons
/// (SPI3/TWISPI0/UARTE1/PWM0/GPIOTE/...) stay owned by and accessible through the caller's `p` variable
/// afterward. `embassy_nrf::Peripherals` is a plain field struct with no `Drop`, so partial moves of just the
/// named pin fields are allowed; this needs no `unsafe`/`steal()` and mirrors the prior esp-hal pin split.
///
/// The argument must be an addressable place expression (a `let` binding, not a temporary): each field is
/// moved out of it by value, so the rest of that same binding remains usable after the call.
#[macro_export]
macro_rules! board_pins {
  ($peripherals:expr) => {
    $crate::BoardPins {
      ppm: $crate::__ppm!($peripherals),
      button: $crate::__button!($peripherals),
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
      gps_rx: $crate::__gps_rx!($peripherals),
      gps_tx: $crate::__gps_tx!($peripherals),
    }
  };
}

// One macro per role so the active board picks the exact `Peripherals` field by name. These are crate
// internals (the leading `__`); only `board_pins!` and `BoardPins` are meant to be used directly. Adding a
// second board means another `mod board` + `role_accessors!` pair behind a `#[cfg(feature = ...)]`.
macro_rules! role_accessors {
  (
    $ppm:ident, $button:ident,
    $servo_pan:ident, $servo_tilt:ident,
    $lora_sck:ident, $lora_mosi:ident, $lora_miso:ident, $lora_nss:ident, $lora_reset:ident, $lora_dio0:ident,
    $i2c_sda:ident, $i2c_scl:ident,
    $gps_rx:ident, $gps_tx:ident $(,)?
  ) => {
    #[doc(hidden)] #[macro_export] macro_rules! __ppm         { ($p:expr) => { $p.$ppm }; }
    #[doc(hidden)] #[macro_export] macro_rules! __button      { ($p:expr) => { $p.$button }; }
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
    #[doc(hidden)] #[macro_export] macro_rules! __gps_rx      { ($p:expr) => { $p.$gps_rx }; }
    #[doc(hidden)] #[macro_export] macro_rules! __gps_tx      { ($p:expr) => { $p.$gps_tx }; }
  };
}

/// The complete set of pins this firmware uses, handed out once at boot by `board_pins!`. Fields are grouped
/// by subsystem; each is a concrete `embassy_nrf::peripherals::Pn_xx` singleton (wrapped in `Peri<'static>`)
/// the binary passes straight into an `Input`/`Output`, an `Spim`/`Twim`/`Uarte` builder, or a `SimplePwm`
/// channel. The controller singletons (SPI3/TWISPI0/UARTE1/PWM0/...) stay with the binary — only the pin
/// routing lives here. nRF routes any function to any GPIO, so these assignments are free choices within the
/// freely-exposed Nice!Nano v2 castellated pads (reserved pins are listed in the crate docs).
pub struct BoardPins {
  /// PPM input from the goggles' HT OUT jack (goggle node). Drive as `Input` + a GPIOTE edge wait.
  pub ppm: PpmPin,
  /// Lap-reset push button (truck node). Active-low to GND with internal pull-up; GPIOTE edge wait.
  pub button: ButtonPin,
  /// Servo PWM outputs (truck node): pan / tilt, two channels of one `SimplePwm` on a shared 50 Hz frame.
  /// Drive the MG90S from a separate 5 V rail with common ground, never the 3.3 V pin.
  pub servo_pan: ServoPanPin,
  pub servo_tilt: ServoTiltPin,
  /// LoRa SX1276/RFM95W SPI bus + control lines. SCK/MOSI/MISO go to an `Spim` instance (suggest `SPI3`); NSS
  /// is an Output wrapped with `embedded-hal-bus` `ExclusiveDevice`; RESET is an Output; DIO0 is the IRQ Input
  /// the RX/TX futures await on via GPIOTE.
  pub lora_sck: LoraSckPin,
  pub lora_mosi: LoraMosiPin,
  pub lora_miso: LoraMisoPin,
  pub lora_nss: LoraNssPin,
  pub lora_reset: LoraResetPin,
  pub lora_dio0: LoraDio0Pin,
  /// Shared async I2C bus (truck node): MPU-6050 (0x68) + SSD1306 (0x3C) behind one `shared_bus` Mutex on a
  /// `Twim` instance (suggest `TWISPI0`). These two pads are the board's native I2C (SDA P0.17 / SCL P0.20).
  pub i2c_sda: I2cSdaPin,
  pub i2c_scl: I2cSclPin,
  /// GPS UART (bonus) on a `Uarte` instance (suggest `UARTE1`): RX carries the module's NMEA stream into the
  /// nRF; TX is wired for completeness / config but the reader is RX-only.
  pub gps_rx: GpsRxPin,
  pub gps_tx: GpsTxPin,
}

mod board {
  // Nice!Nano v2 (nRF52840). Pin numbers verified against the official / CircuitPython `nice_nano` board
  // definition and `docs/01-nrf52840-migration.md`. All roles sit on freely-exposed castellated GPIO; the
  // reserved pins (LED P0.15, LFXO P0.00/P0.01, battery P0.04, VCC_OFF P0.13, QSPI flash P0.19/P0.21/P0.23)
  // are never assigned. SCK/MOSI/MISO and SDA/SCL reuse the board's labelled bus pads where it costs nothing.
  use embassy_nrf::Peri;
  use embassy_nrf::peripherals;
  // Each role is a `Peri<'static, Pn_xx>` — the exact ownership-token type `embassy_nrf::Peripherals` stores,
  // so `board_pins!` moves the field out with its type unchanged and the binary passes it straight to a
  // driver builder. nRF routes any function to any GPIO, so the pad choice is free within the exposed set.
  pub type PpmPin = Peri<'static, peripherals::P0_02>; //  PPM in: A0/AIN0 pad, used as a plain digital edge input.
  pub type ButtonPin = Peri<'static, peripherals::P1_00>; //  lap-reset button (D6 pad), active-low + pull-up.
  pub type ServoPanPin = Peri<'static, peripherals::P0_22>; //  pan servo PWM (PWM0 ch0), 5 V-rail, common GND.
  pub type ServoTiltPin = Peri<'static, peripherals::P0_24>; //  tilt servo PWM (PWM0 ch1), same 50 Hz frame.
  pub type LoraSckPin = Peri<'static, peripherals::P1_13>; //  SX1276 SPI clock (board SCK pad).
  pub type LoraMosiPin = Peri<'static, peripherals::P0_11>; //  SX1276 SPI MOSI (D7 pad). NOT the board MOSI pad
  //  P0.10 — that is NFC2 and only exists as GPIO under embassy-nrf's `nfc-pins-as-gpio` feature (not enabled).
  pub type LoraMisoPin = Peri<'static, peripherals::P1_11>; //  SX1276 SPI MISO (board MISO pad).
  pub type LoraNssPin = Peri<'static, peripherals::P0_29>; //  SX1276 chip-select (Output, ExclusiveDevice; D20 pad).
  //  P0.12 is NOT broken out on the Nice!Nano v2 (it is absent from the official pinout), so NSS uses the free D20 pad.
  pub type LoraResetPin = Peri<'static, peripherals::P0_06>; //  SX1276 RESET (Output, active-low; TX pad).
  pub type LoraDio0Pin = Peri<'static, peripherals::P0_08>; //  SX1276 DIO0 IRQ (Input via GPIOTE).
  pub type I2cSdaPin = Peri<'static, peripherals::P0_17>; //  shared I2C SDA (board SDA pad): MPU-6050 + SSD1306.
  pub type I2cSclPin = Peri<'static, peripherals::P0_20>; //  shared I2C SCL (board SCL pad).
  pub type GpsRxPin = Peri<'static, peripherals::P1_04>; //  UART RX: the GPS module's NMEA stream into the nRF.
  pub type GpsTxPin = Peri<'static, peripherals::P1_06>; //  UART TX to the GPS module (config only; RX matters).

  // Role -> port/pin constants for self-documenting diagnostic banners (the typed pins above carry the real
  // binding). `PORT` is 0 or 1; `PIN` is the within-port number, so the pad is `P{PORT}.{PIN:02}`.
  pub const PPM_PORT: u8 = 0;
  pub const PPM_PIN: u8 = 2;
  pub const BUTTON_PORT: u8 = 1;
  pub const BUTTON_PIN: u8 = 0;
  pub const SERVO_PAN_PORT: u8 = 0;
  pub const SERVO_PAN_PIN: u8 = 22;
  pub const SERVO_TILT_PORT: u8 = 0;
  pub const SERVO_TILT_PIN: u8 = 24;
  pub const LORA_SCK_PORT: u8 = 1;
  pub const LORA_SCK_PIN: u8 = 13;
  pub const LORA_MOSI_PORT: u8 = 0;
  pub const LORA_MOSI_PIN: u8 = 11;
  pub const LORA_MISO_PORT: u8 = 1;
  pub const LORA_MISO_PIN: u8 = 11;
  pub const LORA_NSS_PORT: u8 = 0;
  pub const LORA_NSS_PIN: u8 = 29;
  pub const LORA_RESET_PORT: u8 = 0;
  pub const LORA_RESET_PIN: u8 = 6;
  pub const LORA_DIO0_PORT: u8 = 0;
  pub const LORA_DIO0_PIN: u8 = 8;
  pub const I2C_SDA_PORT: u8 = 0;
  pub const I2C_SDA_PIN: u8 = 17;
  pub const I2C_SCL_PORT: u8 = 0;
  pub const I2C_SCL_PIN: u8 = 20;
  pub const GPS_RX_PORT: u8 = 1;
  pub const GPS_RX_PIN: u8 = 4;
  pub const GPS_TX_PORT: u8 = 1;
  pub const GPS_TX_PIN: u8 = 6;
}

role_accessors!(
  P0_02, P1_00, P0_22, P0_24, P1_13, P0_11, P1_11, P0_29, P0_06, P0_08, P0_17, P0_20, P1_04, P1_06,
);

pub use board::*;
