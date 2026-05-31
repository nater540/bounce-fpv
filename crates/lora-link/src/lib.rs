//! LoRa link helpers around lora-phy 3.0.1 for the SX1276/RFM95W, tuned for low-latency P2P head tracking.
//!
//! The pan/tilt packet is a few protobuf bytes sent ~tens of times a second, so we fix the fastest sane
//! modulation — SF7 / BW 500 kHz / CR 4/5 at 915 MHz (US ISM), short preamble, CRC on — which minimizes air
//! time. [`LoraLink`] wraps lora-phy's `LoRa<RK, DLY>` and pre-builds the modulation + packet params once, so
//! the hot path is just [`send`](LoraLink::send) / [`receive`](LoraLink::receive). It stays generic over the
//! lora-phy `RadioKind` and `DelayNs`, so it compiles standalone; the node supplies the concrete radio,
//! either by hand or via [`build_sx1276`], which follows the doc's verbatim SX1276 construction.
//!
//! Wiring (RFM95W -> C6): SPI SCK/MOSI/MISO + NSS (Output, wrap SPI+CS in `embedded-hal-bus` `ExclusiveDevice`)
//! + RESET (Output) + DIO0 (IRQ Input). A bare RFM95W has no TCXO (`tcxo_used: false`); enable `tx_boost` for
//! the PA_BOOST pin if you need full output power.

#![no_std]

use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::spi::SpiDevice;
use lora_phy::iv::GenericSx127xInterfaceVariant;
use lora_phy::mod_params::{
  Bandwidth, CodingRate, ModulationParams, PacketParams, PacketStatus, RadioError, SpreadingFactor,
};
use lora_phy::mod_traits::RadioKind;
use lora_phy::sx127x::{self, Sx1276, Sx127x};
use lora_phy::{LoRa, RxMode};

/// Operating frequency in Hz. 915 MHz is the US ISM band; change for other regions.
pub const FREQUENCY_HZ: u32 = 915_000_000;

/// Preamble length in symbols. A tiny packet needs no long preamble; both ends must use the same value.
pub const PREAMBLE_LEN: u16 = 8;

/// Largest payload, in bytes, [`receive`] will accept. The pan/tilt protobuf is only a handful of bytes; 64
/// leaves room for added telemetry fields without re-tuning.
pub const MAX_PAYLOAD: u8 = 64;

/// A configured LoRa P2P link. Holds the radio plus the pre-built modulation and TX/RX packet params so the
/// send/receive path does no per-call setup. Construct with [`LoraLink::new`] from any lora-phy `LoRa`.
pub struct LoraLink<RK, DLY>
where
  RK: RadioKind,
  DLY: DelayNs,
{
  lora: LoRa<RK, DLY>,
  modulation: ModulationParams,
  tx_params: PacketParams,
  rx_params: PacketParams,
}

impl<RK, DLY> LoraLink<RK, DLY>
where
  RK: RadioKind,
  DLY: DelayNs,
{
  /// Builds the link over an initialized lora-phy radio, fixing SF7/BW500/CR4-5 @ 915 MHz with CRC on and an
  /// explicit header. Fails only if the radio rejects the modulation/packet parameters.
  pub fn new(mut lora: LoRa<RK, DLY>) -> Result<Self, RadioError> {
    let modulation =
      lora.create_modulation_params(SpreadingFactor::_7, Bandwidth::_500KHz, CodingRate::_4_5, FREQUENCY_HZ)?;
    // TX uses an explicit header, CRC on, no IQ inversion; RX mirrors it and sets the max payload it will accept.
    let tx_params = lora.create_tx_packet_params(PREAMBLE_LEN, false, true, false, &modulation)?;
    let rx_params = lora.create_rx_packet_params(PREAMBLE_LEN, false, MAX_PAYLOAD, true, false, &modulation)?;
    Ok(Self { lora, modulation, tx_params, rx_params })
  }

  /// Transmits one packet at `output_power` dBm. In 3.0.1 `prepare_for_tx` takes the power and payload
  /// directly (no separate boost arg — PA boost is set via the SX1276 `Config.tx_boost`). Blocks until the
  /// TX-done IRQ on DIO0 fires.
  pub async fn send(&mut self, payload: &[u8], output_power: i32) -> Result<(), RadioError> {
    self.lora.prepare_for_tx(&self.modulation, &mut self.tx_params, output_power, payload).await?;
    self.lora.tx().await
  }

  /// Listens (continuous mode) and returns the number of payload bytes written into `buf` once a packet
  /// arrives, driven by the DIO0 IRQ. `buf` should be at least [`MAX_PAYLOAD`] long. The `PacketStatus`
  /// (RSSI/SNR) is available via [`receive_with_status`] if you need it.
  pub async fn receive(&mut self, buf: &mut [u8]) -> Result<usize, RadioError> {
    let (len, _status) = self.receive_with_status(buf).await?;
    Ok(len)
  }

  /// Like [`receive`], but also returns the `PacketStatus` (RSSI/SNR) for link-quality reporting.
  pub async fn receive_with_status(&mut self, buf: &mut [u8]) -> Result<(usize, PacketStatus), RadioError> {
    self.lora.prepare_for_rx(RxMode::Continuous, &self.modulation, &self.rx_params).await?;
    let (len, status) = self.lora.rx(&self.rx_params, buf).await?;
    Ok((len as usize, status))
  }

  /// Puts the radio to sleep (`warm_start` keeps configuration for a fast wake). Useful between bursts.
  pub async fn sleep(&mut self, warm_start: bool) -> Result<(), RadioError> {
    self.lora.sleep(warm_start).await
  }

  /// Borrows the underlying lora-phy radio for advanced use (e.g. custom modulation experiments).
  pub fn radio(&mut self) -> &mut LoRa<RK, DLY> {
    &mut self.lora
  }
}

/// Type of the concrete SX1276 radio this crate constructs: lora-phy's `Sx127x` parameterized for the 1276
/// variant over the caller's SPI device and interface-variant pins. Exposed so the node can name the
/// `LoraLink` it gets back from [`build_sx1276`].
pub type Sx1276Radio<SPI, CTRL, WAIT> = Sx127x<SPI, GenericSx127xInterfaceVariant<CTRL, WAIT>, Sx1276>;

/// Builds and initializes an SX1276/RFM95W `LoRa` from its bus + pins, following the doc's verbatim pattern,
/// then wraps it in a ready-to-use [`LoraLink`]. `spi` is the radio's SPI device (SPI bus + NSS, e.g. an
/// `embedded-hal-bus` `ExclusiveDevice`); `reset` is the RESET Output; `dio0` is the DIO0 IRQ Input; `delay`
/// satisfies `DelayNs`. `tx_boost` drives the RFM95W PA_BOOST pin for full output.
///
/// The two antenna-switch options are `None` for a plain module. Returns the interface-variant or radio-init
/// error from lora-phy if construction fails.
pub async fn build_sx1276<SPI, CTRL, WAIT, DLY>(
  spi: SPI,
  reset: CTRL,
  dio0: WAIT,
  delay: DLY,
  tx_boost: bool,
) -> Result<LoraLink<Sx1276Radio<SPI, CTRL, WAIT>, DLY>, RadioError>
where
  SPI: SpiDevice<u8>,
  CTRL: OutputPin,
  WAIT: Wait,
  DLY: DelayNs,
{
  // Bare RFM95W: no TCXO, no RX boost; PA boost (tx_boost) is the only optional output-stage tweak.
  let config = sx127x::Config { chip: Sx1276, tcxo_used: false, tx_boost, rx_boost: false };
  let iv = GenericSx127xInterfaceVariant::new(reset, dio0, None, None)?;
  // `false` = P2P, not a public LoRaWAN network.
  let lora = LoRa::new(Sx127x::new(spi, iv, config), false, delay).await?;
  LoraLink::new(lora)
}
