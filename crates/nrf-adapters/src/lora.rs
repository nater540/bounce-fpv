//! Shared SX1276/RFM95W bring-up for the nRF52840 nodes — the embassy-nrf SPIM + GPIO half of the radio that
//! every binary used to copy-paste verbatim (goggle-node, truck-node, lora-ping).
//!
//! The generic [`lora_link::build_sx1276`] is platform-agnostic; what was duplicated is the nRF glue around it:
//! `Spim::new(SPI3, Irqs, ...)`, the NSS/RESET push-pull `Output`s, the DIO0 `Input`, wrapping the bus + NSS in
//! an `embedded-hal-bus` `ExclusiveDevice`, and the `RadioSpi`/`Radio`/`Link` type aliases. Centralizing it here
//! keeps the three binaries from drifting (each had identical comments and pin order, a copy-paste hazard) and —
//! the real win — returns `Result` instead of `.expect()`, so a caller can log the `RadioError` and park rather
//! than panicking on a wiring fault.
//!
//! Generic over the SPIM instance `T: spim::Instance` (the binary picks `SPI3`) and the per-crate `Irqs` binding
//! (`bind_interrupts!` generates a fresh type in each binary), and over the six LoRa pins as `Peri<'static, impl
//! Pin>` so the board pin-map's concrete `Peri<'static, P0_xx>` singletons pass straight through. `Spim<'static>`
//! is type-erased over its instance, so all three callers share the one concrete [`Link`] alias regardless of
//! which SPIM they handed in.
//!
//! Call shape (replaces the inline block + the per-binary `RadioSpi`/`Radio`/`Link` aliases):
//! ```ignore
//! let link: nrf_adapters::lora::Link = match nrf_adapters::lora::build_lora_link(
//!   p.SPI3, Irqs,
//!   pins.lora_sck, pins.lora_mosi, pins.lora_miso, pins.lora_nss, pins.lora_reset, pins.lora_dio0,
//!   TX_BOOST,
//! ).await {
//!   Ok(link) => link,
//!   Err(e) => { applog::log_println!("radio init FAILED ({:?}) — parked", e); loop { /* park */ } }
//! };
//! ```
//! SD coexistence note: `Spim::new` enables the SPIM interrupt at the default P0 (which the SoftDevice
//! reserves), but `applog::init_embassy_nrf()` now lowers the whole SERIAL/SPIM/TWIM/UARTE IRQ family to P2
//! centrally (called first in every `main`), so callers no longer need a per-binary `set_priority` here.

use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pin, Pull};
use embassy_nrf::interrupt::typelevel::Binding;
use embassy_nrf::spim::{self, Spim};
use embassy_nrf::Peri;
use embassy_time::Delay;
use embedded_hal_bus::spi::ExclusiveDevice;
use lora_link::{LoraLink, Sx1276Radio};
use lora_phy::mod_params::RadioError;

/// The radio's SPI device: the type-erased async `Spim` bus + the NSS `Output`, wrapped by `ExclusiveDevice`
/// (which toggles CS around each transfer) with a `Delay` for the inter-transfer wait.
pub type RadioSpi = ExclusiveDevice<Spim<'static>, Output<'static>, Delay>;

/// The concrete SX1276 radio: lora-phy's `Sx127x` (1276 variant) over [`RadioSpi`], with the RESET `Output` and
/// DIO0 `Input` as the interface-variant control/IRQ pins.
pub type Radio = Sx1276Radio<RadioSpi, Output<'static>, Input<'static>>;

/// The ready-to-use LoRa link the nodes hold: a [`Radio`] driven by a `Delay`. Shared by goggle-node,
/// truck-node, and lora-ping so the radio type can never drift between them.
pub type Link = LoraLink<Radio, Delay>;

/// Builds the SX1276/RFM95W [`Link`] from the SPIM peripheral, the binding for its interrupt, and the six LoRa
/// pins, returning the [`RadioError`] instead of panicking so the caller can log-and-park on a wiring fault.
///
/// `spim` is the SPIM instance (e.g. `SPI3`); `irqs` is the binding from the binary's `bind_interrupts!`; the
/// pins are in SPI-then-control order (sck/mosi/miso, then nss/reset/dio0). `tx_boost` drives the RFM95W's
/// PA_BOOST pin — leave it true on a bare module or radiated power is near-zero. Uses `spim::Config::default()`
/// (1 MHz, mode 0 — the SX1276 default), idle-high NSS/RESET, and DIO0 with no pull (the radio drives it).
pub async fn build_lora_link<T, IRQ>(
  spim: Peri<'static, T>,
  irqs: IRQ,
  sck: Peri<'static, impl Pin>,
  mosi: Peri<'static, impl Pin>,
  miso: Peri<'static, impl Pin>,
  nss: Peri<'static, impl Pin>,
  reset: Peri<'static, impl Pin>,
  dio0: Peri<'static, impl Pin>,
  tx_boost: bool,
) -> Result<Link, RadioError>
where
  T: spim::Instance,
  IRQ: Binding<T::Interrupt, spim::InterruptHandler<T>> + 'static,
{
  // NOTE the pin order Spim::new wants: sck, MISO, MOSI (miso before mosi). The arguments above are grouped
  // sck/mosi/miso for the caller's readability, so they are reordered to miso/mosi here on the way in.
  let spi_bus = Spim::new(spim, irqs, sck, miso, mosi, spim::Config::default());
  let nss = Output::new(nss, Level::High, OutputDrive::Standard);
  let reset = Output::new(reset, Level::High, OutputDrive::Standard);
  let dio0 = Input::new(dio0, Pull::None);
  // ExclusiveDevice::new is infallible here (the bus + CS are owned, never shared), but it returns Result; map
  // its error into the radio error path so this helper has a single Result type for callers to match on.
  let spi_dev = ExclusiveDevice::new(spi_bus, nss, Delay).map_err(|_| RadioError::SPI)?;
  lora_link::build_sx1276(spi_dev, reset, dio0, Delay, tx_boost).await
}
