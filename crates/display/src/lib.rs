//! SSD1306 128x64 OLED status renderer over the async ssd1306 driver + embedded-graphics.
//!
//! On the truck this shows a quick glance of link/fix state and ground speed. It is built for the shared
//! async I2C bus: [`StatusDisplay::new`] takes any `embedded_hal_async::i2c::I2c` (e.g. an
//! `embassy-embedded-hal` `I2cDevice`) and wraps a buffered-graphics `Ssd1306Async` at the standard 0x3C
//! address. [`render`](StatusDisplay::render) draws a [`Status`] with a 6x10 mono font and flushes over I2C.
//!
//! Speed arrives in centimeters/second (the `Telemetry.speed_cm_s` unit) and is shown as whole km/h, computed
//! with integer math (`km/h = (cm_s * 36 + 500) / 1000`, rounded to nearest) so the display crate pulls in no
//! floating point.

#![no_std]

use core::fmt::Write as _;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::{MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::text::{Baseline, Text};
use embedded_hal_async::i2c::I2c;
use heapless::String;
use ssd1306::mode::{BufferedGraphicsModeAsync, DisplayConfigAsync};
use ssd1306::prelude::{DisplayRotation, I2CInterface};
use ssd1306::size::DisplaySize128x64;
use ssd1306::{I2CDisplayInterface, Ssd1306Async};

/// Default SSD1306 I2C address. Some panels strap to 0x3D; this crate uses the common 0x3C.
pub const ADDR: u8 = 0x3C;

/// The concrete buffered async display type this crate drives: a 128x64 SSD1306 over an I2C interface in
/// buffered-graphics mode. Named so the binary can hold one if it wants direct embedded-graphics access.
pub type OledDisplay<I> =
  Ssd1306Async<I2CInterface<I>, DisplaySize128x64, BufferedGraphicsModeAsync<DisplaySize128x64>>;

/// The status to render: ground speed (cm/s, matching the telemetry unit), whether the LoRa link is live,
/// and whether the GPS has a fix. Kept small and `Copy` so the render task can pull it from a `Signal`.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct Status {
  pub speed_cm_s: u32,
  pub link_up: bool,
  pub gps_fix: bool,
}

/// Converts centimeters/second to whole km/h with integer rounding. 1 cm/s = 0.036 km/h, so km/h =
/// (cm_s * 36 + 500) / 1000 (the +500 rounds to nearest). The multiply is done in u64 so the intermediate
/// `cm_s * 36` cannot overflow u32 (it would for cm_s > ~119M, wrapping silently in the release profile).
pub fn cm_s_to_kmh(cm_s: u32) -> u32 {
  ((cm_s as u64 * 36 + 500) / 1000) as u32
}

/// The mono text style used for every status line. Built once as a `const` rather than rebuilt per `render`
/// (the builder does real work each call), since the truck drives `render` at 50-100 Hz on the shared bus.
const TEXT_STYLE: MonoTextStyle<'static, BinaryColor> =
  MonoTextStyleBuilder::new().font(&FONT_6X10).text_color(BinaryColor::On).build();

/// A ready-to-use status OLED wrapping a buffered async SSD1306 on the shared I2C bus.
pub struct StatusDisplay<I: I2c> {
  display: OledDisplay<I>,
  // The last status actually drawn/flushed, used to skip the clear+draw+flush when nothing changed. `None`
  // before the first render forces an initial draw regardless of the incoming status.
  last: Option<Status>,
}

impl<I: I2c> StatusDisplay<I> {
  /// Builds the display over the given async I2C bus at the default 0x3C address and runs `init().await`.
  /// Returns the display-interface error if initialization fails.
  pub async fn new(i2c: I) -> Result<Self, display_interface::DisplayError> {
    let interface = I2CDisplayInterface::new(i2c);
    let mut display =
      Ssd1306Async::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_buffered_graphics_mode();
    display.init().await?;
    Ok(Self { display, last: None })
  }

  /// Clears the buffer, draws the status lines, and flushes to the panel over I2C. Three lines: a title, the
  /// link/fix indicators, and the speed in km/h. Drawing into the buffer is infallible; only `flush` can fail.
  ///
  /// Dirty-checked: if `status` equals the last rendered status, this returns immediately without clearing,
  /// redrawing, or flushing — so a fixed-rate render loop only touches the I2C bus when something changed.
  pub async fn render(&mut self, status: Status) -> Result<(), display_interface::DisplayError> {
    if self.last == Some(status) {
      return Ok(());
    }

    self.display.clear_buffer();

    // Line 1: title. Line 2: link + fix flags. Line 3: speed in km/h. `unwrap` on draw is safe — the buffered
    // target's error type is `Infallible`.
    Text::with_baseline("FPV HEAD TRACK", Point::zero(), TEXT_STYLE, Baseline::Top).draw(&mut self.display).unwrap();

    let mut flags: String<24> = String::new();
    let _ = write!(flags, "LINK:{} FIX:{}", yn(status.link_up), yn(status.gps_fix));
    Text::with_baseline(&flags, Point::new(0, 16), TEXT_STYLE, Baseline::Top).draw(&mut self.display).unwrap();

    let mut speed: String<24> = String::new();
    let _ = write!(speed, "SPD: {} km/h", cm_s_to_kmh(status.speed_cm_s));
    Text::with_baseline(&speed, Point::new(0, 32), TEXT_STYLE, Baseline::Top).draw(&mut self.display).unwrap();

    // Record the rendered status only after a successful flush, so a failed flush re-renders next call.
    self.display.flush().await?;
    self.last = Some(status);
    Ok(())
  }

  /// Borrows the underlying buffered display for direct embedded-graphics drawing.
  pub fn raw(&mut self) -> &mut OledDisplay<I> {
    &mut self.display
  }
}

/// Renders a flag as "Y"/"N" for the compact status line.
fn yn(b: bool) -> &'static str {
  if b { "Y" } else { "N" }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn cm_s_to_kmh_rounds_to_nearest() {
    assert_eq!(cm_s_to_kmh(0), 0);
    assert_eq!(cm_s_to_kmh(1000), 36); //  1000 cm/s = 36.0 km/h.
    assert_eq!(cm_s_to_kmh(278), 10); //  277.78 cm/s ~= 10 km/h, rounds up.
    assert_eq!(cm_s_to_kmh(13), 0); //  0.468 km/h rounds down to 0.
    assert_eq!(cm_s_to_kmh(14), 1); //  0.504 km/h rounds up to 1.
  }

  #[test]
  fn cm_s_to_kmh_does_not_overflow_at_u32_max() {
    // cm_s * 36 overflows u32 for cm_s > ~119M; the u64 intermediate must keep this from wrapping/panicking.
    let kmh = cm_s_to_kmh(u32::MAX);
    assert_eq!(kmh, ((u32::MAX as u64 * 36 + 500) / 1000) as u32);
    assert_eq!(kmh, 154_618_823);
  }
}
