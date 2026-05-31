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

/// Alternate SSD1306 address for panels that strap SA0 high. [`probe_address`] checks this in addition to [`ADDR`].
pub const ADDR_ALT: u8 = 0x3D;

/// Probes the two standard SSD1306 addresses (0x3C then 0x3D) on `i2c` and returns the first that ACKs, or `None`
/// if neither answers. A `None` means the bus itself is not talking — wrong SDA/SCL pins, a swapped pair, no/weak
/// pull-ups, or no panel — which a dark screen alone cannot distinguish from a panel found-but-failed-to-init. The
/// probe is a 1-byte write of the SSD1306 command control byte (0x00) with no command following, so it is a no-op on
/// a real panel. Borrows the bus, so the caller still owns it to hand to [`StatusDisplay::new_with_addr`] afterward.
pub async fn probe_address<I: I2c>(i2c: &mut I) -> Option<u8> {
  for &addr in &[ADDR, ADDR_ALT] {
    if i2c.write(addr, &[0x00]).await.is_ok() {
      return Some(addr);
    }
  }
  None
}

/// The concrete buffered async display type this crate drives: a 128x64 SSD1306 over an I2C interface in
/// buffered-graphics mode. Named so the binary can hold one if it wants direct embedded-graphics access.
pub type OledDisplay<I> =
  Ssd1306Async<I2CInterface<I>, DisplaySize128x64, BufferedGraphicsModeAsync<DisplaySize128x64>>;

/// The status to render: ground speed (cm/s, matching the telemetry unit), whether the LoRa link is live,
/// whether the GPS has a fix, and the lap stopwatch value in whole seconds since the last reset. Kept small
/// and `Copy` so the render task can pull it from a `Signal`.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct Status {
  pub speed_cm_s: u32,
  pub link_up: bool,
  pub gps_fix: bool,
  // Whole seconds since the last lap reset. Only changes once per second, so the dirty-check below lets the
  // panel re-flush at ~1 Hz while the lap line is the only thing moving — well within the bus budget.
  pub lap_secs: u32,
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

/// Top inset (px) applied to the first text line. Nudges everything off the very top row, which on the common
/// dual-colour panels is the yellow band — so the title doesn't read as clipped against the edge. The layouts below
/// still fit 64 px with this offset (4-line `render`: last baseline 51, glyph bottom 61; 5-line `render_lines`: 51/61).
const TOP_MARGIN: i32 = 3;

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
    Self::new_with_addr(i2c, ADDR).await
  }

  /// Builds the display at an EXPLICIT I2C address ([`ADDR`] 0x3C or [`ADDR_ALT`] 0x3D, per the panel's SA0 strap)
  /// and runs `init().await`. Pair with [`probe_address`] to pick the address a given panel actually answers on,
  /// rather than guessing — a wrong address NAKs every transfer and leaves the panel dark.
  pub async fn new_with_addr(i2c: I, addr: u8) -> Result<Self, display_interface::DisplayError> {
    let interface = I2CDisplayInterface::new_custom_address(i2c, addr);
    let mut display =
      Ssd1306Async::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_buffered_graphics_mode();
    display.init().await?;
    Ok(Self { display, last: None })
  }

  /// Clears the buffer, draws the status lines, and flushes to the panel over I2C. Four lines: a title, the
  /// link/fix indicators, the speed in km/h, and the lap timer in seconds. Drawing into the buffer is
  /// infallible; only `flush` can fail.
  ///
  /// Dirty-checked: if `status` equals the last rendered status, this returns immediately without clearing,
  /// redrawing, or flushing — so a fixed-rate render loop only touches the I2C bus when something changed.
  pub async fn render(&mut self, status: Status) -> Result<(), display_interface::DisplayError> {
    if self.last == Some(status) {
      return Ok(());
    }

    self.display.clear_buffer();

    // Line 1: title. Line 2: link + fix flags. Line 3: speed in km/h. Line 4: lap timer in seconds. `unwrap`
    // on draw is safe — the buffered target's error type is `Infallible`.
    Text::with_baseline("FPV HEAD TRACK", Point::new(0, TOP_MARGIN), TEXT_STYLE, Baseline::Top).draw(&mut self.display).unwrap();

    let mut flags: String<24> = String::new();
    let _ = write!(flags, "LINK:{} FIX:{}", yn(status.link_up), yn(status.gps_fix));
    Text::with_baseline(&flags, Point::new(0, 16 + TOP_MARGIN), TEXT_STYLE, Baseline::Top).draw(&mut self.display).unwrap();

    let mut speed: String<24> = String::new();
    let _ = write!(speed, "SPD: {} km/h", cm_s_to_kmh(status.speed_cm_s));
    Text::with_baseline(&speed, Point::new(0, 32 + TOP_MARGIN), TEXT_STYLE, Baseline::Top).draw(&mut self.display).unwrap();

    let mut lap: String<24> = String::new();
    let _ = write!(lap, "LAP: {}s", status.lap_secs);
    Text::with_baseline(&lap, Point::new(0, 48 + TOP_MARGIN), TEXT_STYLE, Baseline::Top).draw(&mut self.display).unwrap();

    // Record the rendered status only after a successful flush, so a failed flush re-renders next call.
    self.display.flush().await?;
    self.last = Some(status);
    Ok(())
  }

  /// Renders up to ~5 short lines of text top-to-bottom at a fixed 12 px pitch and flushes — a general-purpose
  /// diagnostic counterpart to [`render`](StatusDisplay::render)'s fixed `Status` layout, used by the link-test
  /// bins to show RTT/RSSI/seq without inventing a struct. NOT dirty-checked (the caller owns cadence: drive it at
  /// a few Hz, never inside a latency-critical window). Lines beyond the ~5 that fit a 64 px panel are clipped.
  pub async fn render_lines(&mut self, lines: &[&str]) -> Result<(), display_interface::DisplayError> {
    self.display.clear_buffer();
    // 12 px pitch: FONT_6X10 is 10 px tall, so five lines (y = 0,12,24,36,48; last glyph bottom 58) clear the 64 px
    // panel with room to spare. `draw` into the buffered target is infallible (error type Infallible); only flush fails.
    for (i, line) in lines.iter().enumerate() {
      Text::with_baseline(line, Point::new(0, TOP_MARGIN + i as i32 * 12), TEXT_STYLE, Baseline::Top).draw(&mut self.display).unwrap();
    }
    // Invalidate the Status dirty-cache: a later render() must redraw, since the panel now shows free-form lines.
    self.last = None;
    self.display.flush().await
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

  #[test]
  fn status_default_zeroes_lap_secs() {
    // The lap timer reads zero at boot/reset, before the first tick has elapsed.
    assert_eq!(Status::default().lap_secs, 0);
  }

  #[test]
  fn lap_secs_participates_in_dirty_check() {
    // The dirty-check is `self.last == Some(status)`, so a changed lap_secs must make two Statuses unequal —
    // otherwise the once-per-second lap tick would never re-flush the panel. Conversely an unchanged lap_secs
    // (everything equal) must compare equal so the render is skipped.
    let base = Status { speed_cm_s: 1000, link_up: true, gps_fix: true, lap_secs: 5 };
    assert_eq!(base, base);
    assert_ne!(base, Status { lap_secs: 6, ..base });
  }
}
