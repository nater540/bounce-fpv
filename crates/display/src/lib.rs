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

// no_std for the firmware build; under `cargo test` we link std so the libtest harness and the host unit tests
// (pure integer helpers + dirty-check equality) compile — the embedded image is unaffected. The workspace
// `.cargo/config.toml` forces `target = thumbv7em-none-eabihf` with `-Tlink.x` linker args, so a bare
// `cargo test` targets the chip (no std/test crate) and fails; run the host tests with BOTH a host target and
// the linker flags cleared: `RUSTFLAGS="" cargo test -p display --target aarch64-apple-darwin`.
#![cfg_attr(not(test), no_std)]

use core::fmt::Write as _;
use embedded_graphics::geometry::Size;
use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_6X10};
use embedded_graphics::mono_font::{MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Line, PrimitiveStyle, Rectangle};
use embedded_graphics::text::{Alignment, Baseline, Text, TextStyle, TextStyleBuilder};
use embedded_graphics::Pixel;
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

// The big readout face for the G1 lap clock and the T1 speed number. 10x20 is the largest built-in mono font;
// there is no bold 10x20, but at this cell size it already reads heavy on a 128x64 panel. The committed design's
// "24px Silkscreen" cap height lands close to 20 px, and a 10x20 glyph whose top sits at y26 ends at y45 — clear
// of the G1 divider (y50) and the T1 "MPH" label (y47). The handoff note explicitly invites this GFX-font swap.
const BIG_STYLE: MonoTextStyle<'static, BinaryColor> =
  MonoTextStyleBuilder::new().font(&FONT_10X20).text_color(BinaryColor::On).build();

// Mono advance width of the big font, used to center/measure text without a runtime measure call: the rendered
// width of a mono string is chars * cell width. Only the big face needs measuring (for the G1 trailing-"s" offset).
const BIG_W: i32 = 10; // FONT_10X20 advance

// Top-baseline text styles for the three horizontal anchors the layouts use. Baseline::Top means the supplied y is
// the glyph-cell top row (matching the design canvas's textBaseline='top'); Center/Right anchor the box on x.
const TS_TOP_LEFT: TextStyle = TextStyleBuilder::new().alignment(Alignment::Left).baseline(Baseline::Top).build();
const TS_TOP_CENTER: TextStyle = TextStyleBuilder::new().alignment(Alignment::Center).baseline(Baseline::Top).build();
const TS_TOP_RIGHT: TextStyle = TextStyleBuilder::new().alignment(Alignment::Right).baseline(Baseline::Top).build();

// A solid white fill for the rectangle-based primitives (hline/vline/filled bars). Built once as a const.
const FILL_ON: PrimitiveStyle<BinaryColor> = PrimitiveStyle::with_fill(BinaryColor::On);

/// Full-scale speed for the T1 speedo arc, in mph (the design's `SPEED_MAX`). The arc fills 0..1 across this range.
pub const SPEED_MAX_MPH: u32 = 60;

/// Status for the goggle node's G1 "Stopwatch" layout. All integer/bool so it derives `Eq` for the dirty-check and
/// the render path stays float-free (the arc's trig lives only in the truck layout). `Copy` so the OLED task pulls
/// it together each tick and the dirty-check compares two by value.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct GoggleStatus {
  pub lap: u32,        // lap number, shown as "LAP nn"
  pub lap_tenths: u32, // lap time in tenths of a second, rendered "SS.s"
  pub pan_deg: i16,    // head pan in degrees, -45..45, shown "PAN +dd"
  pub tilt_deg: i16,   // head tilt in degrees, -30..30, shown "+dd TLT"
  pub sats: u32,       // GPS satellites in view (relayed from the truck), shown "SVnn"
  pub bars: u8,        // LoRa signal bars, 0..4
}

/// Status for the truck node's T1 "Arc / Center" layout. Integer/bool throughout (mph drives the arc fraction as a
/// float computed transiently inside the renderer), so it derives `Eq` for the dirty-check and is `Copy`.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct TruckStatus {
  pub mph: u32,      // ground speed in whole mph, the big center readout + arc fraction
  pub kmh: u32,      // ground speed in whole km/h, shown "kk KMH"
  pub bars: u8,      // LoRa signal bars, 0..4
  pub rssi: i16,     // last RSSI in dBm, shown as the number when linked, "--" when not
  pub linked: bool,  // whether the link is live (gates the RSSI number)
}

/// Converts centimeters/second to whole mph with integer rounding. 1 mile = 160 934 cm, so
/// mph = cm_s * 3600 / 160934 (the +160934/2 rounds to nearest). The multiply is widened to u64 so `cm_s * 3600`
/// cannot overflow u32 (it would for cm_s > ~1.19M, wrapping silently in release) — mirrors [`cm_s_to_kmh`].
pub fn cm_s_to_mph(cm_s: u32) -> u32 {
  ((cm_s as u64 * 3600 + 160934 / 2) / 160934) as u32
}

/// Maps an RSSI in dBm to 0..4 signal bars, gated by link liveness. Thresholds match the design's reference sim:
/// stronger than -75 dBm is full (4) bars, then -90/-105 step down, and anything weaker is a single bar; a down
/// link shows zero regardless of the last RSSI. Strictly greater-than, matching the JS reference.
pub fn rssi_to_bars(rssi_dbm: i16, linked: bool) -> u8 {
  if !linked {
    0
  } else if rssi_dbm > -75 {
    4
  } else if rssi_dbm > -90 {
    3
  } else if rssi_dbm > -105 {
    2
  } else {
    1
  }
}

/// Maps a raw PPM pan pulse width (us, ~1000-2000, 1500 = center) to degrees for the G1 readout: +-500 us spans
/// +-45 deg (`(us-1500)*9/100`), clamped to the head-tracker's mechanical range so a garbage/overrange us can't
/// print a nonsense angle. Integer division truncates toward zero (e.g. 1750 us -> 22, not 22.5).
pub fn pan_us_to_deg(us: i32) -> i16 {
  (((us - 1500) * 9 / 100).clamp(-45, 45)) as i16
}

/// Maps a raw PPM tilt pulse width (us) to degrees for the G1 readout: +-500 us spans +-30 deg (`(us-1500)*3/50`),
/// clamped to the tilt range. Integer division truncates toward zero.
pub fn tilt_us_to_deg(us: i32) -> i16 {
  (((us - 1500) * 3 / 50).clamp(-30, 30)) as i16
}

/// A ready-to-use status OLED wrapping a buffered async SSD1306 on the shared I2C bus.
pub struct StatusDisplay<I: I2c> {
  display: OledDisplay<I>,
  // The last status actually drawn/flushed, used to skip the clear+draw+flush when nothing changed. `None`
  // before the first render forces an initial draw regardless of the incoming status. One cache per layout so
  // each render path dirty-checks independently; a render path nulls the OTHER caches so a layout switch always
  // forces a redraw (the panel and the surviving cache can never disagree about what is on screen).
  last: Option<Status>,
  last_goggle: Option<GoggleStatus>,
  last_truck: Option<TruckStatus>,
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
    Ok(Self { display, last: None, last_goggle: None, last_truck: None })
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

    // Record the rendered status only after a successful flush, so a failed flush re-renders next call. Drop the
    // other layouts' caches so a later render_goggle/render_truck always redraws after this Status frame.
    self.display.flush().await?;
    self.last = Some(status);
    self.last_goggle = None;
    self.last_truck = None;
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
    // Invalidate every dirty-cache: a later render()/render_goggle/render_truck must redraw, since the panel now
    // shows free-form lines.
    self.last = None;
    self.last_goggle = None;
    self.last_truck = None;
    self.display.flush().await
  }

  /// Borrows the underlying buffered display for direct embedded-graphics drawing.
  pub fn raw(&mut self) -> &mut OledDisplay<I> {
    &mut self.display
  }

  /// Renders the goggle node's G1 "Stopwatch" layout: a big lap clock with the lap number, live pan/tilt as signed
  /// degree readouts, the GPS satellite count, and LoRa signal bars in the header. Pixel coordinates mirror the
  /// committed design exactly (header rows 0-15, data 16-63). Dirty-checked against the last `GoggleStatus`.
  pub async fn render_goggle(&mut self, st: GoggleStatus) -> Result<(), display_interface::DisplayError> {
    if self.last_goggle == Some(st) {
      return Ok(());
    }
    let d = &mut self.display;
    d.clear_buffer();

    // Header (yellow rows 0-15): label, GPS sats, link bars.
    text(d, "HEAD-TX", 2, 4, TEXT_STYLE, TS_TOP_LEFT);
    let mut sats: String<16> = String::new();
    let _ = write!(sats, "SV{}", Pad2(st.sats));
    text(d, &sats, 110, 4, TEXT_STYLE, TS_TOP_RIGHT);
    signal_bars(d, 114, 3, st.bars);

    // Data (blue rows 16-63): lap number, big lap clock + trailing unit, divider, pan/tilt.
    let mut lap: String<16> = String::new();
    let _ = write!(lap, "LAP {}", Pad2(st.lap));
    text(d, &lap, 2, 17, TEXT_STYLE, TS_TOP_LEFT);

    let mut clock: String<16> = String::new();
    let _ = write!(clock, "{}.{}", st.lap_tenths / 10, st.lap_tenths % 10);
    text(d, &clock, 62, 26, BIG_STYLE, TS_TOP_CENTER);
    // Trailing "s" sits just right of the centered clock: base x 64 + half the big clock width + 4 px gap. The
    // 62-vs-64 split (clock centered on 62, "s" measured from 64) is reproduced verbatim from the committed design.
    let s_x = 64 + (clock.len() as i32 * BIG_W) / 2 + 4;
    text(d, "s", s_x, 36, TEXT_STYLE, TS_TOP_LEFT);

    hline(d, 8, 50, 112);

    let mut pan: String<16> = String::new();
    let _ = write!(pan, "PAN {}", Sgn(st.pan_deg as i32));
    text(d, &pan, 8, 54, TEXT_STYLE, TS_TOP_LEFT);
    let mut tilt: String<16> = String::new();
    let _ = write!(tilt, "{} TLT", Sgn(st.tilt_deg as i32));
    text(d, &tilt, 120, 54, TEXT_STYLE, TS_TOP_RIGHT);

    self.display.flush().await?;
    self.last_goggle = Some(st);
    self.last = None;
    self.last_truck = None;
    Ok(())
  }

  /// Renders the truck node's T1 "Arc / Center" layout: a segmented speedo arc wrapping a big MPH readout, with
  /// KM/H on the baseline and RSSI + LoRa bars in the header. Pixel coordinates mirror the committed design exactly.
  /// Dirty-checked against the last `TruckStatus`.
  pub async fn render_truck(&mut self, st: TruckStatus) -> Result<(), display_interface::DisplayError> {
    if self.last_truck == Some(st) {
      return Ok(());
    }
    let d = &mut self.display;
    d.clear_buffer();

    // Header (yellow rows 0-15): label, RSSI number (or "--" when down), link bars.
    text(d, "TRUCK-RX", 2, 4, TEXT_STYLE, TS_TOP_LEFT);
    let mut rssi: String<16> = String::new();
    if st.linked {
      let _ = write!(rssi, "{}", st.rssi);
    } else {
      let _ = write!(rssi, "--");
    }
    text(d, &rssi, 110, 4, TEXT_STYLE, TS_TOP_RIGHT);
    signal_bars(d, 114, 3, st.bars);

    // Data (blue rows 16-63): segmented arc, big speed, MPH + KM/H labels.
    let frac = (st.mph as f32 / SPEED_MAX_MPH as f32).min(1.0);
    seg_arc(d, 64, 60, 42, 35, frac, 26);

    let mut speed: String<16> = String::new();
    let _ = write!(speed, "{}", st.mph);
    text(d, &speed, 64, 26, BIG_STYLE, TS_TOP_CENTER);
    text(d, "MPH", 64, 47, TEXT_STYLE, TS_TOP_CENTER);
    let mut kmh: String<16> = String::new();
    let _ = write!(kmh, "{} KMH", st.kmh);
    text(d, &kmh, 64, 55, TEXT_STYLE, TS_TOP_CENTER);

    self.display.flush().await?;
    self.last_truck = Some(st);
    self.last = None;
    self.last_goggle = None;
    Ok(())
  }
}

/// Renders a flag as "Y"/"N" for the compact status line.
fn yn(b: bool) -> &'static str {
  if b { "Y" } else { "N" }
}

// ---- low-level draw helpers (port of the design's oled.js primitives) -------------------------------------------
// Each draws white "ink" into the buffered target. Drawing into the buffered SSD1306 is infallible, so the Result
// is discarded; these stay generic over the embedded-graphics target so the unit tests can exercise them on a mock.

/// Draws one line of text with the given mono character style and a top-baseline alignment (Left/Center/Right).
fn text<D: DrawTarget<Color = BinaryColor>>(
  d: &mut D, s: &str, x: i32, y: i32, cs: MonoTextStyle<'static, BinaryColor>, ts: TextStyle,
) {
  let _ = Text::with_text_style(s, Point::new(x, y), cs, ts).draw(d);
}

/// Filled horizontal line: a 1px-tall rectangle from (x, y) running `w` px right.
fn hline<D: DrawTarget<Color = BinaryColor>>(d: &mut D, x: i32, y: i32, w: i32) {
  let _ = Rectangle::new(Point::new(x, y), Size::new(w as u32, 1)).into_styled(FILL_ON).draw(d);
}

/// Filled rectangle at (x, y) of size `w` x `h`.
fn fill_rect<D: DrawTarget<Color = BinaryColor>>(d: &mut D, x: i32, y: i32, w: i32, h: i32) {
  let _ = Rectangle::new(Point::new(x, y), Size::new(w as u32, h as u32)).into_styled(FILL_ON).draw(d);
}

/// LoRa signal-strength bars in a 13x9 cell at (x, y): four 2px-wide bars of increasing height, `level` (0..4) of
/// them filled, the rest drawn hollow (a 1px left edge plus a 1px base tick). Ported verbatim from the design.
fn signal_bars<D: DrawTarget<Color = BinaryColor>>(d: &mut D, x: i32, y: i32, level: u8) {
  const H: [i32; 4] = [3, 5, 7, 9];
  for i in 0..4i32 {
    let bx = x + i * 3;
    let bh = H[i as usize];
    let by = y + (9 - bh);
    if i < level as i32 {
      fill_rect(d, bx, by, 2, bh); // active: solid 2 x bh
    } else {
      fill_rect(d, bx, by, 1, bh); // idle: 1px left edge ...
      let _ = Pixel(Point::new(bx + 1, y + 8), BinaryColor::On).draw(d); // ... plus a 1px base tick.
    }
  }
}

/// Segmented speedometer arc centered at (cx, cy) sweeping the upper half (PI..2PI = left -> top -> right). `frac`
/// (0..1) of the `n` segments are "lit" as a 3px radial stroke from `r_in` to `r_out`; the rest are a 2px tick at
/// the outer edge. Uses libm trig (f32 is ample precision for 26 segments on a 128px panel); endpoints are rounded
/// to the nearest pixel. Both the active stroke and the canvas reference stroke are centered on the segment line.
fn seg_arc<D: DrawTarget<Color = BinaryColor>>(d: &mut D, cx: i32, cy: i32, r_out: i32, r_in: i32, frac: f32, n: i32) {
  use core::f32::consts::PI;
  let lit = libm::roundf(frac * n as f32) as i32;
  let (cxf, cyf) = (cx as f32, cy as f32);
  let active = PrimitiveStyle::with_stroke(BinaryColor::On, 3);
  let tick = PrimitiveStyle::with_stroke(BinaryColor::On, 2);
  for i in 0..n {
    let t = (i as f32 + 0.5) / n as f32;
    let a = PI + t * PI;
    let ca = libm::cosf(a);
    let sa = libm::sinf(a);
    let pt = |r: f32| Point::new(libm::roundf(cxf + r * ca) as i32, libm::roundf(cyf + r * sa) as i32);
    if i < lit {
      let _ = Line::new(pt(r_in as f32), pt(r_out as f32)).into_styled(active).draw(d);
    } else {
      let _ = Line::new(pt((r_out - 2) as f32), pt(r_out as f32)).into_styled(tick).draw(d);
    }
  }
}

/// Formats an unsigned value zero-padded to at least two digits (7 -> "07", 12 -> "12", 123 -> "123").
struct Pad2(u32);
impl core::fmt::Display for Pad2 {
  fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
    write!(f, "{:02}", self.0)
  }
}

/// Formats a signed value as an explicit sign plus the magnitude zero-padded to two digits (-5 -> "-05", 42 -> "+42").
struct Sgn(i32);
impl core::fmt::Display for Sgn {
  fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
    write!(f, "{}{:02}", if self.0 >= 0 { '+' } else { '-' }, self.0.unsigned_abs())
  }
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
  fn cm_s_to_mph_rounds_to_nearest() {
    assert_eq!(cm_s_to_mph(0), 0);
    assert_eq!(cm_s_to_mph(447), 10); //  447 cm/s ~= 9.999 mph, rounds to 10.
    assert_eq!(cm_s_to_mph(448), 10); //  448 cm/s ~= 10.02 mph.
    assert_eq!(cm_s_to_mph(2235), 50); //  2235 cm/s ~= 49.99 mph, rounds to 50.
    assert_eq!(cm_s_to_mph(22), 0); //  0.492 mph rounds down to 0.
    assert_eq!(cm_s_to_mph(23), 1); //  0.514 mph rounds up to 1.
  }

  #[test]
  fn cm_s_to_mph_does_not_overflow_at_u32_max() {
    // cm_s * 3600 overflows u32 for cm_s > ~1.19M; the u64 intermediate must keep this from wrapping/panicking.
    let mph = cm_s_to_mph(u32::MAX);
    assert_eq!(mph, ((u32::MAX as u64 * 3600 + 160934 / 2) / 160934) as u32);
  }

  #[test]
  fn rssi_to_bars_steps_at_thresholds() {
    // Down link is always zero bars regardless of the last RSSI.
    assert_eq!(rssi_to_bars(-40, false), 0);
    // Strictly-greater-than thresholds: -75/-90/-105 are the step-DOWN boundaries.
    assert_eq!(rssi_to_bars(-74, true), 4);
    assert_eq!(rssi_to_bars(-75, true), 3);
    assert_eq!(rssi_to_bars(-89, true), 3);
    assert_eq!(rssi_to_bars(-90, true), 2);
    assert_eq!(rssi_to_bars(-104, true), 2);
    assert_eq!(rssi_to_bars(-105, true), 1);
    assert_eq!(rssi_to_bars(-130, true), 1);
  }

  #[test]
  fn us_to_deg_maps_and_clamps() {
    // Center us is zero degrees; +-500 us spans the full +-45 (pan) / +-30 (tilt) range.
    assert_eq!(pan_us_to_deg(1500), 0);
    assert_eq!(pan_us_to_deg(2000), 45);
    assert_eq!(pan_us_to_deg(1000), -45);
    assert_eq!(pan_us_to_deg(1750), 22); //  250*9/100 = 22.5, truncates toward zero to 22.
    assert_eq!(pan_us_to_deg(2200), 45); //  overrange clamps.
    assert_eq!(pan_us_to_deg(800), -45);
    assert_eq!(tilt_us_to_deg(1500), 0);
    assert_eq!(tilt_us_to_deg(2000), 30);
    assert_eq!(tilt_us_to_deg(1000), -30);
    assert_eq!(tilt_us_to_deg(1600), 6); //  100*3/50 = 6.
    assert_eq!(tilt_us_to_deg(2200), 30); //  overrange clamps.
  }

  #[test]
  fn goggle_status_participates_in_dirty_check() {
    // The render dirty-check is `self.last_goggle == Some(st)`, so any changed field must make two unequal.
    let base = GoggleStatus { lap: 3, lap_tenths: 127, pan_deg: 5, tilt_deg: -2, sats: 11, bars: 4 };
    assert_eq!(base, base);
    assert_ne!(base, GoggleStatus { lap_tenths: 128, ..base });
    assert_ne!(base, GoggleStatus { pan_deg: 6, ..base });
  }

  #[test]
  fn truck_status_participates_in_dirty_check() {
    let base = TruckStatus { mph: 42, kmh: 67, bars: 3, rssi: -68, linked: true };
    assert_eq!(base, base);
    assert_ne!(base, TruckStatus { mph: 43, ..base });
    assert_ne!(base, TruckStatus { linked: false, ..base });
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
