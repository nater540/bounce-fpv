//! UART NMEA ground-speed reader (bonus). `no_std` + no-alloc, generic over `embedded_io_async::Read` (the
//! esp-hal async UART), with a hand-rolled minimal NMEA 0183 parser — no `nmea` crate, which needs `alloc`.
//!
//! It reads bytes into a fixed heapless line buffer, splits on CR/LF, validates the trailing `*HH` checksum,
//! and parses the speed-bearing sentences: `$..RMC` (field 7 = speed over ground in KNOTS) and `$..VTG`
//! (field 7 = speed over ground in KM/H), plus `$..GGA` for fix quality / satellite count when available.
//! Ground speed is reported as integer **centimeters per second**, the unit shared with the proto
//! `Telemetry.speed_cm_s` field — keep them identical. All conversions are integer/fixed-point (no floats):
//! decimal speed strings are parsed to milli-units, then `1 knot = 51.4444 cm/s` and `1 km/h = 27.7778 cm/s`
//! are applied as exact integer rationals (see [`milliknots_to_cm_s`] / [`milli_kmh_to_cm_s`]).
//!
//! The pure parsing logic lives in [`parse_line`] over `&[u8]`, isolated from the async I/O so it is trivially
//! testable; [`GpsReader::next_fix`] is the thin async loop that fills the buffer and calls it.

#![no_std]

use embedded_io_async::Read;
use heapless::Vec;

/// Maximum NMEA sentence length we buffer. The standard caps a sentence at 82 chars including `$`, CR, LF;
/// 96 leaves margin for the occasional non-conforming receiver without unbounded growth.
pub const MAX_SENTENCE_LEN: usize = 96;

/// Internal UART read chunk. Reads land here before being fed byte-by-byte into the line assembler.
const READ_CHUNK: usize = 64;

/// GNSS fix quality, decoded from the GGA fix-quality field (0 = no fix, 1 = GPS, 2 = DGPS, ...). We only
/// distinguish "no fix" from "have a fix" for the status display; the raw value is preserved in `raw`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct FixQuality {
  pub raw: u8,
}

impl FixQuality {
  /// True when the receiver reports any usable fix (GGA quality != 0).
  pub fn has_fix(&self) -> bool {
    self.raw != 0
  }
}

/// A decoded speed/fix snapshot. `speed_cm_s` is ground speed in centimeters per second (matches
/// `Telemetry.speed_cm_s`). `satellites` and `fix` are populated when a GGA sentence supplied them since the
/// last speed sentence; both are `None` if the receiver has not emitted GGA yet.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub struct GpsFix {
  pub speed_cm_s: u32,
  pub satellites: Option<u8>,
  pub fix: Option<FixQuality>,
}

/// What a single parsed NMEA line yielded. RMC/VTG carry speed; GGA carries satellites + fix quality; other
/// (or checksum-failed, or fieldwise-empty) sentences yield `None`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Parsed {
  /// Ground speed over ground, already converted to centimeters per second.
  Speed(u32),
  /// Fix status from GGA: satellite count and fix quality.
  Status { satellites: Option<u8>, fix: FixQuality },
  /// A valid sentence we do not extract anything from, or a sentence with the relevant field empty.
  None,
}

/// Upper plausibility bound on reported ground speed, in centimeters per second. `parse_decimal_milli` will
/// accept a huge (checksum-valid but corrupt) speed field, and a value near u32::MAX overflows the downstream
/// `cm_s * 36` km/h math, so any conversion above this is clamped here before it can propagate. 10,000,000
/// cm/s is 100 km/s (~360,000 km/h) — absurdly generous for any vehicle yet leaves `* 36` far under u32::MAX.
pub const MAX_SPEED_CM_S: u32 = 10_000_000;

/// Converts milli-knots (knots * 1000) to centimeters per second. 1 knot = 0.514444 m/s = 51.4444 cm/s, so
/// cm/s = milliknots * 514444 / 10_000_000. Computed in u64 to avoid overflow, rounded to nearest, then
/// clamped to [`MAX_SPEED_CM_S`] so a corrupt speed field cannot yield an overflow-prone value downstream.
pub fn milliknots_to_cm_s(milliknots: u32) -> u32 {
  let num = milliknots as u64 * 514_444 + 5_000_000; //  + half-divisor for round-to-nearest.
  ((num / 10_000_000) as u32).min(MAX_SPEED_CM_S)
}

/// Converts milli-(km/h) (km/h * 1000) to centimeters per second. 1 km/h = 1000 m / 3600 s = 27.7778 cm/s,
/// exactly cm/s = milli_kmh / 36. Rounded to nearest, then clamped to [`MAX_SPEED_CM_S`] so a corrupt speed
/// field cannot yield an overflow-prone value downstream.
pub fn milli_kmh_to_cm_s(milli_kmh: u32) -> u32 {
  (((milli_kmh as u64 + 18) / 36) as u32).min(MAX_SPEED_CM_S) //  + half-divisor (18) for round-to-nearest.
}

/// Validates and strips the NMEA framing of one raw line (everything between `$` and the line terminator,
/// possibly including the `*HH` checksum). Returns the sentence body (after `$`, before `*`) when the
/// checksum is present and correct, or when no checksum is present at all. Returns `None` on a bad checksum
/// or malformed framing, so a corrupted line is dropped rather than mis-parsed.
pub fn validate_and_body(line: &[u8]) -> Option<&[u8]> {
  // Trim leading whitespace/control and require a leading `$` (or `!` for AIS-style, which we still accept).
  let start = line.iter().position(|&b| b == b'$' || b == b'!')?;
  let rest = &line[start + 1..];

  // Split off an optional `*HH` checksum. Per NMEA the checksum is the XOR of every byte between `$` and `*`.
  if let Some(star) = rest.iter().position(|&b| b == b'*') {
    let body = &rest[..star];
    let hex = &rest[star + 1..];
    let hi = hex_nibble(*hex.first()?)?;
    let lo = hex_nibble(*hex.get(1)?)?;
    let expected = (hi << 4) | lo;
    let actual = body.iter().fold(0u8, |acc, &b| acc ^ b);
    if actual != expected {
      return None;
    }
    Some(body)
  } else {
    // No checksum field — accept the body as-is (some emulators/loggers omit it). Trim trailing CR/LF.
    let end = rest.iter().position(|&b| b == b'\r' || b == b'\n').unwrap_or(rest.len());
    Some(&rest[..end])
  }
}

/// Parses one already-de-framed NMEA line (raw bytes including `$` and any `*HH`). This is the pure entry
/// point used by [`GpsReader::next_fix`] and the tests: it validates the checksum, identifies the sentence
/// type by its last three talker-agnostic letters (RMC/VTG/GGA), and extracts the field of interest.
pub fn parse_line(line: &[u8]) -> Parsed {
  let Some(body) = validate_and_body(line) else {
    return Parsed::None;
  };

  // The sentence type is the address field up to the first comma, e.g. "GPRMC" / "GNVTG". We match on the
  // final three letters so any talker prefix (GP, GN, GL, GA, BD, ...) is handled uniformly.
  let addr_end = body.iter().position(|&b| b == b',').unwrap_or(body.len());
  let addr = &body[..addr_end];
  if addr.len() < 3 {
    return Parsed::None;
  }
  let kind = &addr[addr.len() - 3..];

  if kind == b"RMC" {
    // RMC field 7 (0-based, where field 0 is the address) is speed over ground in knots.
    match field(body, 7).and_then(parse_decimal_milli) {
      Some(milliknots) => Parsed::Speed(milliknots_to_cm_s(milliknots)),
      None => Parsed::None,
    }
  } else if kind == b"VTG" {
    // VTG field 7 is speed over ground in km/h; field 5 is the same speed in knots. Some receivers populate
    // only knots, so fall back to field 5 (via milliknots_to_cm_s) when field 7 is empty/absent.
    match field(body, 7).and_then(parse_decimal_milli) {
      Some(milli_kmh) => Parsed::Speed(milli_kmh_to_cm_s(milli_kmh)),
      None => match field(body, 5).and_then(parse_decimal_milli) {
        Some(milliknots) => Parsed::Speed(milliknots_to_cm_s(milliknots)),
        None => Parsed::None,
      },
    }
  } else if kind == b"GGA" {
    // GGA field 6 is fix quality, field 7 is the satellite count. Both come from parse_uint, which accepts
    // arbitrary digit counts, so saturate to u8::MAX instead of `as u8` truncating (300 -> 44, 256 -> 0).
    let fix = field(body, 6).and_then(parse_uint).map(|q| FixQuality { raw: q.min(u8::MAX as u32) as u8 });
    let sats = field(body, 7).and_then(parse_uint).map(|n| n.min(u8::MAX as u32) as u8);
    match fix {
      Some(fix) => Parsed::Status { satellites: sats, fix },
      None => Parsed::None,
    }
  } else {
    Parsed::None
  }
}

/// Returns the comma-separated field at `index` of an NMEA sentence body (field 0 = the address). Returns
/// `None` if the field is missing or empty, which is how NMEA marks "no data" (e.g. speed before a fix).
fn field(body: &[u8], index: usize) -> Option<&[u8]> {
  let mut i = 0usize;
  for f in body.split(|&b| b == b',') {
    if i == index {
      return if f.is_empty() { None } else { Some(f) };
    }
    i += 1;
  }
  None
}

/// Parses an unsigned-decimal ASCII field like `12.34` into milli-units (value * 1000), integer-only. Reads
/// at most three fractional digits and ignores the rest. Returns `None` on any non-digit (besides one `.`).
fn parse_decimal_milli(s: &[u8]) -> Option<u32> {
  let mut int_part: u32 = 0;
  let mut frac: u32 = 0;
  let mut frac_digits = 0u32;
  let mut seen_dot = false;
  let mut seen_any = false;

  for &b in s {
    match b {
      b'0'..=b'9' => {
        seen_any = true;
        if seen_dot {
          if frac_digits < 3 {
            frac = frac * 10 + (b - b'0') as u32;
            frac_digits += 1;
          }
        } else {
          int_part = int_part.checked_mul(10)?.checked_add((b - b'0') as u32)?;
        }
      }
      b'.' if !seen_dot => seen_dot = true,
      _ => return None,
    }
  }
  if !seen_any {
    return None;
  }
  // Scale the fractional part up to exactly three digits (milli-resolution).
  while frac_digits < 3 {
    frac *= 10;
    frac_digits += 1;
  }
  int_part.checked_mul(1000)?.checked_add(frac)
}

/// Parses a plain unsigned integer ASCII field. Returns `None` on any non-digit byte.
fn parse_uint(s: &[u8]) -> Option<u32> {
  if s.is_empty() {
    return None;
  }
  let mut v: u32 = 0;
  for &b in s {
    if !b.is_ascii_digit() {
      return None;
    }
    v = v.checked_mul(10)?.checked_add((b - b'0') as u32)?;
  }
  Some(v)
}

/// Maps an ASCII hex digit to its 0-15 value, or `None` if it is not a hex digit.
fn hex_nibble(b: u8) -> Option<u8> {
  match b {
    b'0'..=b'9' => Some(b - b'0'),
    b'a'..=b'f' => Some(b - b'a' + 10),
    b'A'..=b'F' => Some(b - b'A' + 10),
    _ => None,
  }
}

/// Async NMEA reader over an `embedded_io_async::Read` UART. Assembles lines into a fixed buffer and parses
/// them; [`next_fix`](GpsReader::next_fix) returns once a speed sentence (RMC or VTG) completes, carrying the
/// latest satellite/fix info seen from interleaved GGA sentences.
pub struct GpsReader<R> {
  uart: R,
  line: Vec<u8, MAX_SENTENCE_LEN>,
  chunk: [u8; READ_CHUNK],
  // Bytes of `chunk` currently buffered and how far we have consumed them. A read fills `chunk[..chunk_len]`
  // and `chunk_pos` walks it; when a speed sentence returns mid-chunk we save the resume point here so the
  // remainder (the start of following sentences read in the same UART chunk) is parsed on the next call, not
  // overwritten by a fresh read.
  chunk_len: usize,
  chunk_pos: usize,
  // Latest fix status seen since the last returned speed; folded into the next `GpsFix`.
  last_sats: Option<u8>,
  last_fix: Option<FixQuality>,
}

impl<R: Read> GpsReader<R> {
  /// Wraps a UART that yields the receiver's NMEA byte stream. The reader only consumes RX; nothing is sent.
  pub fn new(uart: R) -> Self {
    Self { uart, line: Vec::new(), chunk: [0; READ_CHUNK], chunk_len: 0, chunk_pos: 0, last_sats: None, last_fix: None }
  }

  /// Reads from the UART until a speed sentence (RMC or VTG) completes, returning a [`GpsFix`] with the
  /// ground speed in cm/s plus the most recent satellite/fix status. GGA sentences seen along the way update
  /// the cached status but do not themselves return. Propagates the UART error type on a read failure.
  ///
  /// Lines longer than [`MAX_SENTENCE_LEN`] are discarded (the buffer resets at the next terminator), and a
  /// read returning 0 bytes is treated as "no data yet" and retried, so a quiet UART simply keeps awaiting.
  pub async fn next_fix(&mut self) -> Result<GpsFix, R::Error> {
    loop {
      // Refill from the UART only once the previously read chunk is fully consumed; otherwise resume from the
      // unconsumed tail so a sentence following the one we returned on (read in the same chunk) is not lost.
      if self.chunk_pos >= self.chunk_len {
        self.chunk_len = self.uart.read(&mut self.chunk).await?;
        self.chunk_pos = 0;
      }
      while self.chunk_pos < self.chunk_len {
        let b = self.chunk[self.chunk_pos];
        self.chunk_pos += 1;
        if b == b'\r' || b == b'\n' {
          if !self.line.is_empty() {
            // Borrow the line for parsing, then clear it for the next sentence regardless of the outcome.
            let parsed = parse_line(&self.line);
            self.line.clear();
            match parsed {
              Parsed::Speed(speed_cm_s) => {
                // `chunk_pos` already points past this terminator, so the next call resumes mid-chunk.
                return Ok(GpsFix { speed_cm_s, satellites: self.last_sats, fix: self.last_fix });
              }
              Parsed::Status { satellites, fix } => {
                self.last_sats = satellites;
                self.last_fix = Some(fix);
              }
              Parsed::None => {}
            }
          }
        } else if self.line.push(b).is_err() {
          // Overlong / runaway line: drop it and resync on the next terminator.
          self.line.clear();
        }
      }
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn rmc_speed_in_knots_to_cm_s() {
    // Standard GPRMC; field 7 (speed) = 22.4 knots. 22.4 kn * 51.4444 = 1152.35 cm/s -> 1152.
    let line = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
    match parse_line(line) {
      Parsed::Speed(cm_s) => assert_eq!(cm_s, milliknots_to_cm_s(22_400)),
      other => panic!("expected Speed, got {:?}", other),
    }
    assert_eq!(milliknots_to_cm_s(22_400), 1152);
  }

  #[test]
  fn vtg_speed_in_kmh_to_cm_s() {
    // GPVTG field 7 = 10.0 km/h -> 277 cm/s. Checksum computed over the body.
    let body = b"GPVTG,054.7,T,034.4,M,005.5,N,010.0,K";
    let cksum = body.iter().fold(0u8, |a, &b| a ^ b);
    let mut line: heapless::Vec<u8, 96> = heapless::Vec::new();
    line.push(b'$').unwrap();
    line.extend_from_slice(body).unwrap();
    line.push(b'*').unwrap();
    let hex = b"0123456789ABCDEF";
    line.push(hex[(cksum >> 4) as usize]).unwrap();
    line.push(hex[(cksum & 0xF) as usize]).unwrap();
    match parse_line(&line) {
      Parsed::Speed(cm_s) => assert_eq!(cm_s, milli_kmh_to_cm_s(10_000)),
      other => panic!("expected Speed, got {:?}", other),
    }
    assert_eq!(milli_kmh_to_cm_s(10_000), 278); //  10 km/h = 277.78 cm/s, rounded.
  }

  #[test]
  fn bad_checksum_is_rejected() {
    let line = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*00";
    assert_eq!(parse_line(line), Parsed::None);
  }

  #[test]
  fn gga_yields_fix_status() {
    let line = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
    match parse_line(line) {
      Parsed::Status { satellites, fix } => {
        assert_eq!(satellites, Some(8));
        assert!(fix.has_fix());
        assert_eq!(fix.raw, 1);
      }
      other => panic!("expected Status, got {:?}", other),
    }
  }

  #[test]
  fn empty_speed_field_before_fix_is_none() {
    // RMC with status 'V' (void) and an empty speed field — common before a fix.
    let line = b"$GPRMC,,V,,,,,,,,,,N*53";
    assert_eq!(parse_line(line), Parsed::None);
  }

  #[test]
  fn enormous_speed_field_is_clamped_not_overflowed() {
    // FIX #6: a checksum-valid but corrupt huge speed (4,000,000 knots ~= 205M cm/s) must clamp at
    // MAX_SPEED_CM_S, never propagate an overflow-prone value into the downstream `* 36` km/h math.
    let body: &[u8] = b"GPRMC,123519,A,4807.038,N,01131.000,E,4000000.0,084.4,230394,003.1,W";
    let line = framed(body);
    match parse_line(&line) {
      Parsed::Speed(cm_s) => {
        assert_eq!(cm_s, MAX_SPEED_CM_S);
        assert!(cm_s.checked_mul(36).is_some()); //  the downstream km/h math stays within u32.
      }
      other => panic!("expected clamped Speed, got {:?}", other),
    }
  }

  #[test]
  fn three_digit_sats_and_huge_quality_saturate_not_wrap() {
    // FIX #9: 300 sats must not wrap to 44, and quality 300 must not wrap to 0 ('no fix').
    let body: &[u8] = b"GPGGA,123519,4807.038,N,01131.000,E,300,300,0.9,545.4,M,46.9,M,,";
    let line = framed(body);
    match parse_line(&line) {
      Parsed::Status { satellites, fix } => {
        assert_eq!(satellites, Some(u8::MAX));
        assert_eq!(fix.raw, u8::MAX);
        assert!(fix.has_fix());
      }
      other => panic!("expected saturated Status, got {:?}", other),
    }
  }

  #[test]
  fn vtg_falls_back_to_knots_when_kmh_blank() {
    // FIX #11: field 7 (km/h) blank but field 5 (knots) = 005.5 — speed must come from the knots field.
    let body: &[u8] = b"GPVTG,054.7,T,034.4,M,005.5,N,,K";
    let line = framed(body);
    match parse_line(&line) {
      Parsed::Speed(cm_s) => assert_eq!(cm_s, milliknots_to_cm_s(5_500)),
      other => panic!("expected Speed from knots fallback, got {:?}", other),
    }
  }

  // ----- FIX #1: mid-chunk resume across next_fix calls -------------------------------------------------

  use core::future::Future;
  use core::pin::Pin;
  use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

  /// A `Read` mock that serves a fixed script from a cursor, filling each `read` buffer fully (so a script
  /// longer than one READ_CHUNK is delivered across successive reads, the same as a real UART), then returns
  /// 0 (quiet UART) once exhausted. The point under test is that an RMC and the GGA/VTG that follow it land in
  /// one logical stream and the bytes after the RMC's terminator survive across next_fix calls.
  struct ScriptReader {
    data: &'static [u8],
    pos: usize,
  }

  impl embedded_io_async::ErrorType for ScriptReader {
    type Error = core::convert::Infallible;
  }

  impl Read for ScriptReader {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
      let n = (self.data.len() - self.pos).min(buf.len());
      buf[..n].copy_from_slice(&self.data[self.pos..self.pos + n]);
      self.pos += n;
      Ok(n)
    }
  }

  /// Minimal poll-to-completion executor for the always-ready mock — no external async runtime needed.
  fn block_on<F: Future>(mut fut: F) -> F::Output {
    fn noop(_: *const ()) {}
    fn clone(_: *const ()) -> RawWaker {
      RawWaker::new(core::ptr::null(), &VTABLE)
    }
    static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, noop, noop, noop);
    let waker = unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) };
    let mut cx = Context::from_waker(&waker);
    let mut fut = unsafe { Pin::new_unchecked(&mut fut) };
    loop {
      if let Poll::Ready(out) = fut.as_mut().poll(&mut cx) {
        return out;
      }
    }
  }

  #[test]
  fn rmc_then_gga_in_one_chunk_does_not_drop_the_gga() {
    // RMC completes mid-chunk and returns the speed; the GGA that followed it in the SAME read must survive so
    // its sats/fix are observed on the next call instead of being overwritten by a fresh read.
    static SCRIPT: &[u8] = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n\
$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n\
$GPVTG,054.7,T,034.4,M,005.5,N,010.0,K*4A\r\n";
    let mut reader = GpsReader::new(ScriptReader { data: SCRIPT, pos: 0 });

    // First fix is the RMC speed. The GGA had not been parsed yet, so sats/fix are still None here.
    let first = block_on(reader.next_fix()).unwrap();
    assert_eq!(first.speed_cm_s, milliknots_to_cm_s(22_400));

    // Second fix resumes from the buffered tail: the GGA is parsed (updating cached status) and the trailing
    // VTG returns its speed — now carrying the satellites/fix the GGA supplied, proving the GGA was not lost.
    let second = block_on(reader.next_fix()).unwrap();
    assert_eq!(second.speed_cm_s, milli_kmh_to_cm_s(10_000));
    assert_eq!(second.satellites, Some(8));
    assert_eq!(second.fix.map(|f| f.raw), Some(1));
  }

  /// Builds a `$<body>*HH\r\n` framed NMEA line with a correct checksum, into a heapless buffer.
  fn framed(body: &[u8]) -> heapless::Vec<u8, MAX_SENTENCE_LEN> {
    let cksum = body.iter().fold(0u8, |a, &b| a ^ b);
    let hex = b"0123456789ABCDEF";
    let mut line: heapless::Vec<u8, MAX_SENTENCE_LEN> = heapless::Vec::new();
    line.push(b'$').unwrap();
    line.extend_from_slice(body).unwrap();
    line.push(b'*').unwrap();
    line.push(hex[(cksum >> 4) as usize]).unwrap();
    line.push(hex[(cksum & 0xF) as usize]).unwrap();
    line
  }
}
