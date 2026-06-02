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
/// last speed sentence; both are `None` if the receiver has not emitted GGA yet. `lat_e7`/`lon_e7` are the
/// position in degrees × 1e7 (+N/+E), `None` until a sentence carried coordinates. They are reported as-parsed
/// regardless of fix quality, so consumers MUST gate on `fix` before trusting them — 0,0 is a valid point.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub struct GpsFix {
  pub speed_cm_s: u32,
  pub satellites: Option<u8>,
  pub fix: Option<FixQuality>,
  pub lat_e7: Option<i32>,
  pub lon_e7: Option<i32>,
}

/// What a single parsed NMEA line yielded. RMC carries speed AND (when its status is 'A') position; VTG carries
/// speed only; GGA carries satellites + fix quality + position. Other (or checksum-failed, or fieldwise-empty)
/// sentences yield `None`. Coordinates ride alongside the sentence that carried them — RMC returns them on the
/// same Speed value that triggers a fix, so position and speed stay atomic instead of lagging a cache.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Parsed {
  /// Ground speed (cm/s), plus the sentence's own coordinates when it carried valid ones (RMC; `None` for VTG).
  Speed { speed_cm_s: u32, lat_e7: Option<i32>, lon_e7: Option<i32> },
  /// Fix status from GGA: satellite count, fix quality, and the GGA position when present.
  Status { satellites: Option<u8>, fix: FixQuality, lat_e7: Option<i32>, lon_e7: Option<i32> },
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
    // RMC field 7 (0-based, where field 0 is the address) is speed over ground in knots; without it there is
    // no fix to return. Position rides fields 3/4 (lat, N/S) and 5/6 (lon, E/W), but ONLY when field 2 (status)
    // is 'A' (active); 'V' (void) carries a stale/garbage position before a real fix.
    let Some(milliknots) = field(body, 7).and_then(parse_decimal_milli) else {
      return Parsed::None;
    };
    let (lat_e7, lon_e7) = if matches!(field(body, 2), Some(b"A")) {
      (coord(body, 3, 4), coord(body, 5, 6))
    } else {
      (None, None)
    };
    Parsed::Speed { speed_cm_s: milliknots_to_cm_s(milliknots), lat_e7, lon_e7 }
  } else if kind == b"VTG" {
    // VTG field 7 is speed over ground in km/h; field 5 is the same speed in knots. Some receivers populate
    // only knots, so fall back to field 5 (via milliknots_to_cm_s) when field 7 is empty/absent. VTG carries
    // no position, so its coordinates are always `None`.
    match field(body, 7).and_then(parse_decimal_milli) {
      Some(milli_kmh) => Parsed::Speed { speed_cm_s: milli_kmh_to_cm_s(milli_kmh), lat_e7: None, lon_e7: None },
      None => match field(body, 5).and_then(parse_decimal_milli) {
        Some(milliknots) => Parsed::Speed { speed_cm_s: milliknots_to_cm_s(milliknots), lat_e7: None, lon_e7: None },
        None => Parsed::None,
      },
    }
  } else if kind == b"GGA" {
    // GGA field 6 is fix quality, field 7 is the satellite count. Both come from parse_uint, which accepts
    // arbitrary digit counts, so saturate to u8::MAX instead of `as u8` truncating (300 -> 44, 256 -> 0).
    let fix = field(body, 6).and_then(parse_uint).map(|q| FixQuality { raw: q.min(u8::MAX as u32) as u8 });
    let sats = field(body, 7).and_then(parse_uint).map(|n| n.min(u8::MAX as u32) as u8);
    // Position rides fields 2/3 (lat, N/S) and 4/5 (lon, E/W), but ONLY when the fix is usable (quality != 0).
    // A no-fix GGA can still carry empty or last-known lat/lon; surfacing those would seed the position cache
    // with a stale point that a later void RMC's cache fallback would then resurface as a "live" position.
    let (lat_e7, lon_e7) = match fix {
      Some(f) if f.has_fix() => (coord(body, 2, 3), coord(body, 4, 5)),
      _ => (None, None),
    };
    match fix {
      Some(fix) => Parsed::Status { satellites: sats, fix, lat_e7, lon_e7 },
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

/// Reads a coordinate from a value field (`value_index`, the `ddmm.mmmm`) and its adjacent hemisphere field
/// (`hemi_index`, one of N/S/E/W), returning degrees × 1e7 or `None` when either field is absent/malformed.
fn coord(body: &[u8], value_index: usize, hemi_index: usize) -> Option<i32> {
  match (field(body, value_index), field(body, hemi_index)) {
    (Some(value), Some(hemi)) => parse_coord_e7(value, hemi),
    _ => None,
  }
}

/// Folds a run of ASCII digits into an i64, returning `None` on any non-digit byte (an empty slice yields 0).
/// Shared by the degree and whole-minute fields of [`parse_coord_e7`] so the accumulation idiom lives once.
fn fold_digits_i64(bytes: &[u8]) -> Option<i64> {
  let mut v: i64 = 0;
  for &b in bytes {
    if !b.is_ascii_digit() {
      return None;
    }
    v = v * 10 + (b - b'0') as i64;
  }
  Some(v)
}

/// Parses an NMEA latitude/longitude magnitude (`ddmm.mmmm` / `dddmm.mmmm`) plus its hemisphere byte into
/// degrees × 1e7 (i32, +N/+E). The degree/minute split is positional — the two integer digits left of the
/// decimal point are whole minutes, everything to their left is degrees — so one code path handles 2-digit
/// latitude and 3-digit longitude (and rejects a longitude mis-fed as latitude, whose surplus digit pushes
/// minutes past 60). All scaling is integer i64, rounded to nearest. The hemisphere byte sets BOTH the sign and
/// the valid range (latitude <= 90°, longitude <= 180°), so a corrupt over-range magnitude is rejected per-axis.
/// Returns `None` on any malformed field: a non-digit byte, fewer than four integer digits, minutes >= 60, an
/// out-of-range result, or a missing/unknown hemisphere. Deliberately NOT built on `parse_decimal_milli`, which
/// caps at three fractional digits and would coarsen a coordinate to a couple meters of minute resolution.
fn parse_coord_e7(value: &[u8], hemi: &[u8]) -> Option<i32> {
  // Split the integer part from the optional fractional minutes at the decimal point.
  let dot = value.iter().position(|&b| b == b'.').unwrap_or(value.len());
  let int_part = &value[..dot];
  let frac_part = value.get(dot + 1..).unwrap_or(&[]);
  // Need at least dd + mm: two degree-or-more digits plus the two whole-minute digits.
  if int_part.len() < 4 {
    return None;
  }
  let (deg_bytes, min_bytes) = int_part.split_at(int_part.len() - 2);

  // Degrees (any number of leading digits) and whole minutes (exactly the two digits left of the point).
  let deg = fold_digits_i64(deg_bytes)?;
  let min_whole = fold_digits_i64(min_bytes)?;
  if min_whole >= 60 {
    return None;
  }
  // Fractional minutes scaled to exactly five digits (1e-5 minute ~= 1.85 cm — far finer than needed): read at
  // most five, ignore any beyond, and right-pad a short field.
  let mut frac: i64 = 0;
  let mut digits = 0;
  for &b in frac_part {
    if !b.is_ascii_digit() {
      return None;
    }
    if digits < 5 {
      frac = frac * 10 + (b - b'0') as i64;
      digits += 1;
    }
  }
  while digits < 5 {
    frac *= 10;
    digits += 1;
  }
  // min_scaled is minutes in units of 1e-5 minute (0..5_999_999); deg_e7 = deg*1e7 + round(min_scaled*1e7 /
  // (60*1e5)). The +half-divisor rounds to nearest, matching the speed converters. All in i64 (the numerator
  // peaks near 6e13).
  let min_scaled = min_whole * 100_000 + frac;
  let deg_e7 = deg * 10_000_000 + (min_scaled * 10_000_000 + 3_000_000) / 6_000_000;
  // Hemisphere sets the sign AND the per-axis range bound: latitude (N/S) caps at 90°, longitude (E/W) at 180°.
  // A missing/unknown hemisphere, or a magnitude beyond the axis bound (e.g. a corrupt 91° latitude), is rejected.
  let (sign, limit): (i32, i64) = match hemi.first() {
    Some(b'N') => (1, 900_000_000),
    Some(b'S') => (-1, 900_000_000),
    Some(b'E') => (1, 1_800_000_000),
    Some(b'W') => (-1, 1_800_000_000),
    _ => return None,
  };
  if deg_e7 > limit {
    return None;
  }
  Some(sign * deg_e7 as i32)
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
  // Latest fix status + position seen since the last returned speed; folded into the next `GpsFix`. The
  // coordinate cache is a fallback for VTG-speed receivers (whose Speed sentence carries no position); an RMC
  // return uses its OWN coordinates and never reaches the cache, so position stays atomic with the speed.
  last_sats: Option<u8>,
  last_fix: Option<FixQuality>,
  last_lat_e7: Option<i32>,
  last_lon_e7: Option<i32>,
}

impl<R: Read> GpsReader<R> {
  /// Wraps a UART that yields the receiver's NMEA byte stream. The reader only consumes RX; nothing is sent.
  pub fn new(uart: R) -> Self {
    Self {
      uart,
      line: Vec::new(),
      chunk: [0; READ_CHUNK],
      chunk_len: 0,
      chunk_pos: 0,
      last_sats: None,
      last_fix: None,
      last_lat_e7: None,
      last_lon_e7: None,
    }
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
              Parsed::Speed { speed_cm_s, lat_e7, lon_e7 } => {
                // Prefer this sentence's OWN coordinates (RMC carries them atomically with the speed return);
                // fall back to the last GGA-cached position so a VTG-speed receiver still reports a (one-fix-
                // old) location. `chunk_pos` already points past this terminator, so the next call resumes mid-chunk.
                let lat_e7 = lat_e7.or(self.last_lat_e7);
                let lon_e7 = lon_e7.or(self.last_lon_e7);
                return Ok(GpsFix { speed_cm_s, satellites: self.last_sats, fix: self.last_fix, lat_e7, lon_e7 });
              }
              Parsed::Status { satellites, fix, lat_e7, lon_e7 } => {
                self.last_sats = satellites;
                self.last_fix = Some(fix);
                self.last_lat_e7 = lat_e7;
                self.last_lon_e7 = lon_e7;
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
      Parsed::Speed { speed_cm_s, lat_e7, lon_e7 } => {
        assert_eq!(speed_cm_s, milliknots_to_cm_s(22_400));
        // RMC status is 'A', so it carries its own position alongside the speed (48° 07.038' N, 11° 31.000' E).
        assert_eq!(lat_e7, Some(481_173_000));
        assert_eq!(lon_e7, Some(115_166_667));
      }
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
      Parsed::Speed { speed_cm_s, lat_e7, lon_e7 } => {
        assert_eq!(speed_cm_s, milli_kmh_to_cm_s(10_000));
        assert_eq!((lat_e7, lon_e7), (None, None)); //  VTG carries no position.
      }
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
      Parsed::Status { satellites, fix, lat_e7, lon_e7 } => {
        assert_eq!(satellites, Some(8));
        assert!(fix.has_fix());
        assert_eq!(fix.raw, 1);
        assert_eq!(lat_e7, Some(481_173_000)); //  48° 07.038' N.
        assert_eq!(lon_e7, Some(115_166_667)); //  11° 31.000' E.
      }
      other => panic!("expected Status, got {:?}", other),
    }
  }

  #[test]
  fn southern_western_hemispheres_negate() {
    // Same magnitudes as the canonical fixture, but S/W must flip the sign.
    assert_eq!(parse_coord_e7(b"4807.038", b"S"), Some(-481_173_000));
    assert_eq!(parse_coord_e7(b"01131.000", b"W"), Some(-115_166_667));
  }

  #[test]
  fn longitude_three_degree_digits_and_range_bound() {
    // Longitude has three degree digits; the positional split must still take the last two as whole minutes.
    assert_eq!(parse_coord_e7(b"01131.000", b"E"), Some(115_166_667));
    // 180° 00.000' is the longitude upper bound (valid); past it the result is rejected rather than overflowing.
    assert_eq!(parse_coord_e7(b"18000.000", b"E"), Some(1_800_000_000));
    assert_eq!(parse_coord_e7(b"18100.000", b"E"), None);
  }

  #[test]
  fn latitude_bounded_to_90_degrees() {
    // Latitude caps at 90° (a tighter bound than longitude's 180°): 90°00.000' is valid, 90°01' is rejected as
    // an impossible/corrupt coordinate rather than passing the generic 180° check.
    assert_eq!(parse_coord_e7(b"9000.000", b"N"), Some(900_000_000));
    assert_eq!(parse_coord_e7(b"9001.000", b"N"), None);
    assert_eq!(parse_coord_e7(b"9000.000", b"S"), Some(-900_000_000));
  }

  #[test]
  fn malformed_coords_rejected() {
    assert_eq!(parse_coord_e7(b"4860.000", b"N"), None); //  minutes >= 60.
    assert_eq!(parse_coord_e7(b"4807.038", b""), None); //  missing hemisphere.
    assert_eq!(parse_coord_e7(b"4807.038", b"X"), None); //  unknown hemisphere.
    assert_eq!(parse_coord_e7(b"480.0", b"N"), None); //  fewer than four integer digits.
    assert_eq!(parse_coord_e7(b"48a7.038", b"N"), None); //  non-digit byte.
  }

  #[test]
  fn gga_before_fix_has_no_coords() {
    // Quality 0, empty lat/lon fields — a fix-status update with no position. Status returns, coords are None.
    let body: &[u8] = b"GPGGA,123519,,,,,0,00,,,,,,,";
    let line = framed(body);
    match parse_line(&line) {
      Parsed::Status { fix, lat_e7, lon_e7, .. } => {
        assert!(!fix.has_fix());
        assert_eq!((lat_e7, lon_e7), (None, None));
      }
      other => panic!("expected Status, got {:?}", other),
    }
  }

  #[test]
  fn rmc_void_status_drops_position() {
    // A void ('V') RMC may still carry coordinate digits, but they are unreliable before a fix — drop them while
    // still returning the (zero) speed so the reader does not stall.
    let body: &[u8] = b"GPRMC,123519,V,4807.038,N,01131.000,E,000.0,084.4,230394,003.1,W";
    let line = framed(body);
    match parse_line(&line) {
      Parsed::Speed { lat_e7, lon_e7, .. } => assert_eq!((lat_e7, lon_e7), (None, None)),
      other => panic!("expected Speed, got {:?}", other),
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
      Parsed::Speed { speed_cm_s, .. } => {
        assert_eq!(speed_cm_s, MAX_SPEED_CM_S);
        assert!(speed_cm_s.checked_mul(36).is_some()); //  the downstream km/h math stays within u32.
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
      Parsed::Status { satellites, fix, .. } => {
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
      Parsed::Speed { speed_cm_s, .. } => assert_eq!(speed_cm_s, milliknots_to_cm_s(5_500)),
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

    // First fix is the RMC speed. The GGA had not been parsed yet, so sats/fix are still None here — but the
    // RMC carries its OWN position (status 'A'), so the location is present and atomic with the speed.
    let first = block_on(reader.next_fix()).unwrap();
    assert_eq!(first.speed_cm_s, milliknots_to_cm_s(22_400));
    assert_eq!(first.satellites, None);
    assert_eq!(first.lat_e7, Some(481_173_000));

    // Second fix resumes from the buffered tail: the GGA is parsed (updating cached status + position) and the
    // trailing VTG returns its speed — now carrying the satellites/fix AND the cached position the GGA supplied,
    // proving the GGA was not lost (the VTG itself has no coordinates).
    let second = block_on(reader.next_fix()).unwrap();
    assert_eq!(second.speed_cm_s, milli_kmh_to_cm_s(10_000));
    assert_eq!(second.satellites, Some(8));
    assert_eq!(second.fix.map(|f| f.raw), Some(1));
    assert_eq!(second.lat_e7, Some(481_173_000));
    assert_eq!(second.lon_e7, Some(115_166_667));
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
