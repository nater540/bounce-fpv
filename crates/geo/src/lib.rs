//! GPS navigation math: distance + bearing from a "home" point to a current point, both given as integer
//! degrees × 1e7 (the domain the `gps` crate emits). Pure, `no_std`, no I/O, host-tested — the truck node
//! calls [`nav`] once per fix and ships only the resulting distance/bearing over LoRa, so the goggle never
//! does this math.
//!
//! Two deliberate numerical choices, both load-bearing:
//!   1. The coordinate **difference** is taken in the **integer i64 domain** ([`nav`] step 1), and only the small
//!      delta is ever cast to f32. Differencing absolute coordinates in f32 would be catastrophic: deg×1e7 is
//!      ~4.8e8 for 48°, which already eats f32's ~7 significant digits, so subtracting two nearby f32 coords
//!      destroys exactly the precision that matters at RC-truck range. i64 deltas also survive an antimeridian
//!      pair or a garbage 0,0 current point that would overflow an i32 subtraction. (The absolute home latitude
//!      is converted to f32 once, for the `cos(lat)` longitude scale — a benign exception: the cosine needs only
//!      a few digits of latitude, and no *subtraction* of large f32 values is involved.)
//!   2. Distance/bearing use an **equirectangular** (flat-tangent-plane) approximation, not haversine. Its
//!      error vs haversine grows like (d/R)² — at a few km that is sub-millimeter — so at the hundreds-of-
//!      meters-to-a-couple-km range of "find my truck" it is indistinguishable from haversine while costing
//!      one `cosf` of the (per-session-constant) home latitude instead of haversine's extra trig.

#![no_std]

use core::f32::consts::PI;

/// Mean meters per degree of latitude (WGS84-ish). Also the meters-per-degree of longitude AT the equator;
/// scaled by `cos(latitude)` for the east/west axis. One constant for both axes keeps the model self-consistent.
const M_PER_DEG: f32 = 111_320.0;

/// Distance (meters) below which the bearing is meaningless: within a few meters of home, GPS noise (±2-5 m)
/// swings the bearing through all 360°, so callers should render "AT HOME" rather than a spinning needle.
pub const AT_HOME_M: u32 = 5;

/// A geographic coordinate in the integer 1e7 domain: degrees × 1e7, +North / +East. This is the SAME domain
/// the `gps` crate decodes NMEA into, so absolute coordinates never pass through f32. `Copy`/`Eq` so it can
/// ride a `Signal` and participate in a dirty-check.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub struct CoordE7 {
  pub lat_e7: i32,
  pub lon_e7: i32,
}

/// Derived navigation from a home point to a current point: whole-meter distance and a compass bearing in
/// `0..=359` degrees (0 = due North, 90 = East). Integer fields so it stays `Eq` for downstream dirty-checks.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub struct Nav {
  pub distance_m: u32,
  pub bearing_deg: u16,
}

/// Computes distance + initial bearing from `home` to `current` using the equirectangular approximation
/// described in the module docs. The coordinate subtraction happens in i64 (no f32 cancellation); only the
/// small resulting delta is converted to meters. Bearing is the compass direction one would walk from home to
/// reach the current point, folded into `0..=359` (0 = North). At `distance_m < AT_HOME_M` the bearing is not
/// meaningful (callers should suppress it), but a deterministic value is still returned.
pub fn nav(home: CoordE7, current: CoordE7) -> Nav {
  // 1. Difference in the integer domain. i64 so an antimeridian pair (±180°, ~3.6e9 apart) or a garbage 0,0
  //    current paired with a real home cannot overflow the i32 subtraction; the result is small in practice.
  let dlat_e7 = current.lat_e7 as i64 - home.lat_e7 as i64;
  let dlon_e7 = current.lon_e7 as i64 - home.lon_e7 as i64;

  // 2. The deltas are now tiny, so f32's ~7 significant digits are ample. Convert delta-e7 -> delta-degrees.
  let dlat_deg = dlat_e7 as f32 / 1e7;
  let dlon_deg = dlon_e7 as f32 / 1e7;

  // 3. Degrees -> meters on the local tangent plane. Latitude scales straight by M_PER_DEG; longitude shrinks
  //    by cos(latitude). Use HOME's latitude for the cosine (constant per session, and the truck barely moves
  //    in latitude over a few km), feeding it in radians.
  let home_lat_rad = (home.lat_e7 as f32 / 1e7) * (PI / 180.0);
  let dlat_m = dlat_deg * M_PER_DEG;
  let dlon_m = dlon_deg * M_PER_DEG * libm::cosf(home_lat_rad);

  // 4. Distance is the hypotenuse of the two meter-deltas, rounded and saturated into u32 (it can never
  //    approach u32::MAX at any real range, but the cast is guarded so a corrupt coordinate cannot wrap).
  let d = libm::roundf(libm::hypotf(dlat_m, dlon_m));
  let distance_m = if d <= 0.0 {
    0
  } else if d >= u32::MAX as f32 {
    u32::MAX
  } else {
    d as u32
  };

  // 5. Bearing: atan2(East, North) gives a compass angle measured clockwise from North (0 = N, 90 = E). Map
  //    the atan2 range [-180, 180] into [0, 360) by adding 360 to the negatives, round, and fold a rounded-up
  //    360 back to 0 so the result is always 0..=359.
  let mut deg = libm::atan2f(dlon_m, dlat_m) * (180.0 / PI);
  if deg < 0.0 {
    deg += 360.0;
  }
  let mut b = libm::roundf(deg) as i32;
  if b >= 360 {
    b -= 360;
  }
  if b < 0 {
    b += 360;
  }
  Nav { distance_m, bearing_deg: b as u16 }
}

#[cfg(test)]
mod tests {
  use super::*;

  // A latitude where cos(lat) is meaningfully != 1, so the east/west longitude scaling is actually exercised.
  const LAT_DEG: f32 = 48.0;
  const LAT_E7: i32 = 480_000_000;
  const LON_E7: i32 = 110_000_000; // 11.0°E

  /// Builds a CoordE7 `north_m` meters north and `east_m` meters east of (LAT_E7, LON_E7), inverting the same
  /// equirectangular relation the implementation uses so the tests state displacements in meters, not e7.
  fn offset(north_m: f32, east_m: f32) -> CoordE7 {
    let dlat_deg = north_m / M_PER_DEG;
    let dlon_deg = east_m / (M_PER_DEG * libm::cosf(LAT_DEG * PI / 180.0));
    CoordE7 {
      lat_e7: LAT_E7 + libm::roundf(dlat_deg * 1e7) as i32,
      lon_e7: LON_E7 + libm::roundf(dlon_deg * 1e7) as i32,
    }
  }

  fn home() -> CoordE7 {
    CoordE7 { lat_e7: LAT_E7, lon_e7: LON_E7 }
  }

  #[test]
  fn same_point_is_zero_distance() {
    let n = nav(home(), home());
    assert_eq!(n.distance_m, 0);
    assert!(n.bearing_deg < 360);
  }

  #[test]
  fn due_north_100m() {
    let n = nav(home(), offset(100.0, 0.0));
    assert!((n.distance_m as i32 - 100).abs() <= 2, "dist {}", n.distance_m);
    // Bearing ~0 (North); fold the wrap so 359 reads as ~0.
    let off = n.bearing_deg.min(360 - n.bearing_deg);
    assert!(off <= 2, "bearing {}", n.bearing_deg);
  }

  #[test]
  fn due_east_100m() {
    let n = nav(home(), offset(0.0, 100.0));
    assert!((n.distance_m as i32 - 100).abs() <= 2, "dist {}", n.distance_m);
    assert!((n.bearing_deg as i32 - 90).abs() <= 2, "bearing {}", n.bearing_deg);
  }

  #[test]
  fn due_south_and_west_quadrants() {
    let s = nav(home(), offset(-100.0, 0.0));
    assert!((s.bearing_deg as i32 - 180).abs() <= 2, "south bearing {}", s.bearing_deg);
    let w = nav(home(), offset(0.0, -100.0));
    assert!((w.bearing_deg as i32 - 270).abs() <= 2, "west bearing {}", w.bearing_deg);
  }

  #[test]
  fn intercardinal_quadrants_land_in_the_right_bin() {
    // NE -> (0,90), SE -> (90,180), SW -> (180,270), NW -> (270,360).
    assert!((1..=89).contains(&(nav(home(), offset(100.0, 100.0)).bearing_deg as i32)));
    assert!((91..=179).contains(&(nav(home(), offset(-100.0, 100.0)).bearing_deg as i32)));
    assert!((181..=269).contains(&(nav(home(), offset(-100.0, -100.0)).bearing_deg as i32)));
    assert!((271..=359).contains(&(nav(home(), offset(100.0, -100.0)).bearing_deg as i32)));
  }

  #[test]
  fn ten_km_north_holds_scale_without_overflow() {
    // A long baseline so a scale/overflow bug is visible (short-range tolerances would hide it). Equirectangular
    // north distance is exactly dlat_deg * M_PER_DEG, so the round-trip through offset() must come back within 1 m.
    let n = nav(home(), offset(10_000.0, 0.0));
    assert!((n.distance_m as i32 - 10_000).abs() <= 1, "dist {}", n.distance_m);
    let off = n.bearing_deg.min(360 - n.bearing_deg);
    assert!(off <= 1, "bearing {}", n.bearing_deg);
  }

  #[test]
  fn ten_km_east_holds_scale_without_overflow() {
    let n = nav(home(), offset(0.0, 10_000.0));
    assert!((n.distance_m as i32 - 10_000).abs() <= 2, "dist {}", n.distance_m);
    assert!((n.bearing_deg as i32 - 90).abs() <= 1, "bearing {}", n.bearing_deg);
  }
}
