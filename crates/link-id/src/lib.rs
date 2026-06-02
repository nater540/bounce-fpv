//! ExpressLRS-style binding for the point-to-point LoRa link.
//!
//! A human-readable *binding phrase* is hashed at compile time into a 6-byte [`Binding::uid`], from which a
//! 2-byte [`Binding::link_id`] prefix and a 16-bit [`Binding::crc_init`] CRC initializer are derived. The wire
//! framing wraps a payload as `[link_id:2][payload][crc16:2]`, and [`deframe`] accepts a frame only if the
//! link-id matches *and* the UID-seeded CRC checks out — so a node silently drops any frame from a differently
//! bound pair (or random RF garbage). This mirrors how ExpressLRS turns its binding-phrase UID into a per-link
//! CRC initializer used as de-facto address filtering, letting nearby pairs share a frequency without crosstalk.
//!
//! Like ELRS, the phrase is for **anti-collision, not security** — the hash is just a stable, well-mixed map
//! from phrase to UID. We use FNV-1a (a tiny `const fn`, zero dependencies) rather than MD5; exact byte-for-byte
//! ELRS compatibility is irrelevant for a closed goggle↔truck pair. If real-ELRS interop were ever needed, swap
//! the body of [`derive`] for an MD5 of the phrase (first 6 bytes of the digest) — `frame`/`deframe`/`crc16` are
//! unchanged, they only consume the derived `link_id`/`crc_init`.

#![cfg_attr(not(test), no_std)]

use heapless::Vec;

/// The per-link identity derived from a binding phrase. `uid` is the 6-byte root; `link_id` is the fast-reject
/// 2-byte frame prefix; `crc_init` seeds the frame CRC so two phrases yield different CRCs over identical bytes.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Binding {
  pub uid: [u8; 6],
  pub link_id: [u8; 2],
  pub crc_init: u16,
}

/// Bytes [`frame`] adds around a payload: the 2-byte link-id prefix plus the trailing 2-byte CRC16.
pub const FRAME_OVERHEAD: usize = 4;

// FNV-1a (64-bit) constants. FNV is a non-cryptographic hash, which is exactly right here: a binding phrase is an
// address namespace, not a secret, and FNV is a short `const fn` with no dependency so `derive` runs at compile time.
const FNV_OFFSET_BASIS: u64 = 0xcbf2_9ce4_8422_2325;
const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;

/// FNV-1a hash of `bytes` from a given seed. `const` so [`derive`] is fully compile-time.
const fn fnv1a(bytes: &[u8], seed: u64) -> u64 {
  let mut hash = seed;
  let mut i = 0;
  while i < bytes.len() {
    hash ^= bytes[i] as u64;
    hash = hash.wrapping_mul(FNV_PRIME);
    i += 1;
  }
  hash
}

/// murmur3 `fmix64` avalanche finalizer. FNV-1a alone barely changes its *high* bytes when only the phrase's last
/// byte differs (e.g. "pair-a" vs "pair-b"), so we run each hash through this before slicing bytes — now a one-bit
/// input change flips ~half the output bits, and the low bytes we take below are well separated.
const fn fmix64(mut h: u64) -> u64 {
  h ^= h >> 33;
  h = h.wrapping_mul(0xff51_afd7_ed55_8ccd);
  h ^= h >> 33;
  h = h.wrapping_mul(0xc4ce_b9fe_1a85_ec53);
  h ^= h >> 33;
  h
}

/// Derives the [`Binding`] for a phrase at compile time. Two FNV-1a passes with different seeds, each avalanched
/// through [`fmix64`], give 16 well-mixed bytes; the low 3 of each become the 6-byte UID. `link_id` is the first
/// two UID bytes (a cheap prefix to reject foreign frames before the CRC walk); `crc_init` is the last two UID
/// bytes — the ELRS "CRC-initializer-as-address" trick, so even two phrases that collided on `link_id` would
/// still produce mismatching CRCs over the same payload.
pub const fn derive(phrase: &str) -> Binding {
  let bytes = phrase.as_bytes();
  let h0 = fmix64(fnv1a(bytes, FNV_OFFSET_BASIS));
  // A distinct seed for the second pass so the two halves of the UID are independent rather than correlated.
  let h1 = fmix64(fnv1a(bytes, FNV_OFFSET_BASIS ^ 0x00ff_00ff_00ff_00ff));
  let uid = [
    h0 as u8, (h0 >> 8) as u8, (h0 >> 16) as u8,
    h1 as u8, (h1 >> 8) as u8, (h1 >> 16) as u8,
  ];
  let link_id = [uid[0], uid[1]];
  let crc_init = ((uid[4] as u16) << 8) | (uid[5] as u16);
  Binding { uid, link_id, crc_init }
}

/// CRC-16/CCITT (polynomial 0x1021) over `data`, starting from `crc_init`. A plain bit-loop — the frames here are
/// only a couple dozen bytes at tens of Hz, so a lookup table would buy nothing.
pub fn crc16(crc_init: u16, data: &[u8]) -> u16 {
  let mut crc = crc_init;
  for &byte in data {
    crc ^= (byte as u16) << 8;
    let mut bit = 0;
    while bit < 8 {
      crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
      bit += 1;
    }
  }
  crc
}

/// Wraps `payload` into `out` as `[link_id:2][payload][crc16:2]`, where the CRC covers the link-id *and* payload
/// (so a flipped id byte also fails the CRC). Clears `out` first. Returns `Err(())` if `out`'s capacity `N` can't
/// hold `payload.len() + FRAME_OVERHEAD` bytes — the single, self-evident failure mode, so a unit error suffices.
#[allow(clippy::result_unit_err)]
pub fn frame<const N: usize>(binding: &Binding, payload: &[u8], out: &mut Vec<u8, N>) -> Result<(), ()> {
  out.clear();
  out.extend_from_slice(&binding.link_id).map_err(|_| ())?;
  out.extend_from_slice(payload).map_err(|_| ())?;
  let crc = crc16(binding.crc_init, out);
  out.extend_from_slice(&crc.to_be_bytes()).map_err(|_| ())
}

/// Validates a received `frame` and returns the inner payload, or `None` if it's too short, the link-id prefix
/// doesn't match, or the UID-seeded CRC fails — i.e. the frame belongs to a different pair or is corrupt. The
/// link-id check is a cheap reject before the CRC walk.
pub fn deframe<'a>(binding: &Binding, frame: &'a [u8]) -> Option<&'a [u8]> {
  if frame.len() < FRAME_OVERHEAD {
    return None;
  }
  let (body, crc_bytes) = frame.split_at(frame.len() - 2);
  if body[0] != binding.link_id[0] || body[1] != binding.link_id[1] {
    return None;
  }
  let want = u16::from_be_bytes([crc_bytes[0], crc_bytes[1]]);
  if crc16(binding.crc_init, body) != want {
    return None;
  }
  Some(&body[2..])
}

#[cfg(test)]
mod tests {
  use super::*;

  const A: Binding = derive("pair-a");
  const B: Binding = derive("pair-b");

  #[test]
  fn distinct_phrases_differ() {
    // Different phrases must yield a different identity on every derived field, or pairs would crosstalk.
    assert_ne!(A.uid, B.uid);
    assert_ne!(A.link_id, B.link_id);
    assert_ne!(A.crc_init, B.crc_init);
  }

  #[test]
  fn same_phrase_is_stable() {
    assert_eq!(derive("pair-a"), A);
  }

  #[test]
  fn round_trips_same_binding() {
    let payload = [0x08, 0xDC, 0x0B, 0x10, 0xD0, 0x0F];
    let mut framed: Vec<u8, 32> = Vec::new();
    frame(&A, &payload, &mut framed).unwrap();
    assert_eq!(framed.len(), payload.len() + FRAME_OVERHEAD);
    assert_eq!(deframe(&A, &framed), Some(&payload[..]));
  }

  #[test]
  fn rejects_foreign_binding() {
    let payload = [1, 2, 3, 4];
    let mut framed: Vec<u8, 32> = Vec::new();
    frame(&A, &payload, &mut framed).unwrap();
    // A frame built for pair A must not deframe under pair B's binding.
    assert_eq!(deframe(&B, &framed), None);
  }

  #[test]
  fn rejects_corruption() {
    let payload = [9, 9, 9];
    let mut framed: Vec<u8, 32> = Vec::new();
    frame(&A, &payload, &mut framed).unwrap();
    let mut bad: Vec<u8, 32> = Vec::new();
    bad.extend_from_slice(&framed).unwrap();
    let last = bad.len() - 3; // flip a payload byte; the CRC must catch it
    bad[last] ^= 0xFF;
    assert_eq!(deframe(&A, &bad), None);
  }

  #[test]
  fn empty_payload_still_frames() {
    // A zero-length proto payload (all-default Telemetry) still produces a non-empty, valid frame — which is why
    // framing incidentally retires the old 0-byte-LoRa-payload workaround.
    let mut framed: Vec<u8, 8> = Vec::new();
    frame(&A, &[], &mut framed).unwrap();
    assert_eq!(framed.len(), FRAME_OVERHEAD);
    assert_eq!(deframe(&A, &framed), Some(&[][..]));
  }

  #[test]
  fn frame_overflow_errs() {
    let mut tiny: Vec<u8, 4> = Vec::new();
    // 2 payload + 4 overhead = 6 > capacity 4 → Err, not a panic.
    assert!(frame(&A, &[1, 2], &mut tiny).is_err());
  }
}
