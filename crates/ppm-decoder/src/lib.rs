//! Pure, hardware-free PPM frame decoder for the Skyzone goggle head-tracking stream.
//!
//! PPM carries several RC channels on one wire as rising-edge-positioned pulses: the rising-to-rising
//! interval IS the channel value (typically ~1000-2000 us, ~1500 = center), and a long "sync" gap
//! (> ~3 ms) marks the frame boundary. This crate is the decode state machine extracted from `ppm-diag`:
//! feed it successive rising-edge interval deltas in microseconds via [`PpmDecoder::feed`] and it returns a
//! completed [`Frame`] each time a sync gap closes the current frame. It owns no hardware — the caller
//! captures edges however it likes (esp-hal `wait_for_rising_edge` + `Instant`, or a timer-capture ISR) and
//! hands the deltas here, which is what lets both `ppm-diag` and `goggle-node` share one implementation.

#![no_std]

/// Maximum channels recorded per frame. A PPM frame is usually 4-12 channels; 16 leaves headroom. Pulses
/// beyond this within one frame are ignored until the next sync.
pub const MAX_CHANNELS: usize = 16;

/// Frame-sync threshold (microseconds). Any inter-pulse gap at least this long is treated as the sync gap:
/// it closes the current frame and resets the channel index. Real PPM sync gaps are typically > ~3 ms.
pub const DEFAULT_SYNC_GAP_US: u32 = 3_000;

/// Plausibility window (microseconds) for a real channel pulse interval. Intervals outside this band are
/// rejected as glitch/noise so a single spurious edge cannot shift every later channel. The standard servo
/// band is ~1000-2000 us; the wider 500-2500 window tolerates extended-range goggles without admitting noise.
pub const DEFAULT_MIN_PULSE_US: u32 = 500;
pub const DEFAULT_MAX_PULSE_US: u32 = 2_500;

/// A fully decoded PPM frame: the channel widths in microseconds plus how many were captured. Only the
/// first `count` entries of `channels` are meaningful; the rest are left at their previous value.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Frame {
  pub channels: [u16; MAX_CHANNELS],
  pub count: usize,
}

impl Frame {
  /// Returns the width (us) of channel `n` (0-based) if it was present in this frame. Goggle menus number
  /// channels from 1, so menu "channel 5 = pan" is `channel(4)` here.
  pub fn channel(&self, n: usize) -> Option<u16> {
    if n < self.count { Some(self.channels[n]) } else { None }
  }
}

/// Tunable thresholds for the decoder. `Default` matches the constants above, which are correct for the
/// Skyzone stream; override only if Phase 0 measurements call for a different sync gap or pulse band.
#[derive(Copy, Clone, Debug)]
pub struct Config {
  pub sync_gap_us: u32,
  pub min_pulse_us: u32,
  pub max_pulse_us: u32,
}

impl Default for Config {
  fn default() -> Self {
    Self { sync_gap_us: DEFAULT_SYNC_GAP_US, min_pulse_us: DEFAULT_MIN_PULSE_US, max_pulse_us: DEFAULT_MAX_PULSE_US }
  }
}

/// PPM decode state machine. Construct with [`PpmDecoder::new`], then call [`feed`](PpmDecoder::feed) once
/// per rising-edge interval. It accumulates channel widths until a sync gap closes the frame, at which point
/// `feed` returns `Some(Frame)`; mid-frame intervals return `None`.
pub struct PpmDecoder {
  config: Config,
  widths: [u16; MAX_CHANNELS],
  index: usize,
  // Set when an implausible mid-frame interval or a channel overflow corrupts the in-progress frame; the whole
  // frame is then suppressed at its sync gap rather than emitting shifted channels. Cleared on each sync reset.
  corrupt: bool,
}

impl PpmDecoder {
  /// New decoder with the given thresholds. Use `PpmDecoder::new(Config::default())` for the Skyzone stream.
  pub fn new(config: Config) -> Self {
    Self { config, widths: [0; MAX_CHANNELS], index: 0, corrupt: false }
  }

  /// Feed one rising-edge-to-rising-edge interval, in microseconds. Returns `Some(Frame)` when this delta is
  /// a sync gap that closes a clean, non-empty frame (the frame just completed), otherwise `None`. A sync gap
  /// closing an empty frame (e.g. the first one after boot) or a frame marked corrupt returns `None` instead.
  pub fn feed(&mut self, delta_us: u32) -> Option<Frame> {
    if delta_us >= self.config.sync_gap_us {
      // Sync gap: the frame that just ended is complete. Snapshot it only if it had channels and was never
      // marked corrupt, then reset (clearing the corrupt flag) to start accumulating the next frame.
      let count = self.index;
      let corrupt = self.corrupt;
      self.index = 0;
      self.corrupt = false;
      if count > 0 && !corrupt {
        return Some(Frame { channels: self.widths, count });
      }
      return None;
    }

    // Mid-frame interval = one channel value. If it is implausible (glitch/dropped channel) or we have run out
    // of channel slots, we cannot know which slot is wrong, so mark the whole frame corrupt — emitting a
    // shifted/extra channel would point goggle-node at the wrong axis. The frame is dropped at its sync; the
    // servo simply holds its last value for that ~frame, far safer than a wrong-axis jump.
    if delta_us >= self.config.min_pulse_us && delta_us <= self.config.max_pulse_us && self.index < MAX_CHANNELS {
      self.widths[self.index] = delta_us as u16;
      self.index += 1;
    } else {
      self.corrupt = true;
    }
    None
  }

  /// Number of channels accumulated in the in-progress (not yet synced) frame. Useful for diagnostics.
  pub fn pending(&self) -> usize {
    self.index
  }

  /// Discard any partially accumulated frame, e.g. after a long signal dropout. The next sync starts clean.
  pub fn reset(&mut self) {
    self.index = 0;
    self.corrupt = false;
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn decodes_a_simple_six_channel_frame() {
    let mut d = PpmDecoder::new(Config::default());
    // Six channel intervals, then a sync gap closing the frame.
    let widths = [1000u32, 1500, 2000, 1500, 1100, 1900];
    for w in widths {
      assert_eq!(d.feed(w), None);
    }
    let frame = d.feed(5_000).expect("sync gap should yield a frame");
    assert_eq!(frame.count, 6);
    assert_eq!(frame.channels[..6], [1000, 1500, 2000, 1500, 1100, 1900]);
    assert_eq!(frame.channel(4), Some(1100));
  }

  #[test]
  fn out_of_band_interval_suppresses_the_whole_frame() {
    let mut d = PpmDecoder::new(Config::default());
    assert_eq!(d.feed(1500), None);
    assert_eq!(d.feed(100), None); //  too short — marks the frame corrupt rather than silently shifting.
    assert_eq!(d.feed(1600), None);
    // The corrupt frame is dropped at its sync gap so no shifted/wrong-axis channels are emitted.
    assert_eq!(d.feed(4_000), None);

    // The next clean frame decodes normally — the corrupt flag was cleared on the sync reset.
    assert_eq!(d.feed(1000), None);
    assert_eq!(d.feed(1500), None);
    let frame = d.feed(4_000).expect("clean frame after a corrupt one should emit");
    assert_eq!(frame.count, 2);
    assert_eq!(frame.channels[..2], [1000, 1500]);
  }

  #[test]
  fn channel_overflow_suppresses_the_whole_frame() {
    let mut d = PpmDecoder::new(Config::default());
    // One more interval than MAX_CHANNELS slots: the overflow marks the frame corrupt, so it is dropped.
    for _ in 0..(MAX_CHANNELS + 1) {
      assert_eq!(d.feed(1500), None);
    }
    assert_eq!(d.feed(4_000), None);
  }

  #[test]
  fn empty_sync_does_not_emit() {
    let mut d = PpmDecoder::new(Config::default());
    assert_eq!(d.feed(4_000), None); //  first sync after boot, nothing accumulated.
  }
}
