use std::env;
use std::fs;
use std::path::PathBuf;

// Standard cortex-m-rt linker setup, identical to nrf-spike/applog: copy the workspace-root memory.x into OUT_DIR and
// add OUT_DIR to the linker search path so link.x (pulled in by -Tlink.x in .cargo/config.toml) can INCLUDE it. This
// build.rs is per-crate and so covers all four binaries under src/bin/. No -Tlinkall.x (esp-hal only); no -Tdefmt.x
// either — none of these four bins link lora-phy, so nothing pulls defmt in (unlike lora-ping).
fn main() {
  let out = PathBuf::from(env::var("OUT_DIR").unwrap());
  // memory.x lives at the workspace root, two levels up from this crate (crates/truck-diag/).
  let memory_x = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap())
    .join("..")
    .join("..")
    .join("memory.x");
  fs::copy(&memory_x, out.join("memory.x")).expect("failed to copy memory.x into OUT_DIR");
  println!("cargo:rustc-link-search={}", out.display());
  // Re-run if the layout changes so a memory.x edit during on-target confirmation is picked up.
  println!("cargo:rerun-if-changed={}", memory_x.display());
  println!("cargo:rerun-if-changed=build.rs");
}
