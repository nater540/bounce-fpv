use std::env;
use std::fs;
use std::path::PathBuf;

// Standard cortex-m-rt linker setup, identical to the nrf-spike crate: copy the workspace-root memory.x into
// OUT_DIR and add OUT_DIR to the linker search path so link.x (pulled in by -Tlink.x in .cargo/config.toml)
// can INCLUDE it. applog itself is a lib, but the throwaway/downstream bins that link it inherit this search
// path, and keeping the build.rs here means a bare `cargo build -p applog` still finds memory.x.
fn main() {
  let out = PathBuf::from(env::var("OUT_DIR").unwrap());
  // memory.x lives at the workspace root, two levels up from this crate (crates/applog/).
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
