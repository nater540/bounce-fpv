use std::env;
use std::fs;
use std::path::PathBuf;

// Standard cortex-m-rt linker setup, identical to lora-ping/truck-diag: copy the workspace-root memory.x into OUT_DIR
// and add OUT_DIR to the linker search path so link.x (pulled in by -Tlink.x in .cargo/config.toml) can INCLUDE it.
// The `-Tdefmt.x` script lora-phy's unconditional defmt dependency needs is added globally in .cargo/config.toml (it
// pairs with applog's defmt-rtt #[global_logger]), so it is no longer handled per-binary here.
fn main() {
  let out = PathBuf::from(env::var("OUT_DIR").unwrap());
  // memory.x lives at the workspace root, two levels up from this crate (crates/truck-node/).
  let memory_x = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap())
    .join("..")
    .join("..")
    .join("memory.x");
  fs::copy(&memory_x, out.join("memory.x")).expect("failed to copy memory.x into OUT_DIR");
  println!("cargo:rustc-link-search={}", out.display());
  // Re-run if the layout changes so a memory.x edit during on-target confirmation is picked up.
  println!("cargo:rerun-if-changed={}", memory_x.display());
  println!("cargo:rerun-if-changed=build.rs");

  // LoRa binding phrase (link-id): the firmware reads env!("BINDING_PHRASE") at compile time to derive its UID +
  // link-id + CRC filter. Provide a default here so a plain `cargo build` works; an explicit shell-exported
  // BINDING_PHRASE is forwarded straight to rustc and takes precedence (we only emit the default when it is unset).
  // The goggle and truck MUST build with the SAME phrase to talk — flash two pairs with BINDING_PHRASE="pair-a"/"pair-b".
  println!("cargo:rerun-if-env-changed=BINDING_PHRASE");
  if env::var("BINDING_PHRASE").is_err() {
    println!("cargo:rustc-env=BINDING_PHRASE=fabulous-default");
  }
}
