use std::env;
use std::path::PathBuf;

// Generates the no_std/no-alloc Rust types from headtrack.proto via micropb-gen. Both Control and Telemetry are
// scalar-only (uint32), so no container type (heapless/arrayvec/alloc) is configured — that is only required for
// string/bytes/repeated/map fields, which this schema deliberately avoids to keep packets and the build minimal.
fn main() {
  println!("cargo:rerun-if-changed=headtrack.proto");

  let out_dir = PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR not set"));
  let out_file = out_dir.join("headtrack.rs");

  let generator = micropb_gen::Generator::new();
  generator
    .compile_protos(&["headtrack.proto"], out_file)
    .expect("micropb-gen failed to compile headtrack.proto");
}
