fn main() {
  linker_be_nice();
  // Keep linkall.x last so esp-hal's memory layout wins over anything earlier. This build.rs is per-crate and so
  // covers all four binaries under src/bin/.
  println!("cargo:rustc-link-arg=-Tlinkall.x");
}

// Registers a small error-handling script with the linker so common undefined-symbol
// failures (missing linker script, missing esp-rtos scheduler, etc.) print an actionable
// hint instead of a raw symbol name. Copied from the esp-generate template.
fn linker_be_nice() {
  let args: Vec<String> = std::env::args().collect();
  if args.len() > 1 {
    let kind = &args[1];
    let what = &args[2];

    match kind.as_str() {
      "undefined-symbol" => match what.as_str() {
        what if what.starts_with("_defmt_") => {
          eprintln!();
          eprintln!("`defmt` not found - add defmt.x as a linker script and `use defmt_rtt as _;`");
          eprintln!();
        }
        "_stack_start" => {
          eprintln!();
          eprintln!("Is the linker script `linkall.x` missing?");
          eprintln!();
        }
        what if what.starts_with("esp_rtos_") => {
          eprintln!();
          eprintln!("esp-rtos scheduler not initialized - call esp_rtos::start(...) before spawning.");
          eprintln!();
        }
        _ => (),
      },
      _ => {
        std::process::exit(1);
      }
    }

    std::process::exit(0);
  }

  println!(
    "cargo:rustc-link-arg=--error-handling-script={}",
    std::env::current_exe().unwrap().display()
  );
}
