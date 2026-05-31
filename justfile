# https://just.systems
#
# nRF52840 (Pro Micro) build / flash / monitor. There is no debug probe: flashing copies a `.uf2` onto the
# Adafruit bootloader's mass-storage volume (double-tap RESET to mount it), and the serial monitor reads the
# app's own USB CDC port. `flash` builds + converts + copies; `monitor` opens the console; `flash-monitor`
# does both. All default to the Phase A `nrf-spike` binary; pass another bin name for the diagnostics/nodes.

target := "thumbv7em-none-eabihf"
# memory.x is the SINGLE SOURCE OF TRUTH for the app flash origin (CONFIRMED s140 v6.1.1 on the Nice!Nano
# via INFO_UF2.TXT -> 0x26000). DERIVE flash_base from its FLASH ORIGIN at parse time so the two can never
# drift (a past mismatch left the board stuck in DFU). bin2uf2.py parses the 0x00026000 form fine (int(.,0)).
flash_base := `sed -n 's/.*FLASH[[:space:]]*:[[:space:]]*ORIGIN[[:space:]]*=[[:space:]]*\(0x[0-9A-Fa-f]*\).*/\1/p' memory.x`
family := "0xADA52840"  # nRF52840 UF2 family id
reldir := "target" / target / "release"

default:
  @just --list

# Build a binary for the nRF target by its bin name (default: the Phase A spike). `--bin` works workspace-wide
# now that Phase B1 removed the esp crates (the critical-section feature collision is gone), so multi-bin
# crates resolve too: `just flash servo` / `imu` / `oled` / `gps` pick truck-diag's bins by name.
build bin="nrf-spike" features="":
  cargo build --release --bin {{bin}} {{ if features != "" { "--features " + features } else { "" } }}

# Build, then convert the ELF to a flashable .uf2 ({{reldir}}/<bin>.uf2).
uf2 bin="nrf-spike" features="": (build bin features)
  rust-objcopy -O binary "{{reldir}}/{{bin}}" "{{reldir}}/{{bin}}.bin"
  python3 scripts/bin2uf2.py "{{reldir}}/{{bin}}.bin" "{{reldir}}/{{bin}}.uf2" {{flash_base}} {{family}}

# Reboot a running-app board into the Adafruit UF2 bootloader, button-free, via the 1200-baud USB-CDC
# "touch": opening the CDC port at 1200 baud issues SET_LINE_CODING, which the firmware watches for and
# answers by setting GPREGRET=0x57 and resetting into DFU. On macOS `stty -f <port> 1200` performs exactly
# that open-at-1200 (no DTR toggle needed — the firmware keys on baud alone). With no arg it auto-selects
# the newest /dev/cu.usbmodem* (same logic as `monitor`); pass a port to override.
reboot-bootloader port="":
  #!/usr/bin/env bash
  set -euo pipefail
  port="{{port}}"
  if [ -z "$port" ]; then
    port="$(ls -t /dev/cu.usbmodem* 2>/dev/null | head -1 || true)"
    if [ -z "$port" ]; then
      echo "No /dev/cu.usbmodem* found. Is the board plugged in and running the app (or already in bootloader)?" >&2
      exit 1
    fi
  fi
  echo "Touching $port at 1200 baud to request the UF2 bootloader..."
  # stty opens the device and sets 1200 baud (SET_LINE_CODING); the firmware reboots into DFU on seeing it.
  stty -f "$port" 1200 || true
  echo "Done. The board should re-appear as the UF2 bootloader volume (INFO_UF2.TXT) in a second or two."

# Flash: build + uf2 + copy onto the mounted UF2 bootloader volume. Button-free: if a running-app CDC port
# is present, do the 1200-baud touch (reboot into the bootloader) and poll up to ~10s for the volume to
# mount; if the board is already in the bootloader (no app port), skip the touch and copy straight away.
flash bin="nrf-spike" features="": (uf2 bin features)
  #!/usr/bin/env bash
  set -euo pipefail
  uf2="{{reldir}}/{{bin}}.uf2"
  find_vol() {
    for d in /Volumes/*; do
      if [ -f "$d/INFO_UF2.TXT" ]; then echo "$d"; return 0; fi
    done
    return 1
  }
  vol="$(find_vol || true)"
  if [ -z "$vol" ]; then
    # No bootloader volume yet. If a running app is exposing a CDC port, trigger the 1200-baud touch and
    # wait for the bootloader to mount; otherwise fall through to the manual double-tap instruction below.
    port="$(ls -t /dev/cu.usbmodem* 2>/dev/null | head -1 || true)"
    if [ -n "$port" ]; then
      echo "App port $port present; touching at 1200 baud to enter the UF2 bootloader..."
      stty -f "$port" 1200 || true
      echo "Waiting up to ~10s for the UF2 bootloader volume to mount..."
      for _ in $(seq 1 20); do
        sleep 0.5
        vol="$(find_vol || true)"
        if [ -n "$vol" ]; then break; fi
      done
    fi
  fi
  if [ -z "$vol" ]; then
    echo "No UF2 bootloader volume found. Double-tap RESET on the board, then re-run 'just flash {{bin}}'." >&2
    exit 1
  fi
  echo "Flashing $uf2 -> $vol"
  # The board reboots and ejects the volume mid-copy, so a copy error here is benign. `-X` skips macOS extended
  # attributes — those are written AFTER the data and fail with "Device not configured" once the volume ejects,
  # which is just noise; the image itself is already on the board by then.
  cp -X "$uf2" "$vol/" || true
  echo "Flashed. The board is rebooting into the app."

# LoRa link test, the two roles of the `lora-ping` bin. Flash ONE board with `flash-pinger` and the OTHER with
# `flash-ponger`, then power both: each mirrors its status to its SSD1306 (RTT/RSSI/seq on the pinger, heard
# seq/RSSI/count on the ponger), so no serial tether is needed. These are thin aliases over `flash` — the role
# is the `ponger` Cargo feature, easy to forget on the bare `just flash lora-ping ponger` form.
flash-pinger: (flash "lora-ping")
flash-ponger: (flash "lora-ping" "ponger")

# Open the USB CDC serial monitor. With no arg it picks the most-recently-enumerated port (the board you just
# flashed/plugged); permanent fixtures like docks sort older. Pass a port to override: just monitor <port>.
monitor port="":
  #!/usr/bin/env bash
  set -euo pipefail
  port="{{port}}"
  if [ -z "$port" ]; then
    port="$(ls -t /dev/cu.usbmodem* 2>/dev/null | head -1 || true)"
    if [ -z "$port" ]; then
      echo "No /dev/cu.usbmodem* found. Is the board plugged in and running the app?" >&2
      exit 1
    fi
    if [ "$(ls /dev/cu.usbmodem* 2>/dev/null | wc -l | tr -d ' ')" -gt 1 ]; then
      echo "Multiple ports present; auto-selected newest: $port  (override: just monitor <port>)" >&2
    fi
  fi
  echo "Monitoring $port  (exit screen with: Ctrl-A then K then Y)"
  exec screen "$port" 115200

# All-in-one: flash, then open the monitor on the freshly-rebooted board (the newest port).
flash-monitor bin="nrf-spike" features="": (flash bin features)
  #!/usr/bin/env bash
  set -euo pipefail
  echo "Waiting ~3s for the board to reboot and its CDC port to enumerate..."
  sleep 3
  exec just monitor
