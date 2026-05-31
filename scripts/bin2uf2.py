#!/usr/bin/env python3
"""Minimal raw-binary -> UF2 converter for the nRF52840 Adafruit UF2 bootloader.

Usage: bin2uf2.py <input.bin> <output.uf2> <base_addr> <family_id>
The base address must match the app's flash origin in memory.x (e.g. 0x27000). Vendored so flashing
needs no external tool (uf2conv/cargo-hf2); the UF2 block format is fixed and simple. See the spec at
https://github.com/microsoft/uf2.
"""
import struct
import sys

UF2_MAGIC_START0 = 0x0A324655  # "UF2\n"
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END = 0x0AB16F30
UF2_FLAG_FAMILY_ID = 0x00002000  # the last header word is a familyID, not a fileSize
PAYLOAD = 256  # bytes of firmware per 512-byte UF2 block


def main() -> int:
  src, dst, base, family = sys.argv[1], sys.argv[2], int(sys.argv[3], 0), int(sys.argv[4], 0)
  data = open(src, "rb").read()
  nblocks = (len(data) + PAYLOAD - 1) // PAYLOAD
  with open(dst, "wb") as out:
    for i in range(nblocks):
      chunk = data[i * PAYLOAD:(i + 1) * PAYLOAD]
      # payloadSize is fixed at 256 for every block (the short final block is zero-padded), matching the
      # reference uf2conv.py byte-for-byte; the bootloader writes 256 bytes per block regardless.
      header = struct.pack(
        "<IIIIIIII",
        UF2_MAGIC_START0, UF2_MAGIC_START1, UF2_FLAG_FAMILY_ID,
        base + i * PAYLOAD, PAYLOAD, i, nblocks, family,
      )
      block = header + chunk + b"\x00" * (476 - len(chunk)) + struct.pack("<I", UF2_MAGIC_END)
      out.write(block)
  print(f"wrote {dst}: {nblocks} blocks, base {hex(base)}, family {hex(family)}")
  return 0


if __name__ == "__main__":
  sys.exit(main())
