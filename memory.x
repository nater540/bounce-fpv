/* nRF52840 (1 MB flash / 256 KB RAM) memory layout, carved around the s140 SoftDevice and the
   Adafruit UF2 bootloader that the Pro Micro nRF52840 ships with. cortex-m-rt's link.x INCLUDEs this. */

/* Flash map (low -> high):
     0x00000000  MBR (Nordic master boot record, 4 KB)
     0x00001000  s140 SoftDevice
     0x00027000  APPLICATION  <- our firmware (FLASH ORIGIN below)
     0x000F4000  Adafruit UF2 bootloader + its config/settings region, up to the 1 MB top

   FLASH ORIGIN: CONFIRMED on-target from the Nice!Nano INFO_UF2.TXT — the resident SoftDevice is s140
   *v6.1.1*, whose SD region (MBR + SoftDevice) ends at 0x26000, so the application starts at 0x26000.
   (An earlier guess of 0x27000 — the v7.0.1 the nrf-softdevice-s140 crate bundles — left the app 4 KB
   too high, so the bootloader found no vector table at 0x26000 and stayed in DFU.) NOTE: the crate's Rust
   bindings are v7.0.1 while the resident SD is v6.1.1; the core enable/power/clock SVCs are compatible
   across v6/v7, but align the bindings to v6.1.1 (or update the board SD to v7) before relying on BLE. */
MEMORY
{
  /* APPLICATION flash: 0x26000 .. 0xF4000 = 0xCE000 (824 KB), leaving the bootloader region untouched. */
  FLASH : ORIGIN = 0x00026000, LENGTH = 0xCE000

  /* RAM: the SoftDevice reserves the low RAM; the app starts at 0x20004180 for a typical s140 config.
     This is the documented starting estimate — the SoftDevice reports the *actual* required base back
     from sd_softdevice_enable, and Phase A is where we read that value and finalize it. If the SD asks
     for a higher base (more BLE resources configured), raise RAM ORIGIN and shrink LENGTH to match. */
  RAM : ORIGIN = 0x20004180, LENGTH = 256K - 0x4180
}
