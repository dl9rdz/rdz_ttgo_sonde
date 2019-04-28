# rdz_ttgo_sonde

See <https://dl9rdz.github.io/rdz_ttgo_sonde/download.html> for automated builds.

For downloading these to the ESP32 board, use something like

```
esptool --chip esp32 --port /dev/cu.SLAB_USBtoUART --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0x1000 <filename.bin>
```

Most important part is the start offset 0x1000.

The binary image contains everything including the flash file system, so all configurations options will be overwritten.
