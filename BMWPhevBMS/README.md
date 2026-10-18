# BMWPhevBMS-ESP32
Port of the Spaceballs edition of SIMPBMS for BMW Phev Modules and ESP32

## Releases

GitHub Actions builds the sketch on every push and pull request (`.github/workflows/build.yml`) and attaches the binaries as a build artifact. Pushing a tag such as `v1.0.0` also publishes a GitHub release containing:

- `*-firmware.bin`: the application only, flash at `0x10000`
- `*-spiffs.bin`: the web UI files from `data/`, flash at `0x290000`
- `*-full.bin`: bootloader, partitions, app and web UI in one image, flash at `0x0`
- `FLASHING.txt` and `SHA256SUMS.txt`

Flash with `esptool.py --chip esp32 --port <PORT> write_flash 0x10000 <firmware.bin>`.

The build assumes a generic ESP32 dev board (`esp32:esp32:esp32`, default 4MB partition table). If your board or partition scheme differs, change `FQBN` and the `SPIFFS_*` values at the top of the workflow.
