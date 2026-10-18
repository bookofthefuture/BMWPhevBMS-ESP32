# Roadmap

## In progress / next
- **ZombieVerter state frame, car side.** The BMS listens for `0x3A1` (opmode, chgtyp). The matching ZombieVerter patch is built and
  needs merging in the ZombieVerter fork. See `zombieverter-bms-state-frame.md`.
- **Test on the car.** The ZombieVerter mode switching, burst sending of module commands (`mG < 3` in `sendcommand()`) and the
  rapid-charging fix have not been run on hardware. Verify with a sniffer that `0x3A1` follows `opmode`, and that Charge mode sends
  balancing commands.
- **Build the CAN map.** `can-map.md` covers the EV bus from the code only. Confirm entries on the car and add the vehicle CAN.

## Planned
- **Over-the-air firmware update.** The web UI's "Upload Update" only saves files to SPIFFS; the firmware is flashed over USB with
  `esptool`. Add a real OTA path (`Update.h`) for `.bin` uploads, using the second app slot in the default partition table, with a
  check that the file is an ESP32 app image, progress in the page, and a clear warning. Remove the dead `.bin`/`.hex` handling
  (`runUpdate()` does not exist; the `.bin` check in `/edit` never matches).
- **Customise the web UI.** Scope to be decided. Candidates: show the ZombieVerter mode and the BMS state (Ready / Drive / Charge) on
  the dashboard, make balancing status clearer, and general layout and styling. The UI lives in `BMWPhevBMS/data/` and ships as
  `spiffs.bin` in releases.

## Ideas
- Send DC-DC enabled as an extra byte in the ZombieVerter state frame if the BMS needs it.
- Confirm the board and partition scheme and set `FQBN` / `SPIFFS_*` in `.github/workflows/build.yml` to match.
