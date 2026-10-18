# Brief: add a "BMS state" CAN frame to the ZombieVerter firmware

## Goal
Patch the ZombieVerter firmware (fork: https://github.com/bookofthefuture/zombieverter, STM32, libopencm3/libopeninv)
so it broadcasts its operating state on a **fixed CAN frame every 100 ms**, on the CAN bus selected by the existing
`BMSCan` parameter. A separate ESP32 BMW PHEV BMS (this repo, `BMWPhevBMS/BMWPhevBMS.ino`) reads the frame to
switch between Ready / Drive / Charge (Charge is what makes it send cell-balancing commands).

The frame must NOT rely on the user-configurable CanMap (`canMap->SendAll()`): that can only target one bus
(`CanMapCan`), and the user already uses it on the car CAN.

## Frame spec
- CAN ID: `0x3A1` (11-bit standard), DLC 8, 500 kbit/s (same as existing buses)
- Period: 100 ms

| Byte | Content | Source |
|------|---------|--------|
| 0 | opmode: 0 Off, 1 Run, 2 Precharge, 3 PchFail, 4 Charge, 5 Preheat | `Param::GetInt(Param::opmode)` (enum `MOD_*` in `include/param_prj.h`) |
| 1 | charge type: 0 Off, 1 AC, 2 DCFC | `Param::GetInt(Param::chgtyp)` |
| 2-7 | 0, reserved | |

## Where to implement
- `src/stm32_vcu.cpp`, `Ms100Task()`: it already calls `selectedBMS->Task100Ms()`, `canMap->SendAll()` etc. Add the send
  there (or as a small helper function called from there).
- Bus selection: `canInterface[Param::GetInt(Param::BMSCan)]` (array of `CanHardware*`, index 0 = CAN1, 1 = CAN2; set up
  near `canInterface[0] = &c;` in `main`). Check how other modules pick their bus, e.g. grep `Param::BMSCan` / `ChargerCan`
  and mimic that. The `BMSCan` default is 1 (CAN2).
- Send with `can->Send(0x3A1, (uint32_t *)bytes, 8)` as used elsewhere (e.g. `src/Can_OI.cpp`, `src/NissLeafMng.cpp`).
  Build the 8 bytes in a `uint8_t bytes[8]` and check how the existing callers cast it (word order on this STM32 library).
- Make sure the frame is sent regardless of which BMS / charger / inverter classes are selected (i.e. do not put it inside
  a vehicle/BMS-specific class that may be `NoBMS`/`notused`).

## Constraints / things to check
- Confirm 0x3A1 isn't already used by any module in `src/` (grep `0x3A1`). The BMS accepts exactly this ID
  through an exact-match hardware filter.
- Do not change behaviour of existing CAN traffic. Don't add a new parameter unless clearly necessary (if you do, add it to
  `param_prj.h` with a fresh unique id, and mention it).
- Keep code style consistent with surrounding code (formatting, comments).
- Build with the repo's Makefile (`make`) if the toolchain is available; otherwise say that it was not compiled.

## Acceptance
- A CAN sniffer on the `BMSCan` bus sees `0x3A1` every ~100 ms with byte0 following `opmode` (Off at boot, Charge while
  charging on AC, Run when driving).
- No change to any other frame.
- Report: files changed, the exact send code, and whether it compiled.

## BMS side (already done, for context only)
`BMWPhevBMS.ino` listens for 0x3A1 (`ZV_STATE_CAN_ID`). Valid for 3 s after the last frame; in that time Charge(4) => BMS
Charge state, Run(1) => Drive, otherwise Ready. After 3 s without a frame it falls back to its old AC_PRESENT pin logic.

Note: 0x3A0 is already used by src/i3LIM.cpp (vehicle condition toward the DC fast-charge port), so the state frame uses 0x3A1.
