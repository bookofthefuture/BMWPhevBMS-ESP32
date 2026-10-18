# EV CAN bus map (500 kbit/s)

Compiled from the BMS source (`BMWPhevBMS.ino`, `BMSModuleManager.cpp`) and ZombieVerter source. Entries marked
**(verify)** have not been confirmed with a sniffer on the car. Add to this as the bus is explored.

| ID | Dir (from BMS) | Producer | Content |
|----|---------------|----------|---------|
| 0x080-0x086 | TX | BMS | BMW PHEV CSC (cell module) commands incl. balancing; byte 7 is CRC8 |
| 0x100-0x17F | RX | CSC modules | Cell voltages (`BMSModuleManager::decodecan`) |
| 0x180-0x18F | RX | CSC modules | Cell temperatures (`decodetemp`) |
| 0x285, 0x286 | TX | BMS | Outlander charger control (voltage/current request) |
| 0x351, 0x355, 0x356, 0x35A, 0x372, 0x373, 0x378, 0x379 | TX | BMS | Victron/SMA-style battery status (VEcan) |
| 0x354 | RX | Chademo | byte 0 == 1 => rapid charging |
| 0x377 | - | Outlander charger (DC-DC enabled) | No longer used by the BMS; contactor state now comes from 0x3A1 |
| 0x389, 0x38A | RX | Outlander charger | voltage/current/temps; status and EVSE duty |
| 0x3A1 | RX | ZombieVerter (new) | byte 0 opmode, byte 1 chgtyp. See `zombieverter-bms-state-frame.md` |
| 0x520-0x528 | RX | ISA shunt | current (0x521), voltages (0x522/523), power (0x526), Ah (0x527), Wh (0x528) |
| 0x52A | RX | ZombieVerter/other | param set: byte 0 == 1 => max AC charge current = byte 1 * 10 |

Not yet documented: the car (vehicle) CAN, which is a separate bus.

Removed: 0x01 (canio from the old Prius Gen 3 board), no longer sent on the car.
