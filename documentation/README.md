# ALS-U Dual Event Generator

Documentation for the dual event generator — a Marble FPGA carrier with two EVIO FMC cards that distributes RF reference and timing signals for both ALS-U frequency domains over MRF-compatible fiber links.

| Document | What's inside |
|---|---|
| 🔧 [Hardware](Hardware.md) | Front/rear panels, display, Marble FPGA card, FMC cards, GPS, pinouts |
| 💾 [Firmware](Firmware.md) | Configuration, clock synchronization, event codes, TFTP, console commands, building |
| 📡 [EPICS Support](EPICSnotes.md) | ASYN driver, database records, IOC startup |
| ⬆️ [How to Update Firmware](HowtoUpdateFirmware.md) | Step-by-step flash update procedure |
| 🆕 [Bringing Up a New Board](BringingUpNewBoard.md) | First-time programming over USB/JTAG |

## Quick facts

| | |
|---|---|
| Console serial | 115200-8N1 (USB port C) |
| Recovery IP | `192.168.1.129/24` |
| Recovery MAC | `AA:4C:42:4E:4C:04` |
| Boot images | `DEVG_A.bit` (primary), `DEVG_B.bit` (alternate) |
