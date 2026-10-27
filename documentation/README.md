# ALS-U Dual Event Generator — Hardware and Firmware

This folder documents the dual event generator **hardware** and the **`dual-evg` firmware** (gateware and µBlaze software). The unit is a Marble FPGA carrier with two EVIO FMC cards that distributes RF reference and timing signals for both ALS-U frequency domains over MRF-compatible fiber links.

| Document | What's inside |
|---|---|
| 🔧 [Hardware](Hardware.md) | Front/rear panels, display, Marble FPGA card, FMC cards, GPS, pinouts |
| 💾 [Firmware](Firmware.md) | Configuration, clock synchronization, event codes, TFTP, console commands, building, JTAG deploy |
| ⬆️ [How to Update Firmware](HowtoUpdateFirmware.md) | Step-by-step flash update procedure |
| 🆕 [Bringing Up a New Board](BringingUpNewBoard.md) | First-time programming over USB/JTAG |

## Getting started

The repository holds both the gateware and the software for the dual event generator.

### Dependencies

| Needed for | Tool |
|---|---|
| Gateware | GNU Make |
| Gateware | [Xilinx Vivado](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vivado-design-tools.html) 2020.2.2 (tested) |
| Gateware and software | [Xilinx Vitis](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vitis.html) 2020.2.2 (tested) |
| Software | `mb-gcc` µBlaze toolchain, bundled with Vitis |

Make sure `vivado` and `vitis` are on your `PATH`, and fetch the submodules after cloning:

```sh
git submodule update --init
```

### Building

With the dependencies in place, a plain `make` from the repository top builds both gateware and software:

```sh
make
```

It's worth timing the build and saving its output to inspect later:

```sh
MB_TOOLCHAIN_LOCATION=/media/Xilinx/Vivado/2020.2.2/Vitis/2020.2/gnu/microblaze/lin
(time make CROSS_COMPILE=${MB_TOOLCHAIN_LOCATION}/bin/mb- APP=devg PLATFORM=marble EVIO_TYPE=evio \
  && notify-send 'Compilation SUCCESS' || notify-send 'Compilation ERROR'; date) 2>&1 | tee make_output
```

| Variable | Supported values |
|---|---|
| `APP` | `devg` (production), `devg_test` |
| `EVIO_TYPE` | `evio`, `evio_trig_nibble_reversed` |

The bootable image is written to `software/app/<APP>/<APP>_marble.<git-hash>.bit`. See [Building](Firmware.md#building) for details.

### Deploying

For development, load the image straight into the FPGA over JTAG:

```sh
cd scripts
BITFILE=../software/app/<APP>/<APP>_marble.<git-hash>.bit ./deploy.sh
```

Use `APP=devg` or `APP=devg_test` depending on the desired target. This doesn't write the flash; for a permanent update, see [How to Update Firmware](HowtoUpdateFirmware.md).

## Reference documents

| Document | Contents |
|---|---|
| [ALS / ALS-U clock trees](ALS_ALS-U_Clock_Trees.pdf) | Clock relationships between the frequency domains |
| [Analog Devices LTC6957](DataSheets/6957fb.pdf) | Clock buffer datasheet |
| [Texas Instruments LSF010x](DataSheets/lsf0102.pdf) | Level translator datasheet |
| [Digilent PmodGPS](DataSheets/pmodgps-gms-u1lp_rm.pdf) | GPS receiver reference manual |

`Coincidence.graffle` is the OmniGraffle source for `Coincidence.svg`.

## Quick facts

| | |
|---|---|
| Console serial | 115200-8N1 (USB port C), or UDP port 50004 |
| Recovery IP | `192.168.1.129/24` |
| Recovery MAC | `AA:4C:42:4E:4C:04` |
| Boot images | `DEVG_A.bit` (primary), `DEVG_B.bit` (alternate) |
| Build | `make APP=devg PLATFORM=marble` → `software/app/devg/devg_marble.<git-hash>.bit` |
| Toolchain | Vivado/Vitis 2020.2.2 |

## Related projects

| Project | Documentation |
|---|---|
| **[DualEventGeneratorSupport](https://git.als.lbl.gov/accelerator-controls/epics/iocs/DualEventGeneratorSupport)** — EPICS support module and engineering IOC: ASYN driver, database, Phoebus screens, `console.py` | [EPICS Support ↗](https://git.als.lbl.gov/accelerator-controls/epics/iocs/DualEventGeneratorSupport/-/blob/HEAD/documentation/EPICSnotes.md) |
| **[siocDualTiming](https://git.als.lbl.gov/accelerator-controls/epics/iocs/siocDualTiming)** — soft IOC that builds and sends injection sequences | [Timing IOC ↗](https://git.als.lbl.gov/accelerator-controls/epics/iocs/siocDualTiming/-/blob/HEAD/documentation/TimingIOC.md) |
