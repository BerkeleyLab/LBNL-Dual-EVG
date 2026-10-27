# Bootstrapping a New Board

> [!NOTE]
> **dual-evg docs:** [Overview](README.md) · [Hardware](Hardware.md) · [Firmware](Firmware.md) · [Updating Firmware](HowtoUpdateFirmware.md) · [Bringing Up a New Board](BringingUpNewBoard.md)<br>
> **Related projects:** [EPICS support module ↗](https://git.als.lbl.gov/accelerator-controls/epics/iocs/DualEventGeneratorSupport/-/blob/HEAD/documentation/README.md) · [Timing IOC ↗](https://git.als.lbl.gov/accelerator-controls/epics/iocs/siocDualTiming/-/blob/HEAD/documentation/README.md)

A new board has no application in flash, so the first image must be loaded over **JTAG**. Once it is running, the image is written to flash over the network.

**Prerequisites**

- A built image, `software/app/devg/devg_marble.<git-hash>.bit`, as described in [Building](Firmware.md#building).
- A USB cable from the development machine to the front-panel USB connector.

---

## 1. Configure the Marble Si570 (once per board)

```sh
scripts/si570_configuration.sh
```

The script finds the Marble MMC console, reads the board serial number, and programs the Si570 oscillator for PCB rev 1.3 or 1.4 (serial 53 and up).

## 2. Load the image over JTAG

Choose either method.

### Option A — OpenOCD (recommended)

```sh
cd scripts
BITFILE=../software/app/devg/devg_marble.<git-hash>.bit ./deploy.sh
```

`deploy.sh` uses `scripts/marble.cfg`. It reads the image from the `BITFILE` environment variable and ignores command-line arguments.

### Option B — Vivado Hardware Manager over a Xilinx Virtual Cable

1. Start the virtual cable server:

   ```sh
   ftdiJTAG -c 30M -g 11
   ```

   With multiple FTDI devices connected, add `-d vendor:product:serial`, for example `-d 0403:6011:000004`. `-c 30M` runs JTAG at its 30 Mb/s maximum and `-g 11` enables the Marble on-board USB/JTAG. `ftdiJTAG` is a separate tool and is not part of this repository.
2. In Vivado, click **Open Hardware Manager** (or **Flow → Hardware Manager**).
3. Click **Open Target → Open New Target…**, then **Next >**.
4. Select **Connect to Local server**, then **Next >**.
   > [!NOTE]
   > Choose this even if the server was started on a different machine.
5. If the device isn't listed under **Hardware Targets**, click **Add Xilinx Virtual Cable (XVC)** and enter the server's address and port.

   | Setting | Default |
   |---|---|
   | IP address | `127.0.0.1` |
   | Port | `2542` |

6. Highlight the device, click **Next >**, then **Finish**.
7. Click **Program Device** (or **Tools → Program Device**).
8. Clear the *Debug probes file* field, set *Bitstream file* to the image, and click **Program**.

> [!TIP]
> **ChipScope:** steps 1–6 of Option B also start a ChipScope session. Perform them, then click **Refresh Device**.

## 3. Watch it start

When the download completes, the application starts running. Watch its progress on the FPGA console (USB port C, 115200-8N1). See [Console Commands](Firmware.md#console-commands).

## 4. Write the image to flash

A new board starts with the default network configuration, `192.168.1.129/24`. From a host on that subnet:

```sh
cd software/scripts
sh programFlash.sh 192.168.1.129 ../app/devg/devg_marble.<git-hash>.bit
```

This writes `DEVG_A.bit` and verifies it by read-back. Optionally write the same image to the alternate slot too (add `B`), so a fallback is always present.

> [!IMPORTANT]
> Write the flash **before** rebooting. Until it holds an image, a reboot loses the JTAG-loaded application.

## 5. Load the default sequence

```sh
tftp -m binary 192.168.1.129 -c put DefaultSequence.csv
```

A template is in `software/scripts`. See [`DefaultSequence.csv` format](Firmware.md#defaultsequencecsv-format).

## 6. Set the network parameters

Set the real address, MAC, and time source with the `net`, `mac`, and `tod` console commands. See [Setting Network Parameters](Firmware.md#setting-network-parameters).

## 7. Reboot

Power cycle the unit, or run `boot` on the console, and confirm that it starts from flash with the new settings.
