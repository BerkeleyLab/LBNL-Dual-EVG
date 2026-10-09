# Bootstrapping a New Board

> [!NOTE]
> **Docs:** [Hardware](Hardware.md) · [Firmware](Firmware.md) · [EPICS Support](EPICSnotes.md) · [Updating Firmware](HowtoUpdateFirmware.md) · [Bringing Up a New Board](BringingUpNewBoard.md)

This procedure downloads and runs FPGA firmware and software over the board's **JTAG** port.

**Prerequisites**

- A download image is ready, as described in [How to Update Firmware](HowtoUpdateFirmware.md).
- A Vivado session with the desired project is running.

---

## Steps

### Connect

1. Connect the USB ports of the development machine and the FPGA to be programmed.
2. In a terminal, start the Xilinx Virtual Cable server:

   ```sh
   ftdiJTAG -c 30M -g 11
   ```

   See [USB/JTAG](Firmware.md#usbjtag) for options when multiple FTDI devices are connected.

### Open the hardware target in Vivado

3. Click **Open Hardware Manager** (or **Flow → Hardware Manager**).
4. Click **Open Target**.
5. Click **Open New Target…**
6. In the **Open New Hardware Target** wizard, click **Next >**.
7. Select **Connect to Local server**, then **Next >**.
   > [!NOTE]
   > Choose this even if the server was started on a different machine.
8. If the device isn't listed under **Hardware Targets**, click **Add Xilinx Virtual Cable (XVC)** and enter the server's address and port.

   | Setting | Default |
   |---|---|
   | IP address | `127.0.0.1` |
   | Port | `2542` |

9. Make sure the desired device is highlighted, then click **Next >**.
10. Click **Finish**.

### Program the device

11. Click **Program Device** (or **Tools → Program Device**).
12. In the **Program Device** dialog:
    - **Clear** the *Debug probes file* field.
    - Set *Bitstream file* to `download.bit` in the Vitis workspace `scripts` directory.
    - Click **Program**.
13. When the download completes, the application starts running. Watch its progress in a [console](Firmware.md#console-commands) session in another terminal.

### Make it permanent

14. Use a TFTP client to transfer `download.bit` to `DEVG_A.bit` in flash memory. The `programFlash.sh` script in the `scripts` directory helps with this.
15. Set the application [system parameters](Firmware.md#setting-network-parameters) in flash memory as appropriate.

---

> [!TIP]
> **ChipScope:** Steps 1–10 also start a ChipScope session. Perform them, then click **Refresh Device**.
