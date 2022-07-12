Dual EVG
==========

Gateware/Software for Dual EVG

### Building

This repository contains both gateware and software
for the Dual EVG.

#### Gateware Dependencies

To build the gateware the following dependencies are needed:

* GNU Make
* Xilinx Vivado (2020.2.2 tested), available [here](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vivado-design-tools.html)
* Xilinx Vitis (2020.2.2 tested), available [here](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vitis.html)

Make sure `vivado` and `vitis` are in PATH.

#### Software Dependencies

To build the software the following dependencies are needed:

* mb-gcc toolchain, bundled within Vitis

#### Building Instructions

With the dependencies in place a simple `make` should be able to generate
both gateware and software:

```bash
make
```

A suggestion in running the `make` command is to measure the time
and redirect stdout/stderr to a file so you can inspect it later:

```bash
ARM_TOOLCHAIN_LOCATION=/media/Xilinx/Vivado/2020.2.2/Vitis/2020.2/gnu/microblaze/lin
(time make CROSS_COMPILE=${ARM_TOOLCHAIN_LOCATION}/bin/mb- APP=devg PLATFORM=marble && notify-send 'Compilation SUCCESS' || notify-send 'Compilation ERROR'; date) 2>&1 | tee make_output
```

### Deploying

To deploy the gateware and the software we can use a variety of
methods. For development, JTAG is being used. Remember to check
the DIP switches on development boards and ensure the switches
are set to JTAG mode and NOT SD Card mode.

#### Deploying gateware

The following script can download the gateware via JTAG:

```bash
```

#### Deploying software

The following script can download the software via JTAG:

```bash
```
