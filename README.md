# Dual EVG

Gateware and software for the ALS-U dual event generator: a Marble FPGA carrier with two EVIO FMC cards that distributes RF reference and timing signals for both ALS-U frequency domains.

## Quick start

```sh
git submodule update --init
make APP=devg PLATFORM=marble EVIO_TYPE=evio
```

Requires Vivado and Vitis 2020.2.2 on `PATH`. The bootable image is written to `software/app/devg/devg_marble.<git-hash>.bit`.

## Documentation

📖 **[documentation/README.md](documentation/README.md)** — dependencies, build options, deployment, hardware, firmware, and update procedures.
