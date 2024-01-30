#!/bin/sh
# Configure the si570 parameters through mmc console.
# The configuration depends on PCB rev, retrieved by serial number.
# Usage: ./si570_configuration.sh (just run it)

# Get ttyUSB of mmc console and serial number
MMC_TTY=$(ls -l /dev/serial/by-id/ | grep "if03" \
        | sed 's/.*ttyUSB/\/dev\/ttyUSB/')
SERIAL_NUMBER=$(ls -l /dev/serial/by-id/ | grep "if03" \
              | sed -r 's/.*Marble_0{0,6}//' | sed 's/-.*$//')
# Take first one in case multiple device are connected
MMC_TTY=$(echo $MMC_TTY | sed 's/ .*//')
SERIAL_NUMBER=$(echo $SERIAL_NUMBER | sed 's/ .*//')

# Print information
echo "MMC = \"$MMC_TTY\""
echo "SERIAL = \"$SERIAL_NUMBER\""

### Temporary solution to configure Si570 parameters.
# NOTE: I2C address is hex 8-bit, frequency is decimal and the configuration
# contains the 0x40 for validity check.
if [ $SERIAL_NUMBER > 0 ]; then
  if [ "$SERIAL_NUMBER" -ge 53 ]; then # Marble 1.4 starts from #53
    si570_configuration_string="s AA 270000000 40"
    echo "Marble PCB rev = 1.4 - Si570 parameters = {0xAA (0x55 7-bit), 270 MHz, 0x0}"
  else # considering Marble 1.3
    si570_configuration_string="s EE 125000000 42"
    echo "Marble PCB rev = 1.3 - Si570 parameters = {0xEE (0x77 7-bit), 125 MHz, 0x1}"
  fi
fi

# Make load.py runnable from top folder
SCRIPT_PATH=$(pwd | sed 's/dual-evg.*/dual-evg\/scripts/')
if [ -z "$MMC_TTY" ] || [ -z "$si570_configuration_string" ]; then
    echo "[!] ERROR - something about USB connection is wrong - abort."
    else
    python3 $SCRIPT_PATH/load.py -d "$MMC_TTY" "$si570_configuration_string"
fi