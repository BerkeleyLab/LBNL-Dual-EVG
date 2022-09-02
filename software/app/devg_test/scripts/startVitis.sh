#!/bin/bash

APP="vitis"
BVERS="2020.1"
VERS="2020.1.1"

while [ $# -ne 0 ]
do
    case "$1" in
        [0-9.][0-9]*)          VERS="$1"    ;;
        b*)                     APP="bash"   ;;
        v*)                     APP="vitis" ;;
        *) echo "Usage: $0 [#.#] [vitis|bash]" >&2 ; exit 1 ;;
    esac
    shift
done

case `uname -m` in
    *_64)   b=64 ;;
    *)      b=32 ;;
esac
s="/eda/xilinx/$VERS/Vivado/$BVERS/settings$b.sh"
if [ -f "$s" ]
then
    echo "Getting settings from \"$s\"."
    . "$s"
else
    echo "Can't find $s" >&2
    exit 2
fi

export XILINXD_LICENSE_FILE="27004@engvlic3.lbl.gov"
case "$APP" in
    bash) "$APP" -l   ;;
    *)     mkdir -p "$HOME/xilinxPlaypen"
           cd "$HOME/xilinxPlaypen" # Leave logging dregs in one place
           $APP & ;;
esac
