#!/bin/sh

set -ex

export PYTHONPATH="$PYTHONPATH=:$HOME/src/FMC/pts_core/tools"
export FRU_VERBOSE=1

usage()
{
    echo "Usage: $0 ### [### ...]" >&2
    exit 1
}

if [ $# = 0 ]
then
    usage
fi
for sn in "$@"
do
    case "$sn" in
        00[12])          REV="0" ;;
        [0-9][0-9][0-9]) REV="A" ;;
        *)               usage ;;
    esac
    python2.7 fru-generator \
                        -o eeprom_${sn}.bin \
                        -v LBNL \
                        -n "EVIO" \
                        -p "EVIO Rev. ${REV}" \
                        -s "${sn}" 
    fru-dump -i eeprom_$sn.bin -b -c -p -2 -v
done
