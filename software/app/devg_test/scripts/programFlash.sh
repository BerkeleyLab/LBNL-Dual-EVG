#!/bin/sh

IP="${HSD_IP_ADDRESS=131.243.196.245}"
SRC="BOOT.bin"

for i
do
    case "$i" in
        *.bin) SRC="$i" ;;
        *)     IP="$i" ;;
    esac
done

set -ex
test -r "$SRC"
tftp -v -m binary "$IP" -c put "$SRC" BOOT.BIN
sleep 5
tftp -v -m binary "$IP" -c get BOOT.BIN BOOTchk.bin
set +e
if cmp "$SRC" BOOTchk.bin
then
    echo "Flash write succeeded."
    rm BOOTchk.bin
else
    echo "Flash write may have failed!"
fi

