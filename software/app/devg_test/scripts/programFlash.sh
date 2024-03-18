#!/bin/sh

IP="${HSD_IP_ADDRESS=131.243.196.245}"
SRC="DEVG_A.bit"

for i
do
    case "$i" in
        *.bit) SRC="$i" ;;
        *)     IP="$i" ;;
    esac
done

set -ex
test -r "$SRC"
tftp -v -m binary "$IP" -c put "$SRC" DEVG_A.bit
sleep 5
tftp -v -m binary "$IP" -c get DEVG_A.bit DEVG_Achk.bit
set +e
if cmp "$SRC" DEVG_Achk.bit
then
    echo "Flash write succeeded."
    rm DEVG_Achk.bit
else
    echo "Flash write may have failed!"
fi

