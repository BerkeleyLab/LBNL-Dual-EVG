#!/usr/bin/env bash

set -euo pipefail

SCRIPTPATH="$( cd "$( dirname "${BASH_SOURCE[0]}"  )" && pwd  )"
BITFILE=${1:-download.bit}

if test -r "${BITFILE}"; then
    openocd -s "${SCRIPTPATH}" -f marble.cfg -c "transport select jtag; init; xc7_program xc7.tap; pld load 0 ${BITFILE}; exit"
else
    echo "${BITFILE} not found"
fi
