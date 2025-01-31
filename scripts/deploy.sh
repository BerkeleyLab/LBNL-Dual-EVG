#!/usr/bin/env bash

set -euo pipefail

SCRIPTPATH="$( cd "$( dirname "${BASH_SOURCE[0]}"  )" && pwd  )"
EXTRA_OPTS=${EXTRA_OPTS}
BITFILE=${BITFILE:-devg_marble.bit}

if test -r "${BITFILE}"; then
    openocd -s "${SCRIPTPATH}" -f marble.cfg -c "${EXTRA_OPTS}" -c "transport select jtag; init; xc7_program xc7.tap; pld load 0 ${BITFILE}; exit"
else
    echo "${BITFILE} not found"
fi
