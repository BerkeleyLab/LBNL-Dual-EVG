#!/usr/bin/env bash

set -euo pipefail

APP=${1:-devg}
PLATFORM=${2:-marble}

SCRIPTPATH="$( cd "$( dirname "${BASH_SOURCE[0]}"  )" && pwd  )"

cd ${SCRIPTPATH}/../gateware/scripts

cd ${SCRIPTPATH}/../software/scripts
