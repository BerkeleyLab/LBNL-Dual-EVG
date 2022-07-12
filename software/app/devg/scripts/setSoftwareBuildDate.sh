#!/bin/sh

set -eu

SECONDS=`date '+%s'`

echo "// MACHINE GENERATED -- DO NOT EDIT"
echo "#define SOFTWARE_BUILD_DATE $SECONDS"
