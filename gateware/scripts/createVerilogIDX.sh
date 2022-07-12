#!/bin/sh

#
# Convert C preprocessor definitions beginning with the
# characters 'CFG_' to Verilog parameter declarations.
#
# Convert C preprocessor definitions beginning with
# the characters 'VERILOG_' to Verilog definitions.
#

set -eu

HDR_FILES=$@

process() {
    sed -n -e "/^ *# *define *\($1[^ ]*\) *\(.*\)/s//parameter \1 = \2/p" $2 |
    sed -e 's/ *\/[\/\*].*//' -e 's/$/,/'
}


echo '// DO NOT EDIT -- CHANGES WILL BE OVERWRITTEN WHEN'
echo '// THIS FILE IS REGENERATED FROM THE C HEADER FILE'
for f in $HDR_FILES
do
    process "GPIO_IDX" "$f"
    process "CFG_" "$f"
    sed -n -e '/^ *# *define *VERILOG_\([A-Za-z0-9_]*\).*/s//`define \1/'p "$f"
done
