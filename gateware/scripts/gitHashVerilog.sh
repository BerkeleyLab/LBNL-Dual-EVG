#! /bin/sh
# Print git ID in Verilog header format
# Usage: sh gitHashVerilog.sh > gitHash.vh
gitid=$(git rev-parse --short=8 HEAD)
diffHash=$(echo $(git diff) | md5sum | head -c 8)
echo 'localparam GIT_REV_STR = "'$gitid'";'
echo 'localparam GIT_DIRTY = '$(git diff | grep -q . && echo 1 || echo 0)';'
echo 'localparam GIT_REV_32BIT = 32'"'h"$gitid";"
if [ -z "$diffHash" ]; then
  echo 'localparam LOCAL_HASH = 32'"'h0;"
else
  echo 'localparam LOCAL_HASH = 32'"'h"$diffHash";"
fi
