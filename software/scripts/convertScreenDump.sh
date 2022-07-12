#!/bin/sh

# Extract screen dump from console log

usage()
{
    echo "Usage: $0 ifile ofile.png"
    exit 1
}
    
case "$#" in
    2)  ;;
    *)  usage ;;
esac
case "$2" in
    *.png)  ;;
    *)  usage ;;
esac

tr -d '\r' <"$1" | sed -n -e '/^P3/,/^$/p'  | pnmtopng >"$2"

