if { $argc < 1 } {
    puts "Not enough arguments"
    puts "Usage: xsct download_bit.tcl <bistream>"
    exit
}

set bit_file [file normalize [lindex $argv 0]]

# Heavily based on
# https://www.xilinx.com/htmldocs/xilinx2019_1/SDK_Doc/xsct/use_cases/xsdb_debug_app_zynqmp.html

# Connect
connect

# Reset processor. Ottherwise bistream and elf fails...
targets 9
after 1000
rst
after 1000

# Select PL unit
targets 3

# Configure the FPGA. When the active target is not a FPGA device,
# the first FPGA device is configured
fpga -f $bit_file -no-revision-check

# cleanup
exit
