if { $argc < 2 } {
    puts "Not enough arguments"
    puts "Usage: xsct download_elf.tcl <psu_init_tcl> <elf>"
    exit
}

set psu_init_tcl [file normalize [lindex $argv 0]]
set elf_file [file normalize [lindex $argv 1]]

# Heavily based on
# https://www.xilinx.com/htmldocs/xilinx2019_1/SDK_Doc/xsct/use_cases/xsdb_debug_app_zynqmp.html

# Connect
connect
# Select PSU unit
targets 4

# Init PSU unit
source $psu_init_tcl
psu_init
after 1000
psu_ps_pl_isolation_removal
after 1000
psu_ps_pl_reset_config

# Select first APU core
targets 9
rst -processor

# Download application .elf
dow $elf_file

# Continue normal execution
con

# cleanup
exit
