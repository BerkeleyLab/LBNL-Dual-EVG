if { $argc < 3 } {
    puts "Not enough arguments"
    puts "Usage: vivado -mode batch -nojou -nolog -source ip_core_proc.tcl gen_ip_core.tcl -tclargs <xci_file> <project_part> <project_board>"
    exit
}

set xci_file [file normalize [lindex $argv 0]]
set project_part [lindex $argv 1]
set project_board [lindex $argv 2]

gen_ip_core $xci_file $project_part $project_board
