proc gen_mem_info {xpr_file meminfo_file} {

    open_project $xpr_file
    open_run impl_1
    write_mem_info -force $meminfo_file
}

if { $argc < 2 } {
    puts "Not enough arguments"
    puts "Usage: vivado -mode batch -nojou -nolog -source gen_mem_info.tcl -tclargs <xpr_file> <meminfo_file>"
    exit
}

set xpr_file [lindex $argv 0]
set meminfo_file [lindex $argv 1]

gen_mem_info $xpr_file $meminfo_file
