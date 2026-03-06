proc gen_placement_report {xpr_file placement_report_file} {

    open_project $xpr_file
    open_run impl_1
    report_placement -force $placement_report_file
}

if { $argc < 2 } {
    puts "Not enough arguments"
    puts "Usage: vivado -mode batch -nojou -nolog -source gen_placement_report.tcl -tclargs <xpr_file> <placement_report_file>"
    exit
}

set xpr_file [lindex $argv 0]
set placement_report_file [lindex $argv 1]

gen_placement_report $xpr_file $placement_report_file
