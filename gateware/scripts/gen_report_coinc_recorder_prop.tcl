proc gen_report_coinc_recorder_prop {xpr_file report_coinc_recorder_prop_file} {

    open_project $xpr_file
    open_run impl_1
    report_property -append \
        -file $report_coinc_recorder_prop_file \
        [get_cells -hier -filter {NAME =~ *coincidenceRecorder1/genblk2[0].value_a_reg}]
    report_property -append \
        -file $report_coinc_recorder_prop_file \
        [get_cells -hier -filter {NAME =~ *coincidenceRecorder1/genblk2[0].value_m_reg}]
    report_property -append \
        -file $report_coinc_recorder_prop_file \
        [get_cells -hier -filter {NAME =~ *coincidenceRecorder2/genblk2[0].value_a_reg}]
    report_property -append \
        -file $report_coinc_recorder_prop_file \
        [get_cells -hier -filter {NAME =~ *coincidenceRecorder2/genblk2[0].value_m_reg}]
}

if { $argc < 2 } {
    puts "Not enough arguments"
    puts "Usage: vivado -mode batch -nojou -nolog -source gen_report_coinc_recorder_prop.tcl -tclargs <xpr_file> <report_coinc_recorder_prop_file>"
    exit
}

set xpr_file [lindex $argv 0]
set report_coinc_recorder_prop_file [lindex $argv 1]

gen_report_coinc_recorder_prop $xpr_file $report_coinc_recorder_prop_file
