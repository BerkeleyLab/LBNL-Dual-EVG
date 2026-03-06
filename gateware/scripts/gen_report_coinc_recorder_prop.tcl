proc gen_report_coinc_recorder_prop {xpr_file} {

    # Define list of arrays for cells
    set cells [list \
        [dict create name "coincidenceRecorder1/genblk2\[0\].value_a_reg" loc "SLICE_X86Y182"] \
        [dict create name "coincidenceRecorder1/genblk2\[0\].value_m_reg" loc "SLICE_X87Y182"] \
        [dict create name "coincidenceRecorder2/genblk2\[0\].value_a_reg" loc "SLICE_X86Y181"] \
        [dict create name "coincidenceRecorder2/genblk2\[0\].value_m_reg" loc "SLICE_X87Y181"]
    ]

    open_project $xpr_file
    open_run impl_1

    foreach cell $cells {
        set name [dict get $cell name]
        set expected_loc [dict get $cell loc]

        puts "Checking placement for: $name"

        set cell_obj [get_cells -quiet -hier -filter "NAME =~ *$name"]

        if {$cell_obj eq ""} {
            puts "ERROR: Cell \"$name\" was not found in the design!"
            exit 3
        }

        set actual_loc [get_property LOC $cell_obj]
        set is_fixed [get_property IS_LOC_FIXED $cell_obj]

        if {$actual_loc eq $expected_loc} {
            puts "SUCCESS: cell \"$name\" is correctly placed on LOC \"$actual_loc\""
        } else {
            puts "FAILURE: cell \"$name\" is placed on \"$actual_loc\", but expected \"$expected_loc\""
            exit 3
        }
    }
}

if { $argc < 1 } {
    puts "Not enough arguments"
    puts "Usage: vivado -mode batch -nojou -nolog -source gen_report_coinc_recorder_prop.tcl -tclargs <xpr_file>"
    exit
}

set xpr_file [lindex $argv 0]

gen_report_coinc_recorder_prop $xpr_file
