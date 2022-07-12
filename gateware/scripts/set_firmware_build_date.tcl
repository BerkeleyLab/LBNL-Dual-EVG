#
# Get time from epoch in UTC.
#

if { $argc <1} {
    puts "Not enough arguments"
	puts "Usage: vivado -mode batch -nojou -nolog -source setFirmwareBuildDate.tcl -tclargs <output_file>"
	exit
}

set my_output_file [lindex $argv 0]

puts "Set firmware time from [pwd]"
if {![catch {set firmwareDateFile [open "$my_output_file" w]}]} {
    puts $firmwareDateFile "// MACHINE GENERATED -- DO NOT EDIT"
    puts $firmwareDateFile "localparam FIRMWARE_BUILD_DATE = [clock seconds];"
    close $firmwareDateFile
}
