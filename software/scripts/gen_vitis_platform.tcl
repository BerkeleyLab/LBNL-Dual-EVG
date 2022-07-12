proc gen_vitis_platform {platform_name xsa_file proc} {
    # Set workspace
    setws ./$platform_name
    cd $platform_name

    # Create BSP
    app create -name $platform_name -hw $xsa_file -proc $proc -os standalone -lang c -template {Empty Application}

    # Add libraries to BSP
    bsp setlib -name libmetal

    # Generate platform
    platform generate
}

if { $argc < 3 } {
    puts "Not enough arguments"
    puts "Usage: xcst gen_vitis_platform.tcl <platform_name> <xsa_file> <proc_name>"
    exit
}

set platform_name [lindex $argv 0]
set xsa_file [lindex $argv 1]
set proc [lindex $argv 2]

gen_vitis_platform $platform_name $xsa_file $proc
