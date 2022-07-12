proc gen_bd {bd_file project_part project_board ipcore_dirs} {
    # create_ip requires that a project is open in memory. Create project
    # but don't do anything with it
    create_project -in_memory -part $project_part -force my_project

    # specify board_part if existent
    if {$project_board ne "none"} {
        set_property board_part $project_board [current_project]
    }

    # specify additional library directories for custom IPs
    set_property ip_repo_paths $ipcore_dirs [current_fileset]
    update_ip_catalog -rebuild

    # read an BD file into project
    read_bd $bd_file

    # make top level wrapper
    make_wrapper -files [get_files $bd_file] -top

    # Generate all the output products
    generate_target all [get_files $bd_file] -force

    # export and validate hardware platform for use with
    # Vitis
    set bd_basename [file rootname [file tail $bd_file]]
    write_hw_platform -fixed -force $bd_basename.xsa
    validate_hw_platform ./$bd_basename.xsa
}
