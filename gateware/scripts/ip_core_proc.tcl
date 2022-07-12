proc gen_ip_core {xci_file project_part project_board} {
    # create_ip requires that a project is open in memory. Create project
    # but don't do anything with it
    create_project -in_memory -part $project_part -force my_project

    # specify board_part if existent
    if {$project_board ne "none"} {
        set_property board_part $project_board [current_project]
    }

    # read an IP customization
    read_ip $xci_file

    # Generate all the output products
    generate_target all [get_files $xci_file] -force
}
