# FMC 1
# CLK0_M2C -- H4/H5
set_property -dict {PACKAGE_PIN F17 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports FMC1_CLK0_M2C_P]
set_property -dict {PACKAGE_PIN E17 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports FMC1_CLK0_M2C_N]
create_clock -period 8.001 -name refCoinc1 [get_ports FMC1_CLK0_M2C_P]
# LA03_N -- G10
set_property -dict {PACKAGE_PIN L18 IOSTANDARD LVCMOS25} [get_ports {FMC1_diagnosticOut[0]}]
# LA04_N -- H11
set_property -dict {PACKAGE_PIN G20 IOSTANDARD LVCMOS25} [get_ports {FMC1_diagnosticOut[1]}]
# LA06_P -- C10
set_property -dict {PACKAGE_PIN L19 IOSTANDARD LVCMOS25 PULLUP true} [get_ports FMC1_EVIO_SCL]
# LA06_N -- C11
set_property -dict {PACKAGE_PIN L20 IOSTANDARD LVCMOS25 PULLUP true} [get_ports FMC1_EVIO_SDA]
# LA07_P -- H13
set_property -dict {PACKAGE_PIN D19 IOSTANDARD LVCMOS25} [get_ports {FMC1_fireflySelect_n[0]}]
# LA07_N -- H14
set_property -dict {PACKAGE_PIN D20 IOSTANDARD LVCMOS25} [get_ports {FMC1_fireflySelect_n[1]}]
# LA08_P -- G12
set_property -dict {PACKAGE_PIN G19 IOSTANDARD LVCMOS25 PULLUP true} [get_ports {FMC1_fireflyPresent_n[0]}]
# LA08_N -- G13
set_property -dict {PACKAGE_PIN F20 IOSTANDARD LVCMOS25 PULLUP true} [get_ports {FMC1_fireflyPresent_n[1]}]
# LA11_P -- H16
set_property -dict {PACKAGE_PIN L17 IOSTANDARD LVCMOS25} [get_ports {FMC1_fireflySelect_n[2]}]
# LA11_N -- H17
set_property -dict {PACKAGE_PIN K18 IOSTANDARD LVCMOS25} [get_ports FMC1_sysReset_n]
# LA12_P -- G15
set_property -dict {PACKAGE_PIN G15 IOSTANDARD LVCMOS25 PULLUP true} [get_ports {FMC1_fireflyPresent_n[2]}]
# LA12_N -- G16
set_property -dict {PACKAGE_PIN F15 IOSTANDARD LVCMOS25 PULLUP true} [get_ports {FMC1_fireflyPresent_n[3]}]
# LA15_P -- H19
set_property -dict {PACKAGE_PIN J15 IOSTANDARD LVCMOS25 PULLUP true} [get_ports FMC1_FAN1_TACH]
# LA15_N -- H20
set_property -dict {PACKAGE_PIN J16 IOSTANDARD LVCMOS25 PULLUP true} [get_ports FMC1_FAN2_TACH]
# LA16_P -- G18
set_property -dict {PACKAGE_PIN K16 IOSTANDARD LVCMOS25 PULLUP true} [get_ports {FMC1_fireflyPresent_n[4]}]
# LA16_N -- G19
set_property -dict {PACKAGE_PIN K17 IOSTANDARD LVCMOS25 PULLUP true} [get_ports {FMC1_fireflyPresent_n[5]}]

# LA19_P -- H14
set_property -dict {PACKAGE_PIN H14 IOSTANDARD LVCMOS25} [get_ports {FMC1_hwTrigger[0]}]
# LA19_N -- G14
set_property -dict {PACKAGE_PIN G14 IOSTANDARD LVCMOS25} [get_ports {FMC1_hwTrigger[1]}]
# LA21_P -- D14
set_property -dict {PACKAGE_PIN D14 IOSTANDARD LVCMOS25} [get_ports {FMC1_hwTrigger[2]}]
# LA21_N -- D13
set_property -dict {PACKAGE_PIN D13 IOSTANDARD LVCMOS25} [get_ports {FMC1_hwTrigger[3]}]
# LA24_P -- A9
set_property -dict {PACKAGE_PIN A9 IOSTANDARD LVCMOS25} [get_ports {FMC1_hwTrigger[4]}]
# LA24_N -- A8
set_property -dict {PACKAGE_PIN A8 IOSTANDARD LVCMOS25} [get_ports {FMC1_hwTrigger[5]}]
# LA28_P -- J13
set_property -dict {PACKAGE_PIN J13 IOSTANDARD LVCMOS25} [get_ports FMC1_auxInput]
# LA28_N -- H13
set_property -dict {PACKAGE_PIN H13 IOSTANDARD LVCMOS25} [get_ports {FMC1_diagnosticIn[0]}]

# FMC 2
# CLK0_M2C -- H4/H5
set_property -dict {PACKAGE_PIN Y23 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports FMC2_CLK0_M2C_P]
set_property -dict {PACKAGE_PIN AA24 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports FMC2_CLK0_M2C_N]
create_clock -period 7.999 -name refCoinc2 [get_ports FMC2_CLK0_M2C_P]
# LA03_N -- G10
set_property -dict {PACKAGE_PIN AE26 IOSTANDARD LVCMOS25} [get_ports {FMC2_diagnosticOut[0]}]
# LA04_N -- H11
set_property -dict {PACKAGE_PIN W21 IOSTANDARD LVCMOS25} [get_ports {FMC2_diagnosticOut[1]}]
# LA06_P -- C10
set_property -dict {PACKAGE_PIN AD23 IOSTANDARD LVCMOS25 PULLUP true} [get_ports FMC2_EVIO_SCL]
# LA06_N -- C11
set_property -dict {PACKAGE_PIN AD24 IOSTANDARD LVCMOS25 PULLUP true} [get_ports FMC2_EVIO_SDA]
# LA07_P -- H13
set_property -dict {PACKAGE_PIN AB22 IOSTANDARD LVCMOS25} [get_ports {FMC2_fireflySelect_n[0]}]
# LA07_N -- H14
set_property -dict {PACKAGE_PIN AC22 IOSTANDARD LVCMOS25} [get_ports {FMC2_fireflySelect_n[1]}]
# LA08_P -- G12
set_property -dict {PACKAGE_PIN AC23 IOSTANDARD LVCMOS25 PULLUP true} [get_ports {FMC2_fireflyPresent_n[0]}]
# LA08_N -- G13
set_property -dict {PACKAGE_PIN AC24 IOSTANDARD LVCMOS25 PULLUP true} [get_ports {FMC2_fireflyPresent_n[1]}]
# LA11_P -- H16
set_property -dict {PACKAGE_PIN W23 IOSTANDARD LVCMOS25} [get_ports {FMC2_fireflySelect_n[2]}]
# LA11_N -- H17
set_property -dict {PACKAGE_PIN W24 IOSTANDARD LVCMOS25} [get_ports FMC2_sysReset_n]
# LA12_P -- G15
set_property -dict {PACKAGE_PIN AA25 IOSTANDARD LVCMOS25 PULLUP true} [get_ports {FMC2_fireflyPresent_n[2]}]
# LA12_N -- G16
set_property -dict {PACKAGE_PIN AB25 IOSTANDARD LVCMOS25 PULLUP true} [get_ports {FMC2_fireflyPresent_n[3]}]
# LA15_P -- H19
set_property -dict {PACKAGE_PIN U22 IOSTANDARD LVCMOS25 PULLUP true} [get_ports FMC2_FAN1_TACH]
# LA15_N -- H20
set_property -dict {PACKAGE_PIN V22 IOSTANDARD LVCMOS25 PULLUP true} [get_ports FMC2_FAN2_TACH]
# LA16_P -- G18
set_property -dict {PACKAGE_PIN W25 IOSTANDARD LVCMOS25 PULLUP true} [get_ports {FMC2_fireflyPresent_n[4]}]
# LA16_N -- G19
set_property -dict {PACKAGE_PIN W26 IOSTANDARD LVCMOS25 PULLUP true} [get_ports {FMC2_fireflyPresent_n[5]}]

# LA19_P -- K23
set_property -dict {PACKAGE_PIN K23 IOSTANDARD LVCMOS25} [get_ports {FMC2_hwTrigger[0]}]
# LA19_N -- J23
set_property -dict {PACKAGE_PIN J23 IOSTANDARD LVCMOS25} [get_ports {FMC2_hwTrigger[1]}]
# LA21_P -- J21
set_property -dict {PACKAGE_PIN J21 IOSTANDARD LVCMOS25} [get_ports {FMC2_hwTrigger[2]}]
# LA21_N -- H22
set_property -dict {PACKAGE_PIN H22 IOSTANDARD LVCMOS25} [get_ports {FMC2_hwTrigger[3]}]
# LA24_P -- J24
set_property -dict {PACKAGE_PIN J24 IOSTANDARD LVCMOS25} [get_ports {FMC2_hwTrigger[4]}]
# LA24_N -- J25
set_property -dict {PACKAGE_PIN J25 IOSTANDARD LVCMOS25} [get_ports {FMC2_hwTrigger[5]}]
# LA28_P -- G25
set_property -dict {PACKAGE_PIN G25 IOSTANDARD LVCMOS25} [get_ports FMC2_auxInput]
# LA28_N -- G26
set_property -dict {PACKAGE_PIN G26 IOSTANDARD LVCMOS25} [get_ports {FMC2_diagnosticIn[0]}]
