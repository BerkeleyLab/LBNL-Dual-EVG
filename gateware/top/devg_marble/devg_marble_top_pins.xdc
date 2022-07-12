# RGMII Rx
set_property -dict {PACKAGE_PIN E11 IOSTANDARD LVCMOS25} [get_ports RGMII_RX_CLK]
set_property -dict {PACKAGE_PIN J11 IOSTANDARD LVCMOS25} [get_ports RGMII_RX_CTRL]
set_property -dict {PACKAGE_PIN J10 IOSTANDARD LVCMOS25} [get_ports {RGMII_RXD[0]}]
set_property -dict {PACKAGE_PIN J8 IOSTANDARD LVCMOS25} [get_ports {RGMII_RXD[1]}]
set_property -dict {PACKAGE_PIN H8 IOSTANDARD LVCMOS25} [get_ports {RGMII_RXD[2]}]
set_property -dict {PACKAGE_PIN H9 IOSTANDARD LVCMOS25} [get_ports {RGMII_RXD[3]}]

# RGMII Tx
set_property -dict {PACKAGE_PIN F10 IOSTANDARD LVCMOS25} [get_ports RGMII_TX_CLK]
set_property -dict {PACKAGE_PIN C9 IOSTANDARD LVCMOS25} [get_ports RGMII_TX_CTRL]
set_property -dict {PACKAGE_PIN H11 IOSTANDARD LVCMOS25} [get_ports {RGMII_TXD[0]}]
set_property -dict {PACKAGE_PIN H12 IOSTANDARD LVCMOS25} [get_ports {RGMII_TXD[1]}]
set_property -dict {PACKAGE_PIN D8 IOSTANDARD LVCMOS25} [get_ports {RGMII_TXD[2]}]
set_property -dict {PACKAGE_PIN D9 IOSTANDARD LVCMOS25} [get_ports {RGMII_TXD[3]}]

# Special pin properties for RGMII
# Invalid to flag RGMII_RX_CLK as IOB TRUE
set_property IOB TRUE [get_ports RGMII_RX_CTRL]
set_property IOB TRUE [get_ports RGMII_RXD*]
set_property IOB TRUE [get_ports RGMII_TX*]
set_property SLEW FAST [get_ports RGMII_TX*]

# QSPI Boot Flash
set_property -dict {PACKAGE_PIN C23 IOSTANDARD LVCMOS25} [get_ports BOOT_CS_B]
set_property -dict {PACKAGE_PIN B24 IOSTANDARD LVCMOS25} [get_ports BOOT_MOSI]
set_property -dict {PACKAGE_PIN A25 IOSTANDARD LVCMOS25} [get_ports BOOT_MISO]

# I2C, shared access with microcontroller
set_property -dict {PACKAGE_PIN A17 IOSTANDARD LVCMOS25} [get_ports TWI_SDA]
set_property -dict {PACKAGE_PIN B16 IOSTANDARD LVCMOS25} [get_ports TWI_SCL]

# SPI from microcontroller
set_property -dict {PACKAGE_PIN AE21 IOSTANDARD LVCMOS25} [get_ports FPGA_SCLK]
set_property -dict {PACKAGE_PIN AD21 IOSTANDARD LVCMOS25} [get_ports FPGA_CSB]
set_property -dict {PACKAGE_PIN AB21 IOSTANDARD LVCMOS25} [get_ports FPGA_MOSI]
set_property -dict {PACKAGE_PIN AC21 IOSTANDARD LVCMOS25} [get_ports FPGA_MISO]

# PMOD1 -- Optional GPS receiver
set_property -dict {PACKAGE_PIN C24 IOSTANDARD LVCMOS25} [get_ports PMOD1_0]
set_property -dict {PACKAGE_PIN C22 IOSTANDARD LVCMOS25} [get_ports PMOD1_1]
set_property -dict {PACKAGE_PIN L23 IOSTANDARD LVCMOS25} [get_ports PMOD1_2]
set_property -dict {PACKAGE_PIN D21 IOSTANDARD LVCMOS25} [get_ports PMOD1_3]
set_property -dict {PACKAGE_PIN K21 IOSTANDARD LVCMOS25} [get_ports PMOD1_4]
set_property -dict {PACKAGE_PIN C18 IOSTANDARD LVCMOS25} [get_ports PMOD1_5]
set_property -dict {PACKAGE_PIN C19 IOSTANDARD LVCMOS25} [get_ports PMOD1_6]
set_property -dict {PACKAGE_PIN C17 IOSTANDARD LVCMOS25} [get_ports PMOD1_7]
set_property PULLUP true [get_ports PMOD1_0]
set_property PULLUP true [get_ports PMOD1_1]
set_property PULLUP true [get_ports PMOD1_2]
set_property PULLUP true [get_ports PMOD1_3]

# PMOD2 -- Display and front panel switches
set_property -dict {PACKAGE_PIN AE7 IOSTANDARD LVCMOS15 DRIVE 16} [get_ports PMOD2_0]
set_property -dict {PACKAGE_PIN V7 IOSTANDARD LVCMOS15 DRIVE 16} [get_ports PMOD2_1]
set_property -dict {PACKAGE_PIN Y7 IOSTANDARD LVCMOS15 DRIVE 16} [get_ports PMOD2_2]
set_property -dict {PACKAGE_PIN AF7 IOSTANDARD LVCMOS15 DRIVE 16} [get_ports PMOD2_3]
set_property -dict {PACKAGE_PIN V8 IOSTANDARD LVCMOS15 DRIVE 16} [get_ports PMOD2_4]
set_property -dict {PACKAGE_PIN AA8 IOSTANDARD LVCMOS15 DRIVE 16} [get_ports PMOD2_5]
set_property -dict {PACKAGE_PIN Y8 IOSTANDARD LVCMOS15} [get_ports PMOD2_6]
set_property -dict {PACKAGE_PIN W9 IOSTANDARD LVCMOS15} [get_ports PMOD2_7]

# 125 MHz from U20
set_property -dict {PACKAGE_PIN AC9 IOSTANDARD DIFF_SSTL15} [get_ports DDR_REF_CLK_P]
set_property -dict {PACKAGE_PIN AD9 IOSTANDARD DIFF_SSTL15} [get_ports DDR_REF_CLK_N]

# VCXO
set_property -dict {PACKAGE_PIN V9 IOSTANDARD LVCMOS15} [get_ports VCXO_EN]

# UART to USB
set_property -dict {PACKAGE_PIN C16 IOSTANDARD LVCMOS25} [get_ports FPGA_TxD]
set_property -dict {PACKAGE_PIN K15 IOSTANDARD LVCMOS25} [get_ports FPGA_RxD]

# Miscellaneous
set_property -dict {PACKAGE_PIN B9 IOSTANDARD LVCMOS25} [get_ports PHY_RSTN]

# MGTREFCLK0_115 (schematic MGT_CLK_2), U2 output 4
set_property PACKAGE_PIN H6 [get_ports MGT_CLK_0_P]
set_property PACKAGE_PIN H5 [get_ports MGT_CLK_0_N]

# MGTREFCLK1_115 (schematic MGT_CLK_3), U2 output 5
set_property -dict {PACKAGE_PIN K6} [get_ports MGT_CLK_1_P]
set_property -dict {PACKAGE_PIN K5} [get_ports MGT_CLK_1_N]

# Transceivers
# FMC1-DP0, Bank 115 MGT 2, X0Y2
set_property -dict {PACKAGE_PIN K2} [get_ports MGT_TX_1_P]
set_property -dict {PACKAGE_PIN K1} [get_ports MGT_TX_1_N]
set_property -dict {PACKAGE_PIN L4} [get_ports MGT_RX_1_P]
set_property -dict {PACKAGE_PIN L3} [get_ports MGT_RX_1_N]
# FMC2-DP0, Bank 115 MGT 0, X0Y0
set_property -dict {PACKAGE_PIN P2} [get_ports MGT_TX_2_P]
set_property -dict {PACKAGE_PIN P1} [get_ports MGT_TX_2_N]
set_property -dict {PACKAGE_PIN R4} [get_ports MGT_RX_2_P]
set_property -dict {PACKAGE_PIN R3} [get_ports MGT_RX_2_N]
### QSFP1-2/11, Bank 116 MGT 2, X0Y6
##set_property -dict {PACKAGE_PIN B2} [get_ports MGT_TX_t1_P]
##set_property -dict {PACKAGE_PIN B1} [get_ports MGT_TX_t1_N]
##set_property -dict {PACKAGE_PIN C4} [get_ports MGT_RX_t1_P]
##set_property -dict {PACKAGE_PIN C3} [get_ports MGT_RX_t1_N]
### QSFP1-3/11, Bank 116 MGT 0, X0Y4
##set_property -dict {PACKAGE_PIN F2} [get_ports MGT_TX_t2_P]
##set_property -dict {PACKAGE_PIN F1} [get_ports MGT_TX_t2_N]
##set_property -dict {PACKAGE_PIN G4} [get_ports MGT_RX_t2_P]
##set_property -dict {PACKAGE_PIN G3} [get_ports MGT_RX_t2_N]

# Direct monitoring of MGT reference clocks
# FMC1 -- H4/H5
set_property -dict {PACKAGE_PIN F17 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports FMC1_CLK0_M2C_P]
set_property -dict {PACKAGE_PIN E17 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports FMC1_CLK0_M2C_N]
# FMC2 -- H4/H5
set_property -dict {PACKAGE_PIN Y23 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports FMC2_CLK0_M2C_P]
set_property -dict {PACKAGE_PIN AA24 IOSTANDARD LVDS_25 DIFF_TERM 1} [get_ports FMC2_CLK0_M2C_N]

# Bank 0 setup
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
