# RGMII clock
create_clock -period 8.000 -name rx_clk [get_ports RGMII_RX_CLK]

# DDR clock ref
#create_clock -period 8.000 -name ddr_ref_clk [get_ports DDR_REF_CLK_P]

# 20 MHz from Y3
#set_property -dict {PACKAGE_PIN W11 IOSTANDARD LVCMOS15} [get_ports CLK20_VCXO]
#create_clock -period 50.000 -name clk20_vcxo [get_ports CLK20_VCXO]
#set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets CLK20_VCXO]

# MGTREFCLK0_116 (schematic MGT_CLK_0), U2 output 0
create_clock -period 8.001 -name gtx_ref0 [get_ports MGT_CLK_0_P]

# MGTREFCLK1_116 (schematic MGT_CLK_1), U2 output 1
create_clock -period 8.001 -name gtx_ref1 [get_ports MGT_CLK_1_P]

# MGTREFCLK0_115 (schematic MGT_CLK_2), U2 output 4
create_clock -period 7.999 -name gtx_ref2 [get_ports MGT_CLK_2_P]

# MGTREFCLK1_115 (schematic MGT_CLK_3), U2 output 5
create_clock -period 7.999 -name gtx_ref3 [get_ports MGT_CLK_3_P]

# EXT0_CLK
create_clock -period 8.001 -name refCoinc1 [get_ports EXT0_CLK_P]
set clk_refCoinc1_period                   [get_property PERIOD [get_clocks refCoinc1]]

# EXT1_CLK
create_clock -period 7.999 -name refCoinc2 [get_ports EXT1_CLK_P]
set clk_refCoinc2_period                   [get_property PERIOD [get_clocks refCoinc2]]

# MGT generated clocks
set clk_evg1mgt_0_TXOUTCLK_period            [get_property PERIOD [get_clocks evg1_mgt_fanout[0].evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/TXOUTCLK]]
set clk_evg1mgt_1_TXOUTCLK_period            [get_property PERIOD [get_clocks evg1_mgt_fanout[1].evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/TXOUTCLK]]
set clk_evg1mgt_2_TXOUTCLK_period            [get_property PERIOD [get_clocks evg1_mgt_fanout[2].evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/TXOUTCLK]]
set clk_evg1mgt_3_TXOUTCLK_period            [get_property PERIOD [get_clocks evg1_mgt_fanout[3].evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/TXOUTCLK]]

set clk_evg2mgt_0_TXOUTCLK_period            [get_property PERIOD [get_clocks evg2_mgt_fanout[0].evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/TXOUTCLK]]
set clk_evg2mgt_1_TXOUTCLK_period            [get_property PERIOD [get_clocks evg2_mgt_fanout[1].evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/TXOUTCLK]]
set clk_evg2mgt_2_TXOUTCLK_period            [get_property PERIOD [get_clocks evg2_mgt_fanout[2].evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/TXOUTCLK]]
set clk_evg2mgt_3_TXOUTCLK_period            [get_property PERIOD [get_clocks evg2_mgt_fanout[3].evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/TXOUTCLK]]

#########################################
# Sampling a clock with another
#########################################

set_max_delay -datapath_only -from [get_ports EXT0_CLK_P] -to [get_clocks refCoinc2] $clk_refCoinc2_period
set_max_delay -datapath_only -from [get_ports EXT1_CLK_P] -to [get_clocks refCoinc1] $clk_refCoinc1_period

#########################################
# Don't check timing across clock domains.
#########################################

set_false_path -from [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT1]]
set_false_path -from [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]] -to [get_clocks rx_clk]
set_false_path -from [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT1]] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]]
set_false_path -from [get_clocks rx_clk] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]]
set_false_path -from [get_clocks rx_clk] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT1]]
set_false_path -from [get_clocks refCoinc1] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]]
set_false_path -from [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]] -to [get_clocks refCoinc1]
set_false_path -from [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]] -to [get_clocks refCoinc2]
set_false_path -from [get_clocks refCoinc2] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]]
set_false_path -from [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT3]] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]]
set_false_path -from [get_clocks *evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/TXOUTCLK] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]]
set_false_path -from [get_clocks *evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/RXOUTCLK] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]]
set_false_path -from [get_clocks *evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/TXOUTCLK] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]]
set_false_path -from [get_clocks *evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/RXOUTCLK] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT3]]
set_false_path -from [get_clocks *evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/TXOUTCLK] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT3]]
set_false_path -from [get_clocks *evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/RXOUTCLK] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT3]]
set_false_path -from [get_clocks *evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/TXOUTCLK] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT3]]
set_false_path -from [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]] -to [get_clocks *evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/TXOUTCLK]
set_max_delay -datapath_only -from [get_clocks refCoinc2] -to [get_clocks evg1_mgt_fanout[0].evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/TXOUTCLK] $clk_evg1mgt_0_TXOUTCLK_period
set_max_delay -datapath_only -from [get_clocks refCoinc2] -to [get_clocks evg1_mgt_fanout[1].evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/TXOUTCLK] $clk_evg1mgt_1_TXOUTCLK_period
set_max_delay -datapath_only -from [get_clocks refCoinc2] -to [get_clocks evg1_mgt_fanout[2].evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/TXOUTCLK] $clk_evg1mgt_2_TXOUTCLK_period
set_max_delay -datapath_only -from [get_clocks refCoinc2] -to [get_clocks evg1_mgt_fanout[3].evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/TXOUTCLK] $clk_evg1mgt_3_TXOUTCLK_period
set_false_path -from [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]] -to [get_clocks *evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/TXOUTCLK]
set_max_delay -datapath_only -from [get_clocks refCoinc1] -to [get_clocks evg2_mgt_fanout[0].evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/TXOUTCLK] $clk_evg2mgt_0_TXOUTCLK_period
set_max_delay -datapath_only -from [get_clocks refCoinc1] -to [get_clocks evg2_mgt_fanout[1].evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/TXOUTCLK] $clk_evg2mgt_1_TXOUTCLK_period
set_max_delay -datapath_only -from [get_clocks refCoinc1] -to [get_clocks evg2_mgt_fanout[2].evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/TXOUTCLK] $clk_evg2mgt_2_TXOUTCLK_period
set_max_delay -datapath_only -from [get_clocks refCoinc1] -to [get_clocks evg2_mgt_fanout[3].evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/TXOUTCLK] $clk_evg2mgt_3_TXOUTCLK_period
set_max_delay -datapath_only -from [get_clocks *evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/TXOUTCLK] -to [get_clocks refCoinc1] $clk_refCoinc1_period
set_max_delay -datapath_only -from [get_clocks *evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/TXOUTCLK] -to [get_clocks refCoinc2] $clk_refCoinc2_period
set_false_path -from [get_clocks *evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/RXOUTCLK] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]]
