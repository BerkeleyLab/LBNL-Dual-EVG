# RGMII clock
create_clock -period 8.000 -name rx_clk [get_ports RGMII_RX_CLK]

# DDR clock ref
#create_clock -period 8.000 -name ddr_ref_clk [get_ports DDR_REF_CLK_P]

# 20 MHz from Y3
#set_property -dict {PACKAGE_PIN W11 IOSTANDARD LVCMOS15} [get_ports CLK20_VCXO]
#create_clock -period 50.000 -name clk20_vcxo [get_ports CLK20_VCXO]
#set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets CLK20_VCXO]

# MGTREFCLK0_115 (schematic MGT_CLK_2), U2 output 4
create_clock -period 8.001 -name gtx_ref0 [get_ports MGT_CLK_0_P]
set clk_gtx_ref0_period                   [get_property PERIOD [get_clocks gtx_ref0]]

# MGTREFCLK1_115 (schematic MGT_CLK_3), U2 output 5
create_clock -period 7.999 -name gtx_ref1 [get_ports MGT_CLK_1_P]
set clk_gtx_ref1_period                   [get_property PERIOD [get_clocks gtx_ref1]]

# MGT generated clocks
set clk_evg1mgt_TXOUTCLK_period           [get_property PERIOD [get_clocks evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/TXOUTCLK]]
set clk_evg2mgt_TXOUTCLK_period           [get_property PERIOD [get_clocks evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/TXOUTCLK]]

#########################################
# Phase measurement FF. We need them to be always have the same
# delay from the clock that is driving them, as the following CDC
# FF will sample this signal and compare essentially the edges of
# this
#########################################

set_property LOC SLICE_X86Y182 [get_cells coincidenceRecorder1/genblk2[0].value_a_reg[0]]
set_property LOC SLICE_X86Y181 [get_cells coincidenceRecorder2/genblk2[0].value_a_reg[0]]

#########################################
# Sampling a clock with another. We want to account
# for the source and destination clock delay too, as
# the total delay matters when calculating the clock
# phase difference offset
#########################################

set_max_delay -from [get_clocks gtx_ref0] -to [get_clocks gtx_ref1] 2.0
set_max_delay -from [get_clocks gtx_ref1] -to [get_clocks gtx_ref0] 2.0

#########################################
# Don't check timing across clock domains.
#########################################

set_false_path -from [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT1]]
set_false_path -from [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]] -to [get_clocks rx_clk]
set_false_path -from [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT1]] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]]
set_false_path -from [get_clocks rx_clk] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]]
set_false_path -from [get_clocks rx_clk] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT1]]
set_false_path -from [get_clocks gtx_ref0] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]]
set_false_path -from [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]] -to [get_clocks gtx_ref0]
set_false_path -from [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]] -to [get_clocks gtx_ref1]
set_false_path -from [get_clocks gtx_ref1] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]]
set_false_path -from [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT3]] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]]
set_false_path -from [get_clocks evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/TXOUTCLK] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]]
set_false_path -from [get_clocks evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/RXOUTCLK] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]]
set_false_path -from [get_clocks evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/TXOUTCLK] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]]
set_false_path -from [get_clocks evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/RXOUTCLK] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT3]]
set_false_path -from [get_clocks evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/TXOUTCLK] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT3]]
set_false_path -from [get_clocks evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/RXOUTCLK] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT3]]
set_false_path -from [get_clocks evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/TXOUTCLK] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT3]]
set_false_path -from [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]] -to [get_clocks evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/TXOUTCLK]
set_max_delay -datapath_only -from [get_clocks gtx_ref1] -to [get_clocks evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/TXOUTCLK] $clk_evg1mgt_TXOUTCLK_period
set_false_path -from [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]] -to [get_clocks evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/TXOUTCLK]
set_max_delay -datapath_only -from [get_clocks gtx_ref0] -to [get_clocks evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/TXOUTCLK] $clk_evg2mgt_TXOUTCLK_period
set_max_delay -datapath_only -from [get_clocks evg2mgt/evg2mgt_i/inst/evg2mgt_i/gt0_evg2mgt_i/gtxe2_i/TXOUTCLK] -to [get_clocks gtx_ref0] $clk_gtx_ref0_period
set_max_delay -datapath_only -from [get_clocks evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/TXOUTCLK] -to [get_clocks gtx_ref1] $clk_gtx_ref1_period
set_false_path -from [get_clocks evg1mgt/evg1mgt_i/inst/evg1mgt_i/gt0_evg1mgt_i/gtxe2_i/RXOUTCLK] -to [get_clocks -of_objects [get_pins bd_i/clk_wiz_1/inst/mmcm_adv_inst/CLKOUT0]]
