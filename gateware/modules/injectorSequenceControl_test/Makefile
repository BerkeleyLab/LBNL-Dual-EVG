include ../../../dir_list.mk

DUT                 = injectorSequenceControl_test_wrapper
TOPLEVEL            = $(DUT)
COCOTB_TEST_MODULES = $(DUT)_test

VERILOG_SOURCES += \
	$(MODULES_DIR)/$(DUT).v \
	$(MODULES_DIR)/injectorSequenceControl.v \
	$(MODULES_DIR)/powerlineTrigger.v \
	$(MODULES_DIR)/forwardData.v \
	$(MODULES_DIR)/alignmentGenerator.v \
	$(MODULES_DIR)/mod125_reduction.sv \
	$(MODULES_DIR)/evgCounters.v \
	$(MODULES_DIR)/clkGen.v \
	$(MODULES_DIR)/pulseWatchdog.v \
	$(MODULES_DIR)/pulseSync.v \
	$(MODULES_DIR)/heartbeatGenerator.v \
	$(MODULES_DIR)/clkIntervalCounters.v \
	$(MODULES_DIR)/debounceFallingEdge.v

include ../rules_cocotb.mk
