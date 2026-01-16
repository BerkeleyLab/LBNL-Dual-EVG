HARDWARE = marble
PROJECT_PART = xc7k160tffg676-2
PROJECT_BOARD = none
COMMUNICATION = gtx
XILINX_TOOL := VIVADO
FPGA_VENDOR := xilinx
FPGA_PLATFORM := 7series

CMD_SEP = ;

include ../../../dir_list.mk

# Top level design name
TARGET ?= devg_marble
FPGA_APPLICATION ?= devg
EVIO_TYPE ?= evio

VENDOR_PLATFORM_DIR = $(PLATFORM_DIR)/$(FPGA_VENDOR)
TARGET_PLATFORM_DIR = $(VENDOR_PLATFORM_DIR)/$(FPGA_PLATFORM)/$(FPGA_APPLICATION)
GW_DEVG_TGT_DIR = $(GW_SYN_DIR)/$(TARGET)
XPR             = $(GW_DEVG_TGT_DIR)/_xilinx/$(TARGET)_top/$(TARGET)_top.xpr

# Top level bitstream name
all: $(TARGET)_top.bit $(TARGET)_top.mmi

# Search for .v files here
vpath %.v $(MODULES_DIR)
vpath %.v $(VENDOR_PLATFORM_DIR)
# Bedrock .v files
vpath %.v $(DSP_DIR)
vpath %.v $(BADGER_DIR)
vpath %.v $(PERIPH_DRIVERS_DIR)
vpath %.v $(PERIPH_DRIVERS_DIR)/i2cbridge

TOP_LVL_DIR = ../../top/$(TARGET)

# top-level files go here
vpath %.v $(TOP_LVL_DIR)
vpath %.xdc $(TOP_LVL_DIR)

VIVADO_SYNTH_EXTRA_TCL += prop.tcl ipcores_repos.tcl prio_xci_files.tcl

include $(BUILD_DIR)/top_rules.mk

# ignore xilinx primitives
UNISIM_CRAP += -e 'GTXE2_CHANNEL|MMCME2_ADV'

# Top level files for depedency tracking
SYSTEM_TOP_PINS_XDC = $(TARGET)_top_pins.xdc $(EVIO_TYPE).xdc
SYSTEM_TOP_TIM_XDC = $(TARGET)_top_tim.xdc

# ip_cores generation scripts/names

# custom ip_cores that requires packaging
include $(MODULES_DIR)/axi_lite_generic_reg_rules.mk
include $(MODULES_DIR)/bantamweightUDP_rules.mk

IP_CORES += evg1mgt evg2mgt evLogFIFO ila_td256_s4096_cap
evg1mgt_DIR = $(TARGET_PLATFORM_DIR)/evg1mgt
evg1mgt_TOP = $(evg1mgt_DIR)/evg1mgt.v
evg2mgt_DIR = $(TARGET_PLATFORM_DIR)/evg2mgt
evg2mgt_TOP = $(evg2mgt_DIR)/evg2mgt.v
evLogFIFO_DIR = $(TARGET_PLATFORM_DIR)/evLogFIFO
evLogFIFO_TOP = $(evLogFIFO_DIR)/synth/evLogFIFO.vhd
ila_td256_s4096_cap_DIR = $(TARGET_PLATFORM_DIR)/ila_td256_s4096_cap
ila_td256_s4096_cap_TOP = $(ila_td256_s4096_cap_DIR)/synth/ila_td256_s4096_cap.v

IP_CORES_TOP_LVL_SRCS += $(evg1mgt_TOP) $(evg2mgt_TOP) $(evLogFIFO_TOP) $(ila_td256_s4096_cap_TOP)
IP_CORES_DIRS += $(evg1mgt_DIR) $(evg1mgt_DIR)/evg1mgt/example_design \
				 $(evg2mgt_DIR) $(evg2mgt_DIR)/evg2mgt/example_design \
				 $(evLogFIFO_DIR) $(ila_td256_s4096_cap_DIR)
IP_CORES_XDCS += $(evg1mgt_TOP:%.v=%.xdc) $(evg2mgt_TOP:%.v=%.xdc)

IP_CORES_SRCS_DIRS = $(IP_CORES_DIRS)
IP_CORES_SRCS_DIRS += $(addsuffix /synth, $(IP_CORES_DIRS))
IP_CORES_SRCS_DIRS += $(addsuffix /hdl, $(IP_CORES_DIRS))

BD += bd
bd_DIR = $(TARGET_PLATFORM_DIR)/bd
bd_TOP = $(bd_DIR)/synth/bd.v

BD_TOP_LVL_SRCS += $(bd_TOP)
BD_DIRS += $(bd_DIR)

BD_SRCS_DIRS = $(BD_DIRS)
BD_SRCS_DIRS += $(addsuffix /synth, $(BD_DIRS))
BD_SRCS_DIRS += $(addsuffix /hdl, $(BD_DIRS))

BD_bds = $(addsuffix .bd, $(BD))

# For iverilog to search for dependencies

# Bedrock directories for searching
VFLAGS_DEP += -y$(DSP_DIR)
VFLAGS_DEP += -y$(FPGA_FAMILY_DIR)
VFLAGS_DEP += -y$(FPGA_FAMILY_DIR)/xilinx
VFLAGS_DEP += -y$(FPGA_FAMILY_DIR)/iserdes
VFLAGS_DEP += -y$(ISERDES_DIR)
VFLAGS_DEP += -y$(XILINX_DIR)
VFLAGS_DEP += -y$(SERIAL_IO_DIR)
VFLAGS_DEP += -y$(PERIPH_DRIVERS_DIR)
VFLAGS_DEP += -y$(PERIPH_DRIVERS_DIR)/i2cbridge
VFLAGS_DEP += -y$(HOMELESS_DIR)
VFLAGS_DEP += -y$(BADGER_DIR)
VFLAGS_DEP += -y$(BADGER_DIR)/tests

# Local directories for searching
VFLAGS_DEP += -y.
VFLAGS_DEP += -y$(MODULES_DIR)
VFLAGS_DEP += -y$(TOP_LVL_DIR)
VFLAGS_DEP += -y$(VENDOR_PLATFORM_DIR)
VFLAGS_DEP += -y$(PLATFORM_7SERIES_MGT_DIR)
VFLAGS_DEP += $(addprefix -y, $(IP_CORES_SRCS_DIRS))
VFLAGS_DEP += $(addprefix -y, $(BD_SRCS_DIRS))

# Local directories for include searching
VFLAGS_DEP += -I.
VFLAGS_DEP += -I$(MODULES_DIR)
VFLAGS_DEP += -I$(TOP_LVL_DIR)
VFLAGS_DEP += $(addprefix -I, $(IP_CORES_SRCS_DIRS))
VFLAGS_DEP += $(addprefix -I, $(BD_SRCS_DIRS))

# Files to be treated as library files. This is needed if the filename
# does not match the module name, which is the case for Xilinx ip cores.
# As we don't exactly know the names of the sources, create a file with
# the switches that will be consumed by iverilog later
evg1mgt_VFLAGS_COMMAND_FILE = evg1mgt_iverilog_cfile.txt
evg2mgt_VFLAGS_COMMAND_FILE = evg2mgt_iverilog_cfile.txt
evLogFIFO_VFLAGS_COMMAND_FILE = evLogFIFO_iverilog_cfile.txt
ila_td256_s4096_cap_VFLAGS_COMMAND_FILE = ila_td256_s4096_cap_iverilog_cfile.txt

VFLAGS_COMMAND_FILE += \
					  $(evg1mgt_VFLAGS_COMMAND_FILE) \
					  $(evg2mgt_VFLAGS_COMMAND_FILE)
# $(evLogFIFO_VFLAGS_COMMAND_FILE)
# using ILA with verilog gives an error:
# input buffer overflow, can't enlarge buffer because scanner uses REJECT,
# but we don't need it for iverilog as we only use it when not simulating
# $(ila_td256_s4096_cap_VFLAGS_COMMAND_FILE)

VFLAGS_DEP += $(addprefix -c, $(VFLAGS_COMMAND_FILE))

VFLAGS_DEP += -DSIMULATE

# Badger build
RTEFI_CLIENT_LIST = hello.v
include $(BADGER_DIR)/rules.mk

# Search prerequisites for generated .v/.vh/.vhd/.xdc here
vpath %.v $(IP_CORES_SRCS_DIRS) $(BD_SRCS_DIRS)
vpath %.vh $(IP_CORES_SRCS_DIRS) $(BD_SRCS_DIRS)
vpath %.vhd $(IP_CORES_SRCS_DIRS) $(BD_SRCS_DIRS)
vpath %.xdc $(IP_CORES_SRCS_DIRS) $(BD_SRCS_DIRS)
vpath %.tcl $(GW_SCRIPTS_DIR)
vpath %.sh $(GW_SCRIPTS_DIR)
vpath %.dat $(MODULES_DIR)

# List top level depedencies here. Eveything named .bit will be used as
# a top-level target in top_rules.mk. .v and .sv files will be used as
# prerequisites as well as a system_top.xdc file
$(TARGET)_top.bit: system_top.xdc $(VIVADO_SYNTH_EXTRA_TCL) $(TOP_LVL_DIR)/$(TARGET)_top.v iicCommandTable.dat $(BD_bds) evLogFIFO.xci ila_td256_s4096_cap.xci
$(TOP_LVL_DIR)/$(TARGET)_top.v: $(TOP_LVL_DIR)/gitHash.vh $(TOP_LVL_DIR)/firmwareBuildDate.v $(TOP_LVL_DIR)/gpioIDX.vh $(RTEFI_V) $(IP_CORES_CUSTOM) $(IP_CORES_TOP_LVL_SRCS) $(BD_TOP_LVL_SRCS)

$(TARGET)_top.mmi: $(XPR) $(TARGET)_top.bit
	vivado -mode batch -nojou -nolog -source $(GW_SCRIPTS_DIR)/gen_mem_info.tcl -tclargs $< $@

# Generate bundled .xdc from other .xdc. Don't change the order of the
# prerequisites as we need constraints in tGe order specified
system_top.xdc: $(SYSTEM_TOP_PINS_XDC) $(IP_CORES_XDCS) $(SYSTEM_TOP_TIM_XDC)
	cat $^ > $@

# Generate firmware date and parameters
%firmwareBuildDate.v: set_firmware_build_date.tcl
	$(VIVADO_CMD) -source $< -tclargs $@

%gitHash.vh: gitHashVerilog.sh
	sh $(GW_SCRIPTS_DIR)/gitHashVerilog.sh > $@

%gpioIDX.vh: $(SW_TGT_DIR)/$(FPGA_APPLICATION)_$(HARDWARE)/config.h $(SW_TGT_DIR)/$(FPGA_APPLICATION)_$(HARDWARE)/gpio.h
	sh $(GW_SCRIPTS_DIR)/createVerilogIDX.sh $^ > $@

ipcores_repos.tcl:
	sed -e 's|$$DIRS|$(IP_CORES_CUSTOM_TARGET_DIRS)|g' \
		ipcores_repos.tcl.in > $@

prio_xci_files.tcl: ila_td256_s4096_cap.xci
	sed -e 's|$$XCI_FILES|$<|g' \
		prio_xci_files.tcl.in > $@

# ip_cores commands. Keep track only of the top-level. If this changed or doesn't
# exist regenerate eveything. Otherwise, consider this up to date
vpath %.xci $(IP_CORES_SRCS_DIRS) $(BD_SRCS_DIRS)
vpath %.bd $(IP_CORES_SRCS_DIRS) $(BD_SRCS_DIRS)

# Use second expansion here so we can get the matched target name in the prerequisite
# and use the defined variable to find the .xci one level below.
# FIXME: Is there an easier way to do this?
.SECONDEXPANSION:
$(IP_CORES_TOP_LVL_SRCS): % : $$(basename $$(notdir %)).xci
	$(VIVADO_CMD) -source $(GW_SCRIPTS_DIR)/ip_core_proc.tcl $(GW_SCRIPTS_DIR)/gen_ip_core.tcl -tclargs $< $(PROJECT_PART) $(PROJECT_BOARD)
	# fix line-continuation between (* *) attribute that iverilog doesn't understand
	# from: https://catonmat.net/sed-one-liners-explained-part-one,
	# 39. Append a line to the next if it ends with a backslash "\".
	TOP_LVL="$*"; if [ "$${TOP_LVL##*.}" = "v" ] || [ "$${TOP_LVL##*.}" = "sv" ]; then sed -i -e :a -e '/\\$$/N; s/\\\n//; ta' $${TOP_LVL}; fi
	# convert modules instance names to lowercase so iverilog can match and generate
	# the depedency file for it
	(find $($(basename $(notdir $@))_DIR) -not \( -name \*rfs.v \) -name "*.v" -exec sed -i -e 's/\($(basename $(notdir $@))_[A-Za-z0-9_]\+\)/\L\1/' '{}' \; || true)
	# generate iverilog switches
	rm -f $($(basename $(notdir $@))_VFLAGS_COMMAND_FILE)
	(find $($(basename $(notdir $@))_DIR)/hdl $($(basename $(notdir $@))_DIR)/synth -mindepth 1 -not \( -name \*rfs.v \) -name "*.v" -exec echo "-l"'{}' \; || true) >> $($(basename $(notdir $@))_VFLAGS_COMMAND_FILE)
	(find $($(basename $(notdir $@))_DIR) -mindepth 1 -maxdepth 1 -not \( -name \*rfs.v \) -name "*.v" -exec echo "-l"'{}' \; || true) >> $($(basename $(notdir $@))_VFLAGS_COMMAND_FILE)

$(BD_TOP_LVL_SRCS): % : $$(basename $$(notdir %)).bd
	$(VIVADO_CMD) -source $(GW_SCRIPTS_DIR)/bd_proc.tcl $(GW_SCRIPTS_DIR)/gen_bd.tcl -tclargs $< $(PROJECT_PART) $(PROJECT_BOARD) $(IP_CORES_CUSTOM_TARGET_DIRS)

# clean generate IP cores files, but the source ones (.xci or .bd)
clean::
	$(foreach ipcore, \
		$(IP_CORES), \
		test -f $($(ipcore)_DIR)/$(ipcore).xci && \
		find $($(ipcore)_DIR) -mindepth 1 -not \( -name \*$(ipcore).xci -o -name \*$(ipcore).bd -o -name \*$(ipcore).coe \) -delete $(CMD_SEP))
	$(foreach bd, \
		$(BD), \
		test -f $($(bd)_DIR)/$(bd).bd && \
		find $($(bd)_DIR) -mindepth 1 -not \( -name \*$(ipcore).bd \) -delete $(CMD_SEP))

# Cleaning rules
include $(BUILD_DIR)/bottom_rules.mk

CLEAN += *.bit
CLEAN += *.bin
CLEAN += *.prm
CLEAN += system_top.xdc
CLEAN += $(TOP_LVL_DIR)/firmwareBuildDate.v
CLEAN += $(TOP_LVL_DIR)/gpioIDX.vh
CLEAN += $(TOP_LVL_DIR)/gitHash.vh
CLEAN += $(IP_CORES_CUSTOM)
CLEAN += $(VFLAGS_COMMAND_FILE)
# Not adding it to VFLAGS_COMMAND_FILE because iverilog gives an error:
# input buffer overflow, can't enlarge buffer because scanner uses REJECT,
# but we don't need it for iverilog as we only use it when not simulating
CLEAN += $(ila_td256_s4096_cap_VFLAGS_COMMAND_FILE)
CLEAN += $(evLogFIFO_VFLAGS_COMMAND_FILE)
CLEAN += *.log *.jou *.str
CLEAN += *.xsa
CLEAN += psu_init*
CLEAN += ipcores_repos.tcl
CLEAN += prio_xci_files.tcl
CLEAN += $(RTEFI_CLEAN)
CLEAN_DIRS +=  _xilinx .Xil .ip_user_files .hbs $(IP_CORES_CUSTOM_TARGET_DIRS)
CLEAN_DIRS += _gen

# Dependency rules for top level. It will include the dependencies for
# the bitstream %.bit and use the stem to list the depedencies for the
# top level named %.v or %.sv
ifneq (,$(findstring bit,$(MAKECMDGOALS)))
    ifneq (,$(findstring bits,$(MAKECMDGOALS)))
	include $(BITS_:%.bit=$(DEPDIR)/%.bit.d)
    else
	include $(MAKECMDGOALS:%.bit=$(DEPDIR)/%.bit.d)
    endif
endif
