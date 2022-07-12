axi_lite_generic_reg_DIR = $(MODULES_DIR)/axi_lite_generic_reg
__axi_lite_generic_reg_SRCS = \
							axi_lite_generic_reg_S00_AXI.v \
							axi_lite_generic_reg.v
axi_lite_generic_reg_SRCS = $(addprefix $(axi_lite_generic_reg_DIR)/, $(__axi_lite_generic_reg_SRCS))
axi_lite_generic_reg_VERSION = 2.0
axi_lite_generic_reg_TARGET = _gen/axi_lite_generic_reg

IP_CORES_CUSTOM += axi_lite_generic_reg
IP_CORES_CUSTOM_TARGET_DIRS += $(axi_lite_generic_reg_TARGET)

axi_lite_generic_reg: $(axi_lite_generic_reg_SRCS)
	$(VIVADO_CREATE_IP) $@ $($@_TARGET) $($@_VERSION) $^
	touch $@

CLEAN_DIRS += $(axi_lite_generic_reg_TARGET)
