__DEVG_MARBLE_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
DEVG_MARBLE_DIR := $(__DEVG_MARBLE_DIR:/=)

__HDR_DEVG_MARBLE_FILES = \
	gpio.h \
	config.h
HDR_DEVG_MARBLE_FILES = $(addprefix $(DEVG_MARBLE_DIR)/, $(__HDR_DEVG_MARBLE_FILES))

# For top-level makfile
HDR_FILES += $(HDR_GEN_DEVG_MARBLE_FILES)
# SRC_FILES +=

# clean::
# 	$(RM) -rf
