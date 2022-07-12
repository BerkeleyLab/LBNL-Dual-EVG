TOP := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

GATEWARE_DIR       = $(TOP)gateware
SOFTWARE_DIR       = $(TOP)software

# Gateware

SUBMODULES_DIR     = $(GATEWARE_DIR)/submodules
MODULES_DIR        = $(GATEWARE_DIR)/modules
PLATFORM_DIR       = $(GATEWARE_DIR)/platform
GW_SCRIPTS_DIR     = $(GATEWARE_DIR)/scripts

BEDROCK_DIR        = $(SUBMODULES_DIR)/bedrock
BANTAMWEIGHT_DIR   = $(SUBMODULES_DIR)/bantamweightUDP
PLATFORM_7SERIES_DIR  = $(PLATFORM_DIR)/xilinx/7series
PLATFORM_7SERIES_DEVG_DIR  = $(PLATFORM_7SERIES_DIR)/devg
PLATFORM_7SERIES_MGT_DIR  = $(PLATFORM_7SERIES_DIR)/mgt

GW_SYN_DIR         = $(GATEWARE_DIR)/syn

# Sofware

SW_LIBS_DIR        = $(SOFTWARE_DIR)/libs
SW_TGT_DIR         = $(SOFTWARE_DIR)/target
SW_SCRIPTS_DIR     = $(SOFTWARE_DIR)/scripts
SW_SRC_DIR     	   = $(SOFTWARE_DIR)/src
SW_APP_DIR         = $(SOFTWARE_DIR)/app

# DEVG Software

SW_DEVG_APP_DIR    = $(SW_APP_DIR)/devg
SW_DEVG_SCRIPTS_DIR = $(SW_DEVG_APP_DIR)/scripts

include $(BEDROCK_DIR)/dir_list.mk
