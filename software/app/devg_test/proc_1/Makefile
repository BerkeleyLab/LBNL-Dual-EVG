include ../../../../dir_list.mk

TARGET   ?= devg_test_marble
GW_DEVG_TGT_DIR = $(GW_SYN_DIR)/$(TARGET)
BIT      ?= $(GW_DEVG_TGT_DIR)/$(TARGET)_top.bit

PROC_NAME  = proc_1
CPU_NAME   = microblaze_1
TARGET_DIR = $(SW_TGT_DIR)/$(TARGET)
BSP_DIR    = $(TARGET)_$(PROC_NAME)/bd/$(CPU_NAME)/standalone_domain/bsp/$(CPU_NAME)

all: $(TARGET)_$(PROC_NAME) $(TARGET)_$(PROC_NAME).elf $(TARGET)_$(PROC_NAME).elf.size

__SRC_FILES = \
    NMEA.c \
    epics.c \
    evg.c \
    fmcIO.c \
    gps.c \
    injectionCycle.c \
    ipc.c \
    main.c \
    mgt.c \
    platform.c \
    swapoutCycle.c \
    tod.c \
    util.c
SRC_FILES = $(addprefix $(SW_SRC_DIR)/$(PROC_NAME)/, $(__SRC_FILES))

__HDR_FILES = \
    NMEA.h \
    epics.h \
    evg.h \
    fmcIO.h \
    gps.h \
    injectionCycle.h \
    ipc.h \
    mgt.h \
    platform.h \
    swapoutCycle.h \
	sharedMemory.h \
	systemParameters.h \
    tod.h \
    util.h
HDR_FILES = $(addprefix $(SW_SRC_DIR)/$(PROC_NAME)/, $(__HDR_FILES))

__SRC_BANTAMWEIGHTUDP_FILES = \
	badger.c \
	bwudp.c
SRC_BANTAMWEIGHTUDP_FILES = $(addprefix $(BANTAMWEIGHT_DIR)/, $(__SRC_BANTAMWEIGHTUDP_FILES))

SRC_FILES += $(SRC_BANTAMWEIGHTUDP_FILES)

__HDR_BANTAMWEIGHTUDP_FILES = \
	bwudp_config.h \
	bwudp.h
HDR_BANTAMWEIGHTUDP_FILES = $(addprefix $(BANTAMWEIGHT_DIR)/, $(__HDR_BANTAMWEIGHTUDP_FILES))

HDR_FILES += $(HDR_BANTAMWEIGHTUDP_FILES)

__HDR_GEN_FILES = \
	softwareBuildDate.h
HDR_GEN_FILES = $(addprefix $(SW_SRC_DIR)/$(PROC_NAME)/, $(__HDR_GEN_FILES))

include $(TARGET_DIR)/$(TARGET).mk

OBJ_FILES = $(SRC_FILES:.c=.o)
LINKER_FILES = $(SW_SRC_DIR)/$(PROC_NAME)/lscript.ld

CFLAGS = -Wall -O2 -mlittle-endian -mxl-barrel-shift -mno-xl-soft-div -mcpu=v11.0 -mno-xl-soft-mul -ffunction-sections -fdata-sections
LD_FLAGS = -Wl,-T -Wl,$(LINKER_FILES) -Wl,--no-relax
USER_FLAGS = -D__BAREMETAL__ -DBWUDP_ENDPOINT_CAPACITY=5 -DBWUDP_ENABLE_CLIENT_SUPPORT -DBWUDP_USE_REAL_GPIO_H
LIB_DIRS = $(BSP_DIR)/lib
LIB_FLAGS = $(addprefix -L, $(LIB_DIRS))
LIBS = -Wl,--start-group,-lxil,-lgcc,-lc,--end-group

ifeq ($(TARGET),devg_test_marble)
	USER_FLAGS += -D__TARGET_DEVG_TEST_MARBLE__
else
	USER_FLAGS += -D__TARGET_NOT_RECOGNIZED__
endif

INCLUDE_DIRS = \
	$(SW_SRC_DIR)/$(PROC_NAME) \
	$(TARGET_DIR) \
	$(BSP_DIR)/include \
	$(BANTAMWEIGHT_DIR) \
	$(ST7789V_DIR)
INCLUDE_FLAGS = $(addprefix -I, $(INCLUDE_DIRS))

.PHONY: all clean bsp

vpath *.c $(SW_SRC_DIR)/$(PROC_NAME) $(TARGET_DIR)
vpath *.h $(SW_SRC_DIR)/$(PROC_NAME) $(TARGET_DIR)
vpath *.o $(SW_SRC_DIR)/$(PROC_NAME)

# Actual build rules
include $(SW_DEVG_TEST_APP_DIR)/bottom_rules.mk
