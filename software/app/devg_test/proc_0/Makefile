include ../../../../dir_list.mk

TARGET   ?= devg_test_marble
GW_DEVG_TGT_DIR = $(GW_SYN_DIR)/$(TARGET)
BIT      ?= $(GW_DEVG_TGT_DIR)/$(TARGET)_top.bit

PROC_NAME  = proc_0
CPU_NAME   = microblaze_0
TARGET_DIR = $(SW_TGT_DIR)/$(TARGET)
BSP_DIR    = $(SW_DEVG_TEST_APP_DIR)/$(PROC_NAME)/$(TARGET)_$(PROC_NAME)/bd/$(CPU_NAME)/standalone_domain/bsp/$(CPU_NAME)

all: $(TARGET)_$(PROC_NAME) $(TARGET)_$(PROC_NAME).elf $(TARGET)_$(PROC_NAME).elf.size

__SRC_FILES = \
	bootFlash.c \
	console.c \
	display.c \
	drawEventLog.c \
	evgCoincidence.c \
	evio.c \
	eyescan.c \
	iicChunk.c \
	iicFMC.c \
	iicProc.c \
	main.c \
	mgtClkSwitchTest.c \
	mmcMailbox.c \
	platform.c \
	sysmon.c \
	systemParameters.c \
	tftp.c \
	util.c \
	xadc.c
SRC_FILES = $(addprefix $(SW_SRC_DIR)/$(PROC_NAME)/, $(__SRC_FILES))

__HDR_FILES = \
	bootFlash.h \
	console.h \
	display.h \
	drawEventLog.h \
	evgCoincidence.h \
	evio.h \
	evg.h \
	eyescan.h \
	iicChunk.h \
	iicFMC.h \
	iicProc.h \
	injectionCycle.h \
	mgtClkSwitch.h \
	mmcMailbox.h \
	platform_config.h \
	platform.h \
	sharedMemory.h \
	spiflash.h \
	sysmon.h \
	systemParameters.h \
	tftp.h \
	tod.h \
	util.h \
	xadc.h
HDR_FILES = $(addprefix $(SW_SRC_DIR)/$(PROC_NAME)/, $(__HDR_FILES))

__HDR_BANTAMWEIGHTUDP_FILES = \
	bwudp_config.h \
	bwudp.h
HDR_BANTAMWEIGHTUDP_FILES = $(addprefix $(BANTAMWEIGHT_DIR)/, $(__HDR_BANTAMWEIGHTUDP_FILES))

HDR_FILES += $(HDR_BANTAMWEIGHTUDP_FILES)

__HDR_GEN_FILES = \
	softwareBuildDate.h
HDR_GEN_FILES = $(addprefix $(SW_SRC_DIR)/$(PROC_NAME)/, $(__HDR_GEN_FILES))

include $(TARGET_DIR)/$(TARGET).mk

ST7789V_DIR = $(SW_LIBS_DIR)/st7789v
__ST7789V_SRC_FILES = \
	systemFont.c
__ST7789V_HDR_FILES =\
	lv_font.h \
	st7789v.h

ifeq ($(TARGET),devg_test_marble)
	__ST7789V_SRC_FILES += \
		st7789v.c
endif

ST7789V_SRC_FILES = $(addprefix $(ST7789V_DIR)/, $(__ST7789V_SRC_FILES))
ST7789V_HDR_FILES = $(addprefix $(ST7789V_DIR)/, $(__ST7789V_HDR_FILES))
SRC_FILES += $(ST7789V_SRC_FILES)
HDR_FILES += $(ST7789V_HDR_FILES)

OBJ_FILES = $(SRC_FILES:.c=.o)
LINKER_FILES = $(SW_SRC_DIR)/$(PROC_NAME)/lscript.ld

CFLAGS = -Wall -O2 -mlittle-endian -mxl-barrel-shift -mno-xl-soft-div -mcpu=v11.0 -mno-xl-soft-mul -ffunction-sections -fdata-sections
LD_FLAGS = -Wl,-T -Wl,$(LINKER_FILES) -Wl,--no-relax
USER_FLAGS = -D__BAREMETAL__
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

vpath *.c $(SW_SRC_DIR)/$(PROC_NAME) $(ST7789V_DIR) $(TARGET_DIR)
vpath *.h $(SW_SRC_DIR)/$(PROC_NAME) $(ST7789V_DIR) $(TARGET_DIR)
vpath *.o $(SW_SRC_DIR)/$(PROC_NAME) $(ST7789V_DIR)

# Actual build rules
include $(SW_DEVG_TEST_APP_DIR)/bottom_rules.mk
