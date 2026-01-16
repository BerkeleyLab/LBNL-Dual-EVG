__THIS_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
THIS_DIR := $(__THIS_DIR:/=)

include ../../../dir_list.mk

TARGET       ?= devg_test_marble
GW_DEVG_TGT_DIR = $(GW_SYN_DIR)/$(TARGET)
GIT_COMMIT   = $(shell git rev-parse --short=8 --verify HEAD || echo "NOCOMMIT")
PROC_0_ELF   = $(THIS_DIR)/proc_0/$(TARGET)_proc_0.elf
PROC_1_ELF   = $(THIS_DIR)/proc_1/$(TARGET)_proc_1.elf

BIT          ?= $(GW_DEVG_TGT_DIR)/$(TARGET)_top.bit
DEPLOY_BIT   ?= $(THIS_DIR)/$(TARGET).$(GIT_COMMIT).bit
MMI          ?= $(GW_DEVG_TGT_DIR)/$(TARGET)_top.mmi

.PHONY: all sw

all: sw $(DEPLOY_BIT)

sw:
	make -C proc_0 TARGET=$(TARGET) BIT=$(BIT) all
	make -C proc_1 TARGET=$(TARGET) BIT=$(BIT) all

$(DEPLOY_BIT): $(MMI) $(BIT) $(PROC_0_ELF) $(PROC_1_ELF)
	rm -f updatemem.jou updatemem.log
	updatemem -force -meminfo $< -bit $(BIT) \
		-data $(PROC_0_ELF) -proc bd_i/microblaze_0 \
		-data $(PROC_1_ELF) -proc bd_i/microblaze_1 \
		-out $@

clean:
	make -C proc_0 TARGET=$(TARGET) clean
	make -C proc_1 TARGET=$(TARGET) clean
	rm -f *.log *.jou
