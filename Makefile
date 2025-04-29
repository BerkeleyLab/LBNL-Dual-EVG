include dir_list.mk

CROSS_COMPILE    ?=
PLATFORM         ?= marble
APP              ?= devg
EVIO_TYPE        ?= evio

TARGET       = $(APP)_$(PLATFORM)
GW_TGT_DIR   = $(GW_SYN_DIR)/$(TARGET)
BIT          = $(GW_TGT_DIR)/$(TARGET)_top.bit
SW_TGT_DIR   = $(SW_APP_DIR)/$(APP)

.PHONY: all bit sw

all: bit sw

bit:
	make -C $(GW_TGT_DIR) TARGET=$(TARGET) EVIO_TYPE=$(EVIO_TYPE) $(TARGET)_top.bit
	make -C $(GW_TGT_DIR) TARGET=$(TARGET) EVIO_TYPE=$(EVIO_TYPE) $(TARGET)_top.mmi

sw:
	make -C $(SW_TGT_DIR) TARGET=$(TARGET) BIT=$(BIT) all

clean:
	make -C $(GW_TGT_DIR) TARGET=$(TARGET) clean
	make -C $(SW_TGT_DIR) TARGET=$(TARGET) clean
	rm -f *.log *.jou
