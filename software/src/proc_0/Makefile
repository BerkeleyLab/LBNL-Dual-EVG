all: softwareBuildDate.h
softwareBuildDate.h: bootFlash.c \
                     bootFlash.h \
                     bwudp.h \
                     bwudp_config.h \
                     config.h \
                     console.c \
                     console.h \
					 drawEventLog.c \
					 drawEventLog.h \
                     display.c \
                     display.h \
                     evgCoincidence.c \
                     evgCoincidence.h \
                     evio.c \
                     evio.h \
                     eyescan.c \
                     eyescan.h \
                     gpio.h \
                     iicChunk.c \
                     iicChunk.h \
                     iicFMC.c \
                     iicFMC.h \
                     iicProc.c \
                     iicProc.h \
                     lv_font.h \
                     main.c \
                     mgtClkSwitch.c \
                     mgtClkSwitch.h \
                     mmcMailbox.c \
                     mmcMailbox.h \
                     platform.c \
                     platform.h \
                     platform_config.h \
                     spiflash.h \
                     spiflash.inc \
                     st7789v.c \
                     st7789v.h \
                     sysmon.c \
                     sysmon.h \
                     systemFont.c \
                     systemParameters.c \
                     systemParameters.h \
                     tftp.c \
                     tftp.h \
                     util.c \
                     util.h \
                     xadc.c \
                     xadc.h \
                     ../../Processor1/src/NMEA.c \
                     ../../Processor1/src/NMEA.h \
                     ../../Processor1/src/badger.c \
                     ../../Processor1/src/bwudp.c \
                     ../../Processor1/src/bwudp.h \
                     ../../Processor1/src/bwudp_config.h \
                     ../../Processor1/src/config.h \
                     ../../Processor1/src/epics.c \
                     ../../Processor1/src/epics.h \
                     ../../Processor1/src/evg.c \
                     ../../Processor1/src/evg.h \
                     ../../Processor1/src/evgProtocol.h \
                     ../../Processor1/src/fmcIO.c \
                     ../../Processor1/src/fmcIO.h \
                     ../../Processor1/src/gpio.h \
                     ../../Processor1/src/gps.c \
                     ../../Processor1/src/gps.h \
                     ../../Processor1/src/injectionCycle.c \
                     ../../Processor1/src/injectionCycle.h \
                     ../../Processor1/src/ipc.c \
                     ../../Processor1/src/ipc.h \
                     ../../Processor1/src/main.c \
                     ../../Processor1/src/mgt.c \
                     ../../Processor1/src/mgt.h \
                     ../../Processor1/src/platform.c \
                     ../../Processor1/src/platform.h \
                     ../../Processor1/src/platform_config.h \
                     ../../Processor1/src/sharedMemory.h \
                     ../../Processor1/src/swapoutCycle.c \
                     ../../Processor1/src/swapoutCycle.h \
                     ../../Processor1/src/systemParameters.h \
                     ../../Processor1/src/tod.c \
                     ../../Processor1/src/tod.h \
                     ../../Processor1/src/util.c \
                     ../../Processor1/src/util.h
	sh setSoftwareBuildDate.sh

# Imported stuff
BWUDP_SOURCE = $(HOME)/src/Firmware/bantamweightUDP
SPIFLASH_SOURCE = $(HOME)/src/Firmware/spiflash_driver/src
ST7789V_SOURCE = $(HOME)/src/ip_repo/ST7789V

bwudp.h: $(BWUDP_SOURCE)/bwudp.h
	cp $< $@

spiflash.h: $(SPIFLASH_SOURCE)/spiflash.h
	cp $< $@
spiflash.inc: $(SPIFLASH_SOURCE)/spiflash.c
	cp $< $@

lv_font.h: $(ST7789V_SOURCE)/lv_font.h
	cp $< $@
systemFont.c: $(ST7789V_SOURCE)/systemFont.c
	cp $< $@
st7789v.h: $(ST7789V_SOURCE)/st7789v.h
	cp $< $@
st7789v.c: $(ST7789V_SOURCE)/st7789v.c
	cp $< $@

clean:
	rm -f xrc.log vitis_pid*.str
