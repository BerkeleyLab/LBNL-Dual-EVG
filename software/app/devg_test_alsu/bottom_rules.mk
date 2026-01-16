CROSS_COMPILE ?= mb-
CC = $(CROSS_COMPILE)gcc
SIZE = $(CROSS_COMPILE)size
XSCT ?= xsct -norlwrap

%softwareBuildDate.h: $(SRC_FILES) $(HDR_FILES)
	sh $(SW_DEVG_SCRIPTS_DIR)/setSoftwareBuildDate.sh > $@

# FIXME: We are considering BSP done if the directory exists. This
# could lead to mismatches!
$(TARGET)_$(PROC_NAME):
	$(XSCT) $(SW_SCRIPTS_DIR)/gen_vitis_platform.tcl $@ $(GW_DEVG_TGT_DIR)/bd.xsa $(CPU_NAME)

$(TARGET)_$(PROC_NAME).elf: $(HDR_FILES) $(HDR_GEN_FILES) $(OBJ_FILES) $(LINKER_FILES)
	$(CC) $(CFLAGS) $(USER_FLAGS) $(INCLUDE_FLAGS) $(LIB_FLAGS) $(LD_FLAGS) -o $@ $(filter %.o, $^) $(LIBS)

$(TARGET)_$(PROC_NAME).elf.size: $(TARGET)_$(PROC_NAME).elf
	$(SIZE) $< | tee $@

# Pull in dependency info for *existing* .o files and don't complain if the
# corresponding .d file is not found
-include $(OBJ_FILES:.o=.d)

%.o: %.c
	$(CC) -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" $(CFLAGS) $(USER_FLAGS) $(INCLUDE_FLAGS) -c $*.c -o $@

clean::
	$(RM) -rf $(TARGET)_$(PROC_NAME).elf $(TARGET)_$(PROC_NAME).elf.size $(HDR_GEN_FILES) \
		$(OBJ_FILES) $(OBJ_FILES:.o=.d) $(TARGET)_$(PROC_NAME)
