bantamweightUDP_DIR = $(BANTAMWEIGHT_DIR)
__bantamweightUDP_SRCS = \
							badger.v
bantamweightUDP_SRCS = $(addprefix $(bantamweightUDP_DIR)/, $(__bantamweightUDP_SRCS))
bantamweightUDP_TARGET = _gen/bantamweightUDP

IP_CORES_DIRS += $(bantamweightUDP_DIR)

bantamweightUDP: $(bantamweightUDP_SRCS)
	cp $(bantamweightUDP_SRCS $($@_TARGET)
	touch $@

CLEAN_DIRS += $(bantamweightUDP_TARGET)
