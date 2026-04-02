VFLAGS_DEP += -I. -y.
VFLAGS += -I. -y.

VERILOG += -g2012 -Y .sv

TEST_BENCH = \
	debounceFallingEdge_tb \
	clkIntervalCounters_tb \
	coincidenceRecorder_tb \
	coincidenceRecorder2_tb \
	injectorSequenceControl_tb \
	mod125_reduction_tb

TGT_ := $(TEST_BENCH)
NO_CHECK = coincidenceRecorder2_check
CHK_ = $(filter-out $(NO_CHECK), $(TEST_BENCH:%_tb=%_check))

.PHONY: targets checks
targets: $(TGT_)
checks: $(CHK_)

coincidenceRecorder2_tb: coincidenceRecorder.v
mod125_reduction_tb: mod125_reduction_test_wrapper.sv

CLEAN += $(TGT_) *_tb *.pyc *.bit *.in *.vcd *.fst *~
CLEAN_DIRS += _xilinx __pycache__

ifneq (,$(findstring bit,$(MAKECMDGOALS)))
    ifneq (,$(findstring bits,$(MAKECMDGOALS)))
	-include $(BITS_:%.bit=$(DEPDIR)/%.bit.d)
    else
	-include $(MAKECMDGOALS:%.bit=$(DEPDIR)/%.bit.d)
    endif
endif
ifneq (,$(findstring _tb,$(MAKECMDGOALS)))
    -include $(MAKECMDGOALS:%_tb=$(DEPDIR)/%_tb.d)
endif
ifneq (,$(findstring _view,$(MAKECMDGOALS)))
    -include $(MAKECMDGOALS:%_tb=$(DEPDIR)/%_tb.d)
endif
ifneq (,$(findstring _check,$(MAKECMDGOALS)))
    -include $(MAKECMDGOALS:%_tb=$(DEPDIR)/%_tb.d)
endif
ifeq (,$(MAKECMDGOALS))
    -include $(TEST_BENCH:%_tb=$(DEPDIR)/%_tb.d)
endif
