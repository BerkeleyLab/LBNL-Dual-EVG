TEST_SOURCE = st7789v_tb.v st7789v.v

all: st7789v_tb.vvp

st7789v_tb.vvp: $(TEST_SOURCE)
	iverilog -Wall -o st7789v_tb.vvp $(TEST_SOURCE)

test: st7789v_tb.vvp
	vvp -n st7789v_tb.vvp -lxt2 $(VVP_PLUSARGS)

st7789v_tb.lxt: st7789v_tb.vvp
	vvp st7789v_tb.vvp -lxt2

view: st7789v_tb.lxt force
	gtkwave st7789v_tb.gtkw || open /Applications/gtkwave.app st7789v_tb.gtkw

force:

clean:
	rm -rf st7789v_tb.vvp st7789v_tb.lxt a.out
