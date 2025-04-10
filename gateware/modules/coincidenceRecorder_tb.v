`timescale 1ns/1ps

module coincidenceRecorder_tb();

parameter CHANNEL_COUNT           = 2;
parameter RF1_CLK_PER_COINCIDENCE = 399;
parameter RF2_CLK_PER_COINCIDENCE = 400;
parameter CYCLES_PER_ACQUISITION  = 7;

parameter real RF1_CLK_TO_C1_DELAY = 0.000;
parameter real RF2_CLK_TO_C1_DELAY = 0.000;

parameter real RF1_CLK_TO_C2_DELAY = 0.000;
parameter real RF2_CLK_TO_C2_DELAY = 0.000;

localparam DATA_WIDTH = $clog2(CYCLES_PER_ACQUISITION+1);

reg         sysClk = 1;
reg         sysCsr1strobe = 0, sysCsr2strobe;
reg  [31:0] sysGPIO_OUT;
wire [31:0] sysCsr1, sysCsr2;
wire        rf1heartbeat, rf2heartbeat;
wire        sysRealignToggle;

reg rf1clk_p=1, rf1clk=0, rf1clk_d=0, rf2clk_p=0, rf2clk=0, rf2clk_d=0;
reg rf1clk_c1=0, rf2clk_c1=0;
reg rf1clk_c2=0, rf2clk_c2=0;

//
// Instantiate devices under test
//
coincidenceRecorder #(
    .CHANNEL_COUNT(CHANNEL_COUNT),
    .CYCLES_PER_ACQUISITION(CYCLES_PER_ACQUISITION),
    .SAMPLE_CLKS_PER_COINCIDENCE(RF2_CLK_PER_COINCIDENCE),
    .INPUT_CYCLES_PER_COINCIDENCE(RF1_CLK_PER_COINCIDENCE),
    .TX_CLK_PER_HEARTBEAT(3*RF1_CLK_PER_COINCIDENCE))
  coincidenceRecorder1 (
    .sysClk(sysClk),
    .sysCsrStrobe(sysCsr1strobe),
    .sysGPIO_OUT(sysGPIO_OUT),
    .sysCsr(sysCsr1),
    .sysRealignToggle(sysRealignToggle),
    .sysRealignToggleIn(sysRealignToggle),
    .samplingClk(rf2clk_c1),
    .refClk({rf1clk_d, rf1clk_c1}),
    .txClk(rf1clk_c1),
    .txHeartbeatStrobe(rf1heartbeat));


coincidenceRecorder #(
    .CHANNEL_COUNT(CHANNEL_COUNT),
    .CYCLES_PER_ACQUISITION(CYCLES_PER_ACQUISITION),
    .SAMPLE_CLKS_PER_COINCIDENCE(RF1_CLK_PER_COINCIDENCE),
    .INPUT_CYCLES_PER_COINCIDENCE(RF2_CLK_PER_COINCIDENCE),
    .TX_CLK_PER_HEARTBEAT(3*RF2_CLK_PER_COINCIDENCE))
  coincidenceRecorder2 (
    .sysClk(sysClk),
    .sysCsrStrobe(sysCsr2strobe),
    .sysGPIO_OUT(sysGPIO_OUT),
    .sysCsr(sysCsr2),
    .sysRealignToggleIn(sysRealignToggle),
    .samplingClk(rf1clk_c2),
    .refClk({rf2clk_d, rf2clk_c2}),
    .txClk(rf2clk_c2),
    .txHeartbeatStrobe(rf2heartbeat));

//
// Generate clocks
//
always begin
    #5 sysClk = !sysClk;
end

always begin
    #4 rf1clk_p = !rf1clk_p;
end
always @(rf1clk_p) begin
    #1.1 rf1clk = rf1clk_p;
end
always @(rf1clk) begin
    #1.7 rf1clk_d = rf1clk;
end

always begin
    #3.99 rf2clk_p = !rf2clk_p;
end
always @(rf2clk_p) begin
    #2.302 rf2clk = rf2clk_p;
end
always @(rf2clk) begin
    #1.6 rf2clk_d = rf2clk;
end

always @(rf1clk) begin
    #RF1_CLK_TO_C1_DELAY rf1clk_c1 = rf1clk;
end
always @(rf2clk) begin
    #RF2_CLK_TO_C1_DELAY rf2clk_c1 = rf2clk;
end
always @(rf1clk) begin
    #RF2_CLK_TO_C2_DELAY rf1clk_c2 = rf1clk;
end
always @(rf2clk) begin
    #RF2_CLK_TO_C2_DELAY rf2clk_c2 = rf2clk;
end

//
// Measure alignment
//
realtime timeAtClockEdge[2:1], timeAtHeartbeat[2:1], diff[2:1];
reg [2:1] isAligned = 0, haveHeartbeat = 0;

always @(posedge rf1clk) begin
    timeAtClockEdge[1] = $realtime;
    if (haveHeartbeat[2]) begin
        haveHeartbeat[2] = 0;
        diff[2] = timeAtClockEdge[1] - timeAtHeartbeat[2];
        isAligned[2] = (diff[2] < 0.03);
    end
end
always @(posedge rf2heartbeat) begin
    timeAtHeartbeat[2] = $realtime;
    diff[2] = timeAtHeartbeat[2] - timeAtClockEdge[1];
    if (diff[2] < 0.05) begin
        isAligned[2] = 1;
        haveHeartbeat[2] = 0;
    end
    else begin
        haveHeartbeat[2] = 1;
    end
end

always @(posedge rf2clk) begin
    timeAtClockEdge[2] = $realtime;
    if (haveHeartbeat[1]) begin
        haveHeartbeat[1] = 0;
        diff[1] = timeAtClockEdge[2] - timeAtHeartbeat[1];
        isAligned[1] = (diff[1] < 0.03);
    end
end
always @(posedge rf1heartbeat) begin
    timeAtHeartbeat[1] = $realtime;
    diff[1] = timeAtHeartbeat[1] - timeAtClockEdge[2];
    if (diff[1] < 0.05) begin
        isAligned[1] = 1;
        haveHeartbeat[1] = 0;
    end
    else begin
        haveHeartbeat[1] = 2;
    end
end

//
// Test harness
//

integer hbCount = 0;
always @(posedge rf1heartbeat) begin
    hbCount <= hbCount + 1;
end

integer e;
reg good = 1;
initial
begin
    $dumpfile("coincidenceRecorder.vcd");
    $dumpvars(5, coincidenceRecorder_tb);
    # 50;

    acquire();
    #10000;
    for (e = 1 ; e <= 2 ; e = e + 1) begin
        if (isAligned[e]) begin
            $display("EVG %d aligned at start -- FAIL", e);
            good = 0;
        end
    end
    align1();
    align2();
    //plotData1();
    //plotData2();
    #10000;
    writeCsr1({1'b0, 1'b0, 1'b1, 29'h0});
    writeCsr2({1'b0, 1'b0, 1'b1, 29'h0});
    hbCount = 0;
    while (hbCount < 4) begin
        #10;
    end
    for (e = 1 ; e <= 2 ; e = e + 1) begin
        if (!isAligned[e]) begin
            $display("EVG %d not aligned -- FAIL", e);
            good = 0;
        end
    end
    $display("DIFF 1: %g      DIFF 2: %g", diff[1], diff[2]);

	if (!good) begin
		$display("FAIL");
		$stop(0);
	end else begin
		$display("PASS");
		$finish(0);
	end
end

task acquire;
    begin
    # 50;
    writeCsr1({1'b1, {31{1'b0}}});
    writeCsr2({1'b1, {31{1'b0}}});
    # 50;
    while (sysCsr1[31]) #10;
    while (sysCsr2[31]) #10;
    end
endtask

task writeCsr1;
    input [31:0] value;
    begin
    @(posedge sysClk) begin
        sysGPIO_OUT <= value;
        sysCsr1strobe <= 1;
    end
    @(posedge sysClk) begin
        sysGPIO_OUT <= {32{1'bx}};
        sysCsr1strobe <= 0;
    end
    end
endtask

task plotData1;
    reg [23:0] a;
    reg [7:0] c;
    begin
    for (a = 0 ; a < RF2_CLK_PER_COINCIDENCE ; a = a + 1) begin
        $write("%d ", a);
        for (c = 0 ; c < CHANNEL_COUNT ; c = c + 1) begin
            writeCsr1({c, a});
            #160;
            $write("%d ", sysCsr1[DATA_WIDTH-1:0]);
        end
        $display("");
    end
    end
endtask

task align1;
    reg [23:0] a, risingEdge;
    reg oldValue, newValue;
    begin
    oldValue = 1'bx;
    risingEdge = 0;
    for (a = 0 ; a < RF2_CLK_PER_COINCIDENCE ; a = a + 1) begin
        writeCsr1({8'h0, a});
        #160;
        newValue = (sysCsr1[DATA_WIDTH-1:0] != 0);
        if (newValue && !oldValue) begin
            risingEdge = a;
        end
        oldValue = newValue;
    end
    $display("EVG 1 Rising edge at %d", risingEdge);
    risingEdge = (risingEdge - 5 + RF2_CLK_PER_COINCIDENCE) % RF2_CLK_PER_COINCIDENCE;
    writeCsr1({1'b0, 1'b1, 1'b0, 5'h0, risingEdge});
    end
endtask

task writeCsr2;
    input [31:0] value;
    begin
    @(posedge sysClk) begin
        sysGPIO_OUT <= value;
        sysCsr2strobe <= 1;
    end
    @(posedge sysClk) begin
        sysGPIO_OUT <= {32{1'bx}};
        sysCsr2strobe <= 0;
    end
    end
endtask

task plotData2;
    reg [23:0] a;
    reg [7:0] c;
    begin
    for (a = 0 ; a < RF1_CLK_PER_COINCIDENCE ; a = a + 1) begin
        $write("%d ", a);
        for (c = 0 ; c < CHANNEL_COUNT ; c = c + 1) begin
            writeCsr2({c, a});
            #160;
            $write("%d ", sysCsr2[DATA_WIDTH-1:0]);
        end
        $display("");
    end
    end
endtask

task align2;
    reg [23:0] a, risingEdge;
    reg oldValue, newValue;
    begin
    oldValue = 1'bx;
    risingEdge = 0;
    for (a = 0 ; a < RF1_CLK_PER_COINCIDENCE ; a = a + 1) begin
        writeCsr2({8'h0, a});
        #160;
        newValue = (sysCsr2[DATA_WIDTH-1:0] != 0);
        if (newValue && !oldValue) begin
            risingEdge = a;
        end
        oldValue = newValue;
    end
    $display("EVG 2 Rising edge at %d", risingEdge);
    risingEdge = (risingEdge - 5 + RF1_CLK_PER_COINCIDENCE) % RF1_CLK_PER_COINCIDENCE;
    writeCsr2({1'b0, 1'b1, 1'b0, 5'h0, risingEdge});
    end
endtask

endmodule
