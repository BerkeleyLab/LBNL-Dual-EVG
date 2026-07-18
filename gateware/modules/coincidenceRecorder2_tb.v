`timescale 1ns/1ps

module coincidenceRecorder2_tb();

parameter CHANNEL_COUNT           = 2;
parameter RF1_CLK_PER_COINCIDENCE = 23;
parameter RF2_CLK_PER_COINCIDENCE = 24;
//parameter RF1_CLK_PER_COINCIDENCE = 668;
//parameter RF2_CLK_PER_COINCIDENCE = 669;
parameter CYCLES_PER_ACQUISITION  = 7;

//parameter real RF1_CLK_LOW_PERIOD  = 1.003;
//parameter real RF1_CLK_HIGH_PERIOD = 1.004;
parameter real RF1_CLK_LOW_PERIOD  = 7.992;
parameter real RF1_CLK_HIGH_PERIOD = 7.992;
parameter real RF1_CLK_PERIOD      = RF1_CLK_LOW_PERIOD + RF1_CLK_HIGH_PERIOD;

parameter real RF2_CLK_PERIOD      = (RF1_CLK_PERIOD * RF1_CLK_PER_COINCIDENCE) / RF2_CLK_PER_COINCIDENCE;
parameter real RF2_CLK_LOW_PERIOD  = RF2_CLK_PERIOD / 2;
parameter real RF2_CLK_HIGH_PERIOD = RF2_CLK_PERIOD / 2;

parameter RF1_CLK_T1_DELAY = 0;
parameter RF2_CLK_T2_DELAY = 0;

parameter real RF1_CLK_TO_C1_DELAY = 0.000;
parameter real RF2_CLK_TO_C1_DELAY = 0.000;

parameter real RF1_CLK_TO_C2_DELAY = 0.000;
parameter real RF2_CLK_TO_C2_DELAY = 0.000;

localparam DATA_WIDTH = $clog2(CYCLES_PER_ACQUISITION+1);

reg         sysClk = 0;
reg         sysCsr1strobe = 0, sysCsr2strobe;
reg  [31:0] sysGPIO_OUT;
wire [31:0] sysCsr1, sysCsr2;
wire        rf1heartbeat, rf2heartbeat;
wire        sysRealignToggle;

reg rf1clk_p=1, rf2clk_p=1;
reg rf1div4clk_p=1, rf2div4clk_p=1;
reg rf1div4clk=1, rf2div4clk=1;

reg rf1clk_coinc=0, rf2clk_coinc=0;

wire rf1div4clk_c1, rf2div4clk_c1;
wire rf1div4clk_c2, rf2div4clk_c2;

wire rf1CoincidenceMarker, rf2CoincidenceMarker;
wire rf1TxCoincidenceMarker, rf2TxCoincidenceMarker;
wire rf1div4clk_d;
wire rf2div4clk_d;

//
// Instantiate devices under test
//
coincidenceRecorder #(
    .CHANNEL_COUNT(CHANNEL_COUNT),
    .CYCLES_PER_ACQUISITION(CYCLES_PER_ACQUISITION),
    .SAMPLE_CLKS_PER_COINCIDENCE(RF2_CLK_PER_COINCIDENCE),
    .INPUT_CYCLES_PER_COINCIDENCE(RF1_CLK_PER_COINCIDENCE),
    .HEARTBEAT_GEN_COUNT(1),
    .TX_CLK_PER_HEARTBEAT(3*RF1_CLK_PER_COINCIDENCE))
  coincidenceRecorder1 (
    .sysClk(sysClk),
    .sysCsrStrobe(sysCsr1strobe),
    .sysGPIO_OUT(sysGPIO_OUT),
    .sysCsr(sysCsr1),
    .sysRealignToggleIn(sysRealignToggle),
    .samplingClk(rf2div4clk_c1),
    .refClk({rf1div4clk_d, rf1div4clk_c1}),
    .coincidenceMarker(rf1CoincidenceMarker),
    .txClk(rf1div4clk_c1),
    .txCoincidenceMarker(rf1TxCoincidenceMarker),
    .txHeartbeatStrobe(rf1heartbeat));


coincidenceRecorder #(
    .CHANNEL_COUNT(CHANNEL_COUNT),
    .CYCLES_PER_ACQUISITION(CYCLES_PER_ACQUISITION),
    .SAMPLE_CLKS_PER_COINCIDENCE(RF1_CLK_PER_COINCIDENCE),
    .INPUT_CYCLES_PER_COINCIDENCE(RF2_CLK_PER_COINCIDENCE),
    .HEARTBEAT_GEN_COUNT(1),
    .TX_CLK_PER_HEARTBEAT(3*RF2_CLK_PER_COINCIDENCE))
  coincidenceRecorder2 (
    .sysClk(sysClk),
    .sysCsrStrobe(sysCsr2strobe),
    .sysGPIO_OUT(sysGPIO_OUT),
    .sysCsr(sysCsr2),
    .sysRealignToggle(sysRealignToggle),
    .sysRealignToggleIn(sysRealignToggle),
    .samplingClk(rf1div4clk_c2),
    .refClk({rf2div4clk_d, rf2div4clk_c2}),
    .coincidenceMarker(rf2CoincidenceMarker),
    .txClk(rf2div4clk_c2),
    .txCoincidenceMarker(rf2TxCoincidenceMarker),
    .txHeartbeatStrobe(rf2heartbeat));

//
// Generate clocks
//
always begin
    #5 sysClk = !sysClk;
end

//
// F1 clock generation
//

always begin
    #(RF1_CLK_LOW_PERIOD) rf1clk_p = 1'b0;
    #(RF1_CLK_HIGH_PERIOD) rf1clk_p = 1'b1;
end

always begin
    #(RF1_CLK_PER_COINCIDENCE*RF1_CLK_PERIOD/2) rf1clk_coinc = 1'b0;
    #(RF1_CLK_PER_COINCIDENCE*RF1_CLK_PERIOD/2) rf1clk_coinc = 1'b1;
end

always begin
    #(4*RF1_CLK_LOW_PERIOD) rf1div4clk_p = 1'b0;
    #(4*RF1_CLK_HIGH_PERIOD) rf1div4clk_p = 1'b1;
end

initial begin
    #(RF1_CLK_T1_DELAY*RF1_CLK_PERIOD);

    forever begin
        #(4*RF1_CLK_LOW_PERIOD) rf1div4clk = 1'b0;
        #(4*RF1_CLK_HIGH_PERIOD) rf1div4clk = 1'b1;
    end

end

assign #0.5 rf1div4clk_d = rf1div4clk;

//
// F2 clock generation
//

always begin
    #(RF2_CLK_LOW_PERIOD) rf2clk_p = 1'b0;
    #(RF2_CLK_HIGH_PERIOD) rf2clk_p = 1'b1;
end

always begin
    #(RF2_CLK_PER_COINCIDENCE*RF2_CLK_PERIOD/2) rf2clk_coinc = 1'b0;
    #(RF2_CLK_PER_COINCIDENCE*RF2_CLK_PERIOD/2) rf2clk_coinc = 1'b1;
end

always begin
    #(4*RF2_CLK_LOW_PERIOD) rf2div4clk_p = 1'b0;
    #(4*RF2_CLK_HIGH_PERIOD) rf2div4clk_p = 1'b1;
end

initial begin
    #(RF2_CLK_T2_DELAY*RF2_CLK_PERIOD);

    forever begin
        #(4*RF2_CLK_LOW_PERIOD) rf2div4clk = 1'b0;
        #(4*RF2_CLK_HIGH_PERIOD) rf2div4clk = 1'b1;
    end

end

assign #0.5 rf2div4clk_d = rf2div4clk;

//
// clock delays
//

assign #RF1_CLK_TO_C1_DELAY rf1div4clk_c1 = rf1div4clk;
assign #RF2_CLK_TO_C1_DELAY rf2div4clk_c1 = rf2div4clk;
assign #RF2_CLK_TO_C2_DELAY rf1div4clk_c2 = rf1div4clk;
assign #RF2_CLK_TO_C2_DELAY rf2div4clk_c2 = rf2div4clk;

//
// Measure alignment
//
realtime timeAtClockEdge[2:1], timeAtHeartbeat[2:1], diff[2:1];
reg [2:1] isAligned = 0, haveHeartbeat = 0;

always @(posedge rf1div4clk) begin
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

always @(posedge rf2div4clk) begin
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
    $dumpfile("coincidenceRecorder2.vcd");
    $dumpvars(5, coincidenceRecorder2_tb);
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
    plotData1();
    plotData2();
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
        $display("# FAIL");
        $stop(0);
    end else begin
        $display("# PASS");
        $finish(0);
    end
end

task acquire;
    begin
    repeat(128) begin
        @(posedge sysClk);
    end
    writeCsr1({1'b1, {31{1'b0}}});
    writeCsr2({1'b1, {31{1'b0}}});
    repeat(128) begin
        @(posedge sysClk);
    end
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
    $display("EVG1:");
    for (a = 0 ; a < RF2_CLK_PER_COINCIDENCE ; a = a + 1) begin
        $write("%d ", a);
        for (c = 0 ; c < CHANNEL_COUNT ; c = c + 1) begin
            writeCsr1({c, a});
            repeat(32) begin
                @(posedge sysClk);
            end
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
        repeat(32) begin
            @(posedge sysClk);
        end
        newValue = (sysCsr1[DATA_WIDTH-1:0] != 0);
        if (newValue && !oldValue) begin
            risingEdge = a;
        end
        oldValue = newValue;
    end
    $display("EVG 1 Rising edge at %d", risingEdge);
    risingEdge = (risingEdge - 4 + RF2_CLK_PER_COINCIDENCE) % RF2_CLK_PER_COINCIDENCE;
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
    $display("EVG2:");
    for (a = 0 ; a < RF1_CLK_PER_COINCIDENCE ; a = a + 1) begin
        $write("%d ", a);
        for (c = 0 ; c < CHANNEL_COUNT ; c = c + 1) begin
            writeCsr2({c, a});
            repeat(32) begin
                @(posedge sysClk);
            end
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
        repeat(32) begin
            @(posedge sysClk);
        end
        newValue = (sysCsr2[DATA_WIDTH-1:0] != 0);
        if (newValue && !oldValue) begin
            risingEdge = a;
        end
        oldValue = newValue;
    end
    $display("EVG 2 Rising edge at %d", risingEdge);
    risingEdge = (risingEdge - 4 + RF1_CLK_PER_COINCIDENCE) % RF1_CLK_PER_COINCIDENCE;
    writeCsr2({1'b0, 1'b1, 1'b0, 5'h0, risingEdge});
    end
endtask

endmodule
