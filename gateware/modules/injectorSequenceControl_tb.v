`timescale 1ns / 1ps

module injectorSequenceControl_tb;

// =========================================================================
// Timing relationships, scaled down for faster simulation
// =========================================================================
// Base periods

// Number of BR/AR alignment periods per BR/AR coincidence
localparam SIM_BR_AR_ALIGN_PER_BR_AR_COINC   = 5;
// Number of RF coincidences per BR/AR coincidence
localparam SIM_RF_COINC_PER_BR_AR_COINC      = 125;

// Free running counters must divide evenly into heartbeat
localparam SIM_CLK_PER_BR_ORBIT_CLOCK_DIV4   = 5;
localparam SIM_CLK_PER_TICKS_COINCIDENCE     = 167;
localparam SIM_CLK_PER_BR_AR_ALIGNMENT       = 125 * 167;
localparam SIM_CLK_PER_BR_AR_COINCIDENCE     = 125 * 167 * 5;

localparam SIM_ALT_CLK_PER_BR_AR_ALIGNMENT   = 250 * 41;

// LCM of all periods
localparam SIM_CLK_PER_HEARTBEAT             = 125 * 167 * 5 * 2;
localparam SIM_ALT_CLK_PER_HEARTBEAT         = 250 * 41 * 5 * 7;

localparam ALIGNMENT_SYNC_COUNT                   = 2;
localparam GPIO_IDX_COUNT                         = 128;
localparam GPIO_IDX_INJECTION_CYCLE_CSR           = 29;
localparam GPIO_IDX_INJECTION_ALIGN_CSR           = 30;
localparam GPIO_IDX_INJECTION_TARGET_CSR          = 31;

// =========================================================================
// Clock generation
// =========================================================================
reg sysClk = 0;
reg evgTxClk = 0;

always begin
    #5 sysClk = !sysClk;
end

always begin
    #4 evgTxClk = !evgTxClk;
end

// 100 MHz
localparam SYSCLK_RATE = 100000000;

localparam RF_COINC_IDX_WIDTH          = $clog2(SIM_RF_COINC_PER_BR_AR_COINC + 1);
localparam RF_ALIGN_IDX_WIDTH          = $clog2(SIM_BR_AR_ALIGN_PER_BR_AR_COINC + 1);
localparam CLK_PER_TICKS_COINC_WIDTH   = $clog2(SIM_CLK_PER_TICKS_COINCIDENCE + 1);
localparam CLK_PER_BR_AR_COINC_WIDTH   = $clog2(SIM_CLK_PER_BR_AR_COINCIDENCE + 1);
localparam CLK_PER_BR_ORBIT_DIV4_WIDTH = $clog2(SIM_CLK_PER_BR_ORBIT_CLOCK_DIV4 + 1);
localparam CLK_PER_BR_AR_ALIGN_WIDTH   = $clog2(SIM_CLK_PER_BR_AR_ALIGNMENT + 1);

///////////////////////////////////////////////////////////////////////////////
// CSR Master

localparam CSR_DATA_BUS_WIDTH = 32;
localparam CSR_STROBE_BUS_WIDTH = GPIO_IDX_COUNT;

csrTestMaster #(
    .CSR_DATA_BUS_WIDTH(CSR_DATA_BUS_WIDTH),
    .CSR_STROBE_BUS_WIDTH(CSR_STROBE_BUS_WIDTH)
) CSR0 (
    .clk(sysClk)
);

wire [CSR_DATA_BUS_WIDTH-1:0] GPIO_IN[0:CSR_STROBE_BUS_WIDTH-1];
wire [CSR_STROBE_BUS_WIDTH-1:0] GPIO_STROBES = CSR0.csr_stb_o;
wire [CSR_DATA_BUS_WIDTH-1:0] GPIO_OUT = CSR0.csr_data_o;

genvar i;
generate for(i = 0; i < CSR_STROBE_BUS_WIDTH; i = i + 1) begin
    assign CSR0.csr_data_i[(i+1)*CSR_DATA_BUS_WIDTH-1:i*CSR_DATA_BUS_WIDTH] = GPIO_IN[i];
end
endgenerate

///////////////////////////////////////////////////////////////////////////////
// DUT Interconnects

wire [31:0] sysStatus;
wire [31:0] sysAlignStatus;
wire [31:0] sysTargetStatus;
reg         powerlineA = 0;

wire [ALIGNMENT_SYNC_COUNT-1:0] evgHeartbeat;
wire [ALIGNMENT_SYNC_COUNT-1:0] evgAlignCounterDone;

wire evgHeartbeatAlign;
wire evgHeartbeatCore;
wire evgSequenceStart;

// Route DUT readback statuses into the CSR master's input data bus array
assign GPIO_IN[GPIO_IDX_INJECTION_CYCLE_CSR]  = sysStatus;
assign GPIO_IN[GPIO_IDX_INJECTION_ALIGN_CSR]  = sysAlignStatus;
assign GPIO_IN[GPIO_IDX_INJECTION_TARGET_CSR] = sysTargetStatus;

// Default unused indices to zero to prevent 'z' propagation
generate for(i = 0; i < CSR_STROBE_BUS_WIDTH; i = i + 1) begin
    if (i != GPIO_IDX_INJECTION_CYCLE_CSR &&
        i != GPIO_IDX_INJECTION_ALIGN_CSR &&
        i != GPIO_IDX_INJECTION_TARGET_CSR) begin
        assign GPIO_IN[i] = 32'd0;
    end
end
endgenerate

///////////////////////////////////////////////////////////////////////////////
// 60 Hz Powerline Signal Generation (Accelerated for sim)

integer powerlineCounter = 0;
always @(posedge sysClk) begin
    if (powerlineCounter >= 15000) begin // 150us half-period to pass 100us debounce fast
        powerlineA <= ~powerlineA;
        powerlineCounter <= 0;
    end else begin
        powerlineCounter <= powerlineCounter + 1;
    end
end

///////////////////////////////////////////////////////////////////////////////
// Heartbeat Generation via heartbeatGenerator

wire evgHeartbeatRequest;

heartbeatGenerator #(
    .TX_CLK_PER_HEARTBEAT(SIM_CLK_PER_HEARTBEAT)
) heartbeatGen (
    .sampCoincidenceMarker(1'b0),
    .sysRealignToggleIn(1'b0),
    .txClk(evgTxClk),
    .txCoincidenceMarker(),
    .txHeartbeatStrobe(evgHeartbeatRequest)
);

assign evgHeartbeat = {ALIGNMENT_SYNC_COUNT{evgHeartbeatRequest}};

///////////////////////////////////////////////////////////////////////////////
// evgCounters
localparam NUM_EVG_COUNTERS = 6;
wire [NUM_EVG_COUNTERS*32-1:0] clkGenCounters;
wire [NUM_EVG_COUNTERS-1:0]    clkGenStrobes;
wire [NUM_EVG_COUNTERS-1:0]    clkGenSynceds;

// Extract the counters we need to feed to injectorSequenceControl
wire [31:0] rfCoincPerArBrAlignCounter   = clkGenCounters[191:160]; // Index 5
wire [31:0] brArAlignPerBrArCoincCounter = clkGenCounters[159:128]; // Index 4

// Extract the specific strobes needed to drive the cascaded 'ens' enables
wire rfF1CoincStrobe = clkGenStrobes[3];
wire brArAlignStrobe = clkGenStrobes[0];

// Cascaded enables: Index 5 and 4 are enabled by the lower-level counter strobes
wire [NUM_EVG_COUNTERS-1:0] evgCountersEn = {rfF1CoincStrobe, brArAlignStrobe, 4'b1111};

evgCounters #(
    .SYSCLK_FREQUENCY(SYSCLK_RATE),
    .DEBUG("false"),
    .NUM_COUNTERS(NUM_EVG_COUNTERS),
    .COUNTER_WIDTHS({
        RF_COINC_IDX_WIDTH[31:0],
        RF_ALIGN_IDX_WIDTH[31:0],
        CLK_PER_TICKS_COINC_WIDTH[31:0],
        CLK_PER_BR_AR_COINC_WIDTH[31:0],
        CLK_PER_BR_ORBIT_DIV4_WIDTH[31:0],
        CLK_PER_BR_AR_ALIGN_WIDTH[31:0]
    }),
    .DEFAULT_RATE_COUNTS({
        SIM_RF_COINC_PER_BR_AR_COINC[31:0],
        SIM_BR_AR_ALIGN_PER_BR_AR_COINC[31:0],
        SIM_CLK_PER_TICKS_COINCIDENCE[31:0],
        SIM_CLK_PER_BR_AR_COINCIDENCE[31:0],
        SIM_CLK_PER_BR_ORBIT_CLOCK_DIV4[31:0],
        SIM_CLK_PER_BR_AR_ALIGNMENT[31:0]
    })
) evgCountersInst (
    .sysClk(sysClk),
    .GPIO_OUT(32'b0),
    .csrStrobes({NUM_EVG_COUNTERS{1'b0}}),
    .csrs(),
    .clk(evgTxClk),
    .ens(evgCountersEn),
    .heartbeatStrobe(evgHeartbeatAlign),
    .pulsePerSecondStrobe(1'b0),
    .clkGenSynceds(clkGenSynceds),
    .clkGens(),
    .clkGenStrobes(clkGenStrobes),
    .clkGenCounters(clkGenCounters)
);

///////////////////////////////////////////////////////////////////////////////
// DUT

injectorSequenceControl #(
    .SYSCLK_RATE(SYSCLK_RATE),
    .ALIGNMENT_SYNC_COUNT(ALIGNMENT_SYNC_COUNT),
    .RF_COINC_IDX_WIDTH(RF_COINC_IDX_WIDTH),
    .RF_ALIGN_IDX_WIDTH(RF_ALIGN_IDX_WIDTH),
    .RF_COINC_TERM_WIDTH(RF_ALIGN_IDX_WIDTH),
    .TX_CLK_PER_ALIGNMENT({
        SIM_ALT_CLK_PER_BR_AR_ALIGNMENT[31:0],
        SIM_CLK_PER_BR_AR_ALIGNMENT[31:0]
    })
) dut (
    .sysClk(sysClk),
    .sysGPIO_OUT(GPIO_OUT),
    .sysCsrStrobe(GPIO_STROBES[GPIO_IDX_INJECTION_CYCLE_CSR]),
    .sysCsrAlignStrobe(GPIO_STROBES[GPIO_IDX_INJECTION_ALIGN_CSR]),
    .sysCsrTargetStrobe(GPIO_STROBES[GPIO_IDX_INJECTION_TARGET_CSR]),

    .sysStatus(sysStatus),
    .sysAlignStatus(sysAlignStatus),
    .sysTargetStatus(sysTargetStatus),
    .powerline_a(powerlineA),
    .evgTxClk(evgTxClk),

    .evgRFCoincCount(rfCoincPerArBrAlignCounter[RF_COINC_IDX_WIDTH-1:0]),
    .evgRFAlignCount(brArAlignPerBrArCoincCounter[RF_ALIGN_IDX_WIDTH-1:0]),

    .evgAlignCounterDone(evgAlignCounterDone),
    .evgHeartbeat(evgHeartbeat),
    .evgHeartbeatAlign(evgHeartbeatAlign),
    .evgHeartbeatCore(evgHeartbeatCore),
    .evgSequenceStart(evgSequenceStart)
);

///////////////////////////////////////////////////////////////////////////////
// Synchronization Check
integer errors = 0;
reg clkSynced = 0;

initial begin
    // Wait for two heartbeat alignments to establish initial sync
    @(posedge evgHeartbeatAlign);
    $display("Heartbeat #1 detected");
    @(posedge evgHeartbeatAlign);
    $display("Heartbeat #2 detected");

    @(posedge evgTxClk);

    wait(clkGenSynceds == {NUM_EVG_COUNTERS{1'b1}});
    $display("Clk generation is synced");
    clkSynced = 1;
    @(posedge evgTxClk);

    // Continuously check that the clkGenSynceds are properly aligned
    forever begin
        @(posedge evgTxClk);
        if (clkGenSynceds != {NUM_EVG_COUNTERS{1'b1}}) begin
            $display("@%0t: Error: Counters lost synchronization! clkGenSynceds = %b", $time, clkGenSynceds);
            errors = errors + 1;
        end
    end
end

initial begin
    if ($test$plusargs("vcd")) begin
        $dumpfile("injectorSequenceControl.vcd");
        $dumpvars(5, injectorSequenceControl_tb);
    end

    // Wait for CSR module to assert its readiness
    $display("Waiting for CSR0.ready ...");
    wait(CSR0.ready);
    @(posedge sysClk);

    // Wait for Clockes to be synced
    $display("Waiting for clkSynced ...");
    wait(clkSynced);
    @(posedge evgTxClk);

    // 1. Program the Target CSR
    // bBR= (rfCOINCTERM + 72.iAR,BR(mod125))(mod 125)
    //
    // rfCOINCTERM = (43.rfCOINC)(mod hBR)
    // rfCOINC = (5.bAR)(mod hAR)
    //
    // For this simualation, hAR = 125, hBR = 5
    //
    // rfCOINCTERM = (43.rfCOINC)(mod 5)
    // rfCOINC = (5.bAR)(mod 125)
    //
    // Consider bAR = 32, rfCOINC = 35, rfCOINCTERM = 0
    CSR0.write32(GPIO_IDX_INJECTION_TARGET_CSR, (32'd0 << 16) | 32'd35);
    @(posedge sysClk);

    // 2. Program the Alignment CSR (Select heartbeat & counter indices)
    CSR0.write32(GPIO_IDX_INJECTION_ALIGN_CSR, {1'b1, 31'd0});       // MSB=1 routes to alignCounterSel = 0
    @(posedge sysClk);
    CSR0.write32(GPIO_IDX_INJECTION_ALIGN_CSR, {1'b0, 1'b1, 30'd0}); // Bit 30=1 routes to evgHeartbeatSel = 0
    @(posedge sysClk);

    // 3. Manually trigger injection (sysGPIO_OUT[7] = 1, sysCycleEnabled = 0)
    CSR0.write32(GPIO_IDX_INJECTION_CYCLE_CSR, 32'h0000_0080);
    @(posedge sysClk);

    // 4. Wait for the sequence start flag
    begin : wait_for_sequence
        fork
            begin
                @(posedge evgSequenceStart);
                disable wait_for_sequence;
            end
            begin
                // With 20,875 cycles per heartbeat (approx 167us), the sequence should trigger very quickly.
                // 2,500,000 ns = 2.5 ms timeout
                #2500000;
                $display("Timeout waiting for evgSequenceStart assertion");
                errors = errors + 1;
                disable wait_for_sequence;
            end
        join
    end

    if (errors==0) begin
        $display("# PASS");
        $finish(0);
    end else begin
        $display("# FAIL");
        $stop(0);
    end
end

endmodule
