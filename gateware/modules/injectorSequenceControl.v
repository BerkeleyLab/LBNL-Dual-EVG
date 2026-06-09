// Provide sequencer start signals for injector
module injectorSequenceControl #(
    parameter SYSCLK_RATE               = -1,
    parameter DEBUG                     = "FALSE",
    parameter ALIGNMENT_SYNC_COUNT      =  2,
    parameter RF_COINC_IDX_WIDTH        = -1,
    parameter RF_ALIGN_IDX_WIDTH        = -1,
    parameter RF_COINC_TERM_WIDTH       = -1,
    parameter INJ_DELAY_WIDTH           = -1,
    parameter EXTR_DELAY_WIDTH          = -1,
    parameter [ALIGNMENT_SYNC_COUNT*32-1:0]
        TX_CLK_PER_ALIGNMENT             = {-32'd1, -32'd1}
    ) (
    input              sysClk,
    input       [31:0] sysGPIO_OUT,

    input              sysCsrStrobe,
    output wire [31:0] sysStatus,

    input              sysCsrAlignStrobe,
    output wire [31:0] sysAlignStatus,

    input              sysCsrTargetStrobe,
    output wire [31:0] sysTargetStatus,
    output wire [31:0] sysTargetStatus2,

    input              powerline_a,
    output             sysPowerlineMon,

    input                               evgTxClk,

    input      [RF_COINC_IDX_WIDTH-1:0] evgRFCoincCount,
    input      [RF_ALIGN_IDX_WIDTH-1:0] evgRFAlignCount,
    output   [ALIGNMENT_SYNC_COUNT-1:0] evgAlignCounterDone,

    output      [RF_COINC_IDX_WIDTH-1:0] evgRFCoincCountMon,
    output      [RF_ALIGN_IDX_WIDTH-1:0] evgRFAlignCountMon,
    output                               evgPowerlineMon,
    output reg                           evgSeqBusy = 0,

    input    [ALIGNMENT_SYNC_COUNT-1:0] evgHeartbeat,
    output                              evgHeartbeatAlign,
    output                              evgHeartbeatCore,
    output reg                          evgSequenceStart = 0,
    output        [INJ_DELAY_WIDTH-1:0] evgInjDelay,
    output       [EXTR_DELAY_WIDTH-1:0] evgExtrDelay);

localparam RF_COINC_IDX_WIDTH_MAX = 16;

generate
if (RF_COINC_IDX_WIDTH > RF_COINC_IDX_WIDTH_MAX) begin
    RF_COINC_IDX_WIDTH_bigger_than_RF_COINC_IDX_WIDTH_MAX err();
end
endgenerate

localparam RF_COINC_TERM_WIDTH_MAX = 16;

generate
if (RF_COINC_TERM_WIDTH > RF_COINC_TERM_WIDTH_MAX) begin
    RF_COINC_TERM_WIDTH_bigger_than_RF_COINC_TERM_WIDTH_MAX err2();
end
endgenerate

localparam RF_ALIGN_IDX_WIDTH_MAX = 8;

generate
if (RF_ALIGN_IDX_WIDTH > RF_ALIGN_IDX_WIDTH_MAX) begin
    RF_ALIGN_IDX_WIDTH_bigger_than_RF_ALIGN_IDX_WIDTH_MAX err3();
end
endgenerate

localparam ALIGNMENT_SYNC_COUNT_MAX_WIDTH = 3;

if (ALIGNMENT_SYNC_COUNT > (1 << ALIGNMENT_SYNC_COUNT_MAX_WIDTH)) begin
    ALIGNMENT_SYNC_COUNT_is_bigger_than_8 err4();
end

localparam ALIGNMENT_SYNC_COUNT_WIDTH = (ALIGNMENT_SYNC_COUNT <= 1)?
    1 : $clog2(ALIGNMENT_SYNC_COUNT);

///////////////////////////////////////////////////////////////////////////////
// System clock domain

// Scale sysClk to milliseconds
localparam SYSCLK_PER_MILLISECOND = SYSCLK_RATE / 1000;
localparam SYSCLK_DIVIDER_RELOAD = SYSCLK_PER_MILLISECOND - 2;
localparam SYSCLK_DIVIDER_WIDTH = $clog2(SYSCLK_DIVIDER_RELOAD+1)+1;
reg [SYSCLK_DIVIDER_WIDTH-1:0] sysClkDivider = SYSCLK_DIVIDER_RELOAD;
wire sysClkDividerDone = sysClkDivider[SYSCLK_DIVIDER_WIDTH-1];

// Injection cycle trigger
localparam CYCLE_COUNTER_RELOAD_WIDTH = 16;
localparam CYCLE_COUNTER_WIDTH = CYCLE_COUNTER_RELOAD_WIDTH + 1;
reg [CYCLE_COUNTER_WIDTH-1:0] sysCycleCounter = 0;
wire sysCycleCounterDone = sysCycleCounter[CYCLE_COUNTER_WIDTH-1];
reg [CYCLE_COUNTER_RELOAD_WIDTH-1:0] sysCycleCounterReload = 1400 - 2;
reg sysCycleEnabled = 0;
reg sysInjectorStartToggle = 0;

always @(posedge sysClk) begin
    if (sysClkDividerDone) begin
        sysClkDivider <= SYSCLK_DIVIDER_RELOAD;
    end
    else begin
        sysClkDivider <= sysClkDivider - 1;
    end

    if (sysCsrStrobe) begin
        if (sysGPIO_OUT[31]) begin
            sysCycleCounterReload <= sysGPIO_OUT[CYCLE_COUNTER_RELOAD_WIDTH-1:0];
        end
        else begin
            if (sysGPIO_OUT[1]) begin
                sysCycleEnabled <= 0;
            end
            else if (sysGPIO_OUT[0]) begin
                sysCycleEnabled <= 1;
            end
            else if (sysGPIO_OUT[7] && !sysCycleEnabled) begin
                sysInjectorStartToggle <= !sysInjectorStartToggle;
            end
        end
    end

    if (sysCycleEnabled) begin
        if (sysClkDividerDone) begin
            if (sysCycleCounterDone) begin
                sysCycleCounter <= {1'b0, sysCycleCounterReload};
                sysInjectorStartToggle <= !sysInjectorStartToggle;
            end
            else begin
                sysCycleCounter <= sysCycleCounter - 1;
            end
        end
    end
    else begin
        sysCycleCounter <= 0;
    end
end

// Alignment CSR
reg [ALIGNMENT_SYNC_COUNT_MAX_WIDTH-1:0 ] sysAlignCounterSel = 0;
reg [ALIGNMENT_SYNC_COUNT_MAX_WIDTH-1:0 ] sysEvgHeartbeatSel = 0;
always @(posedge sysClk) begin
    if (sysCsrAlignStrobe) begin
        if (sysGPIO_OUT[31]) begin
            sysAlignCounterSel <= sysGPIO_OUT[ALIGNMENT_SYNC_COUNT_MAX_WIDTH-1:0];
        end
        else if (sysGPIO_OUT[30]) begin
            sysEvgHeartbeatSel <= sysGPIO_OUT[ALIGNMENT_SYNC_COUNT_MAX_WIDTH-1:0];
        end
    end
end

localparam MOD_125_WIDTH = 7;
localparam BR_BUCKET_SUM_WIDTH = (RF_COINC_TERM_WIDTH > MOD_125_WIDTH)?
    RF_COINC_TERM_WIDTH+1 : MOD_125_WIDTH+1;

// Forward to Sys clk
reg  [MOD_125_WIDTH-1:0] evgBRBucketLatch = 0;
reg evgBRBucketLatch_valid = 0;
wire [MOD_125_WIDTH-1:0] sysBRBucketLatch;
reg  [RF_ALIGN_IDX_WIDTH-1:0] evgRFAlignCountLatch = 0;
reg evgRFAlignCountLatch_valid = 0;
wire [RF_ALIGN_IDX_WIDTH-1:0] sysRFAlignCountLatch;

forwardData #(
    .DATA_WIDTH(RF_ALIGN_IDX_WIDTH+
                MOD_125_WIDTH))
  forwardDataToSys (
    .inClk(evgTxClk),
    .inData({evgRFAlignCountLatch,
            evgBRBucketLatch}),
    .outClk(sysClk),
    .outData({sysRFAlignCountLatch,
            sysBRBucketLatch}));

// Target CSR
//
// bBR= (rfCOINCTERM + rfALIGNTERM)(mod 125)
//
// rfCOINCTERM = (43.rfCOINC)(mod 125)
// rfCOINC = (5.bAR)(mod 304)
//
// rfALIGNTERM = (72.iAR,BR) (mod125)
//
reg [RF_COINC_IDX_WIDTH-1:0 ] sysRFCoincIdxSel = 0;
reg [RF_COINC_TERM_WIDTH-1:0 ] sysRFCoincTerm = 0;
always @(posedge sysClk) begin
    if (sysCsrTargetStrobe) begin
        sysRFCoincIdxSel <= sysGPIO_OUT[0+:RF_COINC_IDX_WIDTH];
        // rfCOINCTERM comes pre-calculatede from microblaze
        sysRFCoincTerm <= sysGPIO_OUT[RF_COINC_IDX_WIDTH_MAX+:RF_COINC_TERM_WIDTH];
    end
end

assign sysTargetStatus = {{RF_COINC_TERM_WIDTH_MAX-RF_COINC_TERM_WIDTH{1'b0}}, sysRFCoincTerm,
                        {RF_COINC_IDX_WIDTH_MAX-RF_COINC_IDX_WIDTH{1'b0}}, sysRFCoincIdxSel};
assign sysTargetStatus2 = {{16-RF_ALIGN_IDX_WIDTH{1'b0}}, sysRFAlignCountLatch,
                        {16-MOD_125_WIDTH{1'b0}}, sysBRBucketLatch};

// Power line trigger
wire sysPowerline, sysPowerlineTimeout;
powerlineTrigger #(.CLK_RATE(SYSCLK_RATE))
  powerlineTrigger (
    .clk(sysClk),
    .powerline_a(powerline_a),
    .trigger(sysPowerline),
    .powerlineTimeout(sysPowerlineTimeout));

assign sysPowerlineMon = sysPowerline;

///////////////////////////////////////////////////////////////////////////////
// Event generator clock domain

// Monitor outputs
assign evgRFCoincCountMon = evgRFCoincCount;
assign evgRFAlignCountMon = evgRFAlignCount;

// Forward to TX clk
wire [ALIGNMENT_SYNC_COUNT_MAX_WIDTH-1:0] evgAlignCounterSel;
wire [ALIGNMENT_SYNC_COUNT_MAX_WIDTH-1:0] evgHeartbeatSel;
wire [RF_COINC_IDX_WIDTH-1:0] evgRFCoincIdxSel;
wire [RF_COINC_TERM_WIDTH-1:0] evgRFCoincTerm;

forwardData #(
    .DATA_WIDTH(RF_COINC_TERM_WIDTH+
                RF_COINC_IDX_WIDTH+
                ALIGNMENT_SYNC_COUNT_MAX_WIDTH+
                ALIGNMENT_SYNC_COUNT_MAX_WIDTH))
  forwardDataToEVG (
    .inClk(sysClk),
    .inData({sysRFCoincTerm,
            sysRFCoincIdxSel,
            sysAlignCounterSel,
            sysEvgHeartbeatSel}),
    .outClk(evgTxClk),
    .outData({evgRFCoincTerm,
              evgRFCoincIdxSel,
              evgAlignCounterSel,
              evgHeartbeatSel}));

wire [ALIGNMENT_SYNC_COUNT-1:0] alignmentCounterDone;
wire [ALIGNMENT_SYNC_COUNT-1:0] alignmentCounterSynced;
wire alignmentCounterSyncedAll = &alignmentCounterSynced;

genvar i;
generate
for (i = 0; i < ALIGNMENT_SYNC_COUNT; i = i + 1) begin

localparam TX_CLK_PER_ALIGNMENT_LOCAL = TX_CLK_PER_ALIGNMENT[i*32+:32];

alignmentGenerator #(
    .CLK_PER_ALIGNMENT(TX_CLK_PER_ALIGNMENT_LOCAL))
  alignmentGenerator (
    .clk(evgTxClk),

    .heartbeatStrobe(evgHeartbeat[i]),
    .alignmentCounterDone(alignmentCounterDone[i]),
    .alignmentCounterSynced(alignmentCounterSynced[i]));

end
endgenerate

assign evgAlignCounterDone = alignmentCounterDone;

///////////////////////////////////////////////////////////////////////////////
// Main trigger FSM

// Detect cycle start requests
(*ASYNC_REG="true"*) reg injectorStartToggle_m = 0;
reg injectorStartToggle = 0, injectorStartToggle_d = 0;

// Detect power line coincidence
(*ASYNC_REG="true"*) reg powerline_m = 0;
(*ASYNC_REG="true"*) reg powerlineTimeout_m = 0;
reg powerline = 0, powerline_d = 0, powerlineTimeout = 0;
assign evgPowerlineMon = powerline;

// Delay signals
wire evgDelays_valid;
reg evgDelaysLatch_valid = 0;

// Synchronization state machine
localparam ST_IDLE                = 3'd0,
           ST_AWAIT_POWER_LINE    = 3'd1,
           ST_AWAIT_ALIGNMENT     = 3'd2,
           ST_AWAIT_COINC_IDX_SEL = 3'd3,
           ST_AWAIT_BUCKET_CALC   = 3'd4,
           ST_AWAIT_DELAYS_CALC   = 3'd5,
           ST_TRIGGER             = 3'd6;
reg [2:0] injectorStartState = ST_IDLE;
// Latch only the part that will be used
reg [ALIGNMENT_SYNC_COUNT_WIDTH-1:0] evgAlignCounterSelLatch = 0;
reg [RF_COINC_IDX_WIDTH-1:0] evgRFCoincIdxSelLatch = 0;
reg evgRFCoincIdxSelLatch_valid = 0;

always @(posedge evgTxClk) begin
    injectorStartToggle_m <= sysInjectorStartToggle;
    injectorStartToggle   <= injectorStartToggle_m;
    injectorStartToggle_d <= injectorStartToggle;
    powerline_m <= sysPowerline;
    powerline   <= powerline_m;
    powerline_d <= powerline;
    powerlineTimeout_m <= sysPowerlineTimeout;
    powerlineTimeout   <= powerlineTimeout_m;

    // Default values
    evgRFAlignCountLatch_valid <= 0;
    evgRFCoincIdxSelLatch_valid <= 0;
    evgBRBucketLatch_valid <= 0;
    evgDelaysLatch_valid <= 0;

    case (injectorStartState)
    ST_IDLE: begin
        evgSequenceStart <= 0;
        evgSeqBusy <= 0;
        if (injectorStartToggle != injectorStartToggle_d) begin
            evgSeqBusy <= 1;
            injectorStartState <= ST_AWAIT_POWER_LINE;
            evgAlignCounterSelLatch <= evgAlignCounterSel[ALIGNMENT_SYNC_COUNT_WIDTH-1:0];
            evgRFCoincIdxSelLatch <= evgRFCoincIdxSel;
            // evgRFCoincIdxSelLatch_valid is only asserted when calculating
            // delays
        end
    end

    ST_AWAIT_POWER_LINE: begin
        if (powerlineTimeout || (powerline && !powerline_d)) begin
            injectorStartState <= ST_AWAIT_ALIGNMENT;
        end
    end

    ST_AWAIT_ALIGNMENT: begin
        // State changes always happen on the transition of evgRFCoincCount
        // "Max" -> 0. So, it's always guaranteed that the
        // evgRFCoincIdxSel == evgRFCoincCount will happen in the next alignment cycle.
        if (alignmentCounterDone[evgAlignCounterSelLatch]) begin
            injectorStartState <= ST_AWAIT_COINC_IDX_SEL;
        end
    end

    ST_AWAIT_COINC_IDX_SEL: begin
        if (evgRFCoincIdxSelLatch == evgRFCoincCount) begin
            evgRFAlignCountLatch <= evgRFAlignCount;
            evgRFAlignCountLatch_valid <= 1;
            injectorStartState <= ST_AWAIT_BUCKET_CALC;
        end
    end

    ST_AWAIT_BUCKET_CALC: begin
        if (evgBRBucket_valid) begin
            injectorStartState <= ST_AWAIT_DELAYS_CALC;
            // BR bucket that needs to be selected to inject into
            // the specified AR bucket
            evgBRBucketLatch <= evgBRBucket;
            evgBRBucketLatch_valid <= 1;
            evgRFCoincIdxSelLatch_valid <= 1;
        end
    end

    ST_AWAIT_DELAYS_CALC: begin
        if (evgDelays_valid) begin
            injectorStartState <= ST_TRIGGER;
            evgBRInjDelayLatch <= evgBRInjDelay;
            evgBRExtrDelayLatch <= evgBRExtrDelay;
            evgDelaysLatch_valid <= 1;
        end
    end

    ST_TRIGGER: begin
        evgSequenceStart <= 1;
        injectorStartState <= ST_IDLE;
    end

    default: begin
        injectorStartState <= ST_IDLE;
    end
    endcase
end

generate
if (MOD_125_WIDTH > INJ_DELAY_WIDTH) begin
    RF_COINC_IDX_WIDTH_bigger_than_RF_COINC_IDX_WIDTH_MAX err5();
end
endgenerate

assign evgInjDelay = {{INJ_DELAY_WIDTH-MOD_125_WIDTH{1'b0}},
                        evgBRInjDelayLatch};

generate
if (EXTR_DELAY_TERM_WIDTH > EXTR_DELAY_WIDTH) begin
    EXTR_DELAY_TERM_WIDTH_bigger_than_EXTR_DELAY_WIDTH err6();
end
endgenerate

assign evgExtrDelay = {{EXTR_DELAY_WIDTH-EXTR_DELAY_TERM_WIDTH{1'b0}},
                        evgBRExtrDelayLatch};

///////////////////////////////////////////////////////////////////////////////
// Bucket/Delay calculation
//
// rfALIGNTERM = (72.iAR,BR) (mod125)

localparam RF_ALIGN_COEFF = 72;
localparam RF_ALIGN_COEFF_WIDTH = $clog2(RF_ALIGN_COEFF+1);

localparam RF_ALIGN_TERM_WIDTH = RF_ALIGN_COEFF_WIDTH + RF_ALIGN_IDX_WIDTH;

reg [RF_ALIGN_IDX_WIDTH-1:0] evgRFAlignCount_r = 0;
reg evgRFAlignCount_r_valid = 0;

reg [RF_ALIGN_TERM_WIDTH-1:0] evgRFAlignMult_r = 0;
reg evgRFAlignMult_r_valid = 0;

reg [RF_ALIGN_TERM_WIDTH-1:0] evgRFAlignMult = 0;
reg evgRFAlignMult_valid = 0;

always @(posedge evgTxClk) begin
    evgRFAlignCount_r <= evgRFAlignCountLatch;
    evgRFAlignCount_r_valid <= evgRFAlignCountLatch_valid;

    evgRFAlignMult_r <= RF_ALIGN_COEFF * evgRFAlignCount_r;
    evgRFAlignMult_r_valid <= evgRFAlignCount_r_valid;

    evgRFAlignMult  <= evgRFAlignMult_r;
    evgRFAlignMult_valid <= evgRFAlignMult_r_valid;
end

///////////////////////////////////////////////////////////////////////////////
// Modulo calculation
wire [MOD_125_WIDTH-1:0] evgRFAlignTerm;
wire evgRFAlignTerm_valid;

mod125_reduction #(
    .WIDTH(RF_ALIGN_TERM_WIDTH))
  mod125RFAlign (
    .clk(evgTxClk),
    .data_in(evgRFAlignMult),
    .valid_in(evgRFAlignMult_valid),
    .data_out(evgRFAlignTerm),
    .valid_out(evgRFAlignTerm_valid)
);

reg [BR_BUCKET_SUM_WIDTH-1:0] evgBRBucketSum = 0;
reg evgBRBucketSum_valid = 0;
wire [MOD_125_WIDTH-1:0] evgBRBucket;
wire evgBRBucket_valid;

always @(posedge evgTxClk) begin
    evgBRBucketSum <= evgRFCoincTerm + evgRFAlignTerm;
    evgBRBucketSum_valid <= evgRFAlignTerm_valid;
end

mod125_reduction #(
    .WIDTH(BR_BUCKET_SUM_WIDTH))
  mod125BRBucket (
    .clk(evgTxClk),
    .data_in(evgBRBucketSum),
    .valid_in(evgBRBucketSum_valid),
    .data_out(evgBRBucket),
    .valid_out(evgBRBucket_valid)
);

///////////////////////////////////////////////////////////////////////////////
// Injection delays calculation
//
// INJ_DELAY = (94.bBR) (mod125)

localparam INJ_DELAY_COEFF = 94;
localparam INJ_DELAY_COEFF_WIDTH = $clog2(INJ_DELAY_COEFF+1);

localparam INJ_DELAY_TERM_WIDTH = INJ_DELAY_COEFF_WIDTH + MOD_125_WIDTH;

reg [MOD_125_WIDTH-1:0] evgBRBucketInj = 0;
reg evgBRBucketInj_valid = 0;

reg [INJ_DELAY_TERM_WIDTH-1:0] evgBRBucketInjMult_r = 0;
reg evgBRBucketInjMult_r_valid = 0;

reg [INJ_DELAY_TERM_WIDTH-1:0] evgBRBucketInjMult = 0;
reg evgBRBucketInjMult_valid = 0;

always @(posedge evgTxClk) begin
    evgBRBucketInj <= evgBRBucketLatch;
    evgBRBucketInj_valid <= evgBRBucketLatch_valid;

    evgBRBucketInjMult_r <= INJ_DELAY_COEFF * evgBRBucketInj;
    evgBRBucketInjMult_r_valid <= evgBRBucketInj_valid;

    evgBRBucketInjMult  <= evgBRBucketInjMult_r;
    evgBRBucketInjMult_valid <= evgBRBucketInjMult_r_valid;
end

///////////////////////////////////////////////////////////////////////////////
// Modulo calculation
//

wire [MOD_125_WIDTH-1:0] evgBRInjDelay;
wire evgBRInjDelay_valid;
reg [MOD_125_WIDTH-1:0] evgBRInjDelayLatch = 0;

mod125_reduction #(
    .WIDTH(INJ_DELAY_TERM_WIDTH))
  mod125InjDelay (
    .clk(evgTxClk),
    .data_in(evgBRBucketInjMult),
    .valid_in(evgBRBucketInjMult_valid),
    .data_out(evgBRInjDelay),
    .valid_out(evgBRInjDelay_valid)
);

///////////////////////////////////////////////////////////////////////////////
// Extraction delays calculation
//
// EXTR_DELAY = 152.rfCoinc

localparam EXTR_DELAY_COEFF = 152;
localparam EXTR_DELAY_COEFF_WIDTH = $clog2(EXTR_DELAY_COEFF+1);

localparam EXTR_DELAY_TERM_WIDTH = EXTR_DELAY_COEFF_WIDTH + RF_COINC_IDX_WIDTH;

reg [RF_COINC_IDX_WIDTH-1:0] evgRFCoincIdxSelExtr = 0;
reg evgRFCoincIdxSelExtr_valid = 0;

reg [EXTR_DELAY_TERM_WIDTH-1:0] evgRFCoincIdxSelExtrMult_r = 0;
reg evgRFCoincIdxSelExtrMult_r_valid = 0;

reg [EXTR_DELAY_TERM_WIDTH-1:0] evgRFCoincIdxSelExtrMult = 0;
reg evgRFCoincIdxSelExtrMult_valid = 0;

reg [EXTR_DELAY_TERM_WIDTH-1:0] evgBRExtrDelayLatch = 0;

always @(posedge evgTxClk) begin
    evgRFCoincIdxSelExtr <= evgRFCoincIdxSelLatch;
    evgRFCoincIdxSelExtr_valid <= evgRFCoincIdxSelLatch_valid;

    evgRFCoincIdxSelExtrMult_r <= EXTR_DELAY_COEFF * evgRFCoincIdxSelExtr;
    evgRFCoincIdxSelExtrMult_r_valid <= evgRFCoincIdxSelExtr_valid;

    evgRFCoincIdxSelExtrMult  <= evgRFCoincIdxSelExtrMult_r;
    evgRFCoincIdxSelExtrMult_valid <= evgRFCoincIdxSelExtrMult_r_valid;
end

wire [EXTR_DELAY_TERM_WIDTH-1:0] evgBRExtrDelay = evgRFCoincIdxSelExtrMult;
wire evgBRExtrDelay_valid = evgRFCoincIdxSelExtrMult_valid;

// Get the latest valid only
assign evgDelays_valid = evgBRInjDelay_valid;

///////////////////////////////////////////////////////////////////////////////
// Heartbeats/status
//

// Alignement heartbeat
wire [ALIGNMENT_SYNC_COUNT_WIDTH-1:0] evgHeartbeatAlignSel =
    evgAlignCounterSel[ALIGNMENT_SYNC_COUNT_WIDTH-1:0];
assign evgHeartbeatAlign = evgHeartbeat[evgHeartbeatAlignSel];

// EVG heartbeat
wire [ALIGNMENT_SYNC_COUNT_WIDTH-1:0] evgHeartbeatCoreSel =
    evgHeartbeatSel[ALIGNMENT_SYNC_COUNT_WIDTH-1:0];
assign evgHeartbeatCore = evgHeartbeat[evgHeartbeatCoreSel];

// Don't bother with CDC for alignmentCounterSynced. This is a very slow
// signal. Sampled ~ @1s.
assign sysStatus = { !sysPowerlineTimeout, alignmentCounterSyncedAll,
                     {24-2-CYCLE_COUNTER_RELOAD_WIDTH{1'b0}},
                     sysCycleCounterReload,
                     {8-1{1'b0}}, sysCycleEnabled };

// in TX CLK domain, but these signals change only ~@1s
assign sysAlignStatus = {{12{1'b0}},
                        {4-ALIGNMENT_SYNC_COUNT_MAX_WIDTH{1'b0}}, evgHeartbeatSel,
                        {4-ALIGNMENT_SYNC_COUNT_MAX_WIDTH{1'b0}}, evgAlignCounterSel,
                        {4-ALIGNMENT_SYNC_COUNT_WIDTH{1'b0}}, evgAlignCounterSelLatch,
                        {8-ALIGNMENT_SYNC_COUNT{1'b0}}, alignmentCounterSynced };

generate
if (DEBUG != "TRUE" && DEBUG != "FALSE" && DEBUG != "true" && DEBUG != "false") begin
    DEBUG_only_TRUE_or_FALSE_SUPPORTED();
end
endgenerate

generate
if (DEBUG == "TRUE" || DEBUG == "true") begin

`ifndef SIMULATE
ila_td256_s4096_cap ila_td256_s4096_cap_inst (
    .clk(evgTxClk),
    .probe0({
        injectorStartState,
        evgBRBucketLatch_valid,
        evgRFCoincIdxSelLatch_valid,
        evgDelays_valid,
        evgDelaysLatch_valid,
        evgSequenceStart,
        evgBRBucketLatch,
        evgBRInjDelayLatch,
        evgBRExtrDelayLatch,
        evgInjDelay,
        evgExtrDelay
    })
);
`endif

end // end if
endgenerate

endmodule
