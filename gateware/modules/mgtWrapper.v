//
// Wrap MGT instantiations with common support firmware
//
module mgtWrapper #(
    parameter EVG                 = -1,
    parameter MGT_ID              =  0,
    parameter SAMPLING_CLOCK_RATE = -1,
    parameter DEBUG               = "false",
    parameter DRP_DEBUG           = "false",
    parameter FORCE_GTE_COMMON    = "false"
    ) (
    input         sysClk,
    input         drpStrobe,
    input  [31:0] GPIO_OUT,
    output [31:0] drpStatus,
    output [31:0] latency,

    input         refClk,
    input         samplingClk,
    inout         gt0_qplloutclk_i,    // Xilinx Answer Record 43339
    inout         gt0_qplloutrefclk_i, // Xilinx Answer Record 43339

    input                             evgTxClkIn,
    output                            evgTxClkOut,
    (*mark_debug=DEBUG*) input [15:0] evgTxData,
    (*mark_debug=DEBUG*) input  [1:0] evgTxCharIsK,
    output                            tx_p,
    output                            tx_n,
    input                             evgRxClkIn,
    output                            evgRxClkOut,
    input                             rx_p,
    input                             rx_n);

/*
 * Loopback control (used only during initial lab tests
 * Can't use near-end PCS since the receiver buffer is disabled.
 */
localparam LOOPBACK_OFF          = 3'd0;
localparam LOOPBACK_NEAR_END_PCS = 3'd2;
localparam LOOPBACK = LOOPBACK_OFF;

//////////////////////////////////////////////////////////////////////////////
// DRP and resets
localparam DRP_DATA_WIDTH = 16;
localparam DRP_ADDR_WIDTH = 9;
localparam RESET_CONTROL_WIDTH = 3;
localparam RESET_STATUS_WIDTH = 9;

(*mark_debug=DEBUG*) wire gtTxReset, gtRxReset, cpllReset;
wire rx_fsm_reset_done, rxResetDone, tx_fsm_reset_done, txResetDone, cpllLock;
wire rxIsAligned;

wire [RESET_CONTROL_WIDTH-1:0] resetControl;
assign { gtTxReset,
         gtRxReset,
         cpllReset } = resetControl;
wire [RESET_STATUS_WIDTH-1:0] resetStatus = { gtTxReset,
                                              gtRxReset,
                                              cpllReset,
                                              rxIsAligned,
                                              tx_fsm_reset_done,
                                              rx_fsm_reset_done,
                                              txResetDone,
                                              rxResetDone,
                                              cpllLock };

wire drp_en, drp_we, drp_rdy;
wire [DRP_ADDR_WIDTH-1:0] drp_addr;
wire [DRP_DATA_WIDTH-1:0] drp_di, drp_do;

drpControl #(.DRP_DATA_WIDTH(DRP_DATA_WIDTH),
             .DRP_ADDR_WIDTH(DRP_ADDR_WIDTH),
             .RESET_CONTROL_WIDTH(RESET_CONTROL_WIDTH),
             .RESET_STATUS_WIDTH(RESET_STATUS_WIDTH),
             .DEBUG(DRP_DEBUG))
  drp (
    .clk(sysClk),
    .strobe(drpStrobe),
    .dataIn(GPIO_OUT),
    .dataOut(drpStatus),
    .resetControl(resetControl),
    .resetStatus(resetStatus),
    .drp_en(drp_en),
    .drp_we(drp_we),
    .drp_rdy(drp_rdy),
    .drp_addr(drp_addr),
    .drp_di(drp_di),
    .drp_do(drp_do));

//////////////////////////////////////////////////////////////////////////////
// Receiver alignment detection
localparam COMMAS_NEEDED = 60;
localparam COMMA_COUNTER_RELOAD = COMMAS_NEEDED - 1;
localparam COMMA_COUNTER_WIDTH = $clog2(COMMA_COUNTER_RELOAD+1) + 1;
(*mark_debug=DEBUG*) reg [COMMA_COUNTER_WIDTH-1:0] commaCounter =
                                                           COMMA_COUNTER_RELOAD;
assign rxIsAligned = commaCounter[COMMA_COUNTER_WIDTH-1];
(*mark_debug=DEBUG*) wire [1:0] evgRxCharIsK, evgRxNotInTable;
(*mark_debug=DEBUG*) wire [15:0] evgRxData;
always @(posedge evgRxClkIn) begin
    if ((evgRxNotInTable != 0) || evgRxCharIsK[1]) begin
        commaCounter <= COMMA_COUNTER_RELOAD;
    end
    else if (!rxIsAligned && evgRxCharIsK[0] && (evgRxData[7:0] == 8'hBC)) begin
        commaCounter <= commaCounter - 1;
    end
end

//////////////////////////////////////////////////////////////////////////////
// Round-trip latency measurement
measureLatency #(.SAMPLING_CLOCK_RATE(SAMPLING_CLOCK_RATE),
                 .DEBUG(DEBUG))
  measureLatency_i (
    .sysClk(sysClk),
    .sysLatency(latency),
    .samplingClk(samplingClk),
    .rxValid(rxIsAligned),
    .ping(evgTxData[9]),
    .echo(evgRxData[9]));

//////////////////////////////////////////////////////////////////////////////
// Instantiate appropriate MGT
generate
if (EVG == 1) begin
evg1mgt evg1mgt_i (
    .sysclk_in(sysClk), // input wire sysclk_in
    .soft_reset_tx_in(gtTxReset), // input wire soft_reset_tx_in
    .soft_reset_rx_in(gtRxReset), // input wire soft_reset_rx_in
    .dont_reset_on_data_error_in(1'b1), // input wire dont_reset_on_data_error_in
    .gt0_tx_fsm_reset_done_out(tx_fsm_reset_done), // output wire gt0_tx_fsm_reset_done_out
    .gt0_rx_fsm_reset_done_out(rx_fsm_reset_done), // output wire gt0_rx_fsm_reset_done_out
    .gt0_data_valid_in(1'b1), // input wire gt0_data_valid_in
    .gt0_cpllfbclklost_out(), // output wire gt0_cpllfbclklost_out
    .gt0_cplllock_out(cpllLock), // output wire gt0_cplllock_out
    .gt0_cplllockdetclk_in(sysClk), // input wire gt0_cplllockdetclk_in
    .gt0_cpllreset_in(cpllReset), // input wire gt0_cpllreset_in
    .gt0_gtrefclk0_in(refClk), // input wire gt0_gtrefclk0_in
    .gt0_gtrefclk1_in(1'b0), // input wire gt0_gtrefclk1_in
    .gt0_drpaddr_in(drp_addr), // input wire [8:0] gt0_drpaddr_in
    .gt0_drpclk_in(sysClk), // input wire gt0_drpclk_in
    .gt0_drpdi_in(drp_di), // input wire [15:0] gt0_drpdi_in
    .gt0_drpdo_out(drp_do), // output wire [15:0] gt0_drpdo_out
    .gt0_drpen_in(drp_en), // input wire gt0_drpen_in
    .gt0_drprdy_out(drp_rdy), // output wire gt0_drprdy_out
    .gt0_drpwe_in(drp_we), // input wire gt0_drpwe_in
    .gt0_dmonitorout_out(), // output wire [7:0] gt0_dmonitorout_out
    .gt0_loopback_in(LOOPBACK), // input wire [2:0] gt0_loopback_in
    .gt0_eyescanreset_in(1'b0), // input wire gt0_eyescanreset_in
    .gt0_rxuserrdy_in(1'b1), // input wire gt0_rxuserrdy_in
    .gt0_eyescandataerror_out(), // output wire gt0_eyescandataerror_out
    .gt0_eyescantrigger_in(1'b0), // input wire gt0_eyescantrigger_in
    .gt0_rxusrclk_in(evgRxClkIn), // input wire gt0_rxusrclk_in
    .gt0_rxusrclk2_in(evgRxClkIn), // input wire gt0_rxusrclk2_in
    .gt0_rxdata_out(evgRxData), // output wire [15:0] gt0_rxdata_out
    .gt0_rxdisperr_out(), // output wire [1:0] gt0_rxdisperr_out
    .gt0_rxnotintable_out(evgRxNotInTable), // output wire [1:0] gt0_rxnotintable_out
    .gt0_gtxrxp_in(rx_p), // input wire gt0_gtxrxp_in
    .gt0_gtxrxn_in(rx_n), // input wire gt0_gtxrxn_in
    .gt0_rxdfelpmreset_in(1'b0), // input wire gt0_rxdfelpmreset_in
    .gt0_rxmonitorout_out(), // output wire [6:0] gt0_rxmonitorout_out
    .gt0_rxmonitorsel_in(2'b0), // input wire [1:0] gt0_rxmonitorsel_in
    .gt0_rxoutclk_out(evgRxClkOut), // output wire gt0_rxoutclk_out
    .gt0_rxoutclkfabric_out(), // output wire gt0_rxoutclkfabric_out
    .gt0_gtrxreset_in(gtRxReset), // input wire gt0_gtrxreset_in
    .gt0_rxpmareset_in(1'b0), // input wire gt0_rxpmareset_in
    .gt0_rxcharisk_out(evgRxCharIsK), // output wire [1:0] gt0_rxcharisk_out
    .gt0_rxresetdone_out(rxResetDone), // output wire gt0_rxresetdone_out
    .gt0_gttxreset_in(gtTxReset), // input wire gt0_gttxreset_in
    .gt0_txuserrdy_in(1'b1), // input wire gt0_txuserrdy_in
    .gt0_txusrclk_in(evgTxClkIn), // input wire gt0_txusrclk_in
    .gt0_txusrclk2_in(evgTxClkIn), // input wire gt0_txusrclk2_in
    .gt0_txdata_in(evgTxData), // input wire [15:0] gt0_txdata_in
    .gt0_gtxtxn_out(tx_n), // output wire gt0_gtxtxn_out
    .gt0_gtxtxp_out(tx_p), // output wire gt0_gtxtxp_out
    .gt0_txoutclk_out(evgTxClkOut), // output wire gt0_txoutclk_out
    .gt0_txoutclkfabric_out(), // output wire gt0_txoutclkfabric_out
    .gt0_txoutclkpcs_out(), // output wire gt0_txoutclkpcs_out
    .gt0_txcharisk_in(evgTxCharIsK), // input wire [1:0] gt0_txcharisk_in
    .gt0_txresetdone_out(txResetDone), // output wire gt0_txresetdone_out
    .gt0_qplloutclk_in(gt0_qplloutclk_i), // input wire gt0_qplloutclk_in
    .gt0_qplloutrefclk_in(gt0_qplloutrefclk_i) // input wire gt0_qplloutrefclk_in
     );
end
endgenerate

generate
if ((EVG == 1 && MGT_ID == 0) ||
    (EVG == 2 && MGT_ID == 0 && FORCE_GTE_COMMON == "true")) begin
///////////////////////////////////////////////////////////////////////////////
// Xilinx Answer Record 43339
// Instantiate a GTXE2_COMMON even though QPLL is unused.
// Needed to set BIAS_CFG properly.
// FWIW, I see no change after doing all this.

localparam WRAPPER_SIM_GTRESET_SPEEDUP ="false";
localparam SIM_VERSION = "4.0";
localparam QPLL_FBDIV_IN = 10'b0000100000;
localparam QPLL_FBDIV_RATIO = 1'b1;

wire [15:0] tied_to_ground_vec_i = 0;
wire tied_to_ground_i = 0;
wire tied_to_vcc_i = 1;
wire GT0_GTREFCLK0_COMMON_IN = refClk;
wire GT0_QPLLLOCKDETCLK_IN = sysClk;
wire GT0_QPLLRESET_IN = cpllReset;
wire GT0_QPLLLOCK_OUT;
wire GT0_QPLLREFCLKLOST_OUT;

// Make this a black box for simulation
`ifndef SIMULATE
// The code copied verbatim from the answer record:

//_________________________________________________________________________
    //_________________________________________________________________________
    //_________________________GTXE2_COMMON____________________________________

    GTXE2_COMMON #
    (
            // Simulation attributes
            .SIM_RESET_SPEEDUP   (WRAPPER_SIM_GTRESET_SPEEDUP),
            .SIM_QPLLREFCLK_SEL  (3'b001),
            .SIM_VERSION         (SIM_VERSION),


           //----------------COMMON BLOCK Attributes---------------
            .BIAS_CFG                               (64'h0000040000001000),
            .COMMON_CFG                             (32'h00000000),
            .QPLL_CFG                               (27'h06801C1),
            .QPLL_CLKOUT_CFG                        (4'b0000),
            .QPLL_COARSE_FREQ_OVRD                  (6'b010000),
            .QPLL_COARSE_FREQ_OVRD_EN               (1'b0),
            .QPLL_CP                                (10'b0000011111),
            .QPLL_CP_MONITOR_EN                     (1'b0),
            .QPLL_DMONITOR_SEL                      (1'b0),
            .QPLL_FBDIV                             (QPLL_FBDIV_IN),
            .QPLL_FBDIV_MONITOR_EN                  (1'b0),
            .QPLL_FBDIV_RATIO                       (QPLL_FBDIV_RATIO),
            .QPLL_INIT_CFG                          (24'h000006),
            .QPLL_LOCK_CFG                          (16'h21E8),
            .QPLL_LPF                               (4'b1111),
            .QPLL_REFCLK_DIV                        (1)

    )
    gtxe2_common_0_i
    (
        //----------- Common Block  - Dynamic Reconfiguration Port (DRP) -----------
        .DRPADDR                        (tied_to_ground_vec_i[7:0]),
        .DRPCLK                         (tied_to_ground_i),
        .DRPDI                          (tied_to_ground_vec_i[15:0]),
        .DRPDO                          (),
        .DRPEN                          (tied_to_ground_i),
        .DRPRDY                         (),
        .DRPWE                          (tied_to_ground_i),
        //-------------------- Common Block  - Ref Clock Ports ---------------------
        .GTGREFCLK                      (tied_to_ground_i),
        .GTNORTHREFCLK0                 (tied_to_ground_i),
        .GTNORTHREFCLK1                 (tied_to_ground_i),
        .GTREFCLK0                      (GT0_GTREFCLK0_COMMON_IN),
        .GTREFCLK1                      (tied_to_ground_i),
        .GTSOUTHREFCLK0                 (tied_to_ground_i),
        .GTSOUTHREFCLK1                 (tied_to_ground_i),
        //----------------------- Common Block - QPLL Ports ------------------------
        .QPLLDMONITOR                   (),
        .QPLLFBCLKLOST                  (),
        .QPLLLOCK                       (GT0_QPLLLOCK_OUT),
        .QPLLLOCKDETCLK                 (GT0_QPLLLOCKDETCLK_IN),
        .QPLLLOCKEN                     (tied_to_vcc_i),
        .QPLLOUTCLK                     (gt0_qplloutclk_i),
        .QPLLOUTREFCLK                  (gt0_qplloutrefclk_i),
        .QPLLOUTRESET                   (tied_to_ground_i),
        .QPLLPD                         (tied_to_ground_i),
        .QPLLREFCLKLOST                 (GT0_QPLLREFCLKLOST_OUT),
        .QPLLREFCLKSEL                  (3'b001),
        .QPLLRESET                      (GT0_QPLLRESET_IN),
        .QPLLRSVD1                      (16'b0000000000000000),
        .QPLLRSVD2                      (5'b11111),
        .REFCLKOUTMONITOR               (),
        //--------------------------- Common Block Ports ---------------------------
        .BGBYPASSB                      (tied_to_vcc_i),
        .BGMONITORENB                   (tied_to_vcc_i),
        .BGPDB                          (tied_to_vcc_i),
        .BGRCALOVRD                     (5'b00000),
        .PMARSVD                        (8'b00000000),
        .RCALENB                        (tied_to_vcc_i)

    );
`endif // `ifndef SIMULATE
end
endgenerate

generate
if (EVG == 2) begin
evg2mgt evg2mgt_i (
    .sysclk_in(sysClk), // input wire sysclk_in
    .soft_reset_tx_in(gtTxReset), // input wire soft_reset_tx_in
    .soft_reset_rx_in(gtRxReset), // input wire soft_reset_rx_in
    .dont_reset_on_data_error_in(1'b1), // input wire dont_reset_on_data_error_in
    .gt0_tx_fsm_reset_done_out(tx_fsm_reset_done), // output wire gt0_tx_fsm_reset_done_out
    .gt0_rx_fsm_reset_done_out(rx_fsm_reset_done), // output wire gt0_rx_fsm_reset_done_out
    .gt0_data_valid_in(1'b1), // input wire gt0_data_valid_in
    .gt0_cpllfbclklost_out(), // output wire gt0_cpllfbclklost_out
    .gt0_cplllock_out(cpllLock), // output wire gt0_cplllock_out
    .gt0_cplllockdetclk_in(sysClk), // input wire gt0_cplllockdetclk_in
    .gt0_cpllreset_in(cpllReset), // input wire gt0_cpllreset_in
    .gt0_gtrefclk0_in(1'b0), // input wire gt0_gtrefclk0_in
    .gt0_gtrefclk1_in(refClk), // input wire gt0_gtrefclk1_in
    .gt0_drpaddr_in(drp_addr), // input wire [8:0] gt0_drpaddr_in
    .gt0_drpclk_in(sysClk), // input wire gt0_drpclk_in
    .gt0_drpdi_in(drp_di), // input wire [15:0] gt0_drpdi_in
    .gt0_drpdo_out(drp_do), // output wire [15:0] gt0_drpdo_out
    .gt0_drpen_in(drp_en), // input wire gt0_drpen_in
    .gt0_drprdy_out(drp_rdy), // output wire gt0_drprdy_out
    .gt0_drpwe_in(drp_we), // input wire gt0_drpwe_in
    .gt0_dmonitorout_out(), // output wire [7:0] gt0_dmonitorout_out
    .gt0_loopback_in(LOOPBACK), // input wire [2:0] gt0_loopback_in
    .gt0_eyescanreset_in(1'b0), // input wire gt0_eyescanreset_in
    .gt0_rxuserrdy_in(1'b1), // input wire gt0_rxuserrdy_in
    .gt0_eyescandataerror_out(), // output wire gt0_eyescandataerror_out
    .gt0_eyescantrigger_in(1'b0), // input wire gt0_eyescantrigger_in
    .gt0_rxusrclk_in(evgRxClkIn), // input wire gt0_rxusrclk_in
    .gt0_rxusrclk2_in(evgRxClkIn), // input wire gt0_rxusrclk2_in
    .gt0_rxdata_out(evgRxData), // output wire [15:0] gt0_rxdata_out
    .gt0_rxdisperr_out(), // output wire [1:0] gt0_rxdisperr_out
    .gt0_rxnotintable_out(evgRxNotInTable), // output wire [1:0] gt0_rxnotintable_out
    .gt0_gtxrxp_in(rx_p), // input wire gt0_gtxrxp_in
    .gt0_gtxrxn_in(rx_n), // input wire gt0_gtxrxn_in
    .gt0_rxdfelpmreset_in(1'b0), // input wire gt0_rxdfelpmreset_in
    .gt0_rxmonitorout_out(), // output wire [6:0] gt0_rxmonitorout_out
    .gt0_rxmonitorsel_in(2'b0), // input wire [1:0] gt0_rxmonitorsel_in
    .gt0_rxoutclk_out(evgRxClkOut), // output wire gt0_rxoutclk_out
    .gt0_rxoutclkfabric_out(), // output wire gt0_rxoutclkfabric_out
    .gt0_gtrxreset_in(gtRxReset), // input wire gt0_gtrxreset_in
    .gt0_rxpmareset_in(1'b0), // input wire gt0_rxpmareset_in
    .gt0_rxcharisk_out(evgRxCharIsK), // output wire [1:0] gt0_rxcharisk_out
    .gt0_rxresetdone_out(rxResetDone), // output wire gt0_rxresetdone_out
    .gt0_gttxreset_in(gtTxReset), // input wire gt0_gttxreset_in
    .gt0_txuserrdy_in(1'b1), // input wire gt0_txuserrdy_in
    .gt0_txusrclk_in(evgTxClkIn), // input wire gt0_txusrclk_in
    .gt0_txusrclk2_in(evgTxClkIn), // input wire gt0_txusrclk2_in
    .gt0_txdata_in(evgTxData), // input wire [15:0] gt0_txdata_in
    .gt0_gtxtxn_out(tx_n), // output wire gt0_gtxtxn_out
    .gt0_gtxtxp_out(tx_p), // output wire gt0_gtxtxp_out
    .gt0_txoutclk_out(evgTxClkOut), // output wire gt0_txoutclk_out
    .gt0_txoutclkfabric_out(), // output wire gt0_txoutclkfabric_out
    .gt0_txoutclkpcs_out(), // output wire gt0_txoutclkpcs_out
    .gt0_txcharisk_in(evgTxCharIsK), // input wire [1:0] gt0_txcharisk_in
    .gt0_txresetdone_out(txResetDone), // output wire gt0_txresetdone_out
    .gt0_qplloutclk_in(gt0_qplloutclk_i), // input wire gt0_qplloutclk_in
    .gt0_qplloutrefclk_in(gt0_qplloutrefclk_i) // input wire gt0_qplloutrefclk_in
     );
end
endgenerate
endmodule
