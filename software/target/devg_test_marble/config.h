/*
 * Configuration parameters shared between software and firmware
 * The restrictions noted in gpio.h apply here, too.
 */
#define CFG_MINIMUM_INJECTION_CYCLE_MS      990
#define CFG_MAXIMUM_INJECTION_CYCLE_MS      2010
#define CFG_HARDWARE_TRIGGER_COUNT          6
#define CFG_EVIO_DIAG_IN_COUNT              1
#define CFG_EVIO_FMC_COUNT                  2
#define CFG_EVIO_DIAG_OUT_COUNT             2
#define CFG_EVIO_FIREFLY_COUNT              6
#define CFG_FAN_COUNT                       4
#define CFG_SEQUENCE_RAM_CAPACITY           1024

#define CFG_EVG1_CLK_PER_RF_COINCIDENCE     666
#define CFG_EVG1_CLK_PER_BR_AR_ALIGNMENT    50616
#define CFG_EVG1_CLK_PER_BR_AR_COINCIDENCE  6327000
#define CFG_EVG1_CLK_PER_HEARTBEAT          126540000

#define CFG_EVG2_CLK_PER_RF_COINCIDENCE     667
#define CFG_EVG2_CLOCK_PER_AR_ORBIT_CLOCK   76
#define CFG_EVG2_CLOCK_PER_SR_ORBIT_CLOCK   82
#define CFG_EVG2_CLOCK_PER_ARSR_COINCIDENCE 3116
#define CFG_EVG2_CLK_PER_HEARTBEAT          124640000

// Use QSFP2 for 115 MGT quad. 0 = FMC, 1 = QSFP 2
#define CFG_MGT_FMC_OR_QSFP                1

// Disable MGT Tx:Ref alignement, as we are using
// Automatic MGT TX alignment
#define CFG_MGT_TX_REF_ALIGN               0
