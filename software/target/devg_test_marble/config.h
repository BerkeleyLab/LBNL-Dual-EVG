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
//#define CFG_EVG1_CLK_PER_BR_AR_ALIGNMENT 46208 /* Clock tree (N / E) */
#define CFG_EVG1_CLK_PER_RF_COINCIDENCE     608
#define CFG_EVG2_CLK_PER_RF_COINCIDENCE     609
//#define CFG_EVG1_CLK_PER_HEARTBEAT          121296000
#define CFG_EVG2_CLK_PER_HEARTBEAT          124640000
#define CFG_EVG2_CLOCK_PER_ARSR_COINCIDENCE 3116

// FIXME: Alternate values to work with ALS SR
#define CFG_EVG1_CLK_PER_BR_AR_ALIGNMENT 10250
#define CFG_EVG1_CLK_PER_HEARTBEAT          124640000

// Use QSFP2 for 115 MGT quad. 0 = FMC, 1 = QSFP 2
#define CFG_MGT_FMC_OR_QSFP                1

// Disable MGT Tx:Ref alignement, as we are using
// Automatic MGT TX alignment
#define CFG_MGT_TX_REF_ALIGN               0
