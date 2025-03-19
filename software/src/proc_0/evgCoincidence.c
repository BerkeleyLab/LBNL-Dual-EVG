/*
 * Event generator coincidence detection
 * Can take a while to run so handled by processor 0.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <xparameters.h>
#include "evg.h"
#include "evgCoincidence.h"
#include "gpio.h"
#include "sharedMemory.h"
#include "util.h"

/* CSR read */

#define DATA_HIST_SIZE              10
#define DATA_HIST_SHIFT             0
#define DATA_HIST_MASK              REG_GEN_MASK(DATA_HIST_SHIFT, DATA_HIST_SIZE)
#define DATA_HIST_R(reg)            REG_GEN_READ(reg, DATA_HIST_SHIFT, DATA_HIST_SIZE)

#define ADDRESS_RB_SIZE             10
#define ADDRESS_RB_SHIFT            DATA_HIST_SIZE
#define ADDRESS_RB_MASK             REG_GEN_MASK(ADDRESS_RB_SHIFT, ADDRESS_RB_SIZE)
#define ADDRESS_RB_R(reg)           REG_GEN_READ(reg, ADDRESS_RB_SHIFT, ADDRESS_RB_SIZE)

#define MUX_SEL_RB_SIZE             1
#define MUX_SEL_RB_SHIFT            24
#define MUX_SEL_RB_MASK             REG_GEN_MASK(MUX_SEL_RB_SHIFT, MUX_SEL_RB_SIZE)
#define MUX_SEL_RB_R(reg)           REG_GEN_READ(reg, MUX_SEL_RB_SHIFT, MUX_SEL_RB_SIZE)

#define BUSY_STATUS_SIZE            1
#define BUSY_STATUS_SHIFT           31
#define BUSY_STATUS_MASK            REG_GEN_MASK(BUSY_STATUS_SHIFT, BUSY_STATUS_SIZE)
#define CSR_R_BUSY                  BUSY_STATUS_MASK

/* CSR write */

#define ADDRESS_WR_SIZE             10
#define ADDRESS_WR_SHIFT            0
#define ADDRESS_WR_MASK             REG_GEN_MASK(ADDRESS_WR_SHIFT, ADDRESS_WR_SIZE)
#define ADDRESS_WR_W(value)         REG_GEN_WRITE(value, ADDRESS_WR_SHIFT, ADDRESS_WR_SIZE)

#define MUX_SEL_WR_SIZE             1
#define MUX_SEL_WR_SHIFT            24
#define MUX_SEL_WR_MASK             REG_GEN_MASK(MUX_SEL_WR_SHIFT, MUX_SEL_WR_SIZE)
#define MUX_SEL_WR_W(value)         REG_GEN_WRITE(value, MUX_SEL_WR_SHIFT, MUX_SEL_WR_SIZE)

#define REALIGN_WR_SIZE             1
#define REALIGN_WR_SHIFT            29
#define REALIGN_WR_MASK             REG_GEN_MASK(REALIGN_WR_SHIFT, REALIGN_WR_SIZE)
#define CSR_W_REALIGN               REALIGN_WR_MASK

#define SET_COINCIDENCE_WR_SIZE     1
#define SET_COINCIDENCE_WR_SHIFT    30
#define SET_COINCIDENCE_WR_MASK     REG_GEN_MASK(SET_COINCIDENCE_WR_SHIFT, SET_COINCIDENCE_WR_SIZE)
#define CSR_W_SET_COINCIDENCE       SET_COINCIDENCE_WR_MASK

#define START_WR_SIZE               1
#define START_WR_SHIFT              31
#define START_WR_MASK               REG_GEN_MASK(START_WR_SHIFT, START_WR_SIZE)
#define CSR_W_START                 START_WR_MASK

static struct evgInfo {
    uint16_t evgIndex;
    uint16_t csrIndex;
    uint16_t samplesPerCycle;
    uint16_t sampledPerCycle;
    int16_t  addressOfRisingEdge[EVG_COINCIDENCE_COUNT];
    int16_t  jitter[EVG_COINCIDENCE_COUNT];
    int16_t  oldAddressOfRisingEdge[EVG_COINCIDENCE_COUNT];
    int8_t   acquiring;
} evgs[EVG_COUNT] = {
    { .evgIndex        = 0,
      .csrIndex        = GPIO_IDX_EVG_1_COINC_CSR,
      .samplesPerCycle = CFG_EVG2_CLK_PER_RF_COINCIDENCE,
      .sampledPerCycle = CFG_EVG1_CLK_PER_RF_COINCIDENCE,
    },
    { .evgIndex        = 1,
      .csrIndex        = GPIO_IDX_EVG_2_COINC_CSR,
      .samplesPerCycle = CFG_EVG1_CLK_PER_RF_COINCIDENCE,
      .sampledPerCycle = CFG_EVG2_CLK_PER_RF_COINCIDENCE,
    }
};

// Only call this when not capturing a new histogram
static int
readCoincidenceHist(struct evgInfo *evgp, int muxSel, int address)
{
    uint32_t whenStarted = MICROSECONDS_SINCE_BOOT();
    uint32_t reg;
    int pass = 0;
    int timeout = 0;

    GPIO_WRITE(evgp->csrIndex, MUX_SEL_WR_W(muxSel) | ADDRESS_WR_W(address));
    reg = GPIO_READ(evgp->csrIndex);

    while ((ADDRESS_RB_R(reg) != address) ||
            (MUX_SEL_RB_R(reg) != muxSel)) {
        pass++;

        if ((MICROSECONDS_SINCE_BOOT() - whenStarted) > 50) {
            warn("Coincidence histogram read timeout");
            timeout = 1;

            if (debugFlags & DEBUGFLAG_SHOW_COINC_ADDR_RB) {
                printf("Coincidence readout address %d:%d muxSel %d:%d\n",
                        address, ADDRESS_RB_R(reg),
                        muxSel, MUX_SEL_RB_R(reg));
            }

            break;
        }

        reg = GPIO_READ(evgp->csrIndex);
    }

   if (timeout) {
       return 0;
   }

    return DATA_HIST_R(reg);
}

/*
 * Find midpoint of rising edge
 */
static void
findCoincidence(struct evgInfo *evgp, int inputIndex)
{
    const int loopLimit = (evgp->samplesPerCycle * 3) / 2;
    const int debounceLimit = evgp->samplesPerCycle / 5;
    int consecutiveZeroCount = 0;
    int indexOfFirstNonZero = -1;
    int i;

    evgp->addressOfRisingEdge[inputIndex] = -1;
    evgp->jitter[inputIndex] = -1;
    for (i = 0 ; i < loopLimit ; i++) {
        int address = i % evgp->samplesPerCycle;
        int n = readCoincidenceHist(evgp, inputIndex, address);
        if (n == 0) {
            consecutiveZeroCount++;
        }
        else {
            if ((consecutiveZeroCount > debounceLimit)
             && (indexOfFirstNonZero < 0)) {
                indexOfFirstNonZero = i;
            }
            if (indexOfFirstNonZero >= 0) {
                if ((n >= (DATA_HIST_MASK / 2))
                 && (evgp->addressOfRisingEdge[inputIndex] < 0)) {
                    evgp->addressOfRisingEdge[inputIndex] = address;
                }
                if (n == DATA_HIST_MASK) {
                    evgp->jitter[inputIndex] = i - indexOfFirstNonZero;
                    break;
                }
            }
            consecutiveZeroCount = 0;
        }
    }
}

static void
evgCoincidence(struct evgInfo *evgp)
{
    int i;
    int phaseOffset, phaseError;
    for (i = 0 ; i < EVG_COINCIDENCE_COUNT ; i++) {
        int diff;
        const int coincidenceIndex = (evgp->evgIndex * EVG_COINCIDENCE_COUNT) + i;
        findCoincidence(evgp, i);
        sharedMemory->coincidence[coincidenceIndex] = (evgp->jitter[i] << 16) |
                                        (evgp->addressOfRisingEdge[i] & 0xFFFF);
        diff = (evgp->addressOfRisingEdge[i] - evgp->oldAddressOfRisingEdge[i] +
                                 evgp->samplesPerCycle) % evgp->samplesPerCycle;
        if (diff > (evgp->samplesPerCycle / 2)) {
            diff -= evgp->samplesPerCycle;
        }
        if (abs(diff) > (evgp->samplesPerCycle / 40)) {
            sharedMemory->isAligned = 0;
        }
    }
    phaseOffset = (evgp->addressOfRisingEdge[1] - evgp->addressOfRisingEdge[0] +
                                 evgp->samplesPerCycle) % evgp->samplesPerCycle;
    phaseError = (phaseOffset -
                  sharedMemory->systemParameters.pllPhaseShift[evgp->evgIndex] +
                                 evgp->samplesPerCycle) % evgp->samplesPerCycle;
    if (phaseOffset > (evgp->samplesPerCycle / 2)) {
        phaseOffset -= evgp->samplesPerCycle;
    }
    if (phaseError > (evgp->samplesPerCycle / 2)) {
        phaseError -= evgp->samplesPerCycle;
    }
    sharedMemory->pllPhaseOffset[evgp->evgIndex] = phaseOffset;
    sharedMemory->pllPhaseError[evgp->evgIndex] = phaseError;
}

void
evgCoincidenceCrank(void)
{
    struct evgInfo *evgp;
    static int active;
    static int reportedAlignTimeout;
    static uint32_t whenWarned;
    static uint32_t whenStarted;

    if (sharedMemory->requestAlignment) {
        int good = 1, a;
        for (evgp = evgs ; evgp < &evgs[EVG_COUNT] ; evgp++) {
            if ((a = evgp->addressOfRisingEdge[0]) >= 0) {
                /* Account for clock domain crossing delay */
                a = (a - 2 + evgp->samplesPerCycle) % evgp->samplesPerCycle;
                GPIO_WRITE(evgp->csrIndex, CSR_W_SET_COINCIDENCE | ADDRESS_WR_W(a));
            }
            else {
                good = 0;
            }
        }
        if (good) {
            microsecondSpin(10);
            for (evgp = evgs ; evgp < &evgs[EVG_COUNT] ; evgp++) {
                for (int i = 0 ; i < EVG_COINCIDENCE_COUNT ; i++) {
                    evgp->oldAddressOfRisingEdge[i] =
                                                   evgp->addressOfRisingEdge[i];
                }
                GPIO_WRITE(evgp->csrIndex, CSR_W_REALIGN);
            }
            reportedAlignTimeout = 0;
            sharedMemory->wasAligned = sharedMemory->isAligned = 1;
        }
        sharedMemory->requestAlignment = 0;
    }
    if (active) {
        for (evgp = evgs ; evgp < &evgs[EVG_COUNT] ; evgp++) {
            if (evgp->acquiring) {
                if (!(GPIO_READ(evgp->csrIndex) & CSR_R_BUSY)) {
                    evgCoincidence(evgp);
                    evgp->acquiring = 0;
                }
                if ((MICROSECONDS_SINCE_BOOT() - whenStarted) > 50000) {
                    warn("Coincidence test timeout");
                    evgp->acquiring = 0;
                }
                return;
            }
        }
        active = 0;
        sharedMemory->requestCoincidenceMeasurement = 0;
    }
    else if (sharedMemory->requestCoincidenceMeasurement) {
        if (sharedMemory->wasAligned && !sharedMemory->isAligned) {
            if (!reportedAlignTimeout) {
                reportedAlignTimeout = 1;
                warn("Alignment lost");
                evgCoincidenceShow(0);
            }

            if (!(debugFlags & DEBUGFLAG_FORCE_MEAS_ON_LOA)) {
                return;
            }
        }

        for (evgp = evgs ; evgp < &evgs[EVG_COUNT] ; evgp++) {
            GPIO_WRITE(evgp->csrIndex, CSR_W_START);
            evgp->acquiring = 1;
        }
        active = 1;
        whenStarted = MICROSECONDS_SINCE_BOOT();
    }
}

void
evgCoincidenceShow(int showData)
{
    struct evgInfo *evgp;
    int a, i;

    for (evgp = evgs ; evgp < &evgs[EVG_COUNT] ; evgp++) {
        // Make sure the coincidence recorder is not acquiring and not busy.
        // Otherwise we would mess up the coincidence measurement
        while (evgp->acquiring) {
            evgCoincidenceCrank();
        }

        if (showData) {
            for (a = 0 ; a < evgp->samplesPerCycle ; a++) {
                printf("%d", a);
                for (i = 0 ; i < EVG_COINCIDENCE_COUNT ; i++) {
                    int n = readCoincidenceHist(evgp, i, a);
                    printf(" %d", n);
                }
                printf("\n");
            }
        }
        for (i = 0 ; i < EVG_COINCIDENCE_COUNT ; i++) {
            findCoincidence(evgp, i);
            printf("EVG:%d Input:%d Coinc %d Old Coinc %d (jitter %d)\n", evgp->evgIndex + 1,
                              i, evgp->addressOfRisingEdge[i], evgp->oldAddressOfRisingEdge[i],
                              evgp->jitter[i]);
        }
        printf("EVG:%d Tx:Ref %d\n", evgp->evgIndex + 1,
                                  sharedMemory->pllPhaseOffset[evgp->evgIndex]);
    }
}

void
evgCoincidenceInit(void)
{
    struct evgInfo *evgp;
    int i;
    for (evgp = evgs ; evgp < &evgs[EVG_COUNT] ; evgp++) {
        for (i = 0 ; i < EVG_COINCIDENCE_COUNT ; i++) {
            evgp->addressOfRisingEdge[i] = -1;
        }
    }

    sharedMemory->wasAligned = sharedMemory->isAligned = 0;
}
