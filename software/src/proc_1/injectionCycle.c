/*
 * Copyright 2020, Lawrence Berkeley National Laboratory
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS
 * AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include "injectionCycle.h"
#include "gpio.h"
#include "util.h"

#define MINIMUM_INJECTION_PERIOD_MILLISECONDS  1000
#define MAXIMUM_INJECTION_PERIOD_MILLISECONDS  ((1UL << 16) - 1)

#define CSR_W_SET_CYCLE_MILLISECONDS    (1UL << 31)
#define CSR_W_MANUAL_TRIGGER            (1UL << 7)
#define CSR_W_DISABLE_TIMED_CYCLES      (1UL << 1)
#define CSR_W_ENABLE_TIMED_CYCLES       (1UL << 0)

static unsigned int baseInterval = MINIMUM_INJECTION_PERIOD_MILLISECONDS;
static unsigned int maxExtension = MAXIMUM_INJECTION_PERIOD_MILLISECONDS -
                                          MINIMUM_INJECTION_PERIOD_MILLISECONDS;

void
injectionCycleEnable(int enable)
{
    GPIO_WRITE(GPIO_IDX_INJECTION_CYCLE_CSR, enable ?
                        CSR_W_ENABLE_TIMED_CYCLES : CSR_W_DISABLE_TIMED_CYCLES);
}

void
injectionCycleManualTrigger(void)
{
    GPIO_WRITE(GPIO_IDX_INJECTION_CYCLE_CSR, CSR_W_MANUAL_TRIGGER);
}

void
injectionCycleExtendInterval(int milliseconds)
{
    if (milliseconds < 0) {
        milliseconds = 0;
    }
    else if (milliseconds > maxExtension) {
        milliseconds = maxExtension;
    }
    milliseconds += baseInterval;
    GPIO_WRITE(GPIO_IDX_INJECTION_CYCLE_CSR, CSR_W_SET_CYCLE_MILLISECONDS |
                                                            (milliseconds - 2));
}

void
injectionCycleSetBaseInterval(int milliseconds)
{
    if (milliseconds < MINIMUM_INJECTION_PERIOD_MILLISECONDS) {
        milliseconds = MINIMUM_INJECTION_PERIOD_MILLISECONDS;
    }
    else if (milliseconds > MAXIMUM_INJECTION_PERIOD_MILLISECONDS) {
        milliseconds = MAXIMUM_INJECTION_PERIOD_MILLISECONDS;
    }
    baseInterval = milliseconds;
    maxExtension = MAXIMUM_INJECTION_PERIOD_MILLISECONDS - baseInterval;
    injectionCycleExtendInterval(0);
}

int
injectionCycleFetchStatus(uint32_t *ap)
{
    int idx = 0;
    ap[idx++] = GPIO_READ(GPIO_IDX_INJECTION_CYCLE_CSR);
    return idx;
}
