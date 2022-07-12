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
#include "fmcIO.h"
#include "gpio.h"
#include "util.h"

#define DIAG_CSR_AUX_SWITCH_N               0x8000
#define DIAG_CSR_DIAG_OUTPUT_SELECT_MASK    0x300
#define DIAG_CSR_DIAG_OUTPUT_SELECT_SHIFT   8
#define DIAG_CSR_DIAG_OUTPUT_MASK           0x30
#define DIAG_CSR_DIAG_OUTPUT_SHIFT          4
#define DIAG_CSR_DIAG_INPUT_MASK            0x3

/*
 * Merge diagnostic inputs and hardware triggers
 */
int
fmcIOfetchStatus(uint32_t *ap)
{
    uint32_t evg1, evg2;
    evg1 = (GPIO_READ(GPIO_IDX_FMC1_DIAGNOSTIC) & DIAG_CSR_DIAG_INPUT_MASK) |
           ((GPIO_READ(GPIO_IDX_EVG_1_HW_CSR) & FMCIO_CSR_HW_STATUS_MASK)
                                              >> (FMCIO_CSR_HW_STATUS_SHIFT-8));
    evg2 = (GPIO_READ(GPIO_IDX_FMC2_DIAGNOSTIC) & DIAG_CSR_DIAG_INPUT_MASK) |
           ((GPIO_READ(GPIO_IDX_EVG_2_HW_CSR) & FMCIO_CSR_HW_STATUS_MASK)
                                              >> (FMCIO_CSR_HW_STATUS_SHIFT-8));
    *ap++ = (evg2 << 16) | evg1;
    return 1;
}

void
fmcIOsetDiagnosticOutputs(unsigned int idx, uint32_t value)
{
    switch(idx) {
    case 0: GPIO_WRITE(GPIO_IDX_FMC1_DIAGNOSTIC, value); break;
    case 1: GPIO_WRITE(GPIO_IDX_FMC2_DIAGNOSTIC, value); break;
    }
}
