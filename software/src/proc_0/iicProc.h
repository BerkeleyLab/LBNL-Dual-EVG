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

/*
 * I2C I/O from processor
 */

#ifndef _IICPROC_H_
#define _IICPROC_H_

/*
 * IIC 7-bit address
 */
#define IIC_MUX_ADDRESS 0x70

/*
 * IIC switch ports
 */
#define IIC_MUX_PORT_FMC1           0
#define IIC_MUX_PORT_FMC2           1
#define IIC_MUX_PORT_MGTCLK_SWITCH  2
#define IIC_MUX_PORT_QSFP1          4
#define IIC_MUX_PORT_QSFP2          5
#define IIC_MUX_PORT_PORT_EXPANDER  6

void iicProcInit(void);
const char *iicProcFMCproductType(int fmcIndex);

void iicProcTakeControl(void);
void iicProcRelinquishControl(void);

int iicProcSetMux(int port);
int iicProcRead(int device, int subaddress, uint8_t *buf, int n);
int iicProcWrite(int device, int subaddress, uint8_t *buf, int n);
int iicProcReadFMC_EEPROM(int fmcIndex, uint8_t *buf, int n);
int iicProcWriteFMC_EEPROM(int fmcIndex, uint8_t *buf, int n);
void iicProcScan(void);

#endif /* _IICPROC_H_ */
