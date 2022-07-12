/*
 * Xilinx AXI IIC to FMC slot 1 components
 * EVIO or EVRIO
 */
#ifndef _IIC_FMC_H_
#define _IIC_FMC_H_

void iicFMCinit(void);
int iicFMCsend(int address7, int subaddress, const unsigned char *buf, int len);
int iicFMCrecv(int address7, int subaddress, unsigned char *buf, int len);
int iicFMCgetGPO(void);
void iicFMCsetGPO(int gpo);

#endif /* _IIC_FMC_H_ */
