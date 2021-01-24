/*
 * iic_lib.h
 *
 *  Created on: 18 Oca 2021
 *      Author: AdminPC
 */

#ifndef IIC_LIB_H_
#define IIC_LIB_H_


bool IIC_OpenComm(void);
bool IIC_writeReg(int device_ID, int addr, uint8_t val);
bool IIC_readReg(int device_ID, int addr, int no_of_bytes, char *buf);
void IIC_CloseComm(void);


#endif /* IIC_LIB_H_ */
