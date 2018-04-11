/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
#ifndef _I2C_24LC32_H_
#define _I2C_24LC32_H_

//! \file   solutions/instaspin_motion/src/ctrl.h
//! \brief Contains the public interface, object and function definitions for 
//!        various functions related to the CTRL object 
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "sw/modules/types/src/types.h"
#include "sw/drivers/i2c/src/32b/f28x/f2806x/i2c.h"
#include "sw/drivers/gpio/src/32b/f28x/f2806x/gpio.h"
#include "i2c_message.h"


//#include "hal_obj.h"


//!
//!
//! \defgroup CTRL CTRL
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif

typedef struct _EEPROM24LC32_Obj_
{
	//I2C_Handle i2cHandle;			//!< quadrature pulse encoder handle
	GPIO_Handle gpioHandle;
	I2CMessage_Handle i2cMessageHandle;
} EEPROM24LC32_Obj;


typedef struct _EEPROM24LC32_Obj_ *EEPROM_Handle;


extern EEPROM_Handle EEPROM_init(void *pMemory,const size_t numBytes);
extern void EEPROM_setup(EEPROM_Handle eepromHandle,  GPIO_Handle gpioHandle, I2CMessage_Handle i2cMessageHandle );

extern bool EEPROM_WriteByte(EEPROM_Handle eepromHandle, uint16_t u16Address, uint_least8_t u8Data);
extern bool EEPROM_Write(EEPROM_Handle eepromHandle, uint16_t u16Address, uint16_t u16Data);
extern bool EEPROM_ReadByte(EEPROM_Handle eepromHandle, uint16_t u16Address, uint_least8_t *pu8Data);
extern bool EEPROM_Read(EEPROM_Handle eepromHandle, uint16_t u16Address, uint16_t *pu16Data);
extern bool EEPROM_WriteVerifyByte(EEPROM_Handle eepromHandle, uint16_t u16Address, uint_least8_t u8Data);
extern bool EEPROM_WriteVerify(EEPROM_Handle eepromHandle, uint16_t u16Address, uint16_t u16Data);
extern bool EEPROM_WritePageByte(EEPROM_Handle eepromHandle, uint16_t u16Address, uint_least8_t u8ByteNo, uint_least8_t *pu8Data);
extern bool EEPROM_WritePage(EEPROM_Handle eepromHandle, uint16_t u16Address, uint_least8_t u8WordNo, uint16_t *pu16Data);
extern bool EEPROM_ReadPageByte(EEPROM_Handle eepromHandle, uint16_t u16Address, uint_least8_t u8ByteNo, uint_least8_t *pu8Data);
extern bool EEPROM_ReadPage(EEPROM_Handle eepromHandle, uint16_t u16Address, uint_least8_t u8WordNo, uint16_t *pu16Data);



#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _I2C_24LC32_H_ definition




