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

 
 
//! \file   solutions/instaspin_motion/src/ctrl.c
//! \brief  Contains the various functions related to the controller (CTRL) object
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include <math.h>
#include <stdio.h>

// drivers


// modules


// platforms
#ifndef QEP
#include "sw/modules/ctrl/src/32b/ctrl.h"
#else
#include "sw/modules/ctrl/src/32b/ctrlQEP.h"
#endif

#include "i2c_24lc32.h"

//#include "hal.h"
#include "sw/modules/hal/boards/hvkit_rev1p1/f28x/f2806x/src/hal.h"
#include "sw/modules/user/src/32b/user.h"
#include "user_data.h"


// **************************************************************************
// the defines
// **************************************************************************
// the globals

#define EEPROM_DEVICE_ADDRESS 0x50
//#define EEPROM_Gpio_WP  GPIO_Number_51

EEPROM_Handle EEPROM_init(void *pMemory,const size_t numBytes)
{
	EEPROM_Handle eepromHandle;

	if (numBytes < sizeof(EEPROM24LC32_Obj))
		return((EEPROM_Handle)NULL);

	// assign the handle
	eepromHandle = (EEPROM_Handle)pMemory;

	return(eepromHandle);
}

void EEPROM_setup(EEPROM_Handle eepromHandle, GPIO_Handle gpioHandle, I2CMessage_Handle i2cMessageHandle)
{
	EEPROM24LC32_Obj *pEeprom;
	pEeprom = (EEPROM24LC32_Obj *) eepromHandle;

	pEeprom->gpioHandle = gpioHandle;
	pEeprom->i2cMessageHandle = i2cMessageHandle;
}

bool EEPROM_WriteByte(EEPROM_Handle eepromHandle, uint16_t u16Address, uint_least8_t u8Data)
{
	EEPROM24LC32_Obj *pEeprom = (EEPROM24LC32_Obj *) eepromHandle;
	I2C_Message *pI2cMessage = (I2C_Message *) pEeprom->i2cMessageHandle;

	pI2cMessage->u8TxNumOfBytes = 3;
	pI2cMessage->u8RxNumOfBytes = 0;
	pI2cMessage->u16SlaveAddress = EEPROM_DEVICE_ADDRESS ;
	pI2cMessage->au8TxBuffer[0] =  DATA_HiByte(u16Address);		//u16Address >> 8 ;		// HiByte
	pI2cMessage->au8TxBuffer[1] =  DATA_LoByte(u16Address);		//u16Address & 0x00ff;		// LoByte
	pI2cMessage->au8TxBuffer[2] = u8Data;

	return I2CMessage_WriteMessage(pI2cMessage);
}


bool EEPROM_Write(EEPROM_Handle eepromHandle, uint16_t u16Address, uint16_t u16Data)
{
	EEPROM24LC32_Obj *pEeprom = (EEPROM24LC32_Obj *) eepromHandle;
	I2C_Message *pI2cMessage = (I2C_Message *) pEeprom->i2cMessageHandle;

	pI2cMessage->u8TxNumOfBytes = 4;
	pI2cMessage->u8RxNumOfBytes = 0;
	pI2cMessage->u16SlaveAddress = EEPROM_DEVICE_ADDRESS ;
	pI2cMessage->au8TxBuffer[0] = DATA_HiByte(u16Address);	// u16Address >> 8 ;		// HiByte
	pI2cMessage->au8TxBuffer[1] = DATA_LoByte(u16Address);	// u16Address & 0x00ff;		// LoByte
	pI2cMessage->au8TxBuffer[2] = DATA_HiByte(u16Data);		//  u16Data >> 8;
	pI2cMessage->au8TxBuffer[3] = DATA_LoByte(u16Data);		//u16Data & 0x00ff;

	return I2CMessage_WriteMessage(pI2cMessage);
}

bool EEPROM_ReadByte(EEPROM_Handle eepromHandle, uint16_t u16Address, uint_least8_t *pu8Data)
{
	EEPROM24LC32_Obj *pEeprom = (EEPROM24LC32_Obj *) eepromHandle;
	I2C_Message *pI2cMessage = (I2C_Message *) pEeprom->i2cMessageHandle;

	//pI2cMessage->pau16DataBuffer = pu16Data;
	pI2cMessage->u16SlaveAddress = EEPROM_DEVICE_ADDRESS;
	pI2cMessage->au8TxBuffer[0] =  DATA_HiByte(u16Address);		//u16Address >> 8 ;		// HiByte
	pI2cMessage->au8TxBuffer[1] =  DATA_LoByte(u16Address);		//u16Address & 0x00ff;		// LoByte
	pI2cMessage->u8TxNumOfBytes = 2;
	pI2cMessage->u8RxNumOfBytes = 1;

	if (I2CMessage_ReadMessage(pI2cMessage))
	{
		*pu8Data = pI2cMessage->au8RxBuffer[0];
		return true;
	}
	else
	{
		*pu8Data = 0;
		return false;
	}
}


bool EEPROM_Read(EEPROM_Handle eepromHandle, uint16_t u16Address, uint16_t *pu16Data)
{
	EEPROM24LC32_Obj *pEeprom = (EEPROM24LC32_Obj *) eepromHandle;
	I2C_Message *pI2cMessage = (I2C_Message *) pEeprom->i2cMessageHandle;

	//pI2cMessage->pau16DataBuffer = pu16Data;
	pI2cMessage->u16SlaveAddress = EEPROM_DEVICE_ADDRESS;
	pI2cMessage->au8TxBuffer[0] =  DATA_HiByte(u16Address);	// u16Address >> 8 ;		// HiByte
	pI2cMessage->au8TxBuffer[1] =  DATA_LoByte(u16Address);	// u16Address & 0x00ff;		// LoByte
	pI2cMessage->u8TxNumOfBytes = 2;
	pI2cMessage->u8RxNumOfBytes = 2;

	if (I2CMessage_ReadMessage(pI2cMessage))
	{
		*pu16Data = ( pI2cMessage->au8RxBuffer[0] << 8) | pI2cMessage->au8RxBuffer[1] ;
		return true;
	}
	else
	{
		*pu16Data = 0;
		return false;
	}
}

bool EEPROM_WriteVerifyByte(EEPROM_Handle eepromHandle, uint16_t u16Address, uint_least8_t u8Data)
{

	if (EEPROM_WriteByte(eepromHandle, u16Address, u8Data))
	{
		uint_least8_t u8Temp;
		usDelay(10);
		if (EEPROM_ReadByte(eepromHandle, u16Address, &u8Temp) )
		{
			if (u8Temp == u8Data)
				return true;
		}
	}
	return false;
}


bool EEPROM_WriteVerify(EEPROM_Handle eepromHandle, uint16_t u16Address, uint16_t u16Data)
{

	if (EEPROM_Write(eepromHandle, u16Address, u16Data))
	{
		uint16_t u16Temp;
		usDelay(10);
		if (EEPROM_Read(eepromHandle, u16Address, &u16Temp) )
		{
			if (u16Temp == u16Data)
				return true;
		}
	}
	return false;
}

bool EEPROM_WritePageByte(EEPROM_Handle eepromHandle, uint16_t u16Address, uint_least8_t u8ByteNo, uint_least8_t *pu8Data)
{
	uint_least8_t i;

	EEPROM24LC32_Obj *pEeprom = (EEPROM24LC32_Obj *) eepromHandle;
	I2C_Message *pI2cMessage = (I2C_Message *) pEeprom->i2cMessageHandle;

	//GPIO_setLow(pEeprom->gpioHandle, EEPROM_Gpio_WP);

	pI2cMessage->u16SlaveAddress = EEPROM_DEVICE_ADDRESS;
	pI2cMessage->au8TxBuffer[0] = DATA_HiByte(u16Address);	// u16Address >> 8 ;		// HiByte
	pI2cMessage->au8TxBuffer[1] = DATA_LoByte(u16Address);	// u16Address & 0x00ff;		// LoByte
	for (i=0; i< u8ByteNo; i++)
		pI2cMessage->au8TxBuffer[2+i]= *pu8Data;

	pI2cMessage->u8TxNumOfBytes = u8ByteNo+2;
	pI2cMessage->u8RxNumOfBytes = 0;

	return I2CMessage_WriteMessage(pI2cMessage);
}

bool EEPROM_WritePage(EEPROM_Handle eepromHandle, uint16_t u16Address, uint_least8_t u8WordNo, uint16_t *pu16Data)
{
	uint_least8_t i;

	EEPROM24LC32_Obj *pEeprom = (EEPROM24LC32_Obj *) eepromHandle;
	I2C_Message *pI2cMessage = (I2C_Message *) pEeprom->i2cMessageHandle;

	//GPIO_setLow(pEeprom->gpioHandle, EEPROM_Gpio_WP);

	pI2cMessage->u16SlaveAddress = EEPROM_DEVICE_ADDRESS;
	pI2cMessage->au8TxBuffer[0] = DATA_HiByte(u16Address);	// u16Address >> 8 ;		// HiByte
	pI2cMessage->au8TxBuffer[1] = DATA_LoByte(u16Address);	// u16Address & 0x00ff;		// LoByte
	for (i=0; i< u8WordNo << 1; i+=2)
	{
		pI2cMessage->au8TxBuffer[2+i]= *pu16Data;
		pI2cMessage->au8TxBuffer[3+i]= *pu16Data++;
	}

	pI2cMessage->u8TxNumOfBytes = (u8WordNo << 1)+2;
	pI2cMessage->u8RxNumOfBytes = 0;

	return I2CMessage_WriteMessage(pI2cMessage);
}

bool EEPROM_ReadPageByte(EEPROM_Handle eepromHandle, uint16_t u16Address, uint_least8_t u8ByteNo, uint_least8_t *pu8Data)
{
//	uint_least8_t u8Cnt;
	EEPROM24LC32_Obj *pEeprom = (EEPROM24LC32_Obj *) eepromHandle;
	I2C_Message *pI2cMessage = (I2C_Message *) pEeprom->i2cMessageHandle;

	//pI2cMessage->pau16DataBuffer = pu16Data;
	pI2cMessage->u16SlaveAddress = EEPROM_DEVICE_ADDRESS;
	pI2cMessage->au8TxBuffer[0] = DATA_HiByte(u16Address);	//( u16Address >> 8 ;		// HiByte
	pI2cMessage->au8TxBuffer[1] = DATA_LoByte(u16Address);	// u16Address & 0x00ff;		// LoByte
	pI2cMessage->u8TxNumOfBytes = 2;
	pI2cMessage->u8RxNumOfBytes = u8ByteNo;

	if (I2CMessage_ReadMessage(pI2cMessage))
	{
		uint_least8_t i;
		for (i=0; i< u8ByteNo; i++)
		{
			*pu8Data = pI2cMessage->au8RxBuffer[i] ;
			pu8Data++;
		}
		return true;
	}
	return false;

}


bool EEPROM_ReadPage(EEPROM_Handle eepromHandle, uint16_t u16Address, uint_least8_t u8WordNo, uint16_t *pu16Data)
{
//	uint_least8_t u8Cnt;
	EEPROM24LC32_Obj *pEeprom = (EEPROM24LC32_Obj *) eepromHandle;
	I2C_Message *pI2cMessage = (I2C_Message *) pEeprom->i2cMessageHandle;

	//pI2cMessage->pau16DataBuffer = pu16Data;
	pI2cMessage->u16SlaveAddress = EEPROM_DEVICE_ADDRESS;
	pI2cMessage->au8TxBuffer[0] =  DATA_HiByte(u16Address);	//( u16Address >> 8 ;		// HiByte
	pI2cMessage->au8TxBuffer[1] =  DATA_LoByte(u16Address);	//u16Address & 0x00ff;		// LoByte
	pI2cMessage->u8TxNumOfBytes = 2;
	pI2cMessage->u8RxNumOfBytes = u8WordNo << 1;

	if (I2CMessage_ReadMessage(pI2cMessage))
	{
		uint_least8_t i;
		for (i=0; i< u8WordNo; i+=2)
		{
			*pu16Data = (pI2cMessage->au8RxBuffer[i] << 8) || pI2cMessage->au8RxBuffer[i+1];
			pu16Data++;
		}
		return true;
	}
	return false;

}
// end of file
