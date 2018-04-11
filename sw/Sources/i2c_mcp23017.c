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


// drivers


// modules


// platforms
#include "sw/modules/ctrl/src/32b/ctrlQEP.h"
#include "sw/modules/hal/boards/hvkit_rev1p1/f28x/f2806x/src/hal.h"
//#include "hal.h"
#include "sw/modules/user/src/32b/user.h"
#include "i2c_mcp23017.h"
#include "user_Data.h"



// **************************************************************************
// the defines
// **************************************************************************
// the globals

#define IOEXPAND_DEVICE_ADDRESS 0x20

#define IOEXPAND_IODIRA		0x00
#define IOEXPAND_IODIRB		0x01
#define IOEXPAND_PULLUPA	0x0c
#define IOEXPAND_PULLUPB	0x0d
#define IOEXPAND_GPIOA		0x12
#define IOEXPAND_GPIOB		0x13
#define IOEXPAND_OLATA		0x14
#define IOEXPAND_OLATB		0x15


IOEXPAND_Handle IOEXPAND_init(void *pMemory,const size_t numBytes)
{
	IOEXPAND_Handle ioexpandHandle;

	if (numBytes < sizeof(IOEXPAND23017_Obj))
		return((IOEXPAND_Handle)NULL);

	// assign the handle
	ioexpandHandle = (IOEXPAND_Handle)pMemory;
	return(ioexpandHandle);
}

void IOEXPAND_setup(IOEXPAND_Handle ioexpandHandle, I2CMessage_Handle i2cMessageHandle, uint16_t *pu16InputTerms, uint_least8_t *pu8OutputTerms)
{
	IOEXPAND23017_Obj *pIOexpand;
	//uint16_t u16Data;
	pIOexpand = (IOEXPAND23017_Obj *) ioexpandHandle;
	pIOexpand->i2cMessageHandle = i2cMessageHandle;
	pIOexpand->pu16InputTerms = pu16InputTerms;
	pIOexpand->pu8OutputTerms = pu8OutputTerms;


	IOEXPAND_WriteRegister(ioexpandHandle, IOEXPAND_IODIRA,
						IOEXAND_INPUT_Bit15 | IOEXAND_INPUT_Bit14 | IOEXAND_INPUT_Bit13 | IOEXAND_INPUT_Bit12 |
						IOEXAND_INPUT_Bit11 | IOEXAND_INPUT_Bit10 | IOEXAND_INPUT_Bit9  | IOEXAND_INPUT_Bit8 |
						IOEXAND_OUTPUT_Bit7 | IOEXAND_OUTPUT_Bit6 | IOEXAND_OUTPUT_Bit5 | IOEXAND_OUTPUT_Bit4 |
						IOEXAND_INPUT_Bit3  | IOEXAND_INPUT_Bit2  | IOEXAND_INPUT_Bit1  | IOEXAND_INPUT_Bit0);
	//IOEXPAND_ReadRegister(ioexpandHandle, 0x0a ,&u16Data);


	//IOEXPAND_WriteRegister(ioexpandHandle, IOEXPAND_IODIRB,
	//			IOEXAND_INPUT_Bit0 | IOEXAND_INPUT_Bit1 | IOEXAND_INPUT_Bit2 | IOEXAND_INPUT_Bit3 |
	//			IOEXAND_OUTPUT_Bit4 | IOEXAND_OUTPUT_Bit5 | IOEXAND_OUTPUT_Bit6 | IOEXAND_OUTPUT_Bit7);

	IOEXPAND_WriteRegister(ioexpandHandle,IOEXPAND_PULLUPA, 0x0000);
						//IOEXAND_PullDisable_Bit15 | IOEXAND_PullDisable_Bit14 | IOEXAND_PullDisable_Bit13 | IOEXAND_PullDisable_Bit12 |
						//IOEXAND_PullDisable_Bit11 | IOEXAND_PullDisable_Bit10 | IOEXAND_PullDisable_Bit9 | IOEXAND_PullDisable_Bit8 |
						//IOEXAND_PullUp_Bit3 | IOEXAND_PullUp_Bit2 | IOEXAND_PullUp_Bit1 |IOEXAND_PullUp_Bit0  );

	/*IOEXPAND_WriteRegister(ioexpandHandle, IOEXPAND_PULLUPB,
						IOEXAND_PullDisable_Bit0 | IOEXAND_PullDisable_Bit1 | IOEXAND_PullDisable_Bit2 |
						IOEXAND_PullDisable_Bit3 | IOEXAND_PullDisable_Bit4 | IOEXAND_PullDisable_Bit5 |
						IOEXAND_PullDisable_Bit6 | IOEXAND_PullDisable_Bit7);*/

	//IOEXPAND_setLow(ioexpandHandle,IOEXPAND_Term25 |IOEXPAND_Term26 | IOEXPAND_Term10 |IOEXPAND_Term18);
	IOEXPAND_getInputs(ioexpandHandle);

	//bSetupFlag = true;
}



bool IOEXPAND_WriteRegister(IOEXPAND_Handle ioexpandHandle, uint_least8_t u8RegAdd, uint16_t u16Value)
{
	IOEXPAND23017_Obj *pIOexpand = (IOEXPAND23017_Obj *) ioexpandHandle;
	I2C_Message *pI2cMessage = (I2C_Message *) pIOexpand->i2cMessageHandle;

	pI2cMessage->u16SlaveAddress = IOEXPAND_DEVICE_ADDRESS;
	pI2cMessage->au8TxBuffer[0] =  u8RegAdd & 0x00ff;
	pI2cMessage->au8TxBuffer[1] =  u16Value >> 8;
	pI2cMessage->au8TxBuffer[2] =  u16Value & 0x00ff;
	pI2cMessage->u8TxNumOfBytes = 3;
	pI2cMessage->u8RxNumOfBytes = 0;

	return I2CMessage_WriteMessage(pI2cMessage);
}



bool IOEXPAND_ReadRegister(IOEXPAND_Handle ioexpandHandle,uint_least8_t u8RegAdd, uint16_t *pu16Value)
{

	IOEXPAND23017_Obj *pIOexpand = (IOEXPAND23017_Obj *) ioexpandHandle;
	I2C_Message *pI2cMessage = (I2C_Message *) pIOexpand->i2cMessageHandle;

	//pI2cMessage->pau16DataBuffer = pu16Value;
	pI2cMessage->u16SlaveAddress = IOEXPAND_DEVICE_ADDRESS;
	pI2cMessage->au8TxBuffer[0] =  u8RegAdd & 0x00ff;

	pI2cMessage->u8TxNumOfBytes = 1;
	pI2cMessage->u8RxNumOfBytes = 2;

	if (I2CMessage_ReadMessage(pI2cMessage))
	{
		*pu16Value = ( pI2cMessage->au8RxBuffer[0] << 8) | pI2cMessage->au8RxBuffer[1] ;
		return true;
	}
	else
	{
		*pu16Value = 0;
		return false;
	}


}


bool IOEXPAND_setHigh(IOEXPAND_Handle ioexpandHandle,const uint_least8_t ioexpanderTerm)
{
	IOEXPAND23017_Obj *pIOexpand;
	pIOexpand = (IOEXPAND23017_Obj *) ioexpandHandle;

	g_u8OutputTerms &= ~(IOEXPAND_Term25 |IOEXPAND_Term26 | IOEXPAND_Term10 |IOEXPAND_Term18);
	g_u8OutputTerms |= ioexpanderTerm;
	return IOEXPAND_WriteRegister(ioexpandHandle,IOEXPAND_OLATA, *(pIOexpand->pu8OutputTerms));
}

bool IOEXPAND_setLow(IOEXPAND_Handle ioexpandHandle,const uint_least8_t ioexpanderTerm)
{
	IOEXPAND23017_Obj *pIOexpand;
	pIOexpand = (IOEXPAND23017_Obj *) ioexpandHandle;
	g_u8OutputTerms &= (~ioexpanderTerm);
	return IOEXPAND_WriteRegister(ioexpandHandle,IOEXPAND_OLATA, *(pIOexpand->pu8OutputTerms));
}


bool IOEXPAND_getInputs(IOEXPAND_Handle ioexpandHandle)
{
    IOEXPAND23017_Obj *pIOexpand;
    pIOexpand = (IOEXPAND23017_Obj *) ioexpandHandle;
    return IOEXPAND_ReadRegister(ioexpandHandle,IOEXPAND_GPIOA, pIOexpand->pu16InputTerms);
}


void IOEXPAND_run(IOEXPAND_Handle ioexpandHandle)
{
	IOEXPAND_getInputs(ioexpandHandle);

}


/*uint_least8_t IOEXPAND_InputTerminals(IOEXPAND_Handle ioexpandHandle)
{
	IOEXPAND23017_Obj *pIOexpand;
    pIOexpand = (IOEXPAND23017_Obj *) ioexpandHandle;

    return pIOexpand->u16InputTerms >> 8;

}*/

/*uint_least8_t IOEXPAND_Capacity(IOEXPAND_Handle ioexpandHandle)
{
	IOEXPAND23017_Obj *pIOexpand;
    pIOexpand = (IOEXPAND23017_Obj *) ioexpandHandle;
    return pIOexpand->u16InputTerms & 0x000f;

}*/

// end of file
