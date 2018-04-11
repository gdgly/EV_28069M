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

#include "i2c_message.h"
#include "user_data.h"
#include "sw/modules/user/src/32b/user.h"
#include "sw/modules/usDelay/src/32b/usDelay.h"

#define IC2_TIMEOUT 10


I2CMessage_Handle I2CMessage_init(void *pMemory,const size_t numBytes)
{
	I2CMessage_Handle i2cMessaageHandle;

	if (numBytes < sizeof(I2C_Message))
		return((I2CMessage_Handle)NULL);

	// assign the handle
	i2cMessaageHandle = (I2CMessage_Handle)pMemory;
	return(i2cMessaageHandle);
}


void I2CMessage_setup(I2CMessage_Handle i2cMessageHandle, I2C_Handle i2cHandle)
{
	I2C_Message *pI2cMessaage;
	pI2cMessaage = (I2C_Message *) i2cMessageHandle;

	pI2cMessaage->i2cHandle = i2cHandle;
	pI2cMessaage->u8TxNumOfBytes = pI2cMessaage->u8RxNumOfBytes = 0;
	pI2cMessaage->u8TxIndex = pI2cMessaage->u8RxIndex = 0;
}


I2C_MessageCode_e I2Message_getMessageCode(I2C_Message *pI2CMessage)
{
	return pI2CMessage->msgStatusCode;
}


bool I2CMessage_CheckStatus(I2C_Handle i2cHandle)
{
	uint_least8_t i;
	for (i=0; i<3; i++)
	{
		if ((I2C_getStopStatus(i2cHandle)) ||
			 (I2C_getStatusFlag(i2cHandle) & I2C_StatusFlag_BusBusy	))
		{
			usDelay(200);
			continue;
		}
		return true;
	}
	return false;

	/*for (;;)
	{
		if ((I2C_getStopStatus(i2cHandle)) ||
			(I2C_getStatusFlag(i2cHandle) & I2C_StatusFlag_BusBusy	))
		{
			usDelay(200);
			continue;
		}
		return ;
	}*/
}




I2C_MessageCode_e I2CMessage_Synchronize(I2C_Message *pI2CMessage)
{
	//uint16_t i;
	//for (i=0;i< 10000;i++)
	for(;;)
	{
		I2C_MessageCode_e messageCode = I2Message_getMessageCode(pI2CMessage);
		if (messageCode == I2C_MSGSTAT_INACTIVE )
			return I2C_MSGSTAT_INACTIVE;
		else if (messageCode == I2C_MSGSTAT_ERR)
			return I2C_MSGSTAT_ERR;
		usDelay(50);

	}
	//return I2C_MSGSTAT_ERR;
}


bool I2CMessage_WriteMessage0(I2C_Message *pI2cMessage)
{
	I2C_Handle i2cHandle = pI2cMessage->i2cHandle;
	if (I2CMessage_CheckStatus(i2cHandle))
	{
		pI2cMessage->msgStatusCode = I2C_MSGSTAT_WRITE_BUSY;
		pI2cMessage->u8TxIndex = 0;
		I2C_setAddress(i2cHandle, pI2cMessage->u16SlaveAddress);	//I2caRegs.I2CSAR = msg->SlaveAddress;
		I2C_setDataCount(i2cHandle,pI2cMessage->u8TxNumOfBytes);  	//I2caRegs.I2CCNT = msg->NumOfBytes+2;

		I2C_setMode(i2cHandle, I2C_Mode_Master);
		I2C_setTxRxMode(i2cHandle, I2C_Mode_Transmit);
		I2C_StartCond(i2cHandle);
		I2C_clearTxFifoIntFlag(i2cHandle);

		if (I2CMessage_Synchronize(pI2cMessage)== I2C_MSGSTAT_INACTIVE)
			return true;
	}
	return false;
}

bool I2CMessage_WriteMessage(I2C_Message *pI2cMessage)
{
	uint_least8_t u8RetryNo =3;
	while (I2CMessage_WriteMessage0(pI2cMessage) == false)
	{
		if (--u8RetryNo == 0) return false;
		usDelay(50);
	}
	return true;
}

bool I2CMessage_ReadMessage0(I2C_Message *pI2CMessage)
{
	I2C_Handle i2cHandle = pI2CMessage->i2cHandle;
	if (I2CMessage_CheckStatus(i2cHandle))
	{

		pI2CMessage->msgStatusCode = I2C_MSGSTAT_SEND_NOSTOP_BUSY;
		pI2CMessage->u8TxIndex = 0;
		I2C_setAddress(i2cHandle, pI2CMessage->u16SlaveAddress);	//I2caRegs.I2CSAR = msg->SlaveAddress;
		I2C_setDataCount(i2cHandle,pI2CMessage->u8TxNumOfBytes);  	//I2caRegs.I2CCNT = msg->NumOfBytes+2;

		I2C_setMode(i2cHandle, I2C_Mode_Master);
		I2C_setTxRxMode(i2cHandle, I2C_Mode_Transmit);
		I2C_StartCond(i2cHandle);
		I2C_clearTxFifoIntFlag(i2cHandle);

		if (I2CMessage_Synchronize(pI2CMessage)== I2C_MSGSTAT_INACTIVE)
			return true;

	}
	return false;
}

bool I2CMessage_ReadMessage(I2C_Message *pI2cMessage)
{
	uint_least8_t u8RetryNo =3;
	while (I2CMessage_ReadMessage0(pI2cMessage) == false)
	{
		if (--u8RetryNo == 0) return false;
		usDelay(50);

	}
	return true;
}

/*
bool I2CMessage_WriteMessageAck0(I2C_Message *pI2CMessage)
{
	I2C_Handle i2cHandle = pI2CMessage->i2cHandle;
	if (I2CMessage_CheckStatus(i2cHandle))
	{
		pI2CMessage->msgStatusCode = I2C_MSGSTAT_WRITE_POLL;
		pI2CMessage->u8TxIndex = pI2CMessage->u8RetryNo = 0;
		I2C_setAddress(i2cHandle, pI2CMessage->u16SlaveAddress);	//I2caRegs.I2CSAR = msg->SlaveAddress;
		//I2C_setDataCount(i2cHandle,pI2CMessage->u8TxNumOfBytes);  	//I2caRegs.I2CCNT = msg->NumOfBytes+2;

		I2C_setMode(i2cHandle, I2C_Mode_Master);
		I2C_setTxRxMode(i2cHandle, I2C_Mode_Transmit);
		I2C_StartCond(i2cHandle);
		I2C_clearTxFifoIntFlag(i2cHandle);
		return true;
	}
	return false;

}

bool I2CMessage_WriteMessageAck(I2C_Message *pI2CMessage,bool bSyncFlag)
{

	if (I2CMessage_WriteMessageAck0(pI2CMessage))
	{
		if (bSyncFlag == I2C_SYNC)
		{
			if (I2CMessage_Synchronize(pI2CMessage)== I2C_MSGSTAT_INACTIVE)
				return true;
		}
	}
	return false;

}


bool I2CMessage_ReadMessageAck0(I2C_Message *pI2CMessage)
{
	I2C_Handle i2cHandle = pI2CMessage->i2cHandle;

	if (I2CMessage_CheckStatus(i2cHandle))
	{

		pI2CMessage->msgStatusCode = I2C_MSGSTAT_READ_POLL;
		pI2CMessage->u8TxIndex = pI2CMessage->u8RetryNo = 0;
		I2C_setAddress(i2cHandle, pI2CMessage->u16SlaveAddress);	//I2caRegs.I2CSAR = msg->SlaveAddress;
		//I2C_setDataCount(i2cHandle,pI2CMessage->u8TxNumOfBytes);  	//I2caRegs.I2CCNT = msg->NumOfBytes+2;

		I2C_setMode(i2cHandle, I2C_Mode_Master);
		I2C_setTxRxMode(i2cHandle, I2C_Mode_Transmit);
		I2C_StartCond(i2cHandle);
		I2C_clearTxFifoIntFlag(i2cHandle);
		return true;

	}
	return false;
}

bool I2CMessage_ReadMessageAck(I2C_Message *pI2CMessage,bool bSyncFlag)
{
	//I2C_Handle i2cHandle = pI2CMessage->i2cHandle;

	if (I2CMessage_ReadMessageAck0(pI2CMessage))
	{
		if (bSyncFlag == I2C_SYNC)
		{
			if (I2CMessage_Synchronize(pI2CMessage)== I2C_MSGSTAT_INACTIVE)
				return true;
		}
	}
	return false;
}
*/






// end of file
