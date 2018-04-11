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

#include "sci_message.h"
#include "sw/modules/user/src/32b/user.h"
#include "user_data.h"
#include "sw/Sources/sci_operator.h"
#include "sw/Sources/sci_modbus.h"


SCIMessage_Handle SCIMessage_init(void *pMemory,const size_t numBytes)
{
	SCIMessage_Handle sciMessaageHandle;

	if (numBytes < sizeof(SCI_Message))
		return((SCIMessage_Handle)NULL);

	// assign the handle
	sciMessaageHandle = (SCIMessage_Handle)pMemory;
	return(sciMessaageHandle);
}


void SCIMessage_setup(SCIMessage_Handle sciMessageHandle, SCI_Handle sciHandle)
{
	SCI_Message *pSciMessage;
	pSciMessage = (SCI_Message *) sciMessageHandle;

	pSciMessage->sciHandle = sciHandle;
	pSciMessage->bEnableFlag = false;
	pSciMessage->u8TxNumOfBytes = 0;
	pSciMessage->u8TxIndex = pSciMessage->u8RxIndex = 0;

	pSciMessage->u16TimeoutCnt = MAX_OPERTIMEOUT2;
}


//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
bool CheckPacketFormat0(uint_least8_t u8No, uint_least8_t au8Data[])
{
	uint16_t u16Crc16, u16Crc16Tmp;
	if (u8No < 6)
   		return false;
  	else
  	{
    	u16Crc16 = DATA_CalCRC16(au8Data, u8No - 2);
    	u16Crc16Tmp = DATA_MakeWord(au8Data[u8No - 1], au8Data[u8No - 2]);
    	return  (u16Crc16 == u16Crc16Tmp);
  	}
}
//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
bool CheckPacketFormat(SCIMessage_Handle sciMessageHandle)
{
	SCI_Message *pSciMessage = (SCI_Message *) sciMessageHandle;
	return CheckPacketFormat0(pSciMessage->u8RxIndex, pSciMessage->au8RxBuffer);

}

//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
/*void SCIMessage_setTimeoutCnt(SCIMessage_Handle sciMessageHandle,uint16_t u16Ms)	//SCI_Message *pSciMessage)
{
	SCI_Message *pSciMessage = (SCI_Message *) sciMessageHandle;
	//pSciMessage->u16TimeoutCnt = 10000 /(USER_ISR_PERIOD_usec)  * u16Ms;
	pSciMessage->u16TimeoutCnt =  u16Ms;
}*/

bool SCIMessage_TxWrite0(SCIMessage_Handle sciMessageHandle, uint_least8_t u8TxNo, void *pCallbackFun)
{
	SCI_Message *pSciMessage = (SCI_Message *) sciMessageHandle;
	if (pSciMessage->bEnableFlag) return false;	//Busy

	pSciMessage->u8TxNumOfBytes = u8TxNo;
	pSciMessage->u8TxIndex = pSciMessage->u8RxIndex = 0;
	pSciMessage->pCallbackFunc = pCallbackFun;

	pSciMessage->u16TimeoutCnt = MAX_OPERTIMEOUT2;
	pSciMessage->bEnableFlag = true;
	SCI_enableTxFifoInt(pSciMessage->sciHandle);

	return true;
}

bool SCIMessage_TxWrite(SCIMessage_Handle sciMessageHandle,
						uint_least8_t u8TxNo, const uint_least8_t au8OutData[], void *pCallbackFun)
{
	SCI_Message *pSciMessage = (SCI_Message *) sciMessageHandle;
	uint_least8_t i;
	if (pSciMessage->bEnableFlag) return false;	//Busy

	for (i=0; i<u8TxNo; i++)
			pSciMessage->au8TxBuffer[i] = au8OutData[i];
	pSciMessage->u8TxNumOfBytes = u8TxNo;
	pSciMessage->u8TxIndex = pSciMessage->u8RxIndex = 0;
	pSciMessage->pCallbackFunc = pCallbackFun;

	pSciMessage->u16TimeoutCnt = MAX_OPERTIMEOUT2;
	pSciMessage->bEnableFlag = true;
	SCI_enableTxFifoInt(pSciMessage->sciHandle);

	return true;
}

//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
void SCIMessageSlave_run(SCIMessage_Handle sciMessageHandle)

{
	SCI_Message *pSciMessage = (SCI_Message *) sciMessageHandle;
	SCI_Handle sciHandle = pSciMessage->sciHandle;

	if (pSciMessage->u16TimeoutCnt)
		pSciMessage->u16TimeoutCnt--;
	else
	{
		uint_least8_t i, u8Cnt, u8Index;

		u8Cnt = SCI_getRxFifoStatus(sciHandle) >> 8;
		for (i=0; i< u8Cnt; i++)
		{
			u8Index = pSciMessage->u8RxIndex;
			pSciMessage->au8RxBuffer[u8Index] = (uint_least8_t) SCI_read(sciHandle);
			if (u8Index < SCI_RX_MAX_BUFFER_SZIE) pSciMessage->u8RxIndex++;
		}

		if (CheckPacketFormat(sciMessageHandle))
		{
			uint_least8_t  u8Value1, u8Value2;

			pSciMessage->bEnableFlag = false;
			u8Value1 = pSciMessage->au8RxBuffer[0];
			u8Value2 = pSciMessage->au8RxBuffer[1];
			if (((u8Value1== OPERATOR_STARTCODE1) && (u8Value2 == OPERATOR_STARTCODE2B))  ||
				((u8Value1== OPERATOR_STARTCODE1) && (u8Value2 == OPERATOR_STARTCODE2A)))
				OPERATOR_callback(sciMessageHandle);
			else
				MODBUSSlave_callback(sciMessageHandle);
		}
		//else
		{
			pSciMessage->u8RxIndex = 0;		//Start to receive new message
			pSciMessage->u16TimeoutCnt = MAX_OPERTIMEOUT2;
		}
	}

}


void SCIMessageMaster_run(SCIMessage_Handle sciMessageHandle)

{
	SCI_Message *pSciMessage = (SCI_Message *) sciMessageHandle;
	if (pSciMessage->bEnableFlag)
	{
		if (pSciMessage->u16TimeoutCnt)
			pSciMessage->u16TimeoutCnt--;
		else
		{
			uint_least8_t i, u8Cnt, u8Index;
			pCallbackFunc 	*pFunc;

			u8Cnt = SCI_getRxFifoStatus(pSciMessage->sciHandle) >> 8;
			for (i=0; i< u8Cnt; i++)
			{
			    u8Index = pSciMessage->u8RxIndex;
				pSciMessage->au8RxBuffer[u8Index] = (uint_least8_t) SCI_read(pSciMessage->sciHandle);
				if (u8Index < SCI_RX_MAX_BUFFER_SZIE) pSciMessage->u8RxIndex++;
			}

			pSciMessage->bEnableFlag = false;
			pFunc = pSciMessage->pCallbackFunc;
			(*pFunc)(sciMessageHandle);

		}
	}

}




// end of file
