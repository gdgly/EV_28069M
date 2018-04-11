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
#include "sci_modbus.h"
#include "user_data.h"

//#include "ctrlQEP.h"
//#include "hal.h"
//#include "user.h"

#define MODBUS_READCODE		0x03
#define MOSBUS_WRITECODE 	0x10
#define MODBUS_LOOPBACK	  	0x08



MODBUS_Handle MODBUS_init(void *pMemory,const size_t numBytes)
{
	MODBUS_Handle modbusHandle;

	if (numBytes < sizeof(MODBUS_Obj))
		return((MODBUS_Handle)NULL);

	// assign the handle
	modbusHandle = (MODBUS_Handle)pMemory;

	return(modbusHandle);
}

void MODBUS_setup(MODBUS_Handle modbusHandle)
{
	//MODBUS_Obj *pModbus;
	//pModbus = (MODBUS_Obj *) modbusHandle;



}

//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
uint16_t GetModbusRegData(uint16_t u16RegAdd)
{
	switch(u16RegAdd)
	{
		case 0x002D:
			return 0x012D;
		case 0x003B:
			return 0x23F1;
	}
	return 0xffff;
}
//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
void ModbusReadProcess(SCI_Message *pSciMessage)
{
	uint16_t u16StartReg;
	uint_least8_t u8TotNo, u8HiByte, u8LoByte;
	uint_least8_t *pu8RxBuff= &(pSciMessage->au8RxBuffer[2]);
	uint_least8_t *pu8TxBuff= &(pSciMessage->au8TxBuffer[0]);

	u8HiByte = *pu8RxBuff++;
	u8LoByte = *pu8RxBuff++;
	u16StartReg = DATA_MakeWord(u8HiByte, u8LoByte);

	u8HiByte = *pu8RxBuff++;
	u8LoByte = *pu8RxBuff;
	u8TotNo = (uint_least8_t) DATA_MakeWord(u8HiByte, u8LoByte);

	if (u8TotNo < 0x10)	//Normal Response
	{
		uint_least8_t i;
		uint16_t u16Value;
		*pu8TxBuff++ = pSciMessage->au8RxBuffer[0];	// Address
		*pu8TxBuff++ = MODBUS_READCODE;
		*pu8TxBuff++ = (u8TotNo << 1) & 0x00ff;	// TotByte
		for (i=0; i<u8TotNo; i++)
		{
			u16Value = GetModbusRegData(u16StartReg++);
			*pu8TxBuff++ = (u16Value >> 8);		//HiByte
			*pu8TxBuff++ = (u16Value & 0x00ff);	//LoByte
		}
		u8TotNo = (u8TotNo << 1)+ 3;
		u16Value = DATA_CalCRC16(pSciMessage->au8TxBuffer,u8TotNo);
		*pu8TxBuff++ = DATA_LoByte(u16Value);	//LoByte
		*pu8TxBuff = DATA_HiByte(u16Value);		//HiByte

		pSciMessage->u8TxNumOfBytes = u8TotNo+2;
		pSciMessage->u8TxIndex = 0;
		SCI_enableTxFifoInt(pSciMessage->sciHandle);
		//SCI_clearTxFifoInt(pSciMessage->sciHandle);

	}
	else
	{

	}

}

void ModbusWriteProcess(SCI_Message *pSciMessage)
{
	/*uint16_t u16StartReg;
	uint_least8_t u8TotNo;
	uint_least8_t *pu8RxBuff= &(pSciMessage->au8RxBuffer[2]);
	uint_least8_t *pu8TxBuff= &(pSciMessage->au8TxBuffer[0]);

	u16StartReg = USER_MakeWord(*pu8RxBuff++,*pu8RxBuff++);
	u8TotNo = (uint_least8_t) USER_MakeWord(*pu8RxBuff++,*pu8RxBuff);

	if (u8TotNo < 0x10)	//Normal Response
	{
		uint_least8_t i;
		uint16_t u16Value;
		*pu8TxBuff++ = pSciMessage->au8RxBuffer[0];	// Address
		*pu8TxBuff++ = MODBUS_READCODE;
		*pu8TxBuff++ = (u8TotNo << 1) & 0x00ff;	// TotByte
		for (i=0; i<u8TotNo; i++)
		{
			u16Value = GetModbusRegData(u16StartReg++);
			*pu8TxBuff++ = (u16Value >> 8);		//HiByte
			*pu8TxBuff++ = (u16Value & 0x00ff);	//LoByte
		}
		u8TotNo = (u8TotNo << 1)+ 3;
		u16Value = CalCRC16(pSciMessage->au8TxBuffer,u8TotNo);
		*pu8TxBuff++ = u16Value & 0x00ff;	//LoByte
		*pu8TxBuff = (u16Value >> 8);		//HiByte

		pSciMessage->u8TxNumOfBytes = u8TotNo+2;
		pSciMessage->u8TxIndex = 0;
		SCI_clearTxFifoInt(pSciMessage->sciHandle);

	}
	else
	{

	}*/

}
//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
void MODBUSSlave_callback(SCIMessage_Handle sciMessageHandle)
{
	SCI_Message *pSciMessage = (SCI_Message *) sciMessageHandle;
	switch (pSciMessage->au8RxBuffer[1])
	{
		case MODBUS_READCODE:	//Read
			ModbusReadProcess(pSciMessage);
			break;
		case MOSBUS_WRITECODE:
			ModbusWriteProcess(pSciMessage);
			break;
		case MODBUS_LOOPBACK:
			break;
	}
}
//------------------------------------------------------------------------
//
//------------------------------------------------------------------------

void MODBUSMaster_callback(SCIMessage_Handle sciMessageHandle)
{
	SCI_Message *pSciMessage = (SCI_Message *) sciMessageHandle;
	switch (pSciMessage->au8RxBuffer[1])
	{
		case MODBUS_READCODE:	//Read
			ModbusReadProcess(pSciMessage);
			break;
		case MOSBUS_WRITECODE:
			ModbusWriteProcess(pSciMessage);
			break;
		case MODBUS_LOOPBACK:
			break;
	}
}



// end of file
