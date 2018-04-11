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
#ifndef _I2C_MESSAGE_H_
#define _I2C_MESSAGE_H_

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




//!
//!
//! \defgroup CTRL CTRL
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


#define I2C_MAX_BUFFER_SIZE (32)
#define I2C_MAX_LEVEL	(4)

//#define I2C_SYNC	true
//#define I2C_ASYNC	false

typedef enum
{
	I2C_ERROR = (int16_t) 0xFFFF,
	I2C_ARB_LOST_ERROR = 0x0001,
	I2C_NACK_ERROR = 0x0002,
	I2C_BUS_BUSY_ERROR = 0x1000,
	I2C_STP_NOT_READY_ERROR = 0x5555,
	I2C_NO_FLAGS =(int16_t) 0xAAAA,
	I2C_SUCCESS =  0x0000
} I2C_CommunicationStatus;

// I2C  Message Commands for I2CMSG struct
typedef enum
{
	I2C_MSGSTAT_INACTIVE = 0x0000,
	I2C_MSGSTAT_TIMEOUT = 0x0001,
	I2C_MSGSTAT_ERR		= 0x0002,
	//I2C_MSGSTAT_BUSY	= 0x0004,

	I2C_MSGSTAT_WRITE_BUSY = 0x0011,
	I2C_MSGSTAT_READ_BUSY = 0x0012,
	I2C_MSGSTAT_SEND_NOSTOP_BUSY = 0x0013,

	//I2C_MSGSTAT_WRITE_POLL = 0x0040,
	//I2C_MSGSTAT_READ_POLL = 0x0041,
	I2C_MSGSTAT_NOACK	= 0x0043
} I2C_MessageCode_e;


//typedef void (*I2C_callbackFunc)(uint_least8_t *pau8Buff);

typedef struct _I2C_Message_
{
	I2C_Handle	i2cHandle;
	I2C_MessageCode_e msgStatusCode;

	//uint16_t 	*pau16DataBuffer;
	uint16_t 	u16SlaveAddress;        // I2C address of slave msg is intended for

	//uint_least8_t u8RetryNo;			// Internal
	uint_least8_t u8TxNumOfBytes,u8RxNumOfBytes;       // Num of valid bytes in (or to be put in MsgBuffer)
	uint_least8_t u8TxIndex,u8RxIndex;
	uint_least8_t au8TxBuffer[I2C_MAX_BUFFER_SIZE+2];    // Array holding msg data - max that
	uint_least8_t au8RxBuffer[I2C_MAX_BUFFER_SIZE];    // Array holding msg data - max that

} I2C_Message;



typedef struct _I2C_Message_ *I2CMessage_Handle;



extern I2CMessage_Handle I2CMessage_init(void *pMemory,const size_t numBytes);
extern void I2CMessage_setup(I2CMessage_Handle i2cMessageHandle, I2C_Handle i2cHandle);

//extern bool I2CMessage_CheckStatus(I2C_Handle i2cHandle);
//extern bool I2CMessage_WriteMessage0(I2C_Message *pI2CMessage);
extern bool I2CMessage_WriteMessage(I2C_Message *pI2cMessage);
extern bool I2CMessage_ReadMessage(I2C_Message *pI2CMessage);

//extern bool I2CMessage_WriteMessageAck0(I2C_Message *pI2CMessage);
//extern bool I2CMessage_WriteMessageAck(I2C_Message *pI2CMessage, bool bSyncFlag);
//extern bool I2CMessage_ReadMessageAck(I2C_Message *pI2CMessage, bool bSyncFlag);

//extern I2C_MessageCode_e I2CMessage_Synchronize(I2C_Message *pI2CMessage);



#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _I2C_24LC32_H_ definition




