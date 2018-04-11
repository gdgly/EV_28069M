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
#ifndef _SCI_MESSAGE_H_
#define _SCI_MESSAGE_H_

//! \file   solutions/instaspin_motion/src/ctrl.h
//! \brief Contains the public interface, object and function definitions for 
//!        various functions related to the CTRL object 
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "sw/modules/types/src/types.h"
#include "sw/drivers/sci/src/32b/f28x/f2806x/sci.h"
#include "sw/drivers/gpio/src/32b/f28x/f2806x/gpio.h"




//!
//!
//! \defgroup CTRL CTRL
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


//typedef void (*CALLBACK)(void*);
//void (*A_Task_Ptr)(void);		// State pointer A branch

#define MAX_OPERTIMEOUT1	(20/8)
#define MAX_OPERTIMEOUT2    (400/8)
#define SCI_TX_MAX_BUFFER_SIZE 46
#define SCI_RX_MAX_BUFFER_SZIE 42
#define SCI_MAX_LEVEL	4

//typedef void (* CALLBACK) (int);

typedef struct _SCI_Message_
{
	SCI_Handle	sciHandle;
	bool		bEnableFlag;
    uint16_t u16TimeoutCnt;
    uint_least8_t u8TxNumOfBytes;           	// Num of valid bytes in (or to be put in MsgBuffer)
    uint_least8_t u8TxIndex, u8RxIndex;
    uint_least8_t au8TxBuffer[SCI_TX_MAX_BUFFER_SIZE];    // Array holding msg data - max that
    uint_least8_t au8RxBuffer[SCI_RX_MAX_BUFFER_SZIE];    // Array holding msg data - max that
    void 		*pCallbackFunc;


} SCI_Message;



typedef struct _SCI_Message_ *SCIMessage_Handle;

typedef void (*pCallbackFunc)(SCIMessage_Handle sciMessageHandle);

extern SCIMessage_Handle SCIMessage_init(void *pMemory,const size_t numBytes);
extern void SCIMessage_setup(SCIMessage_Handle sciMessageHandle, SCI_Handle sciHandle);
extern bool SCIMessage_TxWrite0(SCIMessage_Handle sciMessageHandle, uint_least8_t u8TxNo, void *pCallbackFun);
extern bool SCIMessage_TxWrite(SCIMessage_Handle sciMessageHandle, uint_least8_t u8TxNo, const uint_least8_t au8OutData[], void *pCallbackFun);
extern bool CheckPacketFormat0(uint_least8_t u8No, uint_least8_t au8Data[]);
extern bool CheckPacketFormat(SCIMessage_Handle sciMessageHandle);
//extern void SCIMessage_setTimeoutCnt(SCIMessage_Handle sciMessageHandle,uint16_t u16Ms);
//extern void SCIMessage_setTimeoutCnt(uint16_t u16Ms, SCI_Message *pSciMessage);
extern void SCIMessageSlave_run(SCIMessage_Handle sciMessageHandle);
extern void SCIMessageMaster_run(SCIMessage_Handle sciMessageHandle);



#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _I2C_24LC32_H_ definition




