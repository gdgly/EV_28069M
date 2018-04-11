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
//! \file   drivers/sci/src/32b/f28x/f2806x/sci.c
//! \brief  Contains the various functions related to the 
//!         serial communications interface (SCI) object
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "sw/drivers/i2c/src/32b/f28x/f2806x/i2c.h"


//I2C_Message* g_pi2cMessage;				// Used in interrupts

// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions

I2C_Handle I2C_init(void *pMemory,const size_t numBytes)
{
	I2C_Handle i2cHandle;

	if(numBytes < sizeof(I2C_Obj))
		return((I2C_Handle)NULL);

	// assign the handle
	i2cHandle = (I2C_Handle)pMemory;

	return(i2cHandle);
} // end of SCI_init() function


void I2C_disableNAck(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

  // clear the bits
	pI2c->I2CMDR &= (uint16_t) (~I2C_I2CMDR_NACKMOD_BITS);
}

void I2C_disableFreeMode(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CMDR &= (~I2C_I2CMDR_FREE_BITS);
}

void I2C_disableLookbackMode(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CMDR &= (~I2C_I2CMDR_DLB_BITS);
}

void I2C_Reset(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CMDR &= (~I2C_I2CMDR_IRS_BITS);
}

void I2C_disableStartByteMode(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CMDR &= (~I2C_I2CMDR_STB_BITS);
}

void I2C_disableFreeDataFormat(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CMDR &= (~I2C_I2CMDR_FDF_BITS);
}




void I2C_disableAddSlaveInt(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CIER &= (~I2C_I2CIER_AAS_ENA_BITS);
}

void I2C_disableStopCondDectInt(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CIER &= (~I2C_I2CIER_SCD_ENA_BITS);
}

void I2C_disableTXReadyInt(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CIER &= (~I2C_I2CIER_XRDY_ENA_BITS);
}

void I2C_disableRXReadyInt(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CIER &= (~I2C_I2CIER_RRDY_ENA_BITS);
}

void I2C_disableRegAccessReadyInt(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CIER &= (~I2C_I2CIER_ARDY_ENA_BITS);
}

void I2C_disableNoAckInt(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CIER &= (~I2C_I2CIER_NACK_ENA_BITS);
}

void I2C_disableArbiLostInt(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CIER &= (~I2C_I2CIER_AL_ENA_BITS);
}



void I2C_disableTxFifoMode(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CFFTX &= (~I2C_I2CFFTX_I2CFFEN_BITS);
}

void I2C_resetTxFifo(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CFFTX &= (~I2C_I2CFFTX_TXFFRST_BITS);
}

void I2C_disableTxFifoInt(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CFFTX &= (~I2C_I2CFFTX_TXFFIENA_BITS);
}


void I2C_resetRxFifo(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CFFRX &= (~I2C_I2CFFRX_RXFFRST_BITS);
}

void I2C_disableRxFifoInt(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CFFRX &= (~I2C_I2CFFRX_RXFFIENA_BITS);
}

uint_least8_t I2C_getReceiveFifoNo(I2C_Handle i2cHandle)
{
	uint16_t u16Value;
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;
	u16Value = pI2c->I2CFFRX & I2C_I2CFFRX_RXFFST_BITS;
	return (uint_least8_t) (u16Value >>= 8);

}



void I2C_enableAddSlaveInt(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

  // set the bits
	pI2c->I2CIER |= I2C_I2CIER_AAS_ENA_BITS;
}

void I2C_enableStopCondDectInt(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

  // set the bits
	pI2c->I2CIER |= I2C_I2CIER_SCD_ENA_BITS;
}

void I2C_enableTXReadyInt(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

  // set the bits
	pI2c->I2CIER |= I2C_I2CIER_XRDY_ENA_BITS;
}

void I2C_enableRXReadyInt(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

  // set the bits
	pI2c->I2CIER |= I2C_I2CIER_RRDY_ENA_BITS;
}

void I2C_enableRegAccessReadyInt(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// set the bits
	pI2c->I2CIER |= I2C_I2CIER_ARDY_ENA_BITS;
}

void I2C_enableNoAckInt(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// set the bits
	pI2c->I2CIER |= I2C_I2CIER_NACK_ENA_BITS;
}

void I2C_enableArbiLostInt(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

  // set the bits
	pI2c->I2CIER |= I2C_I2CIER_AL_ENA_BITS;
}


void I2C_enableFifoMode(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CFFTX |= (I2C_I2CFFTX_I2CFFEN_BITS);
}

void I2C_enableTxFifo(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CFFTX |= (I2C_I2CFFTX_TXFFRST_BITS);
}

void I2C_clearTxFifoIntFlag(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CFFTX |= (I2C_I2CFFTX_TXFFINTCLR_BITS);
}

void I2C_enableTxFifoInt(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CFFTX |= (I2C_I2CFFTX_TXFFIENA_BITS);
}




void I2C_enableRxFifo(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;
	// clear the bits
	pI2c->I2CFFRX |= (I2C_I2CFFRX_RXFFRST_BITS);
}

void I2C_clearRxFifoIntFlag(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CFFRX |= (I2C_I2CFFRX_RXFFINTCLR_BITS);
}

void I2C_enableRxFifoInt(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CFFRX |= (I2C_I2CFFRX_RXFFIENA_BITS);
}

void I2C_clearNackSend(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CSTR |= I2C_StatusFlag_NackSend;
}


void I2C_clearNackStatus(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CSTR |= I2C_StatusFlag_Nack;
}

void I2C_setCMDR(I2C_Handle i2cHandle, uint16_t u16Value)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;
	pI2c->I2CMDR = u16Value;
}

bool I2C_getStopStatus(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;
	return (pI2c->I2CMDR & I2C_I2CMDR_STP_BITS);
}

I2C_StatusFlag_e I2C_getStatusFlag(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	return (I2C_StatusFlag_e)(pI2c->I2CSTR);
}

I2C_InterruptCode_e I2C_getIntFlagStatus(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;


	// get the status
	I2C_InterruptCode_e status = (I2C_InterruptCode_e)(pI2c->I2CISRC & I2C_I2CISRC_INTCODE_BITS);

	return(status);
}

bool I2C_getFifoTxInt(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;
	return (pI2c->I2CFFTX & I2C_I2CFFTX_TXFFINT_BITS);
}

bool I2C_getFifoRxInt(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;
	return (pI2c->I2CFFRX & I2C_I2CFFRX_RXFFINT_BITS);
}

void I2C_setNAckMode(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

  // set the bits
	pI2c->I2CMDR |= (uint16_t) I2C_I2CMDR_NACKMOD_BITS;
}

void I2C_setFreeMode(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

  // set the bits
	pI2c->I2CMDR |= I2C_I2CMDR_FREE_BITS;
}

void I2C_StartCond(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

  // set the bits
	pI2c->I2CMDR |= I2C_I2CMDR_STT_BITS;
}

void I2C_StopCond(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

  // set the bits
	pI2c->I2CMDR |= I2C_I2CMDR_STP_BITS;
}


void I2C_setRepeatMode(I2C_Handle i2cHandle, const I2C_ModeRepeat_e i2cModeRepeat)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CMDR &= (~I2C_Mode_NonRepeat);

	// set the bits
	pI2c->I2CMDR |= (uint16_t) i2cModeRepeat;

}

void I2C_setLookbackMode(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

  // set the bits
	pI2c->I2CMDR |= I2C_I2CMDR_DLB_BITS;
}

void I2C_Enable(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

  // set the bits
	pI2c->I2CMDR |= I2C_I2CMDR_IRS_BITS;
}

void I2C_setStartByteMode(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

  // set the bits
	pI2c->I2CMDR |= I2C_I2CMDR_STB_BITS;
}

void I2C_setFreeDataFormat(I2C_Handle i2cHandle)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

  // set the bits
	pI2c->I2CMDR |= I2C_I2CMDR_FDF_BITS;
}


void I2C_setBitCountBits(I2C_Handle i2cHandle,const I2C_CharLength_e i2cCharLen)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;
	// clear the bits
	pI2c->I2CMDR &= (~I2C_I2CMDR_BC_BITS);

	// set the bits
	pI2c->I2CMDR |= (uint16_t) i2cCharLen;
 }

void I2C_setMode(I2C_Handle i2cHandle, const I2C_Mode_e i2cMode)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;
	// clear the bits
	pI2c->I2CMDR &= (~I2C_I2CMDR_MST_BITS);

	// set the bits
	pI2c->I2CMDR |= (uint16_t) i2cMode;
}

void I2C_setTxRxMode(I2C_Handle i2cHandle, I2C_ModeTxRx_e i2cModeTxRx)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CMDR &= (~I2C_I2CMDR_TRX_BITS);

  // set the bits
	pI2c->I2CMDR |= (uint16_t) i2cModeTxRx;


}

void I2C_setAddressMode(I2C_Handle i2cHandle, I2C_ModeAddress_e i2cModeAddress)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;

	// clear the bits
	pI2c->I2CMDR &= (~I2C_I2CMDR_XA_BITS);
	// set the bits
	pI2c->I2CMDR |= (uint16_t) i2cModeAddress;

}

void I2C_setTxFifoIntLevel(I2C_Handle i2cHandle, const I2C_FifoLevel_e i2cFifoLevel)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;
	// clear the bits
	pI2c->I2CFFTX &= (~I2C_I2CFFTX_TXFFIL_BITS);
	// set the bits
	pI2c->I2CFFTX |= (uint16_t) i2cFifoLevel;
}

void I2C_setRxFifoIntLevel(I2C_Handle i2cHandle, const I2C_FifoLevel_e i2cFifoLevel)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;
	// clear the bits
	pI2c->I2CFFRX &= (~I2C_I2CFFRX_RXFFIL_BITS);
	// set the bits
	pI2c->I2CFFRX |= (uint16_t) i2cFifoLevel;
}

void I2C_setAddress(I2C_Handle i2cHandle,const uint16_t u16Address)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;
	pI2c->I2CSAR = u16Address;
}

void I2C_setClockLowTime(I2C_Handle i2cHandle,const uint16_t u16Value)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;
	pI2c->I2CCLKL = u16Value;
 }

void I2C_setClockHighTime(I2C_Handle i2cHandle,const uint16_t u16Value)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;
	pI2c->I2CCLKH = u16Value;
 }

void I2C_setPrescaler(I2C_Handle i2cHandle,const uint_least8_t u8ScalerValue)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;
	pI2c->I2CPSC = u8ScalerValue;
 }

void I2C_setDataCount(I2C_Handle i2cHandle,const uint_least8_t u8Value)
{
	I2C_Obj *pI2c = (I2C_Obj *)i2cHandle;
	pI2c->I2CCNT = u8Value;
 }




// end of file
