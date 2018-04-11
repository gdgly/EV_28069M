/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
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
//! \file   drivers/pie/src/32b/f28x/f2806x/pie.c
//! \brief  Contains the various functions related to the peripheral interrupt
//!         expansion (PIE) object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.

// **************************************************************************
// the includes

#include "sw/drivers/pie/src/32b/f28x/f2806x/pie.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions

void PIE_clearAllFlags(PIE_Handle pieHandle)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;
    uint_least8_t u8groupCnt;


    for(u8groupCnt=0;u8groupCnt<12;u8groupCnt++)
    {
      ppie->PIEIER_PIEIFR[u8groupCnt].IFR = 0;
    }


} // end of PIE_clearAllFlags() function


void PIE_clearAllInts(PIE_Handle pieHandle)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;


    // set the bits
    ppie->PIEACK |= 0xFFFF;

} // end of PIE_clearAllInts() function


void PIE_disable(PIE_Handle pieHandle)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;


    // clear the bits
    ppie->PIECTRL &= (~PIE_PIECTRL_ENPIE_BITS);


} // end of PIE_disable() function

void PIE_disableCaptureInt(PIE_Handle pieHandle)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;

    // set the value
    ppie->PIEIER_PIEIFR[3].IER &= ~PIE_IERx_INTx1_BITS;


} // end of PIE_disableCaptureInt() function


void PIE_disableExtInt(PIE_Handle pieHandle, const CPU_ExtIntNumber_e intNumber)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;


    // clear the bits
    ppie->XINTnCR[intNumber] &= (~PIE_XINTnCR_ENABLE_BITS);


} // end of PIE_disableExtInt() function


void PIE_disableAllInts(PIE_Handle pieHandle)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;

    uint_least8_t u8groupCnt;

    for(u8groupCnt=0;u8groupCnt<12;u8groupCnt++)
    {
    	ppie->PIEIER_PIEIFR[u8groupCnt].IER = 0;
    }


} // end of PIE_disableAllInts() function

void PIE_disableInt(PIE_Handle pieHandle, const PIE_GroupNumber_e group, const PIE_InterruptSource_e intSource)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;

    ppie->PIEIER_PIEIFR[group].IER &= ~intSource;
    
    
}


void PIE_enable(PIE_Handle pieHandle)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;


    // set the bits
    ppie->PIECTRL |= PIE_PIECTRL_ENPIE_BITS;


} // end of PIE_enable() function


void PIE_enableAdcInt(PIE_Handle pieHandle, const ADC_IntNumber_e intNumber)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;
    uint16_t u16index;
    uint16_t u16setValue;


    if(intNumber < ADC_IntNumber_9)
    {
    	u16index = 9;
    	u16setValue = 1 << intNumber;
    }
    else if(intNumber == ADC_IntNumber_9)
    {
    	u16index = 0;
    	u16setValue = 1 << 5;
    }
    else
    {
    	u16index = 0;
    	u16setValue = 1 << ((intNumber & 0x07) - 1) ;
    }

    // set the value
    ppie->PIEIER_PIEIFR[u16index].IER |= u16setValue;

} // end of PIE_enableAdcInt() function

void PIE_enableCaptureInt(PIE_Handle pieHandle)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;

    // set the value
    ppie->PIEIER_PIEIFR[3].IER |= PIE_IERx_INTx1_BITS;//$a|=b------->a=a|b  (a or b)


} // end of PIE_enableCaptureInt() function


void PIE_enableExtInt(PIE_Handle pieHandle, const CPU_ExtIntNumber_e intNumber)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;
    uint16_t u16index;
    uint16_t u16setValue;

    
    if(intNumber < CPU_ExtIntNumber_3)
    {
    	u16index = 0;
    	u16setValue = 1 << (intNumber + 3);
    }
    else
    {
    	u16index = 10;
    	u16setValue = 1 << 0;
    }


    // set the value
    ppie->PIEIER_PIEIFR[u16index].IER |= u16setValue;


    // set the bits
    ppie->XINTnCR[intNumber] |= PIE_XINTnCR_ENABLE_BITS;

} // end of PIE_enableExtInt() function

void PIE_enableInt(PIE_Handle pieHandle, const PIE_GroupNumber_e group, const PIE_InterruptSource_e intSource)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;
    
    ppie->PIEIER_PIEIFR[group].IER |= intSource;
    

}


void PIE_enablePwmInt(PIE_Handle pieHandle, const PWM_Number_e pwmNumber)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;
    uint16_t u16index = 2;
    uint16_t u16setValue = (1 << pwmNumber);


    // set the value
    ppie->PIEIER_PIEIFR[u16index].IER |= u16setValue;


} // end of PIE_enablePwmInt() function

void PIE_enablePwmTzInt(PIE_Handle pieHandle, const PWM_Number_e pwmNumber)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;
    uint16_t u16index = 1;
    uint16_t u16setValue = (1 << pwmNumber);


    // set the value
    ppie->PIEIER_PIEIFR[u16index].IER |= u16setValue;


} // end of PIE_enablePwmTzInt() function

void PIE_enableTimer0Int(PIE_Handle pieHandle)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;

    // set the value
    ppie->PIEIER_PIEIFR[0].IER |= PIE_IERx_INTx7_BITS;

} // end of PIE_enableTimer0Int() function

void PIE_enableI2cInt(PIE_Handle pieHandle)
{
	// Enable I2C interrupt 1 in the PIE: Group 8 interrupt 1
	//  PieCtrlRegs.PIEIER8.bit.INTx1 = 1;

	PIE_Obj *ppie = (PIE_Obj *)pieHandle;
	//uint16_t u16Index = 7;
	//uint16_t u16SetValue = 1 << 0;

	// set the value
	ppie->PIEIER_PIEIFR[7].IER |= PIE_IERx_INTx1_BITS;

	//u16SetValue = 1 << 1;
	ppie->PIEIER_PIEIFR[7].IER |= PIE_IERx_INTx2_BITS;
}

void PIE_enableSpiARxInt(PIE_Handle pieHandle)
{
	PIE_Obj *ppie = (PIE_Obj *)pieHandle;
	//uint16_t u16Index = 5;
	//uint16_t u16SetValue = 1;

	// set the value
	ppie->PIEIER_PIEIFR[5].IER |= PIE_IERx_INTx1_BITS;

}

void PIE_enableSpiATxInt(PIE_Handle pieHandle)
{
	PIE_Obj *ppie = (PIE_Obj *)pieHandle;
	//uint16_t u16Index = 5;
	//uint16_t u16SetValue = 1;

	// set the value
	ppie->PIEIER_PIEIFR[5].IER |= PIE_IERx_INTx2_BITS;

}
void PIE_enableSpiBRxInt(PIE_Handle pieHandle)
{
	PIE_Obj *ppie = (PIE_Obj *)pieHandle;
	//uint16_t u16Index = 5;
	//uint16_t u16SetValue = 2;

	// set the value
	ppie->PIEIER_PIEIFR[5].IER |= PIE_IERx_INTx3_BITS;

}

void PIE_enableSpiBTxInt(PIE_Handle pieHandle)
{
	PIE_Obj *ppie = (PIE_Obj *)pieHandle;
	//uint16_t u16Index = 5;
	//uint16_t u16SetValue = 3;

	// set the value
	ppie->PIEIER_PIEIFR[5].IER |= PIE_IERx_INTx4_BITS;

}

void PIE_enableSciARxInt(PIE_Handle pieHandle)
{
	PIE_Obj *ppie = (PIE_Obj *)pieHandle;
	//uint16_t u16Index = 8;
	//uint16_t u16SetValue = 0;

	// set the value
	ppie->PIEIER_PIEIFR[8].IER |= PIE_IERx_INTx1_BITS;

} // end of PIE_enableSciARxInt() function

void PIE_enableSciATxInt(PIE_Handle pieHandle)
{
	PIE_Obj *ppie = (PIE_Obj *)pieHandle;
	//uint16_t u16Index = 8;
	//uint16_t u16SetValue = 1;

	// set the value
	ppie->PIEIER_PIEIFR[8].IER |= PIE_IERx_INTx2_BITS;

} // end of PPIE_enableSciATxInt() function


void PIE_enableSciBRxInt(PIE_Handle pieHandle)
{
	PIE_Obj *ppie = (PIE_Obj *)pieHandle;
	//uint16_t u16Index = 8;
	//uint16_t u16SetValue = 2;

	// set the value
	ppie->PIEIER_PIEIFR[8].IER |= PIE_IERx_INTx3_BITS;

} // end of PIE_enableSciBRxInt() function

void PIE_enableSciBTxInt(PIE_Handle pieHandle)
{
	PIE_Obj *ppie = (PIE_Obj *)pieHandle;
	//uint16_t u16Index = 8;
	//uint16_t u16SetValue = 3;

	// set the value
	ppie->PIEIER_PIEIFR[8].IER |= PIE_IERx_INTx4_BITS;

} // end of PPIE_enableSciBTxInt() function



void PIE_forceInt(PIE_Handle pieHandle, const PIE_GroupNumber_e group, const PIE_InterruptSource_e intSource)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;
    
    ppie->PIEIER_PIEIFR[group].IFR |= intSource;
    

}


uint16_t PIE_getExtIntCount(PIE_Handle pieHandle, const CPU_ExtIntNumber_e intNumber)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;


    // get the count value
    uint16_t u16count = ppie->XINTnCTR[intNumber];

    return(u16count);
} // end of PIE_getExtIntCount() function


uint16_t PIE_getIntEnables(PIE_Handle pieHandle, const PIE_GroupNumber_e group)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;
    
    return (ppie->PIEIER_PIEIFR[group].IER);
} // end of PIE_getIntEnables() function


uint16_t PIE_getIntFlags(PIE_Handle pieHandle, const PIE_GroupNumber_e group)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;
    
    return (ppie->PIEIER_PIEIFR[group].IFR);
} // end of PIE_getIntFlags() function


interrupt void PIE_illegalIsr(void)
{

    // The next two lines are placeholders
    asm(" ESTOP0");

    // endless hold loop
    for(;;);

} // end of PIE_illegalIsr() function


PIE_Handle PIE_init(void *pMemory, const size_t numBytes)
{
    PIE_Handle pieHandle;


    if(numBytes < sizeof(PIE_Obj))
    	return((PIE_Handle)NULL);

    // assign the handle
    pieHandle = (PIE_Handle)pMemory;

    return(pieHandle);
} // end of PIE_init() function



void PIE_registerPieIntHandler(PIE_Handle pieHandle, 
                           const PIE_GroupNumber_e groupNumber, 
                           const PIE_SubGroupNumber_e subGroupNumber, 
                           const PIE_IntVec_t vector)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;
    PIE_IntVec_t *pintPointer;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    
    // Point to the beginning of the PIE table
    pintPointer = (PIE_IntVec_t *)&(ppie->ADCINT1_HP);

    // Increment pointer to the correct group
    pintPointer += groupNumber * 8;

    // Increment point to the correct subgroup
    pintPointer += subGroupNumber;

    // Set the vector
    *pintPointer = vector;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;
    

} // end of PIE_registerIntHandler() function


void PIE_registerSystemIntHandler(PIE_Handle pieHandle, 
                           const PIE_SystemInterrupts_e systemInt, 
                           const PIE_IntVec_t vector)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;
    PIE_IntVec_t *intPointer;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    
    // Point to the beginning of the system interrupt table
    intPointer = (PIE_IntVec_t *)&(pie->Reset);

    // Increment point to the correct interrupt
    intPointer += systemInt;

    // Set the vector
    *intPointer = vector;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    return;
} // end of PIE_registerIntHandler() function


void PIE_setDefaultIntVectorTable(PIE_Handle pieHandle)
{
    PIE_Obj *pie = (PIE_Obj *)pieHandle;
    PIE_IntVec_t *addr = (PIE_IntVec_t *)&(pie->INT1);
    uint16_t u16regCnt;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // initialize the table to PIE_illegalIsr() address
    for(u16regCnt=0;u16regCnt<120;u16regCnt++)
    {
      *addr++ = &PIE_illegalIsr;
    }
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of PIE_setDefaultIntVectorTable() function


void PIE_setExtIntPolarity(PIE_Handle pieHandle, 
                           const CPU_ExtIntNumber_e intNumber, 
                           const PIE_ExtIntPolarity_e polarity)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;


    // clear the bits
    ppie->XINTnCR[intNumber] &= (~PIE_XINTnCR_POLARITY_BITS);

    // set the bits
    ppie->XINTnCR[intNumber] |= polarity;


} // end of PIE_setExtIntPolarity() function


void PIE_unregisterPieIntHandler(PIE_Handle pieHandle, 
                           const PIE_GroupNumber_e groupNumber, 
                           const PIE_SubGroupNumber_e subGroupNumber)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;
    PIE_IntVec_t *pintPointer;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    
    // Point to the beginning of the PIE table
    pintPointer = (PIE_IntVec_t *)&(ppie->ADCINT1_HP);

    // Increment pointer to the correct group
    pintPointer += groupNumber * 8;

    // Increment point to the correct subgroup
    pintPointer += subGroupNumber;

    // Set the vector
    *pintPointer = PIE_illegalIsr;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of PIE_unregisterPieIntHandler() function


void PIE_unregisterSystemIntHandler(PIE_Handle pieHandle, 
                           const PIE_SystemInterrupts_e systemInt)
{
    PIE_Obj *ppie = (PIE_Obj *)pieHandle;
    PIE_IntVec_t *pintPointer;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    
    // Point to the beginning of the system interrupt table
    pintPointer = (PIE_IntVec_t *)&(ppie->Reset);

    // Increment point to the correct interrupt
    pintPointer += systemInt;

    // Set the vector
    *pintPointer = PIE_illegalIsr;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

} // end of PIE_unregisterSystemIntHandler() function

// end of file
