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
//! \file   solutions/instaspin_foc/boards/hvkit_rev1p1/f28x/f2806xF/src/hal.c
//! \brief Contains the various functions related to the HAL object (everything outside the CTRL system) 
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

// drivers

// modules

// platforms
#include "hal.h"
#include "hal_obj.h"
#include "sw/modules/user/src/32b/user.h"


#include "sw/Sources/sci_message.h"
#include "sw/Sources/i2c_message.h"
#include "sw/Sources/user_data.h"

#ifdef FLASH
#pragma CODE_SECTION(HAL_setupFlash,"ramfuncs");
#endif

// **************************************************************************
// the defines


// **************************************************************************
// the globals

HAL_Obj ghal;


// **************************************************************************
// the functions

void HAL_cal(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;


	// enable the ADC clock
	CLK_enableAdcClock(pobj->clkHandle);


	// Run the Device_cal() function
	// This function copies the ADC and oscillator calibration values from TI reserved
	// OTP into the appropriate trim registers
	// This boot ROM automatically calls this function to calibrate the interal
	// oscillators and ADC with device specific calibration data.
	// If the boot ROM is bypassed by Code Composer Studio during the development process,
	// then the calibration must be initialized by the application
	ENABLE_PROTECTED_REGISTER_WRITE_MODE;
	(*Device_cal)();
	DISABLE_PROTECTED_REGISTER_WRITE_MODE;

	// run offsets calibration in user's memory
	HAL_AdcOffsetSelfCal(handle);

	// run oscillator compensation
	HAL_OscTempComp(handle);

	// disable the ADC clock
	CLK_disableAdcClock(pobj->clkHandle);


} // end of HAL_cal() function


void HAL_OscTempComp(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
	uint16_t u16Temperature;

	// disable the ADCs
	ADC_disable(pobj->adcHandle);

	// power up the bandgap circuit
	ADC_enableBandGap(pobj->adcHandle);

	// set the ADC voltage reference source to internal
	ADC_setVoltRefSrc(pobj->adcHandle,ADC_VoltageRefSrc_Int);

	// enable the ADC reference buffers
	ADC_enableRefBuffers(pobj->adcHandle);

	// Set main clock scaling factor (max45MHz clock for the ADC module)
	ADC_setDivideSelect(pobj->adcHandle,ADC_DivideSelect_ClkIn_by_2);

	// power up the ADCs
	ADC_powerUp(pobj->adcHandle);

	// enable the ADCs
	ADC_enable(pobj->adcHandle);

	// enable non-overlap mode
	ADC_enableNoOverlapMode(pobj->adcHandle);

	// connect channel A5 internally to the temperature sensor
	ADC_setTempSensorSrc(pobj->adcHandle, ADC_TempSensorSrc_Int);

	// set SOC0 channel select to ADCINA5
	ADC_setSocChanNumber(pobj->adcHandle, ADC_SocNumber_0, ADC_SocChanNumber_A5);

	// set SOC0 acquisition period to 26 ADCCLK
	ADC_setSocSampleDelay(pobj->adcHandle, ADC_SocNumber_0, ADC_SocSampleDelay_64_cycles);

	// connect ADCINT1 to EOC0
	ADC_setIntSrc(pobj->adcHandle, ADC_IntNumber_1, ADC_IntSrc_EOC0);

	// clear ADCINT1 flag
	ADC_clearIntFlag(pobj->adcHandle, ADC_IntNumber_1);

	// enable ADCINT1
	ADC_enableInt(pobj->adcHandle, ADC_IntNumber_1);

	// force start of conversion on SOC0
	ADC_setSocFrc(pobj->adcHandle, ADC_SocFrc_0);

	// wait for end of conversion
	while (ADC_getIntFlag(pobj->adcHandle, ADC_IntNumber_1) == 0){}

	// clear ADCINT1 flag
	ADC_clearIntFlag(pobj->adcHandle, ADC_IntNumber_1);

	u16Temperature = ADC_readResult(pobj->adcHandle, ADC_ResultNumber_0);

	HAL_osc1Comp(handle, u16Temperature);

	HAL_osc2Comp(handle, u16Temperature);


} // end of HAL_OscTempComp() function


void HAL_osc1Comp(HAL_Handle handle, const int16_t sensorSample)
{
	int16_t i16compOscFineTrim;
	HAL_Obj *pobj = (HAL_Obj *)handle;

	ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    i16compOscFineTrim = ((sensorSample - getRefTempOffset())*(int32_t)getOsc1FineTrimSlope()
                      + OSC_POSTRIM_OFF + FP_ROUND )/FP_SCALE + getOsc1FineTrimOffset() - OSC_POSTRIM;

    if(i16compOscFineTrim > 31)
      {
        i16compOscFineTrim = 31;
      }
	else if(i16compOscFineTrim < -31)
      {
        i16compOscFineTrim = -31;
      }

    OSC_setTrim(pobj->oscHandle, OSC_Number_1, HAL_getOscTrimValue(getOsc1CoarseTrim(), i16compOscFineTrim));

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

} // end of HAL_osc1Comp() function


void HAL_osc2Comp(HAL_Handle handle, const int16_t i16sensorSample)
{
	int16_t i16compOscFineTrim;
	HAL_Obj *pobj = (HAL_Obj *)handle;

	ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    i16compOscFineTrim = ((i16sensorSample - getRefTempOffset())*(int32_t)getOsc2FineTrimSlope()
                      + OSC_POSTRIM_OFF + FP_ROUND )/FP_SCALE + getOsc2FineTrimOffset() - OSC_POSTRIM;

    if(i16compOscFineTrim > 31)
      {
        i16compOscFineTrim = 31;
      }
	else if(i16compOscFineTrim < -31)
      {
        i16compOscFineTrim = -31;
      }

    OSC_setTrim(pobj->oscHandle, OSC_Number_2, HAL_getOscTrimValue(getOsc2CoarseTrim(), i16compOscFineTrim));

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of HAL_osc2Comp() function


uint16_t HAL_getOscTrimValue(int16_t i16coarse, int16_t i16fine)
{
	uint16_t u16regValue = 0;

	if(i16fine < 0)
    {
		u16regValue = ((-i16fine) | 0x20) << 9;
    }
	else
    {
		u16regValue = i16fine << 9;
    }

	if(i16coarse < 0)
    {
		u16regValue |= ((-i16coarse) | 0x80);
    }
	else
    {
		u16regValue |= i16coarse;
    }

	return u16regValue;
} // end of HAL_getOscTrimValue() function


void HAL_AdcOffsetSelfCal(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
	uint16_t u16AdcConvMean;

	// disable the ADCs
	ADC_disable(pobj->adcHandle);

	// power up the bandgap circuit
	ADC_enableBandGap(pobj->adcHandle);

	// set the ADC voltage reference source to internal
	ADC_setVoltRefSrc(pobj->adcHandle,ADC_VoltageRefSrc_Int);

	// enable the ADC reference buffers
	ADC_enableRefBuffers(pobj->adcHandle);

	// Set main clock scaling factor (max45MHz clock for the ADC module)
	ADC_setDivideSelect(pobj->adcHandle,ADC_DivideSelect_ClkIn_by_2);

	// power up the ADCs
	ADC_powerUp(pobj->adcHandle);

	// enable the ADCs
	ADC_enable(pobj->adcHandle);

	//Select VREFLO internal connection on B5
	ADC_enableVoltRefLoConv(pobj->adcHandle);

	//Select channel B5 for all SOC
	HAL_AdcCalChanSelect(handle, ADC_SocChanNumber_B5);

	//Apply artificial offset (+80) to account for a negative offset that may reside in the ADC core
	ADC_setOffTrim(pobj->adcHandle, 80);

	//Capture ADC conversion on VREFLO
	u16AdcConvMean = HAL_AdcCalConversion(handle);

	//Set offtrim register with new value (i.e remove artical offset (+80) and create a two's compliment of the offset error)
	ADC_setOffTrim(pobj->adcHandle, 80 - u16AdcConvMean);

	//Select external ADCIN5 input pin on B5
	ADC_disableVoltRefLoConv(pobj->adcHandle);

} // end of HAL_AdcOffsetSelfCal() function


void HAL_AdcCalChanSelect(HAL_Handle handle, const ADC_SocChanNumber_e chanNumber)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_0,chanNumber);
	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_1,chanNumber);
	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_2,chanNumber);
	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_3,chanNumber);
	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_4,chanNumber);
	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_5,chanNumber);
	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_6,chanNumber);
	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_7,chanNumber);
	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_8,chanNumber);
	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_9,chanNumber);
	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_10,chanNumber);
	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_11,chanNumber);
	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_12,chanNumber);
	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_13,chanNumber);
	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_14,chanNumber);
	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_15,chanNumber);

} // end of HAL_AdcCalChanSelect() function


uint16_t HAL_AdcCalConversion(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
	uint16_t u16index, u16SampleSize, u16Mean;
	uint32_t u32Sum;
	ADC_SocSampleDelay_e ACQPS_Value;

	u16index       = 0;     //initialize index to 0
	u16SampleSize  = 256;   //set sample size to 256 (**NOTE: Sample size must be multiples of 2^x where is an integer >= 4)
	u32Sum         = 0;     //set sum to 0
	u16Mean        = 999;   //initialize mean to known value

	//Set the ADC sample window to the desired value (Sample window = ACQPS + 1)
	ACQPS_Value = ADC_SocSampleDelay_7_cycles;

	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_0,ACQPS_Value);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_1,ACQPS_Value);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_2,ACQPS_Value);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_3,ACQPS_Value);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_4,ACQPS_Value);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_5,ACQPS_Value);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_6,ACQPS_Value);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_7,ACQPS_Value);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_8,ACQPS_Value);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_9,ACQPS_Value);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_10,ACQPS_Value);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_11,ACQPS_Value);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_12,ACQPS_Value);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_13,ACQPS_Value);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_14,ACQPS_Value);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_15,ACQPS_Value);

	// Enabled ADCINT1 and ADCINT2
	ADC_enableInt(pobj->adcHandle, ADC_IntNumber_1);
	ADC_enableInt(pobj->adcHandle, ADC_IntNumber_2);

	// Disable continuous sampling for ADCINT1 and ADCINT2
	ADC_setIntMode(pobj->adcHandle, ADC_IntNumber_1, ADC_IntMode_EOC);
	ADC_setIntMode(pobj->adcHandle, ADC_IntNumber_2, ADC_IntMode_EOC);

	//ADCINTs trigger at end of conversion
	ADC_setIntPulseGenMode(pobj->adcHandle, ADC_IntPulseGenMode_Prior);

	// Setup ADCINT1 and ADCINT2 trigger source
	ADC_setIntSrc(pobj->adcHandle, ADC_IntNumber_1, ADC_IntSrc_EOC6);
	ADC_setIntSrc(pobj->adcHandle, ADC_IntNumber_2, ADC_IntSrc_EOC14);

	// Setup each SOC's ADCINT trigger source
	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_0, ADC_Int2TriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_1, ADC_Int2TriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_2, ADC_Int2TriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_3, ADC_Int2TriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_4, ADC_Int2TriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_5, ADC_Int2TriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_6, ADC_Int2TriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_7, ADC_Int2TriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_8, ADC_Int1TriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_9, ADC_Int1TriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_10, ADC_Int1TriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_11, ADC_Int1TriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_12, ADC_Int1TriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_13, ADC_Int1TriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_14, ADC_Int1TriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_15, ADC_Int1TriggersSOC);

 		// Delay before converting ADC channels
 	usDelay(ADC_DELAY_usec);

 	ADC_setSocFrcWord(pobj->adcHandle, 0x00FF);

 	while( u16index < u16SampleSize )
    {
 		//Wait for ADCINT1 to trigger, then add ADCRESULT0-7 registers to sum
 		while (ADC_getIntFlag(pobj->adcHandle, ADC_IntNumber_1) == 0){}

 		//Must clear ADCINT1 flag since INT1CONT = 0
 		ADC_clearIntFlag(pobj->adcHandle, ADC_IntNumber_1);

 		u32Sum += ADC_readResult(pobj->adcHandle, ADC_ResultNumber_0);
 		u32Sum += ADC_readResult(pobj->adcHandle, ADC_ResultNumber_1);
 		u32Sum += ADC_readResult(pobj->adcHandle, ADC_ResultNumber_2);
 		u32Sum += ADC_readResult(pobj->adcHandle, ADC_ResultNumber_3);
 		u32Sum += ADC_readResult(pobj->adcHandle, ADC_ResultNumber_4);
 		u32Sum += ADC_readResult(pobj->adcHandle, ADC_ResultNumber_5);
 		u32Sum += ADC_readResult(pobj->adcHandle, ADC_ResultNumber_6);
 		u32Sum += ADC_readResult(pobj->adcHandle, ADC_ResultNumber_7);

 		//Wait for ADCINT2 to trigger, then add ADCRESULT8-15 registers to sum
 		while (ADC_getIntFlag(pobj->adcHandle, ADC_IntNumber_2) == 0){}

 		//Must clear ADCINT2 flag since INT2CONT = 0
 		ADC_clearIntFlag(pobj->adcHandle, ADC_IntNumber_2);

 		u32Sum += ADC_readResult(pobj->adcHandle, ADC_ResultNumber_8);
 		u32Sum += ADC_readResult(pobj->adcHandle, ADC_ResultNumber_9);
 		u32Sum += ADC_readResult(pobj->adcHandle, ADC_ResultNumber_10);
 		u32Sum += ADC_readResult(pobj->adcHandle, ADC_ResultNumber_11);
 		u32Sum += ADC_readResult(pobj->adcHandle, ADC_ResultNumber_12);
 		u32Sum += ADC_readResult(pobj->adcHandle, ADC_ResultNumber_13);
 		u32Sum += ADC_readResult(pobj->adcHandle, ADC_ResultNumber_14);
 		u32Sum += ADC_readResult(pobj->adcHandle, ADC_ResultNumber_15);

 		u16index+=16;

    } // end data collection

 	//Disable ADCINT1 and ADCINT2 to STOP the ping-pong sampling
 	ADC_disableInt(pobj->adcHandle, ADC_IntNumber_1);
 	ADC_disableInt(pobj->adcHandle, ADC_IntNumber_2);

 	//Calculate average ADC sample value
 	u16Mean = u32Sum / u16SampleSize;

 	// Clear start of conversion trigger
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_0, ADC_NoIntTriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_1, ADC_NoIntTriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_2, ADC_NoIntTriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_3, ADC_NoIntTriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_4, ADC_NoIntTriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_5, ADC_NoIntTriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_6, ADC_NoIntTriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_7, ADC_NoIntTriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_8, ADC_NoIntTriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_9, ADC_NoIntTriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_10, ADC_NoIntTriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_11, ADC_NoIntTriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_12, ADC_NoIntTriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_13, ADC_NoIntTriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_14, ADC_NoIntTriggersSOC);
 	ADC_setupSocTrigSrc(pobj->adcHandle, ADC_SocNumber_15, ADC_NoIntTriggersSOC);

 	//return the average
 	return(u16Mean);
} // end of HAL_AdcCalConversion() function


void HAL_disableWdog(HAL_Handle halHandle)
{
	HAL_Obj *phal = (HAL_Obj *)halHandle;


	WDOG_disable(phal->wdogHandle);

} // end of HAL_disableWdog() function


void HAL_disableGlobalInts(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;


	CPU_disableGlobalInts(pobj->cpuHandle);

} // end of HAL_disableGlobalInts() function





void HAL_enableAdcInts(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;


	// enable the PIE interrupts associated with the ADC interrupts
	//(pobj->pieHandle,ADC_IntNumber_1);
	PIE_enableAdcInt(pobj->pieHandle,ADC_IntNumber_1HP);//ADC_IntNumber_1HP=9
	PIE_enableAdcInt(pobj->pieHandle,ADC_IntNumber_2HP);

	// enable the ADC interrupts
	ADC_enableInt(pobj->adcHandle,ADC_IntNumber_1);
	ADC_enableInt(pobj->adcHandle,ADC_IntNumber_2);


	// enable the cpu interrupt for ADC interrupts
	//CPU_enableInt(pobj->cpuHandle,CPU_IntNumber_10);
	CPU_enableInt(pobj->cpuHandle,CPU_IntNumber_1);


} // end of HAL_enableAdcInts() function






void HAL_enablePwmInt(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;


	PIE_enablePwmInt(pobj->pieHandle,PWM_Number_1);


	// enable the interrupt
	PWM_enableInt(pobj->pwmHandle[PWM_Number_1]);


	// enable the cpu interrupt for EPWM1_INT
	CPU_enableInt(pobj->cpuHandle,CPU_IntNumber_3);


} // end of HAL_enablePwmInt() function

void HAL_enableTimer0Int(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	PIE_enableTimer0Int(pobj->pieHandle);
	// enable the interrupt
	TIMER_enableInt(pobj->timerHandle[0]);

	// enable the cpu interrupt for TINT0
	CPU_enableInt(pobj->cpuHandle,CPU_IntNumber_1);

} // end of HAL_enablePwmInt() function

void HAL_enableTimer1Int(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	//PIE_enableTimer0Int(pobj->pieHandle);
	// enable the interrupt
	TIMER_enableInt(pobj->timerHandle[1]);

	// enable the cpu interrupt for TINT0
	CPU_enableInt(pobj->cpuHandle,CPU_IntNumber_13);

} // end of HAL_enablePwmInt() function



void HAL_enableI2cInt(HAL_Handle handle)
{
	HAL_Obj *pHalObj = (HAL_Obj *)handle;

	PIE_enableI2cInt(pHalObj->pieHandle);

	// enable the interrupt
	I2C_enableTxFifoInt(pHalObj->i2cHandle);
	I2C_enableRxFifoInt(pHalObj->i2cHandle);

	CPU_enableInt(pHalObj->cpuHandle,CPU_IntNumber_8);

} // end of HAL_enableSpiAInt() function

void HAL_enableSpiAInt(HAL_Handle handle)
{


} // end of HAL_enableSpiAInt() function

void HAL_enableSpiBInt(HAL_Handle handle)
{


 /* HAL_Obj *obj = (HAL_Obj *)HAL_getHandle();

  PIE_enableSciARxInt(pObj->pieHandle);
  PIE_enableSciATxInt(pObj->pieHandle);

  // enable the interrupt
  SCI_enableRxInt(pObj->sciAHandle);
  SCI_enableTxInt(pObj->sciAHandle);

  CPU_enableInt(pObj->cpuHandle,CPU_IntNumber_9);*/

} // end of HAL_enableSpiBInt() function

void HAL_enableSciAInt(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	PIE_enableSciARxInt(pobj->pieHandle);
	PIE_enableSciATxInt(pobj->pieHandle);

	// enable the interrupt
	//SCI_enableTxFifoInt(pHalObj->sciHandle[0]);
	SCI_enableRxFifoInt(pobj->sciHandle[0]);
	//SCI_enableRxInt(pObj->sciHandle[0]);
	//SCI_enableTxInt(pObj->sciHandle[0]);

	CPU_enableInt(pobj->cpuHandle,CPU_IntNumber_9);

} // end of HAL_enableSciAInt() function

void HAL_enableSciBInt(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	PIE_enableSciBRxInt(pobj->pieHandle);
	PIE_enableSciBTxInt(pobj->pieHandle);

	// enable the interrupt
	//SCI_enableTxFifoInt(pHalObj->sciHandle[1]);
	SCI_enableRxFifoInt(pobj->sciHandle[1]);
	//SCI_enableRxInt(pObj->sciHandle[1]);
	//SCI_enableTxInt(pObj->sciHandle[1]);

	CPU_enableInt(pobj->cpuHandle,CPU_IntNumber_9);


} // end of HAL_enableSciBInt() function

void HAL_enableGlobalInts(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;


	CPU_enableGlobalInts(pobj->cpuHandle);

} // end of HAL_enableGlobalInts() function

void HAL_enableDebugInt(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;


	CPU_enableDebugInt(pobj->cpuHandle);


} // end of HAL_enableDebugInt() function

void HAL_setupFaults(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
	uint_least8_t u8cnt;


	// Configure Trip Mechanism for the Motor control software
	// -Cycle by cycle trip on CPU halt
	// -One shot fault trip zone
	// These trips need to be repeated for EPWM1 ,2 & 3
	for(u8cnt=0;u8cnt<3;u8cnt++)
    {
		PWM_enableTripZoneSrc(pobj->pwmHandle[u8cnt],PWM_TripZoneSrc_CycleByCycle_TZ6_NOT);

		PWM_enableTripZoneSrc(pobj->pwmHandle[u8cnt],PWM_TripZoneSrc_OneShot_TZ1_NOT);

		// What do we want the OST/CBC events to do?
		// TZA events can force EPWMxA
		// TZB events can force EPWMxB

		//PWM_setTripZoneState_TZA(pobj->pwmHandle[u8cnt],PWM_TripZoneState_EPWM_Low);	//Org
		//PWM_setTripZoneState_TZB(pobj->pwmHandle[u8cnt],PWM_TripZoneState_EPWM_Low);	//Org

		//PWM_setTripZoneState_TZA(pobj->pwmHandle[u8cnt],PWM_TripZoneState_HighImp);	//Modify
		//PWM_setTripZoneState_TZB(pobj->pwmHandle[u8cnt],PWM_TripZoneState_HighImp);	//Modify

		PWM_setTripZoneState_TZA(pobj->pwmHandle[u8cnt],PWM_TripZoneState_EPWM_High);		//Modify
		PWM_setTripZoneState_TZB(pobj->pwmHandle[u8cnt],PWM_TripZoneState_EPWM_High);		//Modify

		// Clear faults from flip flop
		//GPIO_setLow(obj->gpioHandle,GPIO_Number_9);		//Org
		//GPIO_setHigh(obj->gpioHandle,GPIO_Number_9);		//Org
    }

} // end of HAL_setupFaults() function


HAL_Handle HAL_init(void *pMemory,const size_t numBytes)
{
	uint_least8_t u8cnt;
	HAL_Handle handle;  //$typedef struct _HAL_Obj_ *HAL_Handle;    pointer
	HAL_Obj *pobj;      //$typedef struct _HAL_Obj_ {....}HAL_Obj;  pointer


	if(numBytes < sizeof(HAL_Obj))
		return((HAL_Handle)NULL);


	// assign the handle
	handle = (HAL_Handle)pMemory;   //$typedef struct _HAL_Obj_ *HAL_Handle;


	// assign the object
	pobj = (HAL_Obj *)handle;//$the return



	// initialize the watchdog driver
	pobj->wdogHandle = WDOG_init((void *)WDOG_BASE_ADDR,sizeof(WDOG_Obj));


	// disable watchdog
	HAL_disableWdog(handle);


	// initialize the ADC
	pobj->adcHandle = ADC_init((void *)ADC_BASE_ADDR,sizeof(ADC_Obj));


	// initialize the clock handle
	pobj->clkHandle = CLK_init((void *)CLK_BASE_ADDR,sizeof(CLK_Obj));//$return(clkHandle);


	// initialize the CPU handle
	pobj->cpuHandle = CPU_init(&cpu,sizeof(cpu));


	// initialize the FLASH handle
	pobj->flashHandle = FLASH_init((void *)FLASH_BASE_ADDR,sizeof(FLASH_Obj));


	// initialize the GPIO handle
	pobj->gpioHandle = GPIO_init((void *)GPIO_BASE_ADDR,sizeof(GPIO_Obj));


	// initialize the current offset estimator handles
	for(u8cnt=0;u8cnt<USER_NUM_CURRENT_SENSORS;u8cnt++)
    {
		pobj->offsetHandle_I[u8cnt] = OFFSET_init(&pobj->offset_I[u8cnt],sizeof(pobj->offset_I[u8cnt]));
    }


	// initialize the voltage offset estimator handles
	for(u8cnt=0;u8cnt<USER_NUM_VOLTAGE_SENSORS;u8cnt++)     //$#define USER_NUM_VOLTAGE_SENSORS    (3)
    {
		pobj->offsetHandle_V[u8cnt] = OFFSET_init(&pobj->offset_V[u8cnt],sizeof(pobj->offset_V[u8cnt]));
    }


	// initialize the oscillator handle
	pobj->oscHandle = OSC_init((void *)OSC_BASE_ADDR,sizeof(OSC_Obj));


	// initialize the PIE handle
	pobj->pieHandle = PIE_init((void *)PIE_BASE_ADDR,sizeof(PIE_Obj));


	// initialize the PLL handle
	pobj->pllHandle = PLL_init((void *)PLL_BASE_ADDR,sizeof(PLL_Obj));


	// initialize PWM handles
	pobj->pwmHandle[0] = PWM_init((void *)PWM_ePWM1_BASE_ADDR,sizeof(PWM_Obj));
	pobj->pwmHandle[1] = PWM_init((void *)PWM_ePWM2_BASE_ADDR,sizeof(PWM_Obj));
	pobj->pwmHandle[2] = PWM_init((void *)PWM_ePWM3_BASE_ADDR,sizeof(PWM_Obj));


	// initialize PWM DAC handles
	pobj->pwmDacHandle[0] = PWMDAC_init((void *)PWM_ePWM6_BASE_ADDR,sizeof(PWM_Obj));
	pobj->pwmDacHandle[1] = PWMDAC_init((void *)PWM_ePWM5_BASE_ADDR,sizeof(PWM_Obj));
	pobj->pwmDacHandle[2] = PWMDAC_init((void *)PWM_ePWM4_BASE_ADDR,sizeof(PWM_Obj));


	// initialize power handle
	pobj->pwrHandle = PWR_init((void *)PWR_BASE_ADDR,sizeof(PWR_Obj));

	// initialize the I2C handle
	pobj->i2cHandle = I2C_init((void *)I2C_BASE_ADDR,sizeof(I2C_Obj));

	//initialize SCI handle
	pobj->sciHandle[0] = SCI_init((void *)SCIA_BASE_ADDR,sizeof(SCI_Obj));
	pobj->sciHandle[1] = SCI_init((void *)SCIB_BASE_ADDR,sizeof(SCI_Obj));

	// initialize SPI handle
	pobj->spiHandle[0] = SPI_init((void *)SPIA_BASE_ADDR,sizeof(SPI_Obj));
	pobj->spiHandle[1] = SPI_init((void *)SPIB_BASE_ADDR,sizeof(SPI_Obj));

	// initialize timer handles
	pobj->timerHandle[0] = TIMER_init((void *)TIMER0_BASE_ADDR,sizeof(TIMER_Obj));
	pobj->timerHandle[1] = TIMER_init((void *)TIMER1_BASE_ADDR,sizeof(TIMER_Obj));
	pobj->timerHandle[2] = TIMER_init((void *)TIMER2_BASE_ADDR,sizeof(TIMER_Obj));

#ifdef QEP
	// initialize QEP driver
	pobj->qepHandle[0] = QEP_init((void*)QEP1_BASE_ADDR,sizeof(QEP_Obj));
#endif

  return(handle);
} // end of HAL_init() function





void HAL_setParams(HAL_Handle handle,const USER_Params *pUserParams)
{
	uint_least8_t u8cnt;
	HAL_Obj *pobj = (HAL_Obj *)handle;
	_iq iqbeta_lp_pu = _IQ(pUserParams->foffsetPole_rps/(float_t)pUserParams->u32ctrlFreq_Hz);



	HAL_setNumCurrentSensors(handle,pUserParams->u8numCurrentSensors);
	HAL_setNumVoltageSensors(handle,pUserParams->u8numVoltageSensors);


	for(u8cnt=0;u8cnt<HAL_getNumCurrentSensors(handle);u8cnt++)
    {
		HAL_setOffsetBeta_lp_pu(handle,HAL_SensorType_Current,u8cnt,iqbeta_lp_pu);
		HAL_setOffsetInitCond(handle,HAL_SensorType_Current,u8cnt,_IQ(0.0));
		HAL_setOffsetValue(handle,HAL_SensorType_Current,u8cnt,_IQ(0.0));
    }


	for(u8cnt=0;u8cnt<HAL_getNumVoltageSensors(handle);u8cnt++)
    {
		HAL_setOffsetBeta_lp_pu(handle,HAL_SensorType_Voltage,u8cnt,iqbeta_lp_pu);
		HAL_setOffsetInitCond(handle,HAL_SensorType_Voltage,u8cnt,_IQ(0.0));
		HAL_setOffsetValue(handle,HAL_SensorType_Voltage,u8cnt,_IQ(0.0));
    }


	// disable global interrupts
	CPU_disableGlobalInts(pobj->cpuHandle);


	// disable cpu interrupts
	CPU_disableInts(pobj->cpuHandle);


	// clear cpu interrupt flags
	CPU_clearIntFlags(pobj->cpuHandle);


	// setup the clocks
	HAL_setupClks(handle);


	// Setup the PLL
	HAL_setupPll(handle,PLL_ClkFreq_90_MHz);


	// setup the PIE
	HAL_setupPie(handle);


	// run the device calibration
	HAL_cal(handle);


	// setup the peripheral clocks
	HAL_setupPeripheralClks(handle);


 	// setup the GPIOs
	HAL_setupGpios(handle);


	// setup the flash
	HAL_setupFlash(handle);


	// setup the ADCs
	HAL_setupAdcs(handle);



	// setup the PWMs
	HAL_setupPwms(handle,
                pUserParams->u16systemFreq_MHz,
                pUserParams->fpwmPeriod_usec,
                USER_NUM_PWM_TICKS_PER_ISR_TICK);

#ifdef QEP
	// setup the QEP
	HAL_setupQEP(handle,HAL_Qep_QEP1);
#endif

	// setup the PWM DACs
	HAL_setupPwmDacs(handle);


	// setup the timers
	HAL_setupTimers(handle,pUserParams->u16systemFreq_MHz);

	//setup I2C
	HAL_setupI2C(handle);

	//setup SCI
	HAL_setupSCI(handle);

	//setup SPI
	HAL_setupSPI(handle);

  // set the default current bias
	{
		uint_least8_t u8cnt;
		_iq iqbias = _IQ12mpy(ADC_dataBias,_IQ(pUserParams->fcurrent_sf));
   
		for(u8cnt=0;u8cnt<HAL_getNumCurrentSensors(handle);u8cnt++)
		{
			HAL_setBias(handle,HAL_SensorType_Current,u8cnt,iqbias);
		}
	}


	//  set the current scale factor
	{

		_iq iqcurrent_sf =  _IQ( pUserParams->fcurrent_sf);
		//_iq iqcurrent_sf = (_iq)(pUserParams->fcurrent_sf * 16777216.0f);
		//iqcurrent_sf =  _IQ( pUserParams->fcurrent_sf);
		//float fTemp = pUserParams->fcurrent_sf;
		//_iq iqcurrent_sf = _IQ(fTemp);


		HAL_setCurrentScaleFactor(handle,iqcurrent_sf);
	}


	// set the default voltage bias
	{
		uint_least8_t u8cnt;
		_iq iqbias = _IQ(0.0);
   
		for(u8cnt=0;u8cnt<HAL_getNumVoltageSensors(handle);u8cnt++)
		{
			HAL_setBias(handle,HAL_SensorType_Voltage,u8cnt,iqbias);
		}
	}


  //  set the voltage scale factor
	{
		HAL_setVoltageScaleFactor(handle,_IQ(pUserParams->fvoltage_sf));
	}


	{
		HAL_setExtTempScaleFactor(handle, _IQ(pUserParams->fExtTemp_sf));
		HAL_setExtAdcScaleFactor(handle, _IQ(pUserParams->fExtAdc_sf));
	}


} // end of HAL_setParams() function


void HAL_setupAdcs(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;


	// disable the ADCs
	ADC_disable(pobj->adcHandle);


	// power up the bandgap circuit
	ADC_enableBandGap(pobj->adcHandle);


	// set the ADC voltage reference source to internal
	ADC_setVoltRefSrc(pobj->adcHandle,ADC_VoltageRefSrc_Int);


	// enable the ADC reference buffers
	ADC_enableRefBuffers(pobj->adcHandle);


	// Set main clock scaling factor (max45MHz clock for the ADC module)
 	ADC_setDivideSelect(pobj->adcHandle,ADC_DivideSelect_ClkIn_by_2);


 	// power up the ADCs
 	ADC_powerUp(pobj->adcHandle);


 	// enable the ADCs
 	ADC_enable(pobj->adcHandle);


 	// set the ADC interrupt pulse generation to prior
 	ADC_setIntPulseGenMode(pobj->adcHandle,ADC_IntPulseGenMode_Prior);


 	// set the temperature sensor source to external
 	//ADC_setTempSensorSrc(pobj->adcHandle,ADC_TempSensorSrc_Ext);	//Org
 	ADC_setTempSensorSrc(pobj->adcHandle,ADC_TempSensorSrc_Int);

 	// configure the interrupt sources
 	ADC_disableInt(pobj->adcHandle,ADC_IntNumber_1);
 	ADC_setIntMode(pobj->adcHandle,ADC_IntNumber_1,ADC_IntMode_ClearFlag);
 	ADC_setIntSrc(pobj->adcHandle,ADC_IntNumber_1,ADC_IntSrc_EOC7);	//Org

 	//configure the SOCs for hvkit_rev1p1
 	// EXT IA-FB
 	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_0,ADC_SocChanNumber_A1);
 	ADC_setSocTrigSrc(pobj->adcHandle,ADC_SocNumber_0,ADC_SocTrigSrc_EPWM1_ADCSOCA);
 	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_0,ADC_SocSampleDelay_9_cycles);

 	// EXT IA-FB
 	// Duplicate conversion due to ADC Initial Conversion bug (SPRZ342)
 	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_1,ADC_SocChanNumber_A1);	//U-Curr
 	ADC_setSocTrigSrc(pobj->adcHandle,ADC_SocNumber_1,ADC_SocTrigSrc_EPWM1_ADCSOCA);
 	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_1,ADC_SocSampleDelay_9_cycles);

 	// EXT IB-FB
 	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_2,ADC_SocChanNumber_B1);	//V-Curr
 	ADC_setSocTrigSrc(pobj->adcHandle,ADC_SocNumber_2,ADC_SocTrigSrc_EPWM1_ADCSOCA);
 	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_2,ADC_SocSampleDelay_9_cycles);

 	// EXT IC-FB
 	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_3,ADC_SocChanNumber_A3);	//W-Curr
 	ADC_setSocTrigSrc(pobj->adcHandle,ADC_SocNumber_3,ADC_SocTrigSrc_EPWM1_ADCSOCA);
 	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_3,ADC_SocSampleDelay_9_cycles);

 	// ADC-Vhb1
 	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_4,ADC_SocChanNumber_B7);	//U-Vol
 	ADC_setSocTrigSrc(pobj->adcHandle,ADC_SocNumber_4,ADC_SocTrigSrc_EPWM1_ADCSOCA);
 	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_4,ADC_SocSampleDelay_9_cycles);

 	// ADC-Vhb2
 	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_5,ADC_SocChanNumber_B6);	//V-Vol
 	ADC_setSocTrigSrc(pobj->adcHandle,ADC_SocNumber_5,ADC_SocTrigSrc_EPWM1_ADCSOCA);
 	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_5,ADC_SocSampleDelay_9_cycles);

 	// ADC-Vhb3
 	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_6,ADC_SocChanNumber_B4);	//W-Vol
 	ADC_setSocTrigSrc(pobj->adcHandle,ADC_SocNumber_6,ADC_SocTrigSrc_EPWM1_ADCSOCA);
 	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_6,ADC_SocSampleDelay_9_cycles);

 	// VDCBUS
 	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_7,ADC_SocChanNumber_A7);	//DC_Bus
 	ADC_setSocTrigSrc(pobj->adcHandle,ADC_SocNumber_7,ADC_SocTrigSrc_EPWM1_ADCSOCA);
 	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_7,ADC_SocSampleDelay_9_cycles);


  //Adding ADC_SocNumber_8 ~ ADC_SocNumber_11

 	//ADC_setSocChanNumber(hal.adcHandle,ADC_SocNumber_8,ADC_SocChanNumber_B0);
    //ADC_setSocTrigSrc(hal.adcHandle,ADC_SocNumber_8,ADC_SocTrigSrc_Sw);
 	//    ADC_setSocSampleDelay(hal.adcHandle,ADC_SocNumber_8,ADC_SocSampleDelay_9_cycles);
 	// configure the interrupt sources

 	ADC_disableInt(pobj->adcHandle,ADC_IntNumber_2);
 	ADC_setIntMode(pobj->adcHandle,ADC_IntNumber_2,ADC_IntMode_ClearFlag);
 	ADC_setIntSrc(pobj->adcHandle,ADC_IntNumber_2,ADC_IntSrc_EOC14);	//Org

	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_8,ADC_SocChanNumber_A0);		//Ext. Temp
	ADC_setSocTrigSrc(pobj->adcHandle,ADC_SocNumber_8,ADC_SocTrigSrc_CpuTimer_1);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_8,ADC_SocSampleDelay_9_cycles);

	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_9,ADC_SocChanNumber_A2);		//Ext1
	ADC_setSocTrigSrc(pobj->adcHandle,ADC_SocNumber_9,ADC_SocTrigSrc_CpuTimer_1);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_9,ADC_SocSampleDelay_9_cycles);

	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_10,ADC_SocChanNumber_A4);		//Ext2
	ADC_setSocTrigSrc(pobj->adcHandle, ADC_SocNumber_10, ADC_SocTrigSrc_CpuTimer_1);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_10,ADC_SocSampleDelay_9_cycles);

	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_11,ADC_SocChanNumber_A6);		//Ext3
	ADC_setSocTrigSrc(pobj->adcHandle, ADC_SocNumber_11, ADC_SocTrigSrc_CpuTimer_1);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_11,ADC_SocSampleDelay_9_cycles);

	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_12,ADC_SocChanNumber_B0);		//Ext4
	ADC_setSocTrigSrc(pobj->adcHandle, ADC_SocNumber_12, ADC_SocTrigSrc_CpuTimer_1);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_12,ADC_SocSampleDelay_9_cycles);

	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_13,ADC_SocChanNumber_B2);		//apx1
	ADC_setSocTrigSrc(pobj->adcHandle, ADC_SocNumber_13, ADC_SocTrigSrc_CpuTimer_1);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_13,ADC_SocSampleDelay_9_cycles);

	ADC_setSocChanNumber(pobj->adcHandle,ADC_SocNumber_14,ADC_SocChanNumber_B3);		//apx2
	ADC_setSocTrigSrc(pobj->adcHandle, ADC_SocNumber_14, ADC_SocTrigSrc_CpuTimer_1);
	ADC_setSocSampleDelay(pobj->adcHandle,ADC_SocNumber_14,ADC_SocSampleDelay_9_cycles);


} // end of HAL_setupAdcs() function


void HAL_setupClks(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;


	// enable internal oscillator 1
	CLK_enableOsc1(pobj->clkHandle);

	// set the oscillator source
	CLK_setOscSrc(pobj->clkHandle,CLK_OscSrc_Internal);

	// disable the external clock in
	CLK_disableClkIn(pobj->clkHandle);

	// disable the crystal oscillator
	CLK_disableCrystalOsc(pobj->clkHandle);

	// disable oscillator 2
	CLK_disableOsc2(pobj->clkHandle);

	// set the low speed clock prescaler
	CLK_setLowSpdPreScaler(pobj->clkHandle,CLK_LowSpdPreScaler_SysClkOut_by_4);

	// set the clock out prescaler
	CLK_setClkOutPreScaler(pobj->clkHandle,CLK_ClkOutPreScaler_SysClkOut_by_1);

} // end of HAL_setupClks() function


void HAL_setupFlash(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;


	FLASH_enablePipelineMode(pobj->flashHandle);

	FLASH_setNumPagedReadWaitStates(pobj->flashHandle,FLASH_NumPagedWaitStates_3);

	FLASH_setNumRandomReadWaitStates(pobj->flashHandle,FLASH_NumRandomWaitStates_3);

	FLASH_setOtpWaitStates(pobj->flashHandle,FLASH_NumOtpWaitStates_5);

	FLASH_setStandbyWaitCount(pobj->flashHandle,FLASH_STANDBY_WAIT_COUNT_DEFAULT);

	FLASH_setActiveWaitCount(pobj->flashHandle,FLASH_ACTIVE_WAIT_COUNT_DEFAULT);

} // HAL_setupFlash() function


//$ this set the GPIO
//$ the declaration is in gpio.h
//$ the pin can have different function by setting to different mode(also declare in gpio.h)

void HAL_setupGpios(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	//$GPIO_setMode(handle,number of pin,which mode you're going to use);

	// PWM1A
	GPIO_setMode(pobj->gpioHandle,GPIO_Number_0,GPIO_0_Mode_EPWM1A);

	// PWM1B
	GPIO_setMode(pobj->gpioHandle,GPIO_Number_1,GPIO_1_Mode_EPWM1B);

	// PWM2A
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_2,GPIO_2_Mode_EPWM2A);

 	// PWM2B
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_3,GPIO_3_Mode_EPWM2B);

 	// PWM3A
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_4,GPIO_4_Mode_EPWM3A);

 	// PWM3B
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_5,GPIO_5_Mode_EPWM3B);

 	// PWM
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_6,GPIO_6_Mode_EPWM4A);

 	// PWM4B
 	//GPIO_setMode(pobj->gpioHandle,GPIO_Number_7,GPIO_7_Mode_GeneralPurpose);	// Org
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_7,GPIO_7_Mode_EPWM4B);		//Modify


 	// PWM5A
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_8,GPIO_8_Mode_EPWM5A);

 	// PWM5B

 	/*GPIO_setMode(pobj->gpioHandle,GPIO_Number_9, GPIO_9_Mode_SCITXDB);		//TXD_B (Modify)
 	GPIO_setQualification(pobj->gpioHandle,GPIO_Number_9, GPIO_Qual_ASync);	//Modify
    */
	GPIO_setMode(pobj->gpioHandle,GPIO_Number_9, GPIO_9_Mode_EPWM5B);		//TXD_B (Modify)


 	// DI#4
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_10,GPIO_10_Mode_GeneralPurpose);
	GPIO_setDirection(pobj->gpioHandle,GPIO_Number_10,GPIO_Direction_Input);		//Adding

 	// DI#5
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_11,GPIO_11_Mode_GeneralPurpose);
	GPIO_setDirection(pobj->gpioHandle,GPIO_Number_11,GPIO_Direction_Input);		//Adding

 	// DI#3
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_12,GPIO_12_Mode_GeneralPurpose);
	GPIO_setDirection(pobj->gpioHandle,GPIO_Number_12,GPIO_Direction_Input);		//Adding

 	// DI#1
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_13,GPIO_13_Mode_GeneralPurpose);	//Org
	GPIO_setDirection(pobj->gpioHandle,GPIO_Number_13,GPIO_Direction_Input);		//Adding

 	// DI#2
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_14,GPIO_14_Mode_GeneralPurpose);	//Org
	GPIO_setDirection(pobj->gpioHandle,GPIO_Number_14,GPIO_Direction_Input);		//Adding

 	// DI#0
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_15,GPIO_15_Mode_GeneralPurpose);
	GPIO_setDirection(pobj->gpioHandle,GPIO_Number_15,GPIO_Direction_Input);		//Adding


 	// Set Qualification Period for GPIO16-23, 22*2*(1/90MHz) = 0.48us
 	//GPIO_setQualificationPeriod(pobj->gpioHandle,GPIO_Number_16,22);		//Org

 	// SPI-MOSI_A
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_16,GPIO_16_Mode_SPISIMOA);

 	// SPI-MISO_A
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_17,GPIO_17_Mode_SPISOMIA);

 	// SPI-CLK_A
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_18,GPIO_18_Mode_SPICLKA);		//Org

 	// SPI-CS_A
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_19,GPIO_19_Mode_SPISTEA_NOT);

#ifdef QEP
 	// EQEP1_A
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_20,GPIO_20_Mode_EQEP1A);			// Org (QEP1A)
 	GPIO_setQualification(pobj->gpioHandle,GPIO_Number_20,GPIO_Qual_Sample_3);

 	// EQEP1_B
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_21,GPIO_21_Mode_EQEP1B);			//Org (QEP1B)
 	GPIO_setQualification(pobj->gpioHandle,GPIO_Number_21,GPIO_Qual_Sample_3);

 	// DO#1
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_22,GPIO_22_Mode_GeneralPurpose);
 	GPIO_setDirection(pobj->gpioHandle,GPIO_Number_22,GPIO_Direction_Output);

 	// EQEP1_I
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_23,GPIO_23_Mode_EQEP1I);			//QEP1I
 	GPIO_setQualification(pobj->gpioHandle,GPIO_Number_23,GPIO_Qual_Sample_3);
#else
 	// EQEPA
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_20,GPIO_20_Mode_GeneralPurpose);

 	// EQEPB
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_21,GPIO_21_Mode_GeneralPurpose);

 	// STATUS
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_22,GPIO_22_Mode_GeneralPurpose);

 	// EQEP1I
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_23,GPIO_23_Mode_GeneralPurpose);
#endif

 	// DO#2
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_24,GPIO_24_Mode_GeneralPurpose);		//Driver_Enable
 	GPIO_setDirection(pobj->gpioHandle,GPIO_Number_24,GPIO_Direction_Output);
 	GPIO_setHigh(pobj->gpioHandle, GPIO_Number_24);

 	// CAN_ENable
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_25,GPIO_25_Mode_GeneralPurpose);
 	GPIO_setDirection(pobj->gpioHandle,GPIO_Number_25,GPIO_Direction_Output);

 	// USB D+
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_26,GPIO_26_Mode_GeneralPurpose);		//Org(//USB D+ ???)

 	// USB D-
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_27,GPIO_27_Mode_GeneralPurpose);		//ORg (//USB D- ??)

 	// RXD_A
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_28,GPIO_28_Mode_SCIRXDA);			//Modify
 	GPIO_setQualification(pobj->gpioHandle,GPIO_Number_28, GPIO_Qual_ASync);	//Modify

 	// TXD_A
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_29,GPIO_29_Mode_SCITXDA);		//	Modify (TXD_A)
 	GPIO_setQualification(pobj->gpioHandle,GPIO_Number_29, GPIO_Qual_ASync);	//Moodify

 	// CAN_RX
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_30,GPIO_30_Mode_CANRXA);		// Modify : CAN_RXA
 	//GPIO_setQualification(pobj->gpioHandle,GPIO_Number_30, GPIO_Qual_ASync);	//Moodify

 	// CAN_TX
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_31,GPIO_31_Mode_CANTXA);		//Modify: CAN_TXA

 	// I2C_SDA
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_32,GPIO_32_Mode_SDAA);		//Modify :SDA_A (I2C)

 	// I2C_SCL
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_33,GPIO_33_Mode_SCLA);		//Modify: SCL_A (I2C)


 	// DI#6
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_34,GPIO_34_Mode_GeneralPurpose);		//Org : //DO#4 OL1 Level2
 	GPIO_setDirection(pobj->gpioHandle,GPIO_Number_34,GPIO_Direction_Input);

 	// JTAG
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_35,GPIO_35_Mode_JTAG_TDI);
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_36,GPIO_36_Mode_JTAG_TMS);
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_37,GPIO_37_Mode_JTAG_TDO);
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_38,GPIO_38_Mode_JTAG_TCK);

 	//Brake Level DO
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_39,GPIO_39_Mode_GeneralPurpose);		//Org : //DI#10 KEY1
 	GPIO_setDirection(pobj->gpioHandle,GPIO_Number_39,GPIO_Direction_Output);		//Adding

 	// TXD_B
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_40,GPIO_40_Mode_SCITXDB);

 	// RXD_B
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_41,GPIO_41_Mode_SCIRXDB);

 	// Hall_A
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_42,GPIO_42_Mode_GeneralPurpose);
 	GPIO_setDirection(pobj->gpioHandle,GPIO_Number_42,GPIO_Direction_Input);

 	// Hall_B
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_43,GPIO_43_Mode_GeneralPurpose);
 	GPIO_setDirection(pobj->gpioHandle,GPIO_Number_43,GPIO_Direction_Input);

 	// Hall_C
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_44,GPIO_44_Mode_GeneralPurpose);
 	GPIO_setDirection(pobj->gpioHandle,GPIO_Number_44,GPIO_Direction_Input);


 	// #TZ1
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_50,GPIO_50_Mode_TZ1_NOT);

 	// #TZ2
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_51,GPIO_51_Mode_TZ2_NOT);

 	//#TZ3
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_52,GPIO_52_Mode_TZ3_NOT);

 	// DA_LVL
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_53,GPIO_53_Mode_GeneralPurpose);
 	GPIO_setDirection(pobj->gpioHandle,GPIO_Number_53,GPIO_Direction_Output);		//Adding

 	// QEP2_A
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_54,GPIO_54_Mode_EQEP2A);

 	// QEP2_B
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_55,GPIO_55_Mode_EQEP2B);

 	// QEP2_I
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_56,GPIO_56_Mode_EQEP2I);

 	//Brake DO
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_57,GPIO_57_Mode_GeneralPurpose);
 	GPIO_setDirection(pobj->gpioHandle,GPIO_Number_57,GPIO_Direction_Output);

 	// Reply DO
 	GPIO_setMode(pobj->gpioHandle,GPIO_Number_58,GPIO_58_Mode_GeneralPurpose);
 	GPIO_setDirection(pobj->gpioHandle,GPIO_Number_58,GPIO_Direction_Output);


}  // end of HAL_setupGpios() function


void HAL_setupPie(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;


	PIE_disable(pobj->pieHandle);

	PIE_disableAllInts(pobj->pieHandle);

	PIE_clearAllInts(pobj->pieHandle);

 	PIE_clearAllFlags(pobj->pieHandle);

 	PIE_setDefaultIntVectorTable(pobj->pieHandle);

 	PIE_enable(pobj->pieHandle);

} // end of HAL_setupPie() function


void HAL_setupPeripheralClks(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;


	CLK_enableAdcClock(pobj->clkHandle);

	CLK_enableCompClock(pobj->clkHandle,CLK_CompNumber_1);
	CLK_enableCompClock(pobj->clkHandle,CLK_CompNumber_2);
	CLK_enableCompClock(pobj->clkHandle,CLK_CompNumber_3);

	CLK_enableEcap1Clock(pobj->clkHandle);
	CLK_disableEcanaClock(pobj->clkHandle);

#ifdef QEP
	CLK_enableEqep1Clock(pobj->clkHandle);
	//CLK_disableEqep2Clock(pobj->clkHandle);
#endif

	CLK_enablePwmClock(pobj->clkHandle,PWM_Number_1);
	CLK_enablePwmClock(pobj->clkHandle,PWM_Number_2);
	CLK_enablePwmClock(pobj->clkHandle,PWM_Number_3);
	CLK_enablePwmClock(pobj->clkHandle,PWM_Number_4);
	CLK_enablePwmClock(pobj->clkHandle,PWM_Number_5);
	CLK_enablePwmClock(pobj->clkHandle,PWM_Number_6);
	CLK_enablePwmClock(pobj->clkHandle,PWM_Number_7);

	CLK_disableHrPwmClock(pobj->clkHandle);

	//CLK_disableI2cClock(pobj->clkHandle);
	CLK_enableI2cClock(pobj->clkHandle);

	CLK_disableLinAClock(pobj->clkHandle);

	CLK_disableClaClock(pobj->clkHandle);

	//CLK_disableSciaClock(pobj->clkHandle);
	CLK_enableSciaClock(pobj->clkHandle);

	CLK_disableScibClock(pobj->clkHandle);


	CLK_disableSpiaClock(pobj->clkHandle);

	CLK_disableSpibClock(pobj->clkHandle);
  
	CLK_enableTbClockSync(pobj->clkHandle);

} // end of HAL_setupPeripheralClks() function


void HAL_setupPll(HAL_Handle handle,const PLL_ClkFreq_e clkFreq)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;


	// make sure PLL is not running in limp mode
	if(PLL_getClkStatus(pobj->pllHandle) != PLL_ClkStatus_Normal)
    {
		// reset the clock detect
		PLL_resetClkDetect(pobj->pllHandle);

		// ???????
		asm("        ESTOP0");
    }


  // Divide Select must be ClkIn/4 before the clock rate can be changed
	if(PLL_getDivideSelect(pobj->pllHandle) != PLL_DivideSelect_ClkIn_by_4)
    {
		PLL_setDivideSelect(pobj->pllHandle,PLL_DivideSelect_ClkIn_by_4);
    }


	if(PLL_getClkFreq(pobj->pllHandle) != clkFreq)
    {
		// disable the clock detect
		PLL_disableClkDetect(pobj->pllHandle);

		// set the clock rate
		PLL_setClkFreq(pobj->pllHandle,clkFreq);
    }


	// wait until locked
	while(PLL_getLockStatus(pobj->pllHandle) != PLL_LockStatus_Done) {}


	// enable the clock detect
	PLL_enableClkDetect(pobj->pllHandle);


	// set divide select to ClkIn/2 to get desired clock rate
	// NOTE: clock must be locked before setting this register
	PLL_setDivideSelect(pobj->pllHandle,PLL_DivideSelect_ClkIn_by_2);
;
} // end of HAL_setupPll() function


void HAL_setupPwms(HAL_Handle handle,
                   const uint_least16_t u16systemFreq_MHz,
                   const float_t fpwmPeriod_usec,
                   const uint_least16_t u16numPwmTicksPerIsrTick)
{
	HAL_Obj   *pobj = (HAL_Obj *)handle;
	uint16_t   u16halfPeriod_cycles = (uint16_t)((float_t)u16systemFreq_MHz*fpwmPeriod_usec) >> 1;
	uint_least8_t    u8cnt;


	// turns off the outputs of the EPWM peripherals which will put the power switches
	// into a high impedance state.
	PWM_setOneShotTrip(pobj->pwmHandle[PWM_Number_1]);
	PWM_setOneShotTrip(pobj->pwmHandle[PWM_Number_2]);
	PWM_setOneShotTrip(pobj->pwmHandle[PWM_Number_3]);

	for(u8cnt=0;u8cnt<3;u8cnt++)
    {
		// setup the Time-Base Control Register (TBCTL)
		PWM_setCounterMode(pobj->pwmHandle[u8cnt],PWM_CounterMode_UpDown);
		PWM_disableCounterLoad(pobj->pwmHandle[u8cnt]);
		PWM_setPeriodLoad(pobj->pwmHandle[u8cnt],PWM_PeriodLoad_Immediate);
		PWM_setSyncMode(pobj->pwmHandle[u8cnt],PWM_SyncMode_EPWMxSYNC);
		PWM_setHighSpeedClkDiv(pobj->pwmHandle[u8cnt],PWM_HspClkDiv_by_1);
		PWM_setClkDiv(pobj->pwmHandle[u8cnt],PWM_ClkDiv_by_1);
		PWM_setPhaseDir(pobj->pwmHandle[u8cnt],PWM_PhaseDir_CountUp);
		PWM_setRunMode(pobj->pwmHandle[u8cnt],PWM_RunMode_FreeRun);

		// setup the Timer-Based Phase Register (TBPHS)
		PWM_setPhase(pobj->pwmHandle[u8cnt],0);

		// setup the Time-Base Counter Register (TBCTR)
		PWM_setCount(pobj->pwmHandle[u8cnt],0);

		// setup the Time-Base Period Register (TBPRD)
		// set to zero initially
		PWM_setPeriod(pobj->pwmHandle[u8cnt],0);

		// setup the Counter-Compare Control Register (CMPCTL)
		PWM_setLoadMode_CmpA(pobj->pwmHandle[u8cnt],PWM_LoadMode_Zero);
		PWM_setLoadMode_CmpB(pobj->pwmHandle[u8cnt],PWM_LoadMode_Zero);
		PWM_setShadowMode_CmpA(pobj->pwmHandle[u8cnt],PWM_ShadowMode_Shadow);
		PWM_setShadowMode_CmpB(pobj->pwmHandle[u8cnt],PWM_ShadowMode_Immediate);

		// setup the Action-Qualifier Output A Register (AQCTLA)
		PWM_setActionQual_CntUp_CmpA_PwmA(pobj->pwmHandle[u8cnt],PWM_ActionQual_Set);
		PWM_setActionQual_CntDown_CmpA_PwmA(pobj->pwmHandle[u8cnt],PWM_ActionQual_Clear);

		// setup the Dead-Band Generator Control Register (DBCTL)
		PWM_setDeadBandOutputMode(pobj->pwmHandle[u8cnt],PWM_DeadBandOutputMode_EPWMxA_Rising_EPWMxB_Falling);
		//PWM_setDeadBandPolarity(pobj->pwmHandle[u8cnt],PWM_DeadBandPolarity_EPWMxB_Inverted);		//Org
		PWM_setDeadBandPolarity(pobj->pwmHandle[u8cnt],PWM_DeadBandPolarity_EPWMxA_Inverted);		//Adding

		// setup the Dead-Band Rising Edge Delay Register (DBRED)
		PWM_setDeadBandRisingEdgeDelay(pobj->pwmHandle[u8cnt],HAL_PWM_DBRED_CNT);

		// setup the Dead-Band Falling Edge Delay Register (DBFED)
		PWM_setDeadBandFallingEdgeDelay(pobj->pwmHandle[u8cnt],HAL_PWM_DBFED_CNT);
		// setup the PWM-Chopper Control Register (PCCTL)
		PWM_disableChopping(pobj->pwmHandle[u8cnt]);

		// setup the Trip Zone Select Register (TZSEL)
		PWM_disableTripZones(pobj->pwmHandle[u8cnt]);
    }


	// setup the Event Trigger Selection Register (ETSEL)
	PWM_disableInt(pobj->pwmHandle[PWM_Number_1]);
	PWM_setSocAPulseSrc(pobj->pwmHandle[PWM_Number_1],PWM_SocPulseSrc_CounterEqualZero);	//Org
	//PWM_setSocAPulseSrc(pobj->pwmHandle[PWM_Number_1],PWM_SocPulseSrc_CounterEqualPeriod);	//Modify

	PWM_enableSocAPulse(pobj->pwmHandle[PWM_Number_1]);
  

	// setup the Event Trigger Prescale Register (ETPS)
	if(u16numPwmTicksPerIsrTick == 3)
    {
		PWM_setIntPeriod(pobj->pwmHandle[PWM_Number_1],PWM_IntPeriod_ThirdEvent);
		PWM_setSocAPeriod(pobj->pwmHandle[PWM_Number_1],PWM_SocPeriod_ThirdEvent);
    }
	else if(u16numPwmTicksPerIsrTick == 2)
    {
		PWM_setIntPeriod(pobj->pwmHandle[PWM_Number_1],PWM_IntPeriod_SecondEvent);
		PWM_setSocAPeriod(pobj->pwmHandle[PWM_Number_1],PWM_SocPeriod_SecondEvent);
    }
	else
    {
		PWM_setIntPeriod(pobj->pwmHandle[PWM_Number_1],PWM_IntPeriod_FirstEvent);
     	PWM_setSocAPeriod(pobj->pwmHandle[PWM_Number_1],PWM_SocPeriod_FirstEvent);
    }


	// setup the Event Trigger Clear Register (ETCLR)
	PWM_clearIntFlag(pobj->pwmHandle[PWM_Number_1]);
	PWM_clearSocAFlag(pobj->pwmHandle[PWM_Number_1]);

	// first step to synchronize the pwms
	CLK_disableTbClockSync(pobj->clkHandle);

	// since the PWM is configured as an up/down counter, the period register is set to one-half
	// of the desired PWM period
 	PWM_setPeriod(pobj->pwmHandle[PWM_Number_1],u16halfPeriod_cycles);
 	PWM_setPeriod(pobj->pwmHandle[PWM_Number_2],u16halfPeriod_cycles);
 	PWM_setPeriod(pobj->pwmHandle[PWM_Number_3],u16halfPeriod_cycles);

 	// last step to synchronize the pwms
 	CLK_enableTbClockSync(pobj->clkHandle);


}  // end of HAL_setupPwms() function





#ifdef QEP
void HAL_setupQEP(HAL_Handle handle,HAL_QepSelect_e qep)
{
	HAL_Obj   *pobj = (HAL_Obj *)handle;


	// hold the counter in reset
	QEP_reset_counter(pobj->qepHandle[qep]);

	// set the QPOSINIT register
	QEP_set_posn_init_count(pobj->qepHandle[qep], 0);

	// disable all interrupts
	QEP_disable_all_interrupts(pobj->qepHandle[qep]);

	// clear the interrupt flags
	QEP_clear_all_interrupt_flags(pobj->qepHandle[qep]);

	// clear the position counter
	QEP_clear_posn_counter(pobj->qepHandle[qep]);

	// setup the max position
	QEP_set_max_posn_count(pobj->qepHandle[qep], (4*USER_MOTOR_ENCODER_LINES)-1);

	// setup the QDECCTL register
	QEP_set_QEP_source(pobj->qepHandle[qep], QEP_Qsrc_Quad_Count_Mode);
	QEP_disable_sync_out(pobj->qepHandle[qep]);
	QEP_set_swap_quad_inputs(pobj->qepHandle[qep], QEP_Swap_Not_Swapped);
	QEP_disable_gate_index(pobj->qepHandle[qep]);
 	QEP_set_ext_clock_rate(pobj->qepHandle[qep], QEP_Xcr_2x_Res);
 	QEP_set_A_polarity(pobj->qepHandle[qep], QEP_Qap_No_Effect);
 	QEP_set_B_polarity(pobj->qepHandle[qep], QEP_Qbp_No_Effect);
 	QEP_set_index_polarity(pobj->qepHandle[qep], QEP_Qip_No_Effect);

 	// setup the QEPCTL register
 	QEP_set_emu_control(pobj->qepHandle[qep], QEPCTL_Freesoft_Unaffected_Halt);
 	QEP_set_posn_count_reset_mode(pobj->qepHandle[qep], QEPCTL_Pcrm_Max_Reset);
 	QEP_set_strobe_event_init(pobj->qepHandle[qep], QEPCTL_Sei_Nothing);
 	QEP_set_index_event_init(pobj->qepHandle[qep], QEPCTL_Iei_Nothing);
 	QEP_set_index_event_latch(pobj->qepHandle[qep], QEPCTL_Iel_Rising_Edge);
 	QEP_set_soft_init(pobj->qepHandle[qep], QEPCTL_Swi_Nothing);
 	QEP_disable_unit_timer(pobj->qepHandle[qep]);
 	QEP_disable_watchdog(pobj->qepHandle[qep]);

 	// setup the QPOSCTL register
 	QEP_disable_posn_compare(pobj->qepHandle[qep]);

 	// setup the QCAPCTL register
 	QEP_disable_capture(pobj->qepHandle[qep]);

 	// renable the position counter
 	QEP_enable_counter(pobj->qepHandle[qep]);



}
#endif

void HAL_setupPwmDacs(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
	uint16_t u16halfPeriod_cycles = 512;       // 3000->10kHz, 1500->20kHz, 1000-> 30kHz, 500->60kHz
	uint_least8_t    u8cnt;


	for(u8cnt=0;u8cnt<3;u8cnt++)
    {
		// initialize the Time-Base Control Register (TBCTL)
		PWMDAC_setCounterMode(pobj->pwmDacHandle[u8cnt],PWM_CounterMode_UpDown);
		PWMDAC_disableCounterLoad(pobj->pwmDacHandle[u8cnt]);
		PWMDAC_setPeriodLoad(pobj->pwmDacHandle[u8cnt],PWM_PeriodLoad_Immediate);
		PWMDAC_setSyncMode(pobj->pwmDacHandle[u8cnt],PWM_SyncMode_EPWMxSYNC);
		PWMDAC_setHighSpeedClkDiv(pobj->pwmDacHandle[u8cnt],PWM_HspClkDiv_by_1);
		PWMDAC_setClkDiv(pobj->pwmDacHandle[u8cnt],PWM_ClkDiv_by_1);
		PWMDAC_setPhaseDir(pobj->pwmDacHandle[u8cnt],PWM_PhaseDir_CountUp);
		PWMDAC_setRunMode(pobj->pwmDacHandle[u8cnt],PWM_RunMode_FreeRun);

     	// initialize the Timer-Based Phase Register (TBPHS)
		PWMDAC_setPhase(pobj->pwmDacHandle[u8cnt],0);

		// setup the Time-Base Counter Register (TBCTR)
     	PWMDAC_setCount(pobj->pwmDacHandle[u8cnt],0);

     	// Initialize the Time-Base Period Register (TBPRD)
     	// set to zero initially
     	PWMDAC_setPeriod(pobj->pwmDacHandle[u8cnt],0);

     	// initialize the Counter-Compare Control Register (CMPCTL)
     	PWMDAC_setLoadMode_CmpA(pobj->pwmDacHandle[u8cnt],PWM_LoadMode_Zero);
     	PWMDAC_setLoadMode_CmpB(pobj->pwmDacHandle[u8cnt],PWM_LoadMode_Zero);
     	PWMDAC_setShadowMode_CmpA(pobj->pwmDacHandle[u8cnt],PWM_ShadowMode_Shadow);
     	PWMDAC_setShadowMode_CmpB(pobj->pwmDacHandle[u8cnt],PWM_ShadowMode_Shadow);

     	// Initialize the Action-Qualifier Output A Register (AQCTLA)
     	PWMDAC_setActionQual_CntUp_CmpA_PwmA(pobj->pwmDacHandle[u8cnt],PWM_ActionQual_Clear);
     	PWMDAC_setActionQual_CntDown_CmpA_PwmA(pobj->pwmDacHandle[u8cnt],PWM_ActionQual_Set);

     	// account for EPWM6B
     	if(u8cnt == 0)
        {
     		PWMDAC_setActionQual_CntUp_CmpB_PwmB(pobj->pwmDacHandle[u8cnt],PWM_ActionQual_Clear);
     		PWMDAC_setActionQual_CntDown_CmpB_PwmB(pobj->pwmDacHandle[u8cnt],PWM_ActionQual_Set);
        }

     	// Initialize the Dead-Band Control Register (DBCTL)
     	PWMDAC_disableDeadBand(pobj->pwmDacHandle[u8cnt]);

     	// Initialize the PWM-Chopper Control Register (PCCTL)
     	PWMDAC_disableChopping(pobj->pwmDacHandle[u8cnt]);

     	// Initialize the Trip-Zone Control Register (TZSEL)
     	PWMDAC_disableTripZones(pobj->pwmDacHandle[u8cnt]);

     	// Initialize the Trip-Zone Control Register (TZCTL)
     	PWMDAC_setTripZoneState_TZA(pobj->pwmDacHandle[u8cnt],PWM_TripZoneState_HighImp);
     	PWMDAC_setTripZoneState_TZB(pobj->pwmDacHandle[u8cnt],PWM_TripZoneState_HighImp);
     	PWMDAC_setTripZoneState_DCAEVT1(pobj->pwmDacHandle[u8cnt],PWM_TripZoneState_HighImp);
     	PWMDAC_setTripZoneState_DCAEVT2(pobj->pwmDacHandle[u8cnt],PWM_TripZoneState_HighImp);
     	PWMDAC_setTripZoneState_DCBEVT1(pobj->pwmDacHandle[u8cnt],PWM_TripZoneState_HighImp);
    }

	// since the PWM is configured as an up/down counter, the period register is set to one-half
	// of the desired PWM period
	PWMDAC_setPeriod(pobj->pwmDacHandle[PWMDAC_Number_1],u16halfPeriod_cycles);
	PWMDAC_setPeriod(pobj->pwmDacHandle[PWMDAC_Number_2],u16halfPeriod_cycles);
	PWMDAC_setPeriod(pobj->pwmDacHandle[PWMDAC_Number_3],u16halfPeriod_cycles);


}  // end of HAL_setupPwmDacs() function


void HAL_setupTimers(HAL_Handle handle,const uint_least16_t u16systemFreq_MHz)
{
	HAL_Obj  *pobj = (HAL_Obj *)handle;

	//USER_TIMER_PERIOD_msec
	uint32_t  u32timerPeriod_cnts = (((uint32_t)u16systemFreq_MHz) * 1000)*USER_TIMER_PERIOD_msec - 1;

	// use timer 0 for frequency diagnostics
	TIMER_setDecimationFactor(pobj->timerHandle[0],0);
	TIMER_setEmulationMode(pobj->timerHandle[0],TIMER_EmulationMode_RunFree);
	TIMER_setPeriod(pobj->timerHandle[0],u32timerPeriod_cnts);	 //USER_TIMER_PERIOD_msec
	TIMER_setPreScaler(pobj->timerHandle[0],0);

	//u32timerPeriod_cnts = ((uint32_t)u16systemFreq_MHz * 1000000) - 1;
	// use timer 1 for CPU usage diagnostics
	TIMER_setDecimationFactor(pobj->timerHandle[1],0);
	TIMER_setEmulationMode(pobj->timerHandle[1],TIMER_EmulationMode_RunFree);
	TIMER_setPeriod(pobj->timerHandle[1],u32timerPeriod_cnts);
	TIMER_setPreScaler(pobj->timerHandle[1],0);


}  // end of HAL_setupTimers() function

void HAL_setupI2C(HAL_Handle handle)
{
	HAL_Obj  *pobj = (HAL_Obj *)handle;
	I2C_Handle i2cHandle = pobj->i2cHandle;

	I2C_setPrescaler(i2cHandle,7);		//I2caRegs.I2CPSC.all = 6; Prescaler - need 7-12 Mhz on module clk
	I2C_setClockLowTime(i2cHandle, 45);	//I2caRegs.I2CCLKL = 10; NOTE: must be non zero
	I2C_setClockHighTime(i2cHandle, 50);	//I2caRegs.I2CCLKH = 5; NOTE: must be non zero


	I2C_Reset(i2cHandle);
	//I2caRegs.I2CMDR.all = 0x0420;	// Take I2C out of reset
									// Stop I2C when suspended
	//I2C_disableNAck(i2cHandle);
	I2C_disableFreeMode(i2cHandle);
	I2C_setMode(i2cHandle, I2C_Mode_Master);
	//I2C_setTxRxMode(i2cHandle, I2C_Mode_Receive);
	//I2C_setAddressMode(i2cHandle, I2C_Mode_7Address);
	//I2C_setRepeatMode(i2cHandle,  I2C_Mode_NonRepeat);
	//I2C_disableLookbackMode(i2cHandle);
	//I2C_disableStartByteMode(i2cHandle);
	//I2C_disableFreeDataFormat(i2cHandle);
	//I2C_setBitCountBits(i2cHandle,I2C_CharLength_8_Bits);
	I2C_Enable(i2cHandle);




	//I2caRegs.I2CFFTX.all = 0x6000;	// Enable FIFO mode and TXFIFO
	I2C_enableFifoMode(i2cHandle);
	I2C_enableTxFifo(i2cHandle);
	//I2C_enableTxFifoInt(i2cHandle);
	I2C_setTxFifoIntLevel(i2cHandle, I2C_FifoLevel_Empty);
	//I2C_clearTxFifoIntFlag(i2cHandle);

	//I2caRegs.I2CFFRX.all = 0x2040;	// Enable RXFIFO, clear RXFFINT,
    I2C_enableRxFifo(i2cHandle);
    //I2C_enableRxFifoInt(i2cHandle);
    I2C_setRxFifoIntLevel(i2cHandle, I2C_FifoLevel_4_Words);
    //I2C_clearRxFifoIntFlag(i2cHandle);

    //I2C_StartCond(i2cHandle);
    //I2C_StopCond(i2cHandle);	//I2caRegs.I2CMDR.bit.STP = 1;
    //while (I2C_getStopStatus(i2cHandle));	//Wait for STP = 0 automatic


    I2C_disableAddSlaveInt(i2cHandle);		//I2caRegs.I2CIER.all = 0x24;		// Enable SCD & ARDY interrupts
    I2C_enableStopCondDectInt(i2cHandle);
    I2C_disableTXReadyInt(i2cHandle);
    I2C_disableRXReadyInt(i2cHandle);
    I2C_enableRegAccessReadyInt(i2cHandle);
    //I2C_disableNoAckInt(i2cHandle);
    I2C_enableNoAckInt(i2cHandle);
    I2C_disableArbiLostInt(i2cHandle);

}

void HAL_setupSCI(HAL_Handle handle)
{
	HAL_Obj  *pobj = (HAL_Obj *)handle;
	uint_least8_t    u8Cnt;

	for (u8Cnt=0; u8Cnt<2; u8Cnt++)
	{
		SCI_Handle sciHandle= pobj->sciHandle[u8Cnt];
		//SCI_Message *pSciMessage = &(g_sciMessage[u8Cnt]);
		//SciaRegs.SCIFFCT.all=0x0;
		SCI_disableAutoBaudAlign(sciHandle);
		SCI_setTxDelay(sciHandle, 0);

		//SciaRegs.SCICCR.all =0x0007;    // 1 stop bit,  No loopback
				                          // No parity,8 char bits,
				                          // async mode, idle-line protocol
		SCI_setNumStopBits(sciHandle, SCI_NumStopBits_One);
		SCI_disableParity(sciHandle);
		//SCI_disableLoopBack(sciHandle);
		//SCI_enableLoopBack(sciHandle);

		SCI_setMode(sciHandle, SCI_Mode_IdleLine);
		SCI_setCharLength(sciHandle,SCI_CharLength_8_Bits );

		//SciaRegs.SCICTL1.all =0x0003;  	// enable TX, RX, internal SCICLK,
			                                // Disable RX ERR, SLEEP, TXWAKE
		//SCI_disableRxErrorInt(sciHandle);
		SCI_enableRxErrorInt(sciHandle);
		SCI_reset(sciHandle);
		SCI_disableTxWake(sciHandle);
		SCI_disableSleep(sciHandle);
		SCI_enableTx(sciHandle);
		SCI_enableRx(sciHandle);

		//SciaRegs.SCICTL2.all =0x0003;
		//SciaRegs.SCICTL2.bit.TXINTENA =1;
		//SciaRegs.SCICTL2.bit.RXBKINTENA =1;
		//SCI_enableRxInt(sciHandle);
		//SCI_enableTxInt(sciHandle);

		SCI_setBaudRate(sciHandle, SCI_BaudRate_9_6_kBaud);	//SCIHBUD, SCILBAUD

		//SciaRegs.SCIFFTX.all=0xc022 (0xE040);
		SCI_enableChannels(sciHandle);
		SCI_enableFifo(sciHandle);

		SCI_resetTxFifo(sciHandle);		//
		//SCI_clearTxFifoInt(pObj->sciAHandle[u8Cnt]);
		//SCI_enableTxFifoInt(sciHandle);	//SCI_disableTxFifoInt(pObj->sciAHandle);
		SCI_setTxFifoIntLevel(sciHandle, SCI_FifoLevel_Empty);
		SCI_enableTxFifo(sciHandle);


		//SciaRegs.SCIFFRX.all=0x0022 (0x2044);

		SCI_resetRxFifo(sciHandle);		//
		//SCI_enableRxFifoInt(sciHandle);   //SCI_disableRxFifoInt(0bj->sciAHandle);
		SCI_setRxFifoIntLevel(sciHandle, SCI_FifoLevel_4_Words);
		SCI_clearRxFifoInt(sciHandle);
		SCI_enableRxFifo(sciHandle);

		//SCI_enableLoopBack(pObj->sciAHandle);	// Enable loop back
		SCI_enable(sciHandle);			// SciaRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset
		//SCI_enableTxFifo(sciHandle);
		//SCI_enableRxFifo(sciHandle);



	}
}

void HAL_setupSPI(HAL_Handle handle)
{
	//HAL_Obj *obj = (HAL_Obj *)handle;
}

void HAL_readAdcDataApx(HAL_Handle handle,HAL_AdcData_t *pAdcData)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	_iq iqvalue;
	_iq iqExtAdc_sf = HAL_getExtAdcScaleFactor(handle);


	iqvalue = (_iq)ADC_readResult(pobj->adcHandle,ADC_ResultNumber_8);     // divide by 2^numAdcBits = 2^12
	iqvalue = _IQ12mpy(iqvalue,HAL_getExtTempScaleFactor(handle));
	pAdcData->iqExtTemp = iqvalue;

	iqvalue = (_iq)ADC_readResult(pobj->adcHandle,ADC_ResultNumber_9);     // divide by 2^numAdcBits = 2^12
	iqvalue = _IQ12mpy(iqvalue,iqExtAdc_sf);
	pAdcData->iqExtAdc[0] = iqvalue;

	iqvalue = (_iq)ADC_readResult(pobj->adcHandle,ADC_ResultNumber_10);     // divide by 2^numAdcBits = 2^12
	iqvalue = _IQ12mpy(iqvalue,iqExtAdc_sf);
	pAdcData->iqExtAdc[1] = iqvalue;

	iqvalue = (_iq)ADC_readResult(pobj->adcHandle,ADC_ResultNumber_11);     // divide by 2^numAdcBits = 2^12
	iqvalue = _IQ12mpy(iqvalue,iqExtAdc_sf);
	pAdcData->iqExtAdc[2] = iqvalue;

	iqvalue = (_iq)ADC_readResult(pobj->adcHandle,ADC_ResultNumber_12);     // divide by 2^numAdcBits = 2^12
	iqvalue = _IQ12mpy(iqvalue,iqExtAdc_sf);
	pAdcData->iqExtAdc[3] = iqvalue;

	iqvalue = (_iq)ADC_readResult(pobj->adcHandle,ADC_ResultNumber_13);     // divide by 2^numAdcBits = 2^12
	iqvalue = _IQ12mpy(iqvalue,iqExtAdc_sf);
	pAdcData->iqExtAdc[4] = iqvalue;

	iqvalue = (_iq)ADC_readResult(pobj->adcHandle,ADC_ResultNumber_14);     // divide by 2^numAdcBits = 2^12
	iqvalue = _IQ12mpy(iqvalue,iqExtAdc_sf);
	pAdcData->iqExtAdc[5] = iqvalue;


} // end of HAL_readAdcData() function
// end of file


//$ this is for the showing datalog
//$ set the DAC parameters
void HAL_setDacParameters(HAL_Handle handle, HAL_DacData_t *pDacData)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    pDacData->PeriodMax = PWMDAC_getPeriod(obj->pwmDacHandle[PWMDAC_Number_1]);


    pDacData->offset[0] = _IQ(0.5);
    pDacData->offset[1] = _IQ(0.5);
    pDacData->offset[2] = _IQ(0.5);
    pDacData->offset[3] = _IQ(0.5);

    pDacData->gain[0] = _IQ(1.0);
    pDacData->gain[1] = _IQ(1.0);
    pDacData->gain[2] = _IQ(1.0);
    pDacData->gain[3] = _IQ(1.0);
    //for svgen


    //$for angel_gen current sampling PWM output
   /*
    pDacData->offset[0] = _IQ(0);
    pDacData->offset[1] = _IQ(0);
    pDacData->offset[2] = _IQ(0.5);
    pDacData->offset[3] = _IQ(0.5);

    pDacData->gain[0] = _IQ(1.0);
    pDacData->gain[1] = _IQ(1.0);
    pDacData->gain[2] = _IQ(1.0);
    pDacData->gain[3] = _IQ(1.0);
    */



    return;
}   //end of HAL_setDacParameters() function




// end of file
