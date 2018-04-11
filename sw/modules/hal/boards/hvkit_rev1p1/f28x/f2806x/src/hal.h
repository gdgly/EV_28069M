#ifndef _HAL_H_
#define _HAL_H_
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

//! \file   solutions/instaspin_foc/boards/hvkit_rev1p1/f28x/f2806xF/src/hal.h
//! \brief  Contains public interface to various functions related
//!         to the HAL object
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes


// modules


// platforms
#include "hal_obj.h"


//!
//!
//! \defgroup HAL HAL
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

#define Device_cal (void   (*)(void))0x3D7C80

//! \brief Defines used in oscillator calibration functions
//! \brief Defines the scale factor for Q15 fixed point numbers (2^15)
#define FP_SCALE 32768

//! \brief Defines the quantity added to Q15 numbers before converting to integer to round the number
#define FP_ROUND FP_SCALE/2

//! \brief Defines the amount to add to Q16.15 fixed point number to shift from a fine trim range of
//! \brief (-31 to 31) to (1 to 63).  This guarantees that the trim is positive and can
//! \brief therefore be efficiently rounded
#define OSC_POSTRIM 32
#define OSC_POSTRIM_OFF FP_SCALE*OSC_POSTRIM

//! \brief The following functions return reference values stored in OTP.

//! \brief Defines the slope used to compensate oscillator 1 (fine trim steps / ADC code). Stored in fixed point Q15 format
#define getOsc1FineTrimSlope() (*(int16_t (*)(void))0x3D7E90)()

//! \brief Defines the oscillator 1 fine trim at high temp
#define getOsc1FineTrimOffset() (*(int16_t (*)(void))0x3D7E93)()

//! \brief Defines the oscillator 1 coarse trim
#define getOsc1CoarseTrim() (*(int16_t (*)(void))0x3D7E96)()

//! \brief Defines the slope used to compensate oscillator 2 (fine trim steps / ADC code). Stored
//! \brief in fixed point Q15 format.
#define getOsc2FineTrimSlope() (*(int16_t (*)(void))0x3D7E99)()

//! \brief Defines the oscillator 2 fine trim at high temp
#define getOsc2FineTrimOffset() (*(int16_t (*)(void))0x3D7E9C)()

//! \brief Defines the oscillator 2 coarse trim
#define getOsc2CoarseTrim() (*(int16_t (*)(void))0x3D7E9F)()

//! \brief Defines the ADC reading of temperature sensor at reference temperature for compensation
#define getRefTempOffset() (*(int16_t (*)(void))0x3D7EA2)()

//! \brief Defines the PWM deadband falling edge delay count (system clocks)
//!
//#define HAL_PWM_DBFED_CNT         (uint16_t)(1.6 * (float_t)USER_SYSTEM_FREQ_MHz)            //
//#define HAL_PWM_DBFED_CNT         (uint16_t)(2.0 * (float_t)USER_SYSTEM_FREQ_MHz)            // 1.2 usec
//#define HAL_PWM_DBFED_CNT         (uint16_t)(3.0 * (float_t)USER_SYSTEM_FREQ_MHz)            // 2 usec Modify
//#define HAL_PWM_DBFED_CNT         (uint16_t)(4.0 * (float_t)USER_SYSTEM_FREQ_MHz)            // 4 usec Modify
#define HAL_PWM_DBFED_CNT         (uint16_t)(8.0 * (float_t)USER_SYSTEM_FREQ_MHz)            // 8 usec Modify

//! \brief Defines the PWM deadband rising edge delay count (system clocks)
//!
//#define HAL_PWM_DBRED_CNT        (uint16_t)(1.6 * (float_t)USER_SYSTEM_FREQ_MHz)            //
//#define HAL_PWM_DBRED_CNT        (uint16_t)(2.0 * (float_t)USER_SYSTEM_FREQ_MHz)            // 1.2 usec
//#define HAL_PWM_DBRED_CNT        (uint16_t)(3.0 * (float_t)USER_SYSTEM_FREQ_MHz)            // 2 usec Modify
//#define HAL_PWM_DBRED_CNT         (uint16_t)(4.0 * (float_t)USER_SYSTEM_FREQ_MHz)            // 4 usec Modify
#define HAL_PWM_DBRED_CNT         (uint16_t)(8.0 * (float_t)USER_SYSTEM_FREQ_MHz)            // 8 usec Modify


//! \brief Defines the function to turn LEDs off
//!
#define HAL_turnLedOff            HAL_setGpioLow


//! \brief Defines the function to turn LEDs on
//!
#define HAL_turnLedOn             HAL_setGpioHigh


//! \brief Defines the function to turn LEDs on
//!
#define HAL_toggleLed             HAL_toggleGpio

// **************************************************************************
// the typedefs


//! \brief Enumeration for the QEP setup
//!
typedef enum
{
  HAL_Qep_QEP1=0,  //!< Select QEP1
  HAL_Qep_QEP2=1   //!< Select QEP2
} HAL_QepSelect_e;


//! \brief Enumeration for the LED numbers
//!
typedef enum
{
  HAL_Gpio_LED2=GPIO_Number_31,  //!< GPIO pin number for ControlCARD LED 2
  HAL_Gpio_LED3=GPIO_Number_34   //!< GPIO pin number for ControlCARD LED 3
} HAL_LedNumber_e;
  

//! \brief Enumeration for the sensor types
//!
typedef enum
{
  HAL_SensorType_Current=0,      //!< Enumeration for current sensor
  HAL_SensorType_Voltage         //!< Enumeration for voltage sensor
} HAL_SensorType_e;


// **************************************************************************
// the globals

extern interrupt void adcInt1ISR(void);
extern interrupt void adcInt2ISR(void);
extern interrupt void timer0ISR(void);
extern interrupt void timer1ISR(void);
extern interrupt void i2cInt1AISR(void);
extern interrupt void i2cInt2AISR(void);
extern interrupt void sciATxFifoISR(void);
extern interrupt void sciARxFifoISR(void);
extern interrupt void sciBTxFifoISR(void);
extern interrupt void sciBRxFifoISR(void);



/*static inline bool HAL_getAdcIntFlag(HAL_Handle handle, const ADC_IntNumber_e intNumber)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
	return ADC_getIntFlag(pobj->adcHandle,intNumber);
}*/
// **************************************************************************
// the function prototypes
//! \brief     Acknowledges an interrupt from the ADC so that another ADC interrupt can 
//!            happen again.
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \param[in] intNumber  The interrupt number
static inline void HAL_acqAdcInt(HAL_Handle handle,const ADC_IntNumber_e intNumber)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;


	// clear the ADC interrupt flag
	ADC_clearIntFlag(pobj->adcHandle,intNumber);//$in the mainISR of the main.c ,intNumber=0


	// Acknowledge interrupt from PIE group 10
	//PIE_clearInt(pobj->pieHandle,PIE_GroupNumber_10);
	PIE_clearInt(pobj->pieHandle,PIE_GroupNumber_1);    //$peripheral interrupt expansion(PIE)


} // end of HAL_acqAdcInt() function



//! \brief     Acknowledges an interrupt from the PWM so that another PWM interrupt can
//!            happen again.
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \param[in] pwmNumber  The PWM number
static inline void HAL_acqPwmInt(HAL_Handle handle,const PWM_Number_e pwmNumber)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;


	// clear the PWM interrupt flag
	PWM_clearIntFlag(pobj->pwmHandle[pwmNumber]);


	// clear the SOCA flag
	PWM_clearSocAFlag(pobj->pwmHandle[pwmNumber]);


	// Acknowledge interrupt from PIE group 3
	PIE_clearInt(pobj->pieHandle,PIE_GroupNumber_3);


} // end of HAL_acqPwmInt() function

//! \brief     Acknowledges an interrupt from Timer 0 so that another Timer 0 interrupt can
//!            happen again.
//! \param[in] handle     The hardware abstraction layer (HAL) handle static inline
static inline void HAL_acqTimer0Int(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	// clear the Timer 0 interrupt flag
	TIMER_clearFlag(pobj->timerHandle[0]);

	// Acknowledge interrupt from PIE group 1
	PIE_clearInt(pobj->pieHandle,PIE_GroupNumber_1);

} // end of HAL_acqTimer0Int() function

static inline void HAL_acqTimer1Int(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	// clear the Timer 0 interrupt flag
	TIMER_clearFlag(pobj->timerHandle[1]);

	// Acknowledge interrupt from PIE group 1
	//PIE_clearInt(pobj->pieHandle,PIE_GroupNumber_1);

} // end of HAL_acqTimer0Int() function

static inline void HAL_acqI2cInt(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	// Acknowledge interrupt from PIE group 8
	PIE_clearInt(pobj->pieHandle,PIE_GroupNumber_8);

	// Enable future I2C (PIE Group 8) interrupts
    //PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}

static inline void HAL_acqSciATxInt(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
	// clear the ADC interrupt flag
	SCI_clearTxFifoInt(pobj->sciHandle[0]);

	// Acknowledge interrupt from PIE group 9
	PIE_clearInt(pobj->pieHandle,PIE_GroupNumber_9);

} // end of HAL_acqSciATxInt() function

static inline void HAL_acqSciARxInt(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
	// clear the ADC interrupt flag
	SCI_clearRxFifoInt(pobj->sciHandle[0]);

	// Acknowledge interrupt from PIE group 9
	PIE_clearInt(pobj->pieHandle,PIE_GroupNumber_9);

} // end of HAL_acqSciARxInt() function

static inline void HAL_acqSciBTxInt(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
	// clear the ADC interrupt flag
	SCI_clearTxFifoInt(pobj->sciHandle[1]);

	// Acknowledge interrupt from PIE group 9
	PIE_clearInt(pobj->pieHandle,PIE_GroupNumber_9);

} // end of HAL_acqSciBTxInt() function

static inline void HAL_acqSciBRxInt(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
	// clear the ADC interrupt flag
	SCI_clearRxFifoInt(pobj->sciHandle[1]);

	// Acknowledge interrupt from PIE group 9
	PIE_clearInt(pobj->pieHandle,PIE_GroupNumber_9);


} // end of HAL_acqSciBRxInt() function

//! \brief      Executes calibration routines
//! \details    Values for offset and gain are programmed into OTP memory at
//!             the TI factory.  This calls and internal function that programs
//!             these offsets and gains into the ADC registers.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_cal(HAL_Handle handle);


//! \brief      Disables global interrupts
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_disableGlobalInts(HAL_Handle handle);


//! \brief      Disables the watch dog
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_disableWdog(HAL_Handle handle);


//! \brief      Disables the PWM device
//! \details    Turns off the outputs of the EPWM peripherals which will put
//!             the power switches into a high impedance state.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_disablePwm(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	PWM_setOneShotTrip(pobj->pwmHandle[PWM_Number_1]);
	PWM_setOneShotTrip(pobj->pwmHandle[PWM_Number_2]);
	PWM_setOneShotTrip(pobj->pwmHandle[PWM_Number_3]);

} // end of HAL_disablePwm() function


//! \brief      Enables the ADC interrupts
//! \details    Enables the ADC interrupt in the PIE, and CPU.  Enables the 
//!             interrupt to be sent from the ADC peripheral.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableAdcInts(HAL_Handle handle);



//! \brief      Enables the debug interrupt
//! \details    The debug interrupt is used for the real-time debugger.  It is
//!             not needed if the real-time debugger is not used.  Clears
//!             bit 1 of ST1.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableDebugInt(HAL_Handle handle);


//! \brief     Enables global interrupts
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableGlobalInts(HAL_Handle handle);


//! \brief      Enables the PWM devices
//! \details    Turns on the outputs of the EPWM peripheral which will allow 
//!             the power switches to be controlled.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_enablePwm(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	PWM_clearOneShotTrip(pobj->pwmHandle[PWM_Number_1]);
	PWM_clearOneShotTrip(pobj->pwmHandle[PWM_Number_2]);
	PWM_clearOneShotTrip(pobj->pwmHandle[PWM_Number_3]);


} // end of HAL_enablePwm() function



//! \brief     Enables the PWM interrupt
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_enablePwmInt(HAL_Handle handle);

//! \brief     Enables the Timer 0 interrupt
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableTimer0Int(HAL_Handle handle);
extern void HAL_enableTimer1Int(HAL_Handle handle);
extern void HAL_enableI2cInt(HAL_Handle handle);
extern void HAL_enableSpiAInt(HAL_Handle handle);
extern void HAL_enableSpiBInt(HAL_Handle handle);
extern void HAL_enableSciAInt(HAL_Handle handle);
extern void HAL_enableSciBInt(HAL_Handle handle);


//! \brief     Gets the ADC delay value
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \param[in] socNumber  The ADC SOC number
//! \return    The ADC delay value
static inline ADC_SocSampleDelay_e HAL_getAdcSocSampleDelay(HAL_Handle handle,
                                                            const ADC_SocNumber_e socNumber)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	return(ADC_getSocSampleDelay(pobj->adcHandle,socNumber));
} // end of HAL_getAdcSocSampleDelay() function


//! \brief      Gets the ADC bias value
//! \details    The ADC bias contains the feedback circuit's offset and bias.
//!             Bias is the mathematical offset used when a bi-polar signal
//!             is read into a uni-polar ADC.
//! \param[in]  handle        The hardware abstraction layer (HAL) handle
//! \param[in]  sensorType    The sensor type
//! \param[in]  sensorNumber  The sensor number
//! \return     The ADC bias value
static inline _iq HAL_getBias(HAL_Handle handle,
                              const HAL_SensorType_e sensorType,
                              uint_least8_t u8sensorNumber)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
	_iq iqbias = _IQ(0.0);

	if(sensorType == HAL_SensorType_Current)
    {
		iqbias = pobj->adcBias.I.aiqvalue[u8sensorNumber];
    }
	else if(sensorType == HAL_SensorType_Voltage)
    {
		iqbias = pobj->adcBias.V.aiqvalue[u8sensorNumber];
    }

 	return(iqbias);
} // end of HAL_getBias() function


//! \brief      Gets the current scale factor
//! \details    The current scale factor is defined as
//!             USER_ADC_FULL_SCALE_CURRENT_A/USER_IQ_FULL_SCALE_CURRENT_A.
//!             This scale factor is not used when converting between PU amps
//!             and real amps.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
//! \return     The current scale factor
static inline _iq HAL_getCurrentScaleFactor(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	return(pobj->iqcurrent_sf);
} // end of HAL_getCurrentScaleFactor() function


//! \brief     Gets the number of current sensors
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The number of current sensors
static inline uint_least8_t HAL_getNumCurrentSensors(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
  

	return(pobj->u8numCurrentSensors);
} // end of HAL_getNumCurrentSensors() function


//! \brief     Gets the number of voltage sensors
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The number of voltage sensors
static inline uint_least8_t HAL_getNumVoltageSensors(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
  

	return(pobj->u8numVoltageSensors);
} // end of HAL_getNumVoltageSensors() function


//! \brief      Gets the value used to set the low pass filter pole for offset estimation
//! \details    An IIR single pole low pass filter is used to find the feedback circuit's
//!             offsets.  This function returns the value of that pole.
//! \param[in]  handle        The hardware abstraction layer (HAL) handle
//! \param[in]  sensorType    The sensor type
//! \param[in]  sensorNumber  The sensor number
//! \return     The value used to set the low pass filter pole, pu
static inline _iq HAL_getOffsetBeta_lp_pu(HAL_Handle handle,
                                          const HAL_SensorType_e sensorType,
                                          const uint_least8_t u8sensorNumber)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	_iq iqbeta_lp_pu = _IQ(0.0);
  
	if(sensorType == HAL_SensorType_Current)
    {
		iqbeta_lp_pu = OFFSET_getBeta(pobj->offsetHandle_I[u8sensorNumber]);
    }
	else if(sensorType == HAL_SensorType_Voltage)
    {
		iqbeta_lp_pu = OFFSET_getBeta(pobj->offsetHandle_V[u8sensorNumber]);
    }

	return(iqbeta_lp_pu);
} // end of HAL_getOffsetBeta_lp_pu() function


//! \brief      Gets the offset value
//! \details    The offsets that are calculated during the feedback circuits calibrations
//!             are returned from the IIR filter object.
//! \param[in]  handle        The hardware abstraction layer (HAL) handle
//! \param[in]  sensorType    The sensor type
//! \param[in]  sensorNumber  The sensor number
//! \return     The offset value
static inline _iq HAL_getOffsetValue(HAL_Handle handle,
                                     const HAL_SensorType_e sensorType,
                                     const uint_least8_t u8sensorNumber)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	_iq iqoffset = _IQ(0.0);
  
	if(sensorType == HAL_SensorType_Current)
    {
		iqoffset = OFFSET_getOffset(pobj->offsetHandle_I[u8sensorNumber]);
    }
	else if(sensorType == HAL_SensorType_Voltage)
    {
		iqoffset = OFFSET_getOffset(pobj->offsetHandle_V[u8sensorNumber]);
    }

	return(iqoffset);
} // end of HAL_getOffsetValue() function


//! \brief      Gets the voltage scale factor
//! \details    The voltage scale factor is defined as
//!             USER_ADC_FULL_SCALE_VOLTAGE_V/USER_IQ_FULL_SCALE_VOLTAGE_V.
//!             This scale factor is not used when converting between PU volts
//!             and real volts.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
//! \return     The voltage scale factor
static inline _iq HAL_getVoltageScaleFactor(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	return(pobj->iqvoltage_sf);
} // end of HAL_getVoltageScaleFactor() function


static inline _iq HAL_getExtTempScaleFactor(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	return(pobj->iqExtTemp_sf);
} // end of HAL_getExtTempScaleFactor() function

static inline _iq HAL_getExtAdcScaleFactor(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	return(pobj->iqExtADC_sf);
} // end of HAL_getExtAdcScaleFactor() function


//! \brief      Configures the fault protection logic
//! \details    Sets up the trip zone inputs so that when a comparator
//!             signal from outside the micro-controller trips a fault,
//!             the EPWM peripheral blocks will force the
//!             power switches into a high impedance state.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupFaults(HAL_Handle handle);


//! \brief      Initializes the hardware abstraction layer (HAL) object
//! \details    Initializes all handles to the microcontroller peripherals.
//!             Returns a handle to the HAL object.
//! \param[in]  pMemory   A pointer to the memory for the hardware abstraction layer object
//! \param[in]  numBytes  The number of bytes allocated for the hardware abstraction layer object, bytes
//! \return     The hardware abstraction layer (HAL) object handle
extern HAL_Handle HAL_init(void *pMemory,const size_t numBytes);


//! \brief      Initializes the interrupt vector table
//! \details    Points the ISR to the function mainISR.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_initIntVectorTable(HAL_Handle handle)
 {
	HAL_Obj *pobj = (HAL_Obj *)handle;
	PIE_Obj *ppie = (PIE_Obj *)pobj->pieHandle;


	ENABLE_PROTECTED_REGISTER_WRITE_MODE;

	ppie->TINT0 = &timer0ISR;
	ppie->TINT1 = &timer1ISR;

	ppie->ADCINT1_HP = &adcInt1ISR;//$mainISR
	ppie->ADCINT2_HP = &adcInt2ISR;

	//$pie->ADCINT1 = &mainISR;

	ppie->I2CINT1A = &i2cInt1AISR;
	ppie->I2CINT2A = &i2cInt2AISR;

	ppie->SCITXINTA = &sciATxFifoISR;
	ppie->SCIRXINTA = &sciARxFifoISR;
	ppie->SCITXINTB = &sciBTxFifoISR;
	ppie->SCIRXINTB = &sciBRxFifoISR;

	DISABLE_PROTECTED_REGISTER_WRITE_MODE;


 } // end of HAL_initIntVectorTable() function


//! \brief      Reads the ADC data
//! \details    Reads in the ADC result registers, adjusts for offsets, and
//!             scales the values according to the settings in user.h.  The
//!             structure gAdcData holds three phase voltages, three line
//!             currents, and one DC bus voltage.
//! \param[in]  handle    The hardware abstraction layer (HAL) handle
//! \param[in]  pAdcData  A pointer to the ADC data buffer
static inline void HAL_readAdcData(HAL_Handle handle,HAL_AdcData_t *pAdcData)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	_iq iqvalue;
	_iq iqcurrent_sf = HAL_getCurrentScaleFactor(handle);
	_iq iqvoltage_sf = HAL_getVoltageScaleFactor(handle);
	//_iq iqExtAdc_sf = HAL_getExtAdcScaleFactor(handle);


	// convert current A
	// sample the first sample twice due to errata sprz342f, ignore the first sample
	iqvalue = (_iq)ADC_readResult(pobj->adcHandle,ADC_ResultNumber_1);

#if (DRIVER == AC_DRIVER_495V)
	iqvalue =  _IQ12mpy(iqvalue,iqcurrent_sf) - pobj->adcBias.I.aiqvalue[0];      // divide by 2^numAdcBits = 2^12
	pAdcData->I.aiqvalue[0] = -iqvalue ;
#else
	iqvalue = _IQ12mpy(iqvalue,iqcurrent_sf) - pobj->adcBias.I.aiqvalue[0];      // divide by 2^numAdcBits = 2^12	Org
	pAdcData->I.aiqvalue[0] = iqvalue;				//Org
#endif

	// convert current B
	iqvalue = (_iq)ADC_readResult(pobj->adcHandle,ADC_ResultNumber_2);

#if (DRIVER == AC_DRIVER_495V)
	iqvalue =  _IQ12mpy(iqvalue,iqcurrent_sf) - pobj->adcBias.I.aiqvalue[1];      // divide by 2^numAdcBits = 2^12
	pAdcData->I.aiqvalue[1] = -iqvalue ;
#else
	iqvalue = _IQ12mpy(iqvalue,iqcurrent_sf) - pobj->adcBias.I.aiqvalue[1];      // divide by 2^numAdcBits = 2^12	Org
	pAdcData->I.aiqvalue[1] = iqvalue;				//Org
#endif



	// convert current C
	iqvalue = (_iq)ADC_readResult(pobj->adcHandle,ADC_ResultNumber_3);
#if (DRIVER == AC_DRIVER_495V)
	iqvalue =  _IQ12mpy(iqvalue,iqcurrent_sf) - pobj->adcBias.I.aiqvalue[2];      // divide by 2^numAdcBits = 2^12
	pAdcData->I.aiqvalue[2] = -iqvalue ;
#else
	iqvalue = _IQ12mpy(iqvalue,iqcurrent_sf) - pobj->adcBias.I.aiqvalue[2];      // divide by 2^numAdcBits = 2^12	Org
	pAdcData->I.aiqvalue[2] = iqvalue;				//Org
#endif



	// convert voltage A
	iqvalue = (_iq)ADC_readResult(pobj->adcHandle,ADC_ResultNumber_4);
	iqvalue = _IQ12mpy(iqvalue,iqvoltage_sf) - pobj->adcBias.V.aiqvalue[0];      // divide by 2^numAdcBits = 2^12
	pAdcData->V.aiqvalue[0] = iqvalue;

	// convert voltage B
	iqvalue = (_iq)ADC_readResult(pobj->adcHandle,ADC_ResultNumber_5);
	iqvalue = _IQ12mpy(iqvalue,iqvoltage_sf) - pobj->adcBias.V.aiqvalue[1];      // divide by 2^numAdcBits = 2^12
	pAdcData->V.aiqvalue[1] = iqvalue;

	// convert voltage C
	iqvalue = (_iq)ADC_readResult(pobj->adcHandle,ADC_ResultNumber_6);
	iqvalue = _IQ12mpy(iqvalue,iqvoltage_sf) - pobj->adcBias.V.aiqvalue[2];      // divide by 2^numAdcBits = 2^12
	pAdcData->V.aiqvalue[2] = iqvalue;

	// read the dcBus voltage value
	iqvalue = (_iq)ADC_readResult(pobj->adcHandle,ADC_ResultNumber_7);     // divide by 2^numAdcBits = 2^12
	//iqvalue = _IQmpy(iqvalue, _IQ(1.29));	// Adjust voltage (1.29) 1.28 (Adding)
	iqvalue = _IQ12mpy(iqvalue,iqvoltage_sf);
	pAdcData->iqdcBus = iqvalue;

/*
	iqvalue = (_iq)ADC_readResult(pobj->adcHandle,ADC_ResultNumber_8);     // divide by 2^numAdcBits = 2^12
	iqvalue = _IQ12mpy(iqvalue,HAL_getExtTempScaleFactor(handle));
	pAdcData->iqExtTemp = iqvalue;

	iqvalue = (_iq)ADC_readResult(pobj->adcHandle,ADC_ResultNumber_9);     // divide by 2^numAdcBits = 2^12
	iqvalue = _IQ12mpy(iqvalue,iqExtAdc_sf);
	pAdcData->iqExtAdc1 = iqvalue;

	iqvalue = (_iq)ADC_readResult(pobj->adcHandle,ADC_ResultNumber_10);     // divide by 2^numAdcBits = 2^12
	iqvalue = _IQ12mpy(iqvalue,iqExtAdc_sf);
	pAdcData->iqExtAdc2 = iqvalue;

	iqvalue = (_iq)ADC_readResult(pobj->adcHandle,ADC_ResultNumber_11);     // divide by 2^numAdcBits = 2^12
	iqvalue = _IQ12mpy(iqvalue,iqExtAdc_sf);
	pAdcData->iqExtAdc3 = iqvalue;
*/

} // end of HAL_readAdcData() function

//static inline void HAL_readAdcDataApx(HAL_Handle handle,HAL_AdcData_t *pAdcData)
void HAL_readAdcDataApx(HAL_Handle handle,HAL_AdcData_t *pAdcData);


//! \brief     Reads the timer count
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] timerNumber  The timer number, 0,1 or 2
//! \return    The timer count
static inline uint32_t HAL_readTimerCnt(HAL_Handle handle,const uint_least8_t u8timerNumber)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
	uint32_t u32timerCnt = TIMER_getCount(pobj->timerHandle[u8timerNumber]);

	return(u32timerCnt);
} // end of HAL_readTimerCnt() function


//! \brief     Reloads the timer
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] timerNumber  The timer number, 0,1 or 2
static inline void HAL_reloadTimer(HAL_Handle handle,const uint_least8_t u8timerNumber)
{
	HAL_Obj  *pobj = (HAL_Obj *)handle;

	// reload the specified timer
	TIMER_reload(pobj->timerHandle[u8timerNumber]);


}  // end of HAL_reloadTimer() function


//! \brief     Starts the timer
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] timerNumber  The timer number, 0,1 or 2
static inline void HAL_startTimer(HAL_Handle handle,const uint_least8_t u8timerNumber)
{
	HAL_Obj  *pobj = (HAL_Obj *)handle;

	// start the specified timer
	TIMER_start(pobj->timerHandle[u8timerNumber]);


}  // end of HAL_startTimer() function


//! \brief     Stops the timer
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] timerNumber  The timer number, 0,1 or 2
static inline void HAL_stopTimer(HAL_Handle handle,const uint_least8_t u8timerNumber)
{
	HAL_Obj  *pobj = (HAL_Obj *)handle;

	// stop the specified timer
	TIMER_stop(pobj->timerHandle[u8timerNumber]);


}  // end of HAL_stopTimer() function


//! \brief     Sets the timer period
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] timerNumber  The timer number, 0,1 or 2
//! \param[in] period       The timer period
static inline void HAL_setTimerPeriod(HAL_Handle handle,const uint_least8_t u8timerNumber,
										const uint32_t u32period)
{
	HAL_Obj  *pobj = (HAL_Obj *)handle;

	// set the period
	TIMER_setPeriod(pobj->timerHandle[u8timerNumber], u32period);

}  // end of HAL_setTimerPeriod() function


//! \brief     Gets the timer period
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] timerNumber  The timer number, 0,1 or 2
//! \return    The timer period
static inline uint32_t HAL_getTimerPeriod(HAL_Handle handle,const uint_least8_t u8timerNumber)
{
	HAL_Obj  *pobj = (HAL_Obj *)handle;

	uint32_t u32timerPeriod = TIMER_getPeriod(pobj->timerHandle[u8timerNumber]);

	return(u32timerPeriod);
}  // end of HAL_getTimerPeriod() function


//! \brief     Sets the ADC SOC sample delay value
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] socNumber    The SOC number
//! \param[in] sampleDelay  The delay value for the ADC
static inline void HAL_setAdcSocSampleDelay(HAL_Handle handle,
                                            const ADC_SocNumber_e socNumber,
                                            const ADC_SocSampleDelay_e sampleDelay)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	ADC_setSocSampleDelay(pobj->adcHandle,socNumber,sampleDelay);

} // end of HAL_setAdcSocSampleDelay() function


//! \brief     Sets the ADC bias value
//! \param[in] handle        The hardware abstraction layer (HAL) handle
//! \param[in] sensorType    The sensor type
//! \param[in] sensorNumber  The sensor number
//! \param[in] bias          The ADC bias value
static inline void HAL_setBias(HAL_Handle handle,
                               const HAL_SensorType_e sensorType,
                               uint_least8_t u8sensorNumber,
                               const _iq iqbias)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;


	if(sensorType == HAL_SensorType_Current)
    {
		pobj->adcBias.I.aiqvalue[u8sensorNumber] = iqbias;
    }
	else if(sensorType == HAL_SensorType_Voltage)
    {
		pobj->adcBias.V.aiqvalue[u8sensorNumber] = iqbias;
    }


} // end of HAL_setBias() function


//! \brief      Sets the GPIO pin high
//! \details    Takes in the enumeration GPIO_Number_e and sets that GPIO
//!             pin high.
//! \param[in]  handle      The hardware abstraction layer (HAL) handle
//! \param[in]  gpioNumber  The GPIO number
static inline void HAL_setGpioHigh(HAL_Handle handle,const GPIO_Number_e gpioNumber)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
  

	// set GPIO high
	GPIO_setHigh(pobj->gpioHandle,gpioNumber);


} // end of HAL_setGpioHigh() function


//! \brief     Sets up the timers
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] systemFreq_MHz  The system frequency, MHz
void HAL_setupTimers(HAL_Handle handle,const uint_least16_t u16systemFreq_MHz);


//! \brief      Toggles the GPIO pin
//! \details    Takes in the enumeration GPIO_Number_e and toggles that GPIO
//!             pin.
//! \param[in]  handle      The hardware abstraction layer (HAL) handle
//! \param[in]  gpioNumber  The GPIO number
static inline void HAL_toggleGpio(HAL_Handle handle,const GPIO_Number_e gpioNumber)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;


	// set GPIO high
	GPIO_toggle(pobj->gpioHandle,gpioNumber);


} // end of HAL_setGpioHigh() function


//! \brief      Sets the GPIO pin low
//! \details    Takes in the enumeration GPIO_Number_e and clears that GPIO
//!             pin low.
//! \param[in]  handle      The hardware abstraction layer (HAL) handle
//! \param[in]  gpioNumber  The GPIO number
static inline void HAL_setGpioLow(HAL_Handle handle,const GPIO_Number_e gpioNumber)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
  

	// set GPIO low
	GPIO_setLow(pobj->gpioHandle,gpioNumber);

} // end of HAL_setGpioLow() function


//! \brief     Sets the current scale factor in the hardware abstraction layer
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] current_sf  The current scale factor
static inline void HAL_setCurrentScaleFactor(HAL_Handle handle,const _iq iqcurrent_sf)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
  

	pobj->iqcurrent_sf = iqcurrent_sf;

} // end of HAL_setCurrentScaleFactor() function


//! \brief     Sets the number of current sensors
//! \param[in] handle             The hardware abstraction layer (HAL) handle
//! \param[in] numCurrentSensors  The number of current sensors
static inline void HAL_setNumCurrentSensors(HAL_Handle handle,const uint_least8_t u8numCurrentSensors)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
  

	pobj->u8numCurrentSensors = u8numCurrentSensors;


} // end of HAL_setNumCurrentSensors() function


//! \brief     Sets the number of voltage sensors
//! \param[in] handle             The hardware abstraction layer (HAL) handle
//! \param[in] numVoltageSensors  The number of voltage sensors
static inline void HAL_setNumVoltageSensors(HAL_Handle handle,const uint_least8_t u8numVoltageSensors)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
  

	pobj->u8numVoltageSensors = u8numVoltageSensors;

} // end of HAL_setNumVoltageSensors() function


//! \brief     Sets the value used to set the low pass filter pole for offset estimation
//! \param[in] handle        The hardware abstraction layer (HAL) handle
//! \param[in] sensorType    The sensor type
//! \param[in] sensorNumber  The sensor number
//! \param[in] beta_lp_pu    The value used to set the low pass filter pole, pu
static inline void HAL_setOffsetBeta_lp_pu(HAL_Handle handle,
                                           const HAL_SensorType_e sensorType,
                                           const uint_least8_t u8sensorNumber,
                                           const _iq iqbeta_lp_pu)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	if(sensorType == HAL_SensorType_Current)
    {
		OFFSET_setBeta(pobj->offsetHandle_I[u8sensorNumber],iqbeta_lp_pu);
    }
	else if(sensorType == HAL_SensorType_Voltage)
    {
		OFFSET_setBeta(pobj->offsetHandle_V[u8sensorNumber],iqbeta_lp_pu);
    }


} // end of HAL_setOffsetBeta_lp_pu() function


//! \brief     Sets the offset initial condition value for offset estimation
//! \param[in] handle        The hardware abstraction layer (HAL) handle
//! \param[in] sensorType    The sensor type
//! \param[in] sensorNumber  The sensor number
//! \param[in] initCond      The initial condition value
static inline void HAL_setOffsetInitCond(HAL_Handle handle,
                                         const HAL_SensorType_e sensorType,
                                         const uint_least8_t u8sensorNumber,
                                         const _iq iqinitCond)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	if(sensorType == HAL_SensorType_Current)
    {
		OFFSET_setInitCond(pobj->offsetHandle_I[u8sensorNumber],iqinitCond);
    }
	else if(sensorType == HAL_SensorType_Voltage)
    {
		OFFSET_setInitCond(pobj->offsetHandle_V[u8sensorNumber],iqinitCond);
    }


} // end of HAL_setOffsetInitCond() function


//! \brief     Sets the initial offset value for offset estimation
//! \param[in] handle        The hardware abstraction layer (HAL) handle
//! \param[in] sensorType    The sensor type
//! \param[in] sensorNumber  The sensor number
//! \param[in] value         The initial offset value
static inline void HAL_setOffsetValue(HAL_Handle handle,
                                      const HAL_SensorType_e sensorType,
                                      const uint_least8_t u8sensorNumber,
                                      const _iq iqvalue)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	if(sensorType == HAL_SensorType_Current)
    {
		OFFSET_setOffset(pobj->offsetHandle_I[u8sensorNumber],iqvalue);
    }
	else if(sensorType == HAL_SensorType_Voltage)
    {
		OFFSET_setOffset(pobj->offsetHandle_V[u8sensorNumber],iqvalue);
    }

} // end of HAL_setOffsetValue() function


//! \brief     Sets the voltage scale factor in the hardware abstraction layer
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] voltage_sf  The voltage scale factor
static inline void HAL_setVoltageScaleFactor(HAL_Handle handle,const _iq iqvoltage_sf)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
  
	pobj->iqvoltage_sf = iqvoltage_sf;

} // end of HAL_setVoltageScaleFactor() function

static inline void HAL_setExtTempScaleFactor(HAL_Handle handle,const _iq iqExtTemp_sf)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	pobj->iqExtTemp_sf = iqExtTemp_sf;
}

static inline void HAL_setExtAdcScaleFactor(HAL_Handle handle,const _iq iqExtAdc_sf)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	pobj->iqExtADC_sf = iqExtAdc_sf;
}



//! \brief      Sets the hardware abstraction layer parameters
//! \details    Sets up the microcontroller peripherals.  Creates all of the scale
//!             factors for the ADC voltage and current conversions.  Sets the initial
//!             offset values for voltage and current measurements.
//! \param[in]  handle       The hardware abstraction layer (HAL) handle
//! \param[in]  pUserParams  The pointer to the user parameters
extern void HAL_setParams(HAL_Handle handle,const USER_Params *pUserParams);


//! \brief      Sets up the ADCs (Analog to Digital Converters)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupAdcs(HAL_Handle handle);


//! \brief      Sets up the clocks
//! \details    Sets up the micro-controller's main oscillator
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupClks(HAL_Handle handle);


//! \brief     Sets up the FLASH.
extern void HAL_setupFlash(HAL_Handle handle);


//! \brief     Sets up the GPIO (General Purpose I/O) pins
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupGpios(HAL_Handle handle);


//! \brief     Sets up the peripheral clocks
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupPeripheralClks(HAL_Handle handle);


//! \brief     Sets up the PIE (Peripheral Interrupt Expansion)
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupPie(HAL_Handle handle);


//! \brief     Sets up the PLL (Phase Lock Loop)
//! \param[in] handle   The hardware abstraction layer (HAL) handle
//! \param[in] clkFreq  The clock frequency
extern void HAL_setupPll(HAL_Handle handle,const PLL_ClkFreq_e clkFreq);


//! \brief     Sets up the PWMs (Pulse Width Modulators)
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] systemFreq_MHz  The system frequency, MHz
//! \param[in] pwmPeriod_usec  The PWM period, usec
//! \param[in] numPwmTicksPerIsrTick  The number of PWM clock ticks per ISR clock tick
extern void HAL_setupPwms(HAL_Handle handle,
                   const uint_least16_t u16systemFreq_MHz,
                   const float_t fpwmPeriod_usec,
                   const uint_least16_t u16numPwmTicksPerIsrTick);


//! \brief     Sets up the PWM DACs (Pulse Width Modulator Digital to Analog Converters)
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupPwmDacs(HAL_Handle handle);


//! \brief     Sets up the QEP peripheral
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupQEP(HAL_Handle handle,HAL_QepSelect_e qep);

extern void HAL_setupI2C(HAL_Handle handle);
extern void HAL_setupSCI(HAL_Handle handle);
extern void HAL_setupSPI(HAL_Handle handle);


//! \brief     Sets up the timers
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] systemFreq_MHz  The system frequency, MHz
extern void HAL_setupTimers(HAL_Handle handle,const uint_least16_t u16systemFreq_MHz);


//! \brief      Updates the ADC bias values
//! \details    This function is called before the motor is started.  It sets the voltage
//!             and current measurement offsets.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_updateAdcBias(HAL_Handle handle)
{
	uint_least8_t u8cnt;
	HAL_Obj *pobj = (HAL_Obj *)handle;
	_iq iqbias;


	// update the current bias
	for(u8cnt=0;u8cnt<HAL_getNumCurrentSensors(handle);u8cnt++)
    {
		iqbias = HAL_getBias(handle,HAL_SensorType_Current,u8cnt);
      
#if (DRIVER == AC_DRIVER_495V)
		iqbias -= OFFSET_getOffset(pobj->offsetHandle_I[u8cnt]);

#else
		iqbias += OFFSET_getOffset(pobj->offsetHandle_I[u8cnt]);		//Org
#endif

		//iqbias += OFFSET_getOffset(pobj->offsetHandle_I[u8cnt]);		//Org
		//iqbias -= OFFSET_getOffset(pobj->offsetHandle_I[u8cnt]);	//Modify

		HAL_setBias(handle,HAL_SensorType_Current,u8cnt,iqbias);
    }


	// update the voltage bias
	for(u8cnt=0;u8cnt<HAL_getNumVoltageSensors(handle);u8cnt++)
    {
		iqbias = HAL_getBias(handle,HAL_SensorType_Voltage,u8cnt);

		iqbias += OFFSET_getOffset(pobj->offsetHandle_V[u8cnt]);

		HAL_setBias(handle,HAL_SensorType_Voltage,u8cnt,iqbias);
    }


} // end of HAL_updateAdcBias() function


//! \brief     Writes DAC data to the PWM comparators for DAC (digital-to-analog conversion) output
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pDacData  The pointer to the DAC data
static inline void HAL_writeDacData(HAL_Handle handle,HAL_DacData_t *pDacData)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	// convert values from _IQ to _IQ15
	int16_t i16dacValue_1 = (int16_t)_IQtoIQ15(pDacData->aiqvalue[0]);
	int16_t i16dacValue_2 = (int16_t)_IQtoIQ15(pDacData->aiqvalue[1]);
	int16_t i16dacValue_3 = (int16_t)_IQtoIQ15(pDacData->aiqvalue[2]);
	int16_t i16dacValue_4 = (int16_t)_IQtoIQ15(pDacData->aiqvalue[3]);

	// write the DAC data
	PWMDAC_write_CmpA(pobj->pwmDacHandle[PWMDAC_Number_1],i16dacValue_1);
	PWMDAC_write_CmpB(pobj->pwmDacHandle[PWMDAC_Number_1],i16dacValue_2);
	PWMDAC_write_CmpA(pobj->pwmDacHandle[PWMDAC_Number_2],i16dacValue_3);
	PWMDAC_write_CmpA(pobj->pwmDacHandle[PWMDAC_Number_3],i16dacValue_4);


} // end of HAL_writeDacData() function


//! \brief     Writes PWM data to the PWM comparators for motor control
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pPwmData  The pointer to the PWM data
static inline void HAL_writePwmData(HAL_Handle handle,HAL_PwmData_t *pPwmData)
{
	uint_least8_t u8cnt;
	HAL_Obj *pobj = (HAL_Obj *)handle;
	PWM_Obj *ppwm;
	_iq iqperiod;
	_iq iqpwmData_neg;
	_iq iqpwmData_sat;
	_iq iqpwmData_sat_dc;
	_iq iqvalue;
	uint16_t u16value_sat;

	for(u8cnt=0;u8cnt<3;u8cnt++)
    {
		ppwm = (PWM_Obj *)pobj->pwmHandle[u8cnt];
		iqperiod = (_iq)ppwm->TBPRD;
		iqpwmData_neg = _IQmpy(pPwmData->Tabc.aiqvalue[u8cnt],_IQ(-1.0));
		//iqpwmData_sat = _IQsat(iqpwmData_neg,_IQ(1.0),_IQ(-1.0));
		//iqpwmData_sat_dc = _IQmpy(iqpwmData_sat + _IQ(1.0), _IQ(0.5));

		iqpwmData_sat = _IQsat(iqpwmData_neg,_IQ(0.5),_IQ(-0.5));//$shift to 0~1,like the unit periodic signal
		iqpwmData_sat_dc = iqpwmData_sat + _IQ(0.5);
		iqvalue = _IQmpy(iqpwmData_sat_dc, iqperiod);
		u16value_sat = (uint16_t)_IQsat(iqvalue, iqperiod, _IQ(0.0));

		// write the PWM data
		PWM_write_CmpA(pobj->pwmHandle[u8cnt],u16value_sat);
    }

} // end of HAL_writePwmData() function


//! \brief     Reads PWM compare register A
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pwmNumber  The PWM number
//! \return    The PWM compare value
static inline uint16_t HAL_readPwmCmpA(HAL_Handle handle,const PWM_Number_e pwmNumber)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	// the compare value to be returned
	uint16_t u16pwmValue;

	u16pwmValue = PWM_get_CmpA(pobj->pwmHandle[pwmNumber]);

	return(u16pwmValue);
} // end of HAL_readPwmCmpA() function


//! \brief     Reads PWM compare register B
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pwmNumber  The PWM number
//! \return    The PWM compare value
static inline uint16_t HAL_readPwmCmpB(HAL_Handle handle,const PWM_Number_e pwmNumber)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	// the compare value to be returned
	uint16_t u16pwmValue;

	u16pwmValue = PWM_get_CmpB(pobj->pwmHandle[pwmNumber]);

	return(u16pwmValue);
} // end of HAL_readPwmCmpB() function


//! \brief     Reads PWM period register
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pwmNumber  The PWM number
//! \return    The PWM period value
static inline uint16_t HAL_readPwmPeriod(HAL_Handle handle,const PWM_Number_e pwmNumber)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;

	// the period value to be returned
	uint16_t u16pwmPeriodValue;

	u16pwmPeriodValue = PWM_getPeriod(pobj->pwmHandle[pwmNumber]);

	return(u16pwmPeriodValue);
} // end of HAL_readPwmPeriod() function


static inline void HAL_setTrigger(HAL_Handle handle,const int16_t i16minwidth)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
	PWM_Obj *ppwm1 = (PWM_Obj *)pobj->pwmHandle[PWM_Number_1];
	PWM_Obj *ppwm2 = (PWM_Obj *)pobj->pwmHandle[PWM_Number_2];
	PWM_Obj *ppwm3 = (PWM_Obj *)pobj->pwmHandle[PWM_Number_3];
	PWM_Obj *ppwm;
	bool bignorepwm1 = false;
	bool bignorepwm2 = false;
	bool bignorepwm3 = false;

	uint16_t u16nextPulse1 = (ppwm1->CMPA + ppwm1->CMPAM) / 2;
	uint16_t u16nextPulse2 = (ppwm2->CMPA + ppwm2->CMPAM) / 2;
	uint16_t u16nextPulse3 = (ppwm3->CMPA + ppwm3->CMPAM) / 2;
	uint16_t u16minNextPulse;

	if(u16nextPulse1 < i16minwidth) bignorepwm1 = true;
	if(u16nextPulse2 < i16minwidth) bignorepwm2 = true;
	if(u16nextPulse3 < i16minwidth) bignorepwm3 = true;

	if((bignorepwm1 == false) && (bignorepwm2 == false) && (bignorepwm3 == false))
    {
		u16minNextPulse = u16nextPulse1;
		ppwm = (PWM_Obj *)pobj->pwmHandle[PWM_Number_1];

		if(u16minNextPulse > u16nextPulse2)
        {
			u16minNextPulse = u16nextPulse2;
			ppwm = (PWM_Obj *)pobj->pwmHandle[PWM_Number_2];
        }

		if(u16minNextPulse > u16nextPulse3)
        {
			u16minNextPulse = u16nextPulse3;
			ppwm = (PWM_Obj *)pobj->pwmHandle[PWM_Number_3];
        }
    }
	else if((bignorepwm1 == false) && (bignorepwm2 == false) && (bignorepwm3 == true))
    {
		u16minNextPulse = u16nextPulse1;
		ppwm = (PWM_Obj *)pobj->pwmHandle[PWM_Number_1];

		if(u16minNextPulse > u16nextPulse2)
        {
			u16minNextPulse = u16nextPulse2;
			ppwm = (PWM_Obj *)pobj->pwmHandle[PWM_Number_2];
        }
    }
	else if((bignorepwm1 == false) && (bignorepwm2 == true) && (bignorepwm3 == false))
    {
		u16minNextPulse = u16nextPulse1;
		ppwm = (PWM_Obj *)pobj->pwmHandle[PWM_Number_1];

		if(u16minNextPulse > u16nextPulse3)
        {
			u16minNextPulse = u16nextPulse3;
			ppwm = (PWM_Obj *)pobj->pwmHandle[PWM_Number_3];
        }
    }
	else if((bignorepwm1 == false) && (bignorepwm2 == true) && (bignorepwm3 == true))
    {
		u16minNextPulse = u16nextPulse1;
		ppwm = (PWM_Obj *)pobj->pwmHandle[PWM_Number_1];
    }
	else if((bignorepwm1 == true) && (bignorepwm2 == false) && (bignorepwm3 == false))
    {
		u16minNextPulse = u16nextPulse2;
		ppwm = (PWM_Obj *)pobj->pwmHandle[PWM_Number_2];

		if(u16minNextPulse > u16nextPulse3)
        {
			u16minNextPulse = u16nextPulse3;
			ppwm = (PWM_Obj *)pobj->pwmHandle[PWM_Number_3];
        }
    }
	else if((bignorepwm1 == true) && (bignorepwm2 == false) && (bignorepwm3 == true))
    {
		u16minNextPulse = u16nextPulse2;
		ppwm = (PWM_Obj *)pobj->pwmHandle[PWM_Number_2];
    }
	else if((bignorepwm1 == true) && (bignorepwm2 == true) && (bignorepwm3 == false))
    {
		u16minNextPulse = u16nextPulse3;
		ppwm = (PWM_Obj *)pobj->pwmHandle[PWM_Number_3];
    }
	else
    {
		u16minNextPulse = u16nextPulse1;
		ppwm = (PWM_Obj *)pobj->pwmHandle[PWM_Number_1];
    }

	if(ppwm->CMPAM >= (ppwm->CMPA + ppwm->DBFED))
    {
		ppwm1->CMPB = (ppwm->CMPAM - (ppwm->CMPA + ppwm->DBFED)) / 2 + 1;
	  	PWM_setSocAPulseSrc(pobj->pwmHandle[PWM_Number_1],PWM_SocPulseSrc_CounterEqualCmpBDecr);
    }
	else
    {
		ppwm1->CMPB = ((ppwm->CMPA + ppwm->DBFED) - ppwm->CMPAM) / 2 + 1;
		PWM_setSocAPulseSrc(pobj->pwmHandle[PWM_Number_1],PWM_SocPulseSrc_CounterEqualCmpBIncr);
    }


} // end of HAL_setTrigger() function

#ifdef QEP
//! \brief     Returns the current position count from QEP
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \return    the current position count from QEP
static inline uint32_t HAL_getQepPosnCounts(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
	QEP_Obj *pqep = (QEP_Obj *)pobj->qepHandle[0];

	return pqep->QPOSCNT;
}


//! \brief     Returns the maximum position count from QEP
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \return    the maximum position count from QEP
static inline uint32_t HAL_getQepPosnMaximum(HAL_Handle handle)
{
	HAL_Obj *pobj = (HAL_Obj *)handle;
	QEP_Obj *pqep = (QEP_Obj *)pobj->qepHandle[0];

	return pqep->QPOSMAX;
}
#endif

//! \brief     Selects the analog channel used for calibration
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] chanNumber  The channel number
void HAL_AdcCalChanSelect(HAL_Handle handle, const ADC_SocChanNumber_e chanNumber);


//! \brief     Reads the converted value from the selected calibration channel
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \return    The converted value
uint16_t HAL_AdcCalConversion(HAL_Handle handle);


//! \brief     Executes the offset calibration of the ADC
//! \param[in] handle     The hardware abstraction layer (HAL) handle
void HAL_AdcOffsetSelfCal(HAL_Handle handle);


//! \brief     Converts coarse and fine oscillator trim values into a single 16bit word value
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \param[in] coarse     The coarse trim portion of the oscillator trim
//! \param[in] fine       The fine trim portion of the oscillator trim
//! \return    The combined trim value
uint16_t HAL_getOscTrimValue(int16_t i16coarse, int16_t i16fine);


//! \brief     Executes the oscillator 1 and 2 calibration functions
//! \param[in] handle     The hardware abstraction layer (HAL) handle
void HAL_OscTempComp(HAL_Handle handle);


//! \brief     Executes the oscillator 1 calibration based on input sample
//! \param[in] handle     The hardware abstraction layer (HAL) handle
void HAL_osc1Comp(HAL_Handle handle, const int16_t i16sensorSample);


//! \brief     Executes the oscillator 2 calibration based on input sample
//! \param[in] handle     The hardware abstraction layer (HAL) handle
void HAL_osc2Comp(HAL_Handle handle, const int16_t i16sensorSample);



//! \brief     Writes DAC data to the PWM comparators for DAC (digital-to-analog conversion) output
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pDacData  The pointer to the DAC data
void HAL_setDacParameters(HAL_Handle handle, HAL_DacData_t *pDacData);




#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _HAL_H_ definition


