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

//! \file   ~/sw/modules/hal/boards/hvkit_rev1p1/f28x/f2806x/src/32b/float/hal.h
//! \brief  Contains public interface to various functions related
//!         to the HAL object
//!
//! (C) Copyright 2014, Texas Instruments, Inc.


// **************************************************************************
// the includes

// // drivers
// #include "sw/drivers/adc/src/32b/f28x/f2806x/adc.h"
// #include "sw/drivers/clk/src/32b/f28x/f2806x/clk.h"
// #include "sw/drivers/cpu/src/32b/f28x/f2806x/cpu.h"
// #include "sw/drivers/flash/src/32b/f28x/f2806x/flash.h"
// #include "sw/drivers/gpio/src/32b/f28x/f2806x/gpio.h"
// #include "sw/drivers/osc/src/32b/f28x/f2806x/osc.h"
// #include "sw/drivers/pie/src/32b/f28x/f2806x/pie.h"
// #include "sw/drivers/pll/src/32b/f28x/f2806x/pll.h"
// #include "sw/drivers/pwm/src/32b/f28x/f2806x/pwm.h"
// #include "sw/drivers/pwmdac/src/32b/f28x/f2806x/pwmdac.h"
// #include "sw/drivers/pwr/src/32b/f28x/f2806x/pwr.h"
// #include "sw/drivers/spi/src/32b/f28x/f2806x/spi.h"
// #include "sw/drivers/timer/src/32b/f28x/f2806x/timer.h"
// #include "sw/drivers/wdog/src/32b/f28x/f2806x/wdog.h"


// // modules
// #include "sw/modules/hal/src/float/hal_data.h"
// #include "sw/modules/math/src/float/math.h"
// #include "sw/modules/offset/src/float/offset.h"


// // solutions
// #include "user.h"

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

//! \brief Defines that a DRV8301 chip SPI port is used on the board.
#define DRV8301_SPI

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
//#define HAL_PWM_DBFED_CNT         1
#define HAL_PWM_DBFED_CNT         (uint16_t)(0.050 * (float_t)USER_SYSTEM_FREQ_MHz)            // 2 usec


//! \brief Defines the PWM deadband rising edge delay count (system clocks)
//!
//#define HAL_PWM_DBRED_CNT         1
#define HAL_PWM_DBRED_CNT        (uint16_t)(0.050 * (float_t)USER_SYSTEM_FREQ_MHz)            // 2 usec


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


//! \brief Enumeration for the LED numbers
//!
typedef enum
{
  HAL_Gpio_LED2=GPIO_Number_31,  //!< GPIO pin number for ControlCARD LED 2
  HAL_Gpio_LED3=GPIO_Number_34   //!< GPIO pin number for ControlCARD LED 3
} HAL_LedNumber_e;
  

//! \brief Enumeration for the PWM frequencies
//!
typedef enum
{
  HAL_PwmFreq_2_kHz = 22500,  //!<   2 kHz
  HAL_PwmFreq_3_kHz = 15000,  //!<   3 kHz
  HAL_PwmFreq_4_kHz = 11250,  //!<   4 kHz
  HAL_PwmFreq_5_kHz =  9000,  //!<   5 kHz
  HAL_PwmFreq_6_kHz =  7500,  //!<   6 kHz
  HAL_PwmFreq_8_kHz =  5625,  //!<   8 kHz
  HAL_PwmFreq_9_kHz =  5000,  //!<   9 kHz
  HAL_PwmFreq_10_kHz = 4500,  //!<  10 kHz
  HAL_PwmFreq_12_kHz = 3750,  //!<  12 kHz
  HAL_PwmFreq_15_kHz = 3000,  //!<  15 kHz
  HAL_PwmFreq_18_kHz = 2500,  //!<  18 kHz
  HAL_PwmFreq_20_kHz = 2250,  //!<  20 kHz
  HAL_PwmFreq_24_kHz = 1875,  //!<  24 kHz
  HAL_PwmFreq_25_kHz = 1800,  //!<  25 kHz
  HAL_PwmFreq_30_kHz = 1500,  //!<  30 kHz
  HAL_PwmFreq_36_kHz = 1250,  //!<  36 kHz
  HAL_PwmFreq_40_kHz = 1125,  //!<  40 kHz
  HAL_PwmFreq_45_kHz = 1000,  //!<  45 kHz
  HAL_PwmFreq_50_kHz =  900,  //!<  50 kHz
  HAL_PwmFreq_60_kHz =  750,  //!<  60 kHz
  HAL_PwmFreq_75_kHz =  600,  //!<  75 kHz
  HAL_PwmFreq_90_kHz =  500,  //!<  90 kHz
  HAL_PwmFreq_100_kHz = 450,  //!< 100 kHz
  HAL_PwmFreq_120_kHz = 375,  //!< 120 kHz
  HAL_PwmFreq_125_kHz = 360,  //!< 125 kHz
  HAL_PwmFreq_150_kHz = 300   //!< 150 kHz
} HAL_PwmFreq_e;


//! \brief Enumeration for the sensor types
//!
typedef enum
{
  HAL_SensorType_Current=0,      //!< Enumeration for current sensor
  HAL_SensorType_Voltage         //!< Enumeration for voltage sensor
} HAL_SensorType_e;
  



// //! \brief Defines the hardware abstraction layer (HAL) handle
// //!
// typedef struct _HAL_Handle_   *HAL_Handle;


// **************************************************************************
// the globals

extern interrupt void led2ISR(void);
extern interrupt void led3OffISR(void);
extern interrupt void led3OnISR(void);
extern interrupt void mainISR(void);


// **************************************************************************
// the function prototypes

extern void HAL_AdcCalChanSelect(HAL_Handle handle, const ADC_SocChanNumber_e chanNumber);


extern uint16_t HAL_AdcCalConversion(HAL_Handle handle);


extern void HAL_AdcOffsetSelfCal(HAL_Handle handle);



//! \brief     Acknowledges an interrupt from the ADC
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \param[in] intNumber  The interrupt number
static inline void HAL_acqAdcInt(HAL_Handle handle,const ADC_IntNumber_e intNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;


  // clear the ADC interrupt flag
  ADC_clearIntFlag(obj->adcHandle,intNumber);


  // Acknowledge interrupt from PIE group 10 
  PIE_clearInt(obj->pieHandle,PIE_GroupNumber_10);

  return;
} // end of HAL_acqAdcInt() function


//! \brief     Acknowledges an interrupt from the PWM
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \param[in] pwmNumber  The PWM number
static inline void HAL_acqPwmInt(HAL_Handle handle,const PWM_Number_e pwmNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;


  // clear the PWM interrupt flag
  PWM_clearIntFlag(obj->pwmHandle[pwmNumber]);


  // clear the SOCA flag
  PWM_clearSocAFlag(obj->pwmHandle[pwmNumber]);


  // Acknowledge interrupt from PIE group 3
  PIE_clearInt(obj->pieHandle,PIE_GroupNumber_3);

  return;
} // end of HAL_acqPwmInt() function


//! \brief     Executes calibration routines
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_cal(HAL_Handle handle);


//! \brief     Disables global interrupts
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_disableGlobalInts(HAL_Handle handle);


//! \brief     Disables the PWM device
//! \param[in] handle  The hardware abstraction layer (HAL) handle
static inline void HAL_disablePwm(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

//  // dis-able the drv8301
//  GPIO_setLow(obj->gpioHandle,GPIO_Number_51);

  PWM_setOneShotTrip(obj->pwmHandle[PWM_Number_1]);
  PWM_setOneShotTrip(obj->pwmHandle[PWM_Number_2]);
  PWM_setOneShotTrip(obj->pwmHandle[PWM_Number_3]);

  return;
} // end of HAL_disablePwm() function


//! \brief     Disables the ePWM module time base clock sync signal
//! \param[in] handle  The hardware abstraction layer (HAL) handle
static inline void HAL_disableTbClockSync(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  CLK_disableTbClockSync(obj->clkHandle);

  return;
} // end of HAL_disableTbClockSync() function


//! \brief     Enables the ADC interrupts
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableAdcInts(HAL_Handle handle);


//! \brief     Enables the debug interrupt
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableDebugInt(HAL_Handle handle);


//! \brief     Enables global interrupts
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableGlobalInts(HAL_Handle handle);


//! \brief      Enables the 8301 device
//! \details    Provides the correct timing to enable the drv8301
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableDrv(HAL_Handle handle);


//! \brief     Enables the PWM devices
//! \param[in] handle  The hardware abstraction layer (HAL) handle
static inline void HAL_enablePwm(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

//  // Enable the drv8301
//  GPIO_setHigh(obj->gpioHandle,GPIO_Number_51);

  PWM_clearOneShotTrip(obj->pwmHandle[PWM_Number_1]);
  PWM_clearOneShotTrip(obj->pwmHandle[PWM_Number_2]);
  PWM_clearOneShotTrip(obj->pwmHandle[PWM_Number_3]);

  return;
} // end of HAL_enablePwm() function


//! \brief     Enables the PWM interrupt
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_enablePwmInt(HAL_Handle handle);


//! \brief     Enables the ePWM module time base clock sync signal
//! \param[in] handle  The hardware abstraction layer (HAL) handle
static inline void HAL_enableTbClockSync(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  CLK_enableTbClockSync(obj->clkHandle);

  return;
} // end of HAL_enableTbClockSync() function


//! \brief     Gets the ADC delay value
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \param[in] socNumber  The ADC SOC number
//! \return    The ADC delay value
static inline ADC_SocSampleDelay_e HAL_getAdcSocSampleDelay(HAL_Handle handle,
                                                            const ADC_SocNumber_e socNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  return(ADC_getSocSampleDelay(obj->adcHandle,socNumber));
} // end of HAL_getAdcSocSampleDelay() function


//! \brief     Gets the ADC bias value
//! \param[in] handle        The hardware abstraction layer (HAL) handle
//! \param[in] sensorType    The sensor type
//! \param[in] sensorNumber  The sensor number
//! \return    The ADC bias value
static inline float_t HAL_getBias(HAL_Handle handle,
                                  const HAL_SensorType_e sensorType,
                                  uint_least8_t sensorNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  float_t bias = 0.0;

  if(sensorType == HAL_SensorType_Current)
    {
      bias = obj->adcBias.I_A.value[sensorNumber];
    }
  else if(sensorType == HAL_SensorType_Voltage)
    {
      bias = obj->adcBias.V_V.value[sensorNumber];
    }

  return(bias);
} // end of HAL_getBias() function


//! \brief     Gets the current scale factor
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The current scale factor
static inline float_t HAL_getCurrentScaleFactor(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  return(obj->current_sf);
} // end of HAL_getCurrentScaleFactor() function


//! \brief     Gets the PWM duty cycle times
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] pDutyCycles  A pointer to memory for the duty cycle durations
static inline void HAL_getDutyCycles(HAL_Handle handle,uint16_t *pDutyCycles)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  pDutyCycles[0] = PWM_get_CmpA(obj->pwmHandle[PWM_Number_1]);
  pDutyCycles[1] = PWM_get_CmpA(obj->pwmHandle[PWM_Number_2]);
  pDutyCycles[2] = PWM_get_CmpA(obj->pwmHandle[PWM_Number_3]);

  return;
} // end of HAL_getDutyCycles() function


//! \brief     Gets the number of current sensors
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The number of current sensors
static inline uint_least8_t HAL_getNumCurrentSensors(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  

  return(obj->numCurrentSensors);
} // end of HAL_getNumCurrentSensors() function


//! \brief     Gets the number of voltage sensors
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The number of voltage sensors
static inline uint_least8_t HAL_getNumVoltageSensors(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  

  return(obj->numVoltageSensors);
} // end of HAL_getNumVoltageSensors() function


//! \brief     Gets the voltage scale factor
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The voltage scale factor
static inline float_t HAL_getVoltageScaleFactor(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  return(obj->voltage_sf);
} // end of HAL_getVoltageScaleFactor() function


//! \brief     Runs high voltage protection logic
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_hvProtection(HAL_Handle handle);


//! \brief     Initializes the hardware abstraction layer (HAL) object
//! \param[in] pMemory   A pointer to the memory for the hal object
//! \param[in] numBytes  The number of bytes allocated for the hal object, bytes
//! \return    The hardware abstraction layer (HAL) object handle
extern HAL_Handle HAL_init(void *pMemory,const size_t numBytes);


extern void HAL_OscTempComp(HAL_Handle handle);


extern void HAL_osc1Comp(HAL_Handle handle, const int16_t sensorSample);


extern void HAL_osc2Comp(HAL_Handle handle, const int16_t sensorSample);


extern uint16_t HAL_computeOscTrimValue(int16_t coarse, int16_t fine);


//! \brief     Initializes the interrupt vector table
//! \param[in] handle  The hardware abstraction layer (HAL) handle
static inline void HAL_initIntVectorTable(HAL_Handle handle)
 {
  HAL_Obj *obj = (HAL_Obj *)handle;
  PIE_Obj *pie = (PIE_Obj *)obj->pieHandle;


  ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  pie->EPWM1_INT = &led2ISR;
  pie->ADCINT1 = &led3OffISR;
  pie->ADCINT2 = &led3OnISR;
  pie->ADCINT6 = &mainISR;

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  return;
 } // end of HAL_initIntVectorTable() function


//! \brief     Reads the ADC data 
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pAdcData  A pointer to the ADC data buffer
static inline void HAL_readAdcData(HAL_Handle handle,HAL_AdcData_t *pAdcData)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  float_t value;
  float_t current_sf = HAL_getCurrentScaleFactor(handle);
  float_t voltage_sf = HAL_getVoltageScaleFactor(handle);


  // convert phase A current
  value = (float_t)ADC_readResult(obj->adcHandle,ADC_ResultNumber_1);
  value = value * current_sf - obj->adcBias.I_A.value[0];
  pAdcData->I_A.value[0] = value;

  // convert phase B current
  value = (float_t)ADC_readResult(obj->adcHandle,ADC_ResultNumber_2);
  value = value * current_sf - obj->adcBias.I_A.value[1];
  pAdcData->I_A.value[1] = value;

  // convert phase C current
  value = (float_t)ADC_readResult(obj->adcHandle,ADC_ResultNumber_3);
  value = value * current_sf - obj->adcBias.I_A.value[2];
  pAdcData->I_A.value[2] = value;

  // convert phase A voltage
  value = (float_t)ADC_readResult(obj->adcHandle,ADC_ResultNumber_4);
  value = value * voltage_sf - obj->adcBias.V_V.value[0];
  pAdcData->V_V.value[0] = value;

  // convert phase B voltage
  value = (float_t)ADC_readResult(obj->adcHandle,ADC_ResultNumber_5);
  value = value * voltage_sf - obj->adcBias.V_V.value[1];
  pAdcData->V_V.value[1] = value;

  // convert phase C voltage
  value = (float_t)ADC_readResult(obj->adcHandle,ADC_ResultNumber_6);
  value = value * voltage_sf - obj->adcBias.V_V.value[2];
  pAdcData->V_V.value[2] = value;

  // convert dcBus voltage
  value = (float_t)ADC_readResult(obj->adcHandle,ADC_ResultNumber_7);
  value = value * voltage_sf;
  pAdcData->dcBus_V = value;

  return;
} // end of HAL_readAdcData() function


//! \brief     Reads the timer count
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] timerNumber  The timer number, 0,1 or 2
static inline uint32_t HAL_readTimerCnt(HAL_Handle handle,const uint_least8_t timerNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  uint32_t timerCnt = TIMER_getCount(obj->timerHandle[timerNumber]);

  return(timerCnt);
} // end of HAL_readTimerCnt() function


//! \brief     Resets the PWM count
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] pwmNumber    The PWM number
static inline void HAL_resetPwmCount(HAL_Handle handle,
                                     PWM_Number_e pwmNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  PWM_setCount(obj->pwmHandle[pwmNumber],0);

  return;
} // HAL_resetPwmCount() function


//! \brief     Sets the ADC SOC sample delay value
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] socNumber    The SOC number
//! \param[in] sampleDelay  The delay value for the ADC
static inline void HAL_setAdcSocSampleDelay(HAL_Handle handle,
                                            const ADC_SocNumber_e socNumber,
                                            const ADC_SocSampleDelay_e sampleDelay)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  ADC_setSocSampleDelay(obj->adcHandle,socNumber,sampleDelay);

  return;
} // end of HAL_setAdcSocSampleDelay() function


//! \brief     Sets the ADC bias value
//! \param[in] handle        The hardware abstraction layer (HAL) handle
//! \param[in] sensorType    The sensor type
//! \param[in] sensorNumber  The sensor number
//! \param[in] bias          The ADC bias value
static inline void HAL_setBias(HAL_Handle handle,
                               const HAL_SensorType_e sensorType,
                               uint_least8_t sensorNumber,
                               const float_t bias)
{
  HAL_Obj *obj = (HAL_Obj *)handle;


  if(sensorType == HAL_SensorType_Current)
    {
      obj->adcBias.I_A.value[sensorNumber] = bias;
    }
  else if(sensorType == HAL_SensorType_Voltage)
    {
      obj->adcBias.V_V.value[sensorNumber] = bias;
    }

  return;
} // end of HAL_setBias() function


//! \brief     Sets the current scale factor in the hal
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] current_sf  The current scale factor
static inline void HAL_setCurrentScaleFactor(HAL_Handle handle,const float_t current_sf)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  

  obj->current_sf = current_sf;

  return;
} // end of HAL_setCurrentScaleFactor() function


//! \brief     Sets the GPIO pin high
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] gpioNumber  The GPIO number
static inline void HAL_setGpioHigh(HAL_Handle handle,const GPIO_Number_e gpioNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  

  // set GPIO high
  GPIO_setHigh(obj->gpioHandle,gpioNumber);

  return;
} // end of HAL_setGpioHigh() function


//! \brief     Sets the GPIO pin low
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] gpioNumber  The GPIO number
static inline void HAL_setGpioLow(HAL_Handle handle,const GPIO_Number_e gpioNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  

  // set GPIO low
  GPIO_setLow(obj->gpioHandle,gpioNumber);

  return;
} // end of HAL_setGpioLow() function


//! \brief     Sets the number of current sensors
//! \param[in] handle             The hardware abstraction layer (HAL) handle
//! \param[in] numCurrentSensors  The number of current sensors
static inline void HAL_setNumCurrentSensors(HAL_Handle handle,const uint_least8_t numCurrentSensors)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  

  obj->numCurrentSensors = numCurrentSensors;

  return;
} // end of HAL_setNumCurrentSensors() function


//! \brief     Sets the number of voltage sensors
//! \param[in] handle             The hardware abstraction layer (HAL) handle
//! \param[in] numVoltageSensors  The number of voltage sensors
static inline void HAL_setNumVoltageSensors(HAL_Handle handle,const uint_least8_t numVoltageSensors)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  

  obj->numVoltageSensors = numVoltageSensors;

  return;
} // end of HAL_setNumVoltageSensors() function


//! \brief     Sets the PWM frequency
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] pwmNumber    The PWM number
//! \param[in] pwmFreq_kHz  The PWM frequency, kHz
static inline void HAL_setPwmFreq_kHz(HAL_Handle handle,
                                      PWM_Number_e pwmNumber,
                                      HAL_PwmFreq_e pwmFreq_kHz)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  PWM_setPeriod(obj->pwmHandle[pwmNumber],pwmFreq_kHz);

  return;
} // HAL_setPwmFreq_Hz() function


//! \brief     Sets the voltage scale factor in the hal
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] voltage_sf  The voltage scale factor
static inline void HAL_setVoltageScaleFactor(HAL_Handle handle,const float_t voltage_sf)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  
  obj->voltage_sf = voltage_sf;

  return;
} // end of HAL_setVoltageScaleFactor() function


//! \brief     Sets the hal parameters
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] pUserParams  The pointer to the user parameters
extern void HAL_setParams(HAL_Handle handle,const USER_Params *pUserParams);


//! \brief     Sets up the ADCs (Analog to Digital Converters)
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupAdcs(HAL_Handle handle);


//! \brief     Sets up the clocks
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupClks(HAL_Handle handle);


//! \brief     Sets up the faults
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupFaults(HAL_Handle handle);


//! \brief     Sets up the GATE object
//! \param[in] handle       The hardware abstraction layer (HAL) handle
void HAL_setupGate(HAL_Handle handle);


//! \brief     Sets up the GPIO (General Purpose I/O) pins
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupGpios(HAL_Handle handle);


//! \brief     Sets up the FLASH.
extern void HAL_setupFlash(HAL_Handle handle);


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
                          const uint_least16_t systemFreq_MHz,
                          const float_t pwmPeriod_usec,
                          const uint_least16_t numPwmTicksPerIsrTick);


//! \brief     Sets up the PWM DACs (Pulse Width Modulator Digital to Analof Converters)
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] systemFreq_MHz  The system frequency, MHz
//! \param[in] dacFreq_kHz     The DAC frequency, kHz
extern void HAL_setupPwmDacs(HAL_Handle handle,
                             const uint_least16_t systemFreq_MHz,
                             const uint_least16_t dacFreq_kHz);


//! \brief     Sets up the spiB peripheral
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupSpiB(HAL_Handle handle);


//! \brief     Sets up the timers
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] systemFreq_MHz  The system frequency, MHz
void HAL_setupTimers(HAL_Handle handle,const uint_least16_t systemFreq_MHz);


//! \brief     Toggles the GPIO pin
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] gpioNumber  The GPIO number
static inline void HAL_toggleGpio(HAL_Handle handle,const GPIO_Number_e gpioNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;


  // toggle the GPIO
  GPIO_toggle(obj->gpioHandle,gpioNumber);

  return;
} // end of HAL_toggleGpio() function


//! \brief     Updates the ADC bias values
//! \param[in] handle           The hardware abstraction layer (HAL) handle
//! \param[in] pOffsetHandle_I  The pointer to the the current offset handles
//! \param[in] pOffsetHandle_V  The pointer to the the voltage offset handles
static inline void HAL_updateAdcBias(HAL_Handle handle,
                                     OFFSET_Handle *pOffsetHandle_I,
                                     OFFSET_Handle *pOffsetHandle_V)
{
  uint_least8_t cnt;


  // update the current bias
  for(cnt=0;cnt<HAL_getNumCurrentSensors(handle);cnt++)
    {
      float_t bias = HAL_getBias(handle,HAL_SensorType_Current,cnt);
      
      bias += OFFSET_getOffset(pOffsetHandle_I[cnt]);

      HAL_setBias(handle,HAL_SensorType_Current,cnt,bias);
    }


  // update the voltage bias
  for(cnt=0;cnt<HAL_getNumVoltageSensors(handle);cnt++)
    {
      float_t bias = HAL_getBias(handle,HAL_SensorType_Voltage,cnt);

      bias += OFFSET_getOffset(pOffsetHandle_V[cnt]);

      HAL_setBias(handle,HAL_SensorType_Voltage,cnt,bias);
    }

  return;
} // end of HAL_updateAdcBias() function


//! \brief     Writes DAC data to the PWM comparators for DAC (digital-to-analog conversion) output
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pDacData  The pointer to the DAC data
static inline void HAL_writeDacData(HAL_Handle handle,const HAL_DacData_t *pDacData)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  uint_least8_t pwmCnt;
  uint_least8_t dataCnt=0;

  for(pwmCnt=0;pwmCnt<3;pwmCnt++)
    {
      PWM_Obj *pwm = (PWM_Obj *)obj->pwmDacHandle[pwmCnt];

      // compute the value
      float_t period = (float_t)(pwm->TBPRD);
      float_t V_pu = pDacData->value[dataCnt];
      float_t V_sat_pu = MATH_sat(V_pu,1.0,-1.0);
      float_t V_sat_dc_pu = 0.5 * (V_sat_pu + 1.0);
      int16_t pwmValue  = (int16_t)(V_sat_dc_pu * period);

      // increment the data counter
      dataCnt++;

      // write the PWM data value
      PWM_write_CmpA(obj->pwmDacHandle[pwmCnt],pwmValue);

      if(pwmCnt == 0)
        {
          float_t V_pu = pDacData->value[dataCnt];
          float_t V_sat_pu = MATH_sat(V_pu,1.0,-1.0);
          float_t V_sat_dc_pu = 0.5 * (V_sat_pu + 1.0);
          int16_t pwmValue  = (int16_t)(V_sat_dc_pu * period);
          
          // write the PWM data value
          PWM_write_CmpB(obj->pwmDacHandle[pwmCnt],pwmValue);

          // increment the data counter
          dataCnt++;
        }
    }

  return;
} // end of HAL_writeDacData() function


//! \brief     Writes PWM data to the PWM comparators for motor control
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pPwmData  The pointer to the PWM data
static inline void HAL_writePwmData(HAL_Handle handle,const HAL_PwmData_t *pPwmData)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  uint_least8_t pwmCnt;

  for(pwmCnt=0;pwmCnt<3;pwmCnt++)
    {
      PWM_Obj *pwm = (PWM_Obj *)obj->pwmHandle[pwmCnt];

      // compute the value
      float_t period = (float_t)(pwm->TBPRD);
      float_t V_pu = -pPwmData->Vabc_pu.value[pwmCnt];
      float_t V_sat_pu = MATH_sat(V_pu,1.0,-1.0);
      float_t V_sat_dc_pu = 0.5 * (V_sat_pu + 1.0);
      int16_t pwmValue  = (int16_t)(V_sat_dc_pu * period);

      // write the PWM data value
      PWM_write_CmpA(obj->pwmHandle[pwmCnt],pwmValue);
    }

  return;
} // end of HAL_writePwmData() function

//! \brief     Reads PWM period register
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \param[in] pwmNumber  The PWM number
//! \return    The PWM period value
static inline uint16_t HAL_readPwmPeriod(HAL_Handle handle,const PWM_Number_e pwmNumber)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  // the period value to be returned
  uint16_t pwmPeriodValue;

  pwmPeriodValue = PWM_getPeriod(obj->pwmHandle[pwmNumber]);

  return(pwmPeriodValue);
} // end of HAL_readPwmPeriod() function


//! \brief     Writes data to the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] Spi_8301_Vars  SPI variables
void HAL_writeDrvData(HAL_Handle handle, DRV_SPI_8301_Vars_t *Spi_8301_Vars);


//! \brief     Reads data from the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] Spi_8301_Vars  SPI variables
void HAL_readDrvData(HAL_Handle handle, DRV_SPI_8301_Vars_t *Spi_8301_Vars);


//! \brief     Sets up the SPI interface for the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] Spi_8301_Vars  SPI variables
void HAL_setupDrvSpi(HAL_Handle handle, DRV_SPI_8301_Vars_t *Spi_8301_Vars);


#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup


#endif // end of _HAL_H_ definition

