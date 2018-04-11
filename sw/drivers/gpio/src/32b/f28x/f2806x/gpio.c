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
//! \file   drivers/gpio/src/32b/f28x/f2806x/gpio.c
//! \brief  The functions in this file are used to configure the general
//!         purpose I/O (GPIO) registers
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes


#include "sw/drivers/gpio/src/32b/f28x/f2806x/gpio.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions

bool GPIO_getData(GPIO_Handle gpioHandle, const GPIO_Number_e gpioNumber)
{
	GPIO_Obj *pgpio = (GPIO_Obj *)gpioHandle;


	if(gpioNumber < GPIO_Number_32)
    {
        return (bool)((pgpio->GPADAT >> gpioNumber) & 0x0001);
    }
	else
    {
        return (bool)((pgpio->GPBDAT >> (gpioNumber - GPIO_Number_32)) & 0x0001);
    }

} // end of GPIO_getData() function


uint16_t GPIO_getPortData(GPIO_Handle gpioHandle, const GPIO_Port_e gpioPort)
{
	GPIO_Obj *pgpio = (GPIO_Obj *)gpioHandle;

	if(gpioPort == GPIO_Port_A)
    {
        return (pgpio->GPADAT);
    }
	else if(gpioPort == GPIO_Port_B)
    {
        return (pgpio->GPBDAT);
    }

	return (NULL);

} // end of GPIO_getPortData() function


GPIO_Handle GPIO_init(void *pMemory,const size_t numBytes)
{
	GPIO_Handle gpioHandle;


	if(numBytes < sizeof(GPIO_Obj))
    {
      return((GPIO_Handle)NULL);
    }

	// assign the handle
	gpioHandle = (GPIO_Handle)pMemory;

	return(gpioHandle);
} // end of GPIO_init() function


void GPIO_setPullup(GPIO_Handle gpioHandle,const GPIO_Number_e gpioNumber,const GPIO_Pullup_e pullup)
{
	GPIO_Obj *pgpio = (GPIO_Obj *)gpioHandle;


	ENABLE_PROTECTED_REGISTER_WRITE_MODE;

	if(gpioNumber < GPIO_Number_32)
    {
		// clear the bit
		pgpio->GPAPUD &= (~((uint32_t)1 << gpioNumber));

		// set the bit
		pgpio->GPAPUD |= (uint32_t)pullup << gpioNumber;
    }
	else
    {
		// clear the bit
		pgpio->GPBPUD &= (~((uint32_t)1 << (gpioNumber - GPIO_Number_32)));

		// set the bit
		pgpio->GPBPUD |= (uint32_t)pullup << (gpioNumber - GPIO_Number_32);
    }

  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

} // end of GPIO_setPullup() function


void GPIO_setDirection(GPIO_Handle gpioHandle,const GPIO_Number_e gpioNumber,
					   const GPIO_Direction_e direction)
{
	GPIO_Obj *pgpio = (GPIO_Obj *)gpioHandle;


	ENABLE_PROTECTED_REGISTER_WRITE_MODE;

	if(gpioNumber < GPIO_Number_32)
    {
		// clear the bit
		pgpio->GPADIR &= (~((uint32_t)1 << gpioNumber));

		// set the bit
		pgpio->GPADIR |= (uint32_t)direction << gpioNumber;
    }
	else
    {
		// clear the bit
		pgpio->GPBDIR &= (~((uint32_t)1 << (gpioNumber - GPIO_Number_32)));

		// set the bit
		pgpio->GPBDIR |= (uint32_t)direction << (gpioNumber - GPIO_Number_32);
    }

	DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of GPIO_setDirection() function


void GPIO_setExtInt(GPIO_Handle gpioHandle,const GPIO_Number_e gpioNumber,const CPU_ExtIntNumber_e intNumber)
{
	GPIO_Obj *pgpio = (GPIO_Obj *)gpioHandle;


	ENABLE_PROTECTED_REGISTER_WRITE_MODE;

	// associate the interrupt with the GPIO pin
	pgpio->GPIOXINTnSEL[intNumber] = gpioNumber;

	DISABLE_PROTECTED_REGISTER_WRITE_MODE;

} // end of GPIO_setExtInt() function


bool GPIO_read(GPIO_Handle gpioHandle,const GPIO_Number_e gpioNumber)
{
	GPIO_Obj *pgpio = (GPIO_Obj *)gpioHandle;
	bool bgpio_status = 0;
	uint32_t u32gpio_read = 0;


	if(gpioNumber < GPIO_Number_32)
    {
		u32gpio_read = (pgpio->GPADAT) & ((uint32_t)1 << gpioNumber);
    }
	else
    {
		u32gpio_read = (pgpio->GPBDAT) & ((uint32_t)1 << (gpioNumber - GPIO_Number_32));
    }

	if(u32gpio_read == 0)
    {
		bgpio_status = LOW;
    }
	else
    {
		bgpio_status = HIGH;
    }


  return(bgpio_status);
} // end of GPIO_read() function


void GPIO_setHigh(GPIO_Handle gpioHandle,const GPIO_Number_e gpioNumber)
{
	GPIO_Obj *pgpio = (GPIO_Obj *)gpioHandle;


	ENABLE_PROTECTED_REGISTER_WRITE_MODE;

	if(gpioNumber < GPIO_Number_32)
    {
		pgpio->GPASET = (uint32_t)1 << gpioNumber;//$0000 0000 0000 0000 0000 0000 0000 0001-->right shift gpioNumber
    }
	else
    {
		pgpio->GPBSET = (uint32_t)1 << (gpioNumber - GPIO_Number_32);
    }

	DISABLE_PROTECTED_REGISTER_WRITE_MODE;

} // end of GPIO_setHigh() function


void GPIO_setLow(GPIO_Handle gpioHandle,const GPIO_Number_e gpioNumber)
{
	GPIO_Obj *pgpio = (GPIO_Obj *)gpioHandle;


	ENABLE_PROTECTED_REGISTER_WRITE_MODE;

	if(gpioNumber < GPIO_Number_32)
    {
		pgpio->GPACLEAR = (uint32_t)1 << gpioNumber;
    }
	else
    {
		pgpio->GPBCLEAR = (uint32_t)1 << (gpioNumber - GPIO_Number_32);
    }

	DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of GPIO_setLow() function


void GPIO_setMode(GPIO_Handle gpioHandle,const GPIO_Number_e gpioNumber,const GPIO_Mode_e mode)
{
	GPIO_Obj *pgpio = (GPIO_Obj *)gpioHandle;


	ENABLE_PROTECTED_REGISTER_WRITE_MODE;

	if(gpioNumber < (GPIO_GPMUX_NUMGPIOS_PER_REG * 1))
    {
		uint_least8_t u8lShift = gpioNumber << 1;
		uint32_t u32clearBits = (uint32_t)GPIO_GPMUX_CONFIG_BITS << u8lShift;
		uint32_t u32setBits = (uint32_t)mode << u8lShift;

		// clear the bits
		pgpio->GPAMUX1 &= (~u32clearBits);

		// set the bits
		pgpio->GPAMUX1 |= u32setBits;
    }
	else if(gpioNumber < (GPIO_GPMUX_NUMGPIOS_PER_REG * 2))
    {
		uint_least8_t u8lShift = (gpioNumber - (GPIO_GPMUX_NUMGPIOS_PER_REG * 1)) << 1;
		uint32_t u32clearBits = (uint32_t)GPIO_GPMUX_CONFIG_BITS << u8lShift;
		uint32_t u32setBits = (uint32_t)mode << u8lShift;

		// clear the bits
		pgpio->GPAMUX2 &= (~u32clearBits);

		// set the bits
		pgpio->GPAMUX2 |= u32setBits;
    }
	else if(gpioNumber < (GPIO_GPMUX_NUMGPIOS_PER_REG * 3))
    {
		uint_least8_t u8lShift = (gpioNumber - (GPIO_GPMUX_NUMGPIOS_PER_REG * 2)) << 1;
		uint32_t u32clearBits = (uint32_t)GPIO_GPMUX_CONFIG_BITS << u8lShift;
		uint32_t u32setBits = (uint32_t)mode << u8lShift;

		// clear the bits
		pgpio->GPBMUX1 &= (~u32clearBits);

		// set the bits
		pgpio->GPBMUX1 |= u32setBits;
    }
	else if(gpioNumber < (GPIO_GPMUX_NUMGPIOS_PER_REG * 4))
    {
		uint_least8_t u8lShift = (gpioNumber - (GPIO_GPMUX_NUMGPIOS_PER_REG * 3)) << 1;
		uint32_t u32clearBits = (uint32_t)GPIO_GPMUX_CONFIG_BITS << u8lShift;
		uint32_t u32setBits = (uint32_t)mode << u8lShift;

		// clear the bits
		pgpio->GPBMUX2 &= (~u32clearBits);

		// set the bits
		pgpio->GPBMUX2 |= u32setBits;
    }

	DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of GPIO_setMode() function


void GPIO_setPortData(GPIO_Handle gpioHandle, const GPIO_Port_e gpioPort, const uint16_t u16data)
{
	GPIO_Obj *pgpio = (GPIO_Obj *)gpioHandle;

	ENABLE_PROTECTED_REGISTER_WRITE_MODE;

	if(gpioPort == GPIO_Port_A)
    {
        pgpio->GPADAT = u16data;
    }
	else if(gpioPort == GPIO_Port_B)
    {
        pgpio->GPBDAT = u16data;
    }

	DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of GPIO_setPortData() function

void GPIO_setQualification(GPIO_Handle gpioHandle, const GPIO_Number_e gpioNumber, const GPIO_Qual_e qualification)
{
	GPIO_Obj *pgpio = (GPIO_Obj *)gpioHandle;


	ENABLE_PROTECTED_REGISTER_WRITE_MODE;

  	if(gpioNumber <= ((GPIO_GPxQSELx_NUMGPIOS_PER_REG * 1) - 1))
    {
        uint32_t u32clearBits = (uint32_t)GPIO_GPxQSELy_GPIOx_BITS << (2 * gpioNumber);
        pgpio->GPAQSEL1 &= ~(u32clearBits);
        pgpio->GPAQSEL1 |= (uint32_t)qualification << (2 * gpioNumber);
    }
  	else if(gpioNumber <= ((GPIO_GPxQSELx_NUMGPIOS_PER_REG * 2) - 1))
    {
        uint32_t u32clearBits = (uint32_t)GPIO_GPxQSELy_GPIOx_BITS << (2 * (gpioNumber - (GPIO_GPxQSELx_NUMGPIOS_PER_REG * 1)));
        pgpio->GPAQSEL2 &= ~(u32clearBits);
        pgpio->GPAQSEL2 |= (uint32_t)qualification << (2 * (gpioNumber - (GPIO_GPxQSELx_NUMGPIOS_PER_REG * 1)));
    }
  	else if(gpioNumber <= ((GPIO_GPxQSELx_NUMGPIOS_PER_REG * 3) - 1))
    {
        uint32_t u32clearBits = (uint32_t)GPIO_GPxQSELy_GPIOx_BITS << (2 * (gpioNumber - (GPIO_GPxQSELx_NUMGPIOS_PER_REG * 2)));
        pgpio->GPBQSEL1 &= ~(u32clearBits);
        pgpio->GPBQSEL1 |= (uint32_t)qualification << (2 * (gpioNumber - (GPIO_GPxQSELx_NUMGPIOS_PER_REG * 2)));
    }
  	else if(gpioNumber <= ((GPIO_GPxQSELx_NUMGPIOS_PER_REG * 4) - 1))
    {
        uint32_t u32clearBits = (uint32_t)GPIO_GPxQSELy_GPIOx_BITS << (2 * (gpioNumber - (GPIO_GPxQSELx_NUMGPIOS_PER_REG * 3)));
        pgpio->GPBQSEL2 &= ~(u32clearBits);
        pgpio->GPBQSEL2 |= (uint32_t)qualification << (2 * (gpioNumber - (GPIO_GPxQSELx_NUMGPIOS_PER_REG * 3)));
    }

  	DISABLE_PROTECTED_REGISTER_WRITE_MODE;

} // end of GPIO_setQualification() function

void GPIO_setQualificationPeriod(GPIO_Handle gpioHandle, const GPIO_Number_e gpioNumber, const uint_least8_t period)
{
	GPIO_Obj *pgpio = (GPIO_Obj *)gpioHandle;

	ENABLE_PROTECTED_REGISTER_WRITE_MODE;

	if(gpioNumber <= ((GPIO_GPxCTRL_QUALPRDx_NUMBITS_PER_REG * 1) - 1))
    {
        uint32_t u32clearBits = (uint32_t)GPIO_GPxCTRL_QUALPRDx_BITS;
        pgpio->GPACTRL &= ~(u32clearBits);
        pgpio->GPACTRL |= (uint32_t)period;
    }
	else if(gpioNumber <= ((GPIO_GPxCTRL_QUALPRDx_NUMBITS_PER_REG * 2) - 1))
    {
        uint32_t u32clearBits = (uint32_t)GPIO_GPxCTRL_QUALPRDx_BITS << 8;
        pgpio->GPACTRL &= ~(u32clearBits);
        pgpio->GPACTRL |= (uint32_t)period << 8;
    }
	else if(gpioNumber <= ((GPIO_GPxCTRL_QUALPRDx_NUMBITS_PER_REG * 3) - 1))
    {
        uint32_t u32clearBits = (uint32_t)GPIO_GPxCTRL_QUALPRDx_BITS << 16;
        pgpio->GPACTRL &= ~(u32clearBits);
        pgpio->GPACTRL |= (uint32_t)period << 16;
    }
	else if(gpioNumber <= ((GPIO_GPxCTRL_QUALPRDx_NUMBITS_PER_REG * 4) - 1))
    {
        uint32_t u32clearBits = (uint32_t)GPIO_GPxCTRL_QUALPRDx_BITS << 24;
        pgpio->GPACTRL &= ~(u32clearBits);
        pgpio->GPACTRL |= (uint32_t)period << 24;
    }
	else if(gpioNumber <= ((GPIO_GPxCTRL_QUALPRDx_NUMBITS_PER_REG * 5) - 1))
    {
        uint32_t u32clearBits = (uint32_t)GPIO_GPxCTRL_QUALPRDx_BITS;
        pgpio->GPBCTRL &= ~(u32clearBits);
        pgpio->GPBCTRL |= (uint32_t)period;
    }
	else if(gpioNumber <= ((GPIO_GPxCTRL_QUALPRDx_NUMBITS_PER_REG * 6) - 1))
    {
        uint32_t u32clearBits = (uint32_t)GPIO_GPxCTRL_QUALPRDx_BITS << 8;
        pgpio->GPBCTRL &= ~(u32clearBits);
        pgpio->GPBCTRL |= (uint32_t)period << 8;
    }
	else if(gpioNumber <= ((GPIO_GPxCTRL_QUALPRDx_NUMBITS_PER_REG * 7) - 1))
    {
        uint32_t u32clearBits = (uint32_t)GPIO_GPxCTRL_QUALPRDx_BITS << 16;
        pgpio->GPBCTRL &= ~(u32clearBits);
        pgpio->GPBCTRL |= (uint32_t)period << 16;
    }
	else if(gpioNumber <= ((GPIO_GPxCTRL_QUALPRDx_NUMBITS_PER_REG * 8) - 1))
    {
        uint32_t u32clearBits = (uint32_t)GPIO_GPxCTRL_QUALPRDx_BITS << 24;
        pgpio->GPBCTRL &= ~(u32clearBits);
        pgpio->GPBCTRL |= (uint32_t)period << 24;
    }

	DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of GPIO_setQualificationPeriod() function


void GPIO_toggle(GPIO_Handle gpioHandle,const GPIO_Number_e gpioNumber)
{
	GPIO_Obj *pgpio = (GPIO_Obj *)gpioHandle;


	ENABLE_PROTECTED_REGISTER_WRITE_MODE;

	if(gpioNumber < GPIO_Number_32)
    {
		pgpio->GPATOGGLE = (uint32_t)1 << gpioNumber;
    }
	else
    {
		pgpio->GPBTOGGLE = (uint32_t)1 << (gpioNumber - GPIO_Number_32);
    }

	DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of GPIO_toggle() function


void GPIO_lpmSelect(GPIO_Handle gpioHandle,const GPIO_Number_e gpioNumber)
{
	GPIO_Obj *pgpio = (GPIO_Obj *)gpioHandle;

	ENABLE_PROTECTED_REGISTER_WRITE_MODE;

	pgpio->GPIOLPMSEL |= ((uint32_t)1 << gpioNumber);

	DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of GPIO_lpmSelect() function
