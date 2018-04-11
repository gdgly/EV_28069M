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

#ifndef QEP
#include "sw/modules/ctrl/src/32b/ctrl.h"
#else
#include "sw/modules/ctrl/src/32b/ctrlQEP.h"
#endif

#include "i2c_lcd.h"

//#include "hal.h"
#include "sw/modules/hal/boards/hvkit_rev1p1/f28x/f2806x/src/hal.h"
#include "user_data.h"


// **************************************************************************
// the defines
// **************************************************************************
// the globals

#define LCD_DEVICE_ADDRESS 0x27

#define LCD_EN  0x04  // Enable bit
#define LCD_RW  0x02  // Rea/Write bit
#define LCD_RS  0x01  // Reg. select bit
#define LCD_BACKLIGHT 0x08	//Background light

typedef enum
{
	LCD_CLEARDISPLAY 	= 	0x01,
	LCD_RETURNHOME 	=		0x02,
	LCD_ENTRYMODESET =		0x04,
	LCD_DISPLAYCONTROL = 	0x08,
	LCD_CURSORSHIFT =		0x10,
	LCD_FUNCTIONSET =		0x20,
	LCD_SETCGRAMADDR =		0x40,
	LCD_SETDDRAMADDR =		0x80
} LCD_FUNCTION_e;

typedef enum
{
	LCD_ENTRYRIGHT = 0x00,
	LCD_ENTRYLEFT = 0x02,
	LCD_ENTRYSHIFTINCREMENT = 0x01,
	LCD_ENTRYSHIFTDECREMENT = 0x00
} LCD_ENTRYMODE_e;

typedef enum
{
	LCD_DISPLAYON =	0x04,
	LCD_DISPLAYOFF = 0x00,
	LCD_CURSORON =	0x02,
	LCD_CURSOROFF =	0x00,
	LCD_BLINKON = 0x01,
	LCD_BLINKOFF = 0x00
} LCD_DISPLAY_e;

typedef enum
{
	LCD_DISPLAYMOVE =	0x08,
	LCD_CURSORMOVE =	0x00,
	LCD_MOVERIGHT =	0x04,
	LCD_MOVELEFT =	0x00
} LCD_DISPLAY_SHIFT_e;


typedef enum
{
	LCD_8BITMODE =	0x10,
	LCD_4BITMODE =	0x00,
	LCD_2LINE = 0x08,
	LCD_1LINE 	= 0x00,
	LCD_5_10DOTS = 0x04,
	LCD_5_8DOTS  =	0x00
} LCD_FUNCTIONSET_e;



LCD_Handle LCD_init(void *pMemory,const size_t numBytes)
{
	LCD_Handle lcdHandle;

	if (numBytes < sizeof(LCD1602_Obj))
		return((LCD_Handle)NULL);

	// assign the handle
	lcdHandle = (LCD_Handle)pMemory;
	return(lcdHandle);
}



void LCD_setup(LCD_Handle lcdHandle, I2CMessage_Handle i2cMessageHandle)
{
	LCD1602_Obj *pLCD = (LCD1602_Obj *) lcdHandle;
	pLCD->i2cMessageHandle = i2cMessageHandle;


	//DATA_Delay(100);		// Delay 100 ms
	usDelay(100000);
	LCD_writeCmd8bits(lcdHandle, LCD_FUNCTIONSET | LCD_8BITMODE);

	//DATA_Delay(5);		// Delay 5 ms
	usDelay(5000);
	LCD_writeCmd8bits(lcdHandle, LCD_FUNCTIONSET | LCD_8BITMODE);
	//DATA_Delay(1);		// Delay 5 ms
	usDelay(2000);
	LCD_writeCmd8bits(lcdHandle, LCD_FUNCTIONSET | LCD_8BITMODE);
	LCD_writeCmd8bits(lcdHandle, LCD_FUNCTIONSET | LCD_8BITMODE);

	LCD_writeCmd8bits(lcdHandle, LCD_FUNCTIONSET | LCD_4BITMODE);

	LCD_writeCmd4bits(lcdHandle, LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5_10DOTS );

	LCD_writeCmd4bits(lcdHandle, LCD_DISPLAYCONTROL |LCD_DISPLAYON |LCD_CURSOROFF 	|LCD_BLINKOFF);

	//LCD_clearDisplay(lcdHandle);
	//LCD_entryMode(lcdHandle);
	//LCD_Home(lcdHandle);
}

bool LCD_writeCommnication(LCD_Handle lcdHandle, uint_least8_t *pu8Data, uint_least8_t u8No)
{
	uint_least8_t i;
	I2C_MessageCode_e i2cMessageCode;
	LCD1602_Obj *pLCD = (LCD1602_Obj *) lcdHandle;
	I2C_Message *pI2cMessage= (I2C_Message *)  pLCD->i2cMessageHandle;;


	pI2cMessage->u16SlaveAddress = LCD_DEVICE_ADDRESS;
	for (i=0; i< u8No; i++)
		pI2cMessage->au8TxBuffer[i] = *pu8Data++;

	pI2cMessage->u8TxNumOfBytes = u8No;
	pI2cMessage->u8RxNumOfBytes = 0;

	I2CMessage_WriteMessage( pI2cMessage, true);
	i2cMessageCode = I2CMessage_Synchronize(pI2cMessage);
	return (i2cMessageCode ==I2C_MSGSTAT_INACTIVE)? true : false;
}

void LCD_setDataBuff(uint_least8_t u8Data, uint_least8_t *pu8Data)
{
	*pu8Data++ = u8Data;
	*pu8Data++ = u8Data | LCD_EN ;	//Enable
	*pu8Data++ = u8Data | LCD_EN ;	//Enable
	*pu8Data++ = u8Data | LCD_EN ;	//Enable
	*pu8Data++ = u8Data;
	*pu8Data   = u8Data;
}

bool LCD_writeCmd8bits(LCD_Handle lcdHandle, uint_least8_t u8Data)
{
	uint_least8_t u8Value;
	uint_least8_t au8Data[6];


	u8Value = (u8Data & 0x00f0) | LCD_BACKLIGHT ;
	LCD_setDataBuff(u8Value, &au8Data[0]);

	return LCD_writeCommnication(lcdHandle, au8Data, 6);
}

bool LCD_writeCmd4bits(LCD_Handle lcdHandle, uint_least8_t u8Data)
{
	uint_least8_t u8Value;
	uint_least8_t au8Data[12];

	u8Value = u8Data & 0x00f0;
	LCD_setDataBuff(LCD_BACKLIGHT | u8Value, &au8Data[0]);

	u8Value = (u8Data << 4) & 0x00f0;
	LCD_setDataBuff(LCD_BACKLIGHT | u8Value, &au8Data[6]);

	return LCD_writeCommnication(lcdHandle, au8Data, 12);
}

bool LCD_writeChar4bits(LCD_Handle lcdHandle,uint_least8_t u8Data)
{
	uint_least8_t u8Value;
	uint_least8_t au8Data[12];

	u8Value = (u8Data & 0x00f0) | LCD_RS | LCD_BACKLIGHT;
	LCD_setDataBuff(u8Value, &au8Data[0]);

	u8Value = ((u8Data << 4) & 0x00f0) | LCD_RS | LCD_BACKLIGHT;
	LCD_setDataBuff(u8Value, &au8Data[6]);

	return LCD_writeCommnication(lcdHandle,au8Data, 12);
}

bool LCD_clearDisplay(LCD_Handle lcdHandle)
{
	bool bResult;
	bResult = LCD_writeCmd4bits(lcdHandle,LCD_CLEARDISPLAY);
	//DATA_Delay(2);		// Delay 2 ms
	usDelay(2000);
	return bResult;

}

 bool LCD_entryMode(LCD_Handle lcdHandle)
 {
	 return LCD_writeCmd4bits( lcdHandle,LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
 }

 bool LCD_Home(LCD_Handle lcdHandle)
 {
	 bool bResult;
	 bResult = LCD_writeCmd4bits(lcdHandle,LCD_RETURNHOME);
	 //DATA_Delay(2);		// Delay 2 ms
	 usDelay(2000);
	 return bResult;

 }

 bool LCD_setCursor(LCD_Handle lcdHandle,uint_least8_t u8Col, uint_least8_t u8Row)
 {
     static uint_least8_t au8RowOffsets[]
  	        = { 0x00, 0x40, 0x14, 0x54 };

     uint_least8_t u8Add = u8Col  +au8RowOffsets[u8Row];
     return LCD_writeCmd4bits( lcdHandle,LCD_SETDDRAMADDR |  u8Add );
 }

 void LCD_outString(LCD_Handle lcdHandle, char *pString)
 {
	 while (*pString != 0)
		 LCD_writeChar4bits(lcdHandle,*pString++);


 }




// end of file
