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
#ifndef _SCI_OPERATOR_H_
#define _SCI_OPERATOR_H_

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
#include "sci_message.h"
#include "user_data.h"





#ifdef __cplusplus
extern "C" {
#endif


typedef enum
{
	OPERATOR_INIT1 = 0x00,
	OPERATOR_INIT2 = 0x01,
	OPERATOR_START = 0x02
} OPERRATOR_CommStatus_e;

typedef enum
{
	OPERATOR_LOCAL_KEY = (1 << 10),
	OPERATOR_STOP_KEY  = (1 << 9),
	OPERATOR_RUN_KEY   = (1 << 8),
	OPERATOR_RESET_KEY  = (1 << 7),
	OPERATOR_ENTER_KEY  = (1 << 6),
	OPERATOR_JOG_KEY    = (1 << 5),
	OPERATOR_MENU_KEY   = (1 << 4),
	OPERATOR_DOWN_KEY   = (1 << 3),
	OPERATOR_UP_KEY     = (1 << 2),
	OPERATOR_FWR_KEY     = (1 << 1),
	OPERATOR_ESC_KEY     = (1 << 0),
	OPERATOR_NO_KEY		= 0x00
} OPERRATOR_Key_e;

typedef enum
{
	OPERATOR_STOP_LED  = (1 << 6),
	OPERATOR_RUN_LED  = (1 << 5),
	OPERATOR_DRIVE_LED  = (1 << 4),
	OPERATOR_REV_LED  = (1 << 3),
	OPERATOR_FWD_LED  = (1 << 2),
	OPERATOR_SEQ_LED  = (1 << 1),
	OPERATOR_REF_LED  = (1 << 0)
} OPERRATOR_led_e;

typedef enum
{
	OP_STATUS_MODELEVEL1 = 0x00,
	OP_STATUS_GROUPLEVEL2 = 0x01,
	OP_STATUS_FUNLEVEL3 = 0x02,
	OP_STAUS_CELLLEVEL4 = 0x03,
	OP_STATUS_EDITLEVEL5 = 0x4,
	OP_STATUS_ENTRYLEVEL6 = 0x5

} OPERATOR_Status_e;

typedef enum
{
	OP_MODE_OPERATION = 0x00,
	OP_MODE_INIT = 0x01,
	OP_MODE_PROG = 0x02,
	OP_MODE_AUTOTUNE = 0x03
} OPERATOR_MODE_e;

typedef enum
{
	OP_GROUP_U = 0x00,
	OP_GROUP_A = 0x00,
	OP_GROUP_B = 0x00,
	OP_GROUP_C = 0x01
} OPERATOR_GROUP_e;

typedef enum
{
	OP_ACCESS_MONITOR = 0x00,
	OP_ACCESS_USER    = 0x01,
	OP_ACCESS_QUICK   = 0x02,
	OP_ACCESS_BASIC   = 0x03,
	OP_ACCESS_ADVANCE = 0x04
} OPERATOR_ACCESS_e;
//const char g_ai8DescAccessLevel[][MAX_DESCRIPTION_LENGTH] ={"Monitor Only","User Program", "Quick-start","Basic","Advanced"};

/*typedef enum
{
	OP_GROUP_U = 0x00,
	OP_GROUP_A = 0x01,
	OP_GROUP_B = 0x02,
	OP_HROUP_C = 0x03

} OPERATOR_GROUP_e;*/

typedef struct _OPERATOR_Obj_
{
	OPERRATOR_CommStatus_e OpCommStatus;
	OPERATOR_Status_e	OpStatus;
	DM_Handle		DmHandle;
	uint_least8_t	u8ModeIndex, u8GroupIndex, u8FunIndex, u8CellIndex;
	uint16_t		u16SpecialIndex;



	uint16_t		u16Key;
	uint_least8_t	u8LEDBlink, u8LEDOnOff;
	uint16_t		u16LCDBlink[2];
	int_least8_t 	ai8LCD[2][16];
} OPERATOR_Obj;

extern int32_t 	g_i32EditValue, g_i32EditMaxValue, g_i32EditMinValue, g_i32DeltaValue;

typedef struct _OPERATOR_Obj_ *OPERATOR_Handle;

#define OPERATOR_STARTCODE1		0x00
#define OPERATOR_STARTCODE2A	0x70
#define OPERATOR_STARTCODE2B	0x71

extern OPERATOR_Handle OPERATOR_init(void *pMemory,const size_t numBytes);
extern void OPERATOR_setup(OPERATOR_Handle operatorHandle, DM_Handle dmHandle);
extern void OPERATOR_callback(SCIMessage_Handle sciMessageHandle);

extern void OPERATOR_setLEDBlink(OPERATOR_Handle operatorHandle,uint_least8_t u8BlinkData);
extern void OPERATOR_setLEDOn(OPERATOR_Handle operatorHandle,uint_least8_t u8OnData);
extern void OPERATOR_setLEDOff(OPERATOR_Handle operatorHandle,uint_least8_t u8OffData);
extern void OPERATOR_setLCDBlinkOn(OPERATOR_Handle operatorHandle,uint_least8_t u8LineNo, uint16_t u16OnData);
extern void OPERATOR_setLCDBlinkOff(OPERATOR_Handle operatorHandle,uint_least8_t u8LineNo, uint16_t u16OffData);
extern void OPERATOR_setLCDChar(OPERATOR_Handle operatorHandle, uint_least8_t u8LineNo, uint_least8_t u8PosNo, int_least8_t i8CharValue);
extern void OPERATOR_clearLCDChar(OPERATOR_Handle operatorHandle, uint_least8_t u8LineNo, uint_least8_t u8PosNo);
extern void OPERATOR_setLCDString(OPERATOR_Handle operatorHandle, uint_least8_t u8LineNo, uint_least8_t u8PosNo, char *pi8Data);
extern void OPERATOR_clearLCDLine(OPERATOR_Handle operatorHandle,uint_least8_t u8LineNo);


extern bool OPERATOR_isKey(OPERATOR_Handle operatorHandle, OPERRATOR_Key_e operKey);
extern void OPERATOR_showModeLevel(OPERATOR_Handle operatorHandle);
extern void OPERATOR_showGroupLevel(OPERATOR_Handle operatorHandle);

extern void OPERATOR_run();
extern void OPERATOR_runMenuKey(OPERATOR_Handle operatorHandle);
extern void OPERATOR_runUpKey(OPERATOR_Handle operatorHandle);
extern void OPERATOR_runDownKey(OPERATOR_Handle operatorHandle);

extern void OPERATOR_runStartKey(OPERATOR_Handle operatorHandle);
extern void OPERATOR_runLocalKey(OPERATOR_Handle operatorHandle);
#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _I2C_24LC32_H_ definition




