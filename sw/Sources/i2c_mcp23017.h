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
#ifndef _I2C_MCP23017_
#define _I2C_MCP23017_

//! \file   solutions/instaspin_motion/src/ctrl.h
//! \brief Contains the public interface, object and function definitions for 
//!        various functions related to the CTRL object 
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "sw/modules/types/src/types.h"
#include "sw/drivers/i2c/src/32b/f28x/f2806x/i2c.h"
#include "i2c_message.h"
#include "i2c_24lc32.h"
#include "i2c_lcd.h"
#include "i2c_message.h"
//#include "i2c.h"


//#include "hal_obj.h"


//!
//!
//! \defgroup CTRL CTRL
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  IOEXAND_INPUT_Bit0 = (1 << 0),
  IOEXAND_INPUT_Bit1 = (1 << 1),
  IOEXAND_INPUT_Bit2 = (1 << 2),
  IOEXAND_INPUT_Bit3 = (1 << 3),
  IOEXAND_INPUT_Bit4 = (1 << 4),
  IOEXAND_INPUT_Bit5 = (1 << 5),
  IOEXAND_INPUT_Bit6 = (1 << 6),
  IOEXAND_INPUT_Bit7 = (1 << 7),
  IOEXAND_INPUT_Bit8 = (1 << 8),
  IOEXAND_INPUT_Bit9 = (1 << 9),
  IOEXAND_INPUT_Bit10 = (1 << 10),
  IOEXAND_INPUT_Bit11 = (1 << 11),
  IOEXAND_INPUT_Bit12 = (1 << 12),
  IOEXAND_INPUT_Bit13 = (1 << 13),
  IOEXAND_INPUT_Bit14 = (1 << 14),
  IOEXAND_INPUT_Bit15 = (1 << 15),

  IOEXAND_OUTPUT_Bit0 = (0 << 0),
  IOEXAND_OUTPUT_Bit1 = (0 << 1),
  IOEXAND_OUTPUT_Bit2 = (0 << 2),
  IOEXAND_OUTPUT_Bit3 = (0 << 3),
  IOEXAND_OUTPUT_Bit4 = (0 << 4),
  IOEXAND_OUTPUT_Bit5 = (0 << 5),
  IOEXAND_OUTPUT_Bit6 = (0 << 6),
  IOEXAND_OUTPUT_Bit7 = (0 << 7),
  IOEXAND_OUTPUT_Bit8 = (0 << 8),
  IOEXAND_OUTPUT_Bit9 = (0 << 9),
  IOEXAND_OUTPUT_Bit10 = (0 << 10),
  IOEXAND_OUTPUT_Bit11 = (0 << 11),
  IOEXAND_OUTPUT_Bit12 = (0 << 12),
  IOEXAND_OUTPUT_Bit13 = (0 << 13),
  IOEXAND_OUTPUT_Bit14 = (0 << 14),
  IOEXAND_OUTPUT_Bit15 = (0 << 15)



} IOEXPAND_IO_e;

typedef enum
{
  IOEXAND_PullUp_Bit0 = (1 << 0),
  IOEXAND_PullUp_Bit1 = (1 << 1),
  IOEXAND_PullUp_Bit2 = (1 << 2),
  IOEXAND_PullUp_Bit3 = (1 << 3),
  IOEXAND_PullUp_Bit4 = (1 << 4),
  IOEXAND_PullUp_Bit5 = (1 << 5),
  IOEXAND_PullUp_Bit6 = (1 << 6),
  IOEXAND_PullUp_Bit7 = (1 << 7),
  IOEXAND_PullUp_Bit8 = (1 << 8),
  IOEXAND_PullUp_Bit9 = (1 << 9),
  IOEXAND_PullUp_Bit10 = (1 << 10),
  IOEXAND_PullUp_Bit11 = (1 << 11),
  IOEXAND_PullUp_Bit12 = (1 << 12),
  IOEXAND_PullUp_Bit13 = (1 << 13),
  IOEXAND_PullUp_Bit14 = (1 << 14),
  IOEXAND_PullUp_Bit15 = (1 << 15),

  IOEXAND_PullDisable_Bit0 = (0 << 0),
  IOEXAND_PullDisable_Bit1 = (0 << 1),
  IOEXAND_PullDisable_Bit2 = (0 << 2),
  IOEXAND_PullDisable_Bit3 = (0 << 3),
  IOEXAND_PullDisable_Bit4 = (0 << 4),
  IOEXAND_PullDisable_Bit5 = (0 << 5),
  IOEXAND_PullDisable_Bit6 = (0 << 6),
  IOEXAND_PullDisable_Bit7 = (0 << 7),
  IOEXAND_PullDisable_Bit8 = (0 << 8),
  IOEXAND_PullDisable_Bit9 = (0 << 9),
  IOEXAND_PullDisable_Bit10 = (0 << 10),
  IOEXAND_PullDisable_Bit11 = (0 << 11),
  IOEXAND_PullDisable_Bit12 = (0 << 12),
  IOEXAND_PullDisable_Bit13 = (0 << 13),
  IOEXAND_PullDisable_Bit14 = (0 << 14),
  IOEXAND_PullDisable_Bit15 = (0 << 15)
} IOEXPAND_PULLUP_e;


typedef enum
{
	IOEXPAND_Term1		= (1 << 8),		//Input
	IOEXPAND_Term2		= (1 << 9),
	IOEXPAND_Term3		= (1 << 10),
	IOEXPAND_Term4		= (1 << 11),
	IOEXPAND_Term5		= (1 << 12),
	IOEXPAND_Term6		= (1 << 13),
	IOEXPAND_Term7		= (1 << 14),
	IOEXPAND_Term8		= (1 << 15),

	IOEXPAND_Term25		= (1 << 4),		//Output
	IOEXPAND_Term26		= (1 << 5),
	IOEXPAND_Term10		= (1 << 6),
	IOEXPAND_Term18		= (1 << 7)
} IOEXPAND_TERMINAL_e;


typedef struct _IOEXPAND_Obj_
{
	I2CMessage_Handle 	i2cMessageHandle;
	uint_least8_t* 	pu8OutputTerms;
	uint16_t* 		pu16InputTerms;

} IOEXPAND23017_Obj;


typedef struct _IOEXPAND_Obj_ *IOEXPAND_Handle;



extern IOEXPAND_Handle IOEXPAND_init(void *pMemory,const size_t numBytes);
extern void IOEXPAND_setup(IOEXPAND_Handle ioexpandHandle,  I2CMessage_Handle 	i2cMessageHandle, uint16_t *pu16InputTerms, uint_least8_t *pu8Outputterms);
extern bool IOEXPAND_WriteRegister(IOEXPAND_Handle ioexpandHandle, uint_least8_t u8RegAdd, uint16_t u16Value);
extern bool IOEXPAND_ReadRegister(IOEXPAND_Handle ioexpandHandle,uint_least8_t u8RegAdd, uint16_t *pu16Value);
extern bool IOEXPAND_setHigh(IOEXPAND_Handle ioexpandHandle,const uint_least8_t ioexpanderTerm);
extern bool IOEXPAND_setLow(IOEXPAND_Handle ioexpandHandle,const uint_least8_t ioexpanderTerm);
extern bool IOEXPAND_getInputs(IOEXPAND_Handle ioexpandHandle);
extern void IOEXPAND_run(IOEXPAND_Handle ioexpandHandle);


//extern uint_least8_t IOEXPAND_InputTerminals(IOEXPAND_Handle ioexpandHandle);
//extern uint_least8_t IOEXPAND_Capacity(IOEXPAND_Handle ioexpandHandle);






#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _I2C_MCP23017_ definition




