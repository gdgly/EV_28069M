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
#ifndef _USER_DATA_H_
#define _USER_DATA_H_

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
#include "i2c_24lc32.h"





#ifdef __cplusplus
extern "C" {
#endif

#define DM_OPERATION_RUN_BITS            (1 << 0)
#define DM_OPERATION_FORWARD_BITS		 (1 << 1)
#define DM_OPEARTION_EXTERNAL_FAULT		 (1 << 4)
#define DM_OPEARTION_FAULT_RESET		 (1 << 5)

typedef int32_t (*CallbackValue)();

typedef enum
{
	CTL_FOC_Open = 0,
	CTL_FOC_w_Encoder1 = 1,
	CTL_FOC_w_Encoder2 = 2
} CTL_METHOD_e;

typedef enum
{
	CTL_TORQUE_MODE = 0,
	CTL_SPEED_MODE = 1,
	CTL_POSITION_MODE = 2
} CTL_MODE_e;



typedef enum
{
	DM_TYPE_Int16 = 0x00,
	DM_TYPE_UInt16 = 0x01,
	DM_TYPE_Int8 = 0x02,
	DM_TYPE_UInt8 = 0x03
} DM_TYPE_e;


typedef enum
{
	DM_ATTRIBUTE_Read 	= 0x0001,
	DM_ATTRIBUTE_Write 	= 0x0002,
	DM_ATTRIBUTE_Write_Run = 0x0004,


	DM_ATTRIBUTE_Point0 = 0x0000,
	DM_ATTRIBUTE_Point1 = 0x0010,
	DM_ATTRIBUTE_Point2 = 0x0020,
	DM_ATTRIBUTE_Point3 = 0x0030,
	DM_ATTRIBUTE_Hex2	= 0x0040,
	DM_ATTRIBUTE_Hex4	= 0x0050,
	DM_ATTRIBUTE_Sts    = 0x0060,

	DM_ATTRIBUTE_Int16  = 0x0100,
	DM_ATTRIBUTE_UInt16 = 0x0200,
	DM_ATTRIBUTE_Int8   = 0x0300,
	DM_ATTRIBUTE_UInt8  = 0x0400

	//DM_ATTRIBUTE_Value = 0x1000
} DM_ATTRIBUTE_e;




typedef enum
{
	DM_UNIT_None = 0x0001,
	DM_UNIT_Hz	 = 0x0002,
	DM_UNIT_RPM	 = 0x0003,
	DM_UNIT_Amp	 = 0x0004,
	DM_UNIT_Volt = 0x0005,
	DM_UNIT_Power = 0x0006,
	DM_UNIT_Percent = 0x0007,
	DM_UNIT_Sec	 = 0x0008,
	DM_UNIT_Temp = 0x0009
} DM_UNIT_e;

#define I2C_ADDRESS_INVALID		0xffff
#define MODBUS_ADDRESS_INVALID	0xffff




typedef struct _DM_Cell_
{
	void			*pValue;
	uint16_t		u16EEPromAddr, u16ModbusAdd;
	uint16_t		u16DefValue, u16MaxValue, u16MinValue;
	//void			*pDefValue,*pMaxValue, *pMInValue;
	uint16_t 		u16Attribute, u16Unit;
	CallbackValue	CallbackValue;
} DM_Cell;


typedef struct _DM_FunCell_
{
    const DM_Cell	*pdmCell;
	uint_least8_t	u8Index;
	char			ai8CellName[20];
	const char*		pai8DescArray;
} DM_FunCell;


typedef struct _DM_Function_
{
    const DM_FunCell*	*pdmFunCell;
	uint_least8_t	u8ArraySize;
	char			ai8FunSym[3];
	char			ai8FunName[20];
} DM_Function;


typedef struct _DM_Group_
{
	const DM_Function*	*pdmFunction;
	uint_least8_t	u8ArraySize;
	char			ai8GroupSym[2];
	char			ai8GroupName[20];
} DM_Group;


typedef struct _DM_Mode_
{
    const DM_Group*	*pdmGroup;
	uint_least8_t	u8ArraySize;
	char			ai8ModeName[20];
} DM_Mode;


typedef struct _DM_Obj_
{
    const DM_Mode* *pdmMode;
	EEPROM_Handle   eepromHandle;
	const uint_least8_t	u8ArraySize;
	const char		ai8ObjName[20];
	//void *pdmModbusList;
} DM_Obj;


typedef struct _DM_Obj_ *DM_Handle;

extern uint16_t 	  g_u16InputTerms;
extern uint_least8_t  g_u8OutputTerms;


extern DM_Obj gdmObj;
extern DM_Handle gdmHandle;

extern uint16_t g_u16DelayCnt;


extern uint_least8_t g_u8CtlMode; 	//CTL_TORQUE_MODE = 0, CTL_SPEED_MODE = 1, CTL_POSITION_MODE = 2
extern uint_least8_t g_u8CtlMethod;	//CTL_FOC_Open = 0, CTL_FOC_w_Encoder1 = 1, CTL_FOC_w_Encoder2 = 2



extern uint16_t DATA_min(uint16_t u16Val1, uint16_t u16Val2);
extern uint16_t DATAType_min(DM_TYPE_e dmType, uint16_t u16Val1, uint16_t u16Val2);

extern void Data_SetDelayCnt(uint16_t u16Delay);
extern uint16_t Data_GetDelayCnt();		// internal
extern uint16_t DATA_ConvertToCnt(uint16_t u16Delay);
extern void DATA_Delay(uint16_t u16Delay);
extern uint16_t DATA_MakeWord(uint_least8_t u8HiWord, uint_least8_t u8LoWord );
extern uint_least8_t DATA_HiByte(uint16_t u16Value);
extern uint_least8_t DATA_LoByte(uint16_t u16Value);
extern uint16_t DATA_CalCRC16(uint_least8_t au8Buff[], uint_least8_t u8TotNo);
extern uint16_t DATAType_InRange(DM_TYPE_e dmType, uint16_t u16Value, uint16_t u16DefValue, uint16_t u16MaxValue, uint16_t u16MinValue);

extern void DM_run();
extern DM_Handle DM_init(const void *pMemory,const size_t numBytes);
extern void DM_setup(DM_Handle dmHandle, EEPROM_Handle eepromHandle);


extern const char* DM_getModeName(DM_Handle dmHandle, uint_least8_t u8ModeNo);
extern uint_least8_t DM_getModeSizeNo(DM_Handle dmHandle);
extern uint_least8_t DM_getOperModeSizeNo(DM_Handle dmHandle);
//extern uint_least8_t DM_getInitModeLevel(DM_Handle dmHandle);
extern uint_least8_t DM_getAccessLevel(DM_Handle dmHandle);
extern uint16_t DM_getCellValue(const DM_Cell  *pdmCell);

extern uint_least8_t DM_getGroupSizeNo(DM_Handle dmHandle,uint_least8_t u8ModeIndex);
extern uint_least8_t DM_getFunctionSizeNo(DM_Handle dmHandle,uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex);
extern uint_least8_t DM_getCellSizeNo(DM_Handle dmHandle,uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex, uint_least8_t u8FunctionIndex);
extern uint16_t DM_getTotalFunctionSizeNo(DM_Handle dmHandle);
extern uint16_t DM_getTotalCellSizeNo(DM_Handle dmHandle);
extern void DM_findGroupFunctionIndex(DM_Handle dmHandle, uint16_t u16TotFunIndex,
							          uint_least8_t *pGroupIndex, uint_least8_t *pFunctionIndex);
extern void DM_findGroupFunctionCellIndex(DM_Handle dmHandle, uint16_t u16TotCellIndex,
							       uint_least8_t *pGroupIndex, uint_least8_t *pFunctionIndex, uint_least8_t *pCellIndex);
extern uint16_t DM_DecEditValue(uint16_t u16Value, uint16_t u16IncValue, const DM_Cell* pdmCell);
extern uint16_t DM_IncEditValue(uint16_t u16Value, uint16_t u16IncValue, const DM_Cell* pdmCell);
extern void DM_getToValue(DM_Handle dmHandle,
		 	 	   uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex,
				   uint_least8_t u8FunIndex, uint_least8_t u8CellIndex,
				   uint16_t *pu16Value, uint16_t *pu16IncValue, const DM_Cell* *pdmCell);
extern void DM_setEEPromValue(EEPROM_Handle eepromHandle, const DM_Cell* pdmCell);
extern uint16_t DM_getEEPromValue(EEPROM_Handle eepromHandle, const  DM_Cell* pdmCell);


//extern void DM_getToValueInt32(DM_Handle dmHandle,
//		 	 	 	 	   uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex,
//						   uint_least8_t u8FunIndex, uint_least8_t u8CellIndex,
//						   int32_t *pi32Value, int32_t *pi32MaxValue, int32_t *pi32MinValue, int32_t *pi32IncValue);
extern void DM_setValue(DM_Handle dmHandle,
	 	   	   	   uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex,
	 	   	   	   	uint_least8_t u8FunIndex, uint_least8_t u8CellIndex,
	 	   	   	   	uint16_t u16Value);




extern bool DM_isWrite(DM_Handle dmHandle,
			    		uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex,
			    		uint_least8_t u8FunIndex, uint_least8_t u8CellIndex);
extern bool DM_outEditCell(DM_Handle dmHandle,
				   	   	   uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex,
				   	   	   uint_least8_t u8FunIndex, uint_least8_t u8CellIndex,
				   	   	   uint16_t u16Value,
				   	   	   char *pai8Line1, char *pai8Line2);

extern void DM_outFunCell(DM_Handle dmHandle,
				   uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex,
				   uint_least8_t u8FunIndex, uint_least8_t u8CellIndex,
				   char *pai8Line1, char *pai8Line2);
extern void DM_outGroup(DM_Handle dmHandle,
				   uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex,
				   char *pai8Line1, char *pai8Line2);
extern void DM_outFunction(DM_Handle dmHandle,
				   uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex, uint_least8_t u8FunIndex,
				   char *pai8Line1, char *pai8Line2);

extern int32_t setCallbackFreqRef();
extern int32_t setCallbackOutputTerm();

extern int32_t getCallbackFreqOutHz();
extern int32_t getCallbackFreqOutRpm();
extern int32_t getCallbackCurrentOut();

extern int32_t getCallbackDCBus();
extern int32_t getCallbackVoltageU();
extern int32_t getCallbackVoltageV();
extern int32_t getCallbackVoltageW();
extern int32_t getCallbackCurrentU();
extern int32_t getCallbackCurrentV();
extern int32_t getCallbackCurrentW();
extern int32_t getCallbackInputTerm();
extern int32_t getCallbackExtTemp();
extern int32_t getCallbackExtAd1();
extern int32_t getCallbackExtAd2();
extern int32_t getCallbackExtAd3();
extern int32_t getCallbackExtAd4();
extern int32_t getCallbackExtAd5();
extern int32_t getCallbackExtAd6();




#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _I2C_24LC32_H_ definition




