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
//! \file   solutions/instaspin_motion/src/user.c
//! \brief Contains the function for setting initialization data to the CTRL, HAL, and EST modules
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the includes
#include <stdio.h>
#include <stdint.h>

#include "sw/modules/user/src/32b/user.h"

#include "user_data.h"
#include "sci_operator.h"
#include "main.h"


extern MOTOR_Vars_t gMotorVars;
extern HAL_AdcData_t gAdcData;

extern HAL_Handle 		ghalHandle;
extern IOEXPAND_Handle 	gIoexpandHandle;
//extern IOEXPAND23017_Obj 	gIoexpandObj;

#ifdef FAST_ROM_V1p6
extern CTRL_Obj *gpcontroller_obj;
#else
extern CTRL_Obj ctrl;				//v1p7 format
#endif



/*const uint16_t	u16CONST_MAX = UINT16_MAX;
const uint16_t	u16CONST_MIN = 0;
const int16_t 	i16CONST_MAX = INT16_MAX;
const int16_t   i16CONST_MIN = INT16_MIN;

const uint_least8_t u8CONST_MAX = UINT_LEAST8_MAX;
const uint_least8_t u8CONST_MIN = 0;
const int_least8_t  i8CONST_MAX = INT_LEAST8_MAX;
const int_least8_t  i8CONST_MIN = INT_LEAST8_MIN;
*/


#define MAX_DESCRIPTION_LENGTH		17
uint16_t g_u16Reserved;
uint16_t g_u16DelayCnt;



uint_least8_t g_u8OperSignal;
uint_least8_t g_u8CtlMethod = CTL_FOC_Open;
uint_least8_t g_u8CtlMode = CTL_SPEED_MODE;
int16_t	      g_i16FreqRef = 0, g_i16TorqueRef = 0;
uint16_t      g_u16VoltageOut = 0,  g_u16PowerOut = 0;


uint_least8_t g_u8Language = 0, g_u8AccessLevel = 0xff, g_u8Initial = 0;
uint16_t	  g_u16Password = 0;
uint16_t 	  g_u16InputTerms = 0;
uint_least8_t g_u8OutputTerms = 0;


uint_least8_t g_u8RefSelect = 0, g_u8OperSelection =0, g_u8StoppingMethod = 0;
uint_least8_t g_u8ReverseProhibit =1;

uint16_t	g_u16AccTime1 = 100, g_u16DecTime1=100;


const char g_ai8DescCtrlMode[][MAX_DESCRIPTION_LENGTH] = {"Torque Mode", "Speed Mode", "Position Mode"};
const char g_ai8DescCtrlMethod[][MAX_DESCRIPTION_LENGTH] = {"Sensorless FOC", "FOC w Enc#1", "FOC w Enc#2", "FOC w Hall"};
//const char g_ai8DescLanguage[][MAX_DESCRIPTION_LENGTH] ={"English","Chinese"};
//const char g_ai8DescInitial[][MAX_DESCRIPTION_LENGTH] ={"No Initial","User Initial","2-wire Initial","3-wire Initial"};

const char g_ai8DescAccessLevel[][MAX_DESCRIPTION_LENGTH] ={"Monitor Only","User Program", "Quick-start","Basic","Advanced"};
const char g_ai8DescReferSelect[][MAX_DESCRIPTION_LENGTH]={"Digital Operator", "Terminal","Serial Commun","Optional PCB"};
const char g_ai8DescStopMethod[][MAX_DESCRIPTION_LENGTH]={"Ramp to Stop","Coast to Stop","DC Injection","Coast with Timer"};
const char g_ai8DescEnable[][MAX_DESCRIPTION_LENGTH]={"Enable","Disable"};

const DM_Cell g_dmCellReserved    	= {&g_u16Reserved,  I2C_ADDRESS_INVALID, MODBUS_ADDRESS_INVALID,
										 0, UINT16_MAX,  0,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_UInt16 | DM_ATTRIBUTE_Point0,
										DM_UNIT_None, NULL};
const DM_FunCell g_dmFunCellReserved 	= {&g_dmCellReserved,  1, "Reserved",NULL};



const DM_Cell g_dmCellFreqRef    	= {&g_i16FreqRef,  I2C_ADDRESS_INVALID, 0x0020,
									    0, INT16_MAX, (uint16_t) INT16_MIN,
					 					//0x0000,0x7fff, 0x8000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Write | DM_ATTRIBUTE_Int16 | DM_ATTRIBUTE_Point1 ,
										DM_UNIT_Hz, setCallbackFreqRef};
const DM_Cell g_dmCellFreqOut    	= {NULL,   I2C_ADDRESS_INVALID, 0x0021,
										 0x0000,0x7fff, 0x8000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Int16 | DM_ATTRIBUTE_Point1,
										DM_UNIT_Hz,getCallbackFreqOutHz};
const DM_Cell g_dmCellCurrentOut  	= {NULL, I2C_ADDRESS_INVALID, 0x0022,
										0x0000,0xffff, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Int16 | DM_ATTRIBUTE_Point2,
										DM_UNIT_Amp, getCallbackCurrentOut};
const DM_Cell g_dmCellCtlMethod  	= {&g_u8CtlMethod, 0x0000, 0x0023,
										0x0000,0x0002, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Write | DM_ATTRIBUTE_UInt8 | DM_ATTRIBUTE_Point0,
										DM_UNIT_None, NULL};
const DM_Cell g_dmCellMotorSpd  	= {NULL, I2C_ADDRESS_INVALID, 0x0024,
										0x0000,0x7fff, 0x8000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Int16 | DM_ATTRIBUTE_Point0,
										DM_UNIT_RPM, getCallbackFreqOutRpm};

const DM_Cell g_dmCellVoltageOut  	= {&g_u16VoltageOut, I2C_ADDRESS_INVALID, 0x0025,
										0x0000,0xffff, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_UInt16 | DM_ATTRIBUTE_Point1,
										DM_UNIT_Volt, NULL};
const DM_Cell g_dmCellDCBus  	    = {NULL, I2C_ADDRESS_INVALID, 0x0026 ,
										0x0000,0xffff, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_UInt16| DM_ATTRIBUTE_Point1,
										DM_UNIT_Volt, getCallbackDCBus};

const DM_Cell g_dmCellPowerOut  	= {&g_u16PowerOut, I2C_ADDRESS_INVALID, 0x0027,
										 0x0000,0xffff, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_UInt16| DM_ATTRIBUTE_Point1,
										DM_UNIT_Power, NULL};
const DM_Cell g_dmCellTorqueRef  	= {&g_i16TorqueRef, I2C_ADDRESS_INVALID, 0x0028,
										0x0000,0x7fff, 0x8000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Int16  | DM_ATTRIBUTE_Point1,
										DM_UNIT_Percent, NULL};
const DM_Cell g_dmCellInputTerm  	= {NULL, I2C_ADDRESS_INVALID, 0x0029,
										0x0000,0x00ff, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_UInt8 | DM_ATTRIBUTE_Sts ,
										DM_UNIT_None, getCallbackInputTerm};
const DM_Cell g_dmCellOutputTerm  	= {&g_u8OutputTerms, I2C_ADDRESS_INVALID, 0x002A,
										0x0000,0x00ff, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Write| DM_ATTRIBUTE_UInt8 | DM_ATTRIBUTE_Sts ,
										DM_UNIT_None, NULL};

const DM_Cell g_dmCellVoltageU  	= {NULL, I2C_ADDRESS_INVALID, 0x002B ,
										0x0000,0xffff, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Int16| DM_ATTRIBUTE_Point1,
										DM_UNIT_Volt, getCallbackVoltageU};
const DM_Cell g_dmCellVoltageV  	= {NULL, I2C_ADDRESS_INVALID, 0x002C ,
										0x0000,0xffff, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Int16| DM_ATTRIBUTE_Point1,
										DM_UNIT_Volt, getCallbackVoltageV};
const DM_Cell g_dmCellVoltageW  	= {NULL, I2C_ADDRESS_INVALID, 0x002D ,
										0x0000,0xffff, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Int16| DM_ATTRIBUTE_Point1,
										DM_UNIT_Volt, getCallbackVoltageW};

const DM_Cell g_dmCellCurrentU  	= {NULL, I2C_ADDRESS_INVALID, 0x002E ,
										0x0000,0xffff, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Int16| DM_ATTRIBUTE_Point2,
										DM_UNIT_Amp, getCallbackCurrentU};

const DM_Cell g_dmCellCurrentV  	= {NULL, I2C_ADDRESS_INVALID, 0x002F ,
										0x0000,0xffff, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Int16| DM_ATTRIBUTE_Point2,
										DM_UNIT_Amp, getCallbackCurrentV};

const DM_Cell g_dmCellCurrentW  	= {NULL, I2C_ADDRESS_INVALID, 0x0030 ,
										0x0000,0xffff, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Int16| DM_ATTRIBUTE_Point2,
										DM_UNIT_Amp, getCallbackCurrentW};

const DM_Cell g_dmCellExtTemp	= {NULL, I2C_ADDRESS_INVALID, 0x0031 ,
										0x0000,0xffff, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_UInt16| DM_ATTRIBUTE_Point1,
										DM_UNIT_Temp, getCallbackExtTemp};
const DM_Cell g_dmCellExtAd1	= {NULL, I2C_ADDRESS_INVALID, 0x0032 ,
										0x0000,0xffff, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Int16| DM_ATTRIBUTE_Point2,
										DM_UNIT_Volt, getCallbackExtAd1};
const DM_Cell g_dmCellExtAd2	= {NULL, I2C_ADDRESS_INVALID, 0x0033 ,
										0x0000,0xffff, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Int16| DM_ATTRIBUTE_Point2,
										DM_UNIT_Volt, getCallbackExtAd2};
const DM_Cell g_dmCellExtAd3	= {NULL, I2C_ADDRESS_INVALID, 0x0034 ,
										0x0000,0xffff, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Int16| DM_ATTRIBUTE_Point2,
										DM_UNIT_Volt, getCallbackExtAd3};
const DM_Cell g_dmCellExtAd4	= {NULL, I2C_ADDRESS_INVALID, 0x0035 ,
										0x0000,0xffff, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Int16| DM_ATTRIBUTE_Point2,
										DM_UNIT_Volt, getCallbackExtAd4};
const DM_Cell g_dmCellExtAd5	= {NULL, I2C_ADDRESS_INVALID, 0x0036 ,
										0x0000,0xffff, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Int16| DM_ATTRIBUTE_Point2,
										DM_UNIT_Volt, getCallbackExtAd5};
const DM_Cell g_dmCellExtAd6	= {NULL, I2C_ADDRESS_INVALID, 0x0037 ,
										0x0000,0xffff, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Int16| DM_ATTRIBUTE_Point2,
										DM_UNIT_Volt, getCallbackExtAd6};


const DM_FunCell g_dmFunCellU1_01 			= {&g_dmCellFreqRef, 	1, "Frequency Ref", NULL};
const DM_FunCell g_dmFunCellU1_02 			= {&g_dmCellFreqOut, 	2, "Output Freq", NULL};
const DM_FunCell g_dmFunCellU1_03 			= {&g_dmCellCurrentOut, 3, "Output Current", NULL};
const DM_FunCell g_dmFunCellU1_04 			= {&g_dmCellCtlMethod,  4, "Control Method", &g_ai8DescCtrlMethod[0][0]};
const DM_FunCell g_dmFunCellU1_05 			= {&g_dmCellMotorSpd,   5, "Motor Speed", NULL};
const DM_FunCell g_dmFunCellU1_06 			= {&g_dmCellVoltageOut, 6, "Output Voltage", NULL};
const DM_FunCell g_dmFunCellU1_07 			= {&g_dmCellDCBus, 		7, "DC Bus Voltage", NULL};
const DM_FunCell g_dmFunCellU1_08 			= {&g_dmCellPowerOut, 	8, "Output Power", NULL};
const DM_FunCell g_dmFunCellU1_09 			= {&g_dmCellTorqueRef, 	9, "Torq Reference", NULL};
const DM_FunCell g_dmFunCellU1_10 			= {&g_dmCellInputTerm, 	10, "Input Term Sts", NULL};
const DM_FunCell g_dmFunCellU1_11 			= {&g_dmCellOutputTerm, 11, "Output Term Sts", NULL};

const DM_FunCell g_dmFunCellU1_12 			= {&g_dmCellVoltageU, 	12, "Voltage-U", NULL};
const DM_FunCell g_dmFunCellU1_13 			= {&g_dmCellVoltageV, 	13, "Voltage-V", NULL};
const DM_FunCell g_dmFunCellU1_14 			= {&g_dmCellVoltageW, 	14, "Voltage-W", NULL};
const DM_FunCell g_dmFunCellU1_15 			= {&g_dmCellCurrentU, 	15, "Current-U", NULL};
const DM_FunCell g_dmFunCellU1_16 			= {&g_dmCellCurrentV, 	16, "Current-V", NULL};
const DM_FunCell g_dmFunCellU1_17 			= {&g_dmCellCurrentW, 	17, "Current-W", NULL};
const DM_FunCell g_dmFunCellU1_18 			= {&g_dmCellExtTemp, 	18, "Ext. Temp", NULL};
const DM_FunCell g_dmFunCellU1_19 			= {&g_dmCellExtAd1, 	19, "Ext. AD#1", NULL};
const DM_FunCell g_dmFunCellU1_20 			= {&g_dmCellExtAd2, 	20, "Ext. AD#2", NULL};
const DM_FunCell g_dmFunCellU1_21 			= {&g_dmCellExtAd3, 	21, "Ext. AD#3", NULL};
const DM_FunCell g_dmFunCellU1_22 			= {&g_dmCellExtAd4, 	22, "Ext. AD#4", NULL};
const DM_FunCell g_dmFunCellU1_23 			= {&g_dmCellExtAd5, 	23, "Ext. AD#5", NULL};
const DM_FunCell g_dmFunCellU1_24 			= {&g_dmCellExtAd6, 	24, "Ext. AD#6", NULL};




const DM_FunCell g_dmFunCellU2_01 			= {&g_dmCellReserved, 	1, "Reserved", NULL};
//const DM_FunCell g_dmFunCellU3_01 			= {&g_dmCellReserved, 	1, "Reserved", NULL};


const DM_FunCell* g_dmFunCellArrayU1[] = {&g_dmFunCellU1_01, &g_dmFunCellU1_02, &g_dmFunCellU1_03, &g_dmFunCellU1_04, &g_dmFunCellU1_05,
										  &g_dmFunCellU1_06, &g_dmFunCellU1_07, &g_dmFunCellU1_08, &g_dmFunCellU1_09, &g_dmFunCellU1_10,
										  &g_dmFunCellU1_11, &g_dmFunCellU1_12, &g_dmFunCellU1_13, &g_dmFunCellU1_14, &g_dmFunCellU1_15,
										  &g_dmFunCellU1_16, &g_dmFunCellU1_17, &g_dmFunCellU1_18, &g_dmFunCellU1_19, &g_dmFunCellU1_20,
										  &g_dmFunCellU1_21, &g_dmFunCellU1_22, &g_dmFunCellU1_23, &g_dmFunCellU1_24};
const DM_FunCell* g_dmFunCellArrayU2[] = {&g_dmFunCellU2_01};
//const DM_FunCell* g_dmFunCellArrayU3[] = {&g_dmFunCellU3_01};

const DM_Function g_dmFunU1 ={&g_dmFunCellArrayU1[0], 24, "U1","Monitor"};	//21
const DM_Function g_dmFunU2 ={&g_dmFunCellArrayU2[0], 1, "U2","Fault Trace"};
//const DM_Function g_dmFunU3 ={&g_dmFunCellArrayU3[0], 1, "U3","Fault History"};

const DM_Function* g_dmFunUArray[] = {&g_dmFunU1, &g_dmFunU2};	//, &g_dmFunU3};

//const DM_Cell g_dmCellLanguage  	= {&g_u8Language, I2C_ADDRESS_INVALID, 0x0100,
//										0x0000,0x0001, 0x0000,
//										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Write | DM_ATTRIBUTE_UInt8 | DM_ATTRIBUTE_Point0,
//										DM_UNIT_None, NULL};

//const DM_Cell g_dmCellInitial  	= {&g_u8Initial, I2C_ADDRESS_INVALID, 0x0103,
//										0x0000,0x0004, 0x0000,
//										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Write | DM_ATTRIBUTE_UInt8 | DM_ATTRIBUTE_Point0,
//										DM_UNIT_None, NULL};
//const DM_Cell g_dmCellPassword  	= {&g_u16Password, I2C_ADDRESS_INVALID, 0x0104,
//										0x0000,9999, 0x0000,
//										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Write | DM_ATTRIBUTE_UInt8 | DM_ATTRIBUTE_Point0,
//										DM_UNIT_None, NULL};

const DM_Cell g_dmCellCtlMode  	= {&g_u8CtlMode, 0x0001, 0x0100,
										0x0001,0x0002, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Write | DM_ATTRIBUTE_UInt8 | DM_ATTRIBUTE_Point0,
										DM_UNIT_None, NULL};
const DM_Cell g_dmCellReferenceSel  = {&g_u8RefSelect, 0x0002, 0x0101,
										0x0000,0x0003, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Write | DM_ATTRIBUTE_UInt8 | DM_ATTRIBUTE_Point0,
										DM_UNIT_None, NULL};
const DM_Cell g_dmCellOperationSel  = {&g_u8OperSelection, 0x0003, 0x0102,
										0x0000,0x0003, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Write | DM_ATTRIBUTE_UInt8 | DM_ATTRIBUTE_Point0,
										DM_UNIT_None, NULL};

const DM_Cell g_dmCellAccessLvl  = {&g_u8AccessLevel, 0x0004, 0x0103,
										0x0004,0x0004, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Write | DM_ATTRIBUTE_UInt8 | DM_ATTRIBUTE_Point0,
										DM_UNIT_None, NULL};

//const DM_FunCell g_dmFunCellA1_01 	= {&g_dmCellLanguage,  1, "Select Language", &g_ai8DescLanguage[0][0]};

const DM_FunCell g_dmFunCellA1_01   = {&g_dmCellCtlMode, 1, "Control Mode", &g_ai8DescCtrlMode[0][0]};
const DM_FunCell g_dmFunCellA1_02 	= {&g_dmCellCtlMethod,  2, "Control Method", &g_ai8DescCtrlMethod[0][0]};
const DM_FunCell g_dmFunCellA1_03 	= {&g_dmCellReferenceSel,  3, "Reference Sel",&g_ai8DescReferSelect[0][0]};
const DM_FunCell g_dmFunCellA1_04 	= {&g_dmCellOperationSel,  4, "Operation Sel",&g_ai8DescReferSelect[0][0]};
const DM_FunCell g_dmFunCellA1_05 	= {&g_dmCellAccessLvl,  5, "Access Level", &g_ai8DescAccessLevel[0][0]};
//const DM_FunCell g_dmFunCellA1_04 	= {&g_dmCellInitial,  4, "Initial Param", &g_ai8DescInitial[0][0]};
//const DM_FunCell g_dmFunCellA1_05 	= {&g_dmCellPassword,  5, "Password", NULL};





const DM_FunCell* g_dmFunCellArrayA1[] = {&g_dmFunCellA1_01, &g_dmFunCellA1_02, &g_dmFunCellA1_03, &g_dmFunCellA1_04, &g_dmFunCellA1_05};
//const DM_FunCell* g_dmFunCellArrayA2[] = {&g_dmFunCellReserved};
const DM_Function g_dmFunA1 ={&g_dmFunCellArrayA1[0], 5, "A1","Initial"};
//const DM_Function g_dmFunA2 ={&g_dmFunCellArrayA2[0], 1, "A2","User Consants"};
const DM_Function* g_dmFunAArray[] = {&g_dmFunA1};	//, &g_dmFunA2};


/*const DM_Cell g_dmCellReferenceSel  = {&g_u8RefSelect, I2C_ADDRESS_INVALID, 0x0180,
										0x0000,0x0003, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Write | DM_ATTRIBUTE_UInt8 | DM_ATTRIBUTE_Point0,
										DM_UNIT_None, NULL};
const DM_Cell g_dmCellOperationSel  = {&g_u8OperSelection, I2C_ADDRESS_INVALID, 0x0181,
										0x0000,0x0003, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Write | DM_ATTRIBUTE_UInt8 | DM_ATTRIBUTE_Point0,
										DM_UNIT_None, NULL};*/
const DM_Cell g_dmCellStopMethod  = {&g_u8StoppingMethod, I2C_ADDRESS_INVALID, 0x0182,
										0x0000,0x0003, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Write | DM_ATTRIBUTE_UInt8 | DM_ATTRIBUTE_Point0,
										DM_UNIT_None, NULL};
const DM_Cell g_dmCellReverseProhibit  = {&g_u8ReverseProhibit, I2C_ADDRESS_INVALID, 0x0183,
										0x0000,0x0001, 0x0000,
										DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Write | DM_ATTRIBUTE_UInt8 | DM_ATTRIBUTE_Point0,
										DM_UNIT_None, NULL};


//const DM_FunCell g_dmFunCellb1_01 	= {&g_dmCellReferenceSel,  1, "Reference Sel",&g_ai8DescReferSelect[0][0]};
//const DM_FunCell g_dmFunCellb1_02 	= {&g_dmCellOperationSel,  2, "Operation Sel",&g_ai8DescReferSelect[0][0]};
const DM_FunCell g_dmFunCellb1_03 	= {&g_dmCellStopMethod,  3, "Stopping Sel",&g_ai8DescStopMethod[0][0]};
const DM_FunCell g_dmFunCellb1_04 	= {&g_dmCellReverseProhibit,  4, "Reverse Oper",&g_ai8DescEnable[0][0]};


const DM_FunCell* g_dmFunCellArrayb1[] = {&g_dmFunCellb1_03, &g_dmFunCellb1_04};
const DM_FunCell* g_dmFunCellArrayb2[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayb3[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayb4[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayb5[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayb6[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayb7[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayb8[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayb9[] = {&g_dmFunCellReserved};


const DM_Function g_dmFunb1 ={&g_dmFunCellArrayb1[0], 2, "b1","Sequence"};
const DM_Function g_dmFunb2 ={&g_dmFunCellArrayb2[0], 1, "b2","DC Braking"};
const DM_Function g_dmFunb3 ={&g_dmFunCellArrayb3[0], 1, "b3","Speed Search"};
const DM_Function g_dmFunb4 ={&g_dmFunCellArrayb4[0], 1, "b4","Delay Timers"};
const DM_Function g_dmFunb5 ={&g_dmFunCellArrayb5[0], 1, "b5","PID Control"};
const DM_Function g_dmFunb6 ={&g_dmFunCellArrayb6[0], 1, "b6","Reference Dwell"};
const DM_Function g_dmFunb7 ={&g_dmFunCellArrayb7[0], 1, "b7","Droop Control"};
const DM_Function g_dmFunb8 ={&g_dmFunCellArrayb8[0], 1, "b8","Energy Saving"};
const DM_Function g_dmFunb9 ={&g_dmFunCellArrayb9[0], 1, "b9","Zero Servo"};
const DM_Function* g_dmFunbArray[] = {&g_dmFunb1, &g_dmFunb2, &g_dmFunb3, &g_dmFunb4, &g_dmFunb5, &g_dmFunb6,
									  &g_dmFunb7, &g_dmFunb8, &g_dmFunb9  };

const DM_Cell g_dmCellAccTime1  = {&g_u16AccTime1, I2C_ADDRESS_INVALID, 0x0200,
									100,60000, 0x0000,
									DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Write | DM_ATTRIBUTE_UInt16 | DM_ATTRIBUTE_Point1,
									DM_UNIT_Sec, NULL};
const DM_Cell g_dmCellDecTime1  = {&g_u16DecTime1, I2C_ADDRESS_INVALID, 0x0201,
									100,60000, 0x0000,
									DM_ATTRIBUTE_Read | DM_ATTRIBUTE_Write | DM_ATTRIBUTE_UInt16 | DM_ATTRIBUTE_Point1,
									DM_UNIT_Sec, NULL};
const DM_FunCell g_dmFunCellC1_01 	= {&g_dmCellAccTime1,  1, "Accel Time 1",NULL};
const DM_FunCell g_dmFunCellC1_02 	= {&g_dmCellDecTime1,  2, "Decel Time 1",NULL};
const DM_FunCell* g_dmFunCellArrayC1[] = {&g_dmFunCellC1_01, &g_dmFunCellC1_02};
const DM_FunCell* g_dmFunCellArrayC2[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayC3[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayC4[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayC5[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayC6[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayC7[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayC8[] = {&g_dmFunCellReserved};


const DM_Function g_dmFunC1 ={&g_dmFunCellArrayC1[0], 2, "C1","Accel/Decel"};
const DM_Function g_dmFunC2 ={&g_dmFunCellArrayC2[0], 1, "C2","S-Curve Acc/Dec"};
const DM_Function g_dmFunC3 ={&g_dmFunCellArrayC3[0], 1, "C3","Motor-Slip Comp"};
const DM_Function g_dmFunC4 ={&g_dmFunCellArrayC4[0], 1, "C4","Torque Comp"};
const DM_Function g_dmFunC5 ={&g_dmFunCellArrayC5[0], 1, "C5","ASR Tuning"};
const DM_Function g_dmFunC6 ={&g_dmFunCellArrayC6[0], 1, "C6","Carrier Freq"};
const DM_Function g_dmFunC7 ={&g_dmFunCellArrayC7[0], 1, "C7","Hunting Prev"};
const DM_Function g_dmFunC8 ={&g_dmFunCellArrayC8[0], 1, "C8","Factory Tuning"};
const DM_Function* g_dmFunCArray[] = {&g_dmFunC1, &g_dmFunC2, &g_dmFunC3, &g_dmFunC4, &g_dmFunC5, &g_dmFunC6,
									  &g_dmFunC7, &g_dmFunC8 };

/*
const DM_FunCell* g_dmFunCellArrayd1[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayd2[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayd3[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayd4[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayd5[] = {&g_dmFunCellReserved};

const DM_Function g_dmFund1 ={&g_dmFunCellArrayd1[0], 1, "d1","Preset Reference"};
const DM_Function g_dmFund2 ={&g_dmFunCellArrayd2[0], 1, "d2","Reference Limits"};
const DM_Function g_dmFund3 ={&g_dmFunCellArrayd3[0], 1, "d3","Jump Frequencies"};
const DM_Function g_dmFund4 ={&g_dmFunCellArrayd4[0], 1, "d4","Sequence"};
const DM_Function g_dmFund5 ={&g_dmFunCellArrayd5[0], 1, "d5","Torque Control"};
const DM_Function* g_dmFundArray[] = {&g_dmFund1, &g_dmFund2, &g_dmFund3, &g_dmFund4, &g_dmFund5 };

const DM_FunCell* g_dmFunCellArrayE1[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayE2[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayE3[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayE4[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayE5[] = {&g_dmFunCellReserved};

const DM_Function g_dmFunE1 ={&g_dmFunCellArrayE1[0], 1, "E1","V/f Pattern"};
const DM_Function g_dmFunE2 ={&g_dmFunCellArrayE2[0], 1, "E2","Motor Setup"};
const DM_Function g_dmFunE3 ={&g_dmFunCellArrayE3[0], 1, "E3","Motor 2 Ctl Meth"};
const DM_Function g_dmFunE4 ={&g_dmFunCellArrayE4[0], 1, "E4","V/F pattern 2"};
const DM_Function g_dmFunE5 ={&g_dmFunCellArrayE5[0], 1, "E5","Motor 2 Setup"};
const DM_Function* g_dmFunEArray[] = {&g_dmFunE1, &g_dmFunE2, &g_dmFunE3, &g_dmFunE4, &g_dmFunE5 };

const DM_FunCell* g_dmFunCellArrayF1[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayF2[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayF3[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayF4[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayF5[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayF6[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayF7[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayF8[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayF9[] = {&g_dmFunCellReserved};

const DM_Function g_dmFunF1 ={&g_dmFunCellArrayF1[0], 1, "F1","PG Option Setup"};
const DM_Function g_dmFunF2 ={&g_dmFunCellArrayF2[0], 1, "F2","AI-14 Setup"};
const DM_Function g_dmFunF3 ={&g_dmFunCellArrayF3[0], 1, "F3","DI-08, 16 Setup"};
const DM_Function g_dmFunF4 ={&g_dmFunCellArrayF4[0], 1, "F4","AO-08, 12 Setup"};
const DM_Function g_dmFunF5 ={&g_dmFunCellArrayF5[0], 1, "F5","DO-02C"};
const DM_Function g_dmFunF6 ={&g_dmFunCellArrayF6[0], 1, "F6","DO-08"};
const DM_Function g_dmFunF7 ={&g_dmFunCellArrayF7[0], 1, "F7","PO-36F Setup"};
const DM_Function g_dmFunF8 ={&g_dmFunCellArrayF8[0], 1, "F8","SI-F/G"};
const DM_Function g_dmFunF9 ={&g_dmFunCellArrayF9[0], 1, "F9","DDS/SI-B"};
const DM_Function* g_dmFunFArray[] = {&g_dmFunF1, &g_dmFunF2, &g_dmFunF3, &g_dmFunF4, &g_dmFunF5,
									  &g_dmFunF6, &g_dmFunF7, &g_dmFunF8, &g_dmFunF9 };

const DM_FunCell* g_dmFunCellArrayH1[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayH2[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayH3[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayH4[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayH5[] = {&g_dmFunCellReserved};

const DM_Function g_dmFunH1 ={&g_dmFunCellArrayH1[0], 1, "H1","Digital Inputs"};
const DM_Function g_dmFunH2 ={&g_dmFunCellArrayH2[0], 1, "H2","Digital Outputs"};
const DM_Function g_dmFunH3 ={&g_dmFunCellArrayH3[0], 1, "H3","Analog Inputs"};
const DM_Function g_dmFunH4 ={&g_dmFunCellArrayH4[0], 1, "H4","Analog Outputs"};
const DM_Function g_dmFunH5 ={&g_dmFunCellArrayH5[0], 1, "H5","Serial Com Setup"};
const DM_Function* g_dmFunHArray[] = {&g_dmFunH1, &g_dmFunH2, &g_dmFunH3, &g_dmFunH4, &g_dmFunH5};

const DM_FunCell* g_dmFunCellArrayL1[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayL2[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayL3[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayL4[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayL5[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayL6[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayL7[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayL8[] = {&g_dmFunCellReserved};

const DM_Function g_dmFunL1 ={&g_dmFunCellArrayL1[0], 1, "L1","Motor Overload"};
const DM_Function g_dmFunL2 ={&g_dmFunCellArrayL2[0], 1, "L2","PwrLoss Ridethru"};
const DM_Function g_dmFunL3 ={&g_dmFunCellArrayL3[0], 1, "L3","Stall Prevention"};
const DM_Function g_dmFunL4 ={&g_dmFunCellArrayL4[0], 1, "L4","Ref Detection"};
const DM_Function g_dmFunL5 ={&g_dmFunCellArrayL5[0], 1, "L5","Fault Restart"};
const DM_Function g_dmFunL6 ={&g_dmFunCellArrayL6[0], 1, "L6","Torque Detection"};
const DM_Function g_dmFunL7 ={&g_dmFunCellArrayL7[0], 1, "L7","Torque Limit"};
const DM_Function g_dmFunL8 ={&g_dmFunCellArrayL8[0], 1, "L8","Hdwe Protection"};
const DM_Function* g_dmFunLArray[] = {&g_dmFunL1, &g_dmFunL2, &g_dmFunL3, &g_dmFunL4, &g_dmFunL5,
									  &g_dmFunL6, &g_dmFunL7, &g_dmFunL8};

const DM_FunCell* g_dmFunCellArrayo1[] = {&g_dmFunCellReserved};
const DM_FunCell* g_dmFunCellArrayo2[] = {&g_dmFunCellReserved};

const DM_Function g_dmFuno1 ={&g_dmFunCellArrayo1[0], 1, "o1","Monitor Select"};
const DM_Function g_dmFuno2 ={&g_dmFunCellArrayo2[0], 1, "o2","Key Selections"};
const DM_Function* g_dmFunoArray[] = {&g_dmFuno1, &g_dmFuno2};
*/

const DM_Group		g_dmGroupU 	={&g_dmFunUArray[0], 2, "U", "Monitor"};
const DM_Group		g_dmGroupA 	={&g_dmFunAArray[0], 1, "A", "Initialize"};
const DM_Group		g_dmGroupb 	={&g_dmFunbArray[0], 9, "b","Application"};
const DM_Group		g_dmGroupC 	={&g_dmFunCArray[0], 8, "C","Tuning"};
//const DM_Group		g_dmGroupd 	={&g_dmFundArray[0], 5, "d","Reference"};
//const DM_Group		g_dmGroupE 	={&g_dmFunEArray[0], 5, "E","Motor"};
//const DM_Group		g_dmGroupF 	={&g_dmFunFArray[0], 9, "F","Option"};
//const DM_Group		g_dmGroupH 	={&g_dmFunHArray[0], 5, "H","Terminal"};
//const DM_Group		g_dmGroupL 	={&g_dmFunLArray[0], 8, "L","Protection"};
//const DM_Group		g_dmGroupo 	={&g_dmFunoArray[0], 2, "o","Operator"};

const DM_Group* 	g_dmGroupOperArray[] = {&g_dmGroupU};
const DM_Group* 	g_dmGroupInitArray[] = {&g_dmGroupA};
const DM_Group* 	g_dmGroupProgArray[] = {&g_dmGroupb, &g_dmGroupC};	//, &g_dmGroupd,
									    //	&g_dmGroupE, &g_dmGroupF, &g_dmGroupH, &g_dmGroupL,
									    //	&g_dmGroupo};

const DM_Mode		g_dmModeOperation ={&g_dmGroupOperArray[0], 1, "Operation"};
const DM_Mode 		g_dmModeInitialize = {&g_dmGroupInitArray[0],1,  "Initialize"};
const DM_Mode   	g_dmModeProgram = {&g_dmGroupProgArray[0], 2, "Programming"};
const DM_Mode   	g_dmModeAutotuning = {NULL,0, "Autotuning"};

const DM_Mode*  	g_dmModeArray[] = {&g_dmModeOperation, &g_dmModeInitialize, &g_dmModeProgram, &g_dmModeAutotuning};

DM_Obj gdmObj = {&g_dmModeArray[0], NULL, 4, "Rich"};
DM_Handle gdmHandle;

//------------------------------------------------------------------------
//
//------------------------------------------------------------------------

uint16_t DATA_min(uint16_t u16Val1, uint16_t u16Val2)
{
	return (u16Val1 < u16Val2) ? u16Val1 : u16Val2;

}

uint16_t DATAType_min(DM_TYPE_e dmType, uint16_t u16Val1, uint16_t u16Val2)
{

	switch (dmType)
	{
		case DM_TYPE_Int16:
		case DM_TYPE_Int8:
		{
			int16_t i16Temp1, i16Temp2;
			i16Temp1 = u16Val1;
			i16Temp2 = u16Val2;
			return (i16Temp1 < i16Temp2) ? u16Val1 : u16Val2;
		}
		case DM_TYPE_UInt16:
		{
			uint16_t u16Temp1, u16Temp2;
			u16Temp1 = u16Val1;
			u16Temp2 = u16Val2;
			return (u16Temp1 < u16Temp2) ? u16Val1 : u16Val2;
		}
	}

	return (u16Val1 < u16Val2) ? u16Val1 : u16Val2;

}


void Data_SetDelayCnt(uint16_t u16Delay)
{
	g_u16DelayCnt =  DATA_ConvertToCnt(u16Delay);
}

uint16_t Data_GetDelayCnt()		// internal
{
	return g_u16DelayCnt;
}

uint16_t DATA_ConvertToCnt(uint16_t u16Delay)
{
	return u16Delay/((uint16_t) USER_TIMER_PERIOD_msec);
}

void DATA_Delay(uint16_t u16Delay)	//ms
{
	uint16_t u16Value = 0;
	Data_SetDelayCnt(u16Delay);
	for (;;)
	{
		u16Value = Data_GetDelayCnt();
		if (u16Value == 0) return;

	}

}

uint16_t DATA_MakeWord(uint_least8_t u8HiWord, uint_least8_t u8LoWord )
{
	return ((uint16_t)(u8HiWord << 8) ) | ((uint16_t) u8LoWord);
}


uint_least8_t DATA_HiByte(uint16_t u16Value)
{
	return (uint_least8_t) (u16Value >> 8);
}

uint_least8_t DATA_LoByte(uint16_t u16Value)
{
	return (uint_least8_t) (u16Value & 0x00ff);
}

uint16_t DATAType_InRange(DM_TYPE_e dmType, uint16_t u16Value, uint16_t u16DefValue, uint16_t u16MaxValue, uint16_t u16MinValue)
{
	switch (dmType)
	{
		case DM_TYPE_Int16:
		{
			int16_t i16Value, i16MaxValue, i16MinValue;
			i16Value = u16Value;
			i16MaxValue = u16MaxValue;
			i16MinValue = u16MinValue;
			return  ((i16Value >= i16MinValue) && (i16Value <= i16MaxValue)) ?
					u16Value : u16DefValue;
		}

		case DM_TYPE_UInt16:
			return ((u16Value >= u16MinValue) && (u16Value <= u16MaxValue)) ?
					u16Value : u16DefValue;
		case DM_TYPE_Int8:
		{
			int_least8_t i8Value, i8MaxValue, i8MinValue;
			i8Value = u16Value;
			i8MaxValue = u16MaxValue;
			i8MinValue = u16MinValue;
			return  ((i8Value >= i8MinValue) && (i8Value <= i8MaxValue)) ?
					  u16Value : u16DefValue;
		}

		case DM_TYPE_UInt8:
		{
			uint_least8_t u8Value, u8MaxValue, u8MinValue;
			u8Value = u16Value;
			u8MaxValue = u16MaxValue;
			u8MinValue = u16MinValue;
			return  ((u8Value >= u8MinValue) && (u8Value <= u8MaxValue)) ?
					  u16Value : u16DefValue;
		}

	}
	return u16Value;
}


uint16_t DATA_CalCRC16(uint_least8_t au8Buff[], uint_least8_t u8TotNo)
{
	uint16_t u16Crc16 = 0xffff;
	uint_least8_t	i, j;
    for (i = 0; i < u8TotNo; i++)
    {
		u16Crc16 ^=  (au8Buff[i] & 0x00ffu);
        for (j = 0; j < 8; j++)
        {
			if ((u16Crc16 & 0x0001u) != 0)
            {
                u16Crc16 >>= 1;
                u16Crc16 ^= 0xa001u;
            }
            else
				u16Crc16 >>= 1;
        }
    }
    return u16Crc16;
}
//------------------------------------------------------------------------
//


//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
void DM_run()
{
	if (g_u16DelayCnt)
		g_u16DelayCnt--;
}
// end of file


DM_Handle DM_init(const void *pMemory,const size_t numBytes)
{
	DM_Handle dmHandle;

	if (numBytes < sizeof(DM_Obj))
		return((DM_Handle)NULL);

	// assign the handle
	dmHandle = (DM_Handle)pMemory;
	return(dmHandle);
}

DM_TYPE_e getDMType(const DM_Cell *pDMCell)
{
	uint16_t u16Attribute = pDMCell->u16Attribute & 0x0f00;
	switch (u16Attribute)
	{
		case DM_ATTRIBUTE_Int16:
			return DM_TYPE_Int16;
		case DM_ATTRIBUTE_UInt16:
			return DM_TYPE_UInt16;
		case DM_ATTRIBUTE_Int8:
			return DM_TYPE_Int8;
		default:
			return DM_TYPE_UInt8;
	}


}

void DM_setup(DM_Handle dmHandle, EEPROM_Handle eepromHandle)
{
	DM_Obj *pdmObj = (DM_Obj *) dmHandle;
	DM_Mode **pdmModeArray = (DM_Mode **) pdmObj->pdmMode;
	pdmObj->eepromHandle = eepromHandle;

	uint_least8_t i1;
	for (i1=0; i1<pdmObj->u8ArraySize; i1++)
	{
		DM_Mode *pdmMode= pdmModeArray[i1];
		DM_Group **pdmGroupArray = (DM_Group **) pdmMode->pdmGroup;
		uint_least8_t i2;
		for (i2=0; i2<pdmMode->u8ArraySize; i2++)
		{
			DM_Group *pdmGroup = pdmGroupArray[i2];
			DM_Function **pdmFunctionArray = (DM_Function **) pdmGroup->pdmFunction;
			uint_least8_t i3;
			if (pdmGroup == NULL) continue;
			for (i3=0; i3<pdmGroup->u8ArraySize; i3++)
			{
				DM_Function *pdmFunction = pdmFunctionArray[i3];
				DM_FunCell **pdmFunCellArray = (DM_FunCell **) pdmFunction->pdmFunCell;
				uint_least8_t i4;
				if (pdmFunction == NULL) continue;
				for (i4=0; i4<pdmFunction->u8ArraySize; i4++)
				{
					DM_FunCell *pdmFunCell = pdmFunCellArray[i4];

					//if (pdmFunCell == NULL) continue;
					const DM_Cell *pdmCell = (DM_Cell *) pdmFunCell->pdmCell;
					DM_TYPE_e  dmType = getDMType(pdmCell);
					uint16_t u16Value;

					if ( pdmCell->u16EEPromAddr == I2C_ADDRESS_INVALID)
						u16Value = *((uint16_t *) pdmCell->pValue);
					else
						u16Value = DM_getEEPromValue(eepromHandle, pdmCell);
						//EEPROM_Read(eepromHandle,pdmCell->u16EEPromAddr, &u16Value);

					*((uint16_t *)pdmCell->pValue) = DATAType_InRange(dmType, u16Value, pdmCell->u16DefValue,
																       pdmCell->u16MaxValue, pdmCell->u16MinValue);
				}
			}
		}

	}
}


const char* DM_getModeName(DM_Handle dmHandle, uint_least8_t u8ModeNo)
{
	DM_Obj *pdmObj = (DM_Obj *) dmHandle;

	DM_Mode **pdmModeArray = (DM_Mode **) pdmObj->pdmMode;
	DM_Mode *pdmMode = pdmModeArray[u8ModeNo];
	return pdmMode->ai8ModeName;

}

uint_least8_t DM_getModeSizeNo(DM_Handle dmHandle)
{
	DM_Obj *pdmObj = (DM_Obj *) dmHandle;
	if (g_u8AccessLevel == OP_ACCESS_MONITOR)
		return 2;	// For only operation & initial modes
	return pdmObj->u8ArraySize;
}

uint_least8_t DM_getOperModeSizeNo(DM_Handle dmHandle)
{
	return 24;
}

uint_least8_t DM_getAccessLevel(DM_Handle dmHandle)
{
	return g_u8AccessLevel;
}


DM_Mode *DM_getMode(DM_Handle dmHandle, uint_least8_t u8ModeIndex)
{
	DM_Obj *pdmObj = (DM_Obj *) dmHandle;
	DM_Mode **pdmModeArray = (DM_Mode **) pdmObj->pdmMode;
	return  pdmModeArray[u8ModeIndex];
}

DM_Group *DM_getGroup(DM_Handle dmHandle,
						uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex)
{
	DM_Mode *pdmMode= DM_getMode(dmHandle, u8ModeIndex);
	DM_Group **pdmGroupArray = (DM_Group **) pdmMode->pdmGroup;
	return pdmGroupArray[u8GroupIndex];
}

DM_Function *DM_getFunction(DM_Handle dmHandle,
							  uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex, uint_least8_t u8FunctionIndex)
{
	DM_Group *pdmGroup =DM_getGroup(dmHandle, u8ModeIndex, u8GroupIndex);
	DM_Function **pdmFunctionArray = (DM_Function **) pdmGroup->pdmFunction;
	return pdmFunctionArray[u8FunctionIndex];
}

DM_FunCell *DM_getFunCell(DM_Handle dmHandle,
						  uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex, uint_least8_t u8FunctionIndex, uint_least8_t u8CellIndex)
{

	DM_Function *pdmFunction = DM_getFunction(dmHandle, u8ModeIndex, u8GroupIndex, u8FunctionIndex);
	DM_FunCell*	*pdmFunCellArray= (DM_FunCell **) pdmFunction->pdmFunCell;
	return pdmFunCellArray[u8CellIndex];
}


uint_least8_t DM_getInitModeLevel(DM_Handle dmHandle)
{

	DM_Function *pdmFunction = DM_getFunction(dmHandle,OP_MODE_INIT,0,0);
	return pdmFunction->u8ArraySize;
}

uint_least8_t DM_getGroupSizeNo(DM_Handle dmHandle,uint_least8_t u8ModeIndex)
{
	DM_Mode *pdmMode = DM_getMode(dmHandle,u8ModeIndex);
	return pdmMode->u8ArraySize;

}

uint_least8_t DM_getFunctionSizeNo(DM_Handle dmHandle,uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex)
{

	DM_Group *pdmGroup = DM_getGroup(dmHandle,u8ModeIndex, u8GroupIndex);
	return pdmGroup->u8ArraySize;
}



uint_least8_t DM_getCellSizeNo(DM_Handle dmHandle,uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex, uint_least8_t u8FunctionIndex)
{

	DM_Function *pdmFunction = DM_getFunction(dmHandle,u8ModeIndex, u8GroupIndex, u8FunctionIndex);
	return pdmFunction->u8ArraySize;

}

uint16_t DM_getTotalFunctionSizeNo(DM_Handle dmHandle)
{
	DM_Mode *pdmMode = DM_getMode(dmHandle,OP_MODE_PROG);
	DM_Group **pdmGroupArray = (DM_Group **) pdmMode->pdmGroup;
	uint16_t u16TotNo = 0;
	uint_least8_t i2;
	for (i2=0; i2<pdmMode->u8ArraySize; i2++)
	{
		DM_Group *pdmGroup = pdmGroupArray[i2];
		u16TotNo += pdmGroup->u8ArraySize;
	}
	return u16TotNo;
}



uint16_t DM_getTotalCellSizeNo(DM_Handle dmHandle)
{
	DM_Mode *pdmMode = DM_getMode(dmHandle,OP_MODE_PROG);
	DM_Group **pdmGroupArray = (DM_Group **) pdmMode->pdmGroup;
	uint16_t u16TotNo = 0;
	uint_least8_t i2;
	for (i2=0; i2<pdmMode->u8ArraySize; i2++)
	{
		DM_Group *pdmGroup = pdmGroupArray[i2];
		DM_Function **pdmFunctionArray = (DM_Function **) pdmGroup->pdmFunction;
		uint_least8_t i3;
		//if (pdmGroup == NULL) continue;
		for (i3=0; i3<pdmGroup->u8ArraySize; i3++)
		{
			DM_Function *pdmFunction = pdmFunctionArray[i3];
			u16TotNo += pdmFunction->u8ArraySize;
		}
	}
	return u16TotNo;
}

void DM_findGroupFunctionIndex(DM_Handle dmHandle, uint16_t u16TotFunIndex,
							    uint_least8_t *pGroupIndex, uint_least8_t *pFunctionIndex)
{
	DM_Mode *pdmMode = DM_getMode(dmHandle,OP_MODE_PROG);
	DM_Group **pdmGroupArray = (DM_Group **) pdmMode->pdmGroup;

	uint_least8_t i2;
	*pFunctionIndex = 0;
	for (i2=0; i2<pdmMode->u8ArraySize; i2++)
	{
		DM_Group *pdmGroup = pdmGroupArray[i2];
		uint_least8_t u8ArraySize = pdmGroup->u8ArraySize;
		*pGroupIndex = i2;
		if (u16TotFunIndex < u8ArraySize)
		{
			*pFunctionIndex = (uint_least8_t)u16TotFunIndex;
			return;
		}
		u16TotFunIndex -= u8ArraySize;
	}
}

void DM_findGroupFunctionCellIndex(DM_Handle dmHandle, uint16_t u16TotCellIndex,
							       uint_least8_t *pGroupIndex, uint_least8_t *pFunctionIndex, uint_least8_t *pCellIndex
							       )
{
	DM_Mode *pdmMode = DM_getMode(dmHandle,OP_MODE_PROG);
	DM_Group **pdmGroupArray = (DM_Group **) pdmMode->pdmGroup;

	uint_least8_t i2;
	*pCellIndex = 0;
	for (i2=0; i2<pdmMode->u8ArraySize; i2++)
	{
		DM_Group *pdmGroup = pdmGroupArray[i2];
		DM_Function **pdmFunctionArray = (DM_Function **) pdmGroup->pdmFunction;
		uint_least8_t i3;
		*pGroupIndex = i2;
		for (i3=0; i3<pdmGroup->u8ArraySize; i3++)
		{
			DM_Function *pdmFunction = pdmFunctionArray[i3];
			uint_least8_t u8ArraySize = pdmFunction->u8ArraySize;
			*pFunctionIndex = i3;
			if (u16TotCellIndex < u8ArraySize)
			{
				*pCellIndex = (uint_least8_t)u16TotCellIndex;
				return;
			}
			u16TotCellIndex -= u8ArraySize;
		}

	}
}

char* ConvertToUnit(const DM_Cell *pdmCell)
{
	switch (pdmCell->u16Unit)
	{
		case DM_UNIT_Hz:
			return "Hz";
		case DM_UNIT_RPM:
			return " Rpm";
		case DM_UNIT_Amp:
			return "A";
		case DM_UNIT_Volt:
			return "V";
		case DM_UNIT_Power:
			return "kW";
		case DM_UNIT_Percent:
			return "%";
		case DM_UNIT_Sec:
			return "Sec";
		case DM_UNIT_Temp:
			return "C";
		default:
			return "";
	}
}

char* ConvertrToU8ValueString(uint_least8_t u8Value, uint16_t u16Attribute)
{
	uint16_t  u16Temp = u16Attribute & 0x00f0;
	static char ai8Buff[10];

	switch (u16Temp)
	{
		case DM_ATTRIBUTE_Point0:
			sprintf(ai8Buff,"%3u", u8Value);
			break;
		case DM_ATTRIBUTE_Point1:
			sprintf(ai8Buff,"%4.1f", u8Value/10.0f);
			//sprintf(ai8Buff,"%u.%01u", u8Value/10, u8Value % 10);
			break;
		case DM_ATTRIBUTE_Point2:
			sprintf(ai8Buff,"%4.2f", u8Value/100.0f);
			//sprintf(ai8Buff,"%u.%02u", u8Value/100, u8Value % 100);
			break;
		case DM_ATTRIBUTE_Point3:
			sprintf(ai8Buff,"%4.3f", u8Value/1000.0f);
			//sprintf(ai8Buff,"%u.%03u", u8Value/1000, u8Value % 1000);
			break;
		case  DM_ATTRIBUTE_Hex2:
			sprintf(ai8Buff,"%02x", u8Value);
			break;
		case DM_ATTRIBUTE_Hex4:
			sprintf(ai8Buff,"%04x", u8Value);
			break;
		case DM_ATTRIBUTE_Sts:
		{
			uint_least8_t i;
			for (i=0;i<8; i++)
			{
				ai8Buff[i] = (u8Value & 0x80) ? '1' : '0';
				u8Value <<= 1;
			}
			ai8Buff[8]=0;
			break;
		}
		default:
			ai8Buff[0]=0;

	}
	return ai8Buff;
}

char* ConvertrToU8EditValueString(uint_least8_t u8Value, uint16_t u16Attribute)
{
	uint16_t  u16Temp = u16Attribute & 0x00f0;
	static char ai8Buff[10];

	switch (u16Temp)
	{

		case DM_ATTRIBUTE_Point0:
			sprintf(ai8Buff,"%03u", u8Value);
			break;
		case DM_ATTRIBUTE_Point1:
			//sprintf(ai8Buff,"%02u.%01u", u8Value/10, u8Value % 10);
			sprintf(ai8Buff,"%04.1f", u8Value/10.0f);
			break;
		case DM_ATTRIBUTE_Point2:
			//sprintf(ai8Buff,"%01u.%02u", u8Value/100, u8Value % 100);
			sprintf(ai8Buff,"%04.2f", u8Value/100.0f);
			break;
		case DM_ATTRIBUTE_Point3:
			//sprintf(ai8Buff,"%01u.%03u", u8Value/1000, u8Value % 1000);
			sprintf(ai8Buff,"%04.3f", u8Value/1000.0f);
			break;
		case DM_ATTRIBUTE_Sts:
		case  DM_ATTRIBUTE_Hex2:
			sprintf(ai8Buff,"%02x", u8Value);
			break;
		case DM_ATTRIBUTE_Hex4:
			sprintf(ai8Buff,"%04x", u8Value);
			break;
		default:
			ai8Buff[0]=0;

	}
	return ai8Buff;
}

char* ConvertrToI8ValueString(int_least8_t i8Value, uint16_t u16Attribute)
{
	uint16_t  u16Temp = u16Attribute & 0x00f0;
	static char ai8Buff[10];

	switch (u16Temp)
	{
		case DM_ATTRIBUTE_Point0:
			sprintf(ai8Buff,"%4d", i8Value);
			//sprintf(ai8Buff,"%03d", i8Value);
			break;
		case DM_ATTRIBUTE_Point1:
			sprintf(ai8Buff,"%5.1f", i8Value/10.0f);
			//sprintf(ai8Buff,"%d.%01d", i8Value/10, i8Value % 10);
			break;
		case DM_ATTRIBUTE_Point2:
			sprintf(ai8Buff,"%5.2f", i8Value/100.0f);
			//sprintf(ai8Buff,"%d.%02d", i8Value/100, i8Value % 100);
			break;
		case DM_ATTRIBUTE_Point3:
			sprintf(ai8Buff,"%5.3f", i8Value/1000.0f);
			//sprintf(ai8Buff,"%d.%03d", i8Value/1000, i8Value % 1000);
			break;
		case  DM_ATTRIBUTE_Hex2:
			sprintf(ai8Buff,"%02x", i8Value);
			break;
		case DM_ATTRIBUTE_Hex4:
			sprintf(ai8Buff,"%04x", i8Value);
			break;
		default:
			ai8Buff[0]=0;

	}
	return ai8Buff;
}

char* ConvertrToI8EditValueString(int_least8_t i8Value, uint16_t u16Attribute)
{
	uint16_t  u16Temp = u16Attribute & 0x00f0;
	static char ai8Buff[10];

		switch (u16Temp)
	{
		case DM_ATTRIBUTE_Point0:
			sprintf(ai8Buff,"%+04d", i8Value);
			break;
		case DM_ATTRIBUTE_Point1:
			//sprintf(ai8Buff,"%02d.%01d", i8Value/10, i8Value % 10);
			sprintf(ai8Buff,"%+05.1f", i8Value/10.0f);
			break;
		case DM_ATTRIBUTE_Point2:
			//sprintf(ai8Buff,"%01d.%02d", i8Value/100, i8Value % 100);
			sprintf(ai8Buff,"%+05.2f", i8Value/100.0f);
			break;
		case DM_ATTRIBUTE_Point3:
			//(ai8Buff,"%0d.%03d", i8Value/1000, i8Value % 1000);
			sprintf(ai8Buff,"%+05.3f", i8Value/1000.0f);
			break;
		case  DM_ATTRIBUTE_Hex2:
			sprintf(ai8Buff,"%02x", i8Value);
			break;
		case DM_ATTRIBUTE_Hex4:
			sprintf(ai8Buff,"%04x", i8Value);
			break;
		default:
			ai8Buff[0]=0;

	}
	return ai8Buff;
}

char* ConvertrToU16ValueString(uint16_t u16Value, uint16_t u16Attribute)
{
	uint16_t  u16Temp = u16Attribute & 0x00f0;
	static char ai8Buff[10];

	switch (u16Temp)
	{
		case DM_ATTRIBUTE_Point0:
			sprintf(ai8Buff,"%5u", u16Value);
			break;
		case DM_ATTRIBUTE_Point1:
			//sprintf(ai8Buff,"%u.%01u", u16Value/10, u16Value % 10);
			sprintf(ai8Buff,"%7.1f", u16Value/10.0f);
			break;
		case DM_ATTRIBUTE_Point2:
			sprintf(ai8Buff,"%7.2f", u16Value/100.0f);
			//(ai8Buff,"%u.%02u", u16Value/100, u16Value % 100);
			break;
		case DM_ATTRIBUTE_Point3:
			sprintf(ai8Buff,"%7.3f", u16Value/1000.0f);
			//(ai8Buff,"%u.%03u", u16Value/1000, u16Value % 1000);
			break;
		case  DM_ATTRIBUTE_Hex2:
			sprintf(ai8Buff,"%02x", u16Value);
			break;
		case DM_ATTRIBUTE_Hex4:
			sprintf(ai8Buff,"%04x", u16Value);
			break;
		default:
			ai8Buff[0]=0;

	}
	return ai8Buff;
}

char* ConvertrToU16EditValueString(uint16_t u16Value, uint16_t u16Attribute)
{
	uint16_t  u16Temp = u16Attribute & 0x00f0;
	static char ai8Buff[10];

	switch (u16Temp)
	{
		case DM_ATTRIBUTE_Point0:
			sprintf(ai8Buff,"%05u", u16Value);
			break;
		case DM_ATTRIBUTE_Point1:
			//sprintf(ai8Buff,"%04u.%01u", u16Value/10, u16Value % 10);
			sprintf(ai8Buff,"%06.1f", u16Value/10.0f);
			break;
		case DM_ATTRIBUTE_Point2:
			//sprintf(ai8Buff,"%03u.%02u", u16Value/100, u16Value % 100);
			sprintf(ai8Buff,"%06.2f", u16Value/100.0f);
			break;
		case DM_ATTRIBUTE_Point3:
			//sprintf(ai8Buff,"%02u.%03u", u16Value/1000, u16Value % 1000);
			sprintf(ai8Buff,"%06.3f", u16Value/1000.0f);
			break;
		case  DM_ATTRIBUTE_Hex2:
			sprintf(ai8Buff,"%02x", u16Value);
			break;
		case DM_ATTRIBUTE_Hex4:
			sprintf(ai8Buff,"%04x", u16Value);
			break;
		default:
			ai8Buff[0]=0;

	}
	return ai8Buff;
}

char* ConvertrToI16ValueString(int16_t i16Value, uint16_t u16Attribute)
{
	uint16_t  u16Temp = u16Attribute & 0x00f0;
	//uint16_t  u16ModeValue;
	static char ai8Buff[10];

	switch (u16Temp)
	{
		case DM_ATTRIBUTE_Point0:

			//sprintf(ai8Buff,"%d", i16Value);
			sprintf(ai8Buff,"%6d", i16Value);
			break;
		case DM_ATTRIBUTE_Point1:
			//u16ModeValue = (abs(i16Value) % 10);
			//sprintf(ai8Buff,"%d.%01u", i16Value/10, u16ModeValue);
			sprintf(ai8Buff,"%7.1f", i16Value/10.0f);
			break;
		case DM_ATTRIBUTE_Point2:
			//u16ModeValue = (abs(i16Value) % 100);
			//sprintf(ai8Buff,"%d.%02u", i16Value/100, u16ModeValue);
			sprintf(ai8Buff,"%7.2f", i16Value/100.0f);
			break;
		case DM_ATTRIBUTE_Point3:
			//u16ModeValue = (abs(i16Value) % 100);
			//sprintf(ai8Buff,"%d.%03u", i16Value/1000, u16ModeValue);
			sprintf(ai8Buff,"%7.3f", i16Value/1000.0f);
			break;
		case  DM_ATTRIBUTE_Hex2:
			sprintf(ai8Buff,"%02x", i16Value);
			break;
		case DM_ATTRIBUTE_Hex4:
			sprintf(ai8Buff,"%04x", i16Value);
			break;
		default:
			ai8Buff[0]=0;

	}
	return ai8Buff;
}

char* ConvertrToI16EditValueString(int16_t i16Value, uint16_t u16Attribute)
{
	uint16_t  u16Temp = u16Attribute & 0x00f0;
	//uint16_t  u16ModeValue;
	static char ai8Buff[10];

	switch (u16Temp)
	{
		case DM_ATTRIBUTE_Point0:
			//sprintf(ai8Buff,"%05d", i16Value);
			sprintf(ai8Buff,"%+06d", i16Value);
			break;
		case DM_ATTRIBUTE_Point1:
			//u16ModeValue = abs(i16Value) % 10;
			//sprintf(ai8Buff,"%04d.%01u", i16Value/10, u16ModeValue);
			sprintf(ai8Buff,"%+07.1f", i16Value/10.0f);
			break;
		case DM_ATTRIBUTE_Point2:
			//u16ModeValue = abs(i16Value) % 100;
			//sprintf(ai8Buff,"%03d.%02u", i16Value/100, u16ModeValue);
			sprintf(ai8Buff,"%+07.2f", i16Value/100.0f);
			break;
		case DM_ATTRIBUTE_Point3:
			//u16ModeValue = abs(i16Value) % 1000;
			//sprintf(ai8Buff,"%02d.%03u", i16Value/1000, u16ModeValue);
			sprintf(ai8Buff,"%+07.3f", i16Value/1000.0f);
			break;
		case  DM_ATTRIBUTE_Hex2:
			sprintf(ai8Buff,"%+02x", i16Value);
			break;
		case DM_ATTRIBUTE_Hex4:
			sprintf(ai8Buff,"%+04x", i16Value);
			break;
		default:
			ai8Buff[0]=0;

	}
	return ai8Buff;
}

char* ConverterToValueString(const DM_Cell *pdmCell)
{
	DM_TYPE_e dmType = getDMType(pdmCell);
	uint16_t  u16Attribute = pdmCell->u16Attribute;
	uint16_t u16Value = DM_getCellValue(pdmCell);


	switch (dmType)
	{
		case DM_TYPE_Int16:
			return ConvertrToI16ValueString( u16Value ,u16Attribute);
		case DM_TYPE_UInt16:
			return ConvertrToU16ValueString( u16Value ,u16Attribute);
		case DM_TYPE_Int8:
			return ConvertrToI8ValueString( u16Value ,u16Attribute);
		case DM_TYPE_UInt8:
			return ConvertrToU8ValueString( u16Value ,u16Attribute);
		default:
			return "";
	}
}

char* ConverterToEditValueString(const DM_Cell *pdmCell, uint16_t u16Value)
{
	DM_TYPE_e dmType = getDMType(pdmCell);
	uint16_t  u16Attribute = pdmCell->u16Attribute;
	switch (dmType)
	{
		case DM_TYPE_Int16:
			return ConvertrToI16EditValueString(u16Value ,u16Attribute);
		case DM_TYPE_UInt16:
			return ConvertrToU16EditValueString(u16Value ,u16Attribute);
		case DM_TYPE_Int8:
			return ConvertrToI8EditValueString(u16Value,u16Attribute);
		case DM_TYPE_UInt8:
			return ConvertrToU8EditValueString(u16Value,u16Attribute);
		default:
			return "";
	}
}

uint16_t DM_getCellValue(const DM_Cell  *pdmCell)
{
	if (pdmCell->pValue == NULL)
	{
		return pdmCell->CallbackValue();
	}
	else
	{
		DM_TYPE_e dmType = getDMType(pdmCell);
		switch (dmType)
		{
			case DM_TYPE_Int16:
				return  *((int16_t *)pdmCell->pValue);
			case DM_TYPE_UInt16:
				return  *((uint16_t *)pdmCell->pValue);
			case DM_TYPE_Int8:
				return  *((int_least8_t *)pdmCell->pValue);
			case DM_TYPE_UInt8:
				return *((uint_least8_t *)pdmCell->pValue);
			default:
				return 0;
		}

	}
}
/*
void DM_getToValueInt32(DM_Handle dmHandle,
		 	 	 	 	   uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex,
						   uint_least8_t u8FunIndex, uint_least8_t u8CellIndex,
						   int32_t *pi32Value, int32_t *pi32MaxValue, int32_t *pi32MinValue, int32_t *pi32IncValue )
{

	DM_FunCell *pdmFunCell = DM_getFunCell(dmHandle,u8ModeIndex, u8GroupIndex, u8FunIndex,  u8CellIndex);
	const DM_Cell *pdmCell = pdmFunCell->pdmCell;
	DM_TYPE_e dmType = getDMType(pdmCell);


	switch (dmType)
	{
		case DM_TYPE_Int16:
			*pi32Value = *((int16_t *)pdmCell->pValue);
			*pi32MaxValue = ((int16_t) pdmCell->u16MaxValue);
			*pi32MinValue = ((int16_t) pdmCell->u16MinValue);
			break;
		case DM_TYPE_UInt16:
			*pi32Value = *((uint16_t *)pdmCell->pValue);
			*pi32MaxValue = ((uint16_t) pdmCell->u16MaxValue);
			*pi32MinValue = ((uint16_t) pdmCell->u16MinValue);
			break;
		case DM_TYPE_Int8:
			*pi32Value = *((int_least8_t *)pdmCell->pValue);
			*pi32MaxValue = ((int_least8_t) pdmCell->u16MaxValue);
			*pi32MinValue = ((int_least8_t) pdmCell->u16MinValue);
			break;
		case DM_TYPE_UInt8:
			*pi32Value = *((uint_least8_t *)pdmCell->pValue);
			*pi32MaxValue = ((uint_least8_t) pdmCell->u16MaxValue);
			*pi32MinValue = ((uint_least8_t) pdmCell->u16MinValue);
			break;
		default:
			*pi32Value = *pi32MaxValue = *pi32MinValue = 0;

	}

	switch (pdmCell->u16Attribute & 0x00f0)
	{
		case DM_ATTRIBUTE_Point3:
			*pi32IncValue = 1000;
			break;
		case DM_ATTRIBUTE_Point2:
			*pi32IncValue = 100;
			break;
		case DM_ATTRIBUTE_Point1:
			*pi32IncValue = 10;
			break;
		default:
			*pi32IncValue = 1;
			break;
	}
}*/

uint16_t DM_DecEditValue(uint16_t u16Value, uint16_t u16DecValue,  const DM_Cell* pdmCell)
{
	DM_TYPE_e dmType = getDMType(pdmCell);
	switch (dmType)
	{
		case DM_TYPE_Int16:
		case DM_TYPE_Int8:
		{
			int16_t i16MinValue =  pdmCell->u16MinValue;
			int16_t i16Value = u16Value;
			int16_t i16DecValue = u16DecValue;
			if (i16Value < i16MinValue + i16DecValue)
				i16Value = i16MinValue;
			else
				i16Value -= i16DecValue;
			return i16Value;
		}
		default:
		{
			uint16_t u16MinValue =  pdmCell->u16MinValue;
			//uint16_t u16Value = u16Value;
			//uint16_t u16DecValue = *pu16DecValue;
			if (u16Value < u16MinValue + u16DecValue)
				u16Value = u16MinValue;
			else
				u16Value -= u16DecValue;
			return u16Value;
		}

	}


}

uint16_t DM_IncEditValue(uint16_t u16Value, uint16_t u16IncValue,  const DM_Cell* pdmCell)
{
	DM_TYPE_e dmType = getDMType(pdmCell);
	switch (dmType)
	{
		case DM_TYPE_Int16:
		case DM_TYPE_Int8:
		{
			int16_t i16MaxValue =  pdmCell->u16MaxValue;
			int16_t i16Value = u16Value;
			int16_t i16IncValue = u16IncValue;
			if (i16Value > i16MaxValue - i16IncValue)
				i16Value = i16MaxValue;
			else
				i16Value += i16IncValue;
			return i16Value;
		}
		default:
		{
			uint16_t u16MaxValue =  pdmCell->u16MaxValue;
			//uint16_t u16Value = *pu16Value;
			//uint16_t u16DecValue = *pu16IncValue;
			if (u16Value > u16MaxValue - u16IncValue)
				u16Value = u16MaxValue;
			else
				u16Value += u16IncValue;
			return u16Value;
		}

	}


}

void DM_getToValue(DM_Handle dmHandle,
		 	 	   uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex,
				   uint_least8_t u8FunIndex, uint_least8_t u8CellIndex,
				   uint16_t *pu16Value, uint16_t *pu16IncValue, const DM_Cell* *pdmCell)
{
	DM_FunCell *pdmFunCell = DM_getFunCell(dmHandle,u8ModeIndex, u8GroupIndex, u8FunIndex,  u8CellIndex);
	DM_TYPE_e dmType;

	// DM_Cell *pdmCell = (DM_Cell *) pdmFunCell->pdmCell;
	*pdmCell = (DM_Cell *) pdmFunCell->pdmCell;
	dmType = getDMType(*pdmCell);

	//*pu16MaxValue =  pdmCell->u16MaxValue;
	//*pu16MinValue =  pdmCell->u16MinValue;
	//*pdmType = dmType;

	switch (dmType)
	{
		case DM_TYPE_Int16:
			*pu16Value = *((int16_t *)(*pdmCell)->pValue);
			//*pu16MaxValue =  pdmCell->u16MaxValue;
			//*pu16MinValue =  pdmCell->u16MinValue;
			break;
		case DM_TYPE_UInt16:
			*pu16Value = *((uint16_t *)(*pdmCell)->pValue);
			//*pi32MaxValue = ((uint16_t) pdmCell->u16MaxValue);
			//*pi32MinValue = ((uint16_t) pdmCell->u16MinValue);
			break;
		case DM_TYPE_Int8:
			*pu16Value = *((int_least8_t *)(*pdmCell)->pValue);
			//*pi32MaxValue = ((int_least8_t) pdmCell->u16MaxValue);
			//*pi32MinValue = ((int_least8_t) pdmCell->u16MinValue);
			break;
		case DM_TYPE_UInt8:
			*pu16Value = *((uint_least8_t *)(*pdmCell)->pValue);
			//*pi32MaxValue = ((uint_least8_t) pdmCell->u16MaxValue);
			//*pi32MinValue = ((uint_least8_t) pdmCell->u16MinValue);
			break;
		default:
			*pu16Value  = 0;

	}

	switch ((*pdmCell)->u16Attribute & 0x00f0)
	{
		case DM_ATTRIBUTE_Point3:
			*pu16IncValue = 1000;
			break;
		case DM_ATTRIBUTE_Point2:
			*pu16IncValue = 100;
			break;
		case DM_ATTRIBUTE_Point1:
			*pu16IncValue = 10;
			break;
		default:
			*pu16IncValue = 1;
			break;
	}
}

void DM_setEEPromValue(EEPROM_Handle eepromHandle, const DM_Cell* pdmCell)
{
	if (pdmCell->u16EEPromAddr != I2C_ADDRESS_INVALID)
	{
		DM_TYPE_e dmType = getDMType(pdmCell);
		switch (dmType)
		{
			case DM_TYPE_Int8:
			case DM_TYPE_UInt8:
				EEPROM_WriteByte(eepromHandle, pdmCell->u16EEPromAddr, *((uint_least8_t *)pdmCell->pValue));
				break;
			/*case DM_TYPE_UInt8:
				EEPROM_WriteByte(eepromHandle, pdmCell->u16EEPromAddr, *((uint_least8_t *)pdmCell->pValue));
				break;

			case DM_TYPE_Int16:
				EEPROM_Write(eepromHandle, pdmCell->u16EEPromAddr, *((int16_t *)pdmCell->pValue));
				break;*/
			default:
				EEPROM_Write(eepromHandle, pdmCell->u16EEPromAddr, *((uint16_t *)pdmCell->pValue));
				break;
		}
	}
}

uint16_t DM_getEEPromValue(EEPROM_Handle eepromHandle, const  DM_Cell* pdmCell)
{
	uint16_t u16Value;
	if (pdmCell->u16EEPromAddr != I2C_ADDRESS_INVALID)
	{
		DM_TYPE_e dmType = getDMType(pdmCell);
		switch (dmType)
		{
			case DM_TYPE_Int8:
			case DM_TYPE_UInt8:
			{
				uint_least8_t u8Temp;
				EEPROM_ReadByte(eepromHandle, pdmCell->u16EEPromAddr, &u8Temp);
				u16Value = u8Temp;
				//(*(int_least8_t *)pdmCell->pValue)= u8Temp;
				break;
			}
			//case DM_TYPE_UInt8:
			//{
			//
			//	EEPROM_ReadByte(eepromHandle, pdmCell->u16EEPromAddr, (uint_least8_t *)pdmCell->pValue);
			//	break;
			//}
			//case DM_TYPE_Int16:
			default:
			{
				EEPROM_Read(eepromHandle, pdmCell->u16EEPromAddr, &u16Value);
				//(*(int16_t *)pdmCell->pValue) = u16Temp;
				break;
			}


		}
	}
	else
	{
		u16Value = 0;
	}
	return u16Value;

}

/*uint16_t DM_DecValue(uint16 u16CurValue,uint16 u16DelValue, DM_Cell *pdmCell)
{
	u16CurValue
}
*/


void DM_setValue(DM_Handle dmHandle,
	 	   	   	 uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex,
	 	   	   	 uint_least8_t u8FunIndex, uint_least8_t u8CellIndex,
	 	   	   	 uint16_t u16Value)
{
	DM_Obj *pdmObj = (DM_Obj *) dmHandle;
	DM_FunCell *pdmFunCell = DM_getFunCell(dmHandle,u8ModeIndex, u8GroupIndex, u8FunIndex,  u8CellIndex);
    DM_Cell* pdmCell = (DM_Cell *) pdmFunCell->pdmCell;

	DM_TYPE_e dmType = getDMType(pdmCell);
	switch (dmType)
	{
		case DM_TYPE_Int16:
			*((int16_t *)pdmCell->pValue) = u16Value;
			break;
		case DM_TYPE_UInt16:
			*((uint16_t *)pdmCell->pValue)= u16Value;
			break;
		case DM_TYPE_Int8:
			*((int_least8_t *)pdmCell->pValue) =  u16Value;
			break;
		case DM_TYPE_UInt8:
			*((uint_least8_t *)pdmCell->pValue) = u16Value;

	}

	//void DM_setValueEEProm(EEPROM_Handle eepromHandle, DM_Cell *pdmCell)
	DM_setEEPromValue(pdmObj->eepromHandle, pdmCell);
	if (pdmCell->CallbackValue != NULL)
		pdmCell->CallbackValue();


}



bool DM_isWrite(DM_Handle dmHandle,
			    uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex,
			    uint_least8_t u8FunIndex, uint_least8_t u8CellIndex)
{
	DM_FunCell *pdmFunCell = DM_getFunCell(dmHandle,u8ModeIndex, u8GroupIndex, u8FunIndex,  u8CellIndex);
	const DM_Cell *pdmCell = pdmFunCell->pdmCell;
	return  ((pdmCell->u16Attribute & DM_ATTRIBUTE_Write) != 0);

}

bool DM_outEditCell(DM_Handle dmHandle,
				   uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex,
				   uint_least8_t u8FunIndex, uint_least8_t u8CellIndex,
				   uint16_t u16EditValue,
				   char *pai8Line1, char *pai8Line2)
{
	DM_Function *pdmFunction = DM_getFunction(dmHandle, u8ModeIndex, u8GroupIndex, u8FunIndex);
	DM_FunCell **pdmFunCellArray = (DM_FunCell **) pdmFunction->pdmFunCell;
	DM_FunCell *pdmFunCell = pdmFunCellArray[u8CellIndex];

	if (pdmFunCell->pai8DescArray != NULL)
	{
		uint_least8_t u8Value=  *((uint_least8_t *)pdmFunCell->pdmCell->pValue);
		const char *pi8Desc = pdmFunCell->pai8DescArray;
		sprintf(pai8Line1,"%s-%02u= %u", pdmFunction->ai8FunSym, pdmFunCell->u8Index,(uint_least8_t) u16EditValue);
		if (u8Value == u16EditValue) strcat(pai8Line1, "**");
		strcpy(pai8Line2, pi8Desc+u16EditValue*MAX_DESCRIPTION_LENGTH);
		return false;
	}
	else
	{

		strcpy(pai8Line1,pdmFunCell->ai8CellName);
		sprintf(pai8Line2,"%s %s", ConverterToEditValueString(pdmFunCell->pdmCell, u16EditValue), ConvertToUnit(pdmFunCell->pdmCell));
		return true;

	}

}

void DM_outFunCell(DM_Handle dmHandle,
				   uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex,
				   uint_least8_t u8FunIndex, uint_least8_t u8CellIndex,
				   char *pai8Line1, char *pai8Line2)
{

	DM_Function *pdmFunction = DM_getFunction(dmHandle, u8ModeIndex, u8GroupIndex, u8FunIndex);
	DM_FunCell **pdmFunCellArray = (DM_FunCell **) pdmFunction->pdmFunCell;
	DM_FunCell *pdmFunCell = pdmFunCellArray[u8CellIndex];

	//const DM_FunCell g_dmFunCellA1_01 	= {&g_dmCellLanguage,  0, "Select Language", &g_ai8DescLanguage[0][0]};
	strcpy(pai8Line1,pdmFunCell->ai8CellName);
	if (pdmFunCell->pai8DescArray != NULL)
	{
		const char *pi8Desc = pdmFunCell->pai8DescArray;
		uint_least8_t u8Value=  *((uint_least8_t *)pdmFunCell->pdmCell->pValue);
		strcpy(pai8Line2, pi8Desc+u8Value*MAX_DESCRIPTION_LENGTH);
		//const char g_ai8DescCtrlMethod[][MAX_DESCRIPTION_LENGTH] = {"V/f Control", "V/f with Pg", "Open Loop Vector","Flux Vector"};

	}
	else
	{
		sprintf(pai8Line2,"%s-%02u=%s%s",pdmFunction->ai8FunSym, pdmFunCell->u8Index,
			ConverterToValueString(pdmFunCell->pdmCell), ConvertToUnit(pdmFunCell->pdmCell));
	}


}

void DM_outGroup(DM_Handle dmHandle,
				   uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex,
				   char *pai8Line1, char *pai8Line2)
{
	DM_Group *pdmGroup = DM_getGroup(dmHandle, u8ModeIndex, u8GroupIndex);
	sprintf(pai8Line1, "Group %s", pdmGroup->ai8GroupSym);
	strcpy(pai8Line2, pdmGroup->ai8GroupName);

}



void DM_outFunction(DM_Handle dmHandle,
				   uint_least8_t u8ModeIndex, uint_least8_t u8GroupIndex, uint_least8_t u8FunIndex,
				   char *pai8Line1, char *pai8Line2)
{

	DM_Function *pdmFunction = DM_getFunction(dmHandle, u8ModeIndex, u8GroupIndex, u8FunIndex);
	sprintf(pai8Line1, "Function %s", pdmFunction->ai8FunSym);
	strcpy(pai8Line2, pdmFunction->ai8FunName);
}

//------------------------------------------------------------------------
int32_t setCallbackFreqRef()		//XX.XX Hz
{
	_iq iqHz_to_krpm_sf = _IQ(60.0/USER_MOTOR_NUM_POLE_PAIRS/1000/10);	// g_i16FreqRef/10 (Point1)
	//_iq iqSpeedRef_krpm =   _IQmpyI32( iqHz_to_krpm, (long) g_i16FreqRef);		//  iqHz_to_krpm_sf*g_i16FreqRef;
	//gMotorVars.iqSpeedRef_krpm = iqSpeedRef_krpm;
	gMotorVars.iqSpeedRef_krpm  =   _IQmpyI32( iqHz_to_krpm_sf, (long) g_i16FreqRef);

	return 0;		//Point2
}

int32_t setCallbackOutputTerm()
{
	//g_u8OutputTerm;
	g_u8OutputTerms &= 0x000f;
	//IOEXPAND_setHigh(gIoexpandHandle,(g_u8OutputTerm << 4), I2C_ASYNC);
	return 0;
}


//extern HAL_AdcData_t g_AdcData;
//------------------------------------------------------------------------
int32_t getCallbackFreqOutHz()	//XX.XX Hz Electric
{
	if (gMotorVars.bFlag_Run_Identify)
	{
		int32_t i32KRpmToHz = 10000L* USER_MOTOR_NUM_POLE_PAIRS/ 60L;
		//iq iqFreqHz = _IQmpyI32int(FILTER_FO_get_y1(gFilerOutFreqHandle),iqKRpmToHz);
				//	                   	   EST_get_pu_to_krpm_sf(gpcontroller_obj->estHandle));

		return (int32_t) _IQmpyI32int(FILTER_FO_get_y1(gFilterOutFreqHandle),i32KRpmToHz);

		//_iq pu_to_khzps_sf = _IQ((float_t)USER_TRAJ_FREQ_Hz * USER_IQ_FULL_SCALE_FREQ_Hz / 1000.0);
		//! _iq khzps_to_krpmps_sf = _IQ(60.0 / (float_t)USER_MOTOR_NUM_POLE_PAIRS);

		//iq delta_pu_to_kHz_sf = _IQ((float_t)USER_EST_FREQ_Hz/1000.0);
		//! _iq Force_Angle_Delta_pu = EST_getForceAngleDelta_pu(handle);
		//! _iq Force_Angle_Freq_kHz = _IQmpy(Force_Angle_Delta_pu, delta_pu_to_kHz_sf);
		//FILTER_FO_run(gFilerOutFreqHandle, EST_getSpeed_krpm(gpcontroller_obj->estHandle));
			//	FILTER_FO_run(gFilerOutEncoderHandle, ENC_getSpeedKRPM(gencHandle));

	}
	else
		return 0;

}

int32_t getCallbackFreqOutRpm()	//XXXX RPM
{
	if (gMotorVars.bFlag_Run_Identify)
	{
		//gencHandle
		//return  (int32_t) _IQmpyI32int(FILTER_FO_get_x1(gFilterDCBusHandle),USER_IQ_FULL_SCALE_VOLTAGE_V*10);
		//_iq iqFreqOutKrpm = _IQmpy(FILTER_FO_get_y1(gFilerOutFreqHandle),
		//	                   	   EST_get_pu_to_krpm_sf(gpcontroller_obj->estHandle));

		return (int32_t) _IQmpyI32int(FILTER_FO_get_y1(gFilterOutFreqHandle), 1000);

		//FILTER_FO_run(gFilerOutFreqHandle, EST_getSpeed_krpm(gpcontroller_obj->estHandle));
		//	FILTER_FO_run(gFilerOutEncoderHandle, ENC_getSpeedKRPM(gencHandle));

	}
	else
		return 0;

}

int32_t getCallbackCurrentOut()	//Point2
{
	if (gMotorVars.bFlag_Run_Identify)
	{
		//_iq iqCurrentOut = _IQmpy(gpcontroller_obj->pidHandle_Iq->iqfbackValue,_IQ(USER_IQ_FULL_SCALE_CURRENT_A));
		//return (int32_t) _IQmpyI32int(iqCurrentOut, 100);	//Point2
		return (int32_t) _IQmpyI32int(FILTER_FO_get_y1(gFilterOutCurrentHandle), USER_IQ_FULL_SCALE_CURRENT_A*100L);
		//return (int32_t) _IQmpyI32int(EST_getIab_pu(gpcontroller_obj->estHandle),
		//							  (long) (100*USER_IQ_FULL_SCALE_CURRENT_A));

	}
	else
		return 0;
}



int32_t getCallbackDCBus()
{
	//if (gMotorVars.bFlag_Run_Identify)
	//	return (int32_t) _IQmpyI32int(EST_getDcBus_pu(gpcontroller_obj->estHandle), USER_IQ_FULL_SCALE_VOLTAGE_V);
	//else

	//_iq Vbus_pu = EST_getDcBus_pu(handle);
	//! _iq Vbus_pu_to_kV_sf = _IQ(USER_IQ_FULL_SCALE_VOLTAGE_V / 1000.0);
	//! _iq Vbus_kV = _IQmpy(Vbus_pu,Vbus_pu_to_kV_sf);

	//extern FILTER_FO_Handle  gFilterDCBusHandle, gFilerOutFreqHandle;

	return  (int32_t) _IQmpyI32int(FILTER_FO_get_y1(gFilterDCBusHandle),USER_IQ_FULL_SCALE_VOLTAGE_V*10);


}

int32_t getCallbackVoltageU()
{
	return (int32_t) _IQmpyI32int(gAdcData.V.aiqvalue[0], (long)(USER_IQ_FULL_SCALE_VOLTAGE_V *10));

			//_IQtoF(gAdcData.V.aiqvalue[0]) *USER_IQ_FULL_SCALE_VOLTAGE_V *10);	// Point1
}

int32_t  getCallbackVoltageV()
{
	return (int32_t) _IQmpyI32int(gAdcData.V.aiqvalue[1], (long)(USER_IQ_FULL_SCALE_VOLTAGE_V *10));
	//return (int32_t)( _IQtoF(gAdcData.V.aiqvalue[1]) *USER_IQ_FULL_SCALE_VOLTAGE_V*10);	// Point1
}

int32_t  getCallbackVoltageW()
{
	return (int32_t) _IQmpyI32int(gAdcData.V.aiqvalue[2], (long)(USER_IQ_FULL_SCALE_VOLTAGE_V *10));
	//return (int32_t)( _IQtoF(gAdcData.V.aiqvalue[2]) *USER_IQ_FULL_SCALE_VOLTAGE_V*10);	//Point1

}

int32_t getCallbackCurrentU()
{
	return (int32_t) _IQmpyI32int(gAdcData.I.aiqvalue[0], (long)(USER_IQ_FULL_SCALE_CURRENT_A*100));
	//return (int32_t)( _IQtoF(gAdcData.I.aiqvalue[0]) *USER_IQ_FULL_SCALE_CURRENT_A);
}

int32_t getCallbackCurrentV()
{
	return (int32_t) _IQmpyI32int(gAdcData.I.aiqvalue[1], (long)(USER_IQ_FULL_SCALE_CURRENT_A*100));
	//return (int32_t)( _IQtoF(gAdcData.I.aiqvalue[1]) *USER_IQ_FULL_SCALE_CURRENT_A);
}

int32_t getCallbackCurrentW()
{
	return (int32_t) _IQmpyI32int(gAdcData.I.aiqvalue[2], (long)(USER_IQ_FULL_SCALE_CURRENT_A*100));
	//return (int32_t)( _IQtoF(gAdcData.I.aiqvalue[2]) *USER_IQ_FULL_SCALE_CURRENT_A);
}

int32_t getCallbackInputTerm()
{
	return (int32_t) g_u16InputTerms >> 8;
}


int32_t getCallbackExtTemp()
{
	return (int32_t) _IQmpyI32int(gAdcData.iqExtTemp, (long)(USER_IQ_FULL_SCALE_EXT_TEMP_C*10));
}


int32_t getCallbackExtAd1()
{
	return (int32_t) _IQmpyI32int(gAdcData.iqExtAdc[0], (long)(USER_IQ_FULL_SCALE_EXT_ADC_C*100));

}

int32_t getCallbackExtAd2()
{
	return (int32_t) _IQmpyI32int(gAdcData.iqExtAdc[1], (long)(USER_IQ_FULL_SCALE_EXT_ADC_C*100));

}

int32_t getCallbackExtAd3()
{
	return (int32_t) _IQmpyI32int(gAdcData.iqExtAdc[2], (long)(USER_IQ_FULL_SCALE_EXT_ADC_C*100));

}

int32_t getCallbackExtAd4()
{
	return (int32_t) _IQmpyI32int(gAdcData.iqExtAdc[3], (long)(USER_IQ_FULL_SCALE_EXT_ADC_C*100));

}

int32_t getCallbackExtAd5()
{
	return (int32_t) _IQmpyI32int(gAdcData.iqExtAdc[4], (long)(USER_IQ_FULL_SCALE_EXT_ADC_C*100));

}

int32_t getCallbackExtAd6()
{
	return (int32_t) _IQmpyI32int(gAdcData.iqExtAdc[5], (long)(USER_IQ_FULL_SCALE_EXT_ADC_C*100));

}


