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
 *    notice, this list of conditions and the following disclaimer in thef
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
#include "sci_operator.h"
#include "sci_message.h"
#include "user_data.h"
#include "main.h"


#define KEY_DELAY_500MS	(500/8)
#define KEY_DELAY_200MS	(200/8)
#define KEY_DELAY_100MS	(100/8)
#define KEY_DELAY_50MS	(50/8)

extern  MOTOR_Vars_t gMotorVars;

OPERATOR_Obj *g_pOperatorObj;
DM_TYPE_e 	g_dmTypeCurrent;
uint16_t 	g_u16EditValue, g_u16DeltaValue;
const DM_Cell *g_pdmCellCurrent;

OPERATOR_Handle OPERATOR_init(void *pMemory,const size_t numBytes)
{
	OPERATOR_Handle operatorHandle;

	if (numBytes < sizeof(OPERATOR_Obj))
		return((OPERATOR_Handle)NULL);

	// assign the handle
	operatorHandle = (OPERATOR_Handle)pMemory;

	return(operatorHandle);
}

void OPERATOR_setup(OPERATOR_Handle operatorHandle, DM_Handle DmHandle)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	uint_least8_t i,j;

	pOperator->DmHandle = DmHandle;
	pOperator-> OpCommStatus = OPERATOR_INIT1;
	g_pOperatorObj = pOperator;

	pOperator->u16Key = 0;
	pOperator->u8LEDBlink = 0;
	pOperator->u8LEDOnOff = 0;

	for (j=0; j<2; j++)
	{
		pOperator->u16LCDBlink[j] =0;
		for (i=0; i<16; i++)
			pOperator->ai8LCD[j][i]= ' ';
	}

	OPERATOR_runMenuKey(operatorHandle);
}



void OPERATOR_setLEDBlink(OPERATOR_Handle operatorHandle,uint_least8_t u8BlinkData)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	pOperator->u8LEDBlink |= u8BlinkData;
	pOperator->u8LEDOnOff |= u8BlinkData;
}

void OPERATOR_setLEDOn(OPERATOR_Handle operatorHandle,uint_least8_t u8OnData)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	pOperator->u8LEDOnOff |= u8OnData;
	pOperator->u8LEDBlink &= (~u8OnData);
	pOperator->u8LEDBlink &= 0x00ff;

}

void OPERATOR_setLEDOff(OPERATOR_Handle operatorHandle,uint_least8_t u8OffData)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	pOperator->u8LEDOnOff &= (~u8OffData);
	pOperator->u8LEDOnOff &= 0x00ff;
	pOperator->u8LEDBlink &= (~u8OffData);
	pOperator->u8LEDBlink &= 0x00ff;
}


void OPERATOR_setupLEDs(OPERATOR_Handle operatorHandle)
{
	if (gMotorVars.bFlag_enableSys)
	{
		OPERATOR_setLEDOn(operatorHandle, OPERATOR_DRIVE_LED);
		if (gMotorVars.bFlag_Run_Identify)
		{
			OPERATOR_setLEDOff(operatorHandle, OPERATOR_STOP_LED);
			if (_IQabs(gMotorVars.iqSpeedTraj_krpm) < _IQ(0.01))
				OPERATOR_setLEDBlink(operatorHandle, OPERATOR_RUN_LED);
			else
				OPERATOR_setLEDOn(operatorHandle, OPERATOR_RUN_LED);

			if (gMotorVars.iqSpeedRef_krpm >= 0)
			{
				OPERATOR_setLEDOn (operatorHandle, OPERATOR_FWD_LED);
				OPERATOR_setLEDOff(operatorHandle, OPERATOR_REV_LED);
			}
			else
			{
				OPERATOR_setLEDOff(operatorHandle, OPERATOR_FWD_LED);
				OPERATOR_setLEDOn (operatorHandle, OPERATOR_REV_LED);
			}
		}
		else
		{
			OPERATOR_setLEDOff(operatorHandle, OPERATOR_RUN_LED);

			if (_IQabs(gMotorVars.iqSpeed_krpm) < _IQ(0.2))
			{
				OPERATOR_setLEDOn(operatorHandle, OPERATOR_STOP_LED);
				OPERATOR_setLEDOff(operatorHandle, OPERATOR_FWD_LED | OPERATOR_REV_LED );
			}
			else
				OPERATOR_setLEDBlink(operatorHandle, OPERATOR_STOP_LED);

		}
	}
	else
	{
		OPERATOR_setLEDOff(operatorHandle,
							 OPERATOR_STOP_LED | OPERATOR_RUN_LED | OPERATOR_DRIVE_LED | OPERATOR_REV_LED |
							 OPERATOR_FWD_LED  | OPERATOR_SEQ_LED | OPERATOR_SEQ_LED | OPERATOR_REF_LED);
	}




}

void OPERATOR_setLCDBlinkOn(OPERATOR_Handle operatorHandle,uint_least8_t u8LineNo, uint16_t u16OnData)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	pOperator->u16LCDBlink[u8LineNo] |= u16OnData;
}

void OPERATOR_setLCDBlinkOff(OPERATOR_Handle operatorHandle,uint_least8_t u8LineNo, uint16_t u16OffData)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	pOperator->u16LCDBlink[u8LineNo] &= (~u16OffData);
}

void OPERATOR_setLCDChar(OPERATOR_Handle operatorHandle, uint_least8_t u8LineNo, uint_least8_t u8PosNo, int_least8_t i8CharValue)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	pOperator->ai8LCD[u8LineNo][u8PosNo] = i8CharValue;
}

void OPERATOR_clearLCDChar(OPERATOR_Handle operatorHandle, uint_least8_t u8LineNo, uint_least8_t u8PosNo)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	pOperator->ai8LCD[u8LineNo][u8PosNo] = ' ';
}

void OPERATOR_setLCDString(OPERATOR_Handle operatorHandle, uint_least8_t u8LineNo, uint_least8_t u8PosNo, char *pi8Data)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	int_least8_t *pi8LCDBuff = &pOperator->ai8LCD[u8LineNo][u8PosNo];
	uint_least8_t u8Value;
	while ( (u8Value = *pi8Data++) )
		*pi8LCDBuff++ = u8Value;
}

void OPERATOR_setCenterLCDString(OPERATOR_Handle operatorHandle, int_least8_t u8LineNo, char *pi8String)
{
	int_least8_t i8Len= 0;
	while (pi8String[i8Len]) i8Len++;
	OPERATOR_setLCDString(operatorHandle, u8LineNo, (16-i8Len)>>1, pi8String);

}

void OPERATOR_clearLCDLine(OPERATOR_Handle operatorHandle,uint_least8_t u8LineNo)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	int_least8_t *pi8LCDBuff = &pOperator->ai8LCD[u8LineNo][0];
	uint_least8_t i;
	for (i=0; i< 16; i++)
		*pi8LCDBuff++ = ' ';
	pOperator->u16LCDBlink[u8LineNo] = 0;
}

void OPERATOR_clearLCDAll(OPERATOR_Handle operatorHandle)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	uint_least8_t i,j;
	for (j=0; j<2; j++)
	{
		pOperator->u16LCDBlink[j] = 0;
		for (i=0; i< 16; i++)
			pOperator->ai8LCD[j][i] = ' ';
	}
}

bool OPERATOR_isKey(OPERATOR_Handle operatorHandle, OPERRATOR_Key_e operKey)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	return (pOperator->u16Key & ((uint16_t) operKey));
}


void OPERATOR_showModeLevel(OPERATOR_Handle operatorHandle)
{
	char *pi8ModeName;
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;

	OPERATOR_clearLCDAll(operatorHandle);
	OPERATOR_setCenterLCDString(operatorHandle,0, " * Main Menu * ");

	pi8ModeName = (char *) DM_getModeName(pOperator->DmHandle, pOperator->u8ModeIndex);
	OPERATOR_setCenterLCDString(operatorHandle, 1, pi8ModeName);
}

void OPERATOR_showGroupLevel(OPERATOR_Handle operatorHandle)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	char ai8Line1[20], ai8Line2[20];

	DM_outGroup(pOperator->DmHandle,
				pOperator->u8ModeIndex, pOperator->u8GroupIndex,
				ai8Line1, ai8Line2);
	OPERATOR_clearLCDAll(operatorHandle);
	OPERATOR_setCenterLCDString(operatorHandle, 0, ai8Line1);
	OPERATOR_setCenterLCDString(operatorHandle, 1, ai8Line2);


}

void OPERATOR_showFunLevel(OPERATOR_Handle operatorHandle)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	//DM_Handle dmHandle = pOperator->dmHandle;
	char ai8Line1[20], ai8Line2[20];

	DM_outFunction(pOperator->DmHandle,
				   pOperator->u8ModeIndex, pOperator->u8GroupIndex, pOperator->u8FunIndex,
				   ai8Line1, ai8Line2);
	OPERATOR_clearLCDAll(operatorHandle);
	OPERATOR_setCenterLCDString(operatorHandle, 0, ai8Line1);
	OPERATOR_setCenterLCDString(operatorHandle, 1, ai8Line2);

}

void OPERATOR_showCellLevel(OPERATOR_Handle operatorHandle)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	DM_Handle DmHandle = pOperator->DmHandle;
	char ai8Line1[20], ai8Line2[20];

	switch (pOperator->u8ModeIndex)
	{
		case OP_MODE_OPERATION:
			if (pOperator->u16SpecialIndex == 0xffff)
			{
				uint_least8_t u8ItemNo = DM_getOperModeSizeNo(DmHandle);
				if (pOperator->u8CellIndex == u8ItemNo)
					DM_outFunction(DmHandle,
								   OP_MODE_OPERATION, OP_GROUP_U,0,
								   ai8Line1, ai8Line2);
				else if (pOperator->u8CellIndex == u8ItemNo+1)
					DM_outFunction(DmHandle,
									OP_MODE_OPERATION, OP_GROUP_U,1,
									ai8Line1, ai8Line2);
				//else if (pOperator->u8CellIndex == u8ItemNo+2)
				//	DM_outFunction(DmHandle,
				//				   OP_MODE_OPERATION, OP_GROUP_U,0,
				//				   ai8Line1, ai8Line2);
				else
					DM_outFunCell(DmHandle,
								  OP_MODE_OPERATION, OP_GROUP_U,0,pOperator->u8CellIndex,
								  ai8Line1, ai8Line2);
			}
			else
				DM_outFunCell(DmHandle,
						      OP_MODE_OPERATION, OP_GROUP_U, pOperator->u8FunIndex,  pOperator->u8CellIndex,
							  ai8Line1, ai8Line2);
			break;
		case OP_MODE_INIT:
			if (pOperator->u16SpecialIndex == 0xffff)
			{
				uint_least8_t u8ItemNo = DM_getCellSizeNo(DmHandle, OP_MODE_INIT, OP_GROUP_A,0);
				if (pOperator->u8CellIndex == u8ItemNo)
					DM_outFunction(DmHandle,
								    OP_MODE_INIT, OP_GROUP_A,1,
									 ai8Line1, ai8Line2);
				else
					DM_outFunCell(DmHandle,
							      OP_MODE_INIT, OP_GROUP_A,0,pOperator->u8CellIndex,
								ai8Line1, ai8Line2);
			}
			else
				DM_outFunCell(DmHandle,
							  OP_MODE_INIT, OP_GROUP_A, 1,  pOperator->u8CellIndex,
							  ai8Line1, ai8Line2);
			break;
		case OP_MODE_PROG:
			DM_outFunCell(DmHandle,
						  pOperator->u8ModeIndex, pOperator->u8GroupIndex, pOperator->u8FunIndex,  pOperator->u8CellIndex,
						  ai8Line1, ai8Line2);
			break;
	}

	OPERATOR_clearLCDAll(operatorHandle);
	OPERATOR_setCenterLCDString(operatorHandle, 0, ai8Line1);
	OPERATOR_setCenterLCDString(operatorHandle, 1, ai8Line2);
}

void DetermineBlinkDig(OPERATOR_Handle operatorHandle)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	uint_least8_t u8DigNo=0, u8LoopNo;
	while (pOperator->ai8LCD[1][u8DigNo] == ' ') u8DigNo++;	// Find first char != ' '
	while (pOperator->ai8LCD[1][u8DigNo] != ' ') u8DigNo++; // Find char == ' '
	switch (g_u16DeltaValue)
	{
		case 10:
			u8LoopNo = 2;
			break;
		case 100:
			u8LoopNo = 3;
			break;
		case 1000:
			u8LoopNo = 4;
			break;
		default:
			u8LoopNo = 1;
			break;
	}
	while (u8LoopNo--)
	{
		while (pOperator->ai8LCD[1][--u8DigNo] == '.');
	}
	pOperator->u16LCDBlink[1] = 1 << u8DigNo;
}

void OPERATOR_showCellEditLevel(OPERATOR_Handle operatorHandle)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	char ai8Line1[20], ai8Line2[20];
	bool bFlag;

	bFlag = DM_outEditCell(pOperator->DmHandle,
						   pOperator->u8ModeIndex,pOperator->u8GroupIndex, pOperator->u8FunIndex,pOperator->u8CellIndex,
							g_u16EditValue,
							ai8Line1, ai8Line2);
	OPERATOR_clearLCDAll(operatorHandle);
	OPERATOR_setCenterLCDString(operatorHandle, 0, ai8Line1);
	OPERATOR_setCenterLCDString(operatorHandle, 1, ai8Line2);
	if (bFlag)
		DetermineBlinkDig(operatorHandle);
}

void OPERATOR_showEntryLevel(OPERATOR_Handle operatorHandle)
{
	//OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	char ai8Line1[20], ai8Line2[20];

	ai8Line1[0]=0;
	strcpy(ai8Line2,"Entry Accepted");
	OPERATOR_clearLCDAll(operatorHandle);
	OPERATOR_setCenterLCDString(operatorHandle, 0, ai8Line1);
	OPERATOR_setCenterLCDString(operatorHandle, 1, ai8Line2);
}


void OPERATOR_runMenuKey(OPERATOR_Handle operatorHandle)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	pOperator->OpStatus = OP_STATUS_MODELEVEL1;
	pOperator->u8ModeIndex = pOperator->u8GroupIndex = pOperator->u8FunIndex = pOperator->u8CellIndex = 0;
	OPERATOR_showModeLevel(operatorHandle);
}


void OPERATOR_runUpKey_Level4Default(OPERATOR_Handle operatorHandle)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	uint_least8_t u8ItemNo;
	u8ItemNo = DM_getCellSizeNo(pOperator->DmHandle,
								pOperator->u8ModeIndex,pOperator->u8GroupIndex, pOperator->u8FunIndex );
	pOperator->u8CellIndex =  (pOperator->u8CellIndex+1)% u8ItemNo;
	OPERATOR_showCellLevel(operatorHandle);
}

void OPERATOR_runUpKey(OPERATOR_Handle operatorHandle)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	DM_Handle DmHandle = pOperator->DmHandle;
	switch (pOperator->OpStatus)
	{
		uint_least8_t u8ItemNo;
		case OP_STATUS_MODELEVEL1:
		{
			u8ItemNo = DM_getModeSizeNo(DmHandle);
			pOperator->u8ModeIndex = (pOperator->u8ModeIndex+1)% u8ItemNo;
			OPERATOR_showModeLevel(operatorHandle);
			break;
		}
		case OP_STATUS_GROUPLEVEL2:
		{
			u8ItemNo = DM_getGroupSizeNo(DmHandle, pOperator->u8ModeIndex);
			pOperator->u8GroupIndex = (pOperator->u8GroupIndex+1)% u8ItemNo;
			OPERATOR_showGroupLevel(operatorHandle);
			break;
		}
		case OP_STATUS_FUNLEVEL3:
		{
			uint_least8_t u8AccessLevel = DM_getAccessLevel(DmHandle);
			if (u8AccessLevel == OP_ACCESS_BASIC)
			{
				uint16_t u16ItemNo = DM_getTotalFunctionSizeNo(DmHandle);
				pOperator->u16SpecialIndex = (pOperator->u16SpecialIndex +1 ) % u16ItemNo;
				DM_findGroupFunctionIndex(DmHandle, pOperator->u16SpecialIndex,
										  &pOperator->u8GroupIndex, &pOperator->u8FunIndex);
			}
			else
			{
				u8ItemNo = DM_getFunctionSizeNo(DmHandle, pOperator->u8ModeIndex, pOperator->u8FunIndex);
				pOperator->u8FunIndex = (pOperator->u8FunIndex+1)% u8ItemNo;
			}
			OPERATOR_showFunLevel(operatorHandle);
			break;
		}
		case OP_STAUS_CELLLEVEL4:
		{
			switch (pOperator->u8ModeIndex)
			{
				case OP_MODE_OPERATION:
					if (pOperator->u16SpecialIndex == 0xffff)
					{
						u8ItemNo = DM_getOperModeSizeNo(DmHandle) + 2;
						pOperator->u8CellIndex =  (pOperator->u8CellIndex+1)% u8ItemNo;
						OPERATOR_showCellLevel(operatorHandle);
					}
					else
						OPERATOR_runUpKey_Level4Default(operatorHandle);
					break;
				case OP_MODE_INIT:
					if (pOperator->u16SpecialIndex == 0xffff)
					{
						u8ItemNo = DM_getCellSizeNo(DmHandle, OP_MODE_INIT, OP_GROUP_A,0);
						pOperator->u8CellIndex =  (pOperator->u8CellIndex+1)% u8ItemNo;
						OPERATOR_showCellLevel(operatorHandle);
					}
					else
						OPERATOR_runUpKey_Level4Default(operatorHandle);

					break;
				case OP_MODE_PROG:
				{
					uint_least8_t u8AccessLevel = DM_getAccessLevel(DmHandle);
					if (u8AccessLevel == OP_ACCESS_QUICK)
					{
						uint16_t u16ItemNo = DM_getTotalCellSizeNo(DmHandle);
						pOperator->u16SpecialIndex = (pOperator->u16SpecialIndex +1 ) % u16ItemNo;
						DM_findGroupFunctionCellIndex(DmHandle, pOperator->u16SpecialIndex,
													 &pOperator->u8GroupIndex, &pOperator->u8FunIndex, &pOperator->u8CellIndex);
						OPERATOR_showCellLevel(operatorHandle);
					}
					else
						OPERATOR_runUpKey_Level4Default(operatorHandle);
					break;
				}

			}
			break;

		}
		case OP_STATUS_EDITLEVEL5:
		{
			g_u16EditValue = DM_IncEditValue(g_u16EditValue, g_u16DeltaValue, g_pdmCellCurrent);
			//g_u16EditValue = g_u16EditValue + g_u16DeltaValue;
			//if (g_u16EditValue > g_u16EditMaxValue )
			//	g_u16EditValue = g_u16EditMaxValue;
			OPERATOR_showCellEditLevel(operatorHandle);
			break;
		}
	}

}



void OPERATOR_runDownKey_Level4Default(OPERATOR_Handle operatorHandle)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	uint_least8_t u8ItemNo;
	u8ItemNo = DM_getCellSizeNo(pOperator->DmHandle,
								pOperator->u8ModeIndex,pOperator->u8GroupIndex, pOperator->u8FunIndex );
	if (pOperator->u8CellIndex)
	   pOperator->u8CellIndex--;
	else
	   pOperator->u8CellIndex = u8ItemNo -1;
	OPERATOR_showCellLevel(operatorHandle);
}


void OPERATOR_runDownKey(OPERATOR_Handle operatorHandle)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	DM_Handle DmHandle = pOperator->DmHandle;
	switch (pOperator->OpStatus)
	{
		uint_least8_t u8ItemNo;
		case OP_STATUS_MODELEVEL1:
		{
			u8ItemNo = DM_getModeSizeNo(DmHandle);
			if (pOperator->u8ModeIndex)
				pOperator->u8ModeIndex--;
			else
				pOperator->u8ModeIndex = u8ItemNo-1;
			OPERATOR_showModeLevel(operatorHandle);
			break;
		}
		case OP_STATUS_GROUPLEVEL2:
		{
			u8ItemNo = DM_getGroupSizeNo(DmHandle, pOperator->u8ModeIndex);
			if (pOperator->u8GroupIndex)
				pOperator->u8GroupIndex--;
			else
				pOperator->u8GroupIndex = u8ItemNo-1;
			OPERATOR_showGroupLevel(operatorHandle);
			break;

		}
		case OP_STATUS_FUNLEVEL3:
		{
			uint_least8_t u8AccessLevel = DM_getAccessLevel(DmHandle);
			if (u8AccessLevel == OP_ACCESS_BASIC)
			{
				uint16_t u16ItemNo = DM_getTotalFunctionSizeNo(DmHandle);
				if (pOperator->u16SpecialIndex)
					pOperator->u16SpecialIndex--;
				else
					pOperator->u16SpecialIndex = u16ItemNo -1;

				DM_findGroupFunctionIndex(DmHandle, pOperator->u16SpecialIndex,
										  &pOperator->u8GroupIndex, &pOperator->u8FunIndex);
			}
			else
			{
				u8ItemNo = DM_getFunctionSizeNo(DmHandle, pOperator->u8ModeIndex, pOperator->u8FunIndex);
				if (pOperator->u8FunIndex)
					pOperator->u8FunIndex--;
				else
					pOperator->u8FunIndex = u8ItemNo-1;
			}
			OPERATOR_showFunLevel(operatorHandle);
			break;

		}
		case OP_STAUS_CELLLEVEL4:
		{
			switch (pOperator->u8ModeIndex)
			{
				case OP_MODE_OPERATION:
					if (pOperator->u16SpecialIndex == 0xffff)
					{
						u8ItemNo = DM_getOperModeSizeNo(DmHandle) + 2;
						if (pOperator->u8CellIndex)
							pOperator->u8CellIndex--;
						else
							pOperator->u8CellIndex =  u8ItemNo -1;
						OPERATOR_showCellLevel(operatorHandle);
					}
					else
						OPERATOR_runDownKey_Level4Default(operatorHandle);
					break;
				case OP_MODE_INIT:
					if (pOperator->u16SpecialIndex == 0xffff)
					{
						u8ItemNo = DM_getCellSizeNo(DmHandle, OP_MODE_INIT, OP_GROUP_A, 0);	//A1
						if (pOperator->u8CellIndex)
							pOperator->u8CellIndex--;
						else
							pOperator->u8CellIndex =  u8ItemNo -1;
						OPERATOR_showCellLevel(operatorHandle);
					}
					else
						OPERATOR_runDownKey_Level4Default(operatorHandle);

					break;
				case OP_MODE_PROG:
				{
					uint_least8_t u8AccessLevel = DM_getAccessLevel(DmHandle);
					if (u8AccessLevel == OP_ACCESS_QUICK)
					{
						uint16_t u16ItemNo = DM_getTotalCellSizeNo(DmHandle);
						if (pOperator->u16SpecialIndex)
							pOperator->u16SpecialIndex--;
						else
							pOperator->u16SpecialIndex = u16ItemNo -1;

						DM_findGroupFunctionCellIndex(DmHandle, pOperator->u16SpecialIndex,
													&pOperator->u8GroupIndex, &pOperator->u8FunIndex, &pOperator->u8CellIndex);
						OPERATOR_showCellLevel(operatorHandle);
					}
					else
						OPERATOR_runDownKey_Level4Default(operatorHandle);
					break;
				}

			}
			break;

		}
		case OP_STATUS_EDITLEVEL5:
		{
			g_u16EditValue = DM_DecEditValue(g_u16EditValue, g_u16DeltaValue, g_pdmCellCurrent);
			//g_u16EditValue = g_u16EditValue - g_u16DeltaValue;
			//if (g_u16EditValue < g_u16EditMinValue )
			//	g_u16EditValue = g_u16EditMinValue;
			OPERATOR_showCellEditLevel(operatorHandle);
			break;
		}



	}
}


void OPERATOR_runEnterKey_Level4Default(OPERATOR_Handle operatorHandle)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	if (DM_isWrite(pOperator->DmHandle,
				   pOperator->u8ModeIndex, pOperator->u8GroupIndex, pOperator->u8FunIndex, pOperator->u8CellIndex))
	{
		pOperator->OpStatus = OP_STATUS_EDITLEVEL5;
		DM_getToValue(pOperator->DmHandle,
				      pOperator->u8ModeIndex, pOperator->u8GroupIndex, pOperator->u8FunIndex,  pOperator->u8CellIndex,
				      &g_u16EditValue, &g_u16DeltaValue, &g_pdmCellCurrent);
		OPERATOR_showCellEditLevel(operatorHandle);
	}
}

void OPERATOR_runEnterKey(OPERATOR_Handle operatorHandle)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	switch (pOperator->OpStatus)
	{
		case OP_STATUS_MODELEVEL1:
		{
			switch (pOperator->u8ModeIndex)
			{
				case OP_MODE_OPERATION:
						pOperator->OpStatus = OP_STAUS_CELLLEVEL4;
						pOperator->u8GroupIndex = OP_GROUP_U;
						pOperator->u8FunIndex = pOperator->u8CellIndex = 0;
						pOperator->u16SpecialIndex = 0xffff;
						OPERATOR_showCellLevel(operatorHandle);
						break;
				case OP_MODE_INIT:
						pOperator->OpStatus = OP_STAUS_CELLLEVEL4;
						pOperator->u8GroupIndex = OP_GROUP_A;
						pOperator->u8FunIndex = pOperator->u8CellIndex = 0;
						pOperator->u16SpecialIndex = 0xffff;
						OPERATOR_showCellLevel(operatorHandle);
						break;
				case OP_MODE_PROG:
				{
					uint_least8_t u8AccessLevel = DM_getAccessLevel(pOperator->DmHandle);
					if (u8AccessLevel == OP_ACCESS_ADVANCE)
					{
						pOperator->OpStatus = OP_STATUS_GROUPLEVEL2;
						pOperator->u8GroupIndex = OP_GROUP_B;
						pOperator->u8FunIndex = pOperator->u8CellIndex = 0;
						OPERATOR_showGroupLevel(operatorHandle);
					}
					else if (u8AccessLevel == OP_ACCESS_BASIC)
					{
						pOperator->OpStatus = OP_STATUS_FUNLEVEL3;
						pOperator->u8GroupIndex = OP_GROUP_B;
						pOperator->u8FunIndex = pOperator->u8CellIndex = 0;
						pOperator->u16SpecialIndex = 0;
						OPERATOR_showFunLevel(operatorHandle);
					}
					else if (u8AccessLevel == OP_ACCESS_QUICK)
					{
						pOperator->OpStatus = OP_STAUS_CELLLEVEL4;
						pOperator->u8GroupIndex = OP_GROUP_B;
						pOperator->u8FunIndex = pOperator->u8CellIndex = 0;
						pOperator->u16SpecialIndex = 0;
						OPERATOR_showCellLevel(operatorHandle);
					}
					break;
				}
			}
			break;
		}
		case OP_STATUS_GROUPLEVEL2:
		{
			pOperator->OpStatus = OP_STATUS_FUNLEVEL3;
			OPERATOR_showFunLevel(operatorHandle);
			break;
		}
		case OP_STATUS_FUNLEVEL3:
		{
			pOperator->OpStatus = OP_STAUS_CELLLEVEL4;
			OPERATOR_showCellLevel(operatorHandle);
			break;
		}
		case OP_STAUS_CELLLEVEL4:
			switch (pOperator->u8ModeIndex)
			{
				case OP_MODE_OPERATION:
				{
					if (pOperator->u16SpecialIndex == 0xffff)
					{
						uint_least8_t u8ItemNo = DM_getOperModeSizeNo(pOperator->DmHandle);
						if (pOperator->u8CellIndex < u8ItemNo)
							OPERATOR_runEnterKey_Level4Default(operatorHandle);
						else
						{
							if (pOperator->u8CellIndex == u8ItemNo)
								pOperator->u8FunIndex = 0;	//U1
							else //if (pOperator->u8CellIndex == u8ItemNo+1)
								pOperator->u8FunIndex = 1;	//U2
							//else
							//	pOperator->u8FunIndex = 0;	//U1

							pOperator->u16SpecialIndex = pOperator->u8CellIndex;
							pOperator->u8CellIndex = 0;
							OPERATOR_showCellLevel(operatorHandle);
						}
					}
					else
						OPERATOR_runEnterKey_Level4Default(operatorHandle);
					break;
				}
				case OP_MODE_INIT:
				{
					if (pOperator->u16SpecialIndex == 0xffff)
					{
						uint_least8_t u8ItemNo = DM_getCellSizeNo(pOperator->DmHandle, OP_MODE_INIT, OP_GROUP_A,0);
						if (pOperator->u8CellIndex < u8ItemNo)
							OPERATOR_runEnterKey_Level4Default(operatorHandle);
						else
						{
							pOperator->u8FunIndex = 1;	//A2
							pOperator->u16SpecialIndex = pOperator->u8CellIndex;
							pOperator->u8CellIndex = 0;
							OPERATOR_showCellLevel(operatorHandle);
						}
					}
					else
						OPERATOR_runEnterKey_Level4Default(operatorHandle);
					break;
				}
				case OP_MODE_PROG:
					OPERATOR_runEnterKey_Level4Default(operatorHandle);
					break;
			}
			break;
		case OP_STATUS_EDITLEVEL5:
			pOperator->OpStatus = OP_STATUS_ENTRYLEVEL6;
			DM_setValue(pOperator->DmHandle,
						pOperator->u8ModeIndex,pOperator->u8GroupIndex, pOperator->u8FunIndex,  pOperator->u8CellIndex,
					    g_u16EditValue);
			OPERATOR_showEntryLevel(operatorHandle);
			break;
	}
	OPERATOR_setupLEDs(operatorHandle);
}

void OPERATOR_runEscKey(OPERATOR_Handle operatorHandle)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	switch (pOperator->OpStatus)
	{
		case OP_STATUS_GROUPLEVEL2:
			pOperator->OpStatus = OP_STATUS_MODELEVEL1;
			OPERATOR_showModeLevel(operatorHandle);
			break;
		case OP_STATUS_FUNLEVEL3:
		{
			uint_least8_t u8AccessLevel = DM_getAccessLevel(pOperator->DmHandle);
			if (u8AccessLevel == OP_ACCESS_BASIC)
			{
				pOperator->OpStatus = OP_STATUS_MODELEVEL1;
				OPERATOR_showModeLevel(operatorHandle);

			}
			else	//OP_ACCESS_ADVANCE
			{
				pOperator->OpStatus = OP_STATUS_GROUPLEVEL2;
				OPERATOR_showGroupLevel(operatorHandle);
			}
			break;
		}
		case OP_STAUS_CELLLEVEL4:
			switch (pOperator->u8ModeIndex)
			{
				case OP_MODE_OPERATION:
				case OP_MODE_INIT:
					if (pOperator->u16SpecialIndex == 0xffff)
					{
						pOperator->OpStatus = OP_STATUS_MODELEVEL1;
						OPERATOR_showModeLevel(operatorHandle);
					}
					else
					{
						pOperator->u8CellIndex = (uint_least8_t) pOperator->u16SpecialIndex;
						pOperator->u16SpecialIndex = 0xffff;
						OPERATOR_showCellLevel(operatorHandle);
					}
					break;

				case OP_MODE_PROG:
				{

					uint_least8_t u8AccessLevel = DM_getAccessLevel(pOperator->DmHandle);
					if (u8AccessLevel == OP_ACCESS_QUICK)
					{
						pOperator->OpStatus = OP_STATUS_MODELEVEL1;
						OPERATOR_showModeLevel(operatorHandle);
					}
					else	//OP_ACCESS_BASIC & ADVANCED
					{
						pOperator->OpStatus = OP_STATUS_FUNLEVEL3;
						OPERATOR_showFunLevel(operatorHandle);
					}
					break;

				}
			}
			break;
		case OP_STATUS_EDITLEVEL5:
			pOperator->OpStatus = OP_STAUS_CELLLEVEL4;
			OPERATOR_showCellLevel(operatorHandle);
			break;
		case OP_STATUS_ENTRYLEVEL6:
			pOperator->OpStatus = OP_STATUS_EDITLEVEL5;
			OPERATOR_showCellEditLevel(operatorHandle);
			break;
	}
}

void OPERATOR_runResetKey(OPERATOR_Handle operatorHandle)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	switch (pOperator->OpStatus)
	{
		case OP_STATUS_EDITLEVEL5:	//Shift Left
			g_u16DeltaValue /= 10;
			if (g_u16DeltaValue == 0) g_u16DeltaValue = 1000;
			OPERATOR_showCellEditLevel(operatorHandle);
			break;

	}
	OPERATOR_setupLEDs(operatorHandle);
}

void OPERATOR_runLocalKey(OPERATOR_Handle operatorHandle)
{
	//OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;

	//gMotorVars.bFlag_Run_Identify = false;
	gMotorVars.bFlag_enableSys = (gMotorVars.bFlag_enableSys) ? false : true;
/*		gMotorVars.bFlag_enableSys = false;
	}
	else
	{
		gMotorVars.bFlag_enableSys = true;
	}*/

	OPERATOR_setupLEDs(operatorHandle);
}

void OPERATOR_runStartKey(OPERATOR_Handle operatorHandle)
{
	if (gMotorVars.bFlag_enableSys)
	{
		gMotorVars.bFlag_Run_Identify = true;
	}
	OPERATOR_setupLEDs(operatorHandle);

}

void OPERATOR_runStopKey(OPERATOR_Handle operatorHandle)
{
	if (gMotorVars.bFlag_enableSys)
	{
		gMotorVars.bFlag_Run_Identify = false;
	}
	OPERATOR_setupLEDs(operatorHandle);

}

/*
uint16_t OPERATOR_showRealTimeData(OPERATOR_Handle operatorHandle)
{
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	DM_Handle dmHandle = pOperator->dmHandle;
	char  ai8Line2[20];

	if (pOperator->u16SpecialIndex == 0xffff)
	{
					uint_least8_t u8ItemNo = DM_getOperModeSizeNo(pOperator->dmHandle);
					if (pOperator->u8CellIndex == u8ItemNo)
						return 0;
						//DM_outFunction(dmHandle,
						//			   OP_MODE_OPERATION, OP_GROUP_U,1,
						//			   ai8Line1, ai8Line2);
					else if (pOperator->u8CellIndex == u8ItemNo+1)
						return 0;
						//DM_outFunction(dmHandle,
						//				OP_MODE_OPERATION, OP_GROUP_U,2,
						//				ai8Line1, ai8Line2);
					else if (pOperator->u8CellIndex == u8ItemNo+2)
						return 0;
						//DM_outFunction(dmHandle,
						//			   OP_MODE_OPERATION, OP_GROUP_U,0,
						//			   ai8Line1, ai8Line2);
					else
					{

						DM_outFunCell(dmHandle,
									  OP_MODE_OPERATION, OP_GROUP_U,0,pOperator->u8CellIndex,
									  ai8Line1, ai8Line2);

					}
				}
	//OPERATOR_clearLCDAll(operatorHandle);
	//OPERATOR_setCenterLCDString(operatorHandle, 0, ai8Line1);
	OPERATOR_clearLCDLine(operatorHandle,1);
	OPERATOR_setCenterLCDString(operatorHandle, 1, ai8Line2);
	return 0;	//DATA_ConvertToCnt(1);	// 1 ms
}*/


uint16_t OPERATOR_runNoKey(OPERATOR_Handle operatorHandle)
{
	// For real-time update data
	OPERATOR_Obj *pOperator = (OPERATOR_Obj *) operatorHandle;
	OPERATOR_setupLEDs(operatorHandle);
	switch (pOperator->OpStatus)
	{
		case OP_STAUS_CELLLEVEL4:
		{
			switch (pOperator->u8ModeIndex)
			{
				case OP_MODE_OPERATION:
					if (pOperator->u16SpecialIndex == 0xffff)
					{
						uint_least8_t u8ItemNo = DM_getOperModeSizeNo(pOperator->DmHandle);
						if (pOperator->u8CellIndex == u8ItemNo)
							return 0;
											//DM_outFunction(dmHandle,
											//			   OP_MODE_OPERATION, OP_GROUP_U,1,
											//			   ai8Line1, ai8Line2);
						else if (pOperator->u8CellIndex == u8ItemNo+1)
							return 0;
											//DM_outFunction(dmHandle,
											//				OP_MODE_OPERATION, OP_GROUP_U,2,
											//				ai8Line1, ai8Line2);
					   else if (pOperator->u8CellIndex == u8ItemNo+2)
							return 0;
											//DM_outFunction(dmHandle,
											//			   OP_MODE_OPERATION, OP_GROUP_U,0,
											//			   ai8Line1, ai8Line2);
					   else
					   {

						    char  ai8Line1[20], ai8Line2[20];
							DM_outFunCell( pOperator->DmHandle,
										  OP_MODE_OPERATION, OP_GROUP_U,0,pOperator->u8CellIndex,
										  ai8Line1, ai8Line2);
							OPERATOR_clearLCDLine(operatorHandle,1);
							OPERATOR_setCenterLCDString(operatorHandle, 1, ai8Line2);
							return DATA_ConvertToCnt(KEY_DELAY_100MS);	// 100 ms

					   }
					}

			}
			break;

		}

	}

	return 0;

}

//OPERATOR_Handle		gOperHandle;
void OPERATOR_run(OPERATOR_Handle operatorHandle)
{
	static uint_least8_t u8LastKey = OPERATOR_NO_KEY;
	static uint16_t u16ContNo = 0;
	static uint16_t u16KeyDelayCnt=0;

	//OPERATOR_Handle operatorHandle = g_pOperatorObj;
	OPERATOR_Obj *pOperatorObj = operatorHandle;

	if (u16KeyDelayCnt)
		u16KeyDelayCnt--;
	else
	{
		if  (pOperatorObj->OpStatus ==OP_STATUS_ENTRYLEVEL6 )
		{
			pOperatorObj->OpStatus = OP_STAUS_CELLLEVEL4;
			OPERATOR_showCellLevel(operatorHandle);
			u16KeyDelayCnt = DATA_ConvertToCnt(KEY_DELAY_500MS);	// 500 ms
			u8LastKey = OPERATOR_NO_KEY;
			u16ContNo = 0;
		}
		else if (OPERATOR_isKey(operatorHandle, OPERATOR_UP_KEY))
		{
			OPERATOR_runUpKey(operatorHandle);
			if (u8LastKey == OPERATOR_UP_KEY)
				u16ContNo++;
			if (u16ContNo > 15)
				u16KeyDelayCnt = DATA_ConvertToCnt(KEY_DELAY_100MS);
			else if (u16ContNo > 8)
				u16KeyDelayCnt = DATA_ConvertToCnt(KEY_DELAY_200MS);
			else
			    u16KeyDelayCnt = DATA_ConvertToCnt(KEY_DELAY_500MS);	// 100 ms
			u8LastKey = OPERATOR_UP_KEY;
		}
		else if (OPERATOR_isKey(operatorHandle, OPERATOR_DOWN_KEY))
		{
			OPERATOR_runDownKey(operatorHandle);
			if (u8LastKey == OPERATOR_DOWN_KEY)
				u16ContNo++;
			if (u16ContNo > 15)
				u16KeyDelayCnt = DATA_ConvertToCnt(KEY_DELAY_100MS);
			else if (u16ContNo > 8)
				u16KeyDelayCnt = DATA_ConvertToCnt(KEY_DELAY_200MS);
			else
				 u16KeyDelayCnt = DATA_ConvertToCnt(KEY_DELAY_500MS);	// 500 ms
			u8LastKey = OPERATOR_DOWN_KEY;
		}
		else if (OPERATOR_isKey(operatorHandle, OPERATOR_ENTER_KEY))
		{
			OPERATOR_runEnterKey(operatorHandle);
			u16KeyDelayCnt = DATA_ConvertToCnt(KEY_DELAY_500MS);	// 500 ms
			u8LastKey = OPERATOR_NO_KEY;
			u16ContNo = 0;
		}
		else if (OPERATOR_isKey(operatorHandle, OPERATOR_ESC_KEY))
		{
			OPERATOR_runEscKey(operatorHandle);
			u16KeyDelayCnt = DATA_ConvertToCnt(KEY_DELAY_500MS);	// 500 ms
			u8LastKey = OPERATOR_NO_KEY;
			u16ContNo = 0;
		}
		else if (OPERATOR_isKey(operatorHandle, OPERATOR_RESET_KEY))
		{
			OPERATOR_runResetKey(operatorHandle);
			u16KeyDelayCnt = DATA_ConvertToCnt(KEY_DELAY_500MS);	// 500 ms
			u8LastKey = OPERATOR_NO_KEY;
			u16ContNo = 0;
		}
		else if (OPERATOR_isKey(operatorHandle, OPERATOR_LOCAL_KEY))
		{
			OPERATOR_runLocalKey(operatorHandle);
			u16KeyDelayCnt = DATA_ConvertToCnt(KEY_DELAY_500MS);	// 500 ms
			u8LastKey = OPERATOR_NO_KEY;
			u16ContNo = 0;
		}
		else if (OPERATOR_isKey(operatorHandle, OPERATOR_STOP_KEY))
		{
			OPERATOR_runStopKey(operatorHandle);
			u16KeyDelayCnt = DATA_ConvertToCnt(KEY_DELAY_500MS);	// 500 ms
			u8LastKey = OPERATOR_NO_KEY;
			u16ContNo = 0;
		}
		else if (OPERATOR_isKey(operatorHandle, OPERATOR_RUN_KEY))
		{
			OPERATOR_runStartKey(operatorHandle);
			u16KeyDelayCnt = DATA_ConvertToCnt(KEY_DELAY_500MS);	// 500 ms
			u8LastKey = OPERATOR_NO_KEY;
			u16ContNo = 0;
		}
		else if (OPERATOR_isKey(operatorHandle, OPERATOR_JOG_KEY))
		{

		}
		else if (OPERATOR_isKey(operatorHandle, OPERATOR_MENU_KEY))
		{
			OPERATOR_runMenuKey(operatorHandle);
			u16KeyDelayCnt = DATA_ConvertToCnt(KEY_DELAY_500MS);	// 500 ms
			u8LastKey = OPERATOR_NO_KEY;
			u16ContNo = 0;
		}
		else if (OPERATOR_isKey(operatorHandle, OPERATOR_FWR_KEY))
		{

		}
		else
		{
			u16KeyDelayCnt = OPERATOR_runNoKey(operatorHandle);
			u8LastKey = OPERATOR_NO_KEY;
			u16ContNo = 0;
		}
	}

}


//g_pOperatorObj


//------------------------------------------------------------------------
//
//------------------------------------------------------------------------


void OutInit1(SCIMessage_Handle sciMessageHandle)
{
	const uint_least8_t au8OutData[] =
			{0x00, 0x70, 0x00, 0x08, 0x20, 0x56, 0x55, 0x56,
	         0x78, 0x20, 0x54, 0x56, 0x55, 0x78, 0x20, 0x55,
	         0x55, 0x55, 0x78, 0x38, 0x56, 0x55, 0x56, 0x18,
	         0x38, 0x54, 0x56, 0x55, 0x18, 0x00, 0x48, 0x7a,
	         0x41, 0x00, 0x38, 0x45, 0xfd
			};
	 //uint_least8_t u8TotNo = 37;
	g_pOperatorObj-> OpCommStatus = OPERATOR_INIT2;
	SCIMessage_TxWrite(sciMessageHandle,37, au8OutData, OPERATOR_callback);

/*	uint_least8_t i;
	for (i=0; i<u8TotNo; i++)
		pSciMessage->au8TxBuffer[i] = au8OutData[i];
	pSciMessage->u8TxNumOfBytes = u8TotNo;
	pSciMessage->u8TxIndex = 0;

	SCI_enableTxFifoInt(pSciMessage->sciHandle);*/


}

void OutInit2(SCIMessage_Handle sciMessageHandle)
//void OutInit2(SCI_Message *pSciMessage)
{
	const uint_least8_t au8OutData[] =
			{0x46, 0x45, 0x38, 0x44, 0x44, 0x7e, 0x45, 0x44,
			 0x3c, 0x40, 0x42, 0x41, 0x3c, 0x3c, 0x40, 0x42,
			 0x21, 0x7c, 0x3c, 0x41, 0x40, 0x41, 0x3c, 0x20,
			 0x55, 0x56, 0x54, 0x78, 0x38, 0x55, 0x56, 0x54,
			 0x18, 0x38, 0x46, 0x45, 0x46, 0x38, 0x73, 0xdd,
			 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
			};
	//const uint_least8_t u8TotNo = 46;
	g_pOperatorObj-> OpCommStatus = OPERATOR_START;
	SCIMessage_TxWrite(sciMessageHandle,46, au8OutData, NULL);

	/*
	uint_least8_t i;
	for (i=0; i<u8TotNo; i++)
		pSciMessage->au8TxBuffer[i] = au8OutData[i];
	pSciMessage->u8TxNumOfBytes = u8TotNo;
	pSciMessage->u8TxIndex = 0;
	SCI_enableTxFifoInt(pSciMessage->sciHandle);

	g_pOperatorObj-> OpCommStatus = OPERATOR_START;*/
}

void OutNormal(SCIMessage_Handle sciMessageHandle)
{
	uint_least8_t i;
	uint16_t	u16CRC;
	SCI_Message *pSciMessage = (SCI_Message *) sciMessageHandle;

	g_pOperatorObj->u16Key= DATA_MakeWord(pSciMessage->au8RxBuffer[2], pSciMessage->au8RxBuffer[3]);

	pSciMessage->au8TxBuffer[0] = 0;
	pSciMessage->au8TxBuffer[1] = 0x71;
	pSciMessage->au8TxBuffer[2] = g_pOperatorObj->u8LEDBlink;
	pSciMessage->au8TxBuffer[3] = g_pOperatorObj->u8LEDOnOff;

	pSciMessage->au8TxBuffer[4] = 0;
	pSciMessage->au8TxBuffer[5] = 0;
	pSciMessage->au8TxBuffer[6] = 0;
	pSciMessage->au8TxBuffer[7] = 0;

	pSciMessage->au8TxBuffer[8] = DATA_HiByte(g_pOperatorObj->u16LCDBlink[0]);
	pSciMessage->au8TxBuffer[9] = DATA_LoByte(g_pOperatorObj->u16LCDBlink[0]);
	pSciMessage->au8TxBuffer[10] = DATA_HiByte(g_pOperatorObj->u16LCDBlink[1]);
	pSciMessage->au8TxBuffer[11] = DATA_LoByte(g_pOperatorObj->u16LCDBlink[1]);

	for (i=0; i<16; i++)
		pSciMessage->au8TxBuffer[12+i] = (uint_least8_t)  g_pOperatorObj-> ai8LCD[0][i];

	for (i=0; i<16; i++)
		pSciMessage->au8TxBuffer[28+i] = (uint_least8_t) g_pOperatorObj-> ai8LCD[1][i];

	u16CRC = DATA_CalCRC16(pSciMessage->au8TxBuffer,44);
	pSciMessage->au8TxBuffer[44] = DATA_LoByte(u16CRC) ;
	pSciMessage->au8TxBuffer[45] = DATA_HiByte(u16CRC) ;

	SCIMessage_TxWrite0(sciMessageHandle, 46, NULL);

/*	pSciMessage->u8TxNumOfBytes = 46;
	pSciMessage->u8TxIndex = 0;
	SCI_enableTxFifoInt(pSciMessage->sciHandle);*/

}
//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
void OPERATOR_callback(SCIMessage_Handle sciMessageHandle)
{
	SCI_Message *pSciMessage = (SCI_Message *) sciMessageHandle;
	if (pSciMessage->au8RxBuffer[1] == 0x70)
		OutInit1(sciMessageHandle);
	else
	{
		if (g_pOperatorObj-> OpCommStatus == OPERATOR_START)
			OutNormal(pSciMessage);
		else
			OutInit2(pSciMessage);
	}

}



// end of file
