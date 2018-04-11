/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, LineStream Technologies Incorporated
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
* *  Neither the names of Texas Instruments Incorporated, LineStream
 *    Technologies Incorporated, nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
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
//! \file   solutions/instaspin_motion/src/proj_lab12b.c
//! \brief  SpinTAC Velocity Controller using a quadrature encoder for feedback
//!
//! (C) Copyright 2012, LineStream Technologies, Inc.
//! (C) Copyright 2011, Texas Instruments, Inc.

//! \defgroup PROJ_LAB12b PROJ_LAB12b
//@{

//! \defgroup PROJ_LAB12b_OVERVIEW Project Overview
//!
//! SpinTAC Velocity Controller using a quadrature encoder for feedback

//$ the comments added behind '//$' was written by ting wei chien


// **************************************************************************
// the includes
#include <math.h>
#include <stdio.h>

#include "main.h"
#include "user_Data.h"

#ifdef FLASH
   #pragma CODE_SECTION(adcISR,"ramfuncs");
#endif

extern void setupFilterCoeffs(FILTER_FO_Handle filterHandle, float fPole_rps);

// **************************************************************************
// defines

//#define LED_BLINK_FREQ_Hz   5
//#define PWM_TEST

#define ENCODER_MODULE

#define HAL_Gpio_Relay 			GPIO_Number_58
#define HAL_Gpio_DriveEnable 	GPIO_Number_24


// **************************************************************************
// the globals


uint_least16_t gu16Counter_updateGlobals = 0;

bool bFlag_Latch_softwareUpdate = true;

USER_Params gUserParams;

HAL_PwmData_t gPwmData = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};    //HAL_PwmData_t gPwmData = {_IQ(-0.8), _IQ(0.0), _IQ(0.8)};

HAL_AdcData_t gAdcData;

_iq giqMaxCurrentSlope = _IQ(0.0);

#ifdef FAST_ROM_V1p6        //$The F2806xF/M devices have v1p6 of the ROM. This is the only version they have or will have.
CTRL_Obj *gpcontroller_obj;
#else
CTRL_Obj ctrl;				//v1p7 format
#endif

CTRL_Handle 		gctrlHandle;
HAL_Handle 			ghalHandle;

I2CMessage_Handle 	gi2cMessageHandle;
SCIMessage_Handle 	gsciMessageHandle[2];

ENC_Handle 			gencHandle;
EEPROM_Handle 		gEepromHandle;
IOEXPAND_Handle 	gIoexpandHandle;
LCD_Handle			gLcdHandle;
OPERATOR_Handle		gOperHandle;
MODBUS_Handle		gModbusHandle;



I2C_Message 		gi2cMessage;
SCI_Message			gsciMessage[2];

ENC_Obj 			gencObj;
EEPROM24LC32_Obj 	gEepromObj;
IOEXPAND23017_Obj 	gIoexpandObj;
LCD1602_Obj			gLcdObj;
OPERATOR_Obj		gOperObj;
MODBUS_Obj			gModbusObj;


FILTER_FO_Handle  gFilterDCBusHandle, gFilterOutFreqHandle, gFilterOutEncoderHandle, gFilterOutCurrentHandle;
FILTER_FO_Obj     gFilterDCBus, gFilterOutFreq, gFilterOutEncoder, gFilterOutCurrent;

//SLIP_Handle slipHandle;
//SLIP_Obj slip;

//ST_Obj st_obj;
//ST_Handle stHandle;

//uint16_t gLEDcnt = 0;

volatile MOTOR_Vars_t gMotorVars = MOTOR_Vars_INIT;//$the initial value that you can see in the watch windows at the beginning
//bool bStartAligned = false;			// Start to Alignment for Encoder

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;
#endif

#ifdef DRV8301_SPI
// Watch window interface to the 8301 SPI
DRV_SPI_8301_Vars_t gDrvSpi8301Vars;
#endif


//$ for plot the datalog
HAL_DacData_t gDacData;



_iq giqFlux_pu_to_Wb_sf;
_iq giqFlux_pu_to_VpHz_sf;
_iq giqTorque_Ls_Id_Iq_pu_to_Nm_sf;
_iq giqTorque_Flux_Iq_pu_to_Nm_sf;


//***********************************************************************************************************************************
//***********************************************************************************************************************************
//***********************************************************************************************************************************
//***********************************************************************************************************************************
//*s*********************************************************************************************************************************
// the main functions

void main(void)
{
	//uint_least8_t u8estNumber = 0;

//#ifdef FAST_ROM_V1p6
	//uint_least8_t u8ctrlNumber = 0;
//#endif

	// Only used if running from FLASH
	// Note that the variable FLASH is defined by the project
#ifdef FLASH
	// Copy time critical code and Flash setup code to RAM
	// The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
	// symbols are created by the linker. Refer to the linker files.
	memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,(uint16_t *)&RamfuncsRunStart);
#endif

	// initialize the hardware abstraction layer
	ghalHandle = HAL_init(&ghal,sizeof(ghal));//$HAL_Obj ghal;<<--global variable

	// initialize the ENC module
	gencHandle = ENC_init(&gencObj, sizeof(gencObj));


	gi2cMessageHandle= I2CMessage_init(&gi2cMessage, sizeof(gi2cMessage));
	gsciMessageHandle[0] = SCIMessage_init(&gsciMessage[0], sizeof(gsciMessage[0]));
	gsciMessageHandle[1] = SCIMessage_init(&gsciMessage[1], sizeof(gsciMessage[1]));
	gOperHandle = OPERATOR_init(&gOperObj, sizeof(gOperObj));
	gModbusHandle = MODBUS_init(&gModbusObj, sizeof(gModbusObj));
 	gEepromHandle = EEPROM_init(&gEepromObj, sizeof(gEepromObj));
  	gIoexpandHandle = IOEXPAND_init(&gIoexpandObj, sizeof(gIoexpandObj));
  	//gLcdHandle =LCD_init(&gLcdObj, sizeof(gLcdObj));
	gdmHandle = DM_init(&gdmObj,sizeof(gdmObj));

	gFilterDCBusHandle = FILTER_FO_init(&gFilterDCBus, sizeof(gFilterDCBus));
	gFilterOutFreqHandle = FILTER_FO_init(&gFilterOutFreq, sizeof(gFilterOutFreq));
	gFilterOutEncoderHandle = FILTER_FO_init(&gFilterOutEncoder, sizeof(gFilterOutEncoder));
	gFilterOutCurrentHandle = FILTER_FO_init(&gFilterOutCurrent, sizeof(gFilterOutCurrent));

	setupFilterCoeffs(gFilterDCBusHandle, 10.0f);		//10 Hz
	setupFilterCoeffs(gFilterOutFreqHandle, 10.0f);		//10 Hz
	setupFilterCoeffs(gFilterOutEncoderHandle, 10.0f);	//10 Hz
	setupFilterCoeffs(gFilterOutCurrentHandle, 10.0f);	//10 Hz


	// check for errors in user parameters
	USER_checkForErrors(&gUserParams);

	// store user parameter error in global variable
	gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);

	// do not allow code execution if there is a user parameter error
	if(gMotorVars.UserErrorCode != USER_ErrorCode_NoError)//$USER_ErrorCode_NoError=0,if this equal true,means there is an error occurred
    {
		for(;;)
        {
			gMotorVars.bFlag_enableSys = false;//$ if ErrorCodes occurred ,the enable Flag will automatically reset to 0 just after you set it to 1 and push the resume button
        }
    }

	// initialize the user parameters
	USER_setParams(&gUserParams);//$put the motor parameters,inverter parameters ...in user.h into the gUserParams

	// set the hardware abstraction layer parameters
	HAL_setParams(ghalHandle,&gUserParams);

	// initialize the controller
#ifdef FAST_ROM_V1p6    //$?
	//gctrlHandle = CTRL_initCtrl(u8ctrlNumber, u8estNumber);  		//v1p6 format (06xF and 06xM devices) Org
	gctrlHandle = CTRL_initCtrl(0, 0);  //$?
	gpcontroller_obj = (CTRL_Obj *)gctrlHandle; //$CTRL_Obj *gpcontroller_obj;
#else
	ctrlHandle = CTRL_initCtrl(estNumber,&ctrl,sizeof(ctrl));	//v1p7 format default
#endif

	{
		CTRL_Version version;
		// get the version number
		CTRL_getVersion(gctrlHandle,&version);
		gMotorVars.CtrlVersion = version;

		//CTRL_getVersion(gctrlHandle,&gMotorVars.CtrlVersion);
	}



	// set the default controller parameters
	CTRL_setParams(gctrlHandle,&gUserParams);//$$$$$$$$$$$$$$$$$$$$$$$$$$



	// setup the ENC(encoder) module
	ENC_setup(gencHandle, 100, USER_MOTOR_NUM_POLE_PAIRS, USER_MOTOR_ENCODER_LINES, 0, USER_IQ_FULL_SCALE_FREQ_Hz, USER_ISR_FREQ_Hz, 8000.0); //Org
	I2CMessage_setup(gi2cMessageHandle,((HAL_Obj *)ghalHandle)->i2cHandle);
  	SCIMessage_setup(gsciMessageHandle[0], ((HAL_Obj *)ghalHandle)->sciHandle[0]);
  	SCIMessage_setup(gsciMessageHandle[1], ((HAL_Obj *)ghalHandle)->sciHandle[1]);
	OPERATOR_setup(gOperHandle, gdmHandle);
  	MODBUS_setup(gModbusHandle);
	EEPROM_setup(gEepromHandle,  ((HAL_Obj *)ghalHandle)->gpioHandle, gi2cMessageHandle);



    // set DAC parameters
    gDacData.ptrData[0] = &gPwmData.Tabc.aiqvalue[0];
    gDacData.ptrData[1] = &gPwmData.Tabc.aiqvalue[1];
    gDacData.ptrData[2] = &gPwmData.Tabc.aiqvalue[2];
    gDacData.ptrData[3] = &gAdcData.V.aiqvalue[0];


	/*
	gDacData.ptrData[0] = &angle_gen.Angle_pu;
	gDacData.ptrData[1] = &gAdcData.I.value[0];
	gDacData.ptrData[2] = &gPwmData.Tabc.value[0];
	gDacData.ptrData[3] = &gAdcData.V.value[0];
	*/


	HAL_setDacParameters(ghalHandle, &gDacData);

	// Initialize Datalog
	datalogHandle = DATALOG_init(&datalog,sizeof(datalog));

	// Connect inputs of the datalog module
	datalog.iptr[0] = &gPwmData.Tabc.aiqvalue[0];      // datalogBuff[0]
	datalog.iptr[1] = &gPwmData.Tabc.aiqvalue[1];      // datalogBuff[1]
	datalog.iptr[2] = &gPwmData.Tabc.aiqvalue[2];      // datalogBuff[2]


	/*
	datalog.iptr[0] = &angle_gen.Angle_pu;        // datalogBuff[0]
	datalog.iptr[1] = &gAdcData.I.value[0];       // datalogBuff[1]
	datalog.iptr[2] = &gAdcData.V.value[0];       // datalogBuff[2]
	*/

	datalog.Flag_EnableLogData = true;
	datalog.Flag_EnableLogOneShot = false;

//////////////////////////////////////////////////////////////////////////////////////


	// setup faults
	HAL_setupFaults(ghalHandle);

	// initialize the interrupt vector table
	HAL_initIntVectorTable(ghalHandle);//$

	// enable the ADC interrupts
	HAL_enableAdcInts(ghalHandle);//$


	HAL_enableSciAInt(ghalHandle);
  	HAL_enableSciBInt(ghalHandle);
  	HAL_enableI2cInt(ghalHandle);

  	// enable global interrupts
	HAL_enableGlobalInts(ghalHandle);

	// enable debug interrupts
	HAL_enableDebugInt(ghalHandle);

	// disable the PWM
	HAL_disablePwm(ghalHandle);

//	IOEXPAND_setup(gIoexpandHandle,gi2cMessageHandle, &g_u16InputTerms, &g_u8OutputTerms);
//	LCD_setup(gLcdHandle, gi2cMessageHandle);		//I2C interrupt must turn on
	DM_setup(gdmHandle,gEepromHandle );

	HAL_enableTimer0Int(ghalHandle);
	HAL_enableTimer1Int(ghalHandle);



/* (Org)
  // initialize the SLIP module
  slipHandle = SLIP_init(&slip, sizeof(slip));


  // setup the SLIP module
  SLIP_setup(slipHandle, _IQ(gUserParams.ctrlPeriod_sec));


  // initialize the SpinTAC Components
  stHandle = ST_init(&st_obj, sizeof(st_obj));
  
  
  // setup the SpinTAC Components
  ST_setupVelCtl(stHandle);
  ST_setupPosConv(stHandle);
*/

	HAL_setGpioHigh(ghalHandle, (GPIO_Number_e)HAL_Gpio_Relay);	//  Adding Turn on standby relay
	HAL_setGpioLow(ghalHandle, (GPIO_Number_e)HAL_Gpio_DriveEnable);


#ifdef DRV8301_SPI
	// turn on the DRV8301 if present
	HAL_enableDrv(ghalHandle);
	// initialize the DRV8301 interface
 	HAL_setupDrvSpi(ghalHandle,&gDrvSpi8301Vars);
#endif

 	// enable DC bus compensation
 	CTRL_setFlag_enableDcBusComp(gctrlHandle, true);

 	// compute scaling factors for flux and torque calculations
 	giqFlux_pu_to_Wb_sf = USER_computeFlux_pu_to_Wb_sf();
 	giqFlux_pu_to_VpHz_sf = USER_computeFlux_pu_to_VpHz_sf();
 	giqTorque_Ls_Id_Iq_pu_to_Nm_sf = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf();
 	giqTorque_Flux_Iq_pu_to_Nm_sf = USER_computeTorque_Flux_Iq_pu_to_Nm_sf();

#ifdef PWM_TEST
 	HAL_enablePwm(ghalHandle);
 	for (; ;)
 	{

 	}

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////Forever Loop/////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 	for(;;)
 	{
 		/*uint16_t i;
 		for (i=0;i<1000; i++)
 		{
 			if (EEPROM_WriteVerify(gEepromHandle, i, i+100)== false)
 			{
 				for (;;)
 				{
 					;
 				}
 			}
 		}*/
 		//gMotorVars.bFlag_Aligned = false;
 		//gMotorVars.bFlag_enableRsRecalc = gMotorVars.bFlag_enableOffsetcalc = true;
 		//gMotorVars.iqSpeedRef_krpm = _IQ(0.0);

 		while(!(gMotorVars.bFlag_enableSys));//$when bFlag_enableSys=0,it'll be stucked here until an interrupt reseting it to 1

 		//$this is a great debug method only enable this flag ,but don't enable run identify flag
 		//$you'll see the output signal from the MCU will be the square wave
 		HAL_enablePwm(ghalHandle);



 		//$$
 		//CTRL_setFlag_enableSpeedCtrl(gctrlHandle, false);  //$the torque(current) mode//CTRL_setFlag_enableSpeedCtrl(ctrlHandle, false);	//Org
 		CTRL_setFlag_enableSpeedCtrl(gctrlHandle, true);	//Modify//$the speed mode




 		while(gMotorVars.bFlag_enableSys)//$the loop while the enable system flag is true
 		{
 			CTRL_Obj *pobj = (CTRL_Obj *)gctrlHandle;
 			//ST_Obj *stObj = (ST_Obj *)stHandle;

            gu16Counter_updateGlobals++;

            // enable/disable the use of motor parameters being loaded from user.h
            CTRL_setFlag_enableUserMotorParams(gctrlHandle,gMotorVars.bFlag_enableUserParams);

            // enable/disable Rs recalibration during motor startup
            EST_setFlag_enableRsRecalc(pobj->estHandle,gMotorVars.bFlag_enableRsRecalc);

            // enable/disable automatic calculation of bias values
            CTRL_setFlag_enableOffset(gctrlHandle,gMotorVars.bFlag_enableOffsetcalc);


 			if(CTRL_isError(gctrlHandle))
 			{
 				CTRL_setFlag_enableCtrl(gctrlHandle,false);
 				gMotorVars.bFlag_enableSys = false;
 				HAL_disablePwm(ghalHandle);
 			}
 			else//$no error in ctrl block
 			{

 			   //$ update the controller state,including:"CTRL_setState ,EST_updateState" this two function
  			   bool bflag_ctrlStateChanged = CTRL_updateState(gctrlHandle);

 			   //$Sets the enable controller flag value in the estimator
 			   CTRL_setFlag_enableCtrl(gctrlHandle, gMotorVars.bFlag_Run_Identify);

               //$if the ctrl state change
 			   if(bflag_ctrlStateChanged)
 			   {
 					CTRL_State_e ctrlState = CTRL_getState(gctrlHandle);//$the ctrlState=the changed state now
 					if(ctrlState == CTRL_State_OffLine)                 //$ ctrl execute first
 					{
 						HAL_enablePwm(ghalHandle);//$  ???
 					}
 					else if(ctrlState == CTRL_State_OnLine)
 					{
 						if(gMotorVars.bFlag_enableOffsetcalc == true)
 						{
 							HAL_updateAdcBias(ghalHandle);//$ get bias
 						}
 						else//$ offset disable-->use the offset value store in the user.h
 						{
 							// set the current bias
 							HAL_setBias(ghalHandle,HAL_SensorType_Current,0,_IQ(I_A_offset));//$direct use the define value (include main.c--->main.h-->user.h)
 							HAL_setBias(ghalHandle,HAL_SensorType_Current,1,_IQ(I_B_offset));
 							HAL_setBias(ghalHandle,HAL_SensorType_Current,2,_IQ(I_C_offset));

 							// set the voltage bias
 							HAL_setBias(ghalHandle,HAL_SensorType_Voltage,0,_IQ(V_A_offset));
 							HAL_setBias(ghalHandle,HAL_SensorType_Voltage,1,_IQ(V_B_offset));
 							HAL_setBias(ghalHandle,HAL_SensorType_Voltage,2,_IQ(V_C_offset));
 						}

 						// Return the bias value for currents
 						gMotorVars.I_bias.aiqvalue[0] = HAL_getBias(ghalHandle,HAL_SensorType_Current,0);
 						gMotorVars.I_bias.aiqvalue[1] = HAL_getBias(ghalHandle,HAL_SensorType_Current,1);
 						gMotorVars.I_bias.aiqvalue[2] = HAL_getBias(ghalHandle,HAL_SensorType_Current,2);

 						// Return the bias value for voltages
 						gMotorVars.V_bias.aiqvalue[0] = HAL_getBias(ghalHandle,HAL_SensorType_Voltage,0);
 						gMotorVars.V_bias.aiqvalue[1] = HAL_getBias(ghalHandle,HAL_SensorType_Voltage,1);
 						gMotorVars.V_bias.aiqvalue[2] = HAL_getBias(ghalHandle,HAL_SensorType_Voltage,2);

 						//gMotorVars.bFlag_enableOffsetcalc = false;		// Disable Offset Calc
 						HAL_enablePwm(ghalHandle);
 					}
 					else if(ctrlState == CTRL_State_Idle)//$the state waiting for the input signal,just standby
 					{
 						HAL_disablePwm(ghalHandle);
 						gMotorVars.bFlag_Run_Identify = false;
 					}

 					if((CTRL_getFlag_enableUserMotorParams(gctrlHandle) == true) &&
 							                        (ctrlState > CTRL_State_Idle) &&
 							                        (gMotorVars.CtrlVersion.minor == 6))
 					{
 						//$ call this function to fix 1p6,since there are some MCU hardware problems,using this software to fix
 						USER_softwareUpdate1p6(gctrlHandle);
 					}

 				}//$if the ctrl state change
 			}//$ctrl no error


 			if(EST_isMotorIdentified(pobj->estHandle))  //$true-->identified
 			{
 				EST_setMaxCurrentSlope_pu(pobj->estHandle,giqMaxCurrentSlope);
 				gMotorVars.bFlag_MotorIdentified = true;


 				//$$$
 				// set the speed reference
 				CTRL_setSpd_ref_krpm(gctrlHandle,gMotorVars.iqSpeedRef_krpm);   //$change speed

 				//set the speed acceleration
 				CTRL_setMaxAccel_pu(gctrlHandle,_IQmpy(MAX_ACCEL_KRPMPS_SF,gMotorVars.iqMaxAccel_krpmps));  //$change acceleration





 				/* Org
            		// enable the SpinTAC Speed Controller
            		STVELCTL_setEnable(stObj->velCtlHandle, true);

            		if(EST_getState(obj->estHandle) != EST_State_OnLine)
            		{
            			// if the estimator is not running, place SpinTAC into reset
            			STVELCTL_setEnable(stObj->velCtlHandle, false);
            		}*/

 				if (bFlag_Latch_softwareUpdate)
 				{
 					bFlag_Latch_softwareUpdate = false;

 					USER_calcPIgains(gctrlHandle);

 					// initialize the watch window kp and ki current values with pre-calculated values
 					gMotorVars.iqKp_Idq = CTRL_getKp(gctrlHandle,CTRL_Type_PID_Id);
 					gMotorVars.iqKi_Idq = CTRL_getKi(gctrlHandle,CTRL_Type_PID_Id);

 					gMotorVars.iqKp_spd = CTRL_getKp(gctrlHandle,CTRL_Type_PID_spd);
 					gMotorVars.iqKi_spd = CTRL_getKi(gctrlHandle,CTRL_Type_PID_spd);

 					/* Org
			  	  	  // initialize the watch window Bw value with the default value
              	  	  gMotorVars.SpinTAC.VelCtlBw_radps = STVELCTL_getBandwidth_radps(stObj->velCtlHandle);

              	  	  // initialize the watch window with maximum and minimum Iq reference
              	  	  gMotorVars.SpinTAC.VelCtlOutputMax_A = _IQmpy(STVELCTL_getOutputMaximum(stObj->velCtlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
              	  	  gMotorVars.SpinTAC.VelCtlOutputMin_A = _IQmpy(STVELCTL_getOutputMinimum(stObj->velCtlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
 					 */
 				}
 			}
 			else
 			{
 				bFlag_Latch_softwareUpdate = true;
 				giqMaxCurrentSlope = EST_getMaxCurrentSlope_pu(pobj->estHandle);
 			}

 			// when appropriate, update the global variables
 			if(gu16Counter_updateGlobals >= NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE)
 			{
 				gu16Counter_updateGlobals = 0;
 				updateGlobalVariables_motor(gctrlHandle);	//, stHandle);
 			}

 			// update Kp and Ki gains
 			updateKpKiGains(gctrlHandle);

 			/* Org
        	// set the SpinTAC (ST) bandwidth scale
        	STVELCTL_setBandwidth_radps(stObj->velCtlHandle, gMotorVars.SpinTAC.VelCtlBw_radps);

        	// set the maximum and minimum values for Iq reference
        	STVELCTL_setOutputMaximums(stObj->velCtlHandle, _IQmpy(gMotorVars.SpinTAC.VelCtlOutputMax_A, _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A)), _IQmpy(gMotorVars.SpinTAC.VelCtlOutputMin_A, _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A)));
		   */

 			// enable/disable the forced angle
 			EST_setFlag_enableForceAngle(pobj->estHandle,gMotorVars.bFlag_enableForceAngle);

 			// enable or disable power warp
 			CTRL_setFlag_enablePowerWarp(gctrlHandle,gMotorVars.bFlag_enablePowerWarp);

#ifdef DRV8301_SPI
 			HAL_writeDrvData(ghalHandle,&gDrvSpi8301Vars);

 			HAL_readDrvData(ghalHandle,&gDrvSpi8301Vars);
#endif

 		} // end of while(gFlag_enableSys) loop


 		HAL_disablePwm(ghalHandle);
 		CTRL_setParams(gctrlHandle,&gUserParams);
 		//gMotorVars.bFlag_Run_Identify = false;
	
    /* Org
    // setup the SpinTAC Components
    ST_setupVelCtl(stHandle);
    ST_setupPosConv(stHandle); */

 	} // end of for(;;) loop

} // end of main() function





////////////////////////////////////////////////////////////////////////////////////////////////////
//$//////////////////////////////////////////////////////////////////////////////////////////////$//
//$                                                                                            //$//
//$     *-*-*-*-*-*-*-*                    *-*-*-*-*                      |-*-*-*-*-*          //$//
//$            |                        -*-                               *         |          //$//
//$            *                      -*-                                 |         *          //$//
//$            |                       -*-                                *         |          //$//
//$            *                         -*-                              |-*-*-*-*-*          //$//
//$            |                           -*-                            *-*-                 //$//
//$            *                             -*-                          |  -*-               //$//
//$            |                                -*-                       *    -*-             //$//
//$            *                               -*-                        |      -*-           //$//
//$     *-*-*-*-*-*-*-*              -*-*-*-*-*-*                         *        -*-         //$//
//$                                                                                            //$//
//$//////////////////////////////////////////////////////////////////////////////////////////////$//
////////////////////////////////////////////////////////////////////////////////////////////////////


interrupt void adcInt1ISR(void)	// MainISR//$original:mainISR
{

	/* static uint16_t stCnt = 0;*/
	CTRL_Obj *pobj = (CTRL_Obj *)gctrlHandle;

#ifdef PWM_TEST
	HAL_readAdcData(ghalHandle,&gAdcData);
	HAL_writePwmData(ghalHandle,&gPwmData);	// write the PWM compare values
	HAL_acqAdcInt(ghalHandle,ADC_IntNumber_1);
	return
#endif

	//$check how many times or when this mainISR will occur
	static uint32_t    u32mainISRcnt=0;
	u32mainISRcnt++;


	// compute the electrical angle
	/*ENC_calcElecAngle(gencHandle, HAL_getQepPosnCounts(ghalHandle));*/	//Org
	ENC_run(gencHandle, HAL_getQepPosnCounts(ghalHandle));
	HAL_readAdcData(ghalHandle,&gAdcData);

	FILTER_FO_run(gFilterDCBusHandle,gAdcData.iqdcBus);	//EST_getDcBus_pu(gpcontroller_obj->estHandle));
	FILTER_FO_run(gFilterOutFreqHandle, EST_getSpeed_krpm(gpcontroller_obj->estHandle));
	FILTER_FO_run(gFilterOutEncoderHandle, ENC_getSpeedKRPM(gencHandle));
	FILTER_FO_run(gFilterOutCurrentHandle, gpcontroller_obj->pidHandle_Iq->iqfbackValue);


	 /* gpcontroller_obj->pidHandle_Iq->iqfbackValue*/
	 /* EST_getSpeed_krpm(pobj->estHandle);*/
	 /*gMotorVars.iqSpeedQEP_krpm = ENC_getSpeedKRPM(gencHandle);*/
	 /*setupFilterCoeffs(gFilterDCBusHandle, 10.0f);*/		//20 Hz
	 /*setupFilterCoeffs(gFilerOutFreqHandle, 10.0f);*/		//100 Hz
	 /*setupFilterCoeffs(gFilerOutEncoderHandle, 10.0f);*/		//100 Hz


	/*    //Org
	// Run the SpinTAC Components
	if(stCnt++ >= ISR_TICKS_PER_SPINTAC_TICK)
	{
		ST_runPosConv(stHandle, encHandle, ctrlHandle);
		ST_runVelCtl(stHandle, ctrlHandle);
		stCnt = 1;
	 }
	*/

	if(USER_MOTOR_TYPE == MOTOR_Type_Induction)
	{
		 /*  Org
		// update the electrical angle for the SLIP module
		SLIP_setElectricalAngle(slipHandle, ENC_getElecAngle(encHandle));
		// compute the amount of slip
		SLIP_run(slipHandle);


		// run the controller
		CTRL_run(ctrlHandle,halHandle,&gAdcData,&gPwmData,SLIP_getMagneticAngle(slipHandle));
		*/
	}
	else
	{

	/*	if (g_u8CtlMethod == CTL_FOC_w_Encoder1)
	//		CTRL_run(gctrlHandle,ghalHandle,&gAdcData,&gPwmData,ENC_getElecAnglePu(gencHandle));	//Org
	//	else
	*/






	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////  all of the blocks in control diagram are inside this function  ///////////////////////////////

	CTRL_run(gctrlHandle,ghalHandle,&gAdcData,&gPwmData,EST_getAngle_pu(gctrlHandle->estHandle));// run the controller

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////





	}

    // Writes PWM data to the PWM comparators for motor control
	HAL_writePwmData(ghalHandle,&gPwmData);

	// setup the controller
	CTRL_setup(gctrlHandle);

	// if we are forcing alignment, using the Rs Recalculation, align the eQEP angle with the rotor angle
	if((EST_getState(pobj->estHandle) == EST_State_Rs) && (USER_MOTOR_TYPE == MOTOR_Type_Pm))
	{
		ENC_setZeroOffset(gencHandle, (uint32_t)(HAL_getQepPosnMaximum(ghalHandle) - HAL_getQepPosnCounts(ghalHandle)));
		gMotorVars.bFlag_enableRsRecalc = gMotorVars.bFlag_enableOffsetcalc   = false;
		/*if (gMotorVars.bFlag_Aligned == false)
		{
			gMotorVars.bFlag_enableRsRecalc = gMotorVars.bFlag_enableOffsetcalc   = false;
		}*/

	}

	// acknowledge the ADC interrupt
	HAL_acqAdcInt(ghalHandle,ADC_IntNumber_1);


	DATALOG_update(datalogHandle);

	// connect inputs of the PWMDAC module.
	gDacData.aiqvalue[0] = (*gDacData.ptrData[0]);   //
	gDacData.aiqvalue[1] = (*gDacData.ptrData[1]);   //
	gDacData.aiqvalue[2] = (*gDacData.ptrData[2]);   //
	gDacData.aiqvalue[3] = (*gDacData.ptrData[3]);   //

	HAL_writeDacData(ghalHandle,&gDacData);


} // end of mainISR() function







interrupt void adcInt2ISR(void)
{
	HAL_readAdcDataApx(ghalHandle,&gAdcData);	//External Terminal ADC
	HAL_acqAdcInt(ghalHandle,ADC_IntNumber_2);

} // end of mainISR() function

interrupt void timer0ISR(void)
{
	// acknowledge the Timer 0 interrupt	// 1ms interrupt

	HAL_acqTimer0Int(ghalHandle);


	//HAL_toggleLed(halHandle,HAL_GPIO_LED3);
} // end of timer0ISR() function


interrupt void timer1ISR(void)		//Low priority
{
	// acknowledge the Timer 0 interrupt	// 1ms interrupt
	static uint_least8_t u8Index = 0;
	EINT;

	DM_run();
	u8Index = (u8Index+1) % 0x0007;
	switch (u8Index)
	{
		case 0:
			SCIMessageSlave_run(gsciMessageHandle[0]);
			break;
		case 1:
			OPERATOR_run(gOperHandle);
			break;
		case 2:
			SCIMessageMaster_run(gsciMessageHandle[1]);
			break;
		case 5:
	//		IOEXPAND_run(gIoexpandHandle);
			break;
	}

	DINT;
	HAL_acqTimer1Int(ghalHandle);

} // end of timer1ISR() function


interrupt void i2cInt1AISR(void)
{
	HAL_Obj *pHalObj = (HAL_Obj *)ghalHandle;
	I2C_Handle i2cHandle = pHalObj->i2cHandle;
	I2C_InterruptCode_e i2cInterruptCode;
	uint_least8_t i, u8Cnt;

	i2cInterruptCode = I2C_getIntFlagStatus(i2cHandle);	// Read interrupt source
	switch (i2cInterruptCode)
	{
		case I2C_StopDection:
			 // If completed message was writing data, reset msg to inactive state
			if (gi2cMessage.msgStatusCode == I2C_MSGSTAT_WRITE_BUSY)
				gi2cMessage.msgStatusCode = (I2C_MessageCode_e) I2C_MSGSTAT_INACTIVE;
			else if (gi2cMessage.msgStatusCode == I2C_MSGSTAT_READ_BUSY)
			{
				gi2cMessage.msgStatusCode = I2C_MSGSTAT_INACTIVE;
				u8Cnt = I2C_getReceiveFifoNo(i2cHandle);
				for (i=0;i < u8Cnt; i++)
					gi2cMessage.au8RxBuffer[gi2cMessage.u8RxIndex++] =  I2C_read(i2cHandle);


			}
			else //if (gi2cMessage.msgStatusCode == I2C_MSGSTAT_NOACK)
			{
				gi2cMessage.msgStatusCode = I2C_MSGSTAT_ERR;
				I2C_resetTxFifo(i2cHandle);
				I2C_enableTxFifo(i2cHandle);
			}
			break;
		case I2C_NackDection:


			gi2cMessage.msgStatusCode = I2C_MSGSTAT_NOACK;
			I2C_StopCond(i2cHandle);
			I2C_clearNackStatus(i2cHandle);	//I2C_MSGSTAT_POLL_ACK & I2C_MSGSTAT_WRITE_BUSY
			break;
		case I2C_RegisterReady:

		    if (gi2cMessage.msgStatusCode == I2C_MSGSTAT_SEND_NOSTOP_BUSY) //||
		    	//	 (pI2cMessage->msgStatusCode == I2C_MSGSTAT_READ_CURRENT_POLL))
			{
		    	gi2cMessage.msgStatusCode = I2C_MSGSTAT_READ_BUSY;
		    	gi2cMessage.u8RxIndex = 0;
				I2C_setDataCount(i2cHandle,gi2cMessage.u8RxNumOfBytes);  	//I2caRegs.I2CCNT = msg->NumOfBytes+2;
				I2C_setTxRxMode(i2cHandle, I2C_Mode_Receive);
				I2C_StartCond(i2cHandle);

				I2C_clearRxFifoIntFlag(i2cHandle);
			}
		    else if (gi2cMessage.msgStatusCode == I2C_MSGSTAT_READ_BUSY)
		    {
		    	I2C_StopCond(i2cHandle);
		    }
			break;

		default:
			// Generate some error due to invalid interrupt source
			__asm("   ESTOP0");
	}

	HAL_acqI2cInt(ghalHandle);
}


interrupt void i2cInt2AISR(void) //FIFO Interrupt
{
	HAL_Obj *pHalObj = (HAL_Obj *)ghalHandle;
	I2C_Handle i2cHandle = pHalObj->i2cHandle;
	uint_least8_t i, u8Cnt;



	if ((gi2cMessage.msgStatusCode == I2C_MSGSTAT_WRITE_BUSY) ||
		(gi2cMessage.msgStatusCode == I2C_MSGSTAT_SEND_NOSTOP_BUSY))
	{
		u8Cnt = gi2cMessage.u8TxNumOfBytes-gi2cMessage.u8TxIndex;
		if (u8Cnt != 0)
		{
			u8Cnt = DATA_min(u8Cnt, I2C_MAX_LEVEL);
			for (i=0; i< u8Cnt; i++)
				I2C_write(i2cHandle,gi2cMessage.au8TxBuffer[gi2cMessage.u8TxIndex++]);
			I2C_clearTxFifoIntFlag(i2cHandle);
		}
		else
		{
			if (gi2cMessage.msgStatusCode == I2C_MSGSTAT_WRITE_BUSY)
				I2C_StopCond(i2cHandle);
		}


	}
	else if  (gi2cMessage.msgStatusCode ==I2C_MSGSTAT_READ_BUSY )
	{
		u8Cnt = I2C_getReceiveFifoNo(i2cHandle);
		for (i=0;i < u8Cnt; i++)
			gi2cMessage.au8RxBuffer[gi2cMessage.u8RxIndex++] =  I2C_read(i2cHandle);
		I2C_clearRxFifoIntFlag(i2cHandle);



	}
	HAL_acqI2cInt(ghalHandle);
}

void sciTxFifoISR(SCIMessage_Handle  sciMsgHandle)
{
	SCI_Message *pSCIMessage = (SCI_Message *)  sciMsgHandle;
	SCI_Handle sciHandle = pSCIMessage->sciHandle;
	uint_least8_t i, u8Cnt;

	u8Cnt = pSCIMessage->u8TxNumOfBytes - pSCIMessage->u8TxIndex;
	if (u8Cnt)
	{
		u8Cnt = DATA_min(u8Cnt, SCI_MAX_LEVEL);
		for (i=0; i< u8Cnt; i++)
			SCI_write(sciHandle, pSCIMessage->au8TxBuffer[pSCIMessage->u8TxIndex++]);
	}
	else
		SCI_disableTxFifoInt(sciHandle);
}

interrupt void sciATxFifoISR(void)
{
	sciTxFifoISR(gsciMessageHandle[0]);
	HAL_acqSciATxInt(ghalHandle);

}

interrupt void sciBTxFifoISR(void)
{
	sciTxFifoISR(gsciMessageHandle[1]);
	HAL_acqSciBTxInt(ghalHandle);
}

void sciRxFifoISR(SCIMessage_Handle  sciMsgHandle)
{
	SCI_Message *pSCIMessage = (SCI_Message *)  sciMsgHandle;
	SCI_Handle sciHandle = pSCIMessage->sciHandle;

	if (SCI_rxError(sciHandle))
	{
		SCI_disable(sciHandle);
		pSCIMessage->u8RxIndex = 0;
		SCI_enable(sciHandle);
	}
	else
	{
		if (SCI_rxFiFoOver(sciHandle))
		{
	    	SCI_resetRxFifo(sciHandle);
	    	SCI_clearRxFifoOvf(sciHandle);
	    	pSCIMessage->u8RxIndex=0;

		}
		else
		{
			uint_least8_t i, u8Cnt, u8Index;

			u8Cnt = SCI_getRxFifoStatus(sciHandle) >> 8;
	    	u8Index = pSCIMessage->u8RxIndex;
	    	for (i=0; i< u8Cnt; i++)
	    	{
	    		pSCIMessage->au8RxBuffer[u8Index] = (uint_least8_t) SCI_read(sciHandle);
	    		if (u8Index < SCI_RX_MAX_BUFFER_SZIE)
	    			pSCIMessage->u8RxIndex = ++u8Index;
	    	}
	    	pSCIMessage->u16TimeoutCnt = MAX_OPERTIMEOUT1;
		}
	 }

}

interrupt void sciARxFifoISR(void)
{
	sciRxFifoISR( gsciMessageHandle[0]);
	HAL_acqSciARxInt(ghalHandle);
}


interrupt void sciBRxFifoISR(void)
{
	sciRxFifoISR(gsciMessageHandle[1]);
	HAL_acqSciBRxInt(ghalHandle);
}


//$ this is the function keep updating the variables, gmotorVar ,in watch window
void updateGlobalVariables_motor(CTRL_Handle handle)	//$original:void updateGlobalVariables_motor(CTRL_Handle handle,ST_Handle sthandle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;
	// ST_Obj *stObj = (ST_Obj *)sthandle;
	//int32_t i32tmp;
	//float fLs_q;




	//_iq iqkrpm_to_pu_sf = EST_get_krpm_to_pu_sf(pobj->estHandle);

	// get the speed estimate
	gMotorVars.iqSpeed_krpm = EST_getSpeed_krpm(pobj->estHandle);

	gMotorVars.iqSpeedQEP_krpm = ENC_getSpeedKRPM(gencHandle);

	// get the speed from eQEP
	//gMotorVars.SpeedQEP_krpm = _IQmpy(STPOSCONV_getVelocityFiltered(stObj->posConvHandle), _IQ(ST_SPEED_KRPM_PER_PU));	//Org

	//gMotorVars.Speed_krpm = ENC_getFilteredSpeed(encHandle);	//EST_getAngle_pu(obj->estHandle);
	//gMotorVars.SpeedQEP_krpm =_IQmpy(ENC_getFilteredSpeed(encHandle), _IQ(ST_SPEED_KRPM_PER_PU)) ;
	// gMotorVars.iqSpeedQEP_krpm = _IQ(ENC_getSpeedRPM(gencHandle)/1000.0);

	//ENC_getFilteredSpeedPu

	//gMotorVars.SpeedQEP_krpm = ENC_getElecAngle(encHandle)-EST_getAngle_pu(obj->estHandle);

	// get the real time speed reference coming out of the speed trajectory generator
	gMotorVars.iqSpeedTraj_krpm = _IQmpy(CTRL_getSpd_int_ref_pu(handle),EST_get_pu_to_krpm_sf(pobj->estHandle));	//Org
	//gMotorVars.SpeedTraj_krpm = _IQmpy(ENC_getFilteredSpeed(encHandle),EST_get_pu_to_krpm_sf(obj->estHandle));

	// get the torque estimate
	gMotorVars.iqTorque_Nm = USER_computeTorque_Nm(handle, giqTorque_Flux_Iq_pu_to_Nm_sf, giqTorque_Ls_Id_Iq_pu_to_Nm_sf);

	// get the magnetizing current
	gMotorVars.fMagnCurr_A = EST_getIdRated(pobj->estHandle);

	// get the rotor resistance
	gMotorVars.fRr_Ohm = EST_getRr_Ohm(pobj->estHandle);



	// get the stator resistance
	gMotorVars.fRs_Ohm = EST_getRs_Ohm(pobj->estHandle);

	// get the stator inductance in the direct coordinate direction
	gMotorVars.fLsd_H = EST_getLs_d_H(pobj->estHandle);


	// get the stator inductance in the quadrature coordinate direction
	gMotorVars.fLsq_H = EST_getLs_q_H(pobj->estHandle);

	// i32tmp = EST_getLs_q_H(pobj->estHandle);
	//fLs_q = *((float_t *)&i32tmp);

	// get the flux in V/Hz in floating point
	gMotorVars.fFlux_VpHz = EST_getFlux_VpHz(pobj->estHandle);

	// get the flux in Wb in fixed point
	gMotorVars.iqFlux_Wb = USER_computeFlux(handle, giqFlux_pu_to_Wb_sf);

	// get the controller state
	gMotorVars.CtrlState = CTRL_getState(handle);

	// get the estimator state
	gMotorVars.EstState = EST_getState(pobj->estHandle);

	// Get the DC buss voltage
	gMotorVars.iqVdcBus_kV = _IQmpy(gAdcData.iqdcBus,_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));



	//$for checking identify process in spee_ctrl and current_ctrl--current
	gMotorVars.u32identifyCurrentCTRLcnt_inWW=CTRL_getu32identifyCurrentCTRLcnt(handle);

	//$for checking identify process in spee_ctrl and current_ctrl--speed
	gMotorVars.u32identifySpdCTRLcnt_inWW=CTRL_getu32identifySpdCTRLcnt(handle);


	// get the Iq reference from the speed controller
	/* gMotorVars.IqRef_A = _IQmpy(STVELCTL_getTorqueReference(stObj->velCtlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));

      // gets the Velocity Controller status
      gMotorVars.SpinTAC.VelCtlStatus = STVELCTL_getStatus(stObj->velCtlHandle);

      // get the inertia setting
      gMotorVars.SpinTAC.InertiaEstimate_Aperkrpm = _IQmpy(STVELCTL_getInertia(stObj->velCtlHandle), _IQ(ST_SPEED_PU_PER_KRPM * USER_IQ_FULL_SCALE_CURRENT_A));

      // get the friction setting
      gMotorVars.SpinTAC.FrictionEstimate_Aperkrpm = _IQmpy(STVELCTL_getFriction(stObj->velCtlHandle), _IQ(ST_SPEED_PU_PER_KRPM * USER_IQ_FULL_SCALE_CURRENT_A));

      // get the Velocity Controller error
      gMotorVars.SpinTAC.VelCtlErrorID = STVELCTL_getErrorID(stObj->velCtlHandle);

      // get the Position Converter error
      gMotorVars.SpinTAC.PosConvErrorID = STPOSCONV_getErrorID(stObj->posConvHandle);
     */

} // end of updateGlobalVariables_motor() function



//$allows you to set the kp ki value in watch window in real time
void updateKpKiGains(CTRL_Handle handle)
{
	if((gMotorVars.CtrlState == CTRL_State_OnLine) && (gMotorVars.bFlag_MotorIdentified == true) && (bFlag_Latch_softwareUpdate == false))
    {
		// set the kp and ki speed values from the watch window
		CTRL_setKp(handle,CTRL_Type_PID_spd,gMotorVars.iqKp_spd);//put in the ctrlHandle
		CTRL_setKi(handle,CTRL_Type_PID_spd,gMotorVars.iqKi_spd);

		// set the kp and ki current values for Id and Iq from the watch window
		CTRL_setKp(handle,CTRL_Type_PID_Id,gMotorVars.iqKp_Idq);
		CTRL_setKi(handle,CTRL_Type_PID_Id,gMotorVars.iqKi_Idq);
		CTRL_setKp(handle,CTRL_Type_PID_Iq,gMotorVars.iqKp_Idq);
		CTRL_setKi(handle,CTRL_Type_PID_Iq,gMotorVars.iqKi_Idq);
	}

} // end of updateKpKiGains() function

void setupFilterCoeffs(FILTER_FO_Handle filterHandle, float fPole_rps)
{
	_iq b0 = _IQ(fPole_rps/(float_t)gUserParams.u32ctrlFreq_Hz);
	_iq a1 = (b0 - _IQ(1.0));
	_iq b1 = _IQ(0.0);

	FILTER_FO_setDenCoeffs(filterHandle,a1);
	FILTER_FO_setNumCoeffs(filterHandle,b0,b1);
	FILTER_FO_setInitialConditions(filterHandle,_IQ(0.0),_IQ(0.0));
}



/*for(u8Cnt=0;u8Cnt<3;u8Cnt++)
	{
		gFilterHandle[u8Cnt] = FILTER_FO_init(&gFilterObj[u8Cnt],sizeof(gFilterObj[0]));
		FILTER_FO_setDenCoeffs(gFilterHandle[u8Cnt],a1);
		FILTER_FO_setNumCoeffs(gFilterHandle[u8Cnt],b0,b1);
		FILTER_FO_setInitialConditions(gFilterHandle[u8Cnt],_IQ(0.0),_IQ(0.0));
	 }
*/
	/*_iq b0 = _IQ(gUserParams.offsetPole_rps/(float_t)gUserParams.ctrlFreq_Hz);
	_iq a1 = (b0 - _IQ(1.0));
	_iq b1 = _IQ(0.0);

	for(cnt=0;cnt<6;cnt++)
	{
	filterHandle[cnt] = FILTER_FO_init(&filter[cnt],sizeof(filter[0]));
	FILTER_FO_setDenCoeffs(filterHandle[cnt],a1);
	FILTER_FO_setNumCoeffs(filterHandle[cnt],b0,b1);
	FILTER_FO_setInitialConditions(filterHandle[cnt],_IQ(0.0),_IQ(0.0));
	*/


/*void ST_runPosConv(ST_Handle handle, ENC_Handle encHandle, CTRL_Handle ctrlHandle)
{
	ST_Obj *stObj = (ST_Obj *)handle;

	// get the electrical angle from the ENC module
    STPOSCONV_setElecAngle_erev(stObj->posConvHandle, ENC_getElecAngle(encHandle));

    if(USER_MOTOR_TYPE ==  MOTOR_Type_Induction) {
      // The CurrentVector feedback is only needed for ACIM
      // get the vector of the direct/quadrature current input vector values from CTRL
      STPOSCONV_setCurrentVector(stObj->posConvHandle, CTRL_getIdq_in_addr(ctrlHandle));
    }

	// run the SpinTAC Position Converter
	STPOSCONV_run(stObj->posConvHandle);

	if(USER_MOTOR_TYPE ==  MOTOR_Type_Induction) {
	  // The Slip Velocity is only needed for ACIM
	  // update the slip velocity in electrical angle per second, Q24
	  SLIP_setSlipVelocity(slipHandle, STPOSCONV_getSlipVelocity(stObj->posConvHandle));
	}
}
*/
/*
void ST_runVelCtl(ST_Handle handle, CTRL_Handle ctrlHandle)
{
    _iq speedFeedback, iqReference;
    ST_Obj *stObj = (ST_Obj *)handle;
    CTRL_Obj *ctrlObj = (CTRL_Obj *)ctrlHandle;

    // Get the mechanical speed in pu
    speedFeedback = STPOSCONV_getVelocityFiltered(stObj->posConvHandle);

	// Run the SpinTAC Controller
	// Note that the library internal ramp generator is used to set the speed reference
    STVELCTL_setVelocityReference(stObj->velCtlHandle, TRAJ_getIntValue(ctrlObj->trajHandle_spd));
	STVELCTL_setAccelerationReference(stObj->velCtlHandle, _IQ(0.0));	// Internal ramp generator does not provide Acceleration Reference
	STVELCTL_setVelocityFeedback(stObj->velCtlHandle, speedFeedback);
	STVELCTL_run(stObj->velCtlHandle);

	// select SpinTAC Velocity Controller
	iqReference = STVELCTL_getTorqueReference(stObj->velCtlHandle);

	// Set the Iq reference that came out of SpinTAC Velocity Control
	CTRL_setIq_ref_pu(ctrlHandle, iqReference);
}
*/

//@} //defgroup
// end of file
