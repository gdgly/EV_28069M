#ifndef _MAIN_H_
#define _MAIN_H_
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

//! \file   solutions/instaspin_motion/src/main.h
//! \brief Defines the structures, global initialization, and functions used in MAIN 
//!
//! (C) Copyright 2011, Texas Instruments, Inc.

// **************************************************************************
// the includes

// modules
#include "sw/modules/math/src/32b/math.h"
#include "sw/modules/memCopy/src/memCopy.h"
#include "sw/modules/est/src/32b/est.h"
#include "sw/modules/svgen/src/32b/svgen_current.h"
#include "sw/modules/fw/src/32b/fw.h"
#include "sw/modules/slip/src/32b/slip.h"


// drivers


// platforms
#ifndef QEP
#include "sw/modules/ctrl/src/32b/ctrl.h"
#else
#include "sw/modules/ctrl/src/32b/ctrlQEP.h"
#endif
#include "sw/modules/hal/boards/hvkit_rev1p1/f28x/f2806x/src/hal.h"
#include "sw/modules/user/src/32b/user.h"



#include "i2c_mcp23017.h"
#include "sci_message.h"
#include "sci_modbus.h"
#include "sci_operator.h"


// SpinTAC
//#include "spintac_velocity.h"


// **************************************************************************
// the defines



//#define LITTLE_DRIVER_48V	10
//#define LITTLE_DRIVER_400V	11
//#define DRIVER LITTLE_DRIVER_48V


//! \brief Defines the number of main iterations before global variables are updated
//!
#define NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE  1

//! \brief Defines the speed acceleration scale factor.
//!
#define MAX_ACCEL_KRPMPS_SF  _IQ(USER_MOTOR_NUM_POLE_PAIRS * 1000.0 / USER_TRAJ_FREQ_Hz / USER_IQ_FULL_SCALE_FREQ_Hz / 60.0)

//! \brief Initialization values of global variables
//!
//the declaration can be seen after this block~at LINE 150
#define MOTOR_Vars_INIT {true, /*bFlag_enableSys*/   \
                         false, /*bFlag_Run_Identify*/  \
                         false, /*bFlag_MotorIdentified*/  \
                         true, /*bFlag_enableForceAngle*/  \
                         false, /*bFlag_enableFieldWeakening*/  \
                         true, /*bFlag_enableRsRecalc*/  \
                         false, /*bFlag_enableUserParams*/  \
                         true, /*bFlag_enableOffsetcalc*/  \
                         false, /*bFlag_enablePowerWarp*/  \
                         CTRL_State_Idle,   \
                         EST_State_Idle,   \
                         USER_ErrorCode_NoError,  \
                         {0,CTRL_TargetProc_Unknown,0,0}, \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.0), /*iqSpeedRef_krpm*/  \
						 _IQ(0.0), \
                         _IQ(0.2), /*iqMaxAccel_krpmps change to Q(24) form-->0.2*2^24=3355443*/  \
                         _IQ20(5.0), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(USER_MAX_VS_MAG_PU), \
                         _IQ(0.1 * USER_MOTOR_MAX_CURRENT), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         0.0, \
                         0.0, \
                         0.0, \
                         0.0, \
                         0.0, \
                         0.0, \
                         0.0, \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.9 * USER_MAX_VS_MAG_PU), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         _IQ(0.0), \
                         {0,0,0}, \
                         {0,0,0},\
                          0,\
                          0}
						 //ST_VARS_DEFAULTS}


// **************************************************************************
// the typedefs

typedef struct _MOTOR_Vars_t_
{
  bool bFlag_enableSys;
  bool bFlag_Run_Identify;
  bool bFlag_MotorIdentified;//$(can't be change manually,only for showing that the identification is done)
  //bool bFlag_FirstStart;
  bool bFlag_enableForceAngle;
  bool bFlag_enableFieldWeakening;
  bool bFlag_enableRsRecalc;
  bool bFlag_enableUserParams;
  bool bFlag_enableOffsetcalc;
  bool bFlag_enablePowerWarp;//for ACIM motor ,for PMSM,BLDC set to false

  CTRL_State_e CtrlState;
  EST_State_e EstState;

  USER_ErrorCode_e UserErrorCode;

  CTRL_Version CtrlVersion;

  _iq iqIdRef_A;
  _iq iqIqRef_A;
  _iq iqStopSpeedRef_krpm;
  _iq iqSpeedRef_krpm;
  _iq iqSpeedTraj_krpm;
  _iq iqMaxAccel_krpmps;
  _iq20 iq20MaxJrk_krpmps2;
  _iq iqSpeed_krpm;
  _iq iqSpeedQEP_krpm;
  _iq iqOverModulation;
  _iq iqRsOnLineCurrent_A;
  _iq iqFlux_Wb;
  _iq iqTorque_Nm;

  float_t fMagnCurr_A;
  float_t fRr_Ohm;
  float_t fRs_Ohm;
  float_t fRsOnLine_Ohm;
  float_t fLsd_H;
  float_t fLsq_H;
  float_t fFlux_VpHz;

  _iq iqKp_spd;
  _iq iqKi_spd;

  _iq iqKp_Idq;
  _iq iqKi_Idq;

  _iq iqVd;
  _iq iqVq;
  _iq iqVs;
  _iq iqVsRef;
  _iq iqVdcBus_kV;

  _iq iqId_A;
  _iq iqIq_A;
  _iq iqIs_A;

  MATH_vec3 I_bias;
  MATH_vec3 V_bias;
  

  //$for identify process checking
  uint32_t u32identifySpdCTRLcnt_inWW;      //$the counters to check the speed loop in identify process,which will show in watch window
  uint32_t u32identifyCurrentCTRLcnt_inWW;

  //ST_Vars_t SpinTAC;

}MOTOR_Vars_t;


// **************************************************************************
// the globals
extern FILTER_FO_Handle  gFilterDCBusHandle, gFilterOutFreqHandle, gFilterOutEncoderHandle, gFilterOutCurrentHandle;

// **************************************************************************
// the function prototypes

//! \brief The main interrupt service (ISR) routine
//!
extern interrupt void adcISR(void);
extern interrupt void timer0ISR(void);






//! \brief     Updates the global motor variables 
//! 
void updateGlobalVariables_motor(CTRL_Handle handle);	//, ST_Handle stHandle);


//! \brief     Reset Ls Q format to a higher value when Ls identification starts
//!
void CTRL_resetLs_qFmt(CTRL_Handle handle, const uint_least8_t qFmt);


//! \brief     Recalculate Kp and Ki gains to fix the R/L limitation of 2000.0 and Kp limitation of 0.11
//! \brief     as well as recalculates gains based on estimator state to allow low inductance pmsm to id
//!
void recalcKpKiPmsm(CTRL_Handle handle);


//! \brief     Recalculate Kp and Ki gains to fix the R/L limitation of 2000.0 and Kp limitation of 0.11
//!
void recalcKpKi(CTRL_Handle handle);


//! \brief     Calculates the maximum qFmt value for Ls identification, to get a more accurate Ls per unit
//!
void CTRL_calcMax_Ls_qFmt(CTRL_Handle handle, uint_least8_t *p_qFmt);


//! \brief     Updates Kp and Ki gains in the controller object
//!
void updateKpKiGains(CTRL_Handle handle);


//! \brief     Set electrical frequency limit to zero while identifying an induction motor
//!
void setFeLimitZero(CTRL_Handle handle);


//! \brief     Calculates Dir_qFmt for ACIM
//!
void acim_Dir_qFmtCalc(CTRL_Handle handle);


//@} //defgroup
#endif // end of _MAIN_H_ definition



