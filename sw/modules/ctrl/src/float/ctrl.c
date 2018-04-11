//! \file   ~/sw/modules/ctrl/src/float/ctrl.c
//! \brief  Contains the various functions related to the controller (CTRL) object
//!
//! (C) Copyright 2014, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include <math.h>


// drivers


// modules
#include "sw/modules/math/src/float/math.h"
#include "sw/modules/ctrl/src/float/ctrl.h"


// solutions


// **************************************************************************
// the defines

#ifdef FLASH
#pragma CODE_SECTION(CTRL_computePhasor,"ramfuncs");
#pragma CODE_SECTION(CTRL_getCount_isr,"ramfuncs");
#pragma CODE_SECTION(CTRL_getNumIsrTicksPerCtrlTick,"ramfuncs");
#pragma CODE_SECTION(CTRL_incrCounter_isr,"ramfuncs");
#pragma CODE_SECTION(CTRL_isEnabled,"ramfuncs");
#pragma CODE_SECTION(CTRL_resetCounter_isr,"ramfuncs");
#pragma CODE_SECTION(CTRL_setup,"ramfuncs");
#pragma CODE_SECTION(CTRL_run,"ramfuncs");
#endif


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

void CTRL_getVersion(CTRL_Handle handle,CTRL_Version *pVersion)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  pVersion->rsvd = obj->version.rsvd;
  pVersion->targetProc = obj->version.targetProc;
  pVersion->major = obj->version.major;
  pVersion->minor = obj->version.minor;

  return;
} // end of CTRL_getVersion() function


void CTRL_getWaitTimes(CTRL_Handle handle,int_least32_t *pWaitTimes)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  uint_least16_t stateCnt;

  for(stateCnt=0;stateCnt<CTRL_numStates;stateCnt++)
    {
      pWaitTimes[stateCnt] = obj->waitTimes[stateCnt];
    }

  return;
} // end of CTRL_getWaitTimes() function


CTRL_Handle CTRL_init(void *pMemory,const size_t numBytes)
{
  CTRL_Handle handle;
  CTRL_Obj *obj;


  if(numBytes < sizeof(CTRL_Obj))
    return((CTRL_Handle)NULL);


  // assign the handle
  handle = (CTRL_Handle)pMemory;


  // set the version
  CTRL_setVersion(handle,CTRL_TargetProc_2806x,
                  CTRL_MAJOR_RELEASE_NUMBER,CTRL_MINOR_RELEASE_NUMBER);


  // assign the object
  obj = (CTRL_Obj *)handle;


  // initialize the Id PI controller module
  obj->piHandle_Id = PI_init(&obj->pi_Id,sizeof(obj->pi_Id));


  // initialize the Iq PI controller module
  obj->piHandle_Iq = PI_init(&obj->pi_Iq,sizeof(obj->pi_Iq));


  // initialize the speed PI controller module
  obj->piHandle_spd = PI_init(&obj->pi_spd,sizeof(obj->pi_spd));

  return(handle);
} // end of CTRL_init() function


void CTRL_reset(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  MATH_vec2 null = {(float_t)0.0,(float_t)0.0};

  // reset the integrators
  PI_setUi(obj->piHandle_spd,(float_t)0.0);
  PI_setUi(obj->piHandle_Id,(float_t)0.0);
  PI_setUi(obj->piHandle_Iq,(float_t)0.0);

  // zero internal values
  CTRL_setSpeed_fb_Hz(handle,(float_t)0.0);
  CTRL_setSpeed_ref_Hz(handle,(float_t)0.0);
  CTRL_setSpeed_out_A(handle,(float_t)0.0);

  CTRL_setIdq_A(handle,&null);
  CTRL_setIdq_offset_A(handle,&null);
  CTRL_setVdq_offset_V(handle,&null);

  CTRL_setVdq_V(handle,&null);

  return;
} // end of CTRL_reset() function


void CTRL_setParams(CTRL_Handle handle,USER_Params *pUserParams)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  MATH_vec2 null = {(float_t)0.0,(float_t)0.0};

  float_t Ki,Kp;
  float_t outMin,outMax;

  float_t Rs_d_Ohm = pUserParams->motor_Rs_d_Ohm;
  float_t Rs_q_Ohm = pUserParams->motor_Rs_q_Ohm;
  float_t Rr_d_Ohm = pUserParams->motor_Rr_d_Ohm;
  float_t Rr_q_Ohm = pUserParams->motor_Rr_q_Ohm;
  float_t Ls_d_H = pUserParams->motor_Ls_d_H;
  float_t Ls_q_H = pUserParams->motor_Ls_q_H;
  float_t RoverL_rps;
  float_t BWc_rps = pUserParams->BWc_rps;
  float_t BWdelta = pUserParams->BWdelta;
  float_t Kctrl_Wb_p_kgm2 = pUserParams->Kctrl_Wb_p_kgm2;
  float_t currentCtrlPeriod_sec = (float_t)pUserParams->numCtrlTicksPerCurrentTick / (float_t)pUserParams->ctrlFreq_Hz;
  float_t speedCtrlPeriod_sec = (float_t)pUserParams->numCtrlTicksPerSpeedTick / (float_t)pUserParams->ctrlFreq_Hz;


  // assign the motor type
  CTRL_setMotorParams(handle,pUserParams->motor_type,
                      pUserParams->motor_numPolePairs,
                      pUserParams->motor_ratedFlux_Wb,
                      Ls_d_H,Ls_q_H,
                      Rs_d_Ohm,Rs_q_Ohm,
                      Rr_d_Ohm,Rr_q_Ohm);


  // assign other controller parameters
  CTRL_setNumIsrTicksPerCtrlTick(handle,pUserParams->numIsrTicksPerCtrlTick);
  CTRL_setNumCtrlTicksPerCurrentTick(handle,pUserParams->numCtrlTicksPerCurrentTick);
  CTRL_setNumCtrlTicksPerSpeedTick(handle,pUserParams->numCtrlTicksPerSpeedTick);

  CTRL_setCtrlFreq_Hz(handle,pUserParams->ctrlFreq_Hz);
  CTRL_setCtrlPeriod_sec(handle,pUserParams->ctrlPeriod_sec);
  CTRL_setCurrentCtrlPeriod_sec(handle,currentCtrlPeriod_sec);
  CTRL_setSpeedCtrlPeriod_sec(handle,speedCtrlPeriod_sec);

  CTRL_setIdq_A(handle,&null);
  CTRL_setIdq_offset_A(handle,&null);
  CTRL_setIdq_ref_A(handle,&null);

  CTRL_setVdq_V(handle,&null);
  CTRL_setVdq_offset_V(handle,&null);

  CTRL_setVd_sf(handle,pUserParams->Vd_sf);

  CTRL_setMaxVsMag_V(handle,pUserParams->maxVsMag_V);


  // set the speed reference
  CTRL_setSpeed_fb_Hz(handle,(float_t)0.0);
  CTRL_setSpeed_outMax_A(handle,pUserParams->maxCurrent_A);
  CTRL_setSpeed_ref_Hz(handle,(float_t)0.0);


  // reset the counters
  CTRL_resetCounter_current(handle);
  CTRL_resetCounter_isr(handle);
  CTRL_resetCounter_speed(handle);
  CTRL_resetCounter_state(handle);


  // set the wait times for each state
  CTRL_setWaitTimes(handle,&pUserParams->ctrlWaitTime[0]);


  // set flags
  CTRL_setFlag_enable(handle,false);
  CTRL_setFlag_enableCurrentCtrl(handle,false);
  CTRL_setFlag_enableSpeedCtrl(handle,true);
  CTRL_setFlag_updateKi_Id(handle,false);
  CTRL_setFlag_updateKi_Iq(handle,false);
  CTRL_setFlag_updateKi_spd(handle,false);
  CTRL_setFlag_resetInt_Id(handle,false);
  CTRL_setFlag_resetInt_Iq(handle,false);
  CTRL_setFlag_resetInt_spd(handle,false);
  CTRL_setFlag_useZeroIq_ref(handle,false);


  // initialize the controller error code
  CTRL_setErrorCode(handle,CTRL_ErrorCode_NoError);


  // set the default controller state
  CTRL_setState(handle,CTRL_State_Idle);

  CTRL_setBWc_rps(handle,BWc_rps);
  CTRL_setBWdelta(handle,BWdelta);
  CTRL_setKctrl_Wb_p_kgm2(handle,Kctrl_Wb_p_kgm2);

  // configure the default speed controller gains
  Kp = MATH_TWO_PI * BWc_rps / (BWdelta * Kctrl_Wb_p_kgm2);
  Ki = BWc_rps * speedCtrlPeriod_sec / (BWdelta * BWdelta);
	 
  // set the default speed controller gains
  CTRL_setGains(handle,CTRL_Type_PI_spd,Kp,Ki);


  // configure the default speed controller output minimum/maximum values
  outMax = pUserParams->maxCurrent_A;
  outMin = -outMax;

  // set the default speed controller output minimum/maximum values
  PI_setMinMax(obj->piHandle_spd,outMin,outMax);


  // set the Id current controller gain
  Kp = Ls_d_H * pUserParams->BWc_rps;
  RoverL_rps = Rs_d_Ohm / Ls_d_H;
  Ki = RoverL_rps * currentCtrlPeriod_sec;
  CTRL_setGains(handle,CTRL_Type_PI_Id,Kp,Ki);


  // set the Id current controller gain
  Kp = Ls_q_H * pUserParams->BWc_rps;
  RoverL_rps = Rs_q_Ohm / Ls_q_H;
  CTRL_setGains(handle,CTRL_Type_PI_Iq,Kp,Ki);


  // configure the default current controller output minimum/maximum values
  outMax = pUserParams->dcBus_nominal_V;
  outMin = -outMax;

  // set the default current controller output minimum/maximum values
  PI_setMinMax(obj->piHandle_Id,outMin,outMax);
  PI_setMinMax(obj->piHandle_Iq,outMin,outMax);


  // reset the integrators
  CTRL_setUi(handle,CTRL_Type_PI_spd,(float_t)0.0);
  CTRL_setUi(handle,CTRL_Type_PI_Id,(float_t)0.0);
  CTRL_setUi(handle,CTRL_Type_PI_Iq,(float_t)0.0);

 return;
} // end of CTRL_setParams() function


void CTRL_setVersion(CTRL_Handle handle,const CTRL_TargetProc_e targetProc,
                     const uint16_t majorReleaseNumber,const uint16_t minorReleaseNumber)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->version.rsvd = 107;
  obj->version.targetProc = (uint16_t)targetProc;
  obj->version.major = (uint16_t)majorReleaseNumber;
  obj->version.minor = (uint16_t)minorReleaseNumber;

  return;
} // end of CTRL_setVersion() function


void CTRL_setWaitTimes(CTRL_Handle handle,const int_least32_t *pWaitTimes)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  uint_least16_t stateCnt;

  for(stateCnt=0;stateCnt<CTRL_numStates;stateCnt++)
    {
      obj->waitTimes[stateCnt] = pWaitTimes[stateCnt];
    }

  return;
} // end of CTRL_setWaitTimes() function


bool CTRL_updateState(CTRL_Handle handle)
{
  CTRL_State_e state = CTRL_getState(handle);
  bool flag_enable = CTRL_getFlag_enable(handle);
  bool stateChanged = false;


  if(flag_enable)
    {
      int_least32_t counter_state = CTRL_getCount_state(handle);
      int_least32_t waitTime = CTRL_getWaitTime(handle,state);


      // check for errors
      CTRL_checkForErrors(handle);


      if(counter_state >= waitTime)
        {
          // reset the counter
          CTRL_resetCounter_state(handle);


          if(state == CTRL_State_Idle)
            {
              // set the next controller state
              CTRL_setState(handle,CTRL_State_OnLine);
            }
        }  // if(counter_state >= waitTime) loop
    } 
  else if(state != CTRL_State_Error)
    {
      if(CTRL_isNotIdle(handle))
        {
          // reset the controller
          CTRL_reset(handle);

          // set the next controller state
          CTRL_setState(handle,CTRL_State_Idle);
        }
    }


  // check to see if the state changed
  if(state != CTRL_getState(handle))
    {
      stateChanged = true;
    }

  return(stateChanged);
} // end of CTRL_updateState() function


// end of file

