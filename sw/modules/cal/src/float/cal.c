//! \file   ~/sw/modules/cal/src/float/cal.c
//! \brief  Portable C fixed point code.  These functions define the 
//!         calibrator (CAL) routines
//!
//! (C) Copyright 2014, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "sw/modules/cal/src/float/cal.h"


// **************************************************************************
// the globals


// **************************************************************************
// the functions

CAL_Handle CAL_init(void *pMemory,const size_t numBytes)
{
  uint_least8_t  cnt;
  CAL_Handle     handle;
  CAL_Obj       *obj;


  if(numBytes < sizeof(CAL_Obj))
    return((CAL_Handle)NULL);


  // assign the handle
  handle = (CAL_Handle)pMemory;


  // assign the object
  obj = (CAL_Obj *)handle;


  // initialize the current offset estimator handles
  for(cnt=0;cnt<USER_NUM_CURRENT_SENSORS;cnt++)
    {
      obj->offsetHandle_I[cnt] = OFFSET_init(&obj->offset_I[cnt],sizeof(obj->offset_I[cnt]));
    }


  // initialize the voltage offset estimator handles
  for(cnt=0;cnt<USER_NUM_VOLTAGE_SENSORS;cnt++)
    {
      obj->offsetHandle_V[cnt] = OFFSET_init(&obj->offset_V[cnt],sizeof(obj->offset_V[cnt]));
    }

  return(handle);
} // end of CAL_init() function


void CAL_reset(CAL_Handle handle)
{
//  CAL_Obj *obj = (CAL_Obj *)handle;

  // could reset the offset values here

  return;
} // end of CAL_reset() function


void CAL_setParams(CAL_Handle handle,const USER_Params *pUserParams)
{
  CAL_Obj *obj = (CAL_Obj *)handle;
  float_t beta_lp_rps = pUserParams->offsetPole_rps/(float_t)pUserParams->ctrlFreq_Hz;
  uint_least8_t cnt;


  for(cnt=0;cnt<USER_NUM_CURRENT_SENSORS;cnt++)
    {
      OFFSET_setBeta(obj->offsetHandle_I[cnt],beta_lp_rps);
      OFFSET_setInitCond(obj->offsetHandle_I[cnt],0.0);
      OFFSET_setOffset(obj->offsetHandle_I[cnt],0.0);
    }


  for(cnt=0;cnt<USER_NUM_VOLTAGE_SENSORS;cnt++)
    {
      OFFSET_setBeta(obj->offsetHandle_V[cnt],beta_lp_rps);
      OFFSET_setInitCond(obj->offsetHandle_V[cnt],0.0);
      OFFSET_setOffset(obj->offsetHandle_V[cnt],0.0);
    }


  // set the wait times for each state
  CAL_setWaitTimes(handle,&(pUserParams->calWaitTime[0]));

  CAL_setCount_state(handle,0);

  CAL_setFlag_enable(handle,false);
  CAL_setFlag_enableAdcOffset(handle,true);

  CAL_setState(handle,CAL_State_Idle);

  return;
} // end of CAL_setParams() function


void CAL_setWaitTimes(CAL_Handle handle,const int_least32_t *pWaitTimes)
{
  uint_least8_t cnt;

  for(cnt=0;cnt<CAL_numStates;cnt++)
    {
      CAL_setWaitTime(handle,(CAL_State_e)cnt,pWaitTimes[cnt]);
    }

  return;
} // end of CAL_setWaitTimes() function


bool CAL_updateState(CAL_Handle handle)
{
  CAL_State_e state = CAL_getState(handle);
  bool flag_enable = CAL_getFlag_enable(handle);
  bool stateChanged = false;


  if(flag_enable)
    {
      int_least32_t waitTime = CAL_getWaitTime(handle,state);
      int_least32_t count_state = CAL_getCount_state(handle);


      // check for errors
      // NOTHING FOR NOW


      // check count
      if(count_state >= waitTime)
        {

          // reset the state counter
          CAL_resetCounter_state(handle);


          if(state == CAL_State_Idle)
            {
              // set the next state
              if(CAL_getFlag_enableAdcOffset(handle))
                {
                  CAL_setState(handle,CAL_State_AdcOffset);
                }
              else
                {
                  CAL_setState(handle,CAL_State_Done);
                }
            }
          else if(state == CAL_State_AdcOffset)
            {
              // clear the flag
              CAL_setFlag_enableAdcOffset(handle,false);

              // set the next state
              CAL_setState(handle,CAL_State_Done);
            }
        }
    }
  else if(state != CAL_State_Error)
    {
      if(CAL_isNotIdle(handle))
        {
          // reset the calibrator
          CAL_reset(handle);

          // set the next state
          CAL_setState(handle,CAL_State_Idle);
        }
    }


  // check and see if the state changed
  if(state != CAL_getState(handle))
    {
      stateChanged = true;
    }

  return(stateChanged);
} // end of CAL_updateState() function


// end of file
