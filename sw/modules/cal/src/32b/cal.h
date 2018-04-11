#ifndef _CAL_H_
#define _CAL_H_

//! \file   ~/sw/modules/cal/src/32b/cal.h
//! \brief  Contains the public interface to the calibrator (CAL)
//!         module routines
//!
//! (C) Copyright 2014, Texas Instruments, Inc.


// **************************************************************************
// the includes


// drivers 


// modules
#include "sw/modules/types/src/types.h"
#include "sw/modules/cal/src/cal_states.h"
#include "sw/modules/hal/src/32b/hal_data.h"
#include "sw/modules/math/src/32b/math.h"
#include "sw/modules/offset/src/32b/offset.h"
#include "sw/modules/user/src/32b/userParams.h"


// solutions
#include "user.h"


//!
//! \defgroup CAL

//!
//! \ingroup CAL
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs


//! \brief Defines the calibrator (CAL) object
//!
typedef struct _CAL_Obj_
{
  CAL_State_e      state;                                     //!< the current state

  uint_least32_t   counter_state;                             //!< the state counter
  uint_least32_t   waitTimes[CAL_numStates];                  //!< an array of wait times for each state, calibration clock counts

  OFFSET_Handle    offsetHandle_I[USER_NUM_CURRENT_SENSORS];  //!< the handles for the current offset estimators
  OFFSET_Obj       offset_I[USER_NUM_CURRENT_SENSORS];        //!< the current offset objects

  OFFSET_Handle    offsetHandle_V[USER_NUM_VOLTAGE_SENSORS];  //!< the handles for the voltage offset estimators
  OFFSET_Obj       offset_V[USER_NUM_VOLTAGE_SENSORS];        //!< the voltage offset objects

  bool             flag_enable;                               //!< the enable flag
  bool             flag_enableAdcOffset;                      //!< the enable ADC offset calibration flag
} CAL_Obj;


//! \brief Defines the CAL handle
//!
typedef struct _CAL_Obj_  *CAL_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

//! \brief     Disables the calibrator (CAL) module
//! \param[in] handle  The calibrator (CAL) handle
static inline void CAL_disable(CAL_Handle handle)
{
  CAL_Obj *obj = (CAL_Obj *)handle;

  obj->flag_enable = false;

  return;
} // end of CAL_disable() function


//! \brief     Enables the calibrator (CAL) module
//! \param[in] handle  The calibrator (CAL) handle
static inline void CAL_enable(CAL_Handle handle)
{
  CAL_Obj *obj = (CAL_Obj *)handle;

  obj->flag_enable = true;

  return;
} // end of CAL_enable() function


//! \brief     Gets the state count value
//! \param[in] handle  The calibrator (CAL) handle
//! \return    The state count value
static inline uint_least32_t CAL_getCount_state(CAL_Handle handle)
{
  CAL_Obj *obj = (CAL_Obj *)handle;

  return(obj->counter_state);
} // end of CAL_getCount_state() function


//! \brief     Gets the value of the enable flag
//! \param[in] handle  The calibrator (CAL) handle
//! \return    The enable flag value
static inline bool CAL_getFlag_enable(CAL_Handle handle)
{
  CAL_Obj *obj = (CAL_Obj *)handle;

  return(obj->flag_enable);
} // end of CAL_getFlag_enable() function


//! \brief     Gets the value of the enable ADC offset flag
//! \param[in] handle  The calibrator (CAL) handle
//! \return    The enable ADC offset flag value
static inline bool CAL_getFlag_enableAdcOffset(CAL_Handle handle)
{
  CAL_Obj *obj = (CAL_Obj *)handle;

  return(obj->flag_enableAdcOffset);
} // end of CAL_getFlag_enableAdcOffset() function


//! \brief     Gets the current offset handle address
//! \param[in] handle  The calibrator (CAL) handle
//! \return    The current offset handle address
static inline OFFSET_Handle *CAL_getOffsetHandleAddr_I(CAL_Handle handle)
{
  CAL_Obj *obj = (CAL_Obj *)handle;

  return(&(obj->offsetHandle_I[0]));
} // end of CAL_getOffsetHandleAddr_I() function


//! \brief     Gets the voltage offset handle address
//! \param[in] handle  The calibrator (CAL) handle
//! \return    The voltage offset handle address
static inline OFFSET_Handle *CAL_getOffsetHandleAddr_V(CAL_Handle handle)
{
  CAL_Obj *obj = (CAL_Obj *)handle;

  return(&(obj->offsetHandle_V[0]));
} // end of CAL_getOffsetHandleAddr_V() function


//! \brief     Gets the current offset value
//! \param[in] handle  The calibrator (CAL) handle
//! \param[in] sensorNumber  The sensor number
//! \return    The offset value
static inline _iq CAL_getOffsetValue_I(CAL_Handle handle,
                                       const uint_least8_t sensorNumber)
{
  CAL_Obj *obj = (CAL_Obj *)handle;

  _iq offset = OFFSET_getOffset(obj->offsetHandle_I[sensorNumber]);

  return(offset);
} // end of CAL_getOffsetValue_I() function


//! \brief     Gets the voltage offset value
//! \param[in] handle  The calibrator (CAL) handle
//! \param[in] sensorNumber  The sensor number
//! \return    The offset value
static inline _iq CAL_getOffsetValue_V(CAL_Handle handle,
                                       const uint_least8_t sensorNumber)
{
  CAL_Obj *obj = (CAL_Obj *)handle;

  _iq offset = OFFSET_getOffset(obj->offsetHandle_V[sensorNumber]);

  return(offset);
} // end of CAL_getOffsetValue_V() function


//! \brief     Gets the calibration state
//! \param[in] handle  The calibrator (CAL) handle
//! \return    The calibration state
static inline CAL_State_e CAL_getState(CAL_Handle handle)
{
  CAL_Obj *obj = (CAL_Obj *)handle;

  return(obj->state);
} // end of CAL_getState() function


//! \brief     Gets the wait time for a given state
//! \param[in] handle  The calibrator (CAL) handle
//! \param[in] state   The state
//! \return    The wait time, counts
static inline uint_least32_t CAL_getWaitTime(CAL_Handle handle,const CAL_State_e state)
{
  CAL_Obj *obj = (CAL_Obj *)handle;

  return(obj->waitTimes[state]);
} // end of CAL_getWaitTime() function


//! \brief     Increments the state count value
//! \param[in] handle  The calibrator (CAL) handle
static inline void CAL_incrCounter_state(CAL_Handle handle)
{
  CAL_Obj *obj = (CAL_Obj *)handle;
  uint_least32_t count = obj->counter_state;

  count++;

  obj->counter_state = count;

  return;
} // end of CAL_incrCounter_state() function


//! \brief     Initializes the calibrator (CAL) module
//! \param[in] pMemory   A pointer to the memory for the object
//! \param[in] numBytes  The number of bytes allocated for the object, bytes
//! \return    The calibrator (CAL) object handle
extern CAL_Handle CAL_init(void *pMemory,const size_t numBytes);


//! \brief     Returns a boolean value denoting if the module is enabled (true) or not (false)
//! \param[in] handle  The calibrator (CAL) handle
//! \return    The boolean value
static inline bool CAL_isEnabled(CAL_Handle handle)
{
  CAL_Obj *obj = (CAL_Obj *)handle;

  return(obj->flag_enable);
} // end of CAL_isEnabled() function


//! \brief     Determines if there is a calibrator error
//! \param[in] handle  The calibrator (CAL) handle
//! \return    A boolean value denoting if there is a calibrator error (true) or not (false)
static inline bool CAL_isError(CAL_Handle handle)
{
  CAL_State_e calState = CAL_getState(handle);
  bool state = false;


  // check for controller errors
  if(calState == CAL_State_Error)
    {
      state = true;
    }

  return(state);
} // end of CAL_isError() function


//! \brief     Determines if the calibrator is in the idle state
//! \param[in] handle  The calibrator (CAL) handle
//! \return    A boolean value denoting if the calibrator is in the idle state (true) or not (false)
static inline bool CAL_isIdle(CAL_Handle handle)
{
  CAL_State_e state = CAL_getState(handle);
  bool result = false;

  if(state == CAL_State_Idle)
    {
      result = true;
    }
  
  return(result);
} // end of CAL_isIdle() function


//! \brief     Determines if the calibrator is not in the idle state
//! \param[in] handle  The calibrator (CAL) handle
//! \return    A boolean value denoting if the calibrator is in the idle state (false) or not (true)
static inline bool CAL_isNotIdle(CAL_Handle handle)
{
  CAL_State_e state = CAL_getState(handle);
  bool result = true;

  if(state == CAL_State_Idle)
    {
      result = false;
    }
  
  return(result);
} // end of CAL_isNotIdle() function


//! \brief     Resets the calibrator
//! \param[in] handle  The calibrator (CAL) handle
extern void CAL_reset(CAL_Handle handle);


//! \brief     Resets the state count value
//! \param[in] handle  The calibrator (CAL) handle
static inline void CAL_resetCounter_state(CAL_Handle handle)
{
  CAL_Obj *obj = (CAL_Obj *)handle;

  obj->counter_state = 0;

  return;
} // end of CAL_resetCounter_state() function


//! \brief     Sets the state count value
//! \param[in] handle  The calibrator (CAL) handle
//! \param[in] count   The desired state count value
static inline void CAL_setCount_state(CAL_Handle handle,const uint_least32_t count)
{
  CAL_Obj *obj = (CAL_Obj *)handle;

  obj->counter_state = count;

  return;
} // end of CAL_setCount_state() function


//! \brief     Sets the value of the enable flag
//! \param[in] handle  The calibrator (CAL) handle
//! \param[in] value   The desired flag value
static inline void CAL_setFlag_enable(CAL_Handle handle,const bool value)
{
  CAL_Obj *obj = (CAL_Obj *)handle;

  obj->flag_enable = value;

  return;
} // end of CAL_setFlag_enable() function


//! \brief     Sets the value of the enable ADC offset flag
//! \param[in] handle  The calibrator (CAL) handle
//! \param[in] value   The desired flag value
static inline void CAL_setFlag_enableAdcOffset(CAL_Handle handle,const bool value)
{
  CAL_Obj *obj = (CAL_Obj *)handle;

  obj->flag_enableAdcOffset = value;

  return;
} // end of CAL_setFlag_enableAdcOffset() function


//! \brief     Sets the parameters
//! \param[in] handle       The calibrator (CAL) handle
//! \param[in] pUserParams  The pointer to the user parameters
extern void CAL_setParams(CAL_Handle handle,
                          const USER_Params *pUserParams);


//! \brief     Sets the calibration state
//! \param[in] handle  The calibrator (CAL) handle
//! \param[in] state   The calibration state
static inline void CAL_setState(CAL_Handle handle,const CAL_State_e state)
{
  CAL_Obj *obj = (CAL_Obj *)handle;

  obj->state = state;

  return;
} // end of CAL_setState() function


//! \brief     Sets the wait time for a given state
//! \param[in] handle    The calibrator (CAL) handle
//! \param[in] state     The state
//! \param[in] waitTime  The wait time, counts
static inline void CAL_setWaitTime(CAL_Handle handle,const CAL_State_e state,const uint_least32_t waitTime)
{
  CAL_Obj *obj = (CAL_Obj *)handle;

  obj->waitTimes[state] = waitTime;

  return;
} // end of CAL_setWaitTime() function


//! \brief     Sets the wait times 
//! \param[in] handle      The calibrator (CAL) handle
//! \param[in] pWaitTimes  A point to a vector of wait times
extern void CAL_setWaitTimes(CAL_Handle handle,const uint_least32_t *pWaitTimes);


//! \brief     Sets up the calibrator (CAL) module
//! \param[in] handle  The calibrator (CAL) handle
static inline void CAL_setup(CAL_Handle handle)
{

  return;
} // end of CAL_setup() function


//! \brief     Runs the calibrator (CAL) module
//! \param[in] handle  The calibrator (CAL) handle
static inline void CAL_run(CAL_Handle handle,const HAL_AdcData_t *pAdcData)
{
  CAL_State_e state = CAL_getState(handle);


  // compute the Adc offset
  if(state == CAL_State_AdcOffset)
    {
      CAL_Obj *obj = (CAL_Obj *)handle;
      uint_least8_t cnt;

      // estimate the current offsets
      for(cnt=0;cnt<USER_NUM_CURRENT_SENSORS;cnt++)
        {
          OFFSET_run(obj->offsetHandle_I[cnt],pAdcData->I_pu.value[cnt]);
        }


      // estimate the voltage offsets
      for(cnt=0;cnt<USER_NUM_VOLTAGE_SENSORS;cnt++)
        {
          OFFSET_run(obj->offsetHandle_V[cnt],pAdcData->V_pu.value[cnt]);
        }
    }


  // increment the state counter
  CAL_incrCounter_state(handle);

  return;
} // end of CAL_run() function


//! \brief     Updates the calibrator (CAL) state
//! \param[in] handle  The calibrator (CAL) handle
extern bool CAL_updateState(CAL_Handle handle);


#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup

#endif // end of _CAL_H_ definition


