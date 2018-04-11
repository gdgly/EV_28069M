#ifndef _CTRL_H_
#define _CTRL_H_

//! \file   ~/sw/modules/ctrl/src/float/ctrl.h
//! \brief  Contains public interface to various functions related
//!         to the controller (CTRL) object
//!
//! (C) Copyright 2014, Texas Instruments, Inc.


// **************************************************************************
// the includes


// drivers


// modules
#include "sw/modules/types/src/types.h"

#include "sw/modules/ctrl/src/float/ctrl_obj.h"
#include "sw/modules/ctrl/src/ctrl_states.h"
#include "sw/modules/iqmath/src/float/IQMathLib.h"
#include "sw/modules/motor/src/float/motor.h"
#include "sw/modules/pi/src/float/pi.h"
#include "sw/modules/user/src/float/userParams.h"


// solutions


//!
//!
//! \defgroup CTRL CTRL
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

//! \brief      Checks for any controller errors and, if found, sets the controller state to the error state
//! \param[in]  handle  The controller (CTRL) handle
static inline void CTRL_checkForErrors(CTRL_Handle handle)
{

  return;
} // end of CTRL_checkForErrors() function


//! \brief      Computes a phasor for a given angle
//! \param[in]  angle_rad  The angle, rad
//! \param[out] pPhasor    The pointer to the phasor vector values
static inline void CTRL_computePhasor(const float_t angle_rad,MATH_vec2 *pPhasor)
{

  pPhasor->value[0] = cos(angle_rad);
  pPhasor->value[1] = sin(angle_rad);

  return;
} // end of CTRL_computePhasor() function


//! \brief     Disables the controller
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_disable(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->flag_enable = false;

  return;
} // end of CTRL_disable() function


//! \brief     Enables the controller
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_enable(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->flag_enable = true;

  return;
} // end of CTRL_enable() function


//! \brief     Gets the current controller bandwidth
//! \param[in] handle  The controller (CTRL) handle
//! \return    The current controller bandwidth, rad/sec
static inline float_t CTRL_getBWc_rps(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->BWc_rps);
} // end of CTRL_getBWc_rps() function


//! \brief     Gets the bandwidth scale factor used to maximize phase margin
//! \param[in] handle  The controller (CTRL) handle
//! \return    The bandwidth scale factor
static inline float_t CTRL_getBWdelta(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->BWdelta);
} // end of CTRL_getBWdelta() function


//! \brief     Gets the speed controller constant
//! \param[in] handle  The controller (CTRL) handle
//! \return    The speed controller constant, Wb/(kg*m^2)
static inline float_t CTRL_getKctrl_Wb_p_kgm2(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Kctrl_Wb_p_kgm2);
} // end of CTRL_getKctrl_Wb_p_kgm2() function


//! \brief     Gets the direct voltage scale factor
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct voltage scale factor
static inline float_t CTRL_getVd_sf(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Vd_sf);
} // end of CTRL_getVd_sf() function


//! \brief     Gets the maximum stator voltage magnitude value
//! \param[in] handle  The controller (CTRL) handle
//! \return    The maximum stator voltage magnitude value, V
static inline float_t CTRL_getMaxVsMag_V(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->maxVsMag_V);
} // end of CTRL_getMaxVsMag_V() function


//! \brief     Computes the Id controller output limits
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] outMin  The pointer to the minimum output value
//! \param[in] outMax  The pointer to the maximum output value
static inline void CTRL_computeOutputLimits_Id(CTRL_Handle handle,
                                               float_t *outMin,float_t *outMax)
{
  float_t Vd_sf = CTRL_getVd_sf(handle);
  float_t maxVsMag_V = CTRL_getMaxVsMag_V(handle);
  float_t tmp = Vd_sf * maxVsMag_V;

  *outMin = -tmp;
  *outMax = tmp;

  return;
} // end of CTRL_computeOutputLimits_Id() function


//! \brief     Computes the Iq controller output limits
//! \param[in] handle    The controller (CTRL) handle
//! \param[in] Vd_out_V  The Vd output voltage value, V
//! \param[in] outMin    The pointer to the minimum output value
//! \param[in] outMax    The pointer to the maximum output value
static inline void CTRL_computeOutputLimits_Iq(CTRL_Handle handle,
                                               const float_t Vd_out_V,
                                               float_t *outMin,float_t *outMax)
{
  float_t maxVsMag_V = CTRL_getMaxVsMag_V(handle);

  float_t tmp = sqrt((maxVsMag_V * maxVsMag_V) - (Vd_out_V * Vd_out_V));

  *outMin = -tmp;
  *outMax = tmp;

  return;
} // end of CTRL_computeOutputLimits_Iq() function


//! \brief      Gets the current loop count
//! \param[in]  handle  The controller (CTRL) handle
//! \return    The current loop count
static inline int_least16_t CTRL_getCount_current(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->counter_current);
} // end of CTRL_getCount_current() function


//! \brief     Gets the isr count
//! \param[in] handle  The controller (CTRL) handle
//! \return    The isr count
static inline int_least16_t CTRL_getCount_isr(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->counter_isr);
} // end of CTRL_getCount_isr() function


//! \brief     Gets the speed loop count
//! \param[in] handle  The controller (CTRL) handle
//! \return    The speed loop count
static inline int_least16_t CTRL_getCount_speed(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->counter_speed);
} // end of CTRL_getCount_speed() function


//! \brief     Gets the state count 
//! \param[in] handle  The controller (CTRL) handle
//! \return    The state count
static inline int_least32_t CTRL_getCount_state(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->counter_state);
} // end of CTRL_getCount_state() function


//! \brief     Gets the controller execution frequency
//! \param[in] handle  The controller (CTRL) handle
//! \return    The controller execution frequency, Hz
static inline uint_least32_t CTRL_getCtrlFreq_Hz(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->ctrlFreq_Hz);
} // end of CTRL_getCtrlFreq_Hz() function


//! \brief     Gets the current controller period
//! \param[in] handle  The controller (CTRL) handle
//! \return    The current controller period, sec
static inline float_t CTRL_getCurrentCtrlPeriod_sec(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->currentCtrlPeriod_sec);
} // end of CTRL_getCurrentCtrlPeriod_sec() function


//! \brief     Gets the controller execution period
//! \param[in] handle  The controller (CTRL) handle
//! \return    The controller execution period, sec
static inline float_t CTRL_getCtrlPeriod_sec(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->ctrlPeriod_sec);
} // end of CTRL_getCtrlPeriod_sec() function


//! \brief     Gets the error code from the controller (CTRL) object
//! \param[in] handle  The controller (CTRL) handle
//! \return    The error code
static inline CTRL_ErrorCode_e CTRL_getErrorCode(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->errorCode);
} // end of CTRL_getErrorCode() function


//! \brief     Gets the enable controller flag value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The enable controller flag value
static inline bool CTRL_getFlag_enable(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->flag_enable);
} // end of CTRL_getFlag_enable() function


//! \brief     Gets the enable current controllers flag value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The enable current controller flag value
static inline bool CTRL_getFlag_enableCurrentCtrl(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->flag_enableCurrentCtrl);
} // end of CTRL_getFlag_enableCurrentCtrl() function


//! \brief     Gets the enable speed control flag value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The enable speed control flag value
static inline bool CTRL_getFlag_enableSpeedCtrl(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->flag_enableSpeedCtrl);
} // end of CTRL_getFlag_enableSpeedCtrl() function


//! \brief     Gets the reset Id integrator flag value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The reset Id integrator flag value
static inline bool CTRL_getFlag_resetInt_Id(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->flag_resetInt_Id);
} // end of CTRL_getFlag_resetInt_Id() function


//! \brief     Gets the reset Iq integrator flag value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The reset Iq integrator flag value
static inline bool CTRL_getFlag_resetInt_Iq(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->flag_resetInt_Iq);
} // end of CTRL_getFlag_resetInt_Iq() function


//! \brief     Gets the reset speed integrator flag value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The reset speed integrator flag value
static inline bool CTRL_getFlag_resetInt_spd(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->flag_resetInt_spd);
} // end of CTRL_getFlag_resetInt_spd() function


//! \brief     Gets the update Ki_Id flag value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The update Ki_Id flag value
static inline bool CTRL_getFlag_updateKi_Id(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->flag_updateKi_Id);
} // end of CTRL_getFlag_updateKi_Id() function


//! \brief     Gets the update Ki_Iq flag value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The update Ki_Iq flag value
static inline bool CTRL_getFlag_updateKi_Iq(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->flag_updateKi_Iq);
} // end of CTRL_getFlag_updateKi_Iq() function


//! \brief     Gets the update Ki_spd flag value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The update Ki_spd flag value
static inline bool CTRL_getFlag_updateKi_spd(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->flag_updateKi_spd);
} // end of CTRL_getFlag_updateKi_spd() function


//! \brief     Gets the use zero Iq reference flag value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The use zero Iq reference flag value
static inline bool CTRL_getFlag_useZeroIq_ref(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->flag_useZeroIq_ref);
} // end of CTRL_getFlag_useZeroIq_ref() function


//! \brief     Gets the direct current (Id) value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct current value, A
static inline float_t CTRL_getId_A(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Idq_A.value[0]);
} // end of CTRL_getId_A() function


//! \brief     Gets the direct current (Id) memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct current memory address
static inline float_t *CTRL_getId_A_addr(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Idq_A.value[0]));
} // end of CTRL_getId_A_addr() function


//! \brief     Gets the direct offset current (Id_offset) value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct current offset value, A
static inline float_t CTRL_getId_offset_A(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Idq_offset_A.value[0]);
} // end of CTRL_getId_offset_A() function


//! \brief     Gets the direct offset current (Id_offset) memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct offset current memory address
static inline float_t *CTRL_getId_offset_A_addr(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Idq_offset_A.value[0]));
} // end of CTRL_getId_offset_A_addr() function


//! \brief     Gets the direct reference current (Id_ref) value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct current reference value, A
static inline float_t CTRL_getId_ref_A(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Idq_ref_A.value[0]);
} // end of CTRL_getId_ref_A() function


//! \brief     Gets the direct reference current (Id_ref) memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct current reference memory address
static inline float_t *CTRL_getId_ref_A_addr(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Idq_ref_A.value[0]));
} // end of CTRL_getId_ref_A_addr() function


//! \brief      Gets the direct/quadrature current vector values from the controller
//! \param[in]  handle  The controller (CTRL) handle
//! \param[out] pIdq_A  The vector for the direct/quadrature current vector values, A
static inline void CTRL_getIdq_A(CTRL_Handle handle,MATH_vec2 *pIdq_A)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  pIdq_A->value[0] = obj->Idq_A.value[0];
  pIdq_A->value[1] = obj->Idq_A.value[1];

  return;
} // end of CTRL_getIdq_A() function


//! \brief     Gets the direct/quadrature current (Idq) vector memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct/quadrature current vector memory address
static inline MATH_vec2 *CTRL_getIdq_A_addr(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Idq_A));
} // end of CTRL_getIdq_A_addr() function


//! \brief      Gets the direct/quadrature offset current (Idq_offset) vector values from the controller
//! \param[in]  handle         The controller (CTRL) handle
//! \param[out] pIdq_offset_A  The vector for the direct/quadrature offset current vector values, A
static inline void CTRL_getIdq_offset_A(CTRL_Handle handle,MATH_vec2 *pIdq_offset_A)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  pIdq_offset_A->value[0] = obj->Idq_offset_A.value[0];
  pIdq_offset_A->value[1] = obj->Idq_offset_A.value[1];

  return;
} // end of CTRL_getIdq_offset_A() function


//! \brief     Gets the direct/quadrature offset current (Idq_offset) vector memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct/quadrature offset current vector memory address
static inline MATH_vec2 *CTRL_getIdq_offset_A_addr(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Idq_offset_A));
} // end of CTRL_getIdq_offset_A_addr() function


//! \brief      Gets the direct/quadrature reference current (Idq_ref) vector values from the controller
//! \param[in]  handle      The controller (CTRL) handle
//! \param[out] pIdq_ref_A  The vector for the direct/quadrature reference current vector values, A
static inline void CTRL_getIdq_ref_A(CTRL_Handle handle,MATH_vec2 *pIdq_ref_A)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  pIdq_ref_A->value[0] = obj->Idq_ref_A.value[0];
  pIdq_ref_A->value[1] = obj->Idq_ref_A.value[1];

  return;
} // end of CTRL_getIdq_ref_A() function


//! \brief     Gets the direct/quadrature reference current (Idq_ref) vector memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct/quadrature reference current vector memory address
static inline MATH_vec2 *CTRL_getIdq_ref_A_addr(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Idq_ref_A));
} // end of CTRL_getIdq_ref_A_addr() function


//! \brief     Gets the quadrature current (Iq) value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The quadrature current value, A
static inline float_t CTRL_getIq_A(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Idq_A.value[1]);
} // end of CTRL_getIq_A() function


//! \brief     Gets the quadrature current (Iq) memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The quadrature current memory address
static inline float_t *CTRL_getIq_A_addr(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Idq_A.value[1]));
} // end of CTRL_getIq_A_addr() function


//! \brief     Gets the quadrature offset current (Iq_offset) value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The quadrature current offset value, A
static inline float_t CTRL_getIq_offset_A(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Idq_offset_A.value[1]);
} // end of CTRL_getIq_offset_A() function


//! \brief     Gets the quadrature offset current (Iq_offset) memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The quadrature offset current memory address
static inline float_t *CTRL_getIq_offset_A_addr(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Idq_offset_A.value[1]));
} // end of CTRL_getIq_offset_A_addr() function


//! \brief     Gets the quadrature reference current (Iq_ref) value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The quadrature reference current value, A
static inline float_t CTRL_getIq_ref_A(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Idq_ref_A.value[1]);
} // end of CTRL_getIq_ref_A() function


//! \brief     Gets the quadrature reference current (Iq_ref) memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The quadrature reference current memory address
static inline float_t *CTRL_getIq_ref_A_addr(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Idq_ref_A.value[1]));
} // end of CTRL_getIq_ref_A_addr() function


//! \brief     Gets the integral gain (Ki) value from the specified controller
//! \param[in] handle    The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \return    The Ki value
static inline float_t CTRL_getKi(CTRL_Handle handle,const CTRL_Type_e ctrlType)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  float_t Ki = (float_t)0.0;

  if(ctrlType == CTRL_Type_PI_spd)
    {
      Ki = obj->Ki_spd_ApHz;
    }
  else if(ctrlType == CTRL_Type_PI_Id)
    {
      Ki = obj->Ki_Id;
    }
  else if(ctrlType == CTRL_Type_PI_Iq)
    {
      Ki = obj->Ki_Iq;
    }

  return(Ki);
} // end of CTRL_getKi() function


//! \brief     Gets the proportional gain (Kp) value from the specified controller
//! \param[in] handle    The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \return    The Kp value
static inline float_t CTRL_getKp(CTRL_Handle handle,const CTRL_Type_e ctrlType)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  float_t Kp = (float_t)0.0;

  if(ctrlType == CTRL_Type_PI_spd)
    {
      Kp = obj->Kp_spd_ApHz;
    }
  else if(ctrlType == CTRL_Type_PI_Id)
    {
      Kp = obj->Kp_Id_VpA;
    }
  else if(ctrlType == CTRL_Type_PI_Iq)
    {
      Kp = obj->Kp_Iq_VpA;
    }

  return(Kp);
} // end of CTRL_getKp() function


//! \brief     Gets the gain values for the specified controller
//! \param[in] handle    The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \param[in] pKp       The pointer for the Kp value
//! \param[in] pKi       The pointer for the Ki value
static inline void CTRL_getGains(CTRL_Handle handle,const CTRL_Type_e ctrlType,
                                 float_t *pKp,float_t *pKi)
{
  *pKp = CTRL_getKp(handle,ctrlType);
  *pKi = CTRL_getKi(handle,ctrlType);

  return;    
} // end of CTRL_getGains() function


//! \brief     Gets the motor type
//! \param[in] handle  The controller (CTRL) handle
//! \return    The motor type
static inline MOTOR_Type_e CTRL_getMotorType(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->motorParams.type);
} // end of CTRL_getMotorType() function


//! \brief     Gets the number of controller clock ticks per current controller clock tick
//! \param[in] handle  The controller (CTRL) handle
//! \return    The number of controller clock ticks per controller clock tick
static inline int_least16_t CTRL_getNumCtrlTicksPerCurrentTick(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  
  return(obj->numCtrlTicksPerCurrentTick);
} // end of CTRL_getNumCtrlTicksPerCurrentTick() function


//! \brief     Gets the number of controller clock ticks per speed controller clock tick
//! \param[in] handle  The controller (CTRL) handle
//! \return    The number of controller clock ticks per speed clock tick
static inline int_least16_t CTRL_getNumCtrlTicksPerSpeedTick(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  
  return(obj->numCtrlTicksPerSpeedTick);
} // end of CTRL_getNumCtrlTicksPerSpeedTick() function


//! \brief     Gets the number of Interrupt Service Routine (ISR) clock ticks per controller clock tick
//! \param[in] handle  The controller (CTRL) handle
//! \return    The number of Interrupt Service Routine (ISR) clock ticks per controller clock tick
static inline int_least16_t CTRL_getNumIsrTicksPerCtrlTick(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  
  return(obj->numIsrTicksPerCtrlTick);
} // end of CTRL_getNumIsrTicksPerCtrlTick() function


//! \brief     Gets the reference value from the specified controller
//! \param[in] handle    The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \return    The reference value
static inline float_t CTRL_getRefValue(CTRL_Handle handle,const CTRL_Type_e ctrlType)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  float_t ref = (float_t)0.0;

  if(ctrlType == CTRL_Type_PI_spd)
    {
      ref = PI_getRefValue(obj->piHandle_spd);
    }
  else if(ctrlType == CTRL_Type_PI_Id)
    {
      ref = PI_getRefValue(obj->piHandle_Id);
    }
  else if(ctrlType == CTRL_Type_PI_Iq)
    {
      ref = PI_getRefValue(obj->piHandle_Iq);
    }

  return(ref);
} // end of CTRL_getRefValue() function


//! \brief     Gets the feedback speed value from the controller
//! \param[in] handle    The controller (CTRL) handle
//! \return    The feedback speed value, Hz
static inline float_t CTRL_getSpeed_fb_Hz(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->speed_fb_Hz);
} // end of CTRL_getSpeed_fb_Hz() function


//! \brief     Gets the output value from the speed controller
//! \param[in] handle    The controller (CTRL) handle
//! \return    The output value, A
static inline float_t CTRL_getSpeed_out_A(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->speed_out_A);
} // end of CTRL_getSpeed_out_A() function


//! \brief     Gets the output value memory address from the speed controller
//! \param[in] handle    The controller (CTRL) handle
//! \return    The output value memory address
static inline float_t *CTRL_getSpeed_out_A_addr(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->speed_out_A));
} // end of CTRL_getSpeed_out_A_addr() function


//! \brief     Gets the maximum value for the speed controller
//! \param[in] handle    The controller (CTRL) handle
//! \return    The maximum output value, A
static inline float_t CTRL_getSpeed_outMax_A(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->speed_outMax_A);
} // end of CTRL_getSpeed_outMax_A() function


//! \brief     Gets the output speed reference value from the controller
//! \param[in] handle    The controller (CTRL) handle
//! \return    The output speed reference value, Hz
static inline float_t CTRL_getSpeed_ref_Hz(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->speed_ref_Hz);
} // end of CTRL_getSpeed_ref_Hz() function


//! \brief     Gets the speed controller period
//! \param[in] handle  The controller (EST) handle
//! \return    The speed controller period, sec
static inline float_t CTRL_getSpeedCtrlPeriod_sec(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->speedCtrlPeriod_sec);
} // end of CTRL_getSpeedCtrlPeriod_sec() function


//! \brief     Gets the controller state
//! \param[in] handle  The controller (CTRL) handle
//! \return    The controller state
static inline CTRL_State_e CTRL_getState(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->state);
} // end of CTRL_getState() function


//! \brief     Gets the integrator (Ui) value from the specified controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The Ui value
static inline float_t CTRL_getUi(CTRL_Handle handle,const CTRL_Type_e ctrlType)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  float_t Ui = (float_t)0.0;

  if(ctrlType == CTRL_Type_PI_spd)
    {
      Ui = obj->Ui_spd_A;
    }
  else if(ctrlType == CTRL_Type_PI_Id)
    {
      Ui = obj->Ui_Id_V;
    }
  else if(ctrlType == CTRL_Type_PI_Iq)
    {
      Ui = obj->Ui_Iq_V;
    }

  return(Ui);
} // end of CTRL_getUi() function


//! \brief     Gets the direct voltage (Vd) value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct voltage value, V
static inline float_t CTRL_getVd_V(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Vdq_V.value[0]);
} // end of CTRL_getVd_V() function


//! \brief     Gets the direct voltage (Vd) value memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct voltage value memory address
static inline float_t *CTRL_getVd_V_addr(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Vdq_V.value[0]));
} // end of CTRL_getVd_V_addr() function


//! \brief     Gets the direct offset voltage (Vd_offset) value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct voltage offset value, V
static inline float_t CTRL_getVd_offset_V(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Vdq_offset_V.value[0]);
} // end of CTRL_getVd_offset_V() function


//! \brief     Gets the direct offset voltage (Vd_offset) value memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct offset voltage value memory address
static inline float_t *CTRL_getVd_offset_V_addr(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Vdq_offset_V.value[0]));
} // end of CTRL_getVd_offset_V_addr() function


//! \brief      Gets the direct/quadrature voltage (Vdq) vector values from the controller
//! \param[in]  handle  The controller (CTRL) handle
//! \param[out] pVdq_   The vector for the direct/quadrature voltage vector values, V
static inline void CTRL_getVdq_V(CTRL_Handle handle,MATH_vec2 *pVdq_V)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  pVdq_V->value[0] = obj->Vdq_V.value[0];
  pVdq_V->value[1] = obj->Vdq_V.value[1];

  return;
} // end of CTRL_getVdq_V() function


//! \brief     Gets the direct/quadrature voltage (Vdq) vector memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct/quadrature voltage vector memory address
static inline MATH_vec2 *CTRL_getVdq_V_addr(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Vdq_V));
} // end of CTRL_getVdq_V_addr() function


//! \brief      Gets the direct/quadrature offset voltage (Vdq_offset) vector values from the controller
//! \param[in]  handle          The controller (CTRL) handle
//! \param[out] pVdq_offset_V   The vector for the direct/quadrature offset voltage vector values, V
static inline void CTRL_getVdq_offset_V(CTRL_Handle handle,MATH_vec2 *pVdq_offset_V)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  pVdq_offset_V->value[0] = obj->Vdq_offset_V.value[0];
  pVdq_offset_V->value[1] = obj->Vdq_offset_V.value[1];

  return;
} // end of CTRL_getVdq_offset_V() function


//! \brief     Gets the direct/quadrature offset voltage (Vdq_offset) vector memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct/quadrature offset voltage vector memory address
static inline MATH_vec2 *CTRL_getVdq_offset_V_addr(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Vdq_offset_V));
} // end of CTRL_getVdq_offset_V_addr() function


//! \brief     Gets the controller version number
//! \param[in] handle     The controller (CTRL) handle
//! \param[in] pVersion   A pointer to the version 
extern void CTRL_getVersion(CTRL_Handle handle,CTRL_Version *pVersion);


//! \brief     Gets the quadrature voltage (Vq) value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The quadrature voltage value, pu
static inline float_t CTRL_getVq_V(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Vdq_V.value[1]);
} // end of CTRL_getVq_V() function


//! \brief     Gets the quadrature voltage (Vq) value memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The quadrature voltage value memory address
static inline float_t *CTRL_getVq_V_addr(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Vdq_V.value[1]));
} // end of CTRL_getVq_V_addr() function


//! \brief     Gets the quadrature offset voltage (Vq_offset) value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The quadrature voltage offset value, V
static inline float_t CTRL_getVq_offset_V(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->Vdq_offset_V.value[1]);
} // end of CTRL_getVq_offset_V() function


//! \brief     Gets the quadrature offset voltage (Vq_offset) value memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The quadrature voltage value memory address
static inline float_t *CTRL_getVq_offset_V_addr(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(&(obj->Vdq_offset_V.value[1]));
} // end of CTRL_getVq_offset_V_addr() function


//! \brief     Gets the wait time for a given state
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] state   The controller state
//! \return    The wait time, controller clock counts
static inline int_least32_t CTRL_getWaitTime(CTRL_Handle handle,const CTRL_State_e state)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->waitTimes[state]);
} // end of CTRL_getWaitTime() function


//! \brief     Gets the wait times from the controller
//! \param[in] handle      The controller (CTRL) handle
//! \param[in] pWaitTimes  A pointer to a vector for the wait times, isr clock counts
extern void CTRL_getWaitTimes(CTRL_Handle handle,int_least32_t *pWaitTimes);


//! \brief     Increments the current counter
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_incrCounter_current(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  int_least16_t count = obj->counter_current;

  // increment the count
  count++;

  // limit to 0 to INT_LEAST16_MAX - 1
  if(count == INT_LEAST16_MAX)
    {
      count = 0;
    }

  // save the count value
  obj->counter_current = count;

  return;
} // end of CTRL_incrCounter_current() function


//! \brief     Increments the isr counter
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_incrCounter_isr(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  int_least16_t count = obj->counter_isr;

  // increment the count
  count++;

  // limit to 0 to INT_LEAST16_MAX - 1
  if(count == INT_LEAST16_MAX)
    {
      count = 0;
    }

  // save the count value
  obj->counter_isr = count;

  return;
} // end of CTRL_incrCounter_isr() function


//! \brief     Increments the speed counter
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_incrCounter_speed(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  int_least16_t count = obj->counter_speed;

  // increment the count
  count++;

  // limit to 0 to INT_LEAST16_MAX - 1
  if(count == INT_LEAST16_MAX)
    {
      count = 0;
    }

  // save the count value
  obj->counter_speed = count;

  return;
} // end of CTRL_incrCounter_speed() function


//! \brief     Increments the state counter
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_incrCounter_state(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  int_least32_t count = obj->counter_state;

  // increment the count
  count++;

  // limit to 0 to INT_LEAST32_MAX - 1
  if(count == INT_LEAST32_MAX)
    {
      count = 0;
    }

  // save the count value
  obj->counter_state = count;

  return;
} // end of CTRL_incrCounter_state() function


//! \brief     Initializes the controller
//! \param[in] pMemory   A pointer to the memory for the controller object
//! \param[in] numBytes  The number of bytes allocated for the controller object, bytes
//! \return    The controller (CTRL) object handle
extern CTRL_Handle CTRL_init(void *pMemory,const size_t numBytes);


//! \brief     Determines if the controller is enabled
//! \param[in] handle  The controller (CTRL) handle
static inline bool CTRL_isEnabled(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(obj->flag_enable);
} // end of CTRL_isEnabled() function


//! \brief     Determines if there is a controller error
//! \param[in] handle  The controller (CTRL) handle
//! \return    A boolean value denoting if there is a controller error (true) or not (false)
static inline bool CTRL_isError(CTRL_Handle handle)
{
  CTRL_State_e ctrlState = CTRL_getState(handle);
  bool state = false;


  // check for controller errors
  if(ctrlState == CTRL_State_Error)
    {
      state = true;
    }

  return(state);
} // end of CTRL_isError() function


//! \brief     Determines if the controller is in the idle state
//! \param[in] handle  The controller (CTRL) handle
//! \return     A boolean value denoting if the controller is in the idle state (true) or not (false)
static inline bool CTRL_isIdle(CTRL_Handle handle)
{
  CTRL_State_e state = CTRL_getState(handle);
  bool result = false;

  if(state == CTRL_State_Idle)
    {
      result = true;
    }
  
  return(result);
} // end of CTRL_isIdle() function


//! \brief     Determines if the controller is not in the idle state
//! \param[in] handle  The controller (CTRL) handle
//! \return     A boolean value denoting if the controller is in the idle state (false) or not (true)
static inline bool CTRL_isNotIdle(CTRL_Handle handle)
{
  CTRL_State_e state = CTRL_getState(handle);
  bool result = true;

  if(state == CTRL_State_Idle)
    {
      result = false;
    }
  
  return(result);
} // end of CTRL_isNotIdle() function


//! \brief     Resets the controller
//! \param[in] handle  The controller (CTRL) handle
extern void CTRL_reset(CTRL_Handle handle);


//! \brief     Resets the current counter
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_resetCounter_current(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->counter_current = 0;

  return;
} // end of CTRL_resetCounter_current() function


//! \brief     Resets the isr counter
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_resetCounter_isr(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->counter_isr = 0;

  return;
} // end of CTRL_resetCounter_isr() function


//! \brief     Resets the speed counter
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_resetCounter_speed(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->counter_speed = 0;

  return;
} // end of CTRL_resetCounter_speed() function


//! \brief     Resets the state counter
//! \param[in] handle  The controller (CTRL) handle
static inline void CTRL_resetCounter_state(CTRL_Handle handle)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->counter_state = 0;

  return;
} // end of CTRL_resetCounter_state() function


//! \brief     Sets the current controller bandwidth
//! \param[in] handle   The controller (CTRL) handle
//! \param[in] BWc_rps  The current controller bandwidth, rad/sec
static inline void CTRL_setBWc_rps(CTRL_Handle handle,const float_t BWc_rps)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->BWc_rps = BWc_rps;

  return;
} // end of CTRL_setBWc_rps() function


//! \brief     Sets the bandwidth scale factor used to maximize phase margin
//! \param[in] handle   The controller (CTRL) handle
//! \param[in] BWdelta  The bandwidth scale factor
static inline void CTRL_setBWdelta(CTRL_Handle handle,const float_t BWdelta)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->BWdelta = BWdelta;

  return;
} // end of CTRL_setBWdelta() function


//! \brief      Sets the controller frequency
//! \param[in]  handle       The controller (CTRL) handle
//! \param[in]  ctrlFreq_Hz  The controller frequency, Hz
static inline void CTRL_setCtrlFreq_Hz(CTRL_Handle handle,const uint_least32_t ctrlFreq_Hz)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->ctrlFreq_Hz = ctrlFreq_Hz;

  return;
} // end of CTRL_setCtrlFreq_Hz() function


//! \brief      Sets the controller execution period
//! \param[in]  handle          The controller (CTRL) handle
//! \param[in]  ctrlPeriod_sec  The controller execution period, sec
static inline void CTRL_setCtrlPeriod_sec(CTRL_Handle handle,const float_t ctrlPeriod_sec)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->ctrlPeriod_sec = ctrlPeriod_sec;

  return;
} // end of CTRL_setCtrlPeriod_sec() function


//! \brief     Sets the current controller period value
//! \param[in] handle                 The controller (CTRL) handle
//! \param[in] currentCtrlPeriod_sec  The current controller period value, sec
static inline void CTRL_setCurrentCtrlPeriod_sec(CTRL_Handle handle,const float_t currentCtrlPeriod_sec)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->currentCtrlPeriod_sec = currentCtrlPeriod_sec;

  return;
} // end of CTRL_setCurrentCtrlPeriod_sec() function


//! \brief      Sets the error code in the controller
//! \param[in]  handle  The controller (CTRL) handle
//! \param[in]  errorCode   The error code
static inline void CTRL_setErrorCode(CTRL_Handle handle,const CTRL_ErrorCode_e errorCode)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->errorCode = errorCode;

  return;
} // end of CTRL_setErrorCode() function


//! \brief     Sets the enable controller flag value in the controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] state   The desired state
static inline void CTRL_setFlag_enable(CTRL_Handle handle,const bool state)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->flag_enable = state;

  return;
} // end of CTRL_setFlag_enable() function


//! \brief     Sets the enable current controllers flag value in the controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] state   The desired state
static inline void CTRL_setFlag_enableCurrentCtrl(CTRL_Handle handle,const bool state)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->flag_enableCurrentCtrl = state;

  return;
} // end of CTRL_setFlag_enableCurrentCtrl() function


//! \brief     Sets the enable speed control value in the controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] state   The desired state
static inline void CTRL_setFlag_enableSpeedCtrl(CTRL_Handle handle,const bool state)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->flag_enableSpeedCtrl = state;

  return;
} // end of CTRL_setFlag_enableSpeedCtrl() function


//! \brief     Sets the reset Id integrator flag value in the controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] state   The desired state
static inline void CTRL_setFlag_resetInt_Id(CTRL_Handle handle,const bool state)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->flag_resetInt_Id = state;

  return;
} // end of CTRL_setFlag_resetInt_Id() function


//! \brief     Sets the reset Iq integrator flag value in the controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] state   The desired state
static inline void CTRL_setFlag_resetInt_Iq(CTRL_Handle handle,const bool state)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->flag_resetInt_Iq = state;

  return;
} // end of CTRL_setFlag_resetInt_Iq() function


//! \brief     Sets the reset speed integrator flag value in the controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] state   The desired state
static inline void CTRL_setFlag_resetInt_spd(CTRL_Handle handle,const bool state)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->flag_resetInt_spd = state;

  return;
} // end of CTRL_setFlag_resetInt_spd() function


//! \brief     Sets the update Ki_Id flag value in the controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] state   The desired state
static inline void CTRL_setFlag_updateKi_Id(CTRL_Handle handle,const bool state)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->flag_updateKi_Id = state;

  return;
} // end of CTRL_setFlag_updateKi_Id() function


//! \brief     Sets the update Ki_Iq flag value in the controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] state   The desired state
static inline void CTRL_setFlag_updateKi_Iq(CTRL_Handle handle,const bool state)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->flag_updateKi_Iq = state;

  return;
} // end of CTRL_setFlag_updateKi_Iq() function


//! \brief     Sets the update Ki_spd integrator flag value in the controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] state   The desired state
static inline void CTRL_setFlag_updateKi_spd(CTRL_Handle handle,const bool state)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->flag_updateKi_spd = state;

  return;
} // end of CTRL_setFlag_updateKi_spd() function


//! \brief     Sets the use zero Iq reference flag value in the controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] state   The desired state
static inline void CTRL_setFlag_useZeroIq_ref(CTRL_Handle handle,const bool state)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->flag_useZeroIq_ref = state;

  return;
} // end of CTRL_setFlag_userZeroIq_ref() function


//! \brief     Sets the direct current (Id) value in the controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] Id_A    The direct current value, A
static inline void CTRL_setId_A(CTRL_Handle handle,const float_t Id_A)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Idq_A.value[0] = Id_A;

  return;
} // end of CTRL_setId_A() function


//! \brief     Sets the direct current (Id) offset value in the controller
//! \param[in] handle       The controller (CTRL) handle
//! \param[in] Id_offset_A  The direct current offset value, A
static inline void CTRL_setId_offset_A(CTRL_Handle handle,const float_t Id_offset_A)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Idq_offset_A.value[0] = Id_offset_A;

  return;
} // end of CTRL_setId_offset_A() function


//! \brief     Sets the direct current (Id) reference value in the controller
//! \param[in] handle    The controller (CTRL) handle
//! \param[in] Id_ref_A  The direct current reference value, A
static inline void CTRL_setId_ref_A(CTRL_Handle handle,const float_t Id_ref_A)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Idq_ref_A.value[0] = Id_ref_A;

  return;
} // end of CTRL_setId_ref_A() function


//! \brief     Sets the direct/quadrature current (Idq) vector values in the controller
//! \param[in] handle     The controller (CTRL) handle
//! \param[in] pIdq_in_A  The vector of the direct/quadrature current vector values, A
static inline void CTRL_setIdq_A(CTRL_Handle handle,const MATH_vec2 *pIdq_A)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Idq_A.value[0] = pIdq_A->value[0];
  obj->Idq_A.value[1] = pIdq_A->value[1];

  return;
} // end of CTRL_setIdq_A() function


//! \brief     Sets the direct/quadrature current (Idq) offset vector values in the controller
//! \param[in] handle         The controller (CTRL) handle
//! \param[in] pIdq_offset_A  The vector of the direct/quadrature current offset vector values, A
static inline void CTRL_setIdq_offset_A(CTRL_Handle handle,const MATH_vec2 *pIdq_offset_A)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Idq_offset_A.value[0] = pIdq_offset_A->value[0];
  obj->Idq_offset_A.value[1] = pIdq_offset_A->value[1];

  return;
} // end of CTRL_setIdq_offset_A() function


//! \brief     Sets the direct/quadrature current (Idq) reference vector values in the controller
//! \param[in] handle      The controller (CTRL) handle
//! \param[in] pIdq_ref_A  The vector of the direct/quadrature current reference vector values, A
static inline void CTRL_setIdq_ref_A(CTRL_Handle handle,const MATH_vec2 *pIdq_ref_A)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Idq_ref_A.value[0] = pIdq_ref_A->value[0];
  obj->Idq_ref_A.value[1] = pIdq_ref_A->value[1];

  return;
} // end of CTRL_setIdq_ref_A() function


//! \brief     Sets the quadrature current (Iq) value in the controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] Id_A    The direct current value, A
static inline void CTRL_setIq_A(CTRL_Handle handle,const float_t Iq_A)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Idq_A.value[1] = Iq_A;

  return;
} // end of CTRL_setIq_A() function


//! \brief     Sets the quadrature current (Iq) offset value in the controller
//! \param[in] handle       The controller (CTRL) handle
//! \param[in] Iq_offset_A  The quadrature current reference value, A
static inline void CTRL_setIq_offset_pu(CTRL_Handle handle,const float_t Iq_offset_A)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Idq_offset_A.value[1] = Iq_offset_A;

  return;
} // end of CTRL_setIq_offset_A() function


//! \brief     Sets the quadrature current (Iq) reference value in the controller
//! \param[in] handle    The controller (CTRL) handle
//! \param[in] Iq_ref_A  The quadrature current reference value, A
static inline void CTRL_setIq_ref_A(CTRL_Handle handle,const float_t Iq_ref_A)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Idq_ref_A.value[1] = Iq_ref_A;

  return;
} // end of CTRL_setIq_ref_A() function


//! \brief     Sets the speed controller constant
//! \param[in] handle           The controller (CTRL) handle
//! \param[in] Kctrl_Wb_p_kgm2  The speed controller constant, Wb/(kg*m^2)
static inline void CTRL_setKctrl_Wb_p_kgm2(CTRL_Handle handle,const float_t Kctrl_Wb_p_kgm2)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Kctrl_Wb_p_kgm2 = Kctrl_Wb_p_kgm2;

  return;
} // end of CTRL_setKctrl_Wb_p_kgm2() function


//! \brief     Sets the integral gain (Ki) value for the specified controller
//! \param[in] handle    The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \param[in] Ki        The Ki value
static inline void CTRL_setKi(CTRL_Handle handle,const CTRL_Type_e ctrlType,const float_t Ki)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  if(ctrlType == CTRL_Type_PI_spd)
    {
      obj->Ki_spd_ApHz = Ki;

      CTRL_setFlag_updateKi_spd(handle,true);
    }
  else if(ctrlType == CTRL_Type_PI_Id)
    {
      obj->Ki_Id = Ki;

      CTRL_setFlag_updateKi_Id(handle,true);
    }
  else if(ctrlType == CTRL_Type_PI_Iq)
    {
      obj->Ki_Iq = Ki;

      CTRL_setFlag_updateKi_Iq(handle,true);
    }

  return;
} // end of CTRL_setKi() function


//! \brief     Sets the proportional gain (Kp) value for the specified controller
//! \param[in] handle    The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \param[in] Kp        The Kp value
static inline void CTRL_setKp(CTRL_Handle handle,const CTRL_Type_e ctrlType,const float_t Kp)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  if(ctrlType == CTRL_Type_PI_spd)
    {
      obj->Kp_spd_ApHz = Kp;
    }
  else if(ctrlType == CTRL_Type_PI_Id)
    {
      obj->Kp_Id_VpA = Kp;
    }
  else if(ctrlType == CTRL_Type_PI_Iq)
    {
      obj->Kp_Iq_VpA = Kp;
    }

  return;
} // end of CTRL_setKp() function


//! \brief     Sets the gain values for the specified controller
//! \param[in] handle    The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \param[in] Kp        The Kp gain value, pu
//! \param[in] Ki        The Ki gain value, pu
static inline void CTRL_setGains(CTRL_Handle handle,const CTRL_Type_e ctrlType,
                                 const float_t Kp,const float_t Ki)
{
  CTRL_setKp(handle,ctrlType,Kp);
  CTRL_setKi(handle,ctrlType,Ki);

  if(ctrlType == CTRL_Type_PI_spd)
    {
      CTRL_setFlag_updateKi_spd(handle,true);
    }
  else if(ctrlType == CTRL_Type_PI_Id)
    {
      CTRL_setFlag_updateKi_Id(handle,true);
    }
  else if(ctrlType == CTRL_Type_PI_Iq)
    {
      CTRL_setFlag_updateKi_Iq(handle,true);
    }

  return;    
} // end of CTRL_setGains() function


//! \brief     Sets the maximum stator voltage magnitude value
//! \param[in] handle      The controller (CTRL) handle
//! \param[in] maxVsMax_V  The maximum stator voltage magnitude value, V
static inline void CTRL_setMaxVsMag_V(CTRL_Handle handle,const float_t maxVsMag_V)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->maxVsMag_V = maxVsMag_V;

  return;
} // end of CTRL_setMaxVsMag_V() function


//! \brief     Sets the parameters for the motor in the controller
//! \param[in] handle          The controller (CTRL) handle
//! \param[in] motorType       The motor type
//! \param[in] numPolePairs    The number of motor pole pairs
//! \param[in] ratedFlux_Wb    The rated flux value, Wb
//! \param[in] Ls_d_H          The direct stator inductance, Henry
//! \param[in] Ls_q_H          The quadrature stator inductance, Henry
//! \param[in] Rs_d_Ohm        The direct stator resitance, Ohm
//! \param[in] Rs_q_Ohm        The quadrature stator resitance, Ohm
//! \param[in] Rr_d_Ohm        The direct rotor resistance, Ohm
//! \param[in] Rr_q_Ohm        The quadrature rotor resistance, Ohm
static inline void CTRL_setMotorParams(CTRL_Handle handle,
                                       const MOTOR_Type_e motorType,
                                       const uint_least16_t numPolePairs,
                                       const float_t ratedFlux_Wb,
                                       const float_t Ls_d_H,
                                       const float_t Ls_q_H,
                                       const float_t Rs_d_Ohm,
                                       const float_t Rs_q_Ohm,
                                       const float_t Rr_d_Ohm,
                                       const float_t Rr_q_Ohm)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->motorParams.type = motorType;
  obj->motorParams.numPolePairs = numPolePairs;
  obj->motorParams.ratedFlux_Wb = ratedFlux_Wb;
  obj->motorParams.Ls_d_H = Ls_d_H;
  obj->motorParams.Ls_q_H = Ls_q_H;
  obj->motorParams.Rs_d_Ohm = Rs_d_Ohm;
  obj->motorParams.Rs_q_Ohm = Rs_q_Ohm;
  obj->motorParams.Rr_d_Ohm = Rr_d_Ohm;
  obj->motorParams.Rr_q_Ohm = Rr_q_Ohm;

  return;
} // end of CTRL_setMotorParams() function


//! \brief     Sets the number of controller clock ticks per current controller clock tick
//! \param[in] handle                      The controller (CTRL) handle
//! \param[in] numCtrlTicksPerCurrentTick  The number of controller clock ticks per current controller clock tick
static inline void CTRL_setNumCtrlTicksPerCurrentTick(CTRL_Handle handle,
                                                      const int_least16_t numCtrlTicksPerCurrentTick)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->numCtrlTicksPerCurrentTick = numCtrlTicksPerCurrentTick;

  return;
} // end of CTRL_setNumCtrlTicksPerCurrentTick() function


//! \brief     Sets the number of controller clock ticks per speed controller clock tick
//! \param[in] handle                    The controller (CTRL) handle
//! \param[in] numCtrlTicksPerSpeedTick  The number of controller clock ticks per speed clock tick
static inline void CTRL_setNumCtrlTicksPerSpeedTick(CTRL_Handle handle,
                                                    const int_least16_t numCtrlTicksPerSpeedTick)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->numCtrlTicksPerSpeedTick = numCtrlTicksPerSpeedTick;

  return;
} // end of CTRL_setNumCtrlTicksPerSpeedTick() function


//! \brief     Sets the number of Interrupt Service Routine (ISR) clock ticks per controller clock tick
//! \param[in] handle                  The controller (CTRL) handle
//! \param[in] numIsrTicksPerCtrlTick  The number of Interrupt Service Routine (ISR) clock ticks per controller clock tick
static inline void CTRL_setNumIsrTicksPerCtrlTick(CTRL_Handle handle,
                                                  const int_least16_t numIsrTicksPerCtrlTick)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->numIsrTicksPerCtrlTick = numIsrTicksPerCtrlTick;

  return;
} // end of CTRL_setNumIsrTicksPerCtrlTick() function


//! \brief     Sets the controller parameters
//! \param[in] handle       The controller (CTRL) handle
//! \param[in] pUserParams  The pointer to the user parameters
extern void CTRL_setParams(CTRL_Handle handle,USER_Params *pUserParams);


//! \brief     Sets the feedback speed value in the controller
//! \param[in] handle       The controller (CTRL) handle
//! \param[in] speed_fb_Hz  The feedback speed value, Hz
static inline void CTRL_setSpeed_fb_Hz(CTRL_Handle handle,const float_t speed_fb_Hz)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->speed_fb_Hz = speed_fb_Hz;

  return;
} // end of CTRL_setSpeed_fb_pu() function


//! \brief      Sets the maximum output value for the speed controller
//! \param[in]  handle       The controller (CTRL) handle
//! \param[in]  speed_out_A  The output value from the speed controller
static inline void CTRL_setSpeed_out_A(CTRL_Handle handle, const float_t speed_out_A)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->speed_out_A = speed_out_A;

  return;
} // end of CTRL_setSpeed_out_A() function


//! \brief     Sets the maximum output value for the speed controller
//! \param[in] handle          The controller (CTRL) handle
//! \param[in] speed_outMax_A  The maximum speed value, A
static inline void CTRL_setSpeed_outMax_A(CTRL_Handle handle,const float_t speed_outMax_A)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->speed_outMax_A = speed_outMax_A;

  return;
} // end of CTRL_setSpeed_outMax_A() function


//! \brief     Sets the output speed reference value in the controller
//! \param[in] handle        The controller (CTRL) handle
//! \param[in] speed_ref_Hz  The output speed reference value, Hz
static inline void CTRL_setSpeed_ref_Hz(CTRL_Handle handle,const float_t speed_ref_Hz)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->speed_ref_Hz = speed_ref_Hz;

  return;
} // end of CTRL_setSpeed_ref_Hz() function


//! \brief     Sets the speed controller period value in the estimator
//! \param[in] handle               The controller (CTRL) handle
//! \param[in] speedCtrlPeriod_sec  The speed controller period value, sec
static inline void CTRL_setSpeedCtrlPeriod_sec(CTRL_Handle handle,const float_t speedCtrlPeriod_sec)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->speedCtrlPeriod_sec = speedCtrlPeriod_sec;

  return;
} // end of CTRL_setSpeedCtrlPeriod_sec() function


//! \brief     Sets the controller state
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] state   The new state
static inline void CTRL_setState(CTRL_Handle handle,const CTRL_State_e state)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->prevState = obj->state;
  obj->state = state;

  return;
} // end of CTRL_setState() function


//! \brief     Sets the integrator (Ui) value in the specified controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] Ui      The Ui value
static inline void CTRL_setUi(CTRL_Handle handle,const CTRL_Type_e ctrlType,const float_t Ui)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;


  if(ctrlType == CTRL_Type_PI_spd)
    {
      obj->Ui_spd_A = Ui;

      CTRL_setFlag_resetInt_spd(handle,true);
    }
  else if(ctrlType == CTRL_Type_PI_Id)
    {
      obj->Ui_Id_V = Ui;

      CTRL_setFlag_resetInt_Id(handle,true);
    }
  else if(ctrlType == CTRL_Type_PI_Iq)
    {
      obj->Ui_Iq_V = Ui;

      CTRL_setFlag_resetInt_Iq(handle,true);
    }

  return;
} // end of CTRL_setUi() function


//! \brief     Sets the direct voltage scale factor
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] Vd_sf   The direct voltage scale factor
static inline void CTRL_setVd_sf(CTRL_Handle handle,const float_t Vd_sf)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Vd_sf = Vd_sf;

  return;
} // end of CTRL_setVd_sf() function


//! \brief     Sets the direct/quadrature voltage (Vdq) vector values in the controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] pVdq_V  The vector of direct/quadrature voltage vector values, V
static inline void CTRL_setVdq_V(CTRL_Handle handle,const MATH_vec2 *pVdq_V)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Vdq_V.value[0] = pVdq_V->value[0];
  obj->Vdq_V.value[1] = pVdq_V->value[1];

  return;
} // end of CTRL_setVdq_V() function


//! \brief     Sets the direct/quadrature offset voltage (Vdq_offset) vector values in the controller
//! \param[in] handle         The controller (CTRL) handle
//! \param[in] pVdq_offset_V  The vector of direct/quadrature offset voltage vector values, V
static inline void CTRL_setVdq_offset_V(CTRL_Handle handle,const MATH_vec2 *pVdq_offset_V)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  obj->Vdq_offset_V.value[0] = pVdq_offset_V->value[0];
  obj->Vdq_offset_V.value[1] = pVdq_offset_V->value[1];

  return;
} // end of CTRL_setVdq_offset_V() function


// TEMP
//! \brief     Sets the controller version
//! \param[in] handle              The controller (CTRL) handle
//! \param[in] targetProc          The target processor
//! \param[in] majorReleaseNumber  The major release number
//! \param[in] minorReleaseNumber  The minor release number
extern void CTRL_setVersion(CTRL_Handle handle,
                            const CTRL_TargetProc_e targetProc,
                            const uint16_t majorReleaseNumber,
                            const uint16_t minorReleaseNumber);


//! \brief     Sets the wait times for the controller states
//! \param[in] handle      The controller (CTRL) handle
//! \param[in] pWaitTimes  A pointer to a vector of wait times, controller clock counts
extern void CTRL_setWaitTimes(CTRL_Handle handle,const int_least32_t *pWaitTimes);


//! \brief      Updates the controller state
//! \param[in]  handle  The controller (CTRL) handle
//! \return     A boolean value denoting if the state has changed (true) or not (false)
extern bool CTRL_updateState(CTRL_Handle handle);


//! \brief     Sets up the user controller (CTRL) object
//! \param[in] handle                  The controller (CTRL) handle
//! \param[in] speed_ref_Hz            The reference speed value to use for the speed controller, Hz
//! \param[in] speed_fb_Hz             The feedback speed value to use for the speed controller, Hz
//! \param[in] pIdq_A                  The pointer to the Idq values, A
//! \param[in] pIdq_offset_A           The pointer to the Idq offset values, A
//! \param[in] pVdq_offset_V           The pointer to the Vdq offset values, V
//! \param[in] flag_enableSpeedCtrl    The flag used to enable the speed controller
//! \param[in] flag_enableCurrentCtrl  The flag used to enable the current controller
//! \param[in] flag_useZeroIq_ref      The flag used to enable the zero Iq reference input
static inline void CTRL_setup(CTRL_Handle handle,
                              const float_t speed_ref_Hz,
                              const float_t speed_fb_Hz,
                              const MATH_vec2 *pIdq_A,
                              const MATH_vec2 *pIdq_offset_A,
                              const MATH_vec2 *pVdq_offset_V,
                              const bool flag_enableSpeedCtrl,
                              const bool flag_enableCurrentCtrl,
                              const bool flag_useZeroIq_ref)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;


  // set the input values
  CTRL_setSpeed_ref_Hz(handle,speed_ref_Hz);
  CTRL_setSpeed_fb_Hz(handle,speed_fb_Hz);

  CTRL_setIdq_A(handle,pIdq_A);
  CTRL_setIdq_offset_A(handle,pIdq_offset_A);
  CTRL_setVdq_offset_V(handle,pVdq_offset_V);

  CTRL_setFlag_enableSpeedCtrl(handle,flag_enableSpeedCtrl);
  CTRL_setFlag_enableCurrentCtrl(handle,flag_enableCurrentCtrl);
  CTRL_setFlag_useZeroIq_ref(handle,flag_useZeroIq_ref);


  // if needed, reset the Id integrator
  if(CTRL_getFlag_resetInt_Id(handle))
    {
      float_t Ui = CTRL_getUi(handle,CTRL_Type_PI_Id);

      // set the new integrator value
      PI_setUi(obj->piHandle_Id,Ui);

      // reset the flag
      CTRL_setFlag_resetInt_Id(handle,false);
    }


  // if needed, reset the Iq integrator
  if(CTRL_getFlag_resetInt_Iq(handle))
    {
      float_t Ui = CTRL_getUi(handle,CTRL_Type_PI_Iq);

      // set the new integrator value
      PI_setUi(obj->piHandle_Iq,Ui);

      // reset the flag
      CTRL_setFlag_resetInt_Iq(handle,false);
    }


  // if needed, reset the speed integrator
  if(CTRL_getFlag_resetInt_spd(handle))
    {
      float_t Ui = CTRL_getUi(handle,CTRL_Type_PI_spd);

      // set the new integrator value
      PI_setUi(obj->piHandle_spd,Ui);

      // reset the flag
      CTRL_setFlag_resetInt_spd(handle,false);
    }


  // update the Kp gains
  PI_setKp(obj->piHandle_Id,CTRL_getKp(handle,CTRL_Type_PI_Id));
  PI_setKp(obj->piHandle_Iq,CTRL_getKp(handle,CTRL_Type_PI_Iq));
  PI_setKp(obj->piHandle_spd,CTRL_getKp(handle,CTRL_Type_PI_spd));


  // if needed, update the Ki_Id
  if(CTRL_getFlag_updateKi_Id(handle))
    {
      CTRL_Obj *obj = (CTRL_Obj *)handle;
      
      float_t Ki = PI_getKi(obj->piHandle_Id);
      float_t Ki_new = CTRL_getKi(handle,CTRL_Type_PI_Id);

      if(Ki > 0.0)
        {
          float_t Ui = CTRL_getUi(handle,CTRL_Type_PI_Id);
          
          // scale the integrator value
          Ui = (Ki_new / Ki) * Ui;

          // set the new integrator value
          PI_setUi(obj->piHandle_Id,Ui);
        }

      // set the new Ki value
      PI_setKi(obj->piHandle_Id,Ki_new);

      // reset the flag
      CTRL_setFlag_updateKi_Id(handle,false);
    }


  // if needed, update the Ki_Iq
  if(CTRL_getFlag_updateKi_Iq(handle))
    {
      CTRL_Obj *obj = (CTRL_Obj *)handle;
      
      float_t Ki = PI_getKi(obj->piHandle_Iq);
      float_t Ki_new = CTRL_getKi(handle,CTRL_Type_PI_Iq);

      if(Ki > 0.0)
        {
          float_t Ui = CTRL_getUi(handle,CTRL_Type_PI_Iq);
          
          // scale the integrator value
          Ui = (Ki_new / Ki) * Ui;

          // set the new integrator value
          PI_setUi(obj->piHandle_Iq,Ui);
        }

      // set the new Ki value
      PI_setKi(obj->piHandle_Iq,Ki_new);

      // reset the flag
      CTRL_setFlag_updateKi_Iq(handle,false);
    }


  // if needed, update the Ki_spd
  if(CTRL_getFlag_updateKi_spd(handle))
    {
      CTRL_Obj *obj = (CTRL_Obj *)handle;
      
      float_t Ki = PI_getKi(obj->piHandle_spd);
      float_t Ki_new = CTRL_getKi(handle,CTRL_Type_PI_spd);

      if(Ki > 0.0)
        {
          float_t Ui = CTRL_getUi(handle,CTRL_Type_PI_spd);
          
          // scale the integrator value
          Ui = (Ki_new / Ki) * Ui;

          // set the new integrator value
          PI_setUi(obj->piHandle_spd,Ui);
        }

      // set the new Ki value
      PI_setKi(obj->piHandle_spd,Ki_new);

      // reset the flag
      CTRL_setFlag_updateKi_spd(handle,false);
    }


  // increment the counters
  CTRL_incrCounter_current(handle);
  CTRL_incrCounter_speed(handle);

  return;
} // end of CTRL_setup() function


//! \brief       Run the controller
//! \param[in]   handle  The controller (CTRL) handle
//! \param[out]  pVdq_V  The pointer to the direct/quadrature voltage vector, V
static inline void CTRL_run(CTRL_Handle handle,MATH_vec2 *pVdq_V)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;


  // when appropriate, run the PI speed controller
  if(CTRL_getFlag_enableSpeedCtrl(handle))
    {
     int_least16_t count = CTRL_getCount_speed(handle);
     int_least16_t numCtrlTicksPerSpeedTick = CTRL_getNumCtrlTicksPerSpeedTick(handle);

     if(count >= numCtrlTicksPerSpeedTick)
       {
         float_t refValue_Hz = CTRL_getSpeed_ref_Hz(handle);
         float_t fbackValue_Hz = CTRL_getSpeed_fb_Hz(handle);
         float_t ffwdValue_A = (float_t)0.0;
         float_t outMax_A = CTRL_getSpeed_outMax_A(handle);
         float_t outMin_A = -outMax_A;

         // reset the speed count
         CTRL_resetCounter_speed(handle);

         // set the minimum and maximum values
         PI_setMinMax(obj->piHandle_spd,outMin_A,outMax_A);

         // run the speed controller
         PI_run_series(obj->piHandle_spd,refValue_Hz,fbackValue_Hz,ffwdValue_A,CTRL_getSpeed_out_A_addr(handle));
       }
    }
  else
    {
      // zero the speed output value
      CTRL_setSpeed_out_A(handle,0.0);
    }


  // when appropriate, run the PI Id and Iq controllers
  if(CTRL_getFlag_enableCurrentCtrl(handle))
    {
     int_least16_t count = CTRL_getCount_current(handle);
     int_least16_t numCtrlTicksPerCurrentTick = CTRL_getNumCtrlTicksPerCurrentTick(handle);

     if(count >= numCtrlTicksPerCurrentTick)
       {
         float_t refValue_A;
         float_t fbackValue_A;
         float_t ffwdValue_V;
         float_t outMin_V,outMax_V;
         

         // reset the current count
         CTRL_resetCounter_current(handle);


         // ***********************************
         // configure and run the Id controller

         // get the reference value
         refValue_A = CTRL_getId_offset_A(handle);

         // get the feedback value
         fbackValue_A = CTRL_getId_A(handle);

         // get the feedforward value
         ffwdValue_V = CTRL_getVd_offset_V(handle);

         // compute the Id output limits
         CTRL_computeOutputLimits_Id(handle,&outMin_V,&outMax_V);

         // set the minimum and maximum values
         PI_setMinMax(obj->piHandle_Id,outMin_V,outMax_V);

         // run the Id PI controller
         PI_run_series(obj->piHandle_Id,refValue_A,fbackValue_A,ffwdValue_V,&(pVdq_V->value[0]));

         // store the Id reference value
         CTRL_setId_ref_A(handle,refValue_A);


         // ***********************************
         // configure and run the Iq controller

         // compute the reference value
         if(CTRL_getFlag_enableSpeedCtrl(handle))
           {
             if(CTRL_getFlag_useZeroIq_ref(handle))
               {
                 refValue_A = 0.0;
               }
             else
               {
                 refValue_A = CTRL_getSpeed_out_A(handle) + CTRL_getIq_offset_A(handle);
               }
           }
         else
           {
             // get the Iq offset value
             refValue_A = CTRL_getIq_offset_A(handle);
           }

         // get the feedback value
         fbackValue_A = CTRL_getIq_A(handle);

         // get the feedforward value
         ffwdValue_V = CTRL_getVq_offset_V(handle);

         // compute the Iq output limits
         CTRL_computeOutputLimits_Iq(handle,pVdq_V->value[0],&outMin_V,&outMax_V);

         // set the minimum and maximum values
         PI_setMinMax(obj->piHandle_Iq,outMin_V,outMax_V);

         // run the Iq PI controller
         PI_run_series(obj->piHandle_Iq,refValue_A,fbackValue_A,ffwdValue_V,&(pVdq_V->value[1]));

         // store the Iq reference value
         CTRL_setIq_ref_A(handle,refValue_A);
       }
     else
       {
         CTRL_getVdq_V(handle,pVdq_V);
       }
   }
 else
   {
     pVdq_V->value[0] = 0.0;
     pVdq_V->value[1] = 0.0;
   }


 // store the Vdq value
 CTRL_setVdq_V(handle,pVdq_V);


 // increment the state counter
  CTRL_incrCounter_state(handle);

 return;
} // end of CTRL_run() function


#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup

#endif // end of _CTRL_H_ definition





