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
#ifndef _CTRL_H_
#define _CTRL_H_

//! \file   solutions/instaspin_motion/src/ctrlQEP.h
//! \brief Contains the public interface, object and function definitions for 
//!        various functions related to the CTRL object 
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "sw/modules/iqmath/src/32b/IQmathLib.h"

#include "sw/modules/clarke/src/32b/clarke.h"
#include "sw/modules/park/src/32b/park.h"
#include "sw/modules/ipark/src/32b/ipark.h"
#include "sw/modules/motor/src/32b/motor.h"
#include "sw/modules/offset/src/32b/offset.h"
#include "sw/modules/pid/src/32b/pid.h"
#include "sw/modules/est/src/32b/est.h"
#include "sw/modules/svgen/src/32b/svgen.h"
#include "sw/modules/traj/src/32b/traj.h"
#include "sw/modules/ctrl/src/32b/ctrl_obj.h"
#include "sw/modules/enc/src/32b/enc.h"

#include "sw/modules/types/src/types.h"

#include "sw/modules/hal/boards/hvkit_rev1p1/f28x/f2806x/src/hal_obj.h"
//#include "hal_obj.h"


//!
//!
//! \defgroup CTRL CTRL
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif



//$for testing
//uint32_t u32identifyCurrentCTRL=0;
//uint32_t u32identifySpdCTRL=0;


// **************************************************************************
// the function prototypes

//! \brief      Determines if the current controllers should be run
//! \param[in]  handle  The controller (CTRL) handle
//! \return     The value denoting that the current controllers should be run (true) or not (false)
inline bool CTRL_doCurrentCtrl(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;
	bool bresult = false;
  
	if(pobj->u16counter_current >= pobj->u16numCtrlTicksPerCurrentTick)
    {
		bresult = true;
    }

	return(bresult);
} // end of CTRL_doCurrentCtrl() function


//! \brief      Gets the current loop count
//! \param[in]  handle  The controller (CTRL) handle
//! \return    The current loop count
inline uint_least16_t CTRL_getCount_current(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->u16counter_current);
} // end of CTRL_getCount_current() function


//! \brief      Gets the isr count
//! \param[in]  handle  The controller (CTRL) handle
//! \return    The isr count
inline uint_least16_t CTRL_getCount_isr(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

  	return(pobj->u16counter_isr);
} // end of CTRL_getCount_isr() function


//! \brief      Gets the speed loop count
//! \param[in]  handle  The controller (CTRL) handle
//! \return    The speed loop count
inline uint_least16_t CTRL_getCount_speed(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->u16counter_speed);
} // end of CTRL_getCount_speed() function


//! \brief      Gets the state count 
//! \param[in]  handle  The controller (CTRL) handle
//! \return     The state count
inline uint_least32_t CTRL_getCount_state(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->u32counter_state);
} // end of CTRL_getCount_state() function


//! \brief      Gets the trajectory loop count
//! \param[in]  handle  The controller (CTRL) handle
//! \return     The trajectory loop count
inline uint_least16_t CTRL_getCount_traj(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->u16counter_traj);
} // end of CTRL_getCount_traj() function


//! \brief      Gets the controller execution frequency
//! \param[in]  handle  The controller (CTRL) handle
//! \return     The controller execution frequency, Hz
inline uint_least32_t CTRL_getCtrlFreq(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->u32ctrlFreq_Hz);
} // end of CTRL_getCtrlFreq() function


//! \brief      Gets the controller execution period
//! \param[in]  handle  The controller (CTRL) handle
//! \return     The controller execution period, sec
inline float_t CTRL_getCtrlPeriod_sec(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->fctrlPeriod_sec);
} // end of CTRL_getCtrlPeriod_sec() function


//! \brief     Gets the error code from the controller (CTRL) object
//! \param[in] handle  The controller (CTRL) handle
//! \return    The error code
inline CTRL_ErrorCode_e CTRL_getErrorCode(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->errorCode);
} // end of CTRL_getErrorCode() function


//! \brief     Gets the estimator handle for a given controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The estimator handle for the given controller
inline EST_Handle CTRL_getEstHandle(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->estHandle);
} // end of CTRL_getEstHandle() function


//! \brief     Gets the enable controller flag value from the estimator
//! \param[in] handle  The controller (CTRL) handle
//! \return    The enable controller flag value
inline bool CTRL_getFlag_enableCtrl(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->bflag_enableCtrl);
} // end of CTRL_getFlag_enableCtrl() function


//! \brief     Gets the enable DC bus compensation flag value from the estimator
//! \param[in] handle  The controller (CTRL) handle
//! \return    The enable DC bus compensation flag value
inline bool CTRL_getFlag_enableDcBusComp(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->bflag_enableDcBusComp);
} // end of CTRL_getFlag_enableDcBusComp() function


//! \brief     Gets the PowerWarp enable flag value from the estimator
//! \param[in] handle  The controller (CTRL) handle
//! \return    The PowerWarp enable flag value
inline bool CTRL_getFlag_enablePowerWarp(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->bflag_enablePowerWarp);
} // end of CTRL_getFlag_enablePowerWarp() function


//! \brief     Gets the enable offset flag value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The enable offset flag value
inline bool CTRL_getFlag_enableOffset(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->bflag_enableOffset);
} // end of CTRL_getFlag_enableOffset() function


//! \brief     Gets the enable speed control flag value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The enable speed control flag value
inline bool CTRL_getFlag_enableSpeedCtrl(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->bflag_enableSpeedCtrl);
} // end of CTRL_getFlag_enableSpeedCtrl() function


//! \brief     Gets the enable user motor parameters flag value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The enable user motor parameters flag value
inline bool CTRL_getFlag_enableUserMotorParams(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->bflag_enableUserMotorParams);
} // end of CTRL_getFlag_enableUserMotorParams() function


//! \brief     Gets the gain values for the specified controller
//! \param[in] handle    The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \param[in] pKp       The pointer for the Kp value, pu
//! \param[in] pKi       The pointer for the Ki value, pu
//! \param[in] pKd       The pointer for the Kd value, pu
void CTRL_getGains(CTRL_Handle handle,const CTRL_Type_e ctrlType,
                   _iq *piqKp,_iq *piqKi,_iq *piqKd);


//! \brief      Gets the alpha/beta filtered current vector values from the controller
//! \param[in]  handle        The controller (CTRL) handle
//! \param[out] pIab_filt_pu  The vector for the alpha/beta filtered current vector values, pu
void CTRL_getIab_filt_pu(CTRL_Handle handle,MATH_vec2 *pIab_filt_pu);


//! \brief     Gets the alpha/beta filtered current vector memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The alpha/beta filtered current vector memory address
inline MATH_vec2 *CTRL_getIab_filt_addr(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(&(pobj->Iab_filt));
} // end of CTRL_getIab_filt_addr() function


//! \brief     Gets the alpha/beta current input vector memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The alpha/beta current input vector memory address
inline MATH_vec2 *CTRL_getIab_in_addr(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(&(pobj->Iab_in));
} // end of CTRL_getIab_in_addr() function


//! \brief      Gets the alpha/beta current input vector values from the controller
//! \param[in]  handle      The controller (CTRL) handle
//! \param[out] pIab_in_pu  The vector for the alpha/beta current input vector values, pu
void CTRL_getIab_in_pu(CTRL_Handle handle,MATH_vec2 *pIab_in_pu);


//! \brief     Gets the direct current input value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct current input value, pu
inline _iq CTRL_getId_in_pu(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->Idq_in.aiqvalue[0]);
} // end of CTRL_getId_in_pu() function


//! \brief     Gets the direct current (Id) reference value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct current reference value, pu
inline _iq CTRL_getId_ref_pu(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->Idq_ref.aiqvalue[0]);
} // end of CTRL_getId_ref_pu() function


//! \brief     Gets the direct/quadrature current input vector memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct/quadrature current input vector memory address
inline MATH_vec2 *CTRL_getIdq_in_addr(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(&(pobj->Idq_in));
} // end of CTRL_getIdq_in_addr() function


//! \brief      Gets the direct/quadrature current input vector values from the controller
//! \param[in]  handle      The controller (CTRL) handle
//! \param[out] pIdq_in_pu  The vector for the direct/quadrature input current vector values, pu
void CTRL_getIdq_in_pu(CTRL_Handle handle,MATH_vec2 *pIdq_in_pu);


//! \brief     Gets the direct/quadrature current reference vector values from the controller
//! \param[in] handle        The controller (CTRL) handle
//! \param[out] pIdq_ref_pu  The vector for the direct/quadrature current reference vector values, pu
void CTRL_getIdq_ref_pu(CTRL_Handle handle,MATH_vec2 *pIdq_ref_pu);


//! \brief     Gets the Id rated current value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The Id rated current value, pu
inline _iq CTRL_getIdRated_pu(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->iqIdRated);
} // end of CTRL_getIdRated_pu() function


//! \brief     Gets the quadrature current input value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The quadrature current input value, pu
inline _iq CTRL_getIq_in_pu(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->Idq_in.aiqvalue[1]);
} // end of CTRL_getIq_in_pu() function


//! \brief     Gets the quadrature current (Iq) reference value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The quadrature current reference value, pu
inline _iq CTRL_getIq_ref_pu(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->Idq_ref.aiqvalue[1]);
} // end of CTRL_getIq_ref_pu() function


//! \brief     Gets the integral gain (Ki) value from the specified controller
//! \param[in] handle    The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \return    The Ki value
inline _iq CTRL_getKi(CTRL_Handle handle,const CTRL_Type_e ctrlType)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;
	_iq iqKi = _IQ(0.0);

	if(ctrlType == CTRL_Type_PID_spd)
    {
		iqKi = pobj->iqKi_spd;
    }
	else if(ctrlType == CTRL_Type_PID_Id)
    {
		iqKi = pobj->iqKi_Id;
    }
	else if(ctrlType == CTRL_Type_PID_Iq)
    {
		iqKi = pobj->iqKi_Iq;
    }

	return(iqKi);
} // end of CTRL_getKi() function


//! \brief     Gets the derivative gain (Kd) value from the specified controller
//! \param[in] handle    The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \return    The Kd value
inline _iq CTRL_getKd(CTRL_Handle handle,const CTRL_Type_e ctrlType)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;
	_iq iqKd = _IQ(0.0);

	if(ctrlType == CTRL_Type_PID_spd)
    {
		iqKd = pobj->iqKd_spd;
    }
	else if(ctrlType == CTRL_Type_PID_Id)
    {
		iqKd = pobj->iqKd_Id;
    }
	else if(ctrlType == CTRL_Type_PID_Iq)
    {
		iqKd = pobj->iqKd_Iq;
    }

	return(iqKd);
} // end of CTRL_getKd() function


//! \brief     Gets the proportional gain (Kp) value from the specified controller
//! \param[in] handle    The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \return    The Kp value
inline _iq CTRL_getKp(CTRL_Handle handle,const CTRL_Type_e ctrlType)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;
	_iq iqKp = _IQ(0.0);

	if(ctrlType == CTRL_Type_PID_spd)
    {
		iqKp = pobj->iqKp_spd;
    }
	else if(ctrlType == CTRL_Type_PID_Id)
    {
		iqKp = pobj->iqKp_Id;
    }
	else if(ctrlType == CTRL_Type_PID_Iq)
    {
		iqKp = pobj->iqKp_Iq;
    }

  return(iqKp);
} // end of CTRL_getKp() function


//! \brief     Gets the high frequency inductance (Lhf) value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The Lhf value
inline float_t CTRL_getLhf(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->fLhf);
} // end of CTRL_getLhf() function


//! \brief     Gets the magnetizing current value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The magnetizing current value, pu
_iq CTRL_getMagCurrent_pu(CTRL_Handle handle);


//! \brief      Gets the maximum voltage vector
//! \param[in]  handle  The controller (CTRL) handle
//! \return     The maximum voltage vector (value betwen 0 and 4/3)
inline _iq CTRL_getMaxVsMag_pu(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->iqmaxVsMag_pu);
} // end of CTRL_getMaxVsMag_pu() function


//! \brief     Gets the maximum speed value from the controller
//! \param[in] handle    The controller (CTRL) handle
//! \return    The maximum speed value, pu
_iq CTRL_getMaximumSpeed_pu(CTRL_Handle handle);


//! \brief     Gets the motor rated flux from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The motor rated flux, V*sec
inline float_t CTRL_getMotorRatedFlux(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;
  
	return(pobj->motorParams.fratedFlux_VpHz);
} // end of CTRL_getMotorRatedFlux() function


//! \brief     Gets the motor type from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The motor type
inline MOTOR_Type_e CTRL_getMotorType(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;
  
	return(pobj->motorParams.type);
} // end of CTRL_getMotorType() function


//! \brief     Gets the number of controller clock ticks per current controller clock tick
//! \param[in] handle  The controller (CTRL) handle
//! \return    The number of controller clock ticks per estimator clock tick
inline uint_least16_t CTRL_getNumCtrlTicksPerCurrentTick(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;
  
	return(pobj->u16numCtrlTicksPerCurrentTick);
} // end of CTRL_getNumCtrlTicksPerCurrentTick() function


//! \brief     Gets the number of controller clock ticks per speed controller clock tick
//! \param[in] handle  The controller (CTRL) handle
//! \return    The number of controller clock ticks per speed clock tick
inline uint_least16_t CTRL_getNumCtrlTicksPerSpeedTick(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;
  
	return(pobj->u16numCtrlTicksPerSpeedTick);
} // end of CTRL_getNumCtrlTicksPerSpeedTick() function


//! \brief     Gets the number of controller clock ticks per trajectory clock tick
//! \param[in] handle  The controller (CTRL) handle
//! \return    The number of controller clock ticks per trajectory clock tick
inline uint_least16_t CTRL_getNumCtrlTicksPerTrajTick(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;
  
	return(pobj->u16numCtrlTicksPerTrajTick);
} // end of CTRL_getNumCtrlTicksPerTrajTick() function


//! \brief     Gets the number of Interrupt Service Routine (ISR) clock ticks per controller clock tick
//! \param[in] handle  The controller (CTRL) handle
//! \return    The number of Interrupt Service Routine (ISR) clock ticks per controller clock tick
inline uint_least16_t CTRL_getNumIsrTicksPerCtrlTick(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;
  
	return(pobj->u16numIsrTicksPerCtrlTick);
} // end of CTRL_getNumIsrTicksPerCtrlTick() function


//! \brief     Gets the reference value from the specified controller
//! \param[in] handle    The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \return    The reference value, pu
inline _iq CTRL_getRefValue_pu(CTRL_Handle handle,const CTRL_Type_e ctrlType)
{
  CTRL_Obj *pobj = (CTRL_Obj *)handle;
  _iq iqref = _IQ(0.0);

  if(ctrlType == CTRL_Type_PID_spd)
    {
      iqref = PID_getRefValue(pobj->pidHandle_spd);
    }
  else if(ctrlType == CTRL_Type_PID_Id)
    {
      iqref = PID_getRefValue(pobj->pidHandle_Id);
    }
  else if(ctrlType == CTRL_Type_PID_Iq)
    {
      iqref = PID_getRefValue(pobj->pidHandle_Iq);
    }

  return(iqref);
} // end of CTRL_getRefValue_pu() function


//! \brief     Gets the high frequency resistance (Rhf) value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The Rhf value
inline float_t CTRL_getRhf(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->fRhf);
} // end of CTRL_getRhf() function


//! \brief     Gets the R/L value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The the R/L value
inline float_t CTRL_getRoverL(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->fRoverL);
} // end of CTRL_getRoverL() function


//! \brief     Gets the maximum speed value from the controller
//! \param[in] handle    The controller (CTRL) handle
//! \return    The maximum speed value, pu
inline _iq CTRL_getSpd_max_pu(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->iqspd_max);
} // end of CTRL_getSpd_max_pu() function


//! \brief     Gets the output speed memory address from the controller
//! \param[in] handle    The controller (CTRL) handle
//! \return    The output speed memory address
inline _iq *CTRL_getSpd_out_addr(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

  	return(&(pobj->iqspd_out));
} // end of CTRL_getSpd_out_addr() function


//! \brief     Gets the output speed value from the controller
//! \param[in] handle    The controller (CTRL) handle
//! \return    The output speed value, pu
inline _iq CTRL_getSpd_out_pu(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->iqspd_out);
} // end of CTRL_getSpd_out_pu() function


//! \brief     Gets the output speed reference value from the controller
//! \param[in] handle    The controller (CTRL) handle
//! \return    The output speed reference value, pu
inline _iq CTRL_getSpd_ref_pu(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->iqspd_ref);
} // end of CTRL_getSpd_ref_pu() function


//! \brief     Gets the output speed intermediate reference value from the controller
//! \param[in] handle    The controller (CTRL) handle
//! \return    The output speed intermediate reference value, pu
inline _iq CTRL_getSpd_int_ref_pu(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(TRAJ_getIntValue(pobj->trajHandle_spd));
} // end of CTRL_getSpd_int_ref_pu() function


//! \brief     Gets the controller state
//! \param[in] handle  The controller (CTRL) handle
//! \return    The controller state
inline CTRL_State_e CTRL_getState(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->state);
} // end of CTRL_getState() function


//! \brief      Gets the trajectory execution frequency
//! \param[in]  handle  The controller (CTRL) handle
//! \return     The trajectory execution frequency, Hz
inline uint_least32_t CTRL_getTrajFreq(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

 	return(pobj->u32trajFreq_Hz);
} // end of CTRL_getTrajFreq() function


//! \brief      Gets the trajectory execution period
//! \param[in]  handle  The controller (CTRL) handle
//! \return     The trajectory execution period, sec
inline _iq CTRL_getTrajPeriod_sec(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->iqtrajPeriod_sec);
} // end of CTRL_getTrajPeriod_sec() function


//! \brief     Gets the trajectory step size
//! \param[in] handle  The controller (CTRL) handle
void CTRL_getTrajStep(CTRL_Handle handle);


//! \brief     Gets the integrator (Ui) value from the specified controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \return    The Ui value
inline _iq CTRL_getUi(CTRL_Handle handle,const CTRL_Type_e ctrlType)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;
	_iq iqUi = _IQ(0.0);

	if(ctrlType == CTRL_Type_PID_spd)
    {
	  iqUi = pobj->iqUi_spd;
    }
	else if(ctrlType == CTRL_Type_PID_Id)
    {
		iqUi = pobj->iqUi_Id;
    }
	else if(ctrlType == CTRL_Type_PID_Iq)
    {
		iqUi = pobj->iqUi_Iq;
    }

	return(iqUi);
} // end of CTRL_getUi() function


//! \brief     Gets the alpha/beta voltage input vector memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The alpha/beta voltage input vector memory address
inline MATH_vec2 *CTRL_getVab_in_addr(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(&(pobj->Vab_in));
} // end of CTRL_getVab_in_addr() function


//! \brief      Gets the alpha/beta voltage input vector values from the controller
//! \param[in]  handle      The controller (CTRL) handle
//! \param[out] pVab_in_pu  The vector for the alpha/beta voltage input vector values, pu
void CTRL_getVab_in_pu(CTRL_Handle handle,MATH_vec2 *pVab_in_pu);


//! \brief     Gets the alpha/beta voltage output vector memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The alpha/beta voltage output vector memory address
inline MATH_vec2 *CTRL_getVab_out_addr(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(&(pobj->Vab_out));
} // end of CTRL_getVab_out_addr() function


//! \brief      Gets the alpha/beta voltage output vector values from the controller
//! \param[in]  handle       The controller (CTRL) handle
//! \param[out] pVab_out_pu  The vector for the alpha/beta voltage output vector values, pu
void CTRL_getVab_out_pu(CTRL_Handle handle,MATH_vec2 *pVab_out_pu);


//! \brief     Gets the direct voltage output value memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct voltage output value memory address
inline _iq *CTRL_getVd_out_addr(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(&(pobj->Vdq_out.aiqvalue[0]));
} // end of CTRL_getVd_out_addr() function


//! \brief     Gets the direct voltage output value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct voltage output value, pu
inline _iq CTRL_getVd_out_pu(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->Vdq_out.aiqvalue[0]);
} // end of CTRL_getVd_out_pu() function


//! \brief     Gets the direct/quadrature voltage output vector memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The direct/quadrature voltage output vector memory address
inline MATH_vec2 *CTRL_getVdq_out_addr(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(&(pobj->Vdq_out));
} // end of CTRL_getVdq_out_pu() function


//! \brief      Gets the direct/quadrature voltage output vector values from the controller
//! \param[in]  handle       The controller (CTRL) handle
//! \param[out] pVdq_out_pu  The vector for the direct/quadrature voltage output vector values, pu
void CTRL_getVdq_out_pu(CTRL_Handle handle,MATH_vec2 *pVdq_out_pu);


//! \brief     Gets the controller version number
//! \param[in] handle     The controller (CTRL) handle
//! \param[in] pVersion   A pointer to the version 
void CTRL_getVersion(CTRL_Handle handle,CTRL_Version *pVersion);


//! \brief     Gets the quadrature voltage output value memory address from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The quadrature voltage output value memory address
inline _iq *CTRL_getVq_out_addr(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(&(pobj->Vdq_out.aiqvalue[1]));
} // end of CTRL_getVq_out_addr() function


//! \brief     Gets the quadrature voltage output value from the controller
//! \param[in] handle  The controller (CTRL) handle
//! \return    The quadrature voltage output value, pu
inline _iq CTRL_getVq_out_pu(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(pobj->Vdq_out.aiqvalue[1]);
} // end of CTRL_getVq_out_pu() function


//! \brief     Gets the wait time for a given state
//! \param[in] handle     The controller (CTRL) handle
//! \param[in] ctrlState  The controller state
//! \return    The wait time, controller clock counts
inline uint_least32_t CTRL_getWaitTime(CTRL_Handle handle,const CTRL_State_e ctrlState)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

  	return(pobj->au32waitTimes[ctrlState]);
} // end of CTRL_getWaitTime() function


//! \brief     Gets the wait times from the estimator
//! \param[in] handle      The controller (CTRL) handle
//! \param[in] pWaitTimes  A pointer to a vector for the wait times, estimator clock counts
void CTRL_getWaitTimes(CTRL_Handle handle,uint_least32_t *pWaitTimes);


//! \brief     Increments the current counter
//! \param[in] handle  The controller (CTRL) handle
inline void CTRL_incrCounter_current(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;
	pobj->u16counter_current++;

  /*uint_least16_t count = obj->counter_current;

  // increment the count
  count++;

  // save the count value
  obj->counter_current = count;

  return;*/
} // end of CTRL_incrCounter_current() function


//! \brief     Increments the isr counter
//! \param[in] handle  The controller (CTRL) handle
inline void CTRL_incrCounter_isr(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;
	pobj->u16counter_isr++;

 /* uint_least16_t count = obj->counter_isr;

  // increment the count
  count++;

  // save the count value
  obj->counter_isr = count;*/


} // end of CTRL_incrCounter_isr() function


//! \brief     Increments the speed counter
//! \param[in] handle  The controller (CTRL) handle
inline void CTRL_incrCounter_speed(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->u16counter_speed++;

  /*uint_least16_t count = obj->counter_speed;

  // increment the count
  count++;

  // save the count value
  obj->counter_speed = count;

  return;*/
} // end of CTRL_incrCounter_speed() function


//! \brief     Increments the state counter
//! \param[in] handle  The controller (CTRL) handle
inline void CTRL_incrCounter_state(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

  /*uint_least32_t count = obj->counter_state;

  // increment the count
  count++;

  // save the count value
  obj->counter_state = count;*/

	pobj->u32counter_state ++;
} // end of CTRL_incrCounter_state() function


//! \brief     Increments the trajectory counter
//! \param[in] handle  The controller (CTRL) handle
inline void CTRL_incrCounter_traj(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;
	pobj->u16counter_traj++;

  /*uint_least16_t count = obj->counter_traj;

  // increment the count
  count++;

  // save the count value
  obj->counter_traj = count;

  return;*/
} // end of CTRL_incrCounter_traj() function


//! \brief      Initializes a specified controller
//! \details    Initializes all handles required for field oriented control and FAST observer
//!             interface.  Returns a handle to the CTRL object.
//! \param[in]  ctrlNumber  The controller number
//! \param[in]  estNumber   The estimator number
//! \return     The controller (CTRL) object handle
#ifdef FAST_ROM_V1p6
CTRL_Handle CTRL_initCtrl(const uint_least8_t ctrlNumber, const uint_least8_t estNumber);
#else
CTRL_Handle CTRL_initCtrl(const uint_least8_t estNumber,void *pMemory,const size_t numBytes);
#endif


//! \brief     Determines if there is a controller error
//! \param[in] handle  The controller (CTRL) handle
//! \return    A boolean value denoting if there is a controller error (true) or not (false)
inline bool CTRL_isError(CTRL_Handle handle)
{
	CTRL_State_e ctrlState = CTRL_getState(handle);
	bool bstate = false;


	// check for controller errors
	if(ctrlState == CTRL_State_Error)
    {
		bstate = true;
    }

	return(bstate);
} // end of CTRL_isError() function


//! \brief     Resets the current counter
//! \param[in] handle  The controller (CTRL) handle
inline void CTRL_resetCounter_current(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->u16counter_current = 0;

} // end of CTRL_resetCounter_current() function


//! \brief     Resets the isr counter
//! \param[in] handle  The controller (CTRL) handle
inline void CTRL_resetCounter_isr(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->u16counter_isr = 1;


} // end of CTRL_resetCounter_isr() function


//! \brief     Resets the speed counter
//! \param[in] handle  The controller (CTRL) handle
inline void CTRL_resetCounter_speed(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->u16counter_speed = 0;


} // end of CTRL_resetCounter_speed() function


//! \brief     Resets the state counter
//! \param[in] handle  The controller (CTRL) handle
inline void CTRL_resetCounter_state(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->u32counter_state = 0;


} // end of CTRL_resetCounter_state() function


//! \brief     Resets the trajectory counter
//! \param[in] handle  The controller (CTRL) handle
inline void CTRL_resetCounter_traj(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->u16counter_traj = 0;

} // end of CTRL_resetCounter_traj() function


//! \brief      Runs the controller
//! \details    The CTRL_run function implements the field oriented
//!             control.  There are two parts to CTRL_run, CTRL_runOffline,
//!             CTRL_runOnline, and CTRL_runOnlineUser.
//! \param[in]  handle           The controller (CTRL) handle
//! \param[in]  halHandle        The hardware abstraction layer (HAL) handle
//! \param[in]  pAdcData         The pointer to the ADC data
//! \param[out] pPwmData         The pointer to the PWM data
//! \param[in]  electricalAngle  The electrical angle in Q24
void CTRL_run(CTRL_Handle handle,HAL_Handle halHandle,
              const HAL_AdcData_t *pAdcData,
              HAL_PwmData_t *pPwmData,
              uint32_t u32electricalAngle);


//! \brief     Runs the trajectory
//! \param[in] handle   The controller (CTRL) handle
void CTRL_runTraj(CTRL_Handle handle);


//! \brief      Sets the controller frequency
//! \param[in]  handle       The controller (CTRL) handle
//! \param[in]  ctrlFreq_Hz  The controller frequency, Hz
inline void CTRL_setCtrlFreq_Hz(CTRL_Handle handle,const uint_least32_t u32ctrlFreq_Hz)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->u32ctrlFreq_Hz = u32ctrlFreq_Hz;


} // end of CTRL_setCtrlFreq_Hz() function


//! \brief      Sets the controller execution period
//! \param[in]  handle          The controller (CTRL) handle
//! \param[in]  ctrlPeriod_sec  The controller execution period, sec
inline void CTRL_setCtrlPeriod_sec(CTRL_Handle handle,const float_t fctrlPeriod_sec)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->fctrlPeriod_sec = fctrlPeriod_sec;


} // end of CTRL_setCtrlPeriod_sec() function


//! \brief      Sets the error code in the controller
//! \param[in]  handle  The controller (CTRL) handle
//! \param[in]  errorCode   The error code
inline void CTRL_setErrorCode(CTRL_Handle handle,const CTRL_ErrorCode_e errorCode)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->errorCode = errorCode;


} // end of CTRL_setErrorCode() function


//! \brief      Sets the default estimator parameters
//! \details    Copies all scale factors that are defined in the file user.h and used
//!             by CTRL into the CTRL object.
//! \param[in]  estHandle    The estimator (EST) handle
//! \param[in]  pUserParams  The pointer to the user parameters
void CTRL_setEstParams(EST_Handle estHandle,USER_Params *pUserParams);


//! \brief      Sets the enable controller flag value in the estimator
//! \details    Enables/Disables the sensorless controller.  The first time this function
//!             is called and the function CTRL_setFlag_enableUserMotorParams() is not set true,
//!             motor commissioning is performed and then subsequent times it is called the
//!             motor starts to run.
//! \param[in]  handle  The controller (CTRL) handle
//! \param[in]  state   The desired state
inline void CTRL_setFlag_enableCtrl(CTRL_Handle handle,const bool bstate)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->bflag_enableCtrl = bstate;


} // end of CTRL_setFlag_enableCtrl() function


//! \brief      Sets the enable DC bus compensation flag value in the estimator
//! \details    The DC bus compensation algorithm will compensate the Iq and Id PI controller's
//!             proportional gains for DC bus variations.
//! \param[in]  handle  The controller (CTRL) handle
//! \param[in]  state   The desired state
inline void CTRL_setFlag_enableDcBusComp(CTRL_Handle handle,const bool bstate)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->bflag_enableDcBusComp = bstate;

} // end of CTRL_setFlag_enableDcBusComp() function


//! \brief      Sets the PowerWarp enable flag value in the estimator
//! \details    PowerWarp is only used when controlling an induction motor. PowerWarp adjusts field
//!             levels so that the least amount of power is used according to the load on
//!             the motor.
//! \param[in]  handle  The controller (CTRL) handle
//! \param[in]  state   The desired state
inline void CTRL_setFlag_enablePowerWarp(CTRL_Handle handle,const bool bstate)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->bflag_enablePowerWarp = bstate;


} // end of CTRL_setFlag_enablePowerWarp() function


//! \brief      Sets the enable offset flag value in the estimator
//! \param[in]  handle  The controller (CTRL) handle
//! \param[in]  state   The desired state
inline void CTRL_setFlag_enableOffset(CTRL_Handle handle,const bool bstate)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->bflag_enableOffset = bstate;

} // end of CTRL_setFlag_enableOffset() function


//! \brief      Sets the enable speed control value in the estimator
//! \param[in]  handle  The controller (CTRL) handle
//! \param[in]  state   The desired state
inline void CTRL_setFlag_enableSpeedCtrl(CTRL_Handle handle,const bool bstate)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->bflag_enableSpeedCtrl = bstate;

} // end of CTRL_setFlag_enableSpeedCtrl() function


//! \brief      Sets the enable user motor parameters flag value in the estimator
//! \details    When this function is called with a true parameter, the motor parameters
//!             in user.h are used instead of performing a motor commissioning at
//!             startup.
//! \param[in]  handle  The controller (CTRL) handle
//! \param[in]  state   The desired state
inline void CTRL_setFlag_enableUserMotorParams(CTRL_Handle handle,const bool bstate)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->bflag_enableUserMotorParams = bstate;

} // end of CTRL_setFlag_enableUserMotorParams() function


//! \brief      Sets the gain values for the specified controller
//! \param[in]  handle    The controller (CTRL) handle
//! \param[in]  ctrlType  The controller type
//! \param[in]  Kp        The Kp gain value, pu
//! \param[in]  Ki        The Ki gain value, pu
//! \param[in]  Kd        The Kd gain value, pu
void CTRL_setGains(CTRL_Handle handle,const CTRL_Type_e ctrlType,
                   const _iq iqKp,const _iq iqKi,const _iq iqKd);


//! \brief      Sets the alpha/beta current (Iab) input vector values in the controller
//! \param[in]  handle      The controller (CTRL) handle
//! \param[in]  pIab_in_pu  The vector of the alpha/beta current input vector values, pu
inline void CTRL_setIab_in_pu(CTRL_Handle handle,const MATH_vec2 *pIab_in_pu)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->Iab_in.aiqvalue[0] = pIab_in_pu->aiqvalue[0];
	pobj->Iab_in.aiqvalue[1] = pIab_in_pu->aiqvalue[1];


} // end of CTRL_setIab_in_pu() function


//! \brief      Sets the alpha/beta filtered current vector values in the controller
//! \param[in]  handle        The controller (CTRL) handle
//! \param[in]  pIab_filt_pu  The vector of the alpha/beta filtered current vector values, pu
void CTRL_setIab_filt_pu(CTRL_Handle handle,const MATH_vec2 *pIab_filt_pu);


//! \brief      Sets the direct current (Id) reference value in the controller
//! \param[in]  handle     The controller (CTRL) handle
//! \param[in]  Id_ref_pu  The direct current reference value, pu
inline void CTRL_setId_ref_pu(CTRL_Handle handle,const _iq iqId_ref_pu)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->Idq_ref.aiqvalue[0] = iqId_ref_pu;


} // end of CTRL_setId_ref_pu() function


//! \brief      Sets the direct/quadrature current (Idq) input vector values in the controller
//! \param[in]  handle      The controller (CTRL) handle
//! \param[in]  pIdq_in_pu  The vector of the direct/quadrature current input vector values, pu
inline void CTRL_setIdq_in_pu(CTRL_Handle handle,const MATH_vec2 *pIdq_in_pu)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->Idq_in.aiqvalue[0] = pIdq_in_pu->aiqvalue[0];
	pobj->Idq_in.aiqvalue[1] = pIdq_in_pu->aiqvalue[1];


} // end of CTRL_setIdq_in_pu() function


//! \brief      Sets the direct/quadrature current (Idq) reference vector values in the controller
//! \param[in]  handle       The controller (CTRL) handle
//! \param[in]  pIdq_ref_pu  The vector of the direct/quadrature current reference vector values, pu
inline void CTRL_setIdq_ref_pu(CTRL_Handle handle,const MATH_vec2 *pIdq_ref_pu)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->Idq_ref.aiqvalue[0] = pIdq_ref_pu->aiqvalue[0];
	pobj->Idq_ref.aiqvalue[1] = pIdq_ref_pu->aiqvalue[1];


} // end of CTRL_setIdq_ref_pu() function


//! \brief      Sets the Id rated current value in the controller
//! \param[in]  handle      The controller (CTRL) handle
//! \param[in]  IdRated_pu  The Id rated current value, pu
inline void CTRL_setIdRated_pu(CTRL_Handle handle,const _iq iqIdRated_pu)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->iqIdRated = iqIdRated_pu;


} // end of CTRL_setIdRated_pu() function


//! \brief      Sets the quadrature current (Iq) reference value in the controller
//! \param[in]  handle     The controller (CTRL) handle
//! \param[in]  IqRef_pu  The quadrature current reference value, pu
inline void CTRL_setIq_ref_pu(CTRL_Handle handle,const _iq iqIqRef_pu)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->Idq_ref.aiqvalue[1] = iqIqRef_pu;


} // end of CTRL_setIq_ref_pu() function


//! \brief      Sets the derivative gain (Kd) value for the specified controller
//! \param[in]  handle    The controller (CTRL) handle
//! \param[in]  ctrlType  The controller type
//! \param[in]  Kd        The Kd value
inline void CTRL_setKd(CTRL_Handle handle,const CTRL_Type_e ctrlType,const _iq iqKd)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

  	if(ctrlType == CTRL_Type_PID_spd)
    {
  		pobj->iqKd_spd = iqKd;
    }
  	else if(ctrlType == CTRL_Type_PID_Id)
    {
  		pobj->iqKd_Id = iqKd;
    }
  	else if(ctrlType == CTRL_Type_PID_Iq)
    {
  		pobj->iqKd_Iq = iqKd;
    }

} // end of CTRL_setKd() function


//! \brief      Sets the integral gain (Ki) value for the specified controller
//! \param[in]  handle    The controller (CTRL) handle
//! \param[in]  ctrlType  The controller type
//! \param[in]  Ki        The Ki value
inline void CTRL_setKi(CTRL_Handle handle,const CTRL_Type_e ctrlType,const _iq iqKi)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	if(ctrlType == CTRL_Type_PID_spd)
    {
		pobj->iqKi_spd = iqKi;
    }
	else if(ctrlType == CTRL_Type_PID_Id)
    {
		pobj->iqKi_Id = iqKi;
    }
	else if(ctrlType == CTRL_Type_PID_Iq)
    {
		pobj->iqKi_Iq = iqKi;
    }


} // end of CTRL_setKi() function


//! \brief      Sets the proportional gain (Kp) value for the specified controller
//! \param[in]  handle    The controller (CTRL) handle
//! \param[in]  ctrlType  The controller type
//! \param[in]  Kp        The Kp value
inline void CTRL_setKp(CTRL_Handle handle,const CTRL_Type_e ctrlType,const _iq iqKp)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

  	  if(ctrlType == CTRL_Type_PID_spd)
  	  {
  		  pobj->iqKp_spd = iqKp;
  	  }
  	  else if(ctrlType == CTRL_Type_PID_Id)
  	  {
  		  pobj->iqKp_Id = iqKp;
  	  }
  	  else if(ctrlType == CTRL_Type_PID_Iq)
  	  {
  		  pobj->iqKp_Iq = iqKp;
  	  }

  return;
} // end of CTRL_setKp() function


//! \brief      Sets the high frequency inductance (Lhf) value in the controller
//! \param[in]  handle  The controller (CTRL) handle
//! \param[in]  Lhf     The Lhf value
inline void CTRL_setLhf(CTRL_Handle handle,const float_t fLhf)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->fLhf = fLhf;


} // end of CTRL_setLhf() function


//! \brief      Sets the magnetizing current value in the controller
//! \param[in]  handle         The controller (CTRL) handle
//! \param[in]  magCurrent_pu  The magnetizing current value, pu
void CTRL_setMagCurrent_pu(CTRL_Handle handle,const _iq iqmagCurrent_pu);


//! \brief      Sets the maximum voltage vector in the controller
//! \param[in]  handle        The controller (CTRL) handle
//! \param[in]  maxVsMag      The maximum voltage vector (value betwen 0 and 4/3)
inline void CTRL_setMaxVsMag_pu(CTRL_Handle handle,const _iq iqmaxVsMag)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->iqmaxVsMag_pu = iqmaxVsMag;


} // end of CTRL_setmaxVsMag_pu() function


//! \brief      Sets the maximum acceleration of the speed controller
//! \details    Sets the maximum acceleration rate of the speed reference.
//! \param[in]  handle       The controller (CTRL) handle
//! \param[in]  maxAccel_pu  The maximum acceleration (value betwen 0 and 1)
inline void CTRL_setMaxAccel_pu(CTRL_Handle handle,const _iq iqmaxAccel_pu)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->traj_spd.iqmaxDelta = iqmaxAccel_pu;


} // end of CTRL_setMaxAccel_pu() function


//! \brief     Sets the maximum speed value in the controller
//! \param[in] handle       The controller (CTRL) handle
//! \param[in] maxSpeed_pu  The maximum speed value, pu
void CTRL_setMaximumSpeed_pu(CTRL_Handle handle,const _iq iqmaxSpeed_pu);


//! \brief     Sets the parameters for the motor in the controller
//! \param[in] handle        The controller (CTRL) handle
//! \param[in] motorType     The motor type
//! \param[in] numPolePairs  The number of motor pole pairs
//! \param[in] ratedFlux     The rated flux value, V*sec
//! \param[in] Ls_d          The direct stator inductance, Henry
//! \param[in] Ls_q          The quadrature stator inductance, Henry
//! \param[in] Rr            The rotor resistance, ohm
//! \param[in] Rs            The stator resitance, ohm
inline void CTRL_setMotorParams(CTRL_Handle handle,
                                const MOTOR_Type_e motorType,
                                const uint_least16_t u16numPolePairs,
                                const float_t fratedFlux,
                                const float_t fLs_d,
                                const float_t fLs_q,
                                const float_t fRr,
                                const float_t fRs)
{
    //$TI rule:easy to separate the object in the function or outside  the function
	CTRL_Obj *pobj = (CTRL_Obj *)handle;//$ equal to{ CTRL_Obj *pobj = handle }-->let the pobj point to the addess that the handle is point to

	pobj->motorParams.type = motorType;
	pobj->motorParams.u16numPolePairs = u16numPolePairs;
	pobj->motorParams.fratedFlux_VpHz = fratedFlux;
	pobj->motorParams.fLs_d_H = fLs_d;
	pobj->motorParams.fLs_q_H = fLs_q;
	pobj->motorParams.fRr_Ohm = fRr;
	pobj->motorParams.fRs_Ohm = fRs;


} // end of CTRL_setMotorParams() function


//! \brief      Sets the controller parameters
//! \details    This function allows for updates in scale factors during real-time
//!             operation of the controller.
//! \param[in]  handle       The controller (CTRL) handle
//! \param[in]  pUserParams  The pointer to the user parameters
void CTRL_setParams(CTRL_Handle handle,USER_Params *pUserParams);


//! \brief     Sets the number of controller clock ticks per current controller clock tick
//! \param[in] handle                      The controller (CTRL) handle
//! \param[in] numCtrlTicksPerCurrentTick  The number of controller clock ticks per estimator clock tick
inline void CTRL_setNumCtrlTicksPerCurrentTick(CTRL_Handle handle,
                                               const uint_least16_t u16numCtrlTicksPerCurrentTick)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->u16numCtrlTicksPerCurrentTick = u16numCtrlTicksPerCurrentTick;

} // end of CTRL_setNumCtrlTicksPerCurrentTick() function


//! \brief     Sets the number of controller clock ticks per speed controller clock tick
//! \param[in] handle                    The controller (CTRL) handle
//! \param[in] numCtrlTicksPerSpeedTick  The number of controller clock ticks per speed clock tick
inline void CTRL_setNumCtrlTicksPerSpeedTick(CTRL_Handle handle,
                                               const uint_least16_t u16numCtrlTicksPerSpeedTick)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->u16numCtrlTicksPerSpeedTick = u16numCtrlTicksPerSpeedTick;

} // end of CTRL_setNumCtrlTicksPerSpeedTick() function


//! \brief     Sets the number of controller clock ticks per trajectory clock tick
//! \param[in] handle                   The controller (CTRL) handle
//! \param[in] numCtrlTicksPerTrajTick  The number of controller clock ticks per trajectory clock tick
inline void CTRL_setNumCtrlTicksPerTrajTick(CTRL_Handle handle,
                                               const uint_least16_t u16numCtrlTicksPerTrajTick)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->u16numCtrlTicksPerTrajTick = u16numCtrlTicksPerTrajTick;


} // end of CTRL_setNumCtrlTicksPerTrajTick() function


//! \brief     Sets the number of Interrupt Service Routine (ISR) clock ticks per controller clock tick
//! \param[in] handle                  The controller (CTRL) handle
//! \param[in] numIsrTicksPerCtrlTick  The number of Interrupt Service Routine (ISR) clock ticks per controller clock tick
inline void CTRL_setNumIsrTicksPerCtrlTick(CTRL_Handle handle,
                                           const uint_least16_t u16numIsrTicksPerCtrlTick)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->u16numIsrTicksPerCtrlTick = u16numIsrTicksPerCtrlTick;


} // end of CTRL_setNumIsrTicksPerCtrlTick() function


//! \brief     Sets the high frequency resistance (Rhf) value in the controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] Rhf     The Rhf value
inline void CTRL_setRhf(CTRL_Handle handle,const float_t fRhf)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->fRhf = fRhf;


} // end of CTRL_setRhf() function


//! \brief     Sets the R/L value in the controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] RoverL  The R/L value
inline void CTRL_setRoverL(CTRL_Handle handle,const float_t fRoverL)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->fRoverL = fRoverL;


} // end of CTRL_setRoverL() function


//! \brief     Sets the maximum speed value in the controller
//! \param[in] handle     The controller (CTRL) handle
//! \param[in] maxSpd_pu  The maximum speed value, pu
inline void CTRL_setSpd_max_pu(CTRL_Handle handle,const _iq iqmaxSpd_pu)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->iqspd_max = iqmaxSpd_pu;


} // end of CTRL_setSpd_max_pu() function


//! \brief     Sets the output speed value in the controller
//! \param[in] handle      The controller (CTRL) handle
//! \param[in] spd_out_pu  The output speed value, pu
inline void CTRL_setSpd_out_pu(CTRL_Handle handle,const _iq iqspd_out_pu)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->iqspd_out = iqspd_out_pu;


} // end of CTRL_setSpd_out_pu() function


//! \brief     Sets the output speed reference value in the controller
//! \param[in] handle      The controller (CTRL) handle
//! \param[in] spd_ref_pu  The output speed reference value, pu
void CTRL_setSpd_ref_pu(CTRL_Handle handle,const _iq iqspd_ref_pu);


//! \brief     Sets the output speed reference value in the controller
//! \param[in] handle        The controller (CTRL) handle
//! \param[in] spd_ref_krpm  The output speed reference value, krpm
void CTRL_setSpd_ref_krpm(CTRL_Handle handle,const _iq iqspd_ref_krpm);


//! \brief     Sets the controller state
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] state       The new state
inline void CTRL_setState(CTRL_Handle handle,const CTRL_State_e state)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->prevState = pobj->state;
	pobj->state = state;


} // end of CTRL_setState() function


//! \brief      Sets the trajectory execution frequency
//! \param[in]  handle       The controller (CTRL) handle
//! \param[in]  trajFreq_Hz  The trajectory execution frequency, Hz
inline void CTRL_setTrajFreq_Hz(CTRL_Handle handle,const uint_least32_t u32trajFreq_Hz)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->u32trajFreq_Hz = u32trajFreq_Hz;


} // end of CTRL_setTrajFreq_Hz() function


//! \brief      Sets the trajectory execution period
//! \param[in]  handle          The controller (CTRL) handle
//! \param[in]  trajPeriod_sec  The trajectory execution period, sec
inline void CTRL_setTrajPeriod_sec(CTRL_Handle handle,const _iq iqtrajPeriod_sec)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->iqtrajPeriod_sec = iqtrajPeriod_sec;

} // end of CTRL_setTrajPeriod_sec() function


//! \brief     Sets the integrator (Ui) value in the specified controller
//! \param[in] handle  The controller (CTRL) handle
//! \param[in] ctrlType  The controller type
//! \param[in] Ui      The Ui value
inline void CTRL_setUi(CTRL_Handle handle,const CTRL_Type_e ctrlType,const _iq iqUi)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	if(ctrlType == CTRL_Type_PID_spd)
    {
		pobj->iqUi_spd = iqUi;
    }
	else if(ctrlType == CTRL_Type_PID_Id)
    {
		pobj->iqUi_Id = iqUi;
    }
  	else if(ctrlType == CTRL_Type_PID_Iq)
    {
  		pobj->iqUi_Iq = iqUi;
    }

} // end of CTRL_setUi() function


//! \brief      Sets the number of current sensors
//! \details    Different algorithms are used for calculating the Clarke transform
//!             when different number of currents are read in.
//! \param[in]  handle             The controller (CTRL) handle
//! \param[in]  numCurrentSensors  The number of current sensors
void CTRL_setupClarke_I(CTRL_Handle handle,uint_least8_t u8numCurrentSensors);


//! \brief      Sets the number of voltage sensors
//! \details    Different algorithms are used for calculating the Clarke transform
//!             when different number of voltages are read in.
//! \param[in]  handle             The controller (CTRL) handle
//! \param[in]  numVoltageSensors  The number of voltage sensors
void CTRL_setupClarke_V(CTRL_Handle handle,uint_least8_t u8numVoltageSensors);


//! \brief Sets up the controller and trajectory generator for the estimator idle state
//! \param[in] handle  The controller (CTRL) handle
void CTRL_setupEstIdleState(CTRL_Handle handle);


//! \brief     Sets up the controller and trajectory generator for the estimator online state
//! \param[in] handle   The controller (CTRL) handle
void CTRL_setupEstOnLineState(CTRL_Handle handle);


//! \brief Sets the controller and estimator with motor parameters from the user.h file
//! \param[in] handle  The controller (CTRL) handle
void CTRL_setUserMotorParams(CTRL_Handle handle);


//! \brief     Sets the alpha/beta voltage input vector values in the controller
//! \param[in] handle      The controller (CTRL) handle
//! \param[in] pVab_in_pu  The vector of alpha/beta voltage input vector values, pu
inline void CTRL_setVab_in_pu(CTRL_Handle handle,const MATH_vec2 *pVab_in_pu)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->Vab_in.aiqvalue[0] = pVab_in_pu->aiqvalue[0];
	pobj->Vab_in.aiqvalue[1] = pVab_in_pu->aiqvalue[1];


} // end of CTRL_setVab_in_pu() function


//! \brief     Sets the alpha/beta current output vector values in the controller
//! \param[in] handle       The controller (CTRL) handle
//! \param[in] pVab_out_pu  The vector of alpha/beta current output vector values, pu
inline void CTRL_setVab_out_pu(CTRL_Handle handle,const MATH_vec2 *pVab_out_pu)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->Vab_out.aiqvalue[0] = pVab_out_pu->aiqvalue[0];
	pobj->Vab_out.aiqvalue[1] = pVab_out_pu->aiqvalue[1];


} // end of CTRL_setVab_out_pu() function


//! \brief     Sets the direct/quadrature voltage output vector values in the controller
//! \param[in] handle       The controller (CTRL) handle
//! \param[in] pVdq_out_pu  The vector of direct/quadrature voltage output vector values, pu
inline void CTRL_setVdq_out_pu(CTRL_Handle handle,const MATH_vec2 *pVdq_out_pu)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	pobj->Vdq_out.aiqvalue[0] = pVdq_out_pu->aiqvalue[0];
	pobj->Vdq_out.aiqvalue[1] = pVdq_out_pu->aiqvalue[1];


} // end of CTRL_setVdq_out_pu() function


//! \brief     Sets the wait times for the controller states
//! \param[in] handle      The controller (CTRL) handle
//! \param[in] pWaitTimes  A pointer to a vector of wait times, controller clock counts
void CTRL_setWaitTimes(CTRL_Handle handle,const uint_least32_t *pu32WaitTimes);


//! \brief     Sets up the controller (CTRL) object and all of the subordinate objects
//! \details    Is responsible for updating the CTRL state machine and must be called
//!             in the same timing sequence as CTRL_run().
//! \param[in] handle   The controller (CTRL) handle
void CTRL_setup(CTRL_Handle handle);


//! \brief     Sets up the controllers
//! \param[in] handle   The controller (CTRL) handle
void CTRL_setupCtrl(CTRL_Handle handle);


//! \brief     Sets up the estimator (CTRL) object
//! \param[in] handle   The controller (CTRL) handle
void CTRL_setupEst(CTRL_Handle handle);


//! \brief     Sets up the trajectory (TRAJ) object
//! \param[in] handle   The controller (CTRL) handle
void CTRL_setupTraj(CTRL_Handle handle);


//! \brief      Updates the controller state
//! \param[in]  handle  The controller (CTRL) handle
//! \return     A boolean value denoting if the state has changed (true) or not (false)
bool CTRL_updateState(CTRL_Handle handle);


//! \brief     Determines if a zero Iq current reference should be used in the controller
//! \param[in] handle   The controller (CTRL) handle
//! \return    A boolean value denoting if a zero Iq current reference should be used (true) or not (false)
inline bool CTRL_useZeroIq_ref(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(EST_useZeroIq_ref(pobj->estHandle));
} // end of CTRL_useZeroIq_ref() function


//! \brief      Checks for any controller errors and, if found, sets the controller state to the error state
//! \param[in]  handle  The controller (CTRL) handle
inline void CTRL_checkForErrors(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	// check for estimator errors
	if(EST_isError(pobj->estHandle))
    {
		// set the next controller state
		CTRL_setState(handle,CTRL_State_Error);

		// set the error code
		CTRL_setErrorCode(handle,CTRL_ErrorCode_EstError);
    }


} // end of CTRL_checkForErrors() function


//! \brief      Computes a phasor for a given angle
//! \param[in]  angle_pu  The angle, pu
//! \param[out] pPhasor   The pointer to the phasor vector values
inline void CTRL_computePhasor(const _iq angle_pu,MATH_vec2 *pPhasor)
{

	pPhasor->aiqvalue[0] = _IQcosPU(angle_pu);
	pPhasor->aiqvalue[1] = _IQsinPU(angle_pu);


} // end of CTRL_computePhasor() function


//! \brief     Determines if the speed controller should be executed
//! \param[in] handle  The controller (CTRL) handle
//! \return    A boolean value denoting if the speed controller should be executed (true) or not (false)
inline bool CTRL_doSpeedCtrl(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;
	bool bresult = false;
  
  	if (CTRL_getFlag_enableSpeedCtrl(handle) &&
        (pobj->u16counter_speed >= pobj->u16numCtrlTicksPerSpeedTick))
    {
      bresult = true;
    }

  	return(bresult);
} // end of CTRL_doSpeedCtrl() function


//! \brief      Runs the offline controller
//! \param[in]  handle     The controller (CTRL) handle
//! \param[in]  halHandle  The hardware abstraction layer (HAL) handle
//! \param[in]  pAdcData   The pointer to the ADC data
//! \param[out] pPwmData   The pointer to the PWM data
inline void CTRL_runOffLine(CTRL_Handle handle,HAL_Handle halHandle,
                            const HAL_AdcData_t *pAdcData,HAL_PwmData_t *pPwmData)
{

	// run offset estimation
	HAL_runOffsetEst(halHandle,pAdcData);

	// create PWM data
	pPwmData->Tabc.aiqvalue[0] = _IQ(0.0);
	pPwmData->Tabc.aiqvalue[1] = _IQ(0.0);
	pPwmData->Tabc.aiqvalue[2] = _IQ(0.0);


} // end of CTRL_runOffLine() function


//! \brief      Sets maximum speed controller output
//! \param[in]  handle  The controller (CTRL) handle
//! \param[in]  spdMax  The maximum allowed output of the speed controller
inline void CTRL_setSpdMax(CTRL_Handle handle, const _iq spdMax)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	// set the maximum output of the speed controller, or Maximum Iq reference
	TRAJ_setIntValue(pobj->trajHandle_spdMax, spdMax);


} // end of CTRL_setSpdMax() function


//! \brief      Runs angle delay compensation
//! \details    This function takes the decimation rates to calculate a phase delay,
//!             and it compensates the delay. This allows the voltage output to do a
//!             better correction of the error.
//! \param[in]  handle    The controller (CTRL) handle
//! \param[in]  angle_pu  The angle delayed
//! \return     The phase delay compensated angle
inline _iq CTRL_angleDelayComp(CTRL_Handle handle, const _iq iqangle_pu)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;
	_iq iqangleDelta_pu = _IQmpy(EST_getFm_pu(pobj->estHandle),_IQ(USER_IQ_FULL_SCALE_FREQ_Hz/(USER_PWM_FREQ_kHz*1000.0)));
	_iq iqangleUncomp_pu = iqangle_pu;
	_iq iqangleCompFactor = _IQ(1.0 + (float_t)USER_NUM_PWM_TICKS_PER_ISR_TICK * (float_t)USER_NUM_ISR_TICKS_PER_CTRL_TICK * ((float_t)USER_NUM_CTRL_TICKS_PER_EST_TICK - 0.5));
	_iq iqangleDeltaComp_pu = _IQmpy(iqangleDelta_pu, iqangleCompFactor);
	uint32_t u32angleMask = ((uint32_t)0xFFFFFFFF >> (32 - GLOBAL_Q));
	_iq iqangleTmp_pu;
	_iq iqangleComp_pu;

	// increment the angle
	iqangleTmp_pu = iqangleUncomp_pu + iqangleDeltaComp_pu;

	// mask the angle for wrap around
	// note: must account for the sign of the angle
	iqangleComp_pu = _IQabs(iqangleTmp_pu) & u32angleMask;

	// account for sign
	if(iqangleTmp_pu < 0)
    {
		iqangleComp_pu = -iqangleComp_pu;
    }

	return(iqangleComp_pu);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////for testing identify process/////////////////////////////////////////////////

//$ \brief      increment the counter of speed ctrl in CTRL_runOnline
//$             which is inside if(CTRL_doSpeedCtrl(handle)){...}
inline void CTRL_u32identifySpdCTRLcnt_add(CTRL_Handle handle)
{
    CTRL_Obj *pobj=(CTRL_Obj *)handle;
    pobj->u32identifySpdCTRLcnt++;
}

//$ \brief      increment the counter of current ctrl in CTRL_runOnline
//$             which is inside if(CTRL_doCurrentCtrl(handle) && EST_doCurrentCtrl(pobj->estHandle)){....}
inline void CTRL_u32identifyCurrentCTRLcnt_add(CTRL_Handle handle)
{
    CTRL_Obj *pobj=(CTRL_Obj *)handle;
    pobj->u32identifyCurrentCTRLcnt++;
}

//$ \brief      get the counter of current ctrl in CTRL_runOnline
//$             which is inside if(CTRL_doCurrentCtrl(handle) && EST_doCurrentCtrl(pobj->estHandle)){....}
//$ \param[out] return the counter to gMotorVars,which can be shown in watch window
inline uint32_t CTRL_getu32identifyCurrentCTRLcnt(CTRL_Handle handle)
{
    CTRL_Obj *pobj = (CTRL_Obj *)handle;
    return(pobj->u32identifyCurrentCTRLcnt);
}
//$ \brief      get the counter of current ctrl in CTRL_runOnline
//$             which is inside if(CTRL_doSpeedCtrl(handle)){...}
//$ \param[out] return the counter to gMotorVars,which can be shown in watch window
inline uint32_t CTRL_getu32identifySpdCTRLcnt(CTRL_Handle handle)
{
    CTRL_Obj *pobj = (CTRL_Obj *)handle;
    return(pobj->u32identifySpdCTRLcnt);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////





//! \brief      Runs the online controller
//! \param[in]  handle    The controller (CTRL) handle
//! \param[in]  pAdcData  The pointer to the ADC data
//! \param[out] pPwmData  The pointer to the PWM data
//$while Motor is Identified &&Rs Recal Disabled &&Controller Enabled
inline void CTRL_runOnLine(CTRL_Handle handle,
                           const HAL_AdcData_t *pAdcData,HAL_PwmData_t *pPwmData)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	_iq iqangle_pu;

	MATH_vec2 phasor;

	//static uint32_t u32identifyCurrentCTRL=0;
	//static uint32_t u32identifySpdCTRL=0;


	// run Clarke transform on current
	CLARKE_run(pobj->clarkeHandle_I,&pAdcData->I,CTRL_getIab_in_addr(handle));
  

	// run Clarke transform on voltage
	CLARKE_run(pobj->clarkeHandle_V,&pAdcData->V,CTRL_getVab_in_addr(handle));


	// run the estimator
	EST_run(pobj->estHandle,CTRL_getIab_in_addr(handle),CTRL_getVab_in_addr(handle),
			pAdcData->iqdcBus,TRAJ_getIntValue(pobj->trajHandle_spd));


	// generate the motor electrical angle
	iqangle_pu = EST_getAngle_pu(pobj->estHandle);


	// compute the sin/cos phasor
	CTRL_computePhasor(iqangle_pu,&phasor);


	// set the phasor in the Park transform
	PARK_setPhasor(pobj->parkHandle,&phasor);


	// run the Park transform
	PARK_run(pobj->parkHandle,CTRL_getIab_in_addr(handle),CTRL_getIdq_in_addr(handle));

 
	// when appropriate, run the PID speed controller
	if(EST_doSpeedCtrl(pobj->estHandle))
	{


		if(CTRL_doSpeedCtrl(handle))
		{
			_iq iqrefValue = TRAJ_getIntValue(pobj->trajHandle_spd);
         	_iq iqfbackValue = EST_getFm_pu(pobj->estHandle);
         	_iq iqoutMax = TRAJ_getIntValue(pobj->trajHandle_spdMax);
         	_iq iqoutMin = -iqoutMax;

         	//$for testing
         	CTRL_u32identifySpdCTRLcnt_add(handle);


         	// reset the speed count
         	CTRL_resetCounter_speed(handle);

         	PID_setMinMax(pobj->pidHandle_spd,iqoutMin,iqoutMax);

         	PID_run_spd(pobj->pidHandle_spd,iqrefValue,iqfbackValue,CTRL_getSpd_out_addr(handle));
		}
	}
	else
	{
		// zero the speed command
		CTRL_setSpd_out_pu(handle,_IQ(0.0));

		// reset the integrator
		PID_setUi(pobj->pidHandle_spd,_IQ(0.0));
	}


	// when appropriate, run the PID Id and Iq controllers
	if(CTRL_doCurrentCtrl(handle) && EST_doCurrentCtrl(pobj->estHandle))
	{
		_iq iqKp_Id = CTRL_getKp(handle,CTRL_Type_PID_Id);
		_iq iqKp_Iq = CTRL_getKp(handle,CTRL_Type_PID_Iq);
		_iq iqrefValue;
		_iq iqfbackValue;
		_iq iqoutMin,iqoutMax;
         
		_iq iqmaxVsMag = CTRL_getMaxVsMag_pu(handle);


		//$for testing when or the sequence in identified mode
		CTRL_u32identifyCurrentCTRLcnt_add(handle);


		// reset the current count
		CTRL_resetCounter_current(handle);

		// ***********************************
		// configure and run the Id controller

		// compute the Kp gain
		// Scale Kp instead of output to prevent saturation issues
		if(CTRL_getFlag_enableDcBusComp(handle))
       {
			iqKp_Id = _IQmpy(iqKp_Id,EST_getOneOverDcBus_pu(pobj->estHandle));
       }

		PID_setKp(pobj->pidHandle_Id,iqKp_Id);

		// compute the reference value
		iqrefValue = TRAJ_getIntValue(pobj->trajHandle_Id) + CTRL_getId_ref_pu(handle);

		// update the Id reference value
		EST_updateId_ref_pu(pobj->estHandle,&iqrefValue);

		// get the feedback value
		iqfbackValue = CTRL_getId_in_pu(handle);

		// set minimum and maximum for Id controller output
		iqoutMax = iqmaxVsMag;
		iqoutMin = -iqoutMax;

		// set the minimum and maximum values
		PID_setMinMax(pobj->pidHandle_Id,iqoutMin,iqoutMax);

		// run the Id PID controller
		PID_run(pobj->pidHandle_Id,iqrefValue,iqfbackValue,CTRL_getVd_out_addr(handle));

		// set the Id reference value in the estimator
		EST_setId_ref_pu(pobj->estHandle,iqrefValue);

		// ***********************************
		// configure and run the Iq controller

		// compute the Kp gain
     	// Scale Kp instead of output to prevent saturation issues
     	if(CTRL_getFlag_enableDcBusComp(handle))
     	{
     		iqKp_Iq = _IQmpy(iqKp_Iq,EST_getOneOverDcBus_pu(pobj->estHandle));
     	}

     	PID_setKp(pobj->pidHandle_Iq,iqKp_Iq);

     	// get the reference value
     	if(CTRL_getFlag_enableSpeedCtrl(handle))
     	{
     		if(!CTRL_useZeroIq_ref(handle))
     		{
     			iqrefValue = CTRL_getSpd_out_pu(handle);
     		}
     		else
     		{
     			iqrefValue = _IQ(0.0);
     		}
       }
     	else
       {
     		// get the Iq reference value
     		iqrefValue = CTRL_getIq_ref_pu(handle);
       }

     	// get the feedback value
     	iqfbackValue = CTRL_getIq_in_pu(handle);

     	// generate the Iq PID output limits without square root
    	iqoutMax = iqmaxVsMag - _IQabs(CTRL_getVd_out_pu(handle));
    	iqoutMin = -iqoutMax;

    	// set the minimum and maximum values
    	PID_setMinMax(pobj->pidHandle_Iq,iqoutMin,iqoutMax);

    	// run the Iq PID controller
    	PID_run(pobj->pidHandle_Iq,iqrefValue,iqfbackValue,CTRL_getVq_out_addr(handle));

    	// set the Iq reference value in the estimator
    	EST_setIq_ref_pu(pobj->estHandle,iqrefValue);

	}


	// set the phasor in the inverse Park transform
	IPARK_setPhasor(pobj->iparkHandle,&phasor);


	// run the inverse Park module
	IPARK_run(pobj->iparkHandle,CTRL_getVdq_out_addr(handle),CTRL_getVab_out_addr(handle));


	// run the space Vector Generator (SVGEN) module
	SVGEN_run(pobj->svgenHandle,CTRL_getVab_out_addr(handle),&(pPwmData->Tabc));

} // end of CTRL_runOnLine() function


//! \brief      Runs the online user controller
//! \details    An implementation of the field oriented control.  The online user controller
//!             is executed in user's memory i.e. RAM/FLASH and can be changed in any way
//!             suited to the user.
//! \param[in]  handle          The controller (CTRL) handle
//! \param[in]  pAdcData        The pointer to the ADC data
//! \param[out] pPwmData        The pointer to the PWM data
//! \param[in]  electricalAngle The electrical angle in Q24
//!
//!
//$                            (gctrlHandle,ghalHandle,&gAdcData,&gPwmData,EST_getAngle_pu(gctrlHandle->estHandle))
inline void CTRL_runOnLine_User(CTRL_Handle handle,
                           const HAL_AdcData_t *pAdcData,HAL_PwmData_t *pPwmData,
                           uint32_t u32electricalAngle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	_iq iqangle_pu;

	MATH_vec2 phasor;


	// run Clarke transform on current
	CLARKE_run(pobj->clarkeHandle_I,&pAdcData->I,CTRL_getIab_in_addr(handle));//$addr-->memory address


	// run Clarke transform on voltage
	CLARKE_run(pobj->clarkeHandle_V,&pAdcData->V,CTRL_getVab_in_addr(handle));


	// run the estimator
	EST_run(pobj->estHandle,CTRL_getIab_in_addr(handle),CTRL_getVab_in_addr(handle),
			pAdcData->iqdcBus,TRAJ_getIntValue(pobj->trajHandle_spd));


	// generate the motor electrical angle
	//iqangle_pu = EST_getAngle_pu(pobj->estHandle);

	// Update electrical angle from the sensor
	iqangle_pu = u32electricalAngle; // org

	// compute the sin/cos phasor {$[sin(theta),cos(theta),where theta=ianglepu]$}
	CTRL_computePhasor(iqangle_pu,&phasor);


	// set the phasor in the Park transform
	PARK_setPhasor(pobj->parkHandle,&phasor);


	// run the Park transform
	PARK_run(pobj->parkHandle,CTRL_getIab_in_addr(handle),CTRL_getIdq_in_addr(handle));


	// when appropriate, run the PID speed controller
	if(CTRL_doSpeedCtrl(handle))
	{
		_iq iqrefValue = TRAJ_getIntValue(pobj->trajHandle_spd);
    	_iq iqfbackValue = EST_getFm_pu(pobj->estHandle);	//???
    	_iq iqoutMax = TRAJ_getIntValue(pobj->trajHandle_spdMax);
    	_iq iqoutMin = -iqoutMax;

    	// reset the speed count
    	CTRL_resetCounter_speed(handle);

    	PID_setMinMax(pobj->pidHandle_spd,iqoutMin,iqoutMax);

    	PID_run_spd(pobj->pidHandle_spd,iqrefValue,iqfbackValue,CTRL_getSpd_out_addr(handle));
   	 }


	// when appropriate, run the PID Id and Iq controllers
	if(CTRL_doCurrentCtrl(handle))
	{
		_iq iqKp_Id = CTRL_getKp(handle,CTRL_Type_PID_Id);
		_iq iqKp_Iq = CTRL_getKp(handle,CTRL_Type_PID_Iq);
		_iq iqrefValue;
		_iq iqfbackValue;
		_iq iqoutMin,iqoutMax;

		// read max voltage vector to set proper limits to current controllers
		_iq iqmaxVsMag = CTRL_getMaxVsMag_pu(handle);


		// reset the current count
		CTRL_resetCounter_current(handle);

		// ***********************************
		// configure and run the Id controller

		// compute the Kp gain
		// Scale Kp instead of output to prevent saturation issues
		if(CTRL_getFlag_enableDcBusComp(handle))
		{
			iqKp_Id = _IQmpy(iqKp_Id,EST_getOneOverDcBus_pu(pobj->estHandle));
		}

		PID_setKp(pobj->pidHandle_Id,iqKp_Id);

		// compute the reference value
		iqrefValue = TRAJ_getIntValue(pobj->trajHandle_Id) + CTRL_getId_ref_pu(handle);

		// update the Id reference value
		EST_updateId_ref_pu(pobj->estHandle,&iqrefValue);

		// get the feedback value
		iqfbackValue = CTRL_getId_in_pu(handle);

		// set minimum and maximum for Id controller output
		iqoutMax = iqmaxVsMag;
		iqoutMin = -iqoutMax;

		// set the minimum and maximum values
		PID_setMinMax(pobj->pidHandle_Id,iqoutMin,iqoutMax);

		// run the Id PID controller
		PID_run(pobj->pidHandle_Id,iqrefValue,iqfbackValue,CTRL_getVd_out_addr(handle));

		// ***********************************
		// configure and run the Iq controller

		// compute the Kp gain
		// Scale Kp instead of output to prevent saturation issues
		if(CTRL_getFlag_enableDcBusComp(handle))
		{
			iqKp_Iq = _IQmpy(iqKp_Iq,EST_getOneOverDcBus_pu(pobj->estHandle));
		}

		PID_setKp(pobj->pidHandle_Iq,iqKp_Iq);

		// get the reference value
		if(CTRL_getFlag_enableSpeedCtrl(handle))
		{
			iqrefValue = CTRL_getSpd_out_pu(handle);
		}
		else
		{
			// get the Iq reference value
			iqrefValue = CTRL_getIq_ref_pu(handle);
		}

		// get the feedback value
		iqfbackValue = CTRL_getIq_in_pu(handle);

		// set minimum and maximum for Id controller output
		iqoutMax = _IQsqrt(_IQmpy(iqmaxVsMag,iqmaxVsMag) - _IQmpy(CTRL_getVd_out_pu(handle),CTRL_getVd_out_pu(handle)));
		iqoutMin = -iqoutMax;

		// set the minimum and maximum values
		PID_setMinMax(pobj->pidHandle_Iq,iqoutMin,iqoutMax);

		// run the Iq PID controller
		PID_run(pobj->pidHandle_Iq,iqrefValue,iqfbackValue,CTRL_getVq_out_addr(handle));
	}

	{
		_iq iqangleComp_pu;


		// compensate angle delay
		iqangleComp_pu = CTRL_angleDelayComp(handle, iqangle_pu);


		// compute the sin/cos phasor
		CTRL_computePhasor(iqangleComp_pu,&phasor);
	}


   // set the phasor in the inverse Park transform
   IPARK_setPhasor(pobj->iparkHandle,&phasor);


   // run the inverse Park module
   IPARK_run(pobj->iparkHandle,CTRL_getVdq_out_addr(handle),CTRL_getVab_out_addr(handle));


   // run the space Vector Generator (SVGEN) module
   SVGEN_run(pobj->svgenHandle,CTRL_getVab_out_addr(handle),&(pPwmData->Tabc));


} // end of CTRL_runOnLine_User() function


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _CTRL_H_ definition




