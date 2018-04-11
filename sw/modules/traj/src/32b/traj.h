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
#ifndef _TRAJ_H_
#define _TRAJ_H_

//! \file   modules/traj/src/32b/traj.h
//! \brief  Contains public interface to various functions related
//!         to the trajectory (TRAJ) object
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "sw/modules/types/src/types.h"
#include "sw/modules/iqmath/src/32b/IQmathLib.h"

//!
//!
//! \defgroup TRAJ TRAJ
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup TRAJ_OVERVIEW 

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

//! \brief Defines the trajectory (TRAJ) object
//!
typedef struct _TRAJ_Obj_
{
  _iq   iqtargetValue;   //!< the target value for the trajectory
  _iq   iqintValue;      //!< the intermediate value along the trajectory
  _iq   iqminValue;      //!< the minimum value for the trajectory generator
  _iq   iqmaxValue;      //!< the maximum value for the trajectory generator
  _iq   iqmaxDelta;      //!< the maximum delta value for the trajectory generator
} TRAJ_Obj;


//! \brief Defines the TRAJ handle
//!
typedef struct _TRAJ_Obj_ *TRAJ_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

//! \brief     Gets the intermediate value for the trajectory
//! \param[in] handle  The trajectory (TRAJ) handle
//! \return    The intermediate value
static inline _iq TRAJ_getIntValue(TRAJ_Handle handle)
{
	TRAJ_Obj *pobj = (TRAJ_Obj *)handle;

	return(pobj->iqintValue);
} // end of TRAJ_getIntValue() function


//! \brief     Gets the maximum delta value for the trajectory
//! \param[in] handle  The trajectory (TRAJ) handle
//! \return    The maximum delta value
static inline _iq TRAJ_getMaxDelta(TRAJ_Handle handle)
{
	TRAJ_Obj *pobj = (TRAJ_Obj *)handle;

	return(pobj->iqmaxDelta);
} // end of TRAJ_getMaxDelta() function


//! \brief     Gets the maximum value for the trajectory
//! \param[in] handle  The trajectory (TRAJ) handle
//! \return    The maximum value
static inline _iq TRAJ_getMaxValue(TRAJ_Handle handle)
{
	TRAJ_Obj *pobj = (TRAJ_Obj *)handle;

	return(pobj->iqmaxValue);
} // end of TRAJ_getMaxValue() function


//! \brief     Gets the minimum value for the trajectory
//! \param[in] handle  The trajectory (TRAJ) handle
//! \return    The minimum value
static inline _iq TRAJ_getMinValue(TRAJ_Handle handle)
{
	TRAJ_Obj *pobj = (TRAJ_Obj *)handle;

	return(pobj->iqminValue);
} // end of TRAJ_getMinValue() function


//! \brief     Gets the target value for the trajectory
//! \param[in] handle  The trajectory (TRAJ) handle
//! \return    The target value
static inline _iq TRAJ_getTargetValue(TRAJ_Handle handle)
{
	TRAJ_Obj *pobj = (TRAJ_Obj *)handle;

	return(pobj->iqtargetValue);
} // end of TRAJ_getTargetValue() function


//! \brief     Initializes the trajectory (TRAJ) object
//! \param[in] pMemory   A pointer to the memory for the trajectory (TRAJ) object
//! \param[in] numBytes  The number of bytes allocated for the trajectory object, bytes
//! \return The trajectory (TRAJ) object handle
extern TRAJ_Handle TRAJ_init(void *pMemory,const size_t numBytes);


//! \brief     Sets the intermediate value for the trajectory
//! \param[in] handle    The trajectory (TRAJ) handle
//! \param[in] intValue  The intermediate value
static inline void TRAJ_setIntValue(TRAJ_Handle handle,const _iq iqintValue)
{
	TRAJ_Obj *pobj = (TRAJ_Obj *)handle;

	pobj->iqintValue = iqintValue;


} // end of TRAJ_setIntValue() function


//! \brief     Sets the maximum delta value for the trajectory
//! \param[in] handle    The trajectory (TRAJ) handle
//! \param[in] maxDelta  The maximum delta value
static inline void TRAJ_setMaxDelta(TRAJ_Handle handle,const _iq iqmaxDelta)
{
	TRAJ_Obj *pobj = (TRAJ_Obj *)handle;

	pobj->iqmaxDelta = iqmaxDelta;


} // end of TRAJ_setMaxDelta() function


//! \brief     Sets the maximum value for the trajectory
//! \param[in] handle    The trajectory (TRAJ) handle
//! \param[in] maxValue  The maximum value
static inline void TRAJ_setMaxValue(TRAJ_Handle handle,const _iq iqmaxValue)
{
	TRAJ_Obj *pobj = (TRAJ_Obj *)handle;

	pobj->iqmaxValue = iqmaxValue;


} // end of TRAJ_setMaxValue() function


//! \brief     Sets the minimum value for the trajectory
//! \param[in] handle    The trajectory (TRAJ) handle
//! \param[in] minValue  The minimum value
static inline void TRAJ_setMinValue(TRAJ_Handle handle,const _iq iqminValue)
{
	TRAJ_Obj *pobj = (TRAJ_Obj *)handle;

	pobj->iqminValue = iqminValue;


} // end of TRAJ_setMinValue() function


//! \brief     Sets the target value for the trajectory
//! \param[in] handle       The trajectory (TRAJ) handle
//! \param[in] targetValue  The target value
static inline void TRAJ_setTargetValue(TRAJ_Handle handle,const _iq iqtargetValue)
{
	TRAJ_Obj *pobj = (TRAJ_Obj *)handle;

	pobj->iqtargetValue = iqtargetValue;

} // end of TRAJ_setTargetValue() function


//! \brief     Runs the trajectory (TRAJ) object
//! \param[in] handle  The trajectory (TRAJ) handle
static inline void TRAJ_run(TRAJ_Handle handle)
{
	_iq iqtargetValue = TRAJ_getTargetValue(handle);
	_iq iqintValue = TRAJ_getIntValue(handle);
  	_iq iqerror = iqtargetValue - iqintValue;
  	_iq iqmaxDelta = TRAJ_getMaxDelta(handle);
 	_iq iqminValue = TRAJ_getMinValue(handle);
 	_iq iqmaxValue = TRAJ_getMaxValue(handle);

    // increment the value
    iqintValue += _IQsat(iqerror,iqmaxDelta,-iqmaxDelta);

    // bound the value
    iqintValue = _IQsat(iqintValue,iqmaxValue,iqminValue);

    // store the value
    TRAJ_setIntValue(handle,iqintValue);

} // end of TRAJ_run() function


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _TRAJ_H_ definition





