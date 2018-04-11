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
#ifndef _PARK_H_
#define _PARK_H_

//! \file   modules/park/src/32b/park.h
//! \brief  Contains the public interface to the 
//!         Park transform module routines 
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "sw/modules/iqmath/src/32b/IQmathLib.h"
#include "sw/modules/math/src/32b/math.h"
#include "sw/modules/types/src/types.h"

//!
//!
//! \defgroup PARK PARK
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup PARK_OVERVIEW 

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines



// **************************************************************************
// the typedefs

//! \brief Defines the PARK object
//!
typedef struct _PARK_Obj_
{

  _iq  iqsinTh;      //!< the sine of the angle between the d,q and the alpha,beta coordinate systems
  _iq  iqcosTh;      //!< the cosine of the angle between the d,q and the alpha,beta coordinate systems

} PARK_Obj;


//! \brief Defines the PARK handle
//!
typedef struct _PARK_Obj_ *PARK_Handle;


// **************************************************************************
// the function prototypes

//! \brief     Gets the cosine of the angle between the d,q and the alpha,beta coordinate systems
//! \param[in] handle  The Park transform handle
//! \return    The cosine of the angle
static inline _iq PARK_getCosTh(PARK_Handle handle)
{
	PARK_Obj *pobj = (PARK_Obj *)handle;

	return(pobj->iqcosTh);
} // end of PARK_getCosTh() function


//! \brief     Gets the cosine/sine phasor for the Park transform
//! \param[in] handle   The Park transform handle
//! \param[in] pPhasor      The pointer to the cosine/sine phasor
static inline void PARK_getPhasor(PARK_Handle handle,MATH_vec2 *pPhasor)
{
	PARK_Obj *pobj = (PARK_Obj *)handle;

	pPhasor->aiqvalue[0] = pobj->iqcosTh;
	pPhasor->aiqvalue[1] = pobj->iqsinTh;


} // end of PARK_getPhasor() function


//! \brief     Gets the sine of the angle between the d,q and the alpha,beta coordinate systems
//! \param[in] handle  The Park transform handle
//! \return    The sine of the angle
static inline _iq PARK_getSinTh(PARK_Handle handle)
{
	PARK_Obj *pobj = (PARK_Obj *)handle;

	return(pobj->iqsinTh);
} // end of PARK_getSinTh() function


//! \brief     Initializes the Park transform module
//! \param[in] pMemory      A pointer to the memory for the Park object
//! \param[in] numBytes     The number of bytes allocated for the Park object, bytes
//! \return The Park (PARK) object handle
extern PARK_Handle PARK_init(void *pMemory,const size_t numBytes);


//! \brief     Runs the Park transform module
//! \param[in] handle  The Park transform handle
//! \param[in] pInVec      The pointer to the input vector
//! \param[in] pOutVec     The pointer to the output vector


//$the input               (pobj->parkHandle,CTRL_getIab_in_addr(handle),CTRL_getIdq_in_addr(handle))
static inline void PARK_run(PARK_Handle handle,const MATH_vec2 *pInVec,MATH_vec2 *pOutVec)
{
	PARK_Obj *pobj = (PARK_Obj *)handle;

	_iq iqsinTh = pobj->iqsinTh;
	_iq iqcosTh = pobj->iqcosTh;

	_iq iqvalue_0 = pInVec->aiqvalue[0];//$ the output of clarke transform
	_iq iqvalue_1 = pInVec->aiqvalue[1];


	pOutVec->aiqvalue[0] = _IQmpy(iqvalue_0,iqcosTh) + _IQmpy(iqvalue_1,iqsinTh);
	pOutVec->aiqvalue[1] = _IQmpy(iqvalue_1,iqcosTh) - _IQmpy(iqvalue_0,iqsinTh);
	//$ |I_d|=|cos  sin||I_alpha|   --->matrix form
	//$ |I_q|=|-sin cos||I_beta]|

} // end of PARK_run() function


//! \brief     Sets the cosine of the angle between the d,q and the alpha,beta coordinate systems
//! \param[in] handle  The Park transform handle
//! \param[in] cosTh       The cosine of the angle
static inline void PARK_setCosTh(PARK_Handle handle,const _iq iqcosTh)
{
	PARK_Obj *pobj = (PARK_Obj *)handle;

	pobj->iqcosTh = iqcosTh;


} // end of PARK_setCosTh() function


//! \brief     Sets the cosine/sine phasor for the inverse Park transform
//! \param[in] handle   The Park transform handle
//! \param[in] pPhasor      The pointer to the cosine/sine phasor, pu
static inline void PARK_setPhasor(PARK_Handle handle,const MATH_vec2 *pPhasor)
{
	PARK_Obj *pobj = (PARK_Obj *)handle;

	pobj->iqcosTh = pPhasor->aiqvalue[0];
	pobj->iqsinTh = pPhasor->aiqvalue[1];


} // end of PARK_setPhasor() function


//! \brief     Sets the sine of the angle between the d,q and the alpha,beta coordinate systems
//! \param[in] handle  The Park transform handle
//! \param[in] sinTh       The sine of the angle
static inline void PARK_setSinTh(PARK_Handle handle,const _iq iqsinTh)
{
	PARK_Obj *pobj = (PARK_Obj *)handle;

	pobj->iqsinTh = iqsinTh;


} // end of PARK_setSinTh() function


//! \brief     Sets up the Park transform module
//! \param[in] handle  The Park transform handle
//! \param[in] angle_pu    The angle between the d,q and the alpha,beta coordinate systems, pu
static inline void PARK_setup(PARK_Handle handle,const _iq iqangle_pu)
{
	PARK_Obj *pobj = (PARK_Obj *)handle;

	pobj->iqsinTh = _IQsinPU(iqangle_pu);
	pobj->iqcosTh = _IQcosPU(iqangle_pu);

} // end of PARK_setup() function


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _PARK_H_ definition

