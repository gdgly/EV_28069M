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
#ifndef _CLARKE_H_
#define _CLARKE_H_

//! \file   modules/clarke/src/32b/clarke.h
//! \brief  Contains the public interface to the 
//!         Clarke transform (CLARKE) module routines
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

// modules
#include "sw/modules/iqmath/src/32b/IQmathLib.h"
#include "sw/modules/math/src/32b/math.h"
#include "sw/modules/types/src/types.h"

//!
//!
//! \defgroup CLARKE CLARKE
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup CLARKE_OVERVIEW 


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines



// **************************************************************************
// the typedefs

//! \brief Defines the CLARKE object
//!
typedef struct _CLARKE_Obj_
{
  _iq           iqalpha_sf;      //!< the scale factor for the alpha component
  _iq           iqbeta_sf;      //!< the scale factor for the beta component

  uint_least8_t  u8numSensors;   //!< the number of sensors

} CLARKE_Obj;


//! \brief Defines the CLARKE handle
//!
typedef struct _CLARKE_Obj_ *CLARKE_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

//! \brief     Gets the number of sensors
//! \param[in] handle  The Clarke transform handle
//! \return    The number of sensors
static inline uint_least8_t CLARKE_getNumSensors(CLARKE_Handle handle)
{
	CLARKE_Obj *pobj = (CLARKE_Obj *)handle;

	return(pobj->u8numSensors);
} // end of CLARKE_getNumSensors() function


//! \brief     Initializes the Clarke transform module
//! \param[in] pMemory   A pointer to the memory for the Clarke object
//! \param[in] numBytes  The number of bytes allocated for the Clarke object, bytes
//! \return The Clarke (CLARKE) object handle
extern CLARKE_Handle CLARKE_init(void *pMemory,const size_t numBytes);


//! \brief     Runs the Clarke transform module for three inputs
//! \param[in] handle  The Clarke transform handle
//! \param[in] pInVec        The pointer to the input vector
//! \param[in] pOutVec       The pointer to the output vector

//$                          (pobj->clarkeHandle_I,       &pAdcData->I,   CTRL_getIab_in_addr(handle))
static inline void CLARKE_run(CLARKE_Handle handle,const MATH_vec3 *pInVec,MATH_vec2 *pOutVec)
{
	CLARKE_Obj *pobj = (CLARKE_Obj *)handle;
	uint_least8_t u8numSensors = pobj->u8numSensors;

	_iq iqalpha_sf = pobj->iqalpha_sf;//$   [1/3]
	_iq iqbeta_sf = pobj->iqbeta_sf;//$     [1/sqrt(3)]

	if(u8numSensors == 3)   //$
    {
		pOutVec->aiqvalue[0] = _IQmpy(lshft_1(pInVec->aiqvalue[0]) - (pInVec->aiqvalue[1] + pInVec->aiqvalue[2]),iqalpha_sf);//$lshft_1 might for the floating calculation
		pOutVec->aiqvalue[1] = _IQmpy(pInVec->aiqvalue[1] - pInVec->aiqvalue[2],iqbeta_sf);
    }
	else if(u8numSensors == 2)
    {
     	pOutVec->aiqvalue[0] = _IQmpy(pInVec->aiqvalue[0],iqalpha_sf);
     	pOutVec->aiqvalue[1] = _IQmpy(pInVec->aiqvalue[0] + lshft_1(pInVec->aiqvalue[1]),iqbeta_sf);
    }


} // end of CLARKE_run() function


//! \brief     Runs the Clarke transform module for two inputs
//! \param[in] handle  The Clarke transform handle
//! \param[in] pInVec        The pointer to the input vector
//! \param[in] pOutVec       The pointer to the output vector
static inline void CLARKE_run_twoInput(CLARKE_Handle handle,const MATH_vec2 *pInVec,MATH_vec2 *pOutVec)
{
	CLARKE_Obj *pobj = (CLARKE_Obj *)handle;

	_iq iqbeta_sf = pobj->iqbeta_sf;


 	pOutVec->aiqvalue[0] = pInVec->aiqvalue[0];

 	pOutVec->aiqvalue[1] = _IQmpy(pInVec->aiqvalue[0] + _IQmpy2(pInVec->aiqvalue[1]),iqbeta_sf);


} // end of CLARKE_run_twoInput() function


//! \brief     Sets the number of sensors
//! \param[in] handle  The Clarke transform handle
//! \param[in] numSensors    The number of sensors
static inline void CLARKE_setNumSensors(CLARKE_Handle handle,const uint_least8_t u8numSensors)
{
	CLARKE_Obj *pobj = (CLARKE_Obj *)handle;

	pobj->u8numSensors = u8numSensors;


} // end of CLARKE_setNumSensors() function


//! \brief     Sets the scale factors
//! \param[in] handle  The Clarke transform handle
//! \param[in] alpha_sf      The scale factor for the alpha voltage
//! \param[in] beta_sf       The scale factor for the beta voltage

//$ this function will be call at CTRL_setupClarke_V in ctrlQEP.c,which just assign the value into the handle after calculation in CTRL_setupClarke_V
static inline void CLARKE_setScaleFactors(CLARKE_Handle handle,const _iq iqalpha_sf,const _iq iqbeta_sf)
{
	CLARKE_Obj *pobj = (CLARKE_Obj *)handle;


	pobj->iqalpha_sf = iqalpha_sf;
	pobj->iqbeta_sf = iqbeta_sf;


} // end of CLARKE_setScaleFactors() function


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _CLARKE_H_ definition

