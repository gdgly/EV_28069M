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
#ifndef _SVGEN_H_
#define _SVGEN_H_

//! \file   modules/svgen/src/32b/svgen.h
//! \brief  Contains the public interface to the 
//!         Space Vector Generator (SVGEN) module routines 
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "sw/modules/iqmath/src/32b/IQmathLib.h"
#include "sw/modules/types/src/types.h"
#include "sw/modules/math/src/32b/math.h"

//!
//!
//! \defgroup SVGEN SVGEN
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup SVGEN_OVERVIEW 


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

//! \brief Defines the maximum reference alpha and beta voltages in PU
//! \brief 2/sqrt(3)
#define SVGEN_MAX_VAB_VOLTAGES                _IQ(1.1547005384)

//! \brief Defines 100% modulation for Q15
//! \brief 1 - 2^(-15)
#define SVGEN_100_PERCENT_MODULATION          _IQ(1.0 - 1.0/32768)

//! \brief Defines sqrt(3)/2
//!
#define SVGEN_SQRT3_OVER_2                    _IQ(0.8660254038)   

//! \brief Defines 4/3
//!
#define SVGEN_4_OVER_3                        _IQ(4.0/3.0)

// **************************************************************************
// the typedefs

//! \brief Defines the Space Vector Generator object
//!
typedef struct _SVGEN_Obj_
{
  _iq   iqmaxModulation;        //!< The maximum modulation magnitude used in the SVM
} SVGEN_Obj;


//! \brief Defines the SVGEN handle
//!
typedef struct _SVGEN_Obj_ *SVGEN_Handle;


// **************************************************************************
// the function prototypes

//! \brief     Gets the maximum modulation used in the space vector generator module
//! \param[in] handle  The space vector generator (SVGEN) handle
//! \return    The maximum modulation
static inline _iq SVGEN_getMaxModulation(SVGEN_Handle handle)
{
	SVGEN_Obj *pobj = (SVGEN_Obj *)handle;

	return(pobj->iqmaxModulation);
} // end of SVGEN_getMaxModulation() function


//! \brief     Initializes the space vector generator module
//! \param[in] pMemory   A pointer to the space vector generator object memory
//! \param[in] numBytes  The number of bytes allocated for the space vector generator object, bytes
//! \return The state vector generator (SVGEN) object handle
extern SVGEN_Handle SVGEN_init(void *pMemory,const size_t numBytes);


//! \brief	Implements a SVM that saturates at the level of MaxModulation.
//! \param[in] handle  The space vector generator (SVGEN) handle
//! \param[in] pVab    The pointer to the a/b voltages
//! \param[in] pT      The pointer to the pwm duty cycle time durations
static inline void SVGEN_run(SVGEN_Handle handle,const MATH_vec2 *pVab,MATH_vec3 *pT)
{

	_iq iqVmax,iqVmin,iqVcom;
	_iq iqVa,iqVb,iqVc;
	_iq iqVa_tmp = -(pVab->aiqvalue[0]>>1);
	_iq iqVb_tmp = _IQmpy(SVGEN_SQRT3_OVER_2,pVab->aiqvalue[1]);
  
	iqVa = pVab->aiqvalue[0];  //alpha
	iqVb = iqVa_tmp + iqVb_tmp; //-0.5*alpha + sqrt(3)/2 * beta;
	iqVc = iqVa_tmp - iqVb_tmp; //-0.5*alpha - sqrt(3)/2 * beta;
	
	//iqVmax=0;
	//iqVmin=0;

	// find order Vmin,Vmid,Vmax
	if (iqVa > iqVb)
	{
		iqVmax = iqVa;
		iqVmin = iqVb;
	}
	else
	{
		iqVmax = iqVb;
		iqVmin = iqVa;
	}

	if (iqVc > iqVmax)
	{
		iqVmax = iqVc;
	}
	else if (iqVc < iqVmin)
	{
		iqVmin = iqVc;
	}
		
	iqVcom = _IQmpy(iqVmax+iqVmin, _IQ(0.5));

	// Subtract common-mode term to achieve SV modulation
	pT->aiqvalue[0] = (iqVa - iqVcom);
	pT->aiqvalue[1] = (iqVb - iqVcom);
	pT->aiqvalue[2] = (iqVc - iqVcom);
  

} // end of SVGEN_run() function


//! \brief     Sets the maximum modulation in the space vector generator module
//! \param[in] handle         The space vector generator (SVGEN) handle
//! \param[in] maxModulation  The maximum modulation
static inline void SVGEN_setMaxModulation(SVGEN_Handle handle,const _iq iqmaxModulation)
{
	SVGEN_Obj *pobj = (SVGEN_Obj *)handle;
	_iq iqmaxMod = iqmaxModulation;

	if(iqmaxMod > SVGEN_4_OVER_3)
		iqmaxMod = SVGEN_4_OVER_3;
	else if(iqmaxMod < _IQ(0.0))
		iqmaxMod = _IQ(0.0);

	pobj->iqmaxModulation = iqmaxMod;


} // end of SVGEN_setMaxModulation() function


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _SVGEN_H_ definition

