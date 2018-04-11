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
#ifndef _OFFSET_H_
#define _OFFSET_H_

//! \file   modules/offset/src/32b/offset.h
//! \brief  Contains the public interface to the 
//!         offset (OFFSET) module routines
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the module includes

// modules
#include "sw/modules/types/src/types.h"
#include "sw/modules/iqmath/src/32b/IQmathLib.h"
#include "sw/modules/filter/src/32b/filter_fo.h"


//!
//!
//! \defgroup OFFSET OFFSET
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs
 
//! \brief Defines the offset (OFFSET) object
//!

typedef struct _ValueFilter_
{

  _iq                iqValue;          //!< the offset value (iq)
   FILTER_FO_Handle   filterHandle;   //!< the first order filter handle.

} ValueFilter_Obj;
#endif                                                         

//! \brief Defines the OFFSET handle
//!
typedef struct _ValueFilter_ *ValueFilter_Handle;
                                                         

// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


//! \brief     Gets the beta offset filter coefficient
//! \param[in] handle  The offset handle
//! \return The filter coefficient beta
extern _iq ValueFilter_getBeta(ValueFilter_Handle handle);


//! \brief     Gets the offset value
//! \param[in] handle  The offset handle
//! \return    The offset value
static inline _iq ValueFilter_getValue(ValueFilter_Handle handle)
{
	ValueFilter_Obj *pobj = (ValueFilter_Obj *)handle;

	return(pobj->iqValue);
} // end of OFFSET_getOffset() function


//! \brief     Initializes the offset
//! \param[in] pMemory   A pointer to the memory for the offset object
//! \param[in] numBytes  The number of bytes allocated for the offset object, bytes
//! \return The offset (OFFSET) object handle
extern ValueFilter_Handle ValueFilter_init(void *pMemory,const size_t numBytes);


//! \brief     Runs an offset filter of the form
//!            y[n] = beta*(x[n]+bias) + (1 - beta)*y[n-1]
//!            y -> The DC offset
//!            x -> The ADC measurement
//!
//! \param[in] handle  The offset handle
//! \param[in] inputValue    The input value to offset filter
static inline void ValueFilter_run(ValueFilter_Handle handle,const _iq iqinputValue)
{
	ValueFilter_Obj *pobj = (ValueFilter_Obj *)handle;

	pobj->iqvalue = FILTER_FO_run(pobj->filterHandle,iqinputValue);

} // end of OFFSET_run() function


//! \brief     Sets the beta offset filter coefficient
//! \param[in] handle  The offset handle
//! \param[in] beta          The offset filter coefficient beta
extern void ValueFilter_setBeta(ValueFilter_Handle handle,const _iq iqbeta);


//! \brief     Set the initial condition of the integrator or the value of y[n-1]
//! \param[in] handle  The offset handle
//! \param[in] initCond      The mean value that the filter will approximate to
extern void ValueFilter_setInitCond(ValueFilter_Handle handle,const _iq iqinitCond);


//! \brief     Sets the offset value
//! \param[in] handle  The offset handle
//! \param[in] offsetValue   The offset value
extern void ValueFilter_setValue(ValueFilter_Handle handle, _iq iqoffsetValue);


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _OFFSET_H_ definition


