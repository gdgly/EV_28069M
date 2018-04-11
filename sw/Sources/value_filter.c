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
//! \file   modules/offset/src/32b/offset.c
//! \brief  Portable C fixed point code.  These functions define the 
//!         offset (OFFSET) module routines
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "value_Filter.h"



// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions

ValueFilter_Handle ValueFilter_init(void *pMemory,const size_t numBytes)
{
	ValueFilter_Obj *pObj;

	if(numBytes < sizeof(ValueFilter_Obj))
		return((ValueFilter_Handle)NULL);

	// assign the handle
	handle = (ValueFilter_Handle)pMemory;
	pObj = (ValueFilter_Obj *)handle;

	pObj->filterHandle = FILTER_FO_init(&(pObj->filter),sizeof(pObj->filter));

	return (handle);
} // end of OFFSET_init() function

_iq ValueFilter_getBeta(ValueFilter_Handle handle)
{
	ValueFilter_Obj *pobj = (ValueFilter_Obj *)handle;
	_iq iqb0;
	_iq iqb1;

 	FILTER_FO_getNumCoeffs(pobj->filterHandle,&iqb0,&iqb1);

 	return(iqb0);
} // end of OFFSET_getBeta() function



void ValueFilter_setBeta(ValueFilter_Handle handle,const _iq iqbeta)
{
	ValueFilter_Obj *pobj = (ValueFilter_Obj *)handle;
	_iq iqa1 = (iqbeta - _IQ(1.0));
	_iq iqb0 = iqbeta;
	_iq iqb1 = 0;

	FILTER_FO_setDenCoeffs(pobj->filterHandle,iqa1);
	FILTER_FO_setNumCoeffs(pobj->filterHandle,iqb0,iqb1);


} // end of OFFSET_setBeta() function


void ValueFilter_setInitCond(ValueFilter_Handle handle,const _iq iqinitCond)
{
	ValueFilter_Obj *pobj = (ValueFilter_Obj *)handle;

	FILTER_FO_setInitialConditions(pobj->filterHandle,iqinitCond,iqinitCond);
	pobj->iqValue = iqinitCond;


} // end of OFFSET_setInitCond() function


void ValueFilter_setValue(ValueFilter_Handle handle, _iq iqoValue)
{
	ValueFilter_Obj *pobj = (ValueFilter_Obj *)handle;

	pobj->iqvalue = iqoValue;

} // end of OFFSET_setOffset() function

// end of file











