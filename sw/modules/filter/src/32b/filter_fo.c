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
//! \file   modules/filter/src/32b/filter_fo.c
//! \brief  Portable C fixed point code.  These functions define the 
//!         first-order filter (FILTER) module routines
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "sw/modules/filter/src/32b/filter_fo.h"



// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions


void FILTER_FO_getDenCoeffs(FILTER_FO_Handle handle,_iq *piqa1)
{
	FILTER_FO_Obj *pobj = (FILTER_FO_Obj *)handle;


	*piqa1 = pobj->iqa1;


} // end of FILTER_FO_getDenCoeffs() function


void FILTER_FO_getInitialConditions(FILTER_FO_Handle handle,_iq *piqx1,_iq *piqy1)
{
	FILTER_FO_Obj *pobj = (FILTER_FO_Obj *)handle;


	*piqx1 = pobj->iqx1;

	*piqy1 = pobj->iqy1;


} // end of FILTER_FO_getInitialConditions() function


void FILTER_FO_getNumCoeffs(FILTER_FO_Handle handle,_iq *piqb0,_iq *piqb1)
{
	FILTER_FO_Obj *pobj = (FILTER_FO_Obj *)handle;


	*piqb0 = pobj->iqb0;
	*piqb1 = pobj->iqb1;


} // end of FILTER_FO_getNumCoeffs() function


FILTER_FO_Handle FILTER_FO_init(void *pMemory,const size_t numBytes)
{
	FILTER_FO_Handle handle;


	if(numBytes < sizeof(FILTER_FO_Obj))
		return((FILTER_FO_Handle)NULL);

	// assign the handle
	handle = (FILTER_FO_Handle)pMemory;

	return(handle);
} // end of FILTER_FO_init() function


void FILTER_FO_setDenCoeffs(FILTER_FO_Handle handle,const _iq iqa1)
{
 	FILTER_FO_Obj *pobj = (FILTER_FO_Obj *)handle;


 	pobj->iqa1 = iqa1;


} // end of FILTER_FO_setDenCoeffs() function


void FILTER_FO_setInitialConditions(FILTER_FO_Handle handle,const _iq iqx1,const _iq iqy1)
{
	FILTER_FO_Obj *pobj = (FILTER_FO_Obj *)handle;


	pobj->iqx1 = iqx1;

	pobj->iqy1 = iqy1;

} // end of FILTER_FO_setInitialConditions() function


void FILTER_FO_setNumCoeffs(FILTER_FO_Handle handle,const _iq iqb0,const _iq iqb1)
{
	FILTER_FO_Obj *pobj = (FILTER_FO_Obj *)handle;


	pobj->iqb0 = iqb0;
	pobj->iqb1 = iqb1;


} // end of FILTER_FO_setNumCoeffs() function


// end of file



