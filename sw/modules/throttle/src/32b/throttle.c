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
 
 
// **************************************************************************
// the includes

// drivers

// modules

// platforms
#include "sw/modules/throttle/src/32b/throttle.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions


extern Throttle_Handle Throttle_init(void *pMemory, const size_t numBytes)
{
	Throttle_Handle handle;

  if(numBytes < sizeof(Throttle_Obj))
    return((Throttle_Handle)NULL);

  // assign the handle
  handle = (Throttle_Handle)pMemory;

  return(handle);
}


//! \brief     Sets up the throttle module parameters initially
//! \param[in] handle  The throttle handle
extern void Throttle_setParams(Throttle_Handle handle,  \
                                const bool bInvert,
                                const _iq iqMax_adc,      \
                                const _iq iqMin_adc,      \
                                const _iq iqMax_out,      \
                                const _iq iqMin_out)
{
	Throttle_Obj *pObj = (Throttle_Obj *)handle;

  pObj->bFlagSw1 = false;
  pObj->bFlagSw2 = false;
  pObj->iqMax_adc = iqMax_adc;
  pObj->iqMin_adc = iqMin_adc;

  pObj->iqMax_out = iqMax_out;
  pObj->iqMin_out = iqMin_out;

  pObj->iqSlope = _IQdiv((pObj->iqMax_out - pObj->iqMin_out),(pObj->iqMax_adc - pObj->iqMin_adc));
  pObj->iqOffset = pObj->iqMax_out - _IQmpy(pObj->iqSlope,pObj->iqMax_adc);

  pObj->state = Throttle_Run;
  pObj->iqValue = _IQ(0.0);

}


extern void Throttle_setup(Throttle_Handle handle,      \
                                const _iq iqValue,        \
                                const bool bSW1,         \
                                const bool bSW2)
{
	Throttle_Obj *pObj = (Throttle_Obj *)handle;

	pObj->bFlagSw1 = bSW1;
	pObj->bFlagSw2 = bSW2;

 	pObj->iqValue = iqValue;


}

                                
extern void Throttle_runState(Throttle_Handle handle)
{
	Throttle_Obj *pObj = (Throttle_Obj *)handle;


	if(pObj->bFlagSw1)
	{
		pObj->iqResult = _IQ(0.0);
		pObj->state = Throttle_CalMaxMin;
		pObj->iqMax_adc = pObj->iqMin_out;
		pObj->iqMin_adc = pObj->iqMax_out;
	}
	else
	{
		switch (pObj->state)
		{
			case Throttle_CalMaxMin:

				if (pObj->iqValue > pObj->iqMax_adc)
					pObj->iqMax_adc = pObj->iqValue;
				else if (pObj->iqValue < pObj->iqMin_adc)
					pObj->iqMin_adc = pObj->iqValue;
				else if (pObj->bFlagSw2)
					pObj->state = Throttle_CalCalc;

				break;

			case Throttle_CalCalc:
				pObj->iqSlope = _IQdiv((pObj->iqMax_out - pObj->iqMin_out),(pObj->iqMax_adc - pObj->iqMin_adc));
				pObj->iqOffset = pObj->iqMax_out - _IQmpy(pObj->iqSlope,pObj->iqMax_adc);
      
				pObj->state = Throttle_Run;
				break;

			case Throttle_Run:
			{
				_iq iqResult = _IQmpy(pObj->iqValue,pObj->iqSlope) + pObj->iqOffset;
				pObj->iqResult = _IQsat(iqResult,pObj->iqMax_out,pObj->iqMin_out);
			}
				break;
		}
	}


}


