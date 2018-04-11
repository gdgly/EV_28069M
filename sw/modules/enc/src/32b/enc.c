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
//! \file   modules/enc/src/32b/enc.c
//! \brief  Portable C fixed point code.  These functions define the
//! \brief  encoder routines.
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "sw/modules/enc/src/32b/enc.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals

// **************************************************************************
// the functions

ENC_Handle ENC_init(void *pMemory,const size_t numBytes)
{
	ENC_Handle encHandle;
	if(numBytes < sizeof(ENC_Obj))
		return((ENC_Handle)NULL);

	encHandle = (ENC_Handle)pMemory;
	return(encHandle);
} // end of ENC_init() function

void ENC_setup(ENC_Handle encHandle, const int16_t i16sample_period, const int16_t i16num_pole_pairs,
		       const uint16_t u16num_enc_slots, const uint32_t u32enc_zero_offset,
		       const float_t ffull_scale_freq, const float_t fspeed_update_freq, const float_t fspeed_cutoff)
{
	ENC_Obj *pEnc;
	float_t ftemp;
	float_t fspeed_cutoff_radians;
	//int16_t i16;
 
	pEnc = (ENC_Obj *)encHandle;

	pEnc->i16sample_period = i16sample_period;
	pEnc->u16num_enc_slots = u16num_enc_slots;
	pEnc->i16num_pole_pairs = i16num_pole_pairs;

	pEnc->iqmech_angle_gain = (_iq)((((uint32_t)1)<<24)/(4*u16num_enc_slots));	//compute the gain which translates the mech into the elec angle

	ENC_setZeroOffset(encHandle, u32enc_zero_offset);
	//pEnc->iqenc_zero_offset = u32enc_zero_offset * pEnc->iqmech_angle_gain;


	pEnc->iqenc_elec_angle_pu = 0;		// initialize the electrical angle
	pEnc->iqenc_slip_angle_pu = pEnc->iqincremental_slip_pu = pEnc->iqenc_magnetic_angle_pu = 0;

	pEnc->i16sample_count = 0;
	pEnc->u32prev_enc = 0;
	pEnc->i32delta_enc = 0;


	//pEnc->i32delta_enc =  0;		// Calculate Speed

	ftemp = ((float_t)i16num_pole_pairs*fspeed_update_freq*ENC_SPEED_SCALING_FACTOR) /
			  (4.0*(float_t)u16num_enc_slots*ffull_scale_freq*(float_t)i16sample_period);
	pEnc->iqspeed_gain = (_iq) ftemp;	// compute the speed gain
	pEnc->iqspeed_lpf_out = 0;

	fspeed_cutoff_radians = ENC_2PI*fspeed_cutoff;
	ftemp = fspeed_cutoff_radians/(fspeed_update_freq+fspeed_cutoff_radians);
	pEnc->iq6speed_lpf_cx = _IQ6(ftemp);	//(int32_t) (ENC_SPEED_COEFF_SCALING*ftemp);
	pEnc->iq6speed_lpf_cy = _IQ6(1.0f) - pEnc->iq6speed_lpf_cx;		//  ENC_SPEED_COEFF_SCALING - pEnc->i32speed_lpf_cx;



	// compute the rpm gain
	ftemp = (float_t)((ffull_scale_freq*60.0)/i16num_pole_pairs/1000.0);
	pEnc->iqKrpm_gain =  _IQ(ftemp);
	//pEnc->i32rpm_gain =  (int32_t) ftemp;


} // end of ENC setup function


void ENC_calcElecAngle(ENC_Handle encHandle, uint32_t u32posnCnts)
{
	ENC_Obj *pEnc;
	_iq	iqtemp;
 
	pEnc = (ENC_Obj *) encHandle;
	
	iqtemp = u32posnCnts* pEnc->iqmech_angle_gain;		// compute the mechanical angle
	iqtemp += pEnc->iqenc_zero_offset_pu;				// add in calibrated offset
	iqtemp *= pEnc->i16num_pole_pairs;					// convert to electrical angle
	iqtemp &= ((uint32_t) 0x00ffffff);					// wrap around 1.0 (Q24)
	pEnc->iqenc_elec_angle_pu = iqtemp;

	pEnc->iqenc_slip_angle_pu +=   pEnc->iqincremental_slip_pu;	// update the slip angle
	pEnc->iqenc_slip_angle_pu &= ((uint32_t) 0x00ffffff);		// wrap around 1.0 (Q24)

 	iqtemp = iqtemp + pEnc->iqenc_slip_angle_pu;		// add in compensation for slip
  	iqtemp &= ((uint32_t) 0x00ffffff);					// wrap around 1.0 (Q24)
  	pEnc->iqenc_magnetic_angle_pu = iqtemp;				// store encoder magnetic angle


} // end of ENC_calc_elec_angle() function


void ENC_run(ENC_Handle encHandle, uint32_t u32posnCounts)
{

	ENC_Obj *pEnc = (ENC_Obj *) encHandle;				// create an object pointer for manipulation
	int32_t i32delta_enc, i32slots4X;

	ENC_calcElecAngle(encHandle, u32posnCounts);

	i32slots4X = 4*pEnc->u16num_enc_slots;
	i32delta_enc = (int32_t) u32posnCounts - (int32_t) pEnc->u32prev_enc;
	if (i32delta_enc > i32slots4X/2)
		i32delta_enc -= i32slots4X;
	else if (i32delta_enc < -i32slots4X/2)
		i32delta_enc += i32slots4X;

	pEnc->u32prev_enc = u32posnCounts;
	pEnc->i32delta_enc += i32delta_enc;

	pEnc->i16sample_count++;
 	if (pEnc->i16sample_count == pEnc->i16sample_period)	// if it reaches the sample period, read and process encoder data
	{
 		_iq iqtemp;
 		iqtemp = pEnc->i32delta_enc*pEnc->iqspeed_gain;			//Q24
		iqtemp = (pEnc->iq6speed_lpf_cy)*pEnc->iqspeed_lpf_out + (pEnc->iq6speed_lpf_cx)*iqtemp;	//Q30
		pEnc->iqspeed_lpf_out = iqtemp >> 6;	// LPF the encoder Q24

		pEnc->i32delta_enc = 0;
		pEnc->i16sample_count = 0;

    	/*i32temp = pEnc->i32delta_enc*pEnc->i32speed_gain;			//Q24
    	i32temp = (pEnc->i32speed_lpf_cy)*pEnc->i32speed_lpf_out + (pEnc->i32speed_lpf_cx)*i32temp;	//Q30
    	i32temp >>= ENC_SPEED_COEFF_Q;		//Q24
    	pEnc->i32speed_lpf_out = i32temp;	// LPF the encoder*/

    	/*
    	  // shift it to Q24
    	    temp = enc->delta_enc*enc->speed_gain;			//Q24

    	    // LPF the encoder
    	    temp = (enc->speed_lpf_cy)*enc->speed_lpf_out + (enc->speed_lpf_cx)*temp;	//Q30
    	    temp >>= ENC_SPEED_COEFF_Q;		//Q24
    	    enc->speed_lpf_out = temp;

    	    // copy the current into the previous value
    	    enc->prev_enc = enc_val;
    	    */
	}

} // end of ENC_run() function



_iq ENC_getSpeedKRPM(ENC_Handle encHandle)
{
	ENC_Obj *pEnc;
	//_iq iqtemp;

	// create an object pointer for manipulation
	pEnc = (ENC_Obj *) encHandle;

	//iqtemp = (pEnc->i32speed_lpf_out >> ENC_RPM_Q1);
	//iqtemp = (iqtemp * pEnc->i32rpm_gain) >> ENC_RPM_Q2;
	//iqtemp = _IQmpy(pEnc->iqspeed_lpf_out, pEnc->iq16rpm_gain);

	return  _IQmpy(pEnc->iqspeed_lpf_out, pEnc->iqKrpm_gain);
} // end of ENC_getSpeedRPM() function


/* Org
void ENC_run(ENC_Handle encHandle, uint32_t posnCounts, uint16_t indextFlag, uint16_t dirFlag, int16_t log_flag)
{
	uint16_t dir;
	uint16_t index_event_before;
	uint16_t index_event_after;
	ENC_Obj *enc;
	int32_t enc_val;
	int32_t delta_enc;
	int32_t temp;
	int16_t sample_count;

  // create an object pointer for manipulation
  enc = (ENC_Obj *) encHandle;

  // update the encoder counter
  sample_count = enc->sample_count;
  sample_count++;

  // if it reaches the sample period, read and process encoder data
  if (sample_count == enc->sample_period)
  {
    enc->sample_count = 0;

    // check for index event before the encoder reading
    index_event_before = indextFlag;

    // read the encoder
    enc_val = posnCounts;

    // compute the mechanical angle
    // compute the mechanical angle
    temp = (enc_val)*enc->mech_angle_gain;		//Q24
    // add in calibrated offset
    temp += enc->enc_zero_offset;
    // convert to electrical angle
    temp = temp * enc->num_pole_pairs;
    // wrap around 1.0
    temp &= ((uint32_t) 0x00ffffff);

    enc->enc_elec_angle = (_iq)temp;



    // read QEP direction from quadrature direction latch flag
    dir = dirFlag;
  
    // check for index event after the encoder reading
    index_event_after = indextFlag;
  
    // handle a rollover event
    if (index_event_after)
    {
  	   if (dir)
  	   {
  	     delta_enc = enc_val + (4*enc->num_enc_slots-1) - enc->prev_enc;
  	   }
  	   else
  	   {
         delta_enc = enc->prev_enc + (4*enc->num_enc_slots-1) - enc_val;
  	   }
    }
    else
    {
       delta_enc = enc_val - enc->prev_enc;
    }
    
    // save off the delta encoder value in the data structure only if the index event before and after are the same
    if ((index_event_after == index_event_before) && (abs(delta_enc) < enc->num_enc_slots))
    	enc->delta_enc = delta_enc;
    
    // log the startup data
    switch(enc->log_state)
    {
    	// wait for run flag to be set
    	case ENC_LOG_STATE_IDLE:
    	{
	    	if (enc->run_flag)
	    	{
	    		enc->run_flag  = 0;
	    		enc->log_idx   = 0;
	    		enc->log_state = ENC_LOG_STATE_FREERUN;
	    	}
    	}
    	break;
    	
    	// collect data round robin until there's a trigger event
    	case ENC_LOG_STATE_FREERUN:
    	{
			enc->log[enc->log_idx] = (int16_t)enc_val;
			enc->log_idx = enc->log_idx + 1;
			if (enc->log_idx > ENC_LOG_LEN)
				enc->log_idx = 0;
			if (abs(delta_enc)>ENC_LOG_DELTA_TRIGGER_THRES)
			{
				enc->trigger_delta = delta_enc;
				enc->trigger_idx = enc->log_idx;
				enc->post_trigger_cnt = enc->post_trigger_len;
				enc->log_state = ENC_LOG_STATE_ACQUIRE;
			}
    	}    		
    	break;
    	
    	// when trigger occurs collect 1/2 a buffer of post-trigger information
    	case ENC_LOG_STATE_ACQUIRE:
    	{
			enc->log[enc->log_idx] = (int16_t)enc_val;
			enc->log_idx = enc->log_idx + 1;
			if (enc->log_idx > ENC_LOG_LEN)
				enc->log_idx = 0;
			enc->post_trigger_cnt = enc->post_trigger_cnt - 1;
			if (enc->post_trigger_cnt == 0)
				enc->log_state = ENC_LOG_STATE_IDLE;
    	}      	
    	break;
    }

    // shift it to Q24
    temp = enc->delta_enc*enc->speed_gain;			//Q24
  
    // LPF the encoder
    temp = (enc->speed_lpf_cy)*enc->speed_lpf_out + (enc->speed_lpf_cx)*temp;	//Q30
    temp >>= ENC_SPEED_COEFF_Q;		//Q24
    enc->speed_lpf_out = temp;

    // copy the current into the previous value  
    enc->prev_enc = enc_val;
  }
  else
  {
  	enc->sample_count = sample_count;
  }
  
} // end of ENC_run() function
*/


// end of file
