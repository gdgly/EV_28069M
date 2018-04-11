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
#ifndef _ENC_H_
#define _ENC_H_

//! \file   modules/enc/src/32b/enc.h
//! \brief  Contains the public interface to the 
//!         encoder module routines 
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes
#include "sw/modules/types/src/types.h"
#include "sw/modules/iqmath/src/32b/IQmathLib.h"

//!
//!
//! \defgroup ENC ENC
//!
//@{

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines
#define ENC_SPEED_SCALING_FACTOR			16777216.0		/* 2^24   */
#define ENC_ZERO_OFFSET						3813071.0
#define ENC_SPEED_CUTOFF_FREQ				100.0
#define ENC_SPEED_COEFF_Q					6
#define ENC_SPEED_COEFF_SCALING				(1<<ENC_SPEED_COEFF_Q)
#define ENC_2PI								(2.0*3.14159)
#define ENC_LOG_LEN							256
#define ENC_RPM_Q1							8
#define ENC_RPM_Q2							16
#define ENC_LOG_DELTA_TRIGGER_THRES			400


// **************************************************************************
// the typedefs

//! \brief Enumeration for the encoder log states
//!
typedef enum {
  ENC_LOG_STATE_IDLE=0,           //!< idle state
  ENC_LOG_STATE_FREERUN,		  //!< freerun state
  ENC_LOG_STATE_ACQUIRE			  //!< acquire state
} ENC_LOG_State_e;


//! \brief Defines the encoder object
//!
typedef struct _ENC_Obj_
{
	int16_t i16sample_count;			//!< when it reaches the sample period, collect & process encoder data
	int16_t i16sample_period;		//!< sample period of encoder processing
	int16_t i16num_pole_pairs;		//!< number of pole pairs in motor
	uint16_t u16num_enc_slots;		//!< number of encoder slots

	_iq iqmech_angle_gain;			//!< gain which converts the encoder counts to Q24 mechanical degrees
  	_iq iqenc_zero_offset_pu;     //!< encoder zero offset in counts
  	_iq iqenc_elec_angle_pu;		//!< encoder current electrical angle
    _iq iqincremental_slip_pu;     //!< incremental amount of slip
    _iq iqenc_slip_angle_pu;       //!< amount of total slip
    _iq iqenc_magnetic_angle_pu;   //!< encoder current magnetic angle (compensated for slip)
  //uint32_t u32enc_zero_offset;     //!< encoder zero offset in counts

  //int32_t i32enc_elec_angle;		//!< encoder current electrical angle
  //int32_t i32incremental_slip;     //!< incremental amount of slip
  //int32_t i32enc_slip_angle;       //!< amount of total slip
  //int32_t i32enc_magnetic_angle;   //!< encoder current magnetic angle (compensated for slip)



    uint32_t u32prev_enc;				//!< previous encoder reading
    int32_t i32delta_enc;				//!< encoder count delta

  //float_t ffull_scale_freq;	    //!< full scale frequency
  	  _iq 	iqspeed_gain;			//!< gain which converts a difference in encoder counts to Q24 normalized electrical freq
  	  _iq 	iqspeed_lpf_out;		//!< speed lpf output

  //float_t fspeed_cutoff;		    //!< speed cutoff frequency in Hz
  	  _iq6 	iq6speed_lpf_cx;			//!< speed input coefficient
  	  _iq6 	iq6speed_lpf_cy;			//!< speed output coefficient

  	  _iq 	iqKrpm_gain;				//!< gain which converts the Q24 normalized electrical freq to RPM

  //ENC_LOG_State_e log_state;	//!< encoder log state
  //int16_t i16run_flag;				//!< encoder log free run flag
  //int16_t i16post_trigger_len;		//!< encoder log post trigger length
  //int16_t i16post_trigger_cnt;		//!< encoder log post trigger counter
  //int16_t i16trigger_idx;			//!< index where trigger event happened
  //int16_t i16trigger_delta;		//!< calculated delta when trigger happened
  //int32_t i32log_idx;			    //!< encoder log index
  //int16_t ai16log[ENC_LOG_LEN];     //!< encoder log length
} ENC_Obj;


//! \brief Defines the encoder handle
//!
typedef struct _ENC_Obj_ *ENC_Handle;


// **************************************************************************
// the function prototypes

//! \brief Reads encoder and returns the electrical degrees in Q24 format
//! \param[in] encHandle				Handle to the ENC object
//! \param[in] posnCounts               Current position counts from encoder
//! \return								Nothing
void ENC_calcElecAngle(ENC_Handle encHandle, uint32_t u32posnCounts);


//! \brief Returns the electrical angle
//! \param[in] encHandle				Handle to the ENC object
//! \return								Electrical angle in Q24
inline _iq ENC_getElecAnglePu(ENC_Handle encHandle)
{
	ENC_Obj *pEnc = (ENC_Obj *) encHandle;

	return pEnc->iqenc_elec_angle_pu;
}


//! \brief Returns the magnetic angle
//! \param[in] encHandle				Handle to the ENC object
//! \return								Magnetic angle in Q24
inline _iq ENC_getMagneticAnglePu(ENC_Handle encHandle)
{
	ENC_Obj *pEnc = (ENC_Obj *) encHandle;

	return pEnc->iqenc_magnetic_angle_pu;
}


//! \brief Returns the low-pass filtered speed output
//! \param[in] encHandle				Handle to the ENC object
//! \return								LPF speed output in Q24
inline _iq ENC_getFilteredSpeedPu(ENC_Handle encHandle)
{
	ENC_Obj *penc = (ENC_Obj *) encHandle;

	return penc->iqspeed_lpf_out;
}


//! \brief Returns the filtered speed in RPM
//! \param[in] encHandle				Handle to the ENC object
//! \return								RPM in Q0
extern _iq ENC_getSpeedKRPM(ENC_Handle encHandle);


//! \brief Initializes the encoder object
//! \param[in] pMemory		Memory pointer to object
//! \param[in] numBytes		Object size
//! \return					Object handle
extern ENC_Handle ENC_init(void *pMemory,const size_t numBytes);


//! \brief Based on the encoder reading, computes the electrical angle and the electrical "speed"
//! \param[in] encHandle				Handle to the ENC object
//! \param[in] posnCounts               Current position counts from encoder
//! \param[in] indextFlag               If set, there was an index
//! \param[in] dirFlag                  Indicates direction of rotation
//! \param[in] log_flag					If set, logs the encoder data
//! \return								Nothing
void ENC_run(ENC_Handle encHandle, uint32_t u32posnCounts);	// uint16_t u16indextFlag, uint16_t u16dirFlag, int16_t i16log_flag);


//! \brief Set the amount of incremental slip
//! \param[in] encHandle				Handle to the ENC object
//! \param[in] incrementalSlip          Amount of incremental slip in Electrical Angle
inline void ENC_setIncrementalSlipPu(ENC_Handle encHandle, _iq iqincrementalSlip)
{
	ENC_Obj *pEnc = (ENC_Obj *) encHandle;

	// set the amount of incremental slip
	pEnc->iqincremental_slip_pu = iqincrementalSlip;


}


//! \brief Start logging encoder data
//! \param[in] encHandle				Handle to the ENC object
/*inline void ENC_setRunFlag(ENC_Handle encHandle)
{
	ENC_Obj *enc = (ENC_Obj *) encHandle;

	// set the run flag
	enc->i16run_flag = 1;

}*/


//! \brief Sets the value for the encoder object zero offset
//! \param[in] encHandle                        Handle to the ENC object
//! \param[in] zeroOffset                       New zero offset
inline void ENC_setZeroOffset(ENC_Handle encHandle, uint32_t u32zeroOffsetCnt)
{
	ENC_Obj *pEnc = (ENC_Obj *)encHandle;

	pEnc->iqenc_zero_offset_pu = u32zeroOffsetCnt*pEnc->iqmech_angle_gain;
	pEnc->iqenc_zero_offset_pu &= ((uint32_t) 0x00ffffff);					// wrap around 1.0 (Q24)


}


//! \brief Initializes encoder object parameters
//! \param[in] encHandle                        Handle to the ENC object
//! \param[in] sample_period                    How often the encoder is read & processed
//! \param[in] num_pole_pairs                   Number of pole pairs in motor
//! \param[in] num_enc_slots                    Number of encoder slots
//! \param[in] enc_zero_offset                  Encoder zero offset in counts
//! \param[in] full_scale_freq                  Full scale speed for normalization
//! \param[in] speed_update_freq                Update frequency in Hz for speed calculation
//! \param[in] speed_cutoff                     Speed calculation LPF cutoff frequency in Hz
extern void ENC_setup(ENC_Handle encHandle, const int16_t sample_period,
                      const int16_t num_pole_pairs, const uint16_t num_enc_slots,
                      const uint32_t enc_zero_offset, const float_t full_scale_freq, 
                      const float_t speed_update_freq, const float_t speed_cutoff);

#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _ENC_H_ definition

