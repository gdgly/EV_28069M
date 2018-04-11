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
//! \file   solutions/instaspin_foc/src/user.c
//! \brief Contains the function for setting initialization data to the CTRL, HAL, and EST modules
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include <math.h>
#include "sw/modules/user/src/32b/user.h"

#ifndef QEP
#include "sw/modules/ctrl/src/32b/ctrl.h"
#else
#include "sw/modules/ctrl/src/32b/ctrlQEP.h"
#endif
//#include "sw/modules/ctrl/src/32b/ctrl.h"


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs


// **************************************************************************
// the functions


void USER_setParams(USER_Params *pUserParams)//$this is call_by_reference
{
	pUserParams->fFullScaleCurrent_A = USER_IQ_FULL_SCALE_CURRENT_A;
	pUserParams->fFullScaleVoltage_V = USER_IQ_FULL_SCALE_VOLTAGE_V;

	pUserParams->fFullScaleFreq_Hz = USER_IQ_FULL_SCALE_FREQ_Hz;

	pUserParams->u16numIsrTicksPerCtrlTick = USER_NUM_ISR_TICKS_PER_CTRL_TICK;
	pUserParams->u16numCtrlTicksPerCurrentTick = USER_NUM_CTRL_TICKS_PER_CURRENT_TICK;
	pUserParams->u16numCtrlTicksPerEstTick = USER_NUM_CTRL_TICKS_PER_EST_TICK;
	pUserParams->u16numCtrlTicksPerSpeedTick = USER_NUM_CTRL_TICKS_PER_SPEED_TICK;
	pUserParams->u16numCtrlTicksPerTrajTick = USER_NUM_CTRL_TICKS_PER_TRAJ_TICK;

	pUserParams->u8numCurrentSensors = USER_NUM_CURRENT_SENSORS;
	pUserParams->u8numVoltageSensors = USER_NUM_VOLTAGE_SENSORS;

	pUserParams->foffsetPole_rps = USER_OFFSET_POLE_rps;
	pUserParams->ffluxPole_rps = USER_FLUX_POLE_rps;

	pUserParams->fzeroSpeedLimit = USER_ZEROSPEEDLIMIT;

	pUserParams->fforceAngleFreq_Hz = USER_FORCE_ANGLE_FREQ_Hz;

	pUserParams->fmaxAccel_Hzps = USER_MAX_ACCEL_Hzps;

	pUserParams->fmaxAccel_est_Hzps = USER_MAX_ACCEL_EST_Hzps;

	pUserParams->fdirectionPole_rps = USER_DIRECTION_POLE_rps;

	pUserParams->fspeedPole_rps = USER_SPEED_POLE_rps;

	pUserParams->fdcBusPole_rps = USER_DCBUS_POLE_rps;

	pUserParams->ffluxFraction = USER_FLUX_FRACTION;

	pUserParams->findEst_speedMaxFraction = USER_SPEEDMAX_FRACTION_FOR_L_IDENT;

	pUserParams->fpowerWarpGain = USER_POWERWARP_GAIN;

	pUserParams->u16systemFreq_MHz = USER_SYSTEM_FREQ_MHz;

	pUserParams->fpwmPeriod_usec = USER_PWM_PERIOD_usec;

	pUserParams->fvoltage_sf = USER_VOLTAGE_SF;

	pUserParams->fcurrent_sf = USER_CURRENT_SF;

	pUserParams->fvoltageFilterPole_rps = USER_VOLTAGE_FILTER_POLE_rps;

	pUserParams->fmaxVsMag_pu = USER_MAX_VS_MAG_PU;

	pUserParams->festKappa = USER_EST_KAPPAQ;

	pUserParams->motor_type = USER_MOTOR_TYPE;
	pUserParams->u16motor_numPolePairs = USER_MOTOR_NUM_POLE_PAIRS;
	pUserParams->fmotor_ratedFlux = USER_MOTOR_RATED_FLUX;
	pUserParams->fmotor_Rr = USER_MOTOR_Rr;
	pUserParams->fmotor_Rs = USER_MOTOR_Rs;
	pUserParams->fmotor_Ls_d = USER_MOTOR_Ls_d;
	pUserParams->fmotor_Ls_q = USER_MOTOR_Ls_q;

	pUserParams->fmaxCurrent_resEst = USER_MOTOR_RES_EST_CURRENT;
	pUserParams->fmaxCurrent_indEst = USER_MOTOR_IND_EST_CURRENT;
	pUserParams->fmaxCurrent = USER_MOTOR_MAX_CURRENT;

	pUserParams->fmaxCurrentSlope = USER_MAX_CURRENT_SLOPE;
	pUserParams->fmaxCurrentSlope_powerWarp = USER_MAX_CURRENT_SLOPE_POWERWARP;

	pUserParams->fIdRated = USER_MOTOR_MAGNETIZING_CURRENT;
	pUserParams->fIdRatedFraction_ratedFlux = USER_IDRATED_FRACTION_FOR_RATED_FLUX;
	pUserParams->fIdRatedFraction_indEst = USER_IDRATED_FRACTION_FOR_L_IDENT;
	pUserParams->fIdRated_delta = USER_IDRATED_DELTA;

	pUserParams->ffluxEstFreq_Hz = USER_MOTOR_FLUX_EST_FREQ_Hz;

	pUserParams->au32ctrlWaitTime[CTRL_State_Error]         = 0;
	pUserParams->au32ctrlWaitTime[CTRL_State_Idle]          = 0;
	//pUserParams->ctrlWaitTime[CTRL_State_OffLine]       = (uint_least32_t)( 1000 * USER_CTRL_FREQ_Hz);

	pUserParams->au32ctrlWaitTime[CTRL_State_OffLine]       = (uint_least32_t)( 5.0 * USER_CTRL_FREQ_Hz); //Org
	pUserParams->au32ctrlWaitTime[CTRL_State_OnLine]        = 0;

	pUserParams->au32estWaitTime[EST_State_Error]           = 0;
	pUserParams->au32estWaitTime[EST_State_Idle]            = 0;
	//pUserParams->estWaitTime[EST_State_RoverL]          = (uint_least32_t)( 1000.0 * USER_EST_FREQ_Hz);
	pUserParams->au32estWaitTime[EST_State_RoverL]          = (uint_least32_t)( 8.0 * USER_EST_FREQ_Hz); //Org
	pUserParams->au32estWaitTime[EST_State_Rs]              = 0;
	pUserParams->au32estWaitTime[EST_State_RampUp]          = (uint_least32_t)((5.0 + USER_MOTOR_FLUX_EST_FREQ_Hz / USER_MAX_ACCEL_EST_Hzps) * USER_EST_FREQ_Hz);
	pUserParams->au32estWaitTime[EST_State_IdRated]         = (uint_least32_t)(30.0 * USER_EST_FREQ_Hz);
	pUserParams->au32estWaitTime[EST_State_RatedFlux_OL]    = (uint_least32_t)( 0.2 * USER_EST_FREQ_Hz);
	pUserParams->au32estWaitTime[EST_State_RatedFlux]       = 0;
	pUserParams->au32estWaitTime[EST_State_RampDown]        = (uint_least32_t)( 2.0 * USER_EST_FREQ_Hz);
	pUserParams->au32estWaitTime[EST_State_LockRotor]       = 0;
	pUserParams->au32estWaitTime[EST_State_Ls]              = 0;
	pUserParams->au32estWaitTime[EST_State_Rr]              = (uint_least32_t)(20.0 * USER_EST_FREQ_Hz);
	pUserParams->au32estWaitTime[EST_State_MotorIdentified] = 0;
	pUserParams->au32estWaitTime[EST_State_OnLine]          = 0;

	pUserParams->au32FluxWaitTime[EST_Flux_State_Error]     = 0;
	pUserParams->au32FluxWaitTime[EST_Flux_State_Idle]      = 0;
	pUserParams->au32FluxWaitTime[EST_Flux_State_CL1]       = (uint_least32_t)(10.0 * USER_EST_FREQ_Hz);
	pUserParams->au32FluxWaitTime[EST_Flux_State_CL2]       = (uint_least32_t)( 0.2 * USER_EST_FREQ_Hz);
	pUserParams->au32FluxWaitTime[EST_Flux_State_Fine]      = (uint_least32_t)( 8.0 * USER_EST_FREQ_Hz);//original:(uint_least32_t)( 4.0 * USER_EST_FREQ_Hz);
	pUserParams->au32FluxWaitTime[EST_Flux_State_Done]      = 0;

	pUserParams->au32LsWaitTime[EST_Ls_State_Error]        = 0;
	pUserParams->au32LsWaitTime[EST_Ls_State_Idle]         = 0;
	//pUserParams->au32LsWaitTime[EST_Ls_State_RampUp]       = (uint_least32_t)( 3.0 * USER_EST_FREQ_Hz);	Org
	pUserParams->au32LsWaitTime[EST_Ls_State_RampUp]       = (uint_least32_t)( 3.0 * USER_EST_FREQ_Hz);
	pUserParams->au32LsWaitTime[EST_Ls_State_Init]         = (uint_least32_t)( 3.0 * USER_EST_FREQ_Hz);
	pUserParams->au32LsWaitTime[EST_Ls_State_Coarse]       = (uint_least32_t)( 0.2 * USER_EST_FREQ_Hz);
	pUserParams->au32LsWaitTime[EST_Ls_State_Fine]         = (uint_least32_t)(30.0 * USER_EST_FREQ_Hz);
	pUserParams->au32LsWaitTime[EST_Ls_State_Done]         = 0;

	pUserParams->au32RsWaitTime[EST_Rs_State_Error]        = 0;
	pUserParams->au32RsWaitTime[EST_Rs_State_Idle]         = 0;
	pUserParams->au32RsWaitTime[EST_Rs_State_RampUp]       = (uint_least32_t)( 1.0 * USER_EST_FREQ_Hz);
	pUserParams->au32RsWaitTime[EST_Rs_State_Coarse]       = (uint_least32_t)( 2.0 * USER_EST_FREQ_Hz);
	pUserParams->au32RsWaitTime[EST_Rs_State_Fine]         = (uint_least32_t)( 4.0 * USER_EST_FREQ_Hz);

	//pUserParams->RsWaitTime[EST_Rs_State_RampUp]       = (uint_least32_t)( 1.0 * USER_EST_FREQ_Hz);	//Org
	//pUserParams->RsWaitTime[EST_Rs_State_Coarse]       = (uint_least32_t)( 2.0 * USER_EST_FREQ_Hz);	//Org
	//pUserParams->RsWaitTime[EST_Rs_State_Fine]         = (uint_least32_t)( 7.0 * USER_EST_FREQ_Hz);	//Org
	pUserParams->au32RsWaitTime[EST_Rs_State_Done]         = 0;

	pUserParams->u32ctrlFreq_Hz = USER_CTRL_FREQ_Hz;

	pUserParams->u32estFreq_Hz = USER_EST_FREQ_Hz;

	pUserParams->u32RoverL_estFreq_Hz = USER_R_OVER_L_EST_FREQ_Hz;

	pUserParams->u32trajFreq_Hz = USER_TRAJ_FREQ_Hz;

	pUserParams->fctrlPeriod_sec = USER_CTRL_PERIOD_sec;

	pUserParams->fmaxNegativeIdCurrent_a = USER_MAX_NEGATIVE_ID_REF_CURRENT_A;


	pUserParams->fExtTemp_sf = USER_EXT_TEMP_SF;
	pUserParams->fExtAdc_sf = USER_EXT_ADC_SF;


} // end of USER_setParams() function


void USER_checkForErrors(USER_Params *pUserParams)
{
	USER_setErrorCode(pUserParams, USER_ErrorCode_NoError);

	if((USER_IQ_FULL_SCALE_CURRENT_A <= 0.0) ||
    (USER_IQ_FULL_SCALE_CURRENT_A <= (0.02 * USER_MOTOR_MAX_CURRENT * USER_IQ_FULL_SCALE_FREQ_Hz / 128.0)) ||
    (USER_IQ_FULL_SCALE_CURRENT_A <= (2.0 * USER_MOTOR_MAX_CURRENT * USER_IQ_FULL_SCALE_FREQ_Hz * USER_CTRL_PERIOD_sec / 128.0)))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleCurrent_A_Low);
    }

	if((USER_IQ_FULL_SCALE_CURRENT_A < USER_MOTOR_MAGNETIZING_CURRENT) ||
    (USER_IQ_FULL_SCALE_CURRENT_A < USER_MOTOR_RES_EST_CURRENT) ||
    (USER_IQ_FULL_SCALE_CURRENT_A < USER_MOTOR_IND_EST_CURRENT) ||
    (USER_IQ_FULL_SCALE_CURRENT_A < USER_MOTOR_MAX_CURRENT))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleCurrent_A_Low);
    }

	if((USER_MOTOR_RATED_FLUX > 0.0) && (USER_MOTOR_TYPE == MOTOR_Type_Pm))
    {
		if(USER_IQ_FULL_SCALE_VOLTAGE_V >= ((float_t)USER_EST_FREQ_Hz * USER_MOTOR_RATED_FLUX * 0.7))
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleVoltage_V_High);
        }
    }

	if((USER_MOTOR_RATED_FLUX > 0.0) && (USER_MOTOR_TYPE == MOTOR_Type_Induction))
    {
		if(USER_IQ_FULL_SCALE_VOLTAGE_V >= ((float_t)USER_EST_FREQ_Hz * USER_MOTOR_RATED_FLUX * 0.05))
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleVoltage_V_High);
        }
    }

	if((USER_IQ_FULL_SCALE_VOLTAGE_V <= 0.0) ||
    (USER_IQ_FULL_SCALE_VOLTAGE_V <= (0.5 * USER_MOTOR_MAX_CURRENT * USER_MOTOR_Ls_d * USER_VOLTAGE_FILTER_POLE_rps)) ||
    (USER_IQ_FULL_SCALE_VOLTAGE_V <= (0.5 * USER_MOTOR_MAX_CURRENT * USER_MOTOR_Ls_q * USER_VOLTAGE_FILTER_POLE_rps)))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleVoltage_V_Low);
    }

	if((USER_IQ_FULL_SCALE_FREQ_Hz > (4.0 * USER_VOLTAGE_FILTER_POLE_Hz)) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz >= ((128.0 * USER_IQ_FULL_SCALE_CURRENT_A) / (0.02 * USER_MOTOR_MAX_CURRENT))) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz >= ((128.0 * USER_IQ_FULL_SCALE_CURRENT_A) / (2.0 * USER_MOTOR_MAX_CURRENT * USER_CTRL_PERIOD_sec))) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz >= (128.0 * (float_t)USER_MOTOR_NUM_POLE_PAIRS * 1000.0 / 60.0)))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleFreq_Hz_High);
    }

	if((USER_IQ_FULL_SCALE_FREQ_Hz < 50.0) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz < USER_MOTOR_FLUX_EST_FREQ_Hz) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz < USER_SPEED_POLE_rps) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz <= ((float_t)USER_MOTOR_NUM_POLE_PAIRS * 1000.0 / (60.0 * 128.0))) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz < (USER_MAX_ACCEL_Hzps / ((float_t)USER_TRAJ_FREQ_Hz))) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz < (USER_MAX_ACCEL_EST_Hzps / ((float_t)USER_TRAJ_FREQ_Hz))) ||
    (USER_IQ_FULL_SCALE_FREQ_Hz < ((float_t)USER_R_OVER_L_EST_FREQ_Hz)))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleFreq_Hz_Low);
    }

	if(USER_NUM_PWM_TICKS_PER_ISR_TICK > 3)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_numPwmTicksPerIsrTick_High);
    }

	if(USER_NUM_PWM_TICKS_PER_ISR_TICK < 1)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_numPwmTicksPerIsrTick_Low);
    }

	if(USER_NUM_ISR_TICKS_PER_CTRL_TICK < 1)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_numIsrTicksPerCtrlTick_Low);
    }

	if(USER_NUM_CTRL_TICKS_PER_CURRENT_TICK < 1)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_numCtrlTicksPerCurrentTick_Low);
    }

	if(USER_NUM_CTRL_TICKS_PER_EST_TICK < 1)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_numCtrlTicksPerEstTick_Low);
    }

	if((USER_NUM_CTRL_TICKS_PER_SPEED_TICK < 1) ||
    (USER_NUM_CTRL_TICKS_PER_SPEED_TICK < USER_NUM_CTRL_TICKS_PER_CURRENT_TICK))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_numCtrlTicksPerSpeedTick_Low);
    }

	if(USER_NUM_CTRL_TICKS_PER_TRAJ_TICK < 1)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_numCtrlTicksPerTrajTick_Low);
    }

	if(USER_NUM_CURRENT_SENSORS > 3)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_numCurrentSensors_High);
    }

	if(USER_NUM_CURRENT_SENSORS < 2)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_numCurrentSensors_Low);
    }

	if(USER_NUM_VOLTAGE_SENSORS > 3)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_numVoltageSensors_High);
    }

	if(USER_NUM_VOLTAGE_SENSORS < 3)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_numVoltageSensors_Low);
    }

	if(USER_OFFSET_POLE_rps > ((float_t)USER_CTRL_FREQ_Hz))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_offsetPole_rps_High);
    }

	if(USER_OFFSET_POLE_rps <= 0.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_offsetPole_rps_Low);
    }

	if(USER_FLUX_POLE_rps > ((float_t)USER_EST_FREQ_Hz))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_fluxPole_rps_High);
    }

	if(USER_FLUX_POLE_rps <= 0.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_fluxPole_rps_Low);
    }

	if(USER_ZEROSPEEDLIMIT > 1.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_zeroSpeedLimit_High);
    }

	if(USER_ZEROSPEEDLIMIT <= 0.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_zeroSpeedLimit_Low);
    }

	if(USER_FORCE_ANGLE_FREQ_Hz > ((float_t)USER_EST_FREQ_Hz))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_forceAngleFreq_Hz_High);
    }

	if(USER_FORCE_ANGLE_FREQ_Hz <= 0.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_forceAngleFreq_Hz_Low);
    }

	if(USER_MAX_ACCEL_Hzps > ((float_t)USER_TRAJ_FREQ_Hz * USER_IQ_FULL_SCALE_FREQ_Hz))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_maxAccel_Hzps_High);
    }

	if(USER_MAX_ACCEL_Hzps <= 0.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_maxAccel_Hzps_Low);
    }

	if(USER_MAX_ACCEL_EST_Hzps > ((float_t)USER_TRAJ_FREQ_Hz * USER_IQ_FULL_SCALE_FREQ_Hz))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_maxAccel_est_Hzps_High);
    }

	if(USER_MAX_ACCEL_EST_Hzps <= 0.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_maxAccel_est_Hzps_Low);
    }

	if(USER_DIRECTION_POLE_rps > ((float_t)USER_EST_FREQ_Hz))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_directionPole_rps_High);
    }

	if(USER_DIRECTION_POLE_rps <= 0.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_directionPole_rps_Low);
    }

	if((USER_SPEED_POLE_rps > USER_IQ_FULL_SCALE_FREQ_Hz) ||
    (USER_SPEED_POLE_rps > ((float_t)USER_EST_FREQ_Hz)))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_speedPole_rps_High);
    }

	if(USER_SPEED_POLE_rps <= 0.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_speedPole_rps_Low);
    }

	if(USER_DCBUS_POLE_rps > ((float_t)USER_EST_FREQ_Hz))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_dcBusPole_rps_High);
    }

	if(USER_DCBUS_POLE_rps <= 0.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_dcBusPole_rps_Low);
    }

	if(USER_FLUX_FRACTION > 1.2)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_fluxFraction_High);
    }

	if(USER_FLUX_FRACTION < 0.05)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_fluxFraction_Low);
    }

	if(USER_SPEEDMAX_FRACTION_FOR_L_IDENT > (USER_IQ_FULL_SCALE_CURRENT_A / USER_MOTOR_MAX_CURRENT))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_indEst_speedMaxFraction_High);
    }

	if(USER_SPEEDMAX_FRACTION_FOR_L_IDENT <= 0.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_indEst_speedMaxFraction_Low);
    }

	if(USER_POWERWARP_GAIN > 2.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_powerWarpGain_High);
    }

	if(USER_POWERWARP_GAIN < 1.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_powerWarpGain_Low);
    }

	if(USER_SYSTEM_FREQ_MHz > 90.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_systemFreq_MHz_High);
    }

	if(USER_SYSTEM_FREQ_MHz <= 0.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_systemFreq_MHz_Low);
    }

	if(USER_PWM_FREQ_kHz > (1000.0 * USER_SYSTEM_FREQ_MHz / 100.0))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_pwmFreq_kHz_High);
    }

	if(USER_PWM_FREQ_kHz < (1000.0 * USER_SYSTEM_FREQ_MHz / 65536.0))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_pwmFreq_kHz_Low);
    }

	if(USER_VOLTAGE_SF >= 128.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_voltage_sf_High);
    }

	if(USER_VOLTAGE_SF < 0.1)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_voltage_sf_Low);
    }

	if(USER_CURRENT_SF >= 128.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_current_sf_High);
    }

	if(USER_CURRENT_SF < 0.1)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_current_sf_Low);
    }

	if(USER_VOLTAGE_FILTER_POLE_Hz > ((float_t)USER_EST_FREQ_Hz / MATH_PI))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_voltageFilterPole_Hz_High);
    }

	if(USER_VOLTAGE_FILTER_POLE_Hz < (USER_IQ_FULL_SCALE_FREQ_Hz / 4.0))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_voltageFilterPole_Hz_Low);
    }

	if(USER_MAX_VS_MAG_PU > (4.0 / 3.0))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_maxVsMag_pu_High);
    }

	if(USER_MAX_VS_MAG_PU <= 0.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_maxVsMag_pu_Low);
    }

	if(USER_EST_KAPPAQ > 1.5)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_estKappa_High);
    }

	if(USER_EST_KAPPAQ < 1.5)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_estKappa_Low);
    }

	if((USER_MOTOR_TYPE != MOTOR_Type_Induction) && (USER_MOTOR_TYPE != MOTOR_Type_Pm))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_motor_type_Unknown);
    }

	if(USER_MOTOR_NUM_POLE_PAIRS < 1)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_motor_numPolePairs_Low);
    }

	if((USER_MOTOR_RATED_FLUX != 0.0) && (USER_MOTOR_TYPE == MOTOR_Type_Pm))
    {
		if(USER_MOTOR_RATED_FLUX > (USER_IQ_FULL_SCALE_FREQ_Hz * 65536.0 / (float_t)USER_EST_FREQ_Hz / 0.7))
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_motor_ratedFlux_High);
        }

		if(USER_MOTOR_RATED_FLUX < (USER_IQ_FULL_SCALE_VOLTAGE_V / (float_t)USER_EST_FREQ_Hz / 0.7))
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_motor_ratedFlux_Low);
        }
    }

	if((USER_MOTOR_RATED_FLUX != 0.0) && (USER_MOTOR_TYPE == MOTOR_Type_Induction))
    {
		if(USER_MOTOR_RATED_FLUX > (USER_IQ_FULL_SCALE_FREQ_Hz * 65536.0 / (float_t)USER_EST_FREQ_Hz / 0.05))
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_motor_ratedFlux_High);
        }

		if(USER_MOTOR_RATED_FLUX < (USER_IQ_FULL_SCALE_VOLTAGE_V / (float_t)USER_EST_FREQ_Hz / 0.05))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_ratedFlux_Low);
        }
    }

	if(USER_MOTOR_TYPE == MOTOR_Type_Pm)
    {
		if(USER_MOTOR_Rr > 0.0)
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rr_High);
        }

		if(USER_MOTOR_Rr < 0.0)
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rr_Low);
        }
    }

	if((USER_MOTOR_Rr != 0.0) && (USER_MOTOR_TYPE == MOTOR_Type_Induction))
    {
		if(USER_MOTOR_Rr > (0.7 * 65536.0 * USER_IQ_FULL_SCALE_VOLTAGE_V / USER_IQ_FULL_SCALE_CURRENT_A))
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rr_High);
        }

		if(USER_MOTOR_Rr < (0.7 * USER_IQ_FULL_SCALE_VOLTAGE_V / (USER_IQ_FULL_SCALE_CURRENT_A * 65536.0)))
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rr_Low);
        }
    }

	if(USER_MOTOR_Rs != 0.0)
    {
		if(USER_MOTOR_Rs > (0.7 * 65536.0 * USER_IQ_FULL_SCALE_VOLTAGE_V / USER_IQ_FULL_SCALE_CURRENT_A))
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rs_High);
        }

		if(USER_MOTOR_Rs < (0.7 * USER_IQ_FULL_SCALE_VOLTAGE_V / (USER_IQ_FULL_SCALE_CURRENT_A * 65536.0)))
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rs_Low);
        }
    }

	if(USER_MOTOR_Ls_d != 0.0)
    {
		if(USER_MOTOR_Ls_d > (0.7 * 65536.0 * USER_IQ_FULL_SCALE_VOLTAGE_V / (USER_IQ_FULL_SCALE_CURRENT_A * USER_VOLTAGE_FILTER_POLE_rps)))
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Ls_d_High);
        }

		if(USER_MOTOR_Ls_d < (0.7 * USER_IQ_FULL_SCALE_VOLTAGE_V / (USER_IQ_FULL_SCALE_CURRENT_A * USER_VOLTAGE_FILTER_POLE_rps * 65536.0)))
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Ls_d_Low);
        }
    }

	if(USER_MOTOR_Ls_q != 0.0)
    {
		if(USER_MOTOR_Ls_q > (0.7 * 65536.0 * USER_IQ_FULL_SCALE_VOLTAGE_V / (USER_IQ_FULL_SCALE_CURRENT_A * USER_VOLTAGE_FILTER_POLE_rps)))
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Ls_q_High);
        }

		if(USER_MOTOR_Ls_q < (0.7 * USER_IQ_FULL_SCALE_VOLTAGE_V / (USER_IQ_FULL_SCALE_CURRENT_A * USER_VOLTAGE_FILTER_POLE_rps * 65536.0)))
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Ls_q_Low);
        }
    }

	if(USER_MOTOR_RES_EST_CURRENT > USER_MOTOR_MAX_CURRENT)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_resEst_High);
    }

	if(USER_MOTOR_RES_EST_CURRENT < 0.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_resEst_Low);
    }

	if(USER_MOTOR_TYPE == MOTOR_Type_Pm)
    {
		if(USER_MOTOR_IND_EST_CURRENT > 0.0)
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_indEst_High);
        }

		if(USER_MOTOR_IND_EST_CURRENT < (-USER_MOTOR_MAX_CURRENT))
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_indEst_Low);
        }
    }

	if(USER_MOTOR_TYPE == MOTOR_Type_Induction)
    {
		if(USER_MOTOR_IND_EST_CURRENT > 0.0)
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_indEst_High);
        }

      if(USER_MOTOR_IND_EST_CURRENT < 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_indEst_Low);
        }
    }

	if(USER_MOTOR_MAX_CURRENT > USER_IQ_FULL_SCALE_CURRENT_A)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_High);
    }

	if(USER_MOTOR_MAX_CURRENT <= 0.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_Low);
    }

	if(USER_MAX_CURRENT_SLOPE > 1.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrentSlope_High);
    }

	if(USER_MAX_CURRENT_SLOPE <= 0.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrentSlope_Low);
    }

	if(USER_MAX_CURRENT_SLOPE_POWERWARP > 1.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrentSlope_powerWarp_High);
    }

	if(USER_MAX_CURRENT_SLOPE_POWERWARP <= 0.0)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrentSlope_powerWarp_Low);
    }

	if(USER_MOTOR_TYPE == MOTOR_Type_Pm)
    {
		if(USER_MOTOR_MAGNETIZING_CURRENT > 0.0)
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_High);
        }

		if(USER_MOTOR_MAGNETIZING_CURRENT < 0.0)
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_Low);
        }
    }

	if(USER_MOTOR_TYPE == MOTOR_Type_Induction)
    {
		if(USER_MOTOR_MAGNETIZING_CURRENT > USER_MOTOR_MAX_CURRENT)
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_High);
        }

		if(USER_MOTOR_MAGNETIZING_CURRENT < 0.0)
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_Low);
        }
    }

	if(USER_MOTOR_TYPE == MOTOR_Type_Induction)
    {
		if(USER_IDRATED_FRACTION_FOR_RATED_FLUX > (USER_IQ_FULL_SCALE_CURRENT_A / (1.2 * USER_MOTOR_MAX_CURRENT)))
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_IdRatedFraction_ratedFlux_High);
        }

		if(USER_IDRATED_FRACTION_FOR_RATED_FLUX < 0.1)
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_IdRatedFraction_ratedFlux_Low);
        }
    }

	if(USER_MOTOR_TYPE == MOTOR_Type_Induction)
    {
		if(USER_IDRATED_FRACTION_FOR_L_IDENT > (USER_IQ_FULL_SCALE_CURRENT_A / USER_MOTOR_MAX_CURRENT))
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_IdRatedFraction_indEst_High);
        }

		if(USER_IDRATED_FRACTION_FOR_L_IDENT < 0.1)
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_IdRatedFraction_indEst_Low);
        }
    }

	if(USER_MOTOR_TYPE == MOTOR_Type_Induction)
    {
		if(USER_IDRATED_DELTA > (USER_IQ_FULL_SCALE_CURRENT_A / ((float_t)USER_NUM_ISR_TICKS_PER_CTRL_TICK * USER_MOTOR_MAX_CURRENT)))
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_delta_High);
        }

		if(USER_IDRATED_DELTA < 0.0)
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_delta_Low);
        }
    }

	if(USER_MOTOR_FLUX_EST_FREQ_Hz > USER_IQ_FULL_SCALE_FREQ_Hz)
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_fluxEstFreq_Hz_High);
    }

	if((USER_MOTOR_FLUX_EST_FREQ_Hz < 0.0) ||
    (USER_MOTOR_FLUX_EST_FREQ_Hz < (USER_ZEROSPEEDLIMIT * USER_IQ_FULL_SCALE_FREQ_Hz)))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_fluxEstFreq_Hz_Low);
    }

	if(USER_MOTOR_Ls_d != 0.0)
    {
		if(((float_t)USER_CTRL_FREQ_Hz >= (128.0 * USER_IQ_FULL_SCALE_VOLTAGE_V / (0.25 * (USER_MOTOR_Ls_d + 1e-9) * USER_IQ_FULL_SCALE_CURRENT_A))))
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_ctrlFreq_Hz_High);
        }
    }

	if(USER_MOTOR_Ls_q != 0.0)
    {
		if(((float_t)USER_CTRL_FREQ_Hz >= (128.0 * USER_IQ_FULL_SCALE_VOLTAGE_V / (0.25 * (USER_MOTOR_Ls_q + 1e-9) * USER_IQ_FULL_SCALE_CURRENT_A))))
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_ctrlFreq_Hz_High);
        }
    }

	if(((float_t)USER_CTRL_FREQ_Hz < USER_IQ_FULL_SCALE_FREQ_Hz) ||
    ((float_t)USER_CTRL_FREQ_Hz < USER_OFFSET_POLE_rps) ||
    ((float_t)USER_CTRL_FREQ_Hz < 250.0) ||
    ((float_t)USER_CTRL_FREQ_Hz <= (2.0 * USER_IQ_FULL_SCALE_FREQ_Hz * USER_MOTOR_MAX_CURRENT / (128.0 * USER_IQ_FULL_SCALE_CURRENT_A))))
    {
		USER_setErrorCode(pUserParams, USER_ErrorCode_ctrlFreq_Hz_Low);
    }

	if((USER_MOTOR_Rs != 0.0) && (USER_MOTOR_Ls_d != 0.0) && (USER_MOTOR_Ls_q != 0.0))
    {
		if(((float_t)USER_CTRL_FREQ_Hz <= (USER_MOTOR_Rs / (USER_MOTOR_Ls_d + 1e-9))) ||
        ((float_t)USER_CTRL_FREQ_Hz <= (USER_MOTOR_Rs / (USER_MOTOR_Ls_q + 1e-9))))
        {
			USER_setErrorCode(pUserParams, USER_ErrorCode_ctrlFreq_Hz_Low);
        }
    }

 	if(((float_t)USER_EST_FREQ_Hz < USER_FORCE_ANGLE_FREQ_Hz) ||
    ((float_t)USER_EST_FREQ_Hz < USER_VOLTAGE_FILTER_POLE_rps) ||
    ((float_t)USER_EST_FREQ_Hz < USER_DCBUS_POLE_rps) ||
    ((float_t)USER_EST_FREQ_Hz < USER_FLUX_POLE_rps) ||
    ((float_t)USER_EST_FREQ_Hz < USER_DIRECTION_POLE_rps) ||
    ((float_t)USER_EST_FREQ_Hz < USER_SPEED_POLE_rps) ||
    ((float_t)USER_EST_FREQ_Hz < 0.2))
    {
 		USER_setErrorCode(pUserParams, USER_ErrorCode_estFreq_Hz_Low);
    }

 	if(USER_R_OVER_L_EST_FREQ_Hz > USER_IQ_FULL_SCALE_FREQ_Hz)
    {
 		USER_setErrorCode(pUserParams, USER_ErrorCode_RoverL_estFreq_Hz_High);
    }

 	if(((float_t)USER_TRAJ_FREQ_Hz < 1.0) ||
    ((float_t)USER_TRAJ_FREQ_Hz < USER_MAX_ACCEL_Hzps / USER_IQ_FULL_SCALE_FREQ_Hz) ||
    ((float_t)USER_TRAJ_FREQ_Hz < USER_MAX_ACCEL_EST_Hzps / USER_IQ_FULL_SCALE_FREQ_Hz))
    {
 		USER_setErrorCode(pUserParams, USER_ErrorCode_trajFreq_Hz_Low);
    }

 	if(USER_MAX_NEGATIVE_ID_REF_CURRENT_A > 0.0)
    {
 		USER_setErrorCode(pUserParams, USER_ErrorCode_maxNegativeIdCurrent_a_High);
    }

 	if(USER_MAX_NEGATIVE_ID_REF_CURRENT_A < (-USER_MOTOR_MAX_CURRENT))
    {
 		USER_setErrorCode(pUserParams, USER_ErrorCode_maxNegativeIdCurrent_a_Low);
    }


} // end of USER_checkForErrors() function


USER_ErrorCode_e USER_getErrorCode(USER_Params *pUserParams)
{
	return(pUserParams->errorCode);
} // end of USER_getErrorCode() function


void USER_setErrorCode(USER_Params *pUserParams,const USER_ErrorCode_e errorCode)
{
	pUserParams->errorCode = errorCode;


} // end of USER_setErrorCode() function


void USER_softwareUpdate1p6(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;
	float_t ffullScaleInductance = USER_IQ_FULL_SCALE_VOLTAGE_V/(USER_IQ_FULL_SCALE_CURRENT_A*USER_VOLTAGE_FILTER_POLE_rps);
	float_t fLs_coarse_max = _IQ30toF(EST_getLs_coarse_max_pu(pobj->estHandle));
	int_least8_t u8lShift = ceil(log(pobj->motorParams.fLs_d_H/(fLs_coarse_max*ffullScaleInductance))/log(2.0));
	uint_least8_t u8Ls_qFmt = 30 - u8lShift;
	float_t fL_max = ffullScaleInductance * pow(2.0,u8lShift);
	_iq iqLs_d_pu = _IQ30(pobj->motorParams.fLs_d_H / fL_max);
	_iq iqLs_q_pu = _IQ30(pobj->motorParams.fLs_q_H / fL_max);

	int32_t i32tmp;
	i32tmp = EST_getLs_d_H(pobj->estHandle);
	fL_max = *((float_t *)&i32tmp);

	// store the results
	EST_setLs_d_pu(pobj->estHandle,iqLs_d_pu);
    EST_setLs_q_pu(pobj->estHandle,iqLs_q_pu);
	EST_setLs_qFmt(pobj->estHandle,u8Ls_qFmt);

	i32tmp = EST_getLs_d_H(pobj->estHandle);
	fL_max = *((float_t *)&i32tmp);


} // end of softwareUpdate1p6() function


void USER_calcPIgains(CTRL_Handle handle)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;
	float_t ffullScaleCurrent = USER_IQ_FULL_SCALE_CURRENT_A;
	float_t ffullScaleVoltage = USER_IQ_FULL_SCALE_VOLTAGE_V;
	float_t fctrlPeriod_sec = CTRL_getCtrlPeriod_sec(handle);
	float_t fLs_d;
	float_t fLs_q;
	float_t fRs;
	float_t fRoverLs_d;
	float_t fRoverLs_q;
	_iq iqKp_Id;
	_iq iqKi_Id;
	_iq iqKp_Iq;
	_iq iqKi_Iq;
	_iq iqKd;

#ifdef __TMS320C28XX_FPU32__
	int32_t i32tmp;

	// when calling EST_ functions that return a float, and fpu32 is enabled, an integer is needed as a return
	// so that the compiler reads the returned value from the accumulator instead of fpu32 registers
	i32tmp = EST_getLs_d_H(pobj->estHandle);
	fLs_d = *((float_t *)&i32tmp);

	i32tmp = EST_getLs_q_H(pobj->estHandle);
	fLs_q = *((float_t *)&i32tmp);

	i32tmp = EST_getRs_Ohm(pobj->estHandle);
	fRs = *((float_t *)&i32tmp);
#else
	fLs_d = EST_getLs_d_H(pobj->estHandle);

	fLs_q = EST_getLs_q_H(pobj->estHandle);

	fRs = EST_getRs_Ohm(pobj->estHandle);
#endif

	fRoverLs_d = fRs/fLs_d;
	iqKp_Id = _IQ((0.25*fLs_d*ffullScaleCurrent)/(fctrlPeriod_sec*ffullScaleVoltage));//$not 2*pi,see in lab4
	iqKi_Id = _IQ(fRoverLs_d*fctrlPeriod_sec);

	fRoverLs_q = fRs/fLs_q;
	iqKp_Iq = _IQ((0.25*fLs_q*ffullScaleCurrent)/(fctrlPeriod_sec*ffullScaleVoltage));
	iqKi_Iq = _IQ(fRoverLs_q*fctrlPeriod_sec);

	iqKd = _IQ(0.0);

	// set the Id controller gains
	PID_setKi(pobj->pidHandle_Id,iqKi_Id);
	CTRL_setGains(handle,CTRL_Type_PID_Id,iqKp_Id,iqKi_Id,iqKd);

	// set the Iq controller gains
	PID_setKi(pobj->pidHandle_Iq,iqKi_Iq);
	CTRL_setGains(handle,CTRL_Type_PID_Iq,iqKp_Iq,iqKi_Iq,iqKd);



} // end of calcPIgains() function


//! \brief     Computes the scale factor needed to convert from torque created by Ld, Lq, Id and Iq, from per unit to Nm
//!
_iq USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf(void)
{
	float_t fFullScaleInductance = (USER_IQ_FULL_SCALE_VOLTAGE_V/(USER_IQ_FULL_SCALE_CURRENT_A*USER_VOLTAGE_FILTER_POLE_rps));
	float_t fFullScaleCurrent = (USER_IQ_FULL_SCALE_CURRENT_A);
	float_t flShift = ceil(log(USER_MOTOR_Ls_d/(0.7*fFullScaleInductance))/log(2.0));

	return(_IQ(fFullScaleInductance*fFullScaleCurrent*fFullScaleCurrent*USER_MOTOR_NUM_POLE_PAIRS*1.5*pow(2.0,flShift)));
} // end of USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf() function


//! \brief     Computes the scale factor needed to convert from torque created by flux and Iq, from per unit to Nm
//!
_iq USER_computeTorque_Flux_Iq_pu_to_Nm_sf(void)
{
	float_t fFullScaleFlux = (USER_IQ_FULL_SCALE_VOLTAGE_V/(float_t)USER_EST_FREQ_Hz);
	float_t fFullScaleCurrent = (USER_IQ_FULL_SCALE_CURRENT_A);
	float_t fmaxFlux = (USER_MOTOR_RATED_FLUX*((USER_MOTOR_TYPE==MOTOR_Type_Induction)?0.05:0.7));
	float_t flShift = -ceil(log(fFullScaleFlux/fmaxFlux)/log(2.0));

	return(_IQ(fFullScaleFlux/(2.0*MATH_PI)*fFullScaleCurrent*USER_MOTOR_NUM_POLE_PAIRS*1.5*pow(2.0,flShift)));
} // end of USER_computeTorque_Flux_Iq_pu_to_Nm_sf() function


//! \brief     Computes the scale factor needed to convert from per unit to Wb
//!
_iq USER_computeFlux_pu_to_Wb_sf(void)
{
	float_t fFullScaleFlux = (USER_IQ_FULL_SCALE_VOLTAGE_V/(float_t)USER_EST_FREQ_Hz);
	float_t fmaxFlux = (USER_MOTOR_RATED_FLUX*((USER_MOTOR_TYPE==MOTOR_Type_Induction)?0.05:0.7));
	float_t flShift = -ceil(log(fFullScaleFlux/fmaxFlux)/log(2.0));

	return(_IQ(fFullScaleFlux/(2.0*MATH_PI)*pow(2.0,flShift)));
} // end of USER_computeFlux_pu_to_Wb_sf() function


//! \brief     Computes the scale factor needed to convert from per unit to V/Hz
//!
_iq USER_computeFlux_pu_to_VpHz_sf(void)
{
	float_t fFullScaleFlux = (USER_IQ_FULL_SCALE_VOLTAGE_V/(float_t)USER_EST_FREQ_Hz);
	float_t fmaxFlux = (USER_MOTOR_RATED_FLUX*((USER_MOTOR_TYPE==MOTOR_Type_Induction)?0.05:0.7));
	float_t flShift = -ceil(log(fFullScaleFlux/fmaxFlux)/log(2.0));

	return(_IQ(fFullScaleFlux*pow(2.0,flShift)));
} // end of USER_computeFlux_pu_to_VpHz_sf() function


//! \brief     Computes Flux in Wb or V/Hz depending on the scale factor sent as parameter
//!
_iq USER_computeFlux(CTRL_Handle handle, const _iq iqsf)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	return(_IQmpy(EST_getFlux_pu(pobj->estHandle),iqsf));
} // end of USER_computeFlux() function


//! \brief     Computes Torque in Nm
//!
_iq USER_computeTorque_Nm(CTRL_Handle handle, const _iq iqtorque_Flux_sf, const _iq iqtorque_Ls_sf)
{
	CTRL_Obj *pobj = (CTRL_Obj *)handle;

	_iq iqFlux_pu = EST_getFlux_pu(pobj->estHandle);
	_iq iqId_pu = PID_getFbackValue(pobj->pidHandle_Id);
	_iq iqIq_pu = PID_getFbackValue(pobj->pidHandle_Iq);
	_iq iqLd_minus_Lq_pu = _IQ30toIQ(EST_getLs_d_pu(pobj->estHandle)-EST_getLs_q_pu(pobj->estHandle));
	_iq iqTorque_Flux_Iq_Nm = _IQmpy(_IQmpy(iqFlux_pu,iqIq_pu),iqtorque_Flux_sf);
	_iq iqTorque_Ls_Id_Iq_Nm = _IQmpy(_IQmpy(_IQmpy(iqLd_minus_Lq_pu,iqId_pu),iqIq_pu),iqtorque_Ls_sf);
	_iq iqTorque_Nm = iqTorque_Flux_Iq_Nm + iqTorque_Ls_Id_Iq_Nm;

 	return(iqTorque_Nm);
} // end of USER_computeTorque_Nm() function


//! \brief     Computes Torque in Nm
//!
_iq USER_computeTorque_lbin(CTRL_Handle handle, const _iq iqtorque_Flux_sf, const _iq iqtorque_Ls_sf)
{
  CTRL_Obj *pobj = (CTRL_Obj *)handle;

  _iq iqFlux_pu = EST_getFlux_pu(pobj->estHandle);
  _iq iqId_pu = PID_getFbackValue(pobj->pidHandle_Id);
  _iq iqIq_pu = PID_getFbackValue(pobj->pidHandle_Iq);
  _iq iqLd_minus_Lq_pu = _IQ30toIQ(EST_getLs_d_pu(pobj->estHandle)-EST_getLs_q_pu(pobj->estHandle));
  _iq iqTorque_Flux_Iq_Nm = _IQmpy(_IQmpy(iqFlux_pu,iqIq_pu),iqtorque_Flux_sf);
  _iq iqTorque_Ls_Id_Iq_Nm = _IQmpy(_IQmpy(_IQmpy(iqLd_minus_Lq_pu,iqId_pu),iqIq_pu),iqtorque_Ls_sf);
  _iq iqTorque_Nm = iqTorque_Flux_Iq_Nm + iqTorque_Ls_Id_Iq_Nm;

  return(_IQmpy(iqTorque_Nm, _IQ(MATH_Nm_TO_lbin_SF)));
} // end of USER_computeTorque_lbin() function


// end of file

