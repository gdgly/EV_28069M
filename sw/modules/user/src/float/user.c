//! \file   ~/sw/modules/user/src/float/user.c
//! \brief  Contains the user related functions
//!
//! (C) Copyright 2014, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "user.h"


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs


// **************************************************************************
// the functions


void USER_setParams(USER_Params *pUserParams)
{
  pUserParams->dcBus_nominal_V = USER_NOMINAL_DC_BUS_VOLTAGE_V;

  pUserParams->numIsrTicksPerCtrlTick = USER_NUM_ISR_TICKS_PER_CTRL_TICK;
  pUserParams->numIsrTicksPerEstTick = USER_NUM_ISR_TICKS_PER_EST_TICK;
  pUserParams->numIsrTicksPerTrajTick = USER_NUM_ISR_TICKS_PER_TRAJ_TICK;

  pUserParams->numCtrlTicksPerCurrentTick = USER_NUM_CTRL_TICKS_PER_CURRENT_TICK;
  pUserParams->numCtrlTicksPerSpeedTick = USER_NUM_CTRL_TICKS_PER_SPEED_TICK;

  pUserParams->numCurrentSensors = USER_NUM_CURRENT_SENSORS;
  pUserParams->numVoltageSensors = USER_NUM_VOLTAGE_SENSORS;

  pUserParams->systemFreq_MHz = USER_SYSTEM_FREQ_MHz;

  pUserParams->pwmPeriod_usec = USER_PWM_PERIOD_usec;

  pUserParams->voltage_sf = USER_VOLTAGE_SF;

  pUserParams->current_sf = USER_CURRENT_SF;

  pUserParams->offsetPole_rps = USER_OFFSET_POLE_rps;

  pUserParams->voltageFilterPole_rps = USER_VOLTAGE_FILTER_POLE_rps;

  pUserParams->maxDutyCycle = USER_MAX_DUTY_CYCLE;

  pUserParams->motor_type = USER_MOTOR_TYPE;
  pUserParams->motor_numPolePairs = USER_MOTOR_NUM_POLE_PAIRS;
  pUserParams->motor_ratedFlux_Wb = USER_MOTOR_RATED_FLUX_VpHz / MATH_TWO_PI;
  pUserParams->motor_Rr_d_Ohm = USER_MOTOR_Rr_Ohm;
  pUserParams->motor_Rr_q_Ohm = USER_MOTOR_Rr_Ohm;
  pUserParams->motor_Rs_a_Ohm = USER_MOTOR_Rs_Ohm;
  pUserParams->motor_Rs_b_Ohm = USER_MOTOR_Rs_Ohm;
  pUserParams->motor_Rs_d_Ohm = USER_MOTOR_Rs_Ohm;
  pUserParams->motor_Rs_q_Ohm = USER_MOTOR_Rs_Ohm;
  pUserParams->motor_Ls_d_H = USER_MOTOR_Ls_d_H;
  pUserParams->motor_Ls_q_H = USER_MOTOR_Ls_q_H;
  pUserParams->motor_numEncSlots = USER_MOTOR_NUM_ENC_SLOTS;

  pUserParams->maxCurrent_A = USER_MOTOR_MAX_CURRENT_A;

  pUserParams->IdRated_A = USER_MOTOR_MAGNETIZING_CURRENT_A;

  pUserParams->Vd_sf = USER_VD_SF;
  pUserParams->maxVsMag_V = USER_NOMINAL_DC_BUS_VOLTAGE_V;

  // Marathon 56H17T2001
//  pUserParams->BWc_rps = MATH_TWO_PI * 100.0;
//  pUserParams->Kctrl_Wb_p_kgm2 = 3.0 * pUserParams->motor_numPolePairs * 0.1 / (2.0 * 0.0001); // 3.0 * numPolesPairs * rotorFlux_Wb / (2.0 * J_kg_m2);
  // Estun
  pUserParams->BWc_rps = MATH_TWO_PI * 40.0;
  pUserParams->Kctrl_Wb_p_kgm2 = 3.0 * pUserParams->motor_numPolePairs * 0.0625 / (2.0 * 0.000031); // 3.0 * numPolesPairs * rotorFlux_Wb / (2.0 * J_kg_m2);

  pUserParams->BWdelta = 4.0;
  pUserParams->fluxExcFreq_Hz = USER_MOTOR_FLUX_EXC_FREQ_Hz;

  pUserParams->calWaitTime[CAL_State_Error]           = 0;
  pUserParams->calWaitTime[CAL_State_Idle]            = 0;
  pUserParams->calWaitTime[CAL_State_AdcOffset]       = (int_least32_t)( 5.0 * USER_CTRL_FREQ_Hz);
  pUserParams->calWaitTime[CAL_State_Done]            = 0;

  pUserParams->ctrlWaitTime[CTRL_State_Error]         = 0;
  pUserParams->ctrlWaitTime[CTRL_State_Idle]          = 0;
  pUserParams->ctrlWaitTime[CTRL_State_OnLine]        = 0;

  pUserParams->estWaitTime[EST_State_Error]           = 0;
  pUserParams->estWaitTime[EST_State_Idle]            = 0;
  pUserParams->estWaitTime[EST_State_RoverL]          = (int_least32_t)( 5.0 * USER_EST_FREQ_Hz);
  pUserParams->estWaitTime[EST_State_Rs]              = 0;
  pUserParams->estWaitTime[EST_State_RampUp]          = (int_least32_t)(20.0 * USER_EST_FREQ_Hz);
  pUserParams->estWaitTime[EST_State_IdRated]         = (int_least32_t)(10.0 * USER_EST_FREQ_Hz);
  pUserParams->estWaitTime[EST_State_RatedFlux_OL]    = (int_least32_t)( 0.2 * USER_EST_FREQ_Hz);
  pUserParams->estWaitTime[EST_State_RatedFlux]       = 0;
  pUserParams->estWaitTime[EST_State_RampDown]        = (int_least32_t)( 2.0 * USER_EST_FREQ_Hz);
  pUserParams->estWaitTime[EST_State_LockRotor]       = 0;
  pUserParams->estWaitTime[EST_State_Ls]              = 0;
  pUserParams->estWaitTime[EST_State_Rr]              = (int_least32_t)( 5.0 * USER_EST_FREQ_Hz);
  pUserParams->estWaitTime[EST_State_MotorIdentified] = 0;
  pUserParams->estWaitTime[EST_State_OnLine]          = 0;

  pUserParams->FluxWaitTime[EST_Flux_State_Error]     = 0;
  pUserParams->FluxWaitTime[EST_Flux_State_Idle]      = 0;
  pUserParams->FluxWaitTime[EST_Flux_State_CL1]       = (int_least32_t)( 3.0 * USER_EST_FREQ_Hz);
  pUserParams->FluxWaitTime[EST_Flux_State_CL2]       = (int_least32_t)( 0.2 * USER_EST_FREQ_Hz);
  pUserParams->FluxWaitTime[EST_Flux_State_Fine]      = (int_least32_t)( 4.0 * USER_EST_FREQ_Hz);
  pUserParams->FluxWaitTime[EST_Flux_State_Done]      = 0;

  pUserParams->LsWaitTime[EST_Ls_State_Error]        = 0;
  pUserParams->LsWaitTime[EST_Ls_State_Idle]         = 0;
  pUserParams->LsWaitTime[EST_Ls_State_RampUp]       = (int_least32_t)( 3.0 * USER_EST_FREQ_Hz);
  pUserParams->LsWaitTime[EST_Ls_State_Init]         = 0; // state not used in floating point code
  pUserParams->LsWaitTime[EST_Ls_State_Coarse]       = (int_least32_t)(10.0 * USER_EST_FREQ_Hz);
  pUserParams->LsWaitTime[EST_Ls_State_Fine]         = (int_least32_t)(20.0 * USER_EST_FREQ_Hz);
  pUserParams->LsWaitTime[EST_Ls_State_Done]         = 0;

  pUserParams->RrWaitTime[EST_Rr_State_Error]        = 0;
  pUserParams->RrWaitTime[EST_Rr_State_Idle]         = 0;
  pUserParams->RrWaitTime[EST_Rr_State_RampUp]       = (int_least32_t)( 1.0 * USER_EST_FREQ_Hz);
  pUserParams->RrWaitTime[EST_Rr_State_Coarse]       = (int_least32_t)( 2.0 * USER_EST_FREQ_Hz);
  pUserParams->RrWaitTime[EST_Rr_State_Fine]         = (int_least32_t)( 4.0 * USER_EST_FREQ_Hz);
  pUserParams->RrWaitTime[EST_Rr_State_Done]         = 0;

  pUserParams->RsWaitTime[EST_Rs_State_Error]        = 0;
  pUserParams->RsWaitTime[EST_Rs_State_Idle]         = 0;
  pUserParams->RsWaitTime[EST_Rs_State_RampUp]       = (int_least32_t)( 1.0 * USER_EST_FREQ_Hz);
  pUserParams->RsWaitTime[EST_Rs_State_Coarse]       = (int_least32_t)( 2.0 * USER_EST_FREQ_Hz);
  pUserParams->RsWaitTime[EST_Rs_State_Fine]         = (int_least32_t)( 4.0 * USER_EST_FREQ_Hz);
  pUserParams->RsWaitTime[EST_Rs_State_Done]         = 0;

  pUserParams->trajWaitTime[EST_Traj_State_Error]    = 0;
  pUserParams->trajWaitTime[EST_Traj_State_Idle]     = 0;
  pUserParams->trajWaitTime[EST_Traj_State_Est]      = 0;
  pUserParams->trajWaitTime[EST_Traj_State_OnLine]   = 0;

  pUserParams->ctrlFreq_Hz = USER_CTRL_FREQ_Hz;

  pUserParams->estFreq_Hz = USER_EST_FREQ_Hz;

  pUserParams->RoverL_excFreq_Hz = USER_R_OVER_L_EXC_FREQ_Hz;

  pUserParams->trajFreq_Hz = USER_TRAJ_FREQ_Hz;

  pUserParams->ctrlPeriod_sec = USER_CTRL_PERIOD_sec;

  pUserParams->flag_bypassMotorId = USER_BYPASS_MOTOR_ID;

  return;
} // end of USER_setParams() function


// end of file

