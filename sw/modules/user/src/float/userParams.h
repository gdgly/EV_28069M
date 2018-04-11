#ifndef _USERPARAMS_H_
#define _USERPARAMS_H_

//! \file   ~/sw/modules/user/src/float/userParams.h
//! \brief  Contains the user related definitions
//!
//! (C) Copyright 2014, Texas Instruments, Inc.


// **************************************************************************
// the includes

// modules
#include "sw/modules/types/src/types.h"
#include "sw/modules/motor/src/float/motor.h"
#include "sw/modules/cal/src/cal_states.h"
#include "sw/modules/ctrl/src/ctrl_states.h"
#include "sw/modules/est/src/est_Flux_states.h"
#include "sw/modules/est/src/est_Ls_states.h"
#include "sw/modules/est/src/est_Rr_states.h"
#include "sw/modules/est/src/est_Rs_states.h"
#include "sw/modules/est/src/est_Traj_states.h"
#include "sw/modules/est/src/est_states.h"


// platforms


//!
//!
//! \defgroup USERPARAMS USERPARAMS
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Defines a structure for the user parameters
//!
typedef struct _USER_Params_
{
  float_t         dcBus_nominal_V;              //!< Defines the nominal DC bus voltage, V

  int_least16_t   numIsrTicksPerCtrlTick;       //!< Defines the number of Interrupt Service Routine (ISR) clock ticks per controller clock tick
  int_least16_t   numIsrTicksPerEstTick;        //!< Defines the number of Interrupt Service Routine (ISR) clock ticks per controller clock tick
  int_least16_t   numIsrTicksPerTrajTick;       //!< Defines the number of Interrupt Service Routine (ISR) clock ticks per controller clock tick

  int_least16_t   numCtrlTicksPerCurrentTick;   //!< Defines the number of controller clock ticks per current controller clock tick
  int_least16_t   numCtrlTicksPerSpeedTick;     //!< Defines the number of controller clock ticks per speed controller clock tick

  uint_least8_t   numCurrentSensors;            //!< Defines the number of current sensors
  uint_least8_t   numVoltageSensors;            //!< Defines the number of voltage sensors

  uint_least16_t  systemFreq_MHz;               //!< Defines the system clock frequency, MHz

  float_t         pwmPeriod_usec;               //!< Defines the Pulse Width Modulation (PWM) period, usec

  float_t         voltage_sf;                   //!< Defines the voltage scale factor for the system

  float_t         current_sf;                   //!< Defines the current scale factor for the system

  float_t         offsetPole_rps;               //!< Defines the pole location for the voltage and current offset estimation, rad/sec

  float_t         voltageFilterPole_rps;        //!< Defines the analog voltage filter pole location, rad/sec

  float_t         maxDutyCycle;                 //!< Defines the maximum duty cycle

  MOTOR_Type_e    motor_type;                   //!< Defines the motor type

  uint_least16_t  motor_numPolePairs;           //!< Defines the number of pole pairs for the motor
  uint_least16_t  motor_numEncSlots;            //!< Defines the number of encoder slots if quadrature encoder is connected

  float_t         motor_ratedFlux_Wb;           //!< Defines the rated flux of the motor, Wb

  float_t         motor_Rr_d_Ohm;               //!< Defines the direct rotor resistance, Ohm
  float_t         motor_Rr_q_Ohm;               //!< Defines the quadrature rotor resistance, Ohm

  float_t         motor_Rs_a_Ohm;               //!< Defines the alpha stator resistance, Ohm
  float_t         motor_Rs_b_Ohm;               //!< Defines the beta stator resistance, Ohm

  float_t         motor_Rs_d_Ohm;               //!< Defines the direct stator resistance, Ohm
  float_t         motor_Rs_q_Ohm;               //!< Defines the quadrature stator resistance, Ohm

  float_t         motor_Ls_d_H;                 //!< Defines the direct stator inductance, H
  float_t         motor_Ls_q_H;                 //!< Defines the quadrature stator inductance, H

  float_t         maxCurrent_A;                 //!< Defines the maximum current value, A

  float_t         IdRated_A;                    //!< Defines the Id rated current value, A

  float_t         Vd_sf;                        //!< Defines the Vd scale factor to prevent a Vd only component for the Vdq vector
  float_t         maxVsMag_V;                   //!< Defines the maximum stator voltage magnitude, V

  float_t         BWc_rps;                      //!< Defines the bandwidth of the current controllers, rad/sec
  float_t         BWdelta;                      //!< Defines the bandwidth scaling to maximize phase margin
  float_t         Kctrl_Wb_p_kgm2;              //!< Defines the speed controller constant, Wb/(kg*m^2)

  float_t         angleDelay_sf_sec;            //!< Defines the scale factor for computing additional angle delay, sec

  float_t         fluxExcFreq_Hz;               //!< Defines the flux excitation frequency, Hz

  int_least32_t   calWaitTime[CAL_numStates];   //!< Defines the wait times for each calibrator state, isr ticks

  int_least32_t   ctrlWaitTime[CTRL_numStates]; //!< Defines the wait times for each controller state, isr ticks

  int_least32_t   estWaitTime[EST_numStates];   //!< Defines the wait times for each estimator state, isr ticks

  int_least32_t   FluxWaitTime[EST_Flux_numStates]; //!< Defines the wait times for each Ls estimator state, estimator ticks

  int_least32_t   LsWaitTime[EST_Ls_numStates]; //!< Defines the wait times for each Ls estimator state, estimator ticks

  int_least32_t   RrWaitTime[EST_Rr_numStates]; //!< Defines the wait times for each Rr estimator state, estimator ticks

  int_least32_t   RsWaitTime[EST_Rs_numStates]; //!< Defines the wait times for each Rs estimator state, estimator ticks

  int_least32_t   trajWaitTime[EST_Traj_numStates]; //!< Defines the wait times for each trajectory state, isr ticks

  uint_least32_t  ctrlFreq_Hz;                  //!< Defines the controller frequency, Hz

  uint_least32_t  estFreq_Hz;                   //!< Defines the estimator frequency, Hz

  uint_least32_t  RoverL_excFreq_Hz;            //!< Defines the R/L excitation frequency, Hz

  uint_least32_t  trajFreq_Hz;                  //!< Defines the trajectory frequency, Hz

  float_t         ctrlPeriod_sec;               //!< Defines the controller execution period, sec

  bool            flag_bypassMotorId;           //!< A flag to bypass motor identification and use the motor parameters

  float_t         Reserved[60];                 //!< Reserved Section

} USER_Params;


// **************************************************************************
// the functions


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _USERPARAMS_H_ definition

