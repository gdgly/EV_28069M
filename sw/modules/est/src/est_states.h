#ifndef _EST_STATES_H_
#define _EST_STATES_H_

//! \file   ~/sw/modules/est/src/est_states.h
//! \brief  Contains the states for the estimator (EST) module routines
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the includes

//!
//!
//! \defgroup EST_STATES EST_STATES
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the includes


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Enumeration for the estimator error codes
//!
typedef enum
{
  EST_ErrorCode_NoError=0,               //!< no error error code
  EST_ErrorCode_Flux_OL_ShiftOverFlow,   //!< flux open loop shift overflow error code
  EST_ErrorCode_FluxError,               //!< flux estimator error code
  EST_ErrorCode_Dir_ShiftOverFlow,       //!< direction shift overflow error code
  EST_ErrorCode_Ind_ShiftOverFlow,       //!< inductance shift overflow error code
  EST_numErrorCodes                      //!< the number of estimator error codes
} EST_ErrorCode_e;


//! \brief Enumeration for the estimator states
//!
typedef enum
{
  EST_State_Error=0,            //!< error                                      0
  EST_State_Idle,               //!< idle                                       1
  EST_State_RoverL,             //!< R/L estimation                             2
  EST_State_Rs,                 //!< Rs estimation state                        3
  EST_State_RampUp,             //!< ramp up the speed                          4
  EST_State_IdRated,            //!< control Id and estimate the rated flux     5
  EST_State_RatedFlux_OL,       //!< estimate the open loop rated flux          6
  EST_State_RatedFlux,          //!< estimate the rated flux                    7
  EST_State_RampDown,           //!< ramp down the speed                        8
  EST_State_LockRotor,          //!< lock the rotor                             9
  EST_State_Ls,                 //!< stator inductance estimation state         10
  EST_State_Rr,                 //!< rotor resistance estimation state          11
  EST_State_MotorIdentified,    //!< motor identified state                     12
  EST_State_OnLine,             //!< online parameter estimation                13

  EST_numStates                 //!< the number of estimator states:14
} EST_State_e;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _EST_STATES_H_ definition

