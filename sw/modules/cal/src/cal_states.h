#ifndef _CAL_STATES_H_
#define _CAL_STATES_H_

//! \file   ~/sw/modules/cal/src/cal_states.h
//! \brief  Contains the public interface to the calibration (CAL)
//!         module routines
//!
//! (C) Copyright 2014, Texas Instruments, Inc.


// **************************************************************************
// the includes


// drivers 


// modules


// solutions


//!
//! \defgroup CAL

//!
//! \ingroup CAL
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Defines the CAL states
//!
typedef enum 
{
  CAL_State_Error       = 0, //!< error state
  CAL_State_Idle        = 1, //!< idle state
  CAL_State_AdcOffset   = 2, //!< ADC offset calibration state
  CAL_State_Done        = 3, //!< done state
  CAL_numStates         = 4  //!< the total number of states
} CAL_State_e;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup

#endif // end of _CAL_STATES_H_ definition


