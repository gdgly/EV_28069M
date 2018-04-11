#ifndef _CTRL_STATES_H_
#define _CTRL_STATES_H_

//! \file   ~/dmc_mw/sw/modules/fast/src/ctrl_states.h
//! \brief  Contains the states for the controller (CTRL) object
//!
//! (C) Copyright 2013, Texas Instruments, Inc.


// **************************************************************************
// the includes

//!
//!
//! \defgroup CTRL_STATES CTRL_STATES
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Enumeration for the controller states
//!
typedef enum {//$there only be one state at one time ,so ,use the enum can save for more space
  CTRL_State_Error=0,           //!< the controller error state=0
  CTRL_State_Idle,              //!< the controller idle state=1
  CTRL_State_OffLine,           //!< the controller offline state=2
  CTRL_State_OnLine,            //!< the controller online state=3
  CTRL_numStates                //!< the number of controller states=4
} CTRL_State_e;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup

#endif // end of _CTRL_STATES_H_ definition

