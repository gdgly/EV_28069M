#ifndef _CTRL_OBJ_H_
#define _CTRL_OBJ_H_
//! \file   ~/sw/modules/ctrl/src/float/ctrl_obj.h
//! \brief Defines the structures for the CTRL object 
//!
//! (C) Copyright 2014, Texas Instruments, Inc.


// **************************************************************************
// the includes

// drivers


// modules
#include "sw/modules/types/src/types.h"
#include "sw/modules/ctrl/src/ctrl_states.h"
#include "sw/modules/math/src/float/math.h"
#include "sw/modules/motor/src/float/motor.h"
#include "sw/modules/pi/src/float/pi.h"


// solutions


//!
//!
//! \defgroup CTRL_OBJ CTRL_OBJ
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


//! \brief Defines the major release number
//!
#define CTRL_MAJOR_RELEASE_NUMBER       (1)


//! \brief Defines the major release number
//!
#define CTRL_MINOR_RELEASE_NUMBER       (7)


//! \brief Defines the number of controllers
//!
#define CTRL_NUM_CONTROLLERS            (2)


// **************************************************************************
// the typedefs

//! \brief Enumeration for the error codes
//!
typedef enum
{
  CTRL_ErrorCode_NoError=0,        //!< no error error code
  CTRL_ErrorCode_IdClip,           //!< Id clip error code
  CTRL_ErrorCode_EstError,         //!< estimator error code
  CTRL_numErrorCodes               //!< the number of controller error codes
} CTRL_ErrorCode_e;


//! \brief Enumeration for the target processors
//!
typedef enum
{
  CTRL_TargetProc_2806x=0,   //!< 2806x processor
  CTRL_TargetProc_2805x,     //!< 2805x processor
  CTRL_TargetProc_2803x,     //!< 2803x processor
  CTRL_TargetProc_2802x      //!< 2802x processor
} CTRL_TargetProc_e;


//! \brief Enumeration for the controller (CTRL) types
//!
typedef enum
{
  CTRL_Type_PI_spd=0,        //!< PI Speed controller
  CTRL_Type_PI_Id,           //!< PI Id controller
  CTRL_Type_PI_Iq            //!< PI Iq controller
} CTRL_Type_e;


//! \brief Defines the controller (CTRL) version number
//!
typedef struct _CTRL_Version_
{
  uint16_t rsvd;            //!< reserved value
  uint16_t targetProc;      //!< the target processor
  uint16_t major;           //!< the major release number
  uint16_t minor;           //!< the minor release number
} CTRL_Version;


//! \brief Defines the controller (CTRL) object
//!
typedef struct _CTRL_Obj_
{
  CTRL_Version       version;                      //!< the controller version

  CTRL_State_e       state;                        //!< the current state of the controller

  CTRL_State_e       prevState;                    //!< the previous state of the controller

  CTRL_ErrorCode_e   errorCode;                    //!< the error code for the controller

  PI_Handle          piHandle_Id;                  //!< the handle for the Id PI controller
  PI_Obj             pi_Id;                        //!< the Id PI controller object

  PI_Handle          piHandle_Iq;                  //!< the handle for the Iq PI controller
  PI_Obj             pi_Iq;                        //!< the Iq PI controller object

  PI_Handle          piHandle_spd;                 //!< the handle for the speed PI controller
  PI_Obj             pi_spd;                       //!< the speed PI controller object

  MOTOR_Params       motorParams;                  //!< the motor parameters

  int_least32_t      waitTimes[CTRL_numStates];    //!< an array of wait times for each state, isr clock counts

  int_least32_t      counter_state;                //!< the state counter
  
  int_least16_t      numIsrTicksPerCtrlTick;       //!< Defines the number of isr clock ticks per controller clock tick

  int_least16_t      numCtrlTicksPerCurrentTick;   //!< Defines the number of controller clock ticks per current controller clock tick

  int_least16_t      numCtrlTicksPerSpeedTick;     //!< Defines the number of controller clock ticks per speed controller clock tick

  uint_least32_t     ctrlFreq_Hz;                  //!< Defines the controller frequency, Hz

  float_t            ctrlPeriod_sec;               //!< Defines the controller period, sec
  float_t            currentCtrlPeriod_sec;        //!< the period at which the current controller runs, sec
  float_t            speedCtrlPeriod_sec;          //!< the period at which the speed controller runs, sec

  MATH_vec2          Idq_A;                        //!< the Idq values, A

  MATH_vec2          Idq_ref_A;                    //!< the Idq reference values, A

  MATH_vec2          Idq_offset_A;                 //!< the Idq offset values, A

  MATH_vec2          Vdq_V;                        //!< the Vdq values, V

  MATH_vec2          Vdq_offset_V;                 //!< the Vdq offset values, V

  float_t            Vd_sf;                        //!< the Vd scale factor

  float_t            maxVsMag_V;                   //!< the maximum stator voltage magnitude value, V

  float_t            Kp_Id_VpA;                    //!< the desired Kp_Id value, V/A
  float_t            Kp_Iq_VpA;                    //!< the desired Kp_Iq value, V/A
  float_t            Kp_spd_ApHz;                  //!< the desired Kp_spd value, A/Hz

  float_t            Ki_Id;                        //!< the desired Ki_Id value, unitless
  float_t            Ki_Iq;                        //!< the desired Ki_Iq value, unitless
  float_t            Ki_spd_ApHz;                  //!< the desired Ki_spd value, A/Hz

  float_t            Ui_Id_V;                      //!< the start integrator value for the Id controller, V
  float_t            Ui_Iq_V;                      //!< the start integrator value for the Iq controller, V
  float_t            Ui_spd_A;                     //!< the start integrator value for the speed controller, A

  float_t            BWc_rps;                      //!< the bandwidth of the current controllers, rad/sec
  float_t            BWdelta;                      //!< the bandwidth scaling to maximize phase margin
  float_t            Kctrl_Wb_p_kgm2;              //!< the controller constant, Wb/(kg*m^2)

  float_t            speed_ref_Hz;                 //!< the reference speed value, Hz

  float_t            speed_fb_Hz;                  //!< the feedback speed value, Hz

  float_t            speed_out_A;                  //!< the output value from the speed controller, A
  float_t            speed_outMax_A;               //!< the maximum output value for the speed controller, A

  int_least16_t      counter_isr;                  //!< the isr counter
  int_least16_t      counter_current;              //!< the isr counter
  int_least16_t      counter_speed;                //!< the speed counter

  bool               flag_enable;                  //!< a flag to enable the controller           
  bool               flag_enableCurrentCtrl;       //!< a flag to enable the current controllers
  bool               flag_enableSpeedCtrl;         //!< a flag to enable the speed controller

  bool               flag_updateKi_spd;            //!< a flag to update Ki_spd
  bool               flag_updateKi_Id;             //!< a flag to update Ki_Id
  bool               flag_updateKi_Iq;             //!< a flag to update Ki_Iq

  bool               flag_resetInt_spd;            //!< a flag to reset the speed integrator
  bool               flag_resetInt_Id;             //!< a flag to reset the Id integrator
  bool               flag_resetInt_Iq;             //!< a flag to reset the Iq integrator

  bool               flag_useZeroIq_ref;           //!< a flag to force a Iq = 0 reference value
} CTRL_Obj;


//! \brief Defines the CTRL handle
//!
typedef struct _CTRL_Obj_ *CTRL_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup

#endif // end of _CTRL_OBJ_H_ definition

