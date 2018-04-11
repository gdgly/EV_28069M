#ifndef _EST_H_
#define _EST_H_

//! \file   ~/sw/modules/est/src/float/est.h
//! \brief  Contains the public interface to the 
//!         estimator (EST) module routines
//!
//! (C) Copyright 2014, Texas Instruments, Inc.


// **************************************************************************
// the includes

// modules
#include "sw/modules/math/src/float/math.h"

#include "sw/modules/ctrl/src/float/ctrl_obj.h"
#include "sw/modules/est/src/est_states.h"
#include "sw/modules/user/src/float/userParams.h"

//!
//!
//! \defgroup EST EST
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup EST_OVERVIEW 

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//! \brief Enumeration for the Rs online filter types
//!
typedef enum
{
  EST_RsOnLineFilterType_Current=0,        //!< Current Filter
  EST_RsOnLineFilterType_Voltage           //!< Voltage Filter
} EST_RsOnLineFilterType_e;


//! \brief Defines the estimator (EST) input data
//!
typedef struct _EST_InputData_
{
  int_least32_t    timeStamp;        //!< a time stamp for the input data buffer

  MATH_vec2        Iab_A;            //!< the alpha/beta current values, A
  MATH_vec2        Vab_V;            //!< the alpha/beta current values, V
  float_t          dcBus_V;          //!< the DC bus voltage value, V
  float_t          speed_ref_Hz;     //!< the speed reference value, Hz
} EST_InputData_t;


//! \brief Defines the estimator (EST) output data
//!
typedef struct _EST_OutputData_
{
  int_least32_t      timeStamp;                 //!< a time stamp for the output data buffer

  float_t            flux_Wb;                   //!< the flux estimate, Wb

  float_t            angle_rad;                 //!< the estimated angle value at t = m+1, rad

  float_t            fe_rps;                    //!< the electrical frequency estimate, rad/sec
  float_t            fm_rps;                    //!< the mechanical frequency estimate, rad/sec
  float_t            fm_lp_rps;                 //!< the low pass filtered mechanical frequency estimate, rad/sec
  float_t            fmDot_rps2;                //!< the mechanical acceleration estimate, rad/sec^2
  float_t            fslip_rps;                 //!< the slip frequency estimate, rad/sec

  float_t            torque_Nm;                 //!< the estimated electrical torque, N*m

  MATH_vec2          Eab_V;                     //!< the alpha/beta back-EMF estimates, V
  MATH_vec2          Edq_V;                     //!< the direction/quadrature back-EMF estimates, V

  int_least8_t       direction;                 //!< the rotational direction estimate, unitless

  float_t            oneOverDcBus_invV;         //!< the DC Bus inverse, 1/V
} EST_OutputData_t;


//! \brief Defines the estimator (EST) handle
//!
typedef struct _EST_Obj_ *EST_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

//! \brief     Computes the magnetizing inductance in Henries (H)
//! \param[in] handle   The estimator (EST) handle
//! \param[in] current  The current in the rotor, A
//! \return    The magnetizing inductance, H
extern float_t EST_computeLmag_H(EST_Handle handle,const float_t current_A);


//! \brief     Computes the power in Watts (W)
//! \param[in] handle  The estimator (EST) handle
//! \return    The power value, W
extern float_t EST_computePower_W(EST_Handle handle);


//! \brief     Computes the torque value in per Newton-meter (Nm)
//! \param[in] handle  The estimator (EST) handle
//! \return    The torque value, N*m
extern float_t EST_computeTorque_Nm(EST_Handle handle);


//! \brief     Configures the controller for each of the estimator states
//! \param[in] handle      The estimator (EST) handle
//! \param[in] ctrlHandle  The controller (CTRL) handle
extern void EST_configureCtrl(EST_Handle handle,CTRL_Handle ctrlHandle);


//! \brief     Configures the trajectory generator for each of the estimator states
//! \param[in] handle  The estimator (EST) handle
extern void EST_configureTraj(EST_Handle handle);


//! \brief     Disables the estimator
//! \param[in] handle  The estimator (EST) handle
extern void EST_disable(EST_Handle handle);


//! \brief     Disables the estimator trajectory generator
//! \param[in] handle  The estimator (EST) handle
extern void EST_disableTraj(EST_Handle handle);


//! \brief     Determines if current control should be performed during motor identification
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting whether (true) or not (false) to perform current control
extern bool EST_doCurrentCtrl(EST_Handle handle);


//! \brief     Determines if speed control should be performed during motor identification
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting whether (true) or not (false) to perform speed control
extern bool EST_doSpeedCtrl(EST_Handle handle);


//! \brief     Enables the estimator
//! \param[in] handle  The estimator (EST) handle
extern void EST_enable(EST_Handle handle);


//! \brief     Enables the estimator trajectory generator
//! \param[in] handle  The estimator (EST) handle
extern void EST_enableTraj(EST_Handle handle);


//! \brief     Gets the krpm to pu scale factor in per unit (pu), IQ24.
//! \details   This function is needed when a user needs to scale a value of the motor speed from 
//!            kpm (kilo revolutions per minute) to a per units value. This scale factor is calculated
//!            and used as shown below:
//! \code
//! #define USER_MOTOR_NUM_POLE_PAIRS       (2)
//! #define USER_IQ_FULL_SCALE_FREQ_Hz      (500.0)
//!
//! _iq scale_factor = _IQ(USER_MOTOR_NUM_POLE_PAIRS * 1000.0 / (60.0 * USER_IQ_FULL_SCALE_FREQ_Hz));
//!
//! _iq Speed_krpm = EST_getSpeed_krpm(handle);
//! _iq Speed_krpm_to_pu_sf = EST_get_krpm_to_pu_sf(handle);
//! _iq Speed_pu = _IQmpy(Speed_krpm,Speed_krpm_to_pu_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The krpm to pu scale factor. This value is in IQ24.
extern float_t EST_get_krpm_to_pu_sf(EST_Handle handle);


//! \brief     Gets the pu to krpm scale factor in per unit (pu), IQ24.
//! \details   This function is needed when a user needs to scale a value of the motor speed from per 
//!            units to krpm (kilo revolutions per minute) value. This scale factor is calculated as follows:
//! \code
//! #define USER_MOTOR_NUM_POLE_PAIRS       (2)
//! #define USER_IQ_FULL_SCALE_FREQ_Hz      (500.0)
//!
//! _iq scale_factor = IQ(60.0 * USER_IQ_FULL_SCALE_FREQ_Hz / (USER_MOTOR_NUM_POLE_PAIRS * 1000.0));
//!
//! _iq Speed_pu = EST_getFm_pu(handle);
//! _iq Speed_pu_to_krpm_sf = EST_get_pu_to_krpm_sf(handle);
//! _iq Speed_krpm = _IQmpy(Speed_krpm,Speed_krpm_to_pu_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The pu to krpm scale factor. This value is in IQ24.
extern float_t EST_get_pu_to_krpm_sf(EST_Handle handle);


//! \brief     Gets the mechanical acceleration from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The mechanical acceleration, rad/sec^2
extern float_t EST_getAccel_rps2(EST_Handle handle);


//! \brief     Gets the estimated angle from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The estimated angle, rad
extern float_t EST_getAngle_est_rad(EST_Handle handle);


//! \brief     Gets the angle estimate for t = n+1 from the estimator in radians (rad).
//! \details   This function returns the angle value in units of radians.  This value wraps around 
//!            at 2*pi, so the return value is between -pi and pi.
//!            An example of using this angle is shown:
//! \code
//! float_t rotorFluxAngle_rad = EST_getAngle_rad(handle);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The angle value for t = n+1, rad
extern float_t EST_getAngle_rad(EST_Handle handle);


//! \brief     Gets the angle delta estimate for t = n+1 from the estimator in radians (rad).
//! \param[in] handle  The estimator (EST) handle
//! \return    The angle delta value for t = n+1, rad
extern float_t EST_getAngleDelta_rad(EST_Handle handle);


//! \brief     Gets the trajectory angle from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The trajectory angle, rad
extern float_t EST_getAngle_traj_rad(EST_Handle handle);


//! \brief     Gets the alpha/beta back EMF voltage vector from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] pEab_V  The pointer to memory for the Eab vector, V
extern void EST_getEab_V(EST_Handle handle,MATH_vec2 *pEab_V);


//! \brief     Gets the direct/quadrature back EMF voltage vector from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] pEdq_V  The pointer to memory for the Edq vector, V
extern void EST_getEdq_V(EST_Handle handle,MATH_vec2 *pEdq_V);


//! \brief     Gets the ISR count from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The ISR count
extern int_least16_t EST_getCount_isr(EST_Handle handle);


//! \brief     Gets the trajectory ISR count from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The trajectory count value
extern int_least16_t EST_getTrajCount_isr(EST_Handle handle);


//! \brief     Sets the ISR count in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] value   The ISR count value
extern void EST_setCount_isr(EST_Handle handle,const int_least16_t value);


//! \brief     Gets the DC bus value from the estimator in volts (V)
//! \details   This value is originally passed as a parameter when calling function EST_run(). 
//!            A similar function can be simply reading what has been read and scaled by the ADC converter
//!            on pAdcData->dcBus. This value is used by the libraries internally to calculate one over 
//!            dcbus, which is a value used to compensate the proportional gains of the current 
//!            controllers. The following example shows how to use this function to calculate a DC bus value
//!            in kilo volts:
//! \code
//! #define USERFLOAT_T_FULL_SCALE_VOLTAGE_V (300.0)
//!
//! float_t Vbus_V = EST_getDcBus_V(handle);
//! float_t Vbus_kV = Vbus_V / 1000.0;
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The DC bus value, V
extern float_t EST_getDcBus_V(EST_Handle handle);


//! \brief     Gets the error code from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The error code
extern EST_ErrorCode_e EST_getErrorCode(EST_Handle handle);


//! \brief     Gets the electrical frequency of the motor in Hertz (Hz).
//! \details   This frequency, in Hz, is the frequency of currents and voltages going into the motor. 
//!            In order to get the speed of the motor, it is better to use EST_getFm_Hz().
//! \param[in] handle  The estimator (EST) handle
//! \return    The electrical frequency, Hz
extern float_t EST_getFe_Hz(EST_Handle handle);


//! \brief     Gets the electrical frequency of the motor in rad/sec.
//! \details   This frequency, in Hz, is the frequency of currents and voltages going into the motor. 
//!            In order to get the speed of the motor, it is better to use EST_getFm_rps().
//! \param[in] handle  The estimator (EST) handle
//! \return    The electrical frequency, rad/sec
extern float_t EST_getFe_rps(EST_Handle handle);


//! \brief     Gets the bypass lock rotor flag value
//! \param[in] handle  The estimator (EST) handle
//! \return    The bypass lock rotor flag value
extern bool EST_getFlag_bypassLockRotor(EST_Handle handle);


//! \brief     Sets the bypass lock rotor flag value in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired state
extern void EST_setFlag_bypassLockRotor(EST_Handle handle,const bool state);


//! \brief     Gets the enable estimator flag value
//! \param[in] handle  The estimator (EST) handle
//! \return    The enable estimator flag value
extern bool EST_getFlag_enable(EST_Handle handle);


//! \brief     Sets the enable estimator flag value in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired state
extern void EST_setFlag_enable(EST_Handle handle,const bool state);


//! \brief     Gets the enable force angle flag value from the estimator.
//! \param[in] handle  The estimator (EST) handle
//! \return    The value of the flag, in boolean type, bool
//! \retval    
//!            true  Forced angle is enabled, and the estimated angle will be bypassed if the flux
//!                  frequency falls below a threashold defined by:
//! \code 
//! #define USER_ZEROSPEEDLIMIT (0.001)
//! \endcode
//!                  A typical value of this frequency is 0.001 of the full scale frequency defined by:
//! \code 
//! #define USER_IQ_FULL_SCALE_FREQ_Hz (500.0) 
//! \endcode
//!                  The forced angle algorithm, when active, that is, when the rotor flux electrical
//!                  frequency falls below the threashold, will be forcing a rotating angle at a
//!                  frequency set by the following define:
//! \code 
//! #define USER_FORCE_ANGLE_FREQ_Hz  (1.0) 
//! \endcode
//! \retval
//!            false Force angle is disabled, and the estimator will never be bypassed by any forced
//!                  angle algorithm.
extern bool EST_getFlag_enableForceAngle(EST_Handle handle);


//! \brief     Gets the value of the flag which enables online stator resistance (Rs) estimation
//! \param[in] handle  The estimator (EST) handle
//! \return    The enable online Rs flag value
//! \retval
//!            true   Rs online recalibration algorithm is enabled. The estimator will run a set of
//!                   functions related to rs online which recalculates the stator resistance while the
//!                   motor is rotating. This algorithm is useful when motor heats up, and hence stator
//!                   resistance increases.
//! \retval
//!            false  Rs online recalibration algorithm is disabled, and no updates to Rs will be made
//!                   even if the motor heats up. Low speed performace, and startup performance with
//!                   full torque might be affected if stator resistance changes due to motor heating
//!                   up. The stator resistance will be fixed, and equal to the value returned by:
//!                   EST_getRs_Ohm().
extern bool EST_getFlag_enableRsOnLine(EST_Handle handle);


//! \brief     Gets the enable stator resistance re-calibrate flag value from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The value of the enable stator resistance re-calibrate flag
//! \retval
//!            true   Rs recalibration is enabled. The estimator will inject a DC current to the D-axis
//!                   to recalibrate the stator resistance before the motor rotates. It is required that
//!                   the motor is at standstill to perform Rs recalibration. If online recalibration
//!                   of the stator resistance is needed, refer to EST_getFlag_enableRsOnLine() and
//!                   EST_setFlag_enableRsOnLine() functions.
//! \retval
//!            false  Rs recalibration is disabled. The estimator will start the motor with the resistance
//!                   value that was used before the motor was stopped, or what is returned by function:
//!                   EST_getRs_Ohm().
extern bool EST_getFlag_enableRsRecalc(EST_Handle handle);


//! \brief     Gets the value of the flag which denotes when the estimation is complete
//! \details   This flag is set to true every time the EST_run() function is run. 
//!            This flag can be reset to false by using the following example:
//! \code
//! bool estComplete_Flag = EST_getFlag_estComplete(handle);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The estimation complete flag value
//! \retval
//!            true   The estimator has been run at least once since last time
//!                   EST_setFlag_estComplete(handle, false) was called.
//! \retval
//!            false  The estimator has not been run since last time EST_setFlag_estComplete(handle, false)
//!                   was called.
extern bool EST_getFlag_estComplete(EST_Handle handle);


//! \brief     Gets the value of the flag which enables the updating of the stator resistance (Rs) value
//! \details   When the online resistance estimator is enabled, the update flag allows the online resistance
//!            to be copied to the resistance used by the estimator model. If the update flag is not set to true,
//!            the online resistance estimation will not be used by the estimator model, and if the resistance
//!            changes too much due to temperature increase, the model may not work as expected.
//! \code
//! bool update_Flag = EST_getFlag_updateRs(handle);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The update Rs flag value
//! \retval
//!            true   The stator resistance estimated by the Rs OnLine module will be copied to the'
//!                   the stator resistance used by the module, so of the motor's temperature changes,
//!                   the estimated angle will be calculated based on the most up to date stator
//!                   resistance
//! \retval
//!            false  The stator resistance estimated by the Rs OnLine module may or may not be updated
//!                   depending on the enable flag, but will not be used in the motor's model used to generate
//!                   the estimated speed and angle.
extern bool EST_getFlag_updateRs(EST_Handle handle);


//! \brief     Gets the flux value in Weber (Wb).
//! \details   The estimator continuously calculates the flux linkage between the rotor and stator, which is the
//!            portion of the flux that produces torque. This function returns the flux linkage, ignoring the
//!            number of turns, between the rotor and stator coils, in Weber, or Wb, or Volts * Seconds / rad (V.sec/rad).
//!            This functions returns a precise value only after the motor has been identified, which can be 
//!            checked by the following code example:
//! \code
//! if(EST_isMotorIdentified(handle))
//!   {
//!     // once the motor has been identified, get the flux
//!     float_t Flux_Wb = EST_getFlux_Wb(handle);
//!   }
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The flux value, Weber or V*sec/rad, in floating point
extern float_t EST_getFlux_Wb(EST_Handle handle);


//! \brief     Gets the mechanical frequency of the motor in Hertz (Hz).
//! \details   This frequency, in Hz, is the mechanical frequency of the motor. If the motor is a permanent
//!            magnet motor, the mechanical frequency will be equal to the electrical frequency, since it is
//!            a synchronous motor. In the case of AC induction motors, the mechanical frequency will be equal
//!            to the electrical frequency minus the slip frequency. The following code example shows how to
//!            use this function to calculate revolutions per minute (RPM) in floating point:
//! \code
//! #define USER_MOTOR_NUM_POLE_PAIRS  (2)
//!
//! float_t mechFreq_Hz = EST_getFm_Hz(handle);
//! float_t hz_to_rpm_sf = 60.0/(float_t)USER_MOTOR_NUM_POLE_PAIRS;
//! float_t speed_rpm = mechFreq_Hz * hz_to_rpm_sf;
//! \endcode           
//! \param[in] handle  The estimator (EST) handle
//! \return    The mechanical frequency, Hz
extern float_t EST_getFm_Hz(EST_Handle handle);


//! \brief     Gets the mechanical frequency of the motor in rad/sec.
//! \details   This frequency, in rad/sec, is the mechanical frequency of the motor. If the motor is a permanent
//!            magnet motor, the mechanical frequency will be equal to the electrical frequency, since it is
//!            a synchronous motor. In the case of AC induction motors, the mechanical frequency will be equal
//!            to the electrical frequency minus the slip frequency. The following code example shows how to
//!            use this function to calculate revolutions per minute (RPM) in floating point:
//! \code
//! #define USER_MOTOR_NUM_POLE_PAIRS  (2)
//!
//! float_t mechFreq_rps = EST_getFm_rps(handle);
//! float_t rps_to_rpm_sf = (float_t)60.0/(MATH_TWO_PI * (float_t)USER_MOTOR_NUM_POLE_PAIRS);
//! float_t speed_rpm = mechFreq_rps * rps_to_rpm_sf;
//! \endcode           
//! \param[in] handle  The estimator (EST) handle
//! \return    The mechanical frequency, rad/sec
extern float_t EST_getFm_rps(EST_Handle handle);


//! \brief     Gets the force angle delta value from the estimator in per unit radians (rad).
//! \details   This function returns a valid value only after initializing the controller object
//!            by calling CTRL_setParams() function. The force angle delta represents the increments
//!            to be added to or subtracted from the forced angle. The higher this value is, the higher
//!            frequency will be generated when the angle is forced (estimated angle is bypassed when 
//!            in forced angle mode). By default the forced angle frequency is set in user.h. 
//!            The following example shows how to convert delta in radians to kilo Hertz (kHz).
//! \code

//NEED TO UPDATE
//
//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_EST_TICK  (1)
//! #define USER_PWM_FREQ_kHz         (15.0)
//! #define USER_ISR_FREQ_Hz          (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz         (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_EST_FREQ_Hz          (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_EST_TICK)
//!
//! float_t delta_pu_to_kHz_sf = FLOAT_T((float_t)USER_EST_FREQ_Hz/1000.0);
//! float_t forceAngleDelta_pu = EST_getForceAngleDelta_pu(handle);
//! float_t ForceAngleFreq_kHz = FLOAT_Tmpy(Force_Angle_Delta_pu, delta_pu_to_kHz_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The force angle delta, pu
extern float_t EST_getForceAngleDelta_pu(EST_Handle handle);


//! \brief     Gets the status of the force angle operation in the estimator
//! \details   The status can only change to active when forced angle mode has been enabled by
//!            calling the following function:
//! \code
//! EST_setFlag_enableForceAngle(handle, true);
//! \endcode
//! \details   Forced angle mode will be active when the electrical frequency of the motor falls below the
//!            defined threshold in user.h:
//! \code
//! #define USER_ZEROSPEEDLIMIT (0.001)
//! \endcode
//! details    A manual check of forced angle status can be done using the following code example:
//! \code
//! float_t fe_pu = EST_getFe_pu(handle);
//! bool is_forced_angle_active;
//! if(FLOAT_Tabs(fe_pu) < FLOAT_T(USER_ZEROSPEEDLIMIT))
//!   {
//!     is_forced_angle_active = true;
//!   }
//! else
//!   {
//!     is_forced_angle_active = false;
//!   }
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting whether the angle has been forced (true) or not (false)
//! \retval
//!            true   The last iteration of the estimator used a forced angle to run the park and inverse park
//!                   transforms. The estimator was also run in parallel to the forced angle, but the estimator
//!                   output was not used.
//! \retval
//!            false  Forced angle mode is either disabled, or the electrical frequency did not fall below
//!                   the predetermined threshold. The estimator output was used to run the park and
//!                   inverse park transforms.
extern bool EST_getForceAngleStatus(EST_Handle handle);


//! \brief     Gets the value used to set the pole location in the low-pass filter of the frequency estimator
//!            in radians per second (rps).
//! \param[in] handle  The estimator (EST) handle
//! \return    The value used to set the filter pole location, rad
extern float_t EST_getFreqBeta_lp(EST_Handle handle);


//! \brief     Gets the slip frequency of the motor in Hertz (Hz).
//! \details   When running a permanent magnet motor, the slip frequency returned by this function will be zero.
//!            If an induction motor is used, this function will return the slip frequency. This frequency, in Hz,
//!            will be the difference between the electrical frequency and the mechanical frequency.\n
//!            \f[F_{slip}=F_{electrical}-F_{mechanical}\f]
//!
//! \param[in] handle  The estimator (EST) handle
//! \return    The slip frequency, Hz
extern float_t EST_getFslip_Hz(EST_Handle handle);


//! \brief     Gets the slip frequency of the motor in rad/sec.
//! \details   When running a permanent magnet motor, the slip frequency returned by this function will be zero.
//!            If an induction motor is used, this function will return the slip frequency. This frequency, in rad/sec,
//!            will be the difference between the electrical frequency and the mechanical frequency.\n
//!            \f[F_{slip}=F_{electrical}-F_{mechanical}\f]
//!
//! \param[in] handle  The estimator (EST) handle
//! \return    The slip frequency, rad/sec
extern float_t EST_getFslip_rps(EST_Handle handle);


//! \brief     Gets the Iab current vector in Ampere (A).
//! \param[in] handle  The estimator (EST) handle
//! \param[in] pIab_A  The pointer to memory for the Iab vector, A
extern void EST_getIab_A(EST_Handle handle,MATH_vec2 *pIab_A);


//! \brief     Gets the beta value for the Iab low pass filter
//! \param[in] handle  The estimator (EST) handle
//! \return    The beta value, rad
extern float_t EST_getIab_beta_lp(EST_Handle handle);


//! \brief     Gets the Idq current vector in Ampere (A).
//! \param[in] handle  The estimator (EST) handle
//! \param[in] pIdq_A  The pointer to memory for the Idq vector, A
extern void EST_getIdq_A(EST_Handle handle,MATH_vec2 *pIdq_A);


//! \brief     Gets the Id rated current value from the estimator in Ampere (A).
//! \param[in] handle  The estimator (EST) handle
//! \return    The Id rated current value, A
extern float_t EST_getIdRated_A(EST_Handle handle);


//! \brief     Gets the Id current value used for inductance estimation of induction motors
//!            in Ampere (A).
//! \param[in] handle  The estimator (EST) handle
//! \return    The Id rated value, A
extern float_t EST_getIdRated_indEst_A(EST_Handle handle);


//! \brief     Gets the Id current value used for flux estimation of induction motors in Ampere (A).
//! \param[in] handle  The estimator (EST) handle
//! \return    The Id rated value, A
extern float_t EST_getIdRated_ratedFlux_A(EST_Handle handle);


//! \brief     Gets the intermediate value from the Id trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The intermediate value, A
extern float_t EST_getIntValue_Id(EST_Handle handle);


//! \brief     Gets the intermediate value from the speed trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The intermediate value, Hz
extern float_t EST_getIntValue_spd(EST_Handle handle);


//! \brief     Gets the magnetizing inductance value in Henry (H).
//! \param[in] handle  The estimator (EST) handle
//! \return    The magnetizing inductance value, H
extern float_t EST_getLmag_H(EST_Handle handle);


//! \brief     Gets the direct stator inductance value in Henry (H).
//! \param[in] handle  The estimator (EST) handle
//! \return    The direct stator inductance value, H
extern float_t EST_getLs_d_H(EST_Handle handle);


//! \brief     Gets the direct/quadrature stator inductance values from the estimator in Henry (H).
//! \details   Both direct and quadrature stator inductances can be read from the estimator by using
//!            this function call and passing a pointer to a structure where these two values will be
//!            stored.
//! \param[in] handle     The estimator (EST) handle
//! \param[out] pLs_dq_H  The pointer to the vector of direct/quadrature stator inductance values, H
extern void EST_getLs_dq_H(EST_Handle handle,MATH_vec2 *pLs_dq_H);


//! \brief     Gets the stator inductance value in the quadrature coordinate direction in Henry (H).
//! \param[in] handle  The estimator (EST) handle
//! \return    The stator inductance value, H
extern float_t EST_getLs_q_H(EST_Handle handle);


//! \brief     Gets the maximum estimation acceleration value used in the estimator in radians per second^2 (rps2).
//! \details   The maximum acceleration is a setting of the trajectory module, which sets the speed reference.
//!            The acceleration returned by this function call is used during the motor identification process.
//!            This value represents how the speed reference is increased or decreased from an initial value
//!            to a target value. The following example shows how convert the returned value of this function
//!            to kilo radians per second^2 (krps^2), kilo Hertz per Second (kHz/s) and kilo RPM per second (kRPM/s):
//! \code

// NEED TO UPDATE

//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_TRAJ_TICK (10)
//! #define USER_PWM_FREQ_kHz                 (15.0)
//! #define USER_ISR_FREQ_Hz                  (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz                 (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_TRAJ_FREQ_Hz                 (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_TRAJ_TICK)
//! #define USERFLOAT_T_FULL_SCALE_FREQ_Hz        (500.0)
//! #define MATH_PI                           (3.1415926535897932384626433832795)
//! #define USER_MOTOR_NUM_POLE_PAIRS         (4)
//!
//! float_t pu_to_krps2_sf = FLOAT_T((float_t)USER_TRAJ_FREQ_Hz * USERFLOAT_T_FULL_SCALE_FREQ_Hz / 1000.0);
//! float_t krps2_to_khzps_sf = FLOAT_T(1.0 / (2.0 * MATH_PI));
//! float_t khzps_to_krpmps_sf = FLOAT_T(60.0 / (float_t)USER_MOTOR_NUM_POLE_PAIRS);
//! 
//! float_t est_Accel_pu = EST_getMaxAccel_est_pu(handle);
//! float_t est_Accel_kilo_rads_per_sec_2 = FLOAT_Tmpy(est_Accel_pu, pu_to_krps2_sf);
//! float_t est_Accel_kilo_hz_per_sec = FLOAT_Tmpy(est_Accel_kilo_rads_per_sec_2, krps2_to_khzps_sf);
//! float_t est_Accel_kilo_rpm_per_sec = FLOAT_Tmpy(est_Accel_kilo_hz_per_sec, khzps_to_krpmps_sf);
//! \endcode
//! \details   The default value is set by a user's defined value in user.h, and the default value in per
//!            units is calculated internally as follows:
//! \code
//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_TRAJ_TICK (10)
//! #define USER_PWM_FREQ_kHz                 (15.0)
//! #define USER_ISR_FREQ_Hz                  (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz                 (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_TRAJ_FREQ_Hz                 (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_TRAJ_TICK)
//! #define USERFLOAT_T_FULL_SCALE_FREQ_Hz        (500.0)
//! #define USER_MAX_ACCEL_EST_rps2           (2.0)
//!
//! float_t rps2_to_pu_sf = FLOAT_T(1.0 / ((float_t)USER_TRAJ_FREQ_Hz * USERFLOAT_T_FULL_SCALE_FREQ_Hz));
//! 
//! float_t est_Accel_rads_per_sec_2 = FLOAT_T(USER_MAX_ACCEL_EST_rps2);
//! float_t est_Accel_pu = FLOAT_Tmpy(est_Accel_rads_per_sec_2, rps2_to_pu_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum speed delta value during estimation, Hz
extern float_t EST_getMaxSpeedDelta_Hz(EST_Handle handle);


//! \brief     Gets the maximum current slope value used in the estimator in radians per second (rps).
//! \details   Gets the slope of Id reference. The following example shows how to convert the returned
//!            value into kilo Amperes per second (kA/s):

// NEED TO UPDATE

//! \code
//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_TRAJ_TICK (10)
//! #define USER_PWM_FREQ_kHz                 (15.0)
//! #define USER_ISR_FREQ_Hz                  (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz                 (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_TRAJ_FREQ_Hz                 (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_TRAJ_TICK)
//! #define USERFLOAT_T_FULL_SCALE_CURRENT_A      (10.0)
//!
//! float_t pu_to_kA_per_sec_sf = FLOAT_T((float_t)USER_TRAJ_FREQ_Hz * USERFLOAT_T_FULL_SCALE_CURRENT_A / 1000.0);
//!
//! float_t currentSlope_pu = EST_getMaxCurrentDelta_pu(handle);
//! float_t currentSlope_kAps = FLOAT_Tmpy(currentSlope_pu, pu_to_kA_per_sec_sf);
//! \endcode
//! \details   The default value is set by a user's defined value in user.h, and the default value in per
//!            units is calculated internally as follows:
//! \code
//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_TRAJ_TICK (10)
//! #define USER_PWM_FREQ_kHz                 (15.0)
//! #define USER_ISR_FREQ_Hz                  (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz                 (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_TRAJ_FREQ_Hz                 (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_TRAJ_TICK)
//! #define USERFLOAT_T_FULL_SCALE_CURRENT_A      (10.0)
//! #define USER_MOTOR_RES_EST_CURRENT        (1.0)
//!
//! float_t A_per_sec_to_pu_sf = FLOAT_T(1.0 / ((float_t)USER_TRAJ_FREQ_Hz * USERFLOAT_T_FULL_SCALE_CURRENT_A));
//!
//! float_t currentSlope_Aps = FLOAT_T(USER_MOTOR_RES_EST_CURRENT);
//! float_t currentSlope_pu = FLOAT_Tmpy(currentSlope_Aps, A_per_sec_to_pu_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum current delta value, A
extern float_t EST_getMaxCurrentDelta_A(EST_Handle handle);


//! \brief     Gets the maximum EPL (Efficient Partial Load) current slope value used in the estimator
//!            in radians per second (rps).
//! \details   Gets the slope of Id reference change when efficient partial load is enabled. This mode only
//!            applies to induction motors. The following example shows how to convert the returned value into 
//!            kilo Amperes per second (kA/s):
//! \code
//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_TRAJ_TICK (10)
//! #define USER_PWM_FREQ_kHz                 (15.0)
//! #define USER_ISR_FREQ_Hz                  (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz                 (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_TRAJ_FREQ_Hz                 (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_TRAJ_TICK)
//! #define USERFLOAT_T_FULL_SCALE_CURRENT_A      (10.0)
//!
//! float_t pu_to_kA_per_sec_sf = FLOAT_T((float_t)USER_TRAJ_FREQ_Hz * USERFLOAT_T_FULL_SCALE_CURRENT_A / 1000.0);
//!
//! float_t currentSlope_epl_pu = EST_getMaxCurrentDelta_epl_pu(handle);
//! float_t currentSlope_epl_kAps = FLOAT_Tmpy(currentSlope_epl_pu, pu_to_kA_per_sec_sf);
//! \endcode
//! \details   The default value is set by a user's defined value in user.h, and the default value in per
//!            units is calculated internally as follows:
//! \code
//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_TRAJ_TICK (10)
//! #define USER_PWM_FREQ_kHz                 (15.0)
//! #define USER_ISR_FREQ_Hz                  (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz                 (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_TRAJ_FREQ_Hz                 (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_TRAJ_TICK)
//! #define USERFLOAT_T_FULL_SCALE_CURRENT_A      (10.0)
//! #define USER_MOTOR_RES_EST_CURRENT        (1.0)
//!
//! float_t A_per_sec_to_pu_sf = FLOAT_T(1.0 / ((float_t)USER_TRAJ_FREQ_Hz * USERFLOAT_T_FULL_SCALE_CURRENT_A));
//!
//! float_t currentSlope_epl_Aps = FLOAT_T(0.3 * USER_MOTOR_RES_EST_CURRENT);
//! float_t currentSlope_epl_pu = FLOAT_Tmpy(currentSlope_epl_Aps, A_per_sec_to_pu_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum EPL current delta value, A
extern float_t EST_getMaxCurrentDelta_epl_A(EST_Handle handle);


//! \brief     Gets the maximum delta value from the Id trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum delta value, A
extern float_t EST_getMaxDelta_Id(EST_Handle handle);


//! \brief     Gets the maximum delta value from the speed trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum delta value, Hz
extern float_t EST_getMaxDelta_spd(EST_Handle handle);


//! \brief     Gets the maximum acceleration value used in the estimator in radians per second^2 (rps2).
//! \details   The maximum acceleration is a setting of the trajectory module, which sets the speed reference.
//!            The acceleration returned by this function call is used after the motor has been identified.
//!            This value represents how the speed reference is increased or decreased from an initial value
//!            to a target value. The following example shows how convert the returned value of this function
//!            to kilo radians per second^2 (krps^2), kilo Hertz per Second (kHz/s) and kilo RPM per second (kRPM/s):
//! \code
//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_TRAJ_TICK (10)
//! #define USER_PWM_FREQ_kHz                 (15.0)
//! #define USER_ISR_FREQ_Hz                  (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz                 (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_TRAJ_FREQ_Hz                 (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_TRAJ_TICK)
//! #define USERFLOAT_T_FULL_SCALE_FREQ_Hz        (500.0)
//! #define MATH_PI                           (3.1415926535897932384626433832795)
//! #define USER_MOTOR_NUM_POLE_PAIRS         (4)
//!
//! float_t pu_to_krps2_sf = FLOAT_T((float_t)USER_TRAJ_FREQ_Hz * USERFLOAT_T_FULL_SCALE_FREQ_Hz / 1000.0);
//! float_t krps2_to_khzps_sf = FLOAT_T(1.0 / (2.0 * MATH_PI));
//! float_t khzps_to_krpmps_sf = FLOAT_T(60.0 / (float_t)USER_MOTOR_NUM_POLE_PAIRS);
//! 
//! float_t Accel_pu = EST_getMaxAccel_pu(handle);
//! float_t Accel_kilo_rads_per_sec_2 = FLOAT_Tmpy(Accel_pu, pu_to_krps2_sf);
//! float_t Accel_kilo_hz_per_sec = FLOAT_Tmpy(Accel_kilo_rads_per_sec_2, krps2_to_khzps_sf);
//! float_t Accel_kilo_rpm_per_sec = FLOAT_Tmpy(Accel_kilo_hz_per_sec, khzps_to_krpmps_sf);
//! \endcode
//! \details   The default value is set by a user's defined value in user.h, and the default value in per
//!            units is calculated internally as follows:
//! \code
//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_TRAJ_TICK (10)
//! #define USER_PWM_FREQ_kHz                 (15.0)
//! #define USER_ISR_FREQ_Hz                  (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz                 (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_TRAJ_FREQ_Hz                 (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_TRAJ_TICK)
//! #define USERFLOAT_T_FULL_SCALE_FREQ_Hz        (500.0)
//! #define USER_MAX_ACCEL_rps2               (20.0)
//!
//! float_t rps2_to_pu_sf = FLOAT_T(1.0 / ((float_t)USER_TRAJ_FREQ_Hz * USERFLOAT_T_FULL_SCALE_FREQ_Hz));
//! 
//! float_t Accel_rads_per_sec_2 = FLOAT_T(USER_MAX_ACCEL_rps2);
//! float_t Accel_pu = FLOAT_Tmpy(Accel_rads_per_sec_2, rps2_to_pu_sf);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum speed delta value, Hz
extern float_t EST_getMaxSpeedDelta_Hz(EST_Handle handle);


//! \brief     Gets the maximum value from the Id trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum value, A
extern float_t EST_getMaxValue_Id(EST_Handle handle);


//! \brief     Gets the maximum value from the speed trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The maximum value, Hz
extern float_t EST_getMaxValue_spd(EST_Handle handle);


//! \brief     Gets the minimum value from the Id trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The minimum value, A
extern float_t EST_getMinValue_Id(EST_Handle handle);


//! \brief     Gets the minimum value from the speed trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The minimum value, Hz
extern float_t EST_getMinValue_spd(EST_Handle handle);


//! \brief     Gets the number of Interrupt Service Routine (ISR) clock ticks per estimator clock tick
//! \param[in] handle  The estimator (EST) handle
//! \return    The number of Interrupt Service Routine (ISR) clock ticks per estimator clock tick
extern int_least16_t EST_getNumIsrTicksPerEstTick(EST_Handle handle);


//! \brief     Sets the number of Interrupt Service Routine (ISR) clock ticks per estimator clock tick
//! \param[in] handle                 The estimator (EST) handle
//! \return    numIsrTicksPerEstTick  The number of Interrupt Service Routine (ISR) clock ticks per estimator clock tick
extern void EST_setNumIsrTicksPerEstTick(EST_Handle handle,const int_least16_t numIsrTicksPerEstTick);


//! \brief     Gets the number of Interrupt Service Routine (ISR) clock ticks per estimator trajectory clock tick
//! \param[in] handle  The estimator (EST) handle
//! \return    The number of Interrupt Service Routine (ISR) clock ticks per estimator trajectory clock tick
extern int_least16_t EST_getNumIsrTicksPerTrajTick(EST_Handle handle);


//! \brief     Sets the number of Interrupt Service Routine (ISR) clock ticks per estimator trajectory clock tick
//! \param[in] handle                  The estimator (EST) handle
//! \return    numIsrTicksPerTrajTick  The number of Interrupt Service Routine (ISR) clock ticks per estimator trajectory clock tick
extern void EST_setNumIsrTicksPerTrajTick(EST_Handle handle,const int_least16_t numIsrTicksPerTrajTick);


//! \brief     Gets the inverse of the DC bus voltage in 1/Volt (1/V).
//! \param[in] handle  The estimator (EST) handle
//! \return    The inverse of the DC bus voltage, 1/V
extern float_t EST_getOneOverDcBus_invV(EST_Handle handle);


//! \brief     Gets the R/L value from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The R/L value, rad/sec
extern float_t EST_getRoverL_rps(EST_Handle handle);


//! \brief     Gets the direct rotor resistance value in Ohm (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \return    The direct rotor resistance value, Ohm
extern float_t EST_getRr_d_Ohm(EST_Handle handle);


//! \brief     Gets the direct/quadrature rotor resistance values from the estimator in Ohms (\f$\Omega\f$).
//! \details   Both direct and quadrature rotor resistances can be read from the estimator by using
//!            this function call and passing a pointer to a structure where these two values will be
//!            stored.
//! \param[in] handle       The estimator (EST) handle
//! \param[out] pRr_dq_Ohm  The pointer to the vector of direct/quadrature rotor resistance values, Ohm
extern void EST_getRr_dq_Ohm(EST_Handle handle,MATH_vec2 *pRr_dq_Ohm);


//! \brief     Gets the quadrature rotor resistance value in Ohms (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \return    The quadrature rotor resistance value, Ohm
extern float_t EST_getRr_q_Ohm(EST_Handle handle);


//! \brief     Gets the alpha stator resistance value in Ohms (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \return    The alpha stator resistance value, Ohm
extern float_t EST_getRs_a_Ohm(EST_Handle handle);


//! \brief     Gets the alpha/beta stator resistance values from the estimator in Ohms (\f$\Omega\f$).
//! \details   Both alpha and beta stator resistances can be read from the estimator by using
//!            this function call and passing a pointer to a structure where these two values will be
//!            stored.
//! \param[in] handle       The estimator (EST) handle
//! \param[out] pRs_ab_Ohm  The pointer to the vector of alpha/beta stator resistance values, Ohm
extern void EST_getRs_ab_Ohm(EST_Handle handle,MATH_vec2 *pRs_ab_Ohm);


//! \brief     Gets the beta stator resistance value in Ohms (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \return    The beta stator resistance value, Ohm
extern float_t EST_getRs_b_Ohm(EST_Handle handle);


//! \brief     Gets the direct stator resistance value in Ohms (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \return    The direct stator resistance value, Ohm
extern float_t EST_getRs_d_Ohm(EST_Handle handle);


//! \brief     Gets the direct/quadrature stator resistance values from the estimator in Ohms (\f$\Omega\f$).
//! \details   Both direct and quadrature stator resistances can be read from the estimator by using
//!            this function call and passing a pointer to a structure where these two values will be
//!            stored.
//! \param[in] handle       The estimator (EST) handle
//! \param[out] pRs_dq_Ohm  The pointer to the vector of direct/quadrature stator resistance values, Ohm
extern void EST_getRs_dq_Ohm(EST_Handle handle,MATH_vec2 *pRs_dq_Ohm);


//! \brief     Gets the quadrature stator resistance value in Ohms (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \return    The quadrature stator resistance value, Ohm
extern float_t EST_getRs_q_Ohm(EST_Handle handle);


//! \brief     Gets the online stator resistance filter parameters 
//! \param[in] handle      The estimator (EST) handle
//! \param[in] filterType  The filter type
//! \param[in] b0          The pointer for the numerator coefficient value for z^0
//! \param[in] b1          The pointer for the numerator coefficient value for z^(-1)
//! \param[in] b2          The pointer for the numerator coefficient value for z^(-2)
//! \param[in] a1          The pointer for the denominator coefficient value for z^(-1)
//! \param[in] a2          The pointer for the denominator coefficient value for z^(-2)
//! \param[in] x1          The pointer for the input value at time sample n=-1
//! \param[in] x2          The pointer for the input value at time sample n=-2
//! \param[in] y1          The pointer for the output value at time sample n=-1
//! \param[in] y2          The pointer for the output value at time sample n=-2
extern void EST_getRsOnLineFilterParams(EST_Handle handle,const EST_RsOnLineFilterType_e filterType,
                                        float_t *b0,float_t *b1,float_t *b2,
                                        float_t *a1,float_t *a2,
                                        float_t *x1,float_t *x2,
                                        float_t *y1,float_t *y2);


//! \brief     Gets the online stator resistance value in Ohm (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \return    The online stator resistance value, Ohm
extern float_t EST_getRsOnLine_Ohm(EST_Handle handle);


//! \brief     Gets the Id magnitude value used for online stator resistance estimation in Ampere (A).
//! \param[in] handle  The estimator (EST) handle
//! \return    The Id magnitude value, A
extern float_t EST_getRsOnLineId_mag_A(EST_Handle handle);


//! \brief     Gets the Id value used for online stator resistance estimation in Ampere (A).
//! \param[in] handle  The estimator (EST) handle
//! \return    The Id value, A
extern float_t EST_getRsOnLineId_A(EST_Handle handle);


//! \brief     Sets the Id value in the online stator resistance estimator in Ampere (A).
//! \param[in] handle  The estimator (EST) handle
//! \param[in] Id_A    The Id value, A
extern void EST_setRsOnLineId_A(EST_Handle handle,const float_t Id_A);


//! \brief     Gets the sign of the direction value in 8 bit signed integer (int_least8_t).
//! \param[in] handle  The estimator (EST) handle
//! \return    The sign of the direction value (-1 for negative, 1 for positive)
extern int_least8_t EST_getSignOfDirection(EST_Handle handle);


//! \brief     Gets the speed value in kilo-revolutions per second (krpm).
//! \param[in] handle  The estimator (EST) handle
//! \return    The speed value, krpm
extern float_t EST_getSpeed_krpm(EST_Handle handle);


//! \brief     Gets the reference speed value from the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The reference speed value, Hz
extern float_t EST_getSpeed_ref_Hz(EST_Handle handle);


//! \brief     Gets the state of the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The estimator state
extern EST_State_e EST_getState(EST_Handle handle);


//! \brief     Gets the trajectory generator state
//! \param[in] handle  The estimator (EST) handle
//! \return    The state
extern EST_Traj_State_e EST_getTrajState(EST_Handle handle);


//! \brief     Gets the target value from the Id trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The target value, A
extern float_t EST_getTargetValue_Id(EST_Handle handle);


//! \brief     Gets the target value from the speed trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    The target value, Hz
extern float_t EST_getTargetValue_spd(EST_Handle handle);


//! \brief     Increments the ISR counter in the estimator
//! \param[in] handle  The estimator (EST) handle
extern void EST_incrCounter_isr(EST_Handle handle);


//! \brief     Increments the ISR counter in the trajectory generator
//! \param[in] handle  The estimator (EST) handle
extern void EST_incrTrajCounter_isr(EST_Handle handle);


//! \brief     Initializes the estimator
//! \param[in] estNumber  The estimator number
//! \return    The estimator (EST) handle
extern EST_Handle EST_initEst(const uint_least8_t estNumber);


//! \brief     Determines if the estimator (EST) is enabled
//! \param[in] handle  The estimator (EST) handle
extern bool EST_isEnabled(EST_Handle handle);


//! \brief     Determines if there is an error in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if there is an error (true) or not (false)
extern bool EST_isError(EST_Handle handle);


//! \brief     Determines if the estimator is idle
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if the estimator is idle (true) or not (false)
extern bool EST_isIdle(EST_Handle handle);


//! \brief     Determines if the estimator is waiting for the rotor to be locked
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if the estimator is waiting for the rotor to be locked (true) or not (false)
extern bool EST_isLockRotor(EST_Handle handle);


//! \brief     Determines if the motor has been identified
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if the motor is identified (true) or not (false)
extern bool EST_isMotorIdentified(EST_Handle handle);


//! \brief     Determines if the estimator is not ready for online control
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if the estimator is not ready for online control (true) or not (false)
extern bool EST_isNotOnLine(EST_Handle handle);


//! \brief     Determines if the estimator is ready for online control
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if the estimator is ready for online control (true) or not (false)
extern bool EST_isOnLine(EST_Handle handle);


//! \brief     Determines if the trajectory generator is enabled
//! \param[in] handle  The estimator (EST) handle
extern bool EST_isTrajEnabled(EST_Handle handle);


//! \brief     Determines if there is an error in the trajectory generator
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if there is an error (true) or not (false)
extern bool EST_isTrajError(EST_Handle handle);


//! \brief     Resets the isr counter
//! \param[in] handle  The estimator (EST) handle
extern void EST_resetCounter_isr(EST_Handle handle);


//! \brief     Resets the state counter
//! \param[in] handle  The estimator (EST) handle
extern void EST_resetCounter_state(EST_Handle handle);


//! \brief     Resets the trajectory ISR counter
//! \param[in] handle  The estimator (EST) handle
extern void EST_resetTrajCounter_isr(EST_Handle handle);


//! \brief     Runs the estimator
//! \param[in] handle       The estimator (EST) handle
//! \param[in] pInputData   The pointer to the input data
//! \param[in] pOutputData  The pointer to the output data
extern void EST_run(EST_Handle handle,
                    const EST_InputData_t *pInputData,
                    EST_OutputData_t *pOutData);


//! \brief     Runs the trajectory generator
//! \param[in] handle  The estimator (EST) handle
extern void EST_runTraj(EST_Handle handle);


//! \brief     Sets the angle value at t = n in the estimator in per unit radians (rad).
//! \details   This function overwrites the estimated angle with a user's provided angle. 
//!            The set value should be between -2*pi and 2*pi
//!            The following example shows how to overwrite the estimated angle:
//! \code
//! float_t Overwrite_Flux_Angle_rad = MATH_TWO_PI * 0.5;
//! EST_setAngle_rad(handle,Overwrite_Flux_Angle_rad);
//! \endcode
//! \details   This function is not recommended for general use, since this will automatically generate
//!            an axis misalignment between the rotor flux axis and the control signals driving the motor.
//!            The use of this function is recommended for advanced users interested in doing open loop 
//!            startup algorithms that need to bypass the estimator.
//! \param[in] handle     The estimator (EST) handle
//! \param[in] angle_rad  The angle value at t = n, rad
extern void EST_setAngle_rad(EST_Handle handle,const float_t angle_rad);


//! \brief     Sets the angle value at t = n+1 in the estimator in per unit radians (rad).
//! \details   This function overwrites the estimated angle with a user's provided angle. 
//!            The set value should be between -2*pi and 2*pi
//!            The following example shows how to overwrite the estimated angle:
//! \code
//! float_t Overwrite_Flux_Angle_np1_rad = MATH_TWO_PI * 0.5;
//! EST_setAngle_np1_rad(handle,Overwrite_Flux_Angle_np1_rad);
//! \endcode
//! \details   This function is not recommended for general use, since this will automatically generate
//!            an axis misalignment between the rotor flux axis and the control signals driving the motor.
//!            The use of this function is recommended for advanced users interested in doing open loop 
//!            startup algorithms that need to bypass the estimator.
//! \param[in] handle         The estimator (EST) handle
//! \param[in] angle_np1_rad  The angle value at t = n+1, rad
extern void EST_setAngle_np1_rad(EST_Handle handle,const float_t angle_np1_rad);


//! \brief     Sets the DC bus voltage in the estimator in Volt (V).
//! \param[in] handle   The estimator (EST) handle
//! \param[in] dcBus_V  The DC bus voltage, V
extern void EST_setDcBus_V(EST_Handle handle,const float_t dcBus_V);


//! \brief     Sets the electrical frequency in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] fe_Hz   The electrical frequency value, Hz
extern void EST_setFe_Hz(EST_Handle handle,const float_t fe_Hz);


//! \brief     Sets the electrical frequency in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] fe_rps   The electrical frequency value, rad/sec
extern void EST_setFe_rps(EST_Handle handle,const float_t fe_rps);


//! \brief     Sets the asynchronous/mechanical frequency in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] fm_Hz   The mechanical frequency value, Hz
extern void EST_setFm_Hz(EST_Handle handle,const float_t fm_Hz);


//! \brief     Sets the asynchronous/mechanical frequency in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] fm_rps  The mechanical frequency value, rad/sec
extern void EST_setFm_rps(EST_Handle handle,const float_t fm_rps);


//! \brief     Sets the slip frequency in the estimator
//! \param[in] handle    The estimator (EST) handle
//! \param[in] fslip_Hz  The slip frequency value, Hz
extern void EST_setFslip_Hz(EST_Handle handle,const float_t fslip_Hz);


//! \brief     Sets the slip frequency in the estimator
//! \param[in] handle     The estimator (EST) handle
//! \param[in] fslip_rps  The slip frequency value, rad/sec
extern void EST_setFslip_rps(EST_Handle handle,const float_t fslip_rps);


//! \brief     Sets the enable flux control flag in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired flag state, on (1) or off (0)
extern void EST_setFlag_enableFluxControl(EST_Handle handle,const bool state);


//! \brief     Sets the enable force angle flag in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired flag state, on (1) or off (0)
//!            <b>true</b> Enable forced angle. The estimated angle will be bypassed if the flux
//!                    frequency falls below a threashold defined by:
//!                    \code #define USER_ZEROSPEEDLIMIT (0.001) \endcode
//!                    in user.h. A typical value of this frequency is 0.001 of the full scale frequency
//!                    defined in:
//!                    \code #define USERFLOAT_T_FULL_SCALE_FREQ_Hz (500.0) \endcode
//!                    Forced angle algorithm, when active, that is, when the rotor flux electrical
//!                    frequency falls below the threashold, will be forcing a rotating angle at a
//!                    frequency set by the following define:
//!                    \code #define USER_FORCE_ANGLE_FREQ_Hz  (1.0) \endcode
//!            <b>false</b> Disable forced angle. The estimator will never be bypassed by any forced
//!                    angle algorithm.
extern void EST_setFlag_enableForceAngle(EST_Handle handle,const bool state);


//! \brief     Sets the enable Rs online flag in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired flag state, on (1) or off (0)
//!            <b>true</b> Enable the Rs online recalibration algorithm. The estimator will run a set of
//!                    functions related to rs online which recalculates the stator resistance while the
//!                    motor is rotating. This algorithm is useful when motor heats up, and hence stator
//!                    resistance increases.
//!            <b>false</b> Disable the Rs online recalibration algorithm. No updates to Rs will be made
//!                    even if the motor heats up. Low speed performace, and startup performance with
//!                    full torque might be affected if stator resistance changes due to motor heating up.
//!                    The stator resistance will be fixed, and equal to the value returned by EST_getRs_Ohm().
extern void EST_setFlag_enableRsOnLine(EST_Handle handle,const bool state);


//! \brief     Sets the enable stator resistance (Rs) re-calculation flag in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired flag state, on (1) or off (0)
//!            <b>true</b> Enable Rs recalibration. The estimator will inject a DC current to the D-axis
//!                    to recalibrate the stator resistance before the motor rotates. It is required that
//!                    the motor is at standstill to perform Rs recalibration. If online recalibration
//!                    of the stator resistance is needed, refer to EST_getFlag_enableRsOnLine() and
//!                    EST_setFlag_enableRsOnLine() functions.
//!            <b>false</b> Disable Rs recalibration. The estimator will start the motor with the resistance
//!                    value that was used before the motor was stopped, or what is returned by function:
//!                    EST_getRs_Ohm().
extern void EST_setFlag_enableRsRecalc(EST_Handle handle,const bool state);


//! \brief     Sets the estimation complete flag in the estimator
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired flag state, true (1) or false (0)
extern void EST_setFlag_estComplete(EST_Handle handle,const bool state);


//! \brief     Sets the update stator resistance (Rs) flag in the estimator
//! \details   When the online resistance estimator is enabled, the update flag allows the online resistance
//!            to be copied to the resistance used by the estimator model. If the update flag is not set to true,
//!            the online resistance estimation will not be used by the estimator model, and if the resistance
//!            changes too much due to temperature increase, the model may not work as expected.
//! \code
//! EST_setFlag_updateRs(handle, true);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \param[in] state   The desired flag state
//!            <b>true</b> The stator resistance estimated by the Rs OnLine module will be copied to the'
//!                    the stator resistance used by the module, so of the motor's temperature changes,
//!                    the estimated angle will be calculated based on the most up to date stator
//!                    resistance
//!            <b>false</b> The stator resistance estimated by the Rs OnLine module may or may not be updated
//!                    depending on the enable flag, but will not be used in the motor's model used to generate
//!                    the estimated speed and angle.
extern void EST_setFlag_updateRs(EST_Handle handle,const bool state);


//! \brief     Sets the force angle delta value in the estimator in per unit radians (rad).
//! \details   This function sets a forced angle delta, which represents the increments
//!            to be added to or subtracted from the forced angle. The higher this value is, the higher
//!            frequency will be generated when the angle is forced (estimated angle is bypassed when 
//!            in forced angle mode). By default the forced angle frequency is set in user.h. 
//!            The following example shows how to set a forced angle frequency from Hertz (Hz) to per unit:
//! \code

// NEED TO UPDATE

//! #define USER_NUM_ISR_TICKS_PER_CTRL_TICK  (1)
//! #define USER_NUM_CTRL_TICKS_PER_EST_TICK  (1)
//! #define USER_PWM_FREQ_kHz         (15.0)
//! #define USER_ISR_FREQ_Hz          (USER_PWM_FREQ_kHz * 1000.0)
//! #define USER_CTRL_FREQ_Hz         (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
//! #define USER_EST_FREQ_Hz          (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_EST_TICK)
//! #define USER_FORCE_ANGLE_FREQ_Hz  (1.0)
//!
//! float_t delta_hz_to_pu_sf = FLOAT_T(1.0/(float_t)USER_EST_FREQ_Hz);
//! float_t Force_Angle_Freq_Hz = FLOAT_T(USER_FORCE_ANGLE_FREQ_Hz);
//! float_t Force_Angle_Delta_pu = FLOAT_Tmpy(Force_Angle_Freq_Hz, delta_hz_to_pu_sf);
//!
//! EST_setForceAngleDelta_pu(handle, Force_Angle_Delta_pu);
//! \endcode
//! \param[in] handle         The estimator (EST) handle
//! \param[in] angleDelta_pu  The force angle delta value, pu
extern void EST_setForceAngleDelta_pu(EST_Handle handle,const float_t angleDelta_pu);


//! \brief     Sets the value used to set the low pass filter pole location in the frequency estimator
//!            in radians (rad).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] beta_rad  The value used to set the filter pole location, rad
extern void EST_setFreqBeta_lp(EST_Handle handle,const float_t beta_rad);


//! \brief     Sets the estimator to idle
//! \param[in] handle  The estimator (EST) handle
extern void EST_setIdle(EST_Handle handle);


//! \brief     Sets the direct current (Id) reference value in the estimator in Ampere (A).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] Id_ref_A  The Id reference value, A
extern void EST_setId_ref_A(EST_Handle handle,const float_t Id_ref_A);


//! \brief     Sets the direct/quadrature current (Idq) reference value in the estimator in Ampere (A).
//! \param[in] handle      The estimator (EST) handle
//! \param[in] pIdq_ref_A  The pointer to the Idq reference values, A
extern void EST_setIdq_ref_A(EST_Handle handle,const MATH_vec2 *pIdq_ref_A);


//! \brief     Sets the Id rated current value in the estimator in Ampere (A).
//! \param[in] handle      The estimator (EST) handle
//! \param[in] IdRated_A  The Id rated current value, A
extern void EST_setIdRated_A(EST_Handle handle,const float_t IdRated_A);


//! \brief     Sets the intermediate value in the Id trajectory generator
//! \param[in] handle      The estimator (EST) handle
//! \param[in] intValue_A  The intermediate value, A
extern void EST_setIntValue_Id(EST_Handle handle,const float_t intValue_A);


//! \brief     Sets the intermediate value in the speed trajectory generator
//! \param[in] handle       The estimator (EST) handle
//! \param[in] intValue_Hz  The intermediate value, Hz
extern void EST_setIntValue_spd(EST_Handle handle,const float_t intValue_Hz);


//! \brief     Sets the quadrature current (Iq) reference value in the estimator in Ampere (A).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] Iq_ref_A  The Iq reference value, A
extern void EST_setIq_ref_A(EST_Handle handle,const float_t Iq_ref_A);


//! \brief     Sets the direct stator inductance value in the estimator in Henry (H).
//! \details   The internal direct inductance (Ls_d) used by the estimator can be changed in real time 
//!            by calling this function. An example showing how this is done is shown here:
//! \code
//! #define USER_MOTOR_Ls_d  (0.012)
//!
//! float_t fullScaleInductance = EST_getFullScaleInductance(handle);
//! float_t Ls_coarse_max = FLOAT_T30toF(EST_getLs_coarse_max_pu(handle));
//! int_least8_t lShift = (int_least8_t)ceil(log(USER_MOTOR_Ls_d/(Ls_coarse_max*fullScaleInductance))/log(2.0));
//! uint_least8_t Ls_qFmt = 30 - lShift;
//! float_t L_max = fullScaleInductance * pow(2.0,(float_t)lShift);
//! float_t Ls_d_pu = FLOAT_T30(USER_MOTOR_Ls_d / L_max);
//!
//! EST_setLs_d_pu(handle, Ls_d_pu);
//! EST_setLs_qFmt(handle, Ls_qFmt);
//! \endcode
//! \param[in] handle   The estimator (EST) handle
//! \param[in] Ls_d_H  The direct stator inductance value, Henry
extern void EST_setLs_d_H(EST_Handle handle,const float_t Ls_d_H);


//! \brief     Gets the delta stator inductance value during coarse estimation
//! \param[in] handle  The estimator (EST) handle
//! \return    The delta stator inductance value for coarse estimation, Henry
extern float_t EST_getLs_coarse_delta_H(EST_Handle handle);


//! \brief     Sets the delta stator inductance value during coarse estimation
//! \param[in] handle      The estimator (EST) handle
//! \param[in] Ls_delta_H  The delta stator inductance value for coarse estimation, Henry
extern void EST_setLs_coarse_delta_H(EST_Handle handle,const float_t Ls_delta_H);


//! \brief     Gets the delta stator inductance value during fine estimation
//! \param[in] handle  The estimator (EST) handle
//! \return    The delta stator inductance value for fine estimation, Henry
extern float_t EST_getLs_fine_delta_H(EST_Handle handle);


//! \brief     Sets the delta stator inductance value during fine estimation
//! \param[in] handle      The estimator (EST) handle
//! \param[in] Ls_delta_H  The delta stator inductance value for fine estimation, Henry
extern void EST_setLs_fine_delta_H(EST_Handle handle,const float_t Ls_delta_H);


//! \brief     Gets the coarse delta stator resistance value from the stator resistance estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The coarse delta stator resistance value, Ohm
extern float_t EST_getRs_coarse_delta_Ohm(EST_Handle handle);


//! \brief     Sets the coarse delta stator resistance value
//! \param[in] handle               The estimator (EST) handle
//! \param[in] Rs_coarse delta_Ohm  The coarse delta stator resistance value, Ohm
extern void EST_setRs_coarse_delta_Ohm(EST_Handle handle,const float_t Rs_coarse_delta_Ohm);


//! \brief     Gets the fine delta stator resistance value from the stator resistance estimator
//! \param[in] handle  The estimator (EST) handle
//! \return    The fine delta stator resistance value, Ohm
extern float_t EST_getRs_fine_delta_Ohm(EST_Handle handle);


//! \brief     Sets the fine delta stator resistance value
//! \param[in] handle             The estimator (EST) handle
//! \param[in] Rs_fine_delta_Ohm  The fine delta stator resistance value, Ohm
extern void EST_setRs_fine_delta_Ohm(EST_Handle handle,const float_t Rs_fine_delta_Ohm);


//! \brief     Sets the direct/quadrature stator inductance vector values in the estimator in per unit (pu), IQ30.
//! \details   The internal direct and quadrature inductances (Ls_d and Ls_q) used by the estimator can be changed
//!            in real time by calling this function. An example showing how this is done is shown here:
//! \code

// NEED TO UPDATE

//! #define USER_MOTOR_Ls_d  (0.012)
//! #define USER_MOTOR_Ls_q  (0.027)
//!
//! float_t fullScaleInductance = EST_getFullScaleInductance(handle);
//! float_t Ls_coarse_max = FLOAT_T30toF(EST_getLs_coarse_max_pu(handle));
//! int_least8_t lShift = (int_least8_t)ceil(log(USER_MOTOR_Ls_d/(Ls_coarse_max*fullScaleInductance))/log(2.0));
//! uint_least8_t Ls_qFmt = 30 - lShift;
//! float_t L_max = fullScaleInductance * pow(2.0,(float_t)lShift);
//! MATH_vec2 Ls_dq_pu;
//!
//! Ls_dq_pu.value[0] = FLOAT_T30(USER_MOTOR_Ls_d / L_max);
//! Ls_dq_pu.value[1] = FLOAT_T30(USER_MOTOR_Ls_q / L_max);
//!
//! EST_setLs_dq_pu(handle, &Ls_dq_pu);
//! EST_setLs_qFmt(handle, Ls_qFmt);
//! \endcode
//! \param[in] handle    The estimator (EST) handle
//! \param[in] pLs_dq_H  The pointer to the direct/quadrature stator inductance vector values, H
extern void EST_setLs_dq_H(EST_Handle handle,const MATH_vec2 *pLs_dq_H);


//! \brief     Sets the quadrature stator inductance value in the estimator in Henry (H).
//! \details   The internal quadrature inductance (Ls_q) used by the estimator can be changed in real time 
//!            by calling this function. An example showing how this is done is shown here:
//! \code

// NEED TO UPDATE

//! #define USER_MOTOR_Ls_q  (0.027)
//!
//! float_t fullScaleInductance = EST_getFullScaleInductance(handle);
//! float_t Ls_coarse_max = FLOAT_T30toF(EST_getLs_coarse_max_pu(handle));
//! int_least8_t lShift = (int_least8_t)ceil(log(USER_MOTOR_Ls_q/(Ls_coarse_max*fullScaleInductance))/log(2.0));
//! uint_least8_t Ls_qFmt = 30 - lShift;
//! float_t L_max = fullScaleInductance * pow(2.0,(float_t)lShift);
//! float_t Ls_d_pu = FLOAT_T30(USER_MOTOR_Ls_q / L_max);
//!
//! EST_setLs_q_pu(handle, Ls_q_pu);
//! EST_setLs_qFmt(handle, Ls_qFmt);
//! \endcode
//! \param[in] handle  The estimator (EST) handle
//! \param[in] Ls_q_H  The quadrature stator inductance value, H
extern void EST_setLs_q_H(EST_Handle handle,const float_t Ls_q_H);


//! \brief     Sets the maximum speed delta value during estimation
//! \param[in] handle            The estimator (EST) handle
//! \param[in] maxSpeedDelta_Hz  The maximum acceleration value, Hz
extern void EST_setMaxSpeedDelta_Hz(EST_Handle handle,const float_t maxSpeedDelta_Hz);


//! \brief     Sets the maximum current delta value in the estimator in Ampere (A).
//! \param[in] handle             The estimator (EST) handle
//! \param[in] maxCurrentDelta_A  The maximum current delta value, A
extern void EST_setMaxCurrentDelta_A(EST_Handle handle,const float_t maxCurrentDelta_A);


//! \brief     Sets the maximum EPL (Efficient Partial Load) current delta value used in the estimator
//!            in Ampere (A).
//! \param[in] handle             The estimator (EST) handle
//! \param[in] maxCurrentDelta_A  The maximum current delta value, A
extern void EST_setMaxCurrentDelta_epl_A(EST_Handle handle,const float_t maxCurrentDelta_A);


//! \brief     Sets the maximum delta value in the Id trajectory generator
//! \param[in] handle      The estimator (EST) handle
//! \param[in] maxDelta_A  The maximum delta value, A
extern void EST_setMaxDelta_Id(EST_Handle handle,const float_t maxDelta_A);


//! \brief     Sets the maximum delta value in the speed trajectory generator
//! \param[in] handle      The estimator (EST) handle
//! \param[in] maxDelta_Hz  The maximum delta value, Hz
extern void EST_setMaxDelta_spd(EST_Handle handle,const float_t maxDelta_Hz);


//! \brief     Sets the maximum value in the Id trajectory generator
//! \param[in] handle      The estimator (EST) handle
//! \param[in] maxValue_A  The maximum value, A
extern void EST_setMaxValue_Id(EST_Handle handle,const float_t maxValue_A);


//! \brief     Sets the maximum value in the speed trajectory generator
//! \param[in] handle      The estimator (EST) handle
//! \param[in] maxValue_Hz  The maximum value, Hz
extern void EST_setMaxValue_spd(EST_Handle handle,const float_t maxValue_Hz);


//! \brief     Sets the minimum value in the Id trajectory generator
//! \param[in] handle      The estimator (EST) handle
//! \param[in] minValue_A  The minimum value, A
extern void EST_setMaxValue_Id(EST_Handle handle,const float_t minValue_A);


//! \brief     Sets the minimum value in the speed trajectory generator
//! \param[in] handle      The estimator (EST) handle
//! \param[in] minValue_Hz  The minimum value, Hz
extern void EST_setMinValue_spd(EST_Handle handle,const float_t minValue_Hz);


//! \brief     Sets the estimator parameters using the user parameters structreu
//! \param[in] handle       The estimator (EST) handle
//! \param[in] pUserParams  A pointer to the user parameters structure
extern void EST_setParams(EST_Handle handle,USER_Params *pUserParams);


//! \brief     Sets the direct rotor resistance value used in the estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] Rr_d_Ohm  The direct rotor resistance value, Ohm
extern void EST_setRr_d_Ohm(EST_Handle handle,const float_t Rr_d_Ohm);


//! \brief     Sets the direct/quadrature rotor resistance values used in the estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle      The estimator (EST) handle
//! \param[in] pRr_dq_Ohm  The pointer to the vector of direct/quadrature rotor resistance values, Ohm
extern void EST_setRr_dq_Ohm(EST_Handle handle,const MATH_vec2 *pRr_dq_H);


//! \brief     Sets the quadrature rotor resistance value used in the estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] Rr_q_Ohm  The quadrature rotor resistance value, Ohm
extern void EST_setRr_q_Ohm(EST_Handle handle,const float_t Rr_q_Ohm);


//! \brief     Sets the stator resistance value in the online stator resistance estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle  The estimator (EST) handle
//! \param[in] Rs_Ohm  The stator resistance value, Ohm
extern void EST_setRsOnLine_Ohm(EST_Handle handle,const float_t Rs_Ohm);


//! \brief     Sets the online stator resistance filter parameters in per unit (pu), IQ24.
//! \param[in] handle       The estimator (EST) handle
//! \param[in] filterType   The filter type
//! \param[in] filter_0_b0  The filter 0 numerator coefficient value for z^0
//! \param[in] filter_0_a1  The filter 0 denominator coefficient value for z^(-1)
//! \param[in] filter_0_y1  The filter 0 output value at time sample n=-1
//! \param[in] filter_1_b0  The filter 1 numerator coefficient value for z^0
//! \param[in] filter_1_a1  The filter 1 denominator coefficient value for z^(-1)
//! \param[in] filter_1_y1  The filter 1 output value at time sample n=-1
extern void EST_setRsOnLineFilterParams(EST_Handle handle,const EST_RsOnLineFilterType_e filterType,
                                        const float_t filter_0_b0,const float_t filter_0_a1,const float_t filter_0_y1,
                                        const float_t filter_1_b0,const float_t filter_1_a1,const float_t filter_1_y1);


//! \brief     Sets the Id magnitude value used for online stator resistance estimation in Ampere (A).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] Id_mag_A  The Id magnitude value, A
extern void EST_setRsOnLineId_mag_A(EST_Handle handle,const float_t Id_mag_A);


//! \brief     Sets the alpha stator resistance value used in the estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] Rs_a_Ohm  The alpha stator resistance value, Ohm
extern void EST_setRs_a_Ohm(EST_Handle handle,const float_t Rs_a_Ohm);


//! \brief     Sets the alpha/beta stator resistance values used in the estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle      The estimator (EST) handle
//! \param[in] pRs_ab_Ohm  The pointer to the vector of direct/quadrature stator resistance values, Ohm
extern void EST_setRs_ab_Ohm(EST_Handle handle,const MATH_vec2 *pRs_ab_H);


//! \brief     Sets the beta stator resistance value used in the estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] Rs_b_Ohm  The beta stator resistance value, Ohm
extern void EST_setRs_b_Ohm(EST_Handle handle,const float_t Rs_b_Ohm);


//! \brief     Sets the direct stator resistance value used in the estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] Rs_d_Ohm  The direct stator resistance value, Ohm
extern void EST_setRs_d_Ohm(EST_Handle handle,const float_t Rs_d_Ohm);


//! \brief     Sets the direct/quadrature stator resistance values used in the estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle      The estimator (EST) handle
//! \param[in] pRs_dq_Ohm  The pointer to the vector of direct/quadrature stator resistance values, Ohm
extern void EST_setRs_dq_Ohm(EST_Handle handle,const MATH_vec2 *pRs_dq_H);


//! \brief     Sets the quadrature stator resistance value used in the estimator in Ohm (\f$\Omega\f$).
//! \param[in] handle    The estimator (EST) handle
//! \param[in] Rs_q_Ohm  The quadrature stator resistance value, Ohm
extern void EST_setRs_q_Ohm(EST_Handle handle,const float_t Rs_q_Ohm);


//! \brief     Sets the reference speed in the estimator
//! \param[in] handle        The estimator (EST) handle
//! \param[in] speed_ref_Hz  The reference speed, Hz
extern void EST_setSpeed_ref_Hz(EST_Handle handle,const float_t speed_ref_Hz);


//! \brief     Sets the target value in the Id trajectory generator
//! \param[in] handle      The estimator (EST) handle
//! \param[in] intValue_A  The target value, A
extern void EST_setTargetValue_Id(EST_Handle handle,const float_t targetValue_A);


//! \brief     Sets the target value in the speed trajectory generator
//! \param[in] handle      The estimator (EST) handle
//! \param[in] intValue_Hz  The target value, Hz
extern void EST_setTargetValue_spd(EST_Handle handle,const float_t targetValue_Hz);


//! \brief     Sets up the trajectory generator
//! \param[in] handle                The trajectory generator (EST_Traj) handle
//! \param[in] ctrlHandle            The controller (CTRL) handle
//! \param[in] targetValue_spd_Hz    The target speed value during run time, Hz
//! \param[in] targetValue_Id_A      The target Id current value during run time, A
extern void EST_setupTraj(EST_Handle handle,
                          CTRL_Handle ctrlHandle,
                          const float_t targetValue_spd_Hz,
                          const float_t targetValue_Id_A);


//! \brief     Updates the Id reference value used for online stator resistance estimation in Ampere (A).
//! \param[in] handle     The estimator (EST) handle
//! \param[in] pId_ref_A  The pointer to the Id reference value, A
extern void EST_updateId_ref_A(EST_Handle handle,float_t *pId_ref_A);


//! \brief      Updates the estimator state
//! \param[in]  handle       The estimator (EST) handle
//! \param[in]  Id_target_A  The target Id current during each estimator state, A
//! \return     A boolean value denoting if the state has changed (true) or not (false)
extern bool EST_updateState(EST_Handle handle,const float_t Id_target_A);


//! \brief      Updates the trajectory generator state
//! \param[in]  handle  The estimator (EST) handle
//! \return     A boolean value denoting if the state has changed (true) or not (false)
extern bool EST_updateTrajState(EST_Handle handle);


//! \brief     Determines if a zero Iq current reference should be used in the controller
//! \param[in] handle  The estimator (EST) handle
//! \return    A boolean value denoting if a zero Iq current reference should be used (true) or not (false)
extern bool EST_useZeroIq_ref(EST_Handle handle);


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _EST_H_ definition

