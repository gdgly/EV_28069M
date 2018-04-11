/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
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
//! \file   drivers/pwm/src/32b/f28x/f2806x/pwm.c
//!
//! \brief  Contains the various functions related to the 
//!         pulse width modulation (PWM) object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.

// **************************************************************************
// the includes

#include "sw/drivers/pwm/src/32b/f28x/f2806x/pwm.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions

void PWM_clearTripZone(PWM_Handle pwmHandle, const PWM_TripZoneFlag_e tripZoneFlag)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // set the bits
    ppwm->TZCLR |= tripZoneFlag;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

}  // end of PWM_clearTripZone() function


void PWM_decrementDeadBandFallingEdgeDelay(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ppwm->DBFED--;

} // end of PWM_decrementDeadBandFallingEdgeDelay() function


void PWM_decrementDeadBandRisingEdgeDelay(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ppwm->DBRED--;

} // end of PWM_decrementDeadBandRisingEdgeDelay() function


void PWM_disableAutoConvert(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    ppwm->HRCNFG &= ~PWM_HRCNFG_AUTOCONV_BITS;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of PWM_disableAutoConvert() function


void PWM_disableChopping(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;

    // clear the bits
    ppwm->PCCTL &= (~PWM_PCCTL_CHPEN_BITS);


} // end of PWM_disableChopping() function


void PWM_disableCounterLoad(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->TBCTL &= (~PWM_TBCTL_PHSEN_BITS);

} // end of PWM_disableCounterLoad() function


void PWM_disableDeadBand(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->DBCTL = 0;


} // end of PWM_disableDeadBand() function


void PWM_disableDeadBandHalfCycle(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->DBCTL &= (~PWM_DBCTL_HALFCYCLE_BITS);

} // end of PWM_disableDeadBandHalfCycle() function


void PWM_disableDigitalCompareBlankingWindow(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    ppwm->DCFCTL &= ~PWM_DCFCTL_BLANKE_BITS;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

} // end of PWM_disableDigitalCompareBlankingWindow() function


void PWM_disableDigitalCompareBlankingWindowInversion(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    ppwm->DCFCTL &= ~PWM_DCFCTL_BLANKINV_BITS;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

} // end of PWM_disableDigitalCompareBlankingWindowInversion() function


void PWM_disableHrPeriod(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    
    // clear the bits
    ppwm->HRPCTL &= ~PWM_HRPCTL_HRPE_BITS;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of PWM_disableHrPeriod() function


void PWM_disableHrPhaseSync(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    ppwm->HRPCTL &= ~PWM_HRPCTL_TBPHSHRLOADE_BITS;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of PWM_disableHrPhaseSync() function


void PWM_disableInt(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->ETSEL &= (~PWM_ETSEL_INTEN_BITS);
;
} // end of PWM_disableInt() function


void PWM_disableSocAPulse(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->ETSEL &= (~PWM_ETSEL_SOCAEN_BITS);


} // end of PWM_disableSocAPulse() function


void PWM_disableSocBPulse(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->ETSEL &= (~PWM_ETSEL_SOCBEN_BITS);

} // end of PWM_disableSocBPulse() function


void PWM_disableTripZones(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    ppwm->TZSEL = 0;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of PWM_disableTripZones() function


void PWM_disableTripZoneInt(PWM_Handle pwmHandle, const PWM_TripZoneFlag_e interruptSource)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    ppwm->TZEINT &= ~interruptSource;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of PWM_disableTripZoneInt() function


void PWM_disableTripZoneSrc(PWM_Handle pwmHandle, const PWM_TripZoneSrc_e src)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    ppwm->TZSEL &= (~src);

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of PWM_disableTripZoneSrc() function


void PWM_enableAutoConvert(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // set the bits
    ppwm->HRCNFG |= PWM_HRCNFG_AUTOCONV_BITS;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of PWM_enableAutoConvert() function


void PWM_enableChopping(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;

    // set the bits
    ppwm->PCCTL = (uint16_t)1;


} // end of PWM_enableChopping() function


void PWM_enableCounterLoad(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // set the bits
    ppwm->TBCTL |= PWM_TBCTL_PHSEN_BITS;

} // end of PWM_enableCounterLoad() function


void PWM_enableDeadBandHalfCycle(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // set the bits
    ppwm->DBCTL |= (uint16_t)PWM_DBCTL_HALFCYCLE_BITS;

} // end of PWM_enableDeadBandHalfCycle() function


void PWM_enableDigitalCompareBlankingWindow(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // set the bits
    ppwm->DCFCTL |= PWM_DCFCTL_BLANKE_BITS;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of PWM_enableDigitalCompareBlankingWindow() function


void PWM_enableDigitalCompareBlankingWindowInversion(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // set the bits
    ppwm->DCFCTL |= PWM_DCFCTL_BLANKINV_BITS;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of PWM_enableDigitalCompareBlankingWindowInversion() function


void PWM_enableHrPeriod(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // set the bits
    ppwm->HRPCTL |= PWM_HRPCTL_HRPE_BITS;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of PWM_enableHrPeriod() function


void PWM_enableInt(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // set the bits
    ppwm->ETSEL |= PWM_ETSEL_INTEN_BITS;

} // end of PWM_enableInt() function


void PWM_enableHrPhaseSync(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // set the bits
    ppwm->HRPCTL |= PWM_HRPCTL_TBPHSHRLOADE_BITS;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of PWM_enableHrPhaseSync() function


void PWM_enableSocAPulse(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // set the bits
    ppwm->ETSEL |= PWM_ETSEL_SOCAEN_BITS;


} // end of PWM_enableSocAPulse() function


void PWM_enableSocBPulse(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // set the bits
    ppwm->ETSEL |= (uint16_t)PWM_ETSEL_SOCBEN_BITS;


} // end of PWM_enableSocBPulse() function


void PWM_enableTripZoneInt(PWM_Handle pwmHandle, const PWM_TripZoneFlag_e interruptSource)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // set the bits
    ppwm->TZEINT |= interruptSource;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

} // end of PWM_enableTripZoneInt() function


void PWM_enableTripZoneSrc(PWM_Handle pwmHandle, const PWM_TripZoneSrc_e src)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // set the bits
    ppwm->TZSEL |= src;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of PWM_enableTripZoneSrc() function


uint16_t PWM_getDeadBandFallingEdgeDelay(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;

    return (ppwm->DBFED);
} // end of PWM_getDeadBandFallingEdgeDelay() function


uint16_t PWM_getDeadBandRisingEdgeDelay(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;

    return (ppwm->DBRED);
} // end of PWM_getDeadBandRisingEdgeDelay() function


uint16_t PWM_getIntCount(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;
    uint16_t u16intCount;

    u16intCount = ppwm->ETPS & PWM_ETPS_INTCNT_BITS;

    return(u16intCount);
} // end of PWM_getIntCount() function

uint16_t PWM_getSocACount(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;
    uint16_t u16intCount;

    u16intCount = ppwm->ETPS & PWM_ETPS_SOCACNT_BITS;

    u16intCount >>= 10;

    return(u16intCount);
} // end of PWM_getSocACount() function


uint16_t PWM_getSocBCount(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;
    uint16_t u16intCount;

    u16intCount = ppwm->ETPS & (uint16_t)PWM_ETPS_SOCBCNT_BITS;

    u16intCount >>= 14;

    return(u16intCount);
} // end of PWM_getSocBCount() function


PWM_ShadowStatus_e PWM_getShadowStatus_CmpA(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;
    PWM_ShadowStatus_e status;


    // clear the bits
    status = (PWM_ShadowStatus_e)(ppwm->TBCTL & (~PWM_CMPCTL_SHDWAFULL_BITS));

    status >>= 8;
    
    return(status);
} // end of PWM_getShadowStatus_CmpA() function


PWM_ShadowStatus_e PWM_getShadowStatus_CmpB(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;
    PWM_ShadowStatus_e status;


    // clear the bits
    status = (PWM_ShadowStatus_e)(ppwm->TBCTL & (~PWM_CMPCTL_SHDWAFULL_BITS));

    status >>= 9;
    
    return(status);
} // end of PWM_getShadowStatus_CmpB() function


void PWM_setHrControlMode(PWM_Handle pwmHandle, const PWM_HrControlMode_e controlMode)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    ppwm->HRCNFG &= ~PWM_HRCNFG_CTLMODE_BITS;
    
    ppwm->HRCNFG |= controlMode;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of PWM_setHrControlMode() function


void PWM_setHrEdgeMode(PWM_Handle pwmHandle, const PWM_HrEdgeMode_e edgeMode)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    ppwm->HRCNFG &= ~PWM_HRCNFG_EDGMODE_BITS;
    
    ppwm->HRCNFG |= edgeMode;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of PWM_setHrEdgeMode() function


void PWM_setHrShadowMode(PWM_Handle pwmHandle, const PWM_HrShadowMode_e shadowMode)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    ppwm->HRCNFG &= ~PWM_HRCNFG_HRLOAD_BITS;
    
    ppwm->HRCNFG |= shadowMode;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

} // end of PWM_setHrShadowMode() function


void PWM_incrementDeadBandFallingEdgeDelay(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ppwm->DBFED++;

} // end of PWM_incrementDeadBandFallingEdgeDelay() function


void PWM_incrementDeadBandRisingEdgeDelay(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ppwm->DBRED++;

} // end of PWM_incrementDeadBandRisingEdgeDelay() function


PWM_Handle PWM_init(void *pMemory, const size_t numBytes)
{
    PWM_Handle pwmHandle;


    if(numBytes < sizeof(PWM_Obj))
    	return((PWM_Handle)NULL);


    // assign the handle
    pwmHandle = (PWM_Handle)pMemory;

    return(pwmHandle);
} // end of PWM_init() function


void PWM_setActionQual_CntDown_CmpA_PwmA(PWM_Handle pwmHandle, const PWM_ActionQual_e actionQual)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->AQCTLA &= (~PWM_AQCTL_CAD_BITS);

    // set the bits
    ppwm->AQCTLA |= (actionQual << 6);


} // end of PWM_setActionQual_CntDown_CmpA_PwmA() function


void PWM_setActionQual_CntDown_CmpA_PwmB(PWM_Handle pwmHandle, const PWM_ActionQual_e actionQual)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->AQCTLB &= (~PWM_AQCTL_CAD_BITS);

    // set the bits
    ppwm->AQCTLB |= (actionQual << 6);


} // end of PWM_setActionQual_CntDown_CmpA_PwmB() function


void PWM_setActionQual_CntDown_CmpB_PwmA(PWM_Handle pwmHandle, const PWM_ActionQual_e actionQual)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->AQCTLA &= (~PWM_AQCTL_CBD_BITS);

    // set the bits
    ppwm->AQCTLA |= (actionQual << 10);


} // end of PWM_setActionQual_CntDown_CmpB_PwmA() function


void PWM_setActionQual_CntDown_CmpB_PwmB(PWM_Handle pwmHandle, const PWM_ActionQual_e actionQual)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->AQCTLB &= (~PWM_AQCTL_CBD_BITS);

    // set the bits
    ppwm->AQCTLB |= (actionQual << 10);


} // end of PWM_setActionQual_CntDown_CmpB_PwmB() function


void PWM_setActionQual_CntUp_CmpA_PwmA(PWM_Handle pwmHandle, const PWM_ActionQual_e actionQual)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->AQCTLA &= (~PWM_AQCTL_CAU_BITS);

    // set the bits
    ppwm->AQCTLA |= (actionQual << 4);


} // end of PWM_setActionQual_CntUp_CmpA_PwmA() function


void PWM_setActionQual_CntUp_CmpA_PwmB(PWM_Handle pwmHandle, const PWM_ActionQual_e actionQual)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->AQCTLB &= (~PWM_AQCTL_CAU_BITS);

    // set the bits
    ppwm->AQCTLB |= (actionQual << 4);


} // end of PWM_setActionQual_CntUp_CmpA_PwmB() function


void PWM_setActionQual_CntUp_CmpB_PwmA(PWM_Handle pwmHandle, const PWM_ActionQual_e actionQual)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->AQCTLA &= (~PWM_AQCTL_CBU_BITS);

    // set the bits
 	ppwm->AQCTLA |= (actionQual << 8);


} // end of PWM_setActionQual_CntUp_CmpB_PwmA() function


void PWM_setActionQual_CntUp_CmpB_PwmB(PWM_Handle pwmHandle, const PWM_ActionQual_e actionQual)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->AQCTLB &= (~PWM_AQCTL_CBU_BITS);

    // set the bits
    ppwm->AQCTLB |= (actionQual << 8);

} // end of PWM_setActionQual_CntUp_CmpB_PwmB() function


void PWM_setActionQualContSWForce_PwmA(PWM_Handle pwmHandle,const PWM_ActionQualContSWForce_e actionQualContSWForce)
{
	PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


	// clear the bits
	ppwm->AQCSFRC &= (~PWM_AQCSFRC_CSFA_BITS);

	// set the bits
	ppwm->AQCSFRC |= actionQualContSWForce;


} // end of PWM_setActionQualContSWForce_PwmA() function


void PWM_setActionQualContSWForce_PwmB(PWM_Handle pwmHandle,const PWM_ActionQualContSWForce_e actionQualContSWForce)
{
	PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


	// clear the bits
	ppwm->AQCSFRC &= (~PWM_AQCSFRC_CSFB_BITS);

	// set the bits
	ppwm->AQCSFRC |= (actionQualContSWForce << 2);


} // end of PWM_setActionQualContSWForce_PwmA() function


void PWM_setActionQual_Period_PwmA(PWM_Handle pwmHandle,const PWM_ActionQual_e actionQual)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->AQCTLA &= (~PWM_AQCTL_PRD_BITS);

    // set the bits
    ppwm->AQCTLA |= (actionQual << 2);


} // end of PWM_setActionQual_Period_PwmA() function


void PWM_setActionQual_Period_PwmB(PWM_Handle pwmHandle, const PWM_ActionQual_e actionQual)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->AQCTLB &= (~PWM_AQCTL_PRD_BITS);

    // set the bits
    ppwm->AQCTLB |= (actionQual << 2);


} // end of PWM_setActionQual_Period_PwmB() function


void PWM_setActionQual_Zero_PwmA(PWM_Handle pwmHandle, const PWM_ActionQual_e actionQual)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->AQCTLA &= (~PWM_AQCTL_ZRO_BITS);

    // set the bits
    ppwm->AQCTLA |= actionQual;


} // end of PWM_setActionQualZero_PwmA() function


void PWM_setActionQual_Zero_PwmB(PWM_Handle pwmHandle, const PWM_ActionQual_e actionQual)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->AQCTLB &= (~PWM_AQCTL_ZRO_BITS);

    // set the bits
    ppwm->AQCTLB |= actionQual;


} // end of PWM_setActionQualZero_PwmB() function


void PWM_setChoppingClkFreq(PWM_Handle pwmHandle, const PWM_ChoppingClkFreq_e clkFreq)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->PCCTL &= (~PWM_PCCTL_CHPFREQ_BITS);

    // set the bits
    ppwm->PCCTL |= clkFreq;

} // end of PWM_setChoppingClkFreq() function


void PWM_setChoppingDutyCycle(PWM_Handle pwmHandle, const PWM_ChoppingDutyCycle_e dutyCycle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->PCCTL &= (~PWM_PCCTL_CHPDUTY_BITS);

    // set the bits
    ppwm->PCCTL |= dutyCycle;


} // end of PWM_setChoppingDutyCycle() function


void PWM_setChoppingPulseWidth(PWM_Handle pwmHandle, const PWM_ChoppingPulseWidth_e pulseWidth)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->PCCTL &= (~PWM_PCCTL_OSHTWTH_BITS);

    // set the bits
    ppwm->PCCTL |= pulseWidth;


} // end of PWM_setChoppingPulseWidth() function


void PWM_setClkDiv(PWM_Handle pwmHandle, const PWM_ClkDiv_e clkDiv)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->TBCTL &= (~PWM_TBCTL_CLKDIV_BITS);

    // set the bits
    ppwm->TBCTL |= clkDiv;


} // end of PWM_setClkDiv() function


void PWM_setCount(PWM_Handle pwmHandle, const uint16_t u16count)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // set the bits
    ppwm->TBCTR = u16count;

} // end of PWM_setCount() function


void PWM_setCounterMode(PWM_Handle pwmHandle, const PWM_CounterMode_e counterMode)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->TBCTL &= (~PWM_TBCTL_CTRMODE_BITS);

    // set the bits
    ppwm->TBCTL |= counterMode;

} // end of PWM_setCounterMode() function


void PWM_setDeadBandFallingEdgeDelay(PWM_Handle pwmHandle,const uint16_t u16delay)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ppwm->DBFED = u16delay;


} // end of PWM_setDeadBandFallingEdgeDelay() function


void PWM_setDeadBandInputMode(PWM_Handle pwmHandle, const PWM_DeadBandInputMode_e inputMode)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->DBCTL &= (~PWM_DBCTL_INMODE_BITS);
    
    // set the bits
    ppwm->DBCTL |= inputMode;


} // end of PWM_setDeadBandInputMode() function


void PWM_setDeadBandOutputMode(PWM_Handle pwmHandle, const PWM_DeadBandOutputMode_e outputMode)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->DBCTL &= (~PWM_DBCTL_OUTMODE_BITS);
    
    // set the bits
    ppwm->DBCTL |= outputMode;

} // end of PWM_setDeadBandOutputMode() function


void PWM_setDeadBandPolarity(PWM_Handle pwmHandle, const PWM_DeadBandPolarity_e polarity)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->DBCTL &= (~PWM_DBCTL_POLSEL_BITS);
    
    // set the bits
    ppwm->DBCTL |= polarity;
} // end of PWM_setDeadBandPolarity() function


void PWM_setDeadBandRisingEdgeDelay(PWM_Handle pwmHandle,const uint16_t u16delay)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ppwm->DBRED = u16delay;

} // end of PWM_setDeadBandRisingEdgeDelay() function



void PWM_setDigitalCompareFilterSource(PWM_Handle pwmHandle, 
                                const PWM_DigitalCompare_FilterSrc_e input)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // Clear any old values
    ppwm->DCFCTL &= ~PWM_DCFCTL_SRCSEL_BITS;
    
    // Set the new value
    ppwm->DCFCTL |= input;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

} // end of PWM_setDigitalCompareFilterSource() function


void PWM_setDigitalCompareBlankingPulse(PWM_Handle pwmHandle, 
                                const PWM_DigitalCompare_PulseSel_e pulseSelect)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // Clear any old values
    ppwm->DCFCTL &= ~PWM_DCFCTL_PULSESEL_BITS;
    
    // Set the new value
    ppwm->DCFCTL |= pulseSelect << 4;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of PWM_setDigitalCompareBlankingPulse() function


void PWM_setDigitalCompareFilterOffset(PWM_Handle pwmHandle, 
                                const uint16_t u16offset)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;

    // Set the filter offset
    ppwm->DCFOFFSET = u16offset;

} // end of PWM_setDigitalCompareFilterOffset() function


void PWM_setDigitalCompareFilterWindow(PWM_Handle pwmHandle, 
                                const uint16_t u16window)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;

    // Set the window
    ppwm->DCFWINDOW = u16window;

} // end of PWM_setDigitalCompareFilterWindow() function


void PWM_setDigitalCompareInput(PWM_Handle pwmHandle, 
                                const PWM_DigitalCompare_Input_e input, 
                                const PWM_DigitalCompare_InputSel_e inputSel)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // Clear any old values
    ppwm->DCTRIPSEL &= ~(0x000F << input);

    // Set the new value
    ppwm->DCTRIPSEL |= (inputSel << input);
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

} // end of PWM_setDigitalCompareInput() function


void PWM_setDigitalCompareAEvent1(PWM_Handle pwmHandle, 
                                const bool bselectFilter,
                                const bool bdisableSync,
                                const bool benableSoc,
                                const bool bgenerateSync)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // Clear any old values
    ppwm->DCACTL &= ~0x000F;
    
    // Set the new value
    ppwm->DCACTL |= bselectFilter | (bdisableSync << 1) | (benableSoc << 2) | (bgenerateSync << 3);
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of PWM_setDigitalCompareAEvent1() function


void PWM_setDigitalCompareAEvent2(PWM_Handle pwmHandle, 
                                const bool bselectFilter,
                                const bool bdisableSync)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // Clear any old values
    ppwm->DCACTL &= ~0x0300;
    
    // Set the new value
    ppwm->DCACTL |= (bselectFilter << 8) | (bdisableSync << 9) ;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of PWM_setDigitalCompareAEvent2() function


void PWM_setDigitalCompareBEvent1(PWM_Handle pwmHandle, 
                                const bool bselectFilter,
                                const bool bdisableSync,
                                const bool benableSoc,
                                const bool bgenerateSync)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // Clear any old values
    ppwm->DCBCTL &= ~0x000F;
    
    // Set the new value
    ppwm->DCBCTL |= bselectFilter | (bdisableSync << 1) | (benableSoc << 2) | (bgenerateSync << 3);
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

} // end of PWM_setDigitalCompareBEvent1() function


void PWM_setDigitalCompareBEvent2(PWM_Handle pwmHandle, 
                                const bool bselectFilter,
                                const bool bdisableSync)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;
    
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // Clear any old values
    ppwm->DCBCTL &= ~0x0300;
    
    // Set the new value
    ppwm->DCBCTL |= (bselectFilter << 8) | (bdisableSync << 9) ;
    
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


} // end of PWM_setDigitalCompareBEvent2() function


void PWM_setHighSpeedClkDiv(PWM_Handle pwmHandle, const PWM_HspClkDiv_e clkDiv)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->TBCTL &= (~PWM_TBCTL_HSPCLKDIV_BITS);

    // set the bits
    ppwm->TBCTL |= clkDiv;


} // end of PWM_setHighSpeedClkDiv() function


void PWM_setIntMode(PWM_Handle pwmHandle, const PWM_IntMode_e intMode)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->ETSEL &= (~PWM_ETSEL_INTSEL_BITS);

    // set the bits
    ppwm->ETSEL |= intMode;


} // end of PWM_setIntMode() function


void PWM_setIntPeriod(PWM_Handle pwmHandle, const PWM_IntPeriod_e intPeriod)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->ETPS &= (~PWM_ETPS_INTPRD_BITS);

    // set the bits
    ppwm->ETPS |= intPeriod;

} // end of PWM_setIntPeriod() function


void PWM_setLoadMode_CmpA(PWM_Handle pwmHandle, const PWM_LoadMode_e loadMode)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->CMPCTL &= (~PWM_CMPCTL_LOADAMODE_BITS);

    // set the bits
    ppwm->CMPCTL |= loadMode;


} // end of PWM_setLoadMode_CmpA() function


void PWM_setLoadMode_CmpB(PWM_Handle pwmHandle, const PWM_LoadMode_e loadMode)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->CMPCTL &= (~PWM_CMPCTL_LOADBMODE_BITS);

    // set the bits
    ppwm->CMPCTL |= (loadMode << 2);


} // end of PWM_setLoadMode_CmpB() function


void PWM_setPeriod(PWM_Handle pwmHandle, const uint16_t u16period)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // initialize the Time-Base Period Register (TBPRD).  These bits determine the period of the time-base counter.
    ppwm->TBPRD = u16period;


} // end of PWM_setPeriod() function


void PWM_setPeriodHr(PWM_Handle pwmHandle, const uint16_t u16period)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // initialize the Time-Base Period Register (TBPRD).  These bits determine the period of the time-base counter.
    ppwm->TBPRDHR = u16period;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

} // end of PWM_setPeriodHr() function


void PWM_setPhase(PWM_Handle pwmHandle, const uint16_t u16phase)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ppwm->TBPHS = u16phase;


} // end of PWM_setPhase() function


void PWM_setPhaseDir(PWM_Handle pwmHandle, const PWM_PhaseDir_e phaseDir)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->TBCTL &= (~PWM_TBCTL_PHSDIR_BITS);

    // set the bits
    ppwm->TBCTL |= phaseDir;


} // end of PWM_setPhaseDir() function


void PWM_setPeriodLoad(PWM_Handle pwmHandle, const PWM_PeriodLoad_e periodLoad)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->TBCTL &= (~PWM_TBCTL_PRDLD_BITS);

    // set the bits
    ppwm->TBCTL |= periodLoad;


} // end of PWM_setPeriodLoad() function


void PWM_setRunMode(PWM_Handle pwmHandle, const PWM_RunMode_e runMode)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->TBCTL &= (~PWM_TBCTL_FREESOFT_BITS);

    // set the bits
    ppwm->TBCTL |= runMode;


} // end of PWM_setRunMode() function


void PWM_setSocAPeriod(PWM_Handle pwmHandle, const PWM_SocPeriod_e intPeriod)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->ETPS &= (~PWM_ETPS_SOCAPRD_BITS);

    // set the bits
    ppwm->ETPS |= (intPeriod << 8);


} // end of PWM_setSocAPeriod() function


void PWM_setSocAPulseSrc(PWM_Handle pwmHandle, const PWM_SocPulseSrc_e pulseSrc)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->ETSEL &= (~PWM_ETSEL_SOCASEL_BITS);

    // set the bits
    ppwm->ETSEL |= (pulseSrc << 8);


} // end of PWM_setSocAPulseSrc() function


void PWM_setSocBPeriod(PWM_Handle pwmHandle, const PWM_SocPeriod_e intPeriod)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->ETPS &= (~PWM_ETPS_SOCBPRD_BITS);

    // set the bits
    ppwm->ETPS |= (intPeriod << 12);

} // end of PWM_setSocBPeriod() function


void PWM_setSocBPulseSrc(PWM_Handle pwmHandle, const PWM_SocPulseSrc_e pulseSrc)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->ETSEL &= (~PWM_ETSEL_SOCBSEL_BITS);

    // set the bits
    ppwm->ETSEL |= (pulseSrc << 12);


} // end of PWM_setSocBPulseSrc() function


void PWM_setShadowMode_CmpA(PWM_Handle pwmHandle, const PWM_ShadowMode_e shadowMode)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->CMPCTL &= (~PWM_CMPCTL_SHDWAMODE_BITS);

    // set the bits
    ppwm->CMPCTL |= (shadowMode << 4);

} // end of PWM_setShadowMode_CmpA() function


void PWM_setShadowMode_CmpB(PWM_Handle pwmHandle, const PWM_ShadowMode_e shadowMode)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->CMPCTL &= (~PWM_CMPCTL_SHDWBMODE_BITS);

    // set the bits
    ppwm->CMPCTL |= (shadowMode << 6);

} // end of PWM_setShadowMode_CmpB() function


void PWM_setSwSync(PWM_Handle pwmHandle)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // set the bits
    ppwm->TBCTL |= 1 << 6;


} // end of PWM_setSwSync() function


void PWM_setSyncMode(PWM_Handle pwmHandle, const PWM_SyncMode_e syncMode)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    // clear the bits
    ppwm->TBCTL &= (~PWM_TBCTL_SYNCOSEL_BITS);

    // set the bits
    ppwm->TBCTL |= syncMode;
;
} // end of PWM_setSyncMode() function

void PWM_setTripZoneDCEventSelect_DCAEVT1(PWM_Handle pwmHandle, const PWM_TripZoneDCEventSel_e tripZoneEvent)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    ppwm->TZDCSEL &= (~PWM_TZDCSEL_DCAEVT1_BITS);

    // set the bits
    ppwm->TZDCSEL |= tripZoneEvent << 0;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

}  // end of PWM_setTripZoneDCEventSelect_DCAEVT1() function


void PWM_setTripZoneDCEventSelect_DCAEVT2(PWM_Handle pwmHandle, const PWM_TripZoneDCEventSel_e tripZoneEvent)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    ppwm->TZDCSEL &= (~PWM_TZDCSEL_DCAEVT2_BITS);

    // set the bits
    ppwm->TZDCSEL |= tripZoneEvent << 3;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


}  // end of PWM_setTripZoneDCEventSelect_DCAEVT2() function


void PWM_setTripZoneDCEventSelect_DCBEVT1(PWM_Handle pwmHandle, const PWM_TripZoneDCEventSel_e tripZoneEvent)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    ppwm->TZDCSEL &= (~PWM_TZDCSEL_DCBEVT1_BITS);

    // set the bits
    ppwm->TZDCSEL |= tripZoneEvent << 6;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

}  // end of PWM_setTripZoneDCEventSelect_DCBEVT1() function


void PWM_setTripZoneDCEventSelect_DCBEVT2(PWM_Handle pwmHandle, const PWM_TripZoneDCEventSel_e tripZoneEvent)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    ppwm->TZDCSEL &= (~PWM_TZDCSEL_DCBEVT2_BITS);

    // set the bits
    ppwm->TZDCSEL |= tripZoneEvent << 9;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


}  // end of PWM_setTripZoneDCEventSelect_DCBEVT2() function


void PWM_setTripZoneState_DCAEVT1(PWM_Handle pwmHandle, const PWM_TripZoneState_e tripZoneState)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    ppwm->TZCTL &= (~PWM_TZCTL_DCAEVT1_BITS);

    // set the bits
    ppwm->TZCTL |= tripZoneState << 4;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


}  // end of PWM_setTripZoneState_DCAEVT1() function


void PWM_setTripZoneState_DCAEVT2(PWM_Handle pwmHandle, const PWM_TripZoneState_e tripZoneState)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    ppwm->TZCTL &= (~PWM_TZCTL_DCAEVT2_BITS);

    // set the bits
    ppwm->TZCTL |= tripZoneState << 6;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


}  // end of PWM_setTripZoneState_DCAEVT2() function


void PWM_setTripZoneState_DCBEVT1(PWM_Handle pwmHandle, const PWM_TripZoneState_e tripZoneState)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    ppwm->TZCTL &= (~PWM_TZCTL_DCBEVT1_BITS);

    // set the bits
    ppwm->TZCTL |= tripZoneState << 8;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


}  // end of PWM_setTripZoneState_DCBEVT1() function


void PWM_setTripZoneState_DCBEVT2(PWM_Handle pwmHandle, const PWM_TripZoneState_e tripZoneState)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    ppwm->TZCTL &= (~PWM_TZCTL_DCBEVT2_BITS);

    // set the bits
    ppwm->TZCTL |= tripZoneState << 10;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

}  // end of PWM_setTripZoneState_DCBEVT2() function


void PWM_setTripZoneState_TZA(PWM_Handle pwmHandle, const PWM_TripZoneState_e tripZoneState)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    ppwm->TZCTL &= (~PWM_TZCTL_TZA_BITS);

    // set the bits
    ppwm->TZCTL |= tripZoneState << 0;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

}  // end of PWM_setTripZoneState_TZA() function


void PWM_setTripZoneState_TZB(PWM_Handle pwmHandle, const PWM_TripZoneState_e tripZoneState)
{
    PWM_Obj *ppwm = (PWM_Obj *)pwmHandle;


    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // clear the bits
    ppwm->TZCTL &= (~PWM_TZCTL_TZB_BITS);

    // set the bits
    ppwm->TZCTL |= tripZoneState << 2;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;


}  // end of PWM_setTripZoneState_TZB() function


// end of file
