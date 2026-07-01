/**
  ******************************************************************************
  * @file    ics_l4xx_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the ICS
  *          PWM current feedback component for L4xx of the Motor Control SDK.
  ********************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  *
  * @ingroup ICS_L4XX_pwm_curr_fdbk
  */

/* Includes ------------------------------------------------------------------*/
#include "ics_l4xx_pwm_curr_fdbk.h"
#include "pwm_common.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */


/** @addtogroup ConvFOC
  * @{
  */

/** @addtogroup MCLLAPI
  * @{
  */
/** @addtogroup pwm_curr_fdbk
  * @{
  */
 
/** @addtogroup ICS_pwm_curr_fdbk
  * @{
  */

/* Private defines -----------------------------------------------------------*/
#define TIMxCCER_MASK_CH123        (LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3 | \
                                    LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3N)
                                    
/* Private function prototypes -----------------------------------------------*/
void ICS_HFCurrentsCalibration( PWMC_Handle_t * pHdl, ab_t * pStator_Currents );
static void ICS_RLGetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * pStator_Currents );
static void ICS_RLTurnOnLowSides( PWMC_Handle_t * pHdl, uint32_t ticks );
static void ICS_RLSwitchOnPWM( PWMC_Handle_t * pHdl );

/*
  * @brief  Initializes TIMx, ADC, GPIO and NVIC for current reading
  *         in ICS configuration using STM32F30X.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  */
__weak void ICS_Init( PWMC_ICS_Handle_t * pHandle )
{
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;

  if ( ( uint32_t )pHandle == ( uint32_t )&pHandle->_Super )
  {

    /* disable IT and flags in case of LL driver usage
     * workaround for unwanted interrupt enabling done by LL driver */
    LL_ADC_DisableIT_EOC( ADCx_1 );
    LL_ADC_ClearFlag_EOC( ADCx_1 );
    LL_ADC_DisableIT_JEOC( ADCx_1 );
    LL_ADC_ClearFlag_JEOC( ADCx_1 );
    LL_ADC_DisableIT_EOC( ADCx_2 );
    LL_ADC_ClearFlag_EOC( ADCx_2 );
    LL_ADC_DisableIT_JEOC( ADCx_2 );
    LL_ADC_ClearFlag_JEOC( ADCx_2 );


    /* ADCx_1 and ADCx_2 registers configuration ---------------------------------*/
    /* ADCx_1 and ADCx_2 registers reset */

    LL_ADC_EnableInternalRegulator( ADCx_1 );
    LL_ADC_EnableInternalRegulator( ADCx_2 );

    /* Wait for Regulator Startup time, once for both */
    volatile uint32_t wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US / 10UL) * (SystemCoreClock / (100000UL * 2UL)));      
    while(wait_loop_index != 0UL)
    {
      wait_loop_index--;
    }

    LL_ADC_StartCalibration( ADCx_1, LL_ADC_SINGLE_ENDED );
    while ( LL_ADC_IsCalibrationOnGoing( ADCx_1 ) )
    {
    }

    LL_ADC_StartCalibration( ADCx_2, LL_ADC_SINGLE_ENDED );
    while ( LL_ADC_IsCalibrationOnGoing( ADCx_2 ) )
    {
    }

    /* ADCx_1 and ADCx_2 registers configuration ---------------------------------*/
    /* Enable ADCx_1 and ADCx_2 */
    LL_ADC_Enable( ADCx_1 );
    LL_ADC_Enable( ADCx_2 );

    /* reset regular conversion sequencer length set by cubeMX */
    LL_ADC_REG_SetSequencerLength( ADCx_1, LL_ADC_REG_SEQ_SCAN_DISABLE );

    LL_ADC_INJ_SetQueueMode( ADCx_1, LL_ADC_INJ_QUEUE_2CONTEXTS_END_EMPTY );
    LL_ADC_INJ_SetQueueMode( ADCx_2, LL_ADC_INJ_QUEUE_2CONTEXTS_END_EMPTY );

    LL_ADC_INJ_StartConversion( ADCx_1 );
    LL_ADC_INJ_StartConversion( ADCx_2 );

    /* disable main TIM counter to ensure
     * a synchronous start by TIM2 trigger */
    LL_TIM_DisableCounter( TIMx );

    /* Enables the TIMx Preload on CC1 Register */
    LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH1 );
    /* Enables the TIMx Preload on CC2 Register */
    LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH2 );
    /* Enables the TIMx Preload on CC3 Register */
    LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH3 );
    /* Enables the TIMx Preload on CC4 Register */
    LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH4 );

    /* set TRGO to update, this will flush the ADC JSQR
      when update will be sent during TIM init */
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_UPDATE);

    /* Always enable BKIN for safety feature */
    LL_TIM_ClearFlag_BRK( TIMx );
    LL_TIM_ClearFlag_BRK2( TIMx );
    LL_TIM_EnableIT_BRK( TIMx );

    LL_TIM_SetCounter( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u );

    /* Enable PWM channel */
    LL_TIM_CC_EnableChannel( TIMx, TIMxCCER_MASK_CH123 );

    if ( TIMx == TIM1 )
    {
      /* TIM1 Counter Clock stopped when the core is halted */
      LL_DBGMCU_APB2_GRP1_FreezePeriph( LL_DBGMCU_APB2_GRP1_TIM1_STOP );
    }
#if defined(TIM8)
    else
    {
      /* TIM8 Counter Clock stopped when the core is halted */
      LL_DBGMCU_APB2_GRP1_FreezePeriph( LL_DBGMCU_APB2_GRP1_TIM8_STOP );
    }
#endif
    /* Enable Update IRQ */
    LL_TIM_ClearFlag_UPDATE(TIMx);
    LL_TIM_EnableIT_UPDATE(TIMx);
  }
}


/*
  * @brief  Stores in @p pHdl handler the calibrated @p offsets.
  * 
  */
__weak void ICS_SetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets)
{
  PWMC_ICS_Handle_t *pHandle = (PWMC_ICS_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3

  pHandle->PhaseAOffset = offsets->phaseAOffset;
  pHandle->PhaseBOffset = offsets->phaseBOffset;
  pHdl->offsetCalibStatus = true;
}

/*
  * @brief Reads the calibrated @p offsets stored in @p pHdl.
  * 
  */
__weak void ICS_GetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets)
{
  PWMC_ICS_Handle_t *pHandle = (PWMC_ICS_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3

  offsets->phaseAOffset = pHandle->PhaseAOffset;
  offsets->phaseBOffset = pHandle->PhaseBOffset;
}

/*
  * @brief  Stores into the @p pHdl handler the voltage present on Ia and
  *         Ib current feedback analog channels when no current is flowing into the
  *         motor.
  * 
  */
__weak void ICS_CurrentReadingCalibration( PWMC_Handle_t * pHdl )
{
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * ) pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  if (false == pHandle->_Super.offsetCalibStatus)
  {
    pHandle->PhaseAOffset = 0u;
    pHandle->PhaseBOffset = 0u;
    pHandle->PolarizationCounter = 0u;

    /* Force inactive level on TIMx CHy and TIMx CHyN */
    LL_TIM_CC_DisableChannel( TIMx, TIMxCCER_MASK_CH123 );

    /* Change function to be executed in ADCx_ISR */
    __disable_irq();
    pHandle->_Super.pFctGetPhaseCurrents = &ICS_HFCurrentsCalibration;
    __enable_irq();
    ICS_SwitchOnPWM( &pHandle->_Super );

    /* Wait for NB_CONVERSIONS to be executed */
    waitForPolarizationEnd( TIMx,
                            &pHandle->_Super.SWerror,
                            pHandle->pParams_str->RepetitionCounter,
                            &pHandle->PolarizationCounter );

    ICS_SwitchOffPWM( &pHandle->_Super );

    pHandle->PhaseAOffset /= NB_CONVERSIONS;
    pHandle->PhaseBOffset /= NB_CONVERSIONS;
    if (0U == pHandle->_Super.SWerror)
    {
      pHandle->_Super.offsetCalibStatus = true;
    }
    else
    {
      /* nothing to do */
    }

    /* Change back function to be executed in ADCx_ISR */
    __disable_irq();
    pHandle->_Super.pFctGetPhaseCurrents = &ICS_GetPhaseCurrents;
    __enable_irq();
  }
	
  /* Disable TIMx preload */
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  /* It over write TIMx CCRy wrongly written by FOC during calibration so as to
   force 50% duty cycle on the three inverer legs */
  LL_TIM_OC_SetCompareCH1 (TIMx, pHandle->Half_PWMPeriod >> 1u);
  LL_TIM_OC_SetCompareCH2 (TIMx, pHandle->Half_PWMPeriod >> 1u);
  LL_TIM_OC_SetCompareCH3 (TIMx, pHandle->Half_PWMPeriod >> 1u);
  /* Apply new CC values */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);

  /* Set back TIMx CCER register */
  LL_TIM_CC_EnableChannel( TIMx, TIMxCCER_MASK_CH123 );

  pHandle->_Super.BrakeActionLock = false;
}

/*
  * @brief  Computes and stores in @p pHdl handler the latest converted motor phase currents in @p pStator_Currents ab_t format.
  *
  */
__weak void ICS_GetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * pStator_Currents )
{
  int32_t aux;
  uint16_t reg;
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * ) pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;

  /* disable ADC trigger */
  LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);

  /* Ia = (hPhaseAOffset)-(PHASE_A_ADC_CHANNEL value)  */
  reg = ( uint16_t )( ADCx_1->JDR1 );
  aux = ( int32_t )( reg ) - ( int32_t )( pHandle->PhaseAOffset );

  /* Saturation of Ia */
  if ( aux < -INT16_MAX )
  {
    pStator_Currents->a = -INT16_MAX;
  }
  else  if ( aux > INT16_MAX )
  {
    pStator_Currents->a = INT16_MAX;
  }
  else
  {
    pStator_Currents->a = ( int16_t )aux;
  }

  /* Ib = (hPhaseBOffset)-(PHASE_B_ADC_CHANNEL value) */
  reg = ( uint16_t )( ADCx_2->JDR1 );
  aux = ( int32_t )( reg ) - ( int32_t )( pHandle->PhaseBOffset );

  /* Saturation of Ib */
  if ( aux < -INT16_MAX )
  {
    pStator_Currents->b = -INT16_MAX;
  }
  else  if ( aux > INT16_MAX )
  {
    pStator_Currents->b = INT16_MAX;
  }
  else
  {
    pStator_Currents->b = ( int16_t )aux;
  }

  pHandle->_Super.Ia = pStator_Currents->a;
  pHandle->_Super.Ib = pStator_Currents->b;
  pHandle->_Super.Ic = -pStator_Currents->a - pStator_Currents->b;

}


/*
* @brief  Sums up injected conversion data into wPhaseXOffset.
*
* It is called only during current calibration.
*
* @param  pHdl: Handler of the current instance of the PWM component.
* @param  pStator_Currents: Pointer to the structure that will receive motor current
*         of phase A and B in ab_t format.
*/
__weak void ICS_HFCurrentsCalibration( PWMC_Handle_t * pHdl, ab_t * pStator_Currents )
{
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * ) pHdl;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* disable ADC trigger */
  LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);

  if ( pHandle->PolarizationCounter < NB_CONVERSIONS )
  {
    pHandle->PhaseAOffset += ADCx_1->JDR1;
    pHandle->PhaseBOffset += ADCx_2->JDR1;
    pHandle->PolarizationCounter++;
  }

  /* during offset calibration no current is flowing in the phases */
  pStator_Currents->a = 0;
  pStator_Currents->b = 0;
}

/*
  * @brief  Turns on low sides switches.
  * 
  * This function is intended to be used for charging boot capacitors of driving section. It has to be
  * called on each motor start-up when using high voltage drivers.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  ticks: Timer ticks value to be applied
  *                Min value: 0 (low sides ON)
  *                Max value: PWM_PERIOD_CYCLES/2 (low sides OFF)
  */
__weak void ICS_TurnOnLowSides( PWMC_Handle_t * pHdl, uint32_t ticks )
{
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * ) pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.TurnOnLowSidesAction = true;

  /* Disable TIMx preload */
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  /*Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1 ( TIMx, ticks );
  LL_TIM_OC_SetCompareCH2 ( TIMx, ticks );
  LL_TIM_OC_SetCompareCH3 ( TIMx, ticks );
  /* Apply new CC values */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs( TIMx );
  if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
    LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
    LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
  }

  return;
}

/*
  * @brief  Enables PWM generation on the proper Timer peripheral acting on MOE bit.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
__weak void ICS_SwitchOnPWM( PWMC_Handle_t * pHdl )
{
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * ) pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Disable TIMx preload */
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  /* Set all duty to 50% */
  LL_TIM_OC_SetCompareCH1(TIMx, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
  LL_TIM_OC_SetCompareCH2(TIMx, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
  LL_TIM_OC_SetCompareCH3(TIMx, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
  LL_TIM_OC_SetCompareCH4(TIMx, (uint32_t)(pHandle->Half_PWMPeriod - 5u));
  /* Apply new CC values */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);

  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  LL_TIM_EnableAllOutputs( TIMx );

  if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
  {
    if ( LL_TIM_CC_IsEnabledChannel(TIMx, TIMxCCER_MASK_CH123) != 0u )
    {
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
    }
  }
  pHandle->_Super.PWMState = true;
}

/*
  * @brief  Disables PWM generation on the proper Timer peripheral acting on MOE bit.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
__weak void ICS_SwitchOffPWM( PWMC_Handle_t * pHdl )
{
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * ) pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  pHandle->_Super.PWMState = false;
  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs(TIMx);
  if ( pHandle->_Super.BrakeActionLock == true )
  {
  }
  else
  {
    if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
    {
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
    }
  }

  return;
}

/*
  * @brief  Writes into peripheral registers the new duty cycle and sampling point.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  SamplingPoint: Point at which the ADC will be triggered, written in timer clock counts.
  * @retval Returns #MC_NO_ERROR if no error occurred or #MC_DURATION if the duty cycles were
  *         set too late for being taken into account in the next PWM cycle.
  */
__weak uint16_t ICS_WriteTIMRegisters( PWMC_Handle_t * pHdl )
{
  uint16_t aux;
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * ) pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  LL_TIM_OC_SetCompareCH1( TIMx, pHandle->_Super.CntPhA );
  LL_TIM_OC_SetCompareCH2( TIMx, pHandle->_Super.CntPhB );
  LL_TIM_OC_SetCompareCH3( TIMx, pHandle->_Super.CntPhC );

  /* Limit for update event */
  /* Check the status of SOFOC flag. If it is set, an update event has occurred
  and thus the FOC rate is too high */

  if (((TIMx->CR2) & TIM_CR2_MMS_Msk) != LL_TIM_TRGO_RESET )
  {
    aux = MC_DURATION;
  }
  else
  {
    aux = MC_NO_ERROR;
  }
  return aux;
}

/*
  * @brief  Contains the TIMx Update event interrupt.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  */
__weak void * ICS_TIMx_UP_IRQHandler( PWMC_ICS_Handle_t * pHandle )
{
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* set JSQR register */
  ADCx_1->JSQR = (uint32_t) pHandle->pParams_str->ADCConfig1;
  ADCx_2->JSQR = (uint32_t) pHandle->pParams_str->ADCConfig2;

  LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_OC4REF);
    
  return &( pHandle->_Super.Motor );
}

/*
  * @brief  Sets the PWM mode for R/L detection.
  *
  * Specific for Motor Profiling using Isolated Current Sensors.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
void ICS_RLDetectionModeEnable( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  if ( pHandle->_Super.RLDetectionMode == false )
  {
    /*  Channel1 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1 );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH1 );
    LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH1N );
    LL_TIM_OC_SetCompareCH1( TIMx, 0u );

    /*  Channel2 configuration */
    if ( ( pHandle->_Super.LowSideOutputs ) == LS_PWM_TIMER )
    {
      LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_ACTIVE );
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH2 );
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    }
    else if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
    {
      LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_INACTIVE );
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2 );
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    }
    else
    {
    }

    /*  Channel3 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM2 );
    LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH3 );
    LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH3N );

  }
  __disable_irq();
  pHandle->_Super.pFctGetPhaseCurrents = &ICS_RLGetPhaseCurrents;
  pHandle->_Super.pFctTurnOnLowSides = &ICS_RLTurnOnLowSides;
  pHandle->_Super.pFctSwitchOnPwm = &ICS_RLSwitchOnPWM;
  pHandle->_Super.pFctSwitchOffPwm = &ICS_SwitchOffPWM;
  __enable_irq();

  pHandle->_Super.RLDetectionMode = true;
}

/*
  * @brief  Disables the PWM mode for R/L detection.
  *
  * Specific for Motor Profiling using Isolated Current Sensors.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
void ICS_RLDetectionModeDisable( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  if ( pHandle->_Super.RLDetectionMode == true )
  {
    /*  Channel1 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1 );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH1 );

    if ( ( pHandle->_Super.LowSideOutputs ) == LS_PWM_TIMER )
    {
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH1N );
    }
    else if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
    {
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH1N );
    }
    else
    {
    }

    LL_TIM_OC_SetCompareCH1( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) >> 1 );

    /*  Channel2 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1 );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2 );

    if ( ( pHandle->_Super.LowSideOutputs ) == LS_PWM_TIMER )
    {
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    }
    else if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
    {
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    }
    else
    {
    }

    LL_TIM_OC_SetCompareCH2( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) >> 1 );

    /*  Channel3 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1 );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH3 );

    if ( ( pHandle->_Super.LowSideOutputs ) == LS_PWM_TIMER )
    {
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH3N );
    }
    else if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
    {
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH3N );
    }
    else
    {
    }

    LL_TIM_OC_SetCompareCH3( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) >> 1 );

    __disable_irq();
    pHandle->_Super.pFctGetPhaseCurrents = &ICS_GetPhaseCurrents;
    pHandle->_Super.pFctTurnOnLowSides = &ICS_TurnOnLowSides;
    pHandle->_Super.pFctSwitchOnPwm = &ICS_SwitchOnPWM;
    pHandle->_Super.pFctSwitchOffPwm = &ICS_SwitchOffPWM;
    __enable_irq();

    pHandle->_Super.RLDetectionMode = false;
  }
}

/*
  * @brief  Sets the PWM dutycycle for R/L detection.
  *
  * Specific for Motor Profiling using Isolated Current Sensors.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  hDuty: Duty cycle to apply, written in uint16_t.
  * @retval Returns #MC_NO_ERROR if no error occurred or #MC_DURATION if the duty cycles were
  *         set too late for being taken into account in the next PWM cycle.
  */
uint16_t ICS_RLDetectionModeSetDuty( PWMC_Handle_t * pHdl, uint16_t hDuty )
{
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  uint32_t val;
  uint16_t hAux;

  val = ( ( uint32_t )( pHandle->Half_PWMPeriod ) * ( uint32_t )( hDuty ) ) >> 16;
  pHandle->_Super.CntPhA = ( uint16_t )( val );

  /* TIM1 Channel 1 Duty Cycle configuration.
   * In RL Detection mode only the Up-side device of Phase A are controlled*/
  LL_TIM_OC_SetCompareCH1(TIMx, ( uint32_t )pHandle->_Super.CntPhA);

  /* set the sector that correspond to Phase A and B sampling */
  pHdl->Sector = SECTOR_4;

  /* Limit for update event */
  /* Check the status flag. If an update event has occurred before to set new
  values of regs the FOC rate is too high */
  if (((TIMx->CR2) & TIM_CR2_MMS_Msk) != LL_TIM_TRGO_RESET )
  //if (LL_TIM_CC_IsEnabledChannel(TIMx, LL_TIM_CHANNEL_CH4))
  {
    hAux = MC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  if ( pHandle->_Super.SWerror == 1u )
  {
    hAux = MC_DURATION;
    pHandle->_Super.SWerror = 0u;
  }
  return hAux;
}

/*
  * @brief  Computes and stores into @p pHdl latest converted motor phase currents
  *         during RL detection phase.
  *
  * Specific for Motor Profiling using Isolated Current Sensors.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  pStator_Currents: Pointer to the structure that will receive motor current
  *         of phase A and B in ab_t format.
  */
static void ICS_RLGetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * pStator_Currents )
{
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  int32_t wAux;

  /* disable ADC trigger source */
  //LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH4 );
  LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);

  wAux = (int32_t)( pHandle->pParams_str->ADCx_2->JDR1 ) - (int32_t)( pHandle->PhaseBOffset );

  /* Check saturation */
  if ( wAux > -INT16_MAX )
  {
    if ( wAux < INT16_MAX )
    {
    }
    else
    {
      wAux = INT16_MAX;
    }
  }
  else
  {
    wAux = -INT16_MAX;
  }

  pStator_Currents->a = (int16_t)wAux;
  pStator_Currents->b = (int16_t)wAux;
}

/*
  * @brief  Turns on low sides switches.
  * 
  * This function is intended to be used for charging boot capacitors
  * of driving section. It has to be called at each motor start-up when
  * using high voltage drivers.
  * Specific for Motor Profiling using Isolated Current Sensors.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  ticks: Timer ticks value to be applied.
  *                Min value: 0 (low sides ON)
  *                Max value: PWM_PERIOD_CYCLES/2 (low sides OFF)
  */
static void ICS_RLTurnOnLowSides( PWMC_Handle_t * pHdl, uint32_t ticks )
{
  (void)ticks; /* parameter not used */
  
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /*Turn on the phase A low side switch */
  LL_TIM_OC_SetCompareCH1 ( TIMx, 0u );

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* Wait until next update */
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs( TIMx );

  if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
    LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
    LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
  }
  return;
}


/*
  * @brief  Enables PWM generation on the proper Timer peripheral.
  * 
  * Specific for Motor Profiling using Isolated Current Sensors.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
static void ICS_RLSwitchOnPWM( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE( TIMx );
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  LL_TIM_OC_SetCompareCH1( TIMx, 1u );
  LL_TIM_OC_SetCompareCH4( TIMx, ( pHandle->Half_PWMPeriod ) - 5u );

  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  LL_TIM_ClearFlag_UPDATE( TIMx );
  

  
  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE ;
  LL_TIM_EnableAllOutputs( TIMx );

  if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
  {
    if ( ( TIMx->CCER & TIMxCCER_MASK_CH123 ) != 0u )
    {
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
    }
  }
  
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* enable TIMx update interrupt*/
  LL_TIM_EnableIT_UPDATE( TIMx );

  return;
}

/*
  * @brief  Turns on low sides switches and start ADC triggering.
  * 
  * Specific for Motor Profiling using Isolated Current Sensors.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
void ICS_RLTurnOnLowSidesAndStart( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif
  PWMC_ICS_Handle_t * pHandle = ( PWMC_ICS_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCx_1;
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCx_2;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  LL_TIM_OC_SetCompareCH1( TIMx, 0x0u );
  LL_TIM_OC_SetCompareCH2( TIMx, 0x0u );
  LL_TIM_OC_SetCompareCH3( TIMx, 0x0u );
  LL_TIM_OC_SetCompareCH4( TIMx, ( pHandle->Half_PWMPeriod - 5u));

  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}

  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE ;
  LL_TIM_EnableAllOutputs ( TIMx );

  if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
  {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
  }

  pHdl->Sector = SECTOR_4;

  LL_ADC_INJ_StartConversion( ADCx_1 );
  LL_ADC_INJ_StartConversion( ADCx_2 );

  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH4);

  return;
}

/**
* @}
*/

/**
* @}
*/

/**
 * @}
 */

/************************ (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
