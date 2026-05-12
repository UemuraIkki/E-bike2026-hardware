/**
  ******************************************************************************
  * @file    ics_l4xx_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          ICS PWM current feedback component for F30x of the Motor Control SDK.
  ******************************************************************************
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
  * @ingroup ICS_L4XX_pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ICS_L4XX_PWMCURRFDBK_H
#define __ICS_L4XX_PWMCURRFDBK_H

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

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


#define NONE    ((uint8_t)(0x00))
#define EXT_MODE  ((uint8_t)(0x01))
#define INT_MODE  ((uint8_t)(0x02))
#define SOFOC 0x0008u /* This flag is reset at the beginning of FOC
                           and it is set in the TIM UP IRQ. If at the end of
                           FOC this flag is set, it means that FOC rate is too 
                           high and thus an error is generated */


/*
  * @brief  Current feedback component parameters structure definition for ICS configuration.
  */
typedef const struct
{
  /* HW IP involved ------------------------------------------------------------- */
  ADC_TypeDef * ADCx_1;                   /* First ADC peripheral to be used. */
  ADC_TypeDef * ADCx_2;                   /* Second ADC peripheral to be used. */
  TIM_TypeDef * TIMx;                    /* Timer used for PWM generation. */
  
  /* Currents sampling parameters ----------------------------------------------- */
  uint32_t ADCConfig1;                   /* Value for ADC CR2 to properly configure
                                             current sampling during the context switching. 
                                             Either defined in PWMC_ICS_Handle_t or in 
                                             ICS_Params_t. */
  uint32_t ADCConfig2;                   /* Value for ADC CR2 to properly configure
                                             current sampling during the context switching. 
                                             Either defined in PWMC_ICS_Handle_t or in 
                                             ICS_Params_t. */

  /* PWM generation parameters -------------------------------------------------- */
  uint8_t  RepetitionCounter;            /* Expresses the number of PWM
                                            periods to be elapsed before compare
                                            registers are updated again.
                                            In particular:
                                            RepetitionCounter = (2 * PWM periods) - 1 */

} ICS_Params_t;


/*
  * @brief  PWM and Current Feedback ICS handle.
  */
typedef struct
{
  PWMC_Handle_t _Super;                 /* Base component handler. */
  uint32_t PhaseAOffset;                /* Offset of Phase A current sensing network. */
  uint32_t PhaseBOffset;                /* Offset of Phase B current sensing network. */
  uint16_t Half_PWMPeriod;              /* Half PWM Period in timer clock counts. */
  volatile uint8_t PolarizationCounter; /* Number of conversions performed during the calibration phase. */

  ICS_Params_t * pParams_str;
} PWMC_ICS_Handle_t;

/* Exported functions ------------------------------------------------------- */

/*
  * Initializes TIMx, ADC, GPIO and NVIC for current reading
  * with ICS configurations.
  */
void ICS_Init( PWMC_ICS_Handle_t * pHandle );

/*
  * Sums up injected conversion data into wPhaseXOffset.
  */
void ICS_CurrentReadingCalibration( PWMC_Handle_t * pHandle );

/*
  * Computes and stores in the handler the latest converted motor phase currents in ab_t format.
  */
void ICS_GetPhaseCurrents( PWMC_Handle_t * pHandle, ab_t * pStator_Currents );

/*
  * Turns on low sides switches.
  */
void ICS_TurnOnLowSides( PWMC_Handle_t * pHandle, uint32_t ticks );

/*
  * Enables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void ICS_SwitchOnPWM( PWMC_Handle_t * pHandle );

/*
  * Disables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void ICS_SwitchOffPWM( PWMC_Handle_t * pHandle );

/*
  * Writes into peripheral registers the new duty cycle and sampling point.
  */
uint16_t ICS_WriteTIMRegisters( PWMC_Handle_t * pHandle );

/*
  * Sums up injected conversion data into wPhaseXOffset.
  */
void ICS_HFCurrentsCalibration( PWMC_Handle_t * pHandle, ab_t * pStator_Currents );

/*
  * Contains the TIMx Update event interrupt.
  */
void * ICS_TIMx_UP_IRQHandler( PWMC_ICS_Handle_t * pHandle );

/*
  * Contains the TIMx Break1 event interrupt.
  */
void * ICS_BRK_IRQHandler( PWMC_ICS_Handle_t * pHandle );

/*
  * Contains the TIMx Break2 event interrupt.
  */
void * ICS_BRK2_IRQHandler( PWMC_ICS_Handle_t * pHandle );

/*
  * Stores in the handler the calibrated offsets.
  */
void ICS_SetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

/*
  * Reads the calibrated offsets stored in the handler.
  */
void ICS_GetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

/*
  * Sets the PWM mode during RL Detection Mode.
  */
void ICS_RLDetectionModeEnable( PWMC_Handle_t * pHdl );

/*
  * Disables the PWM mode during RL Detection Mode.
  */
void ICS_RLDetectionModeDisable( PWMC_Handle_t * pHdl );

/*
  * Set the PWM dutycycle during RL Detection Mode.
  */
uint16_t ICS_RLDetectionModeSetDuty( PWMC_Handle_t * pHdl, uint16_t hDuty );

/*
  * Turns on low sides switches and start ADC triggering.
  */
void ICS_RLTurnOnLowSidesAndStart( PWMC_Handle_t * pHdl );

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__ICS_L4XX_PWMCURRFDBK_H*/

/************************ (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
