
/**
  ******************************************************************************
  * @file    parameters_conversion_l4xx.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the definitions needed to convert MC SDK parameters
  *          so as to target the STM32F3 Family.
  *
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
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMETERS_CONVERSION_L4XX_H
#define __PARAMETERS_CONVERSION_L4XX_H

#include "pmsm_motor_parameters.h"
#include "drive_parameters.h"
#include "power_stage_parameters.h"
#include "mc_math.h"

/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/
#define SYSCLK_FREQ                      80000000uL
#define TIM_CLOCK_DIVIDER                1
#define ADV_TIM_CLK_MHz                  80
#define ADC_CLK_MHz                      80
#define HALL_TIM_CLK                     80000000uL

/*************************  IRQ Handler Mapping  *********************/
#define TIMx_UP_M1_IRQHandler            TIM1_UP_TIM16_IRQHandler

#define TIMx_BRK_M1_IRQHandler           TIM1_BRK_TIM15_IRQHandler

/*************************  ADC Physical characteristics  ************/
#define ADC_TRIG_CONV_LATENCY_CYCLES     3.5
#define ADC_SAR_CYCLES                   12.5
#define M1_VBUS_SW_FILTER_BW_FACTOR      6u

#endif /*__PARAMETERS_CONVERSION_L4XX_H*/

/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
