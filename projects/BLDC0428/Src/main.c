/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "regular_conversion_manager.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// [REFACTOR] モーター「57DMWH75-2440 (ZGC製)」の実機スペックに基づく定数定義
#define MOTOR_POLE_PAIRS        2       // 極対数 (POLE_PAIRS)
#define MOTOR_MAX_RPM           4000.0f // 定格回転数上限 (rpm)
#define MOTOR_RATED_CURRENT_A   7.0f    // 定格電流(最大) 7A未満
#define BATTERY_NOMINAL_VOLTAGE 24.0f   // バッテリ定格電圧 24V (14Ah)
#define BATTERY_LOW_VOLTAGE     21.0f   // 過放電警告閾値の目安 (24V系で約21V)

// 実行するゴールを選択 (1, 2, 3, 4 のいずれかを指定してください)
// 1: ホールセンサ計算, 2: GPIOポーリングRPM切替, 3: EXTIボタン制御, 4: CN6可変抵抗制御
#define ACTIVE_GOAL 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
#if ACTIVE_GOAL == 4
RegConv_t potConv; // Goal 4: 可変抵抗読み取り用のRCM構造体
#endif
/* USER CODE END PV */
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
#if ACTIVE_GOAL == 1
int16_t Get_Hall_ElectricalAngle(void);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_MotorControl_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
#if ACTIVE_GOAL == 2
  // モーターを初期速度（デフォルト）で起動
  MC_StartMotor1();
#elif ACTIVE_GOAL == 3
  // 初期状態では停止のまま待機します（ボタン割り込みで起動するため）
#elif ACTIVE_GOAL == 4
  /* Goal4: CN6 Potentiometer Speed Control */
  // RCM (Regular Conversion Manager) に可変抵抗(PA4 = ADC1_IN9)を登録
  potConv.regADC = ADC1;
  potConv.channel = MC_ADC_CHANNEL_9;
  potConv.samplingTime = LL_ADC_SAMPLINGTIME_92CYCLES_5; // 十分なサンプリング時間を確保
  RCM_RegisterRegConv(&potConv);
  
  // 初期状態としてモーターを停止状態で待機
  MC_StopMotor1();
#else
  // モーターを起動します（開始直後はデフォルト設定速度でRev-Upします）
  MC_StartMotor1();

  // 起動後、1000ms（1秒）かけて 3000 RPM まで加速する指令を出す
  // ※ RPM_2_SPEED_UNIT() マクロで RPM -> 内部単位(0.1Hz) に自動変換されます
  MC_ProgramSpeedRampMotor1(RPM_2_SPEED_UNIT(3000), 1000);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if ACTIVE_GOAL == 2
    // スイッチ（PC13: Start_Stop_Pin）のポーリングによるRPM切替処理
    static uint8_t target_rpm_idx = 0;
    // [REFACTOR] 上限4000rpmに基づき目標RPMの配列を変更（最大値を MOTOR_MAX_RPM に設定）
    static const float target_rpms[] = {1500.0f, 2000.0f, 3000.0f, MOTOR_MAX_RPM}; // 段階的な目標RPMの配列
    static uint8_t prev_button_state = GPIO_PIN_SET; // PC13は通常Pull-upでHIGH、押下時にLOW

    // [REFACTOR] 定格電流7A未満に基づく過電流等の保護 (FAULT_NOW監視)
    if (MC_GetSTMStateMotor1() == FAULT_NOW)
    {
      // 異常発生時（過電流・過電圧・脱調など）は安全のため速やかに状態をリセット
      // ※MCSDK内部で自動的にハードウェアPWM出力停止が働きます
      target_rpm_idx = 0; 
    }

    uint8_t current_button_state = HAL_GPIO_ReadPin(Start_Stop_GPIO_Port, Start_Stop_Pin);

    // 押された瞬間 (HIGH -> LOW) を検出するエッジ判定
    if (prev_button_state == GPIO_PIN_SET && current_button_state == GPIO_PIN_RESET)
    {
      // インデックスを進めて次のRPM目標値を選択
      target_rpm_idx++;
      if (target_rpm_idx >= 4)
      {
        target_rpm_idx = 0;
      }

      // [REFACTOR] 上限ガード処理（安全確保のため 4000rpm 超えを制限）
      float safe_target_rpm = target_rpms[target_rpm_idx];
      if (safe_target_rpm > MOTOR_MAX_RPM)
      {
        safe_target_rpm = MOTOR_MAX_RPM;
      }

      // 選択したRPMへ、2000ms（2秒）かけて加速・減速する指令
      MC_ProgramSpeedRampMotor1_F(safe_target_rpm, 2000);

      // 簡易的なチャタリング防止のためのウェイト
      HAL_Delay(50);
    }

    prev_button_state = current_button_state;

    // 長いHAL_Delayを入れるとポーリングが反応しなくなるため、ここでは入れません。

#elif ACTIVE_GOAL == 3
    // メインループ処理（モーター駆動制御は割り込みで行われるため空でOK）

#elif ACTIVE_GOAL == 4
    /* Goal 4: CN6 Potentiometer Speed Control (Architecture Rev.1) */
    
    /* ユーザー定義変数初期化 */
    static uint32_t adc_raw = 0;
    static uint32_t adc_filtered = 0;
    int16_t target_rpm = 0;
    static int16_t current_rpm_cmd = 0;
    const int16_t MAX_RPM = 3000;
    const int16_t MIN_RPM = 500;
    const uint16_t RAMP_TIME_MS = 250; /* 250msのスムーズな推移 */
    const int16_t RPM_DEADBAND = 40;   /* 40RPMの不感帯 */

    /* 1. ADC値のサンプリング 
     * ※ドキュメントではHAL_ADC_Start(&hadc1)を使用していますが、
     *   MCSDK(FOC)のADC同期シーケンスとの致命的な競合を避けるため、
     *   MCSDK標準の安全なバックグラウンド読み取りAPI(RCM)を使用します。
     */
    adc_raw = RCM_ExecRegularConv(&potConv);
    
    /* 2. デジタルローパスフィルタの適用 (ビットシフトによる高速化) */
    adc_filtered = (adc_filtered * 7 + adc_raw) >> 3; 

    /* 3. RPMへのスケーリング (線形マッピング) */
    target_rpm = ((adc_filtered * (MAX_RPM - MIN_RPM)) / 4095) + MIN_RPM;

    /* 4. モータ状態の監視と指令値の更新 */
    if (MC_GetSTMStateMotor1() == RUN) {
        // 速度変化がデッドバンドを超えた場合のみAPIを発行 (abs関数の代用)
        int16_t diff = target_rpm - current_rpm_cmd;
        if (diff < 0) diff = -diff;

        if (diff > RPM_DEADBAND) {
            /* 速度変化がデッドバンドを超えた場合のみAPIを発行 */
            MC_ProgramSpeedRampMotor1_F((float)target_rpm, RAMP_TIME_MS);
            current_rpm_cmd = target_rpm;
        }
    } else if (MC_GetSTMStateMotor1() == FAULT_OVER || MC_GetSTMStateMotor1() == FAULT_NOW) {
        /* 手でローターを拘束するなどしてエラーが発生した場合のクリア処理 */
        // MC_AcknowledgeFaultMotor1(); などの障害復帰ロジックを実装可能
    } else if (MC_GetSTMStateMotor1() == IDLE) {
        /* 停止状態からの起動 (安全のため自動起動とするか手動とするか任意) */
        // 今回のシステムは単独稼働のため、安全を確保した上で自動起動させておく
        // MC_StartMotor1();
    }

    /* デバッグ用にUARTで現在のADC値・目標RPMを出力 */
    char msg[64];
    snprintf(msg, sizeof(msg), "ADC: %4lu | Target RPM: %4d\r\n", adc_filtered, current_rpm_cmd);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    
    HAL_Delay(50); /* サンプリングレートの制御 */

#else
    // 5秒間 3000 RPM で回転を維持
    HAL_Delay(5000);

    // 2000ms（2秒）かけて 1500 RPM まで減速する
    // ※ float版のAPIを使う場合は直接RPMを指定できます
    MC_ProgramSpeedRampMotor1_F(1500.0f, 2000);

    // 5秒間 1500 RPM で回転を維持
    HAL_Delay(5000);

    // 再び 2000ms（2秒）かけて 3000 RPM まで加速する
    MC_ProgramSpeedRampMotor1(RPM_2_SPEED_UNIT(3000), 2000);
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 3, 1);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* TIM1_BRK_TIM15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 4, 1);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  /* TIM1_UP_TIM16_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Disable Injected Queue
  */
  HAL_ADCEx_DisableInjectedQueue(&hadc1);

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_5;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Disable Injected Queue
  */
  HAL_ADCEx_DisableInjectedQueue(&hadc2);

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = ((PWM_PERIOD_CYCLES) / 2);
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = (REP_COUNTER);
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_BKIN;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_LOW;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK2, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = ((PWM_PERIOD_CYCLES) / 4);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = (((PWM_PERIOD_CYCLES) / 2) - (HTMIN));
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = ((DEAD_TIME_COUNTS) / 2);
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_ENABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 4;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : Start_Stop_Pin */
  GPIO_InitStruct.Pin = Start_Stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Start_Stop_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // [REFACTOR] ホールセンサー入力ピンの初期化
  // 警告: PA0, PA1, PA2はADCやUARTで使用済みのため衝突します。
  // 代わりに空いているArduino互換ピン(D4, D5, D6)を使用します。
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

#if ACTIVE_GOAL == 4
  /* Goal4: CN6 Potentiometer Speed Control */
  // X-NUCLEO-IHM08M1のCN6 (Potentiometer) が繋がるA2ピン(PA4)をADC入力として初期化
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#if ACTIVE_GOAL == 1

/**
 * @brief  U/V/W のホールセンサ信号（GPIO）を読み取り、ロータの電気角(dpp)を計算するサンプル
 * @note   FOCのパーク変換に渡すためには、この関数を mc_tasks_foc.c の高周波タスク内
 *         (USER CODE BEGIN HighFrequencyTask 0 等) から呼び出し、FOCVars[M1].hElAngle
 *         を上書きする必要があります。
 *         ※ 注意: MCSDKの標準アーキテクチャでは、ST Motor Control Workbench上で
 *            センサをHallに設定しコード再生成することが推奨されます。
 * @retval int16_t ロータ電気角 (DDPフォーマット: s16degrees)
 */
int16_t Get_Hall_ElectricalAngle(void)
{
  // [REFACTOR] ホールセンサーのピン割り当てを衝突のないピンに変更
  // Hall A (青) = D4 ピン (PB5)
  // Hall B (緑) = D5 ピン (PB4)
  // Hall C (白) = D6 ピン (PB10)
  // 結線: U=赤, V=黄, W=黒
  uint8_t hall_a = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);  // Hall A (青)
  uint8_t hall_b = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);  // Hall B (緑)
  uint8_t hall_c = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10); // Hall C (白)

  uint8_t hall_state = (hall_a << 2) | (hall_b << 1) | hall_c;
  int16_t electrical_angle = 0;

  /* 
   * [REFACTOR] 極対数 (MOTOR_POLE_PAIRS = 2) に対応した電気角の決定
   * ホールセンサの状態（1〜6）から電気角(s16degreesフォーマット: -32768〜32767)を決定する
   * ※機械角180度が電気角360度に相当します。このテーブルは電気角を出力します。
   */
  switch (hall_state)
  {
    case 5: electrical_angle = (int16_t)(-32768); break;                 // 0度 (-180度相当)
    case 1: electrical_angle = (int16_t)(-32768 + 65536 / 6 * 1); break; // 60度
    case 3: electrical_angle = (int16_t)(-32768 + 65536 / 6 * 2); break; // 120度
    case 2: electrical_angle = (int16_t)(-32768 + 65536 / 6 * 3); break; // 180度
    case 6: electrical_angle = (int16_t)(-32768 + 65536 / 6 * 4); break; // 240度
    case 4: electrical_angle = (int16_t)(-32768 + 65536 / 6 * 5); break; // 300度
    default: electrical_angle = 0; break; // 異常時
  }

  return electrical_angle;
}

#elif ACTIVE_GOAL == 3

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == Start_Stop_Pin) // PC13 (USER BUTTON)
  {
    /* 
     * TODO: 将来拡張への対応
     * 現在はON/OFFのトグルですが、将来的には以下のように
     * 「トルク指令モード」と「速度指令モード」を切り替えるフラグ操作に変更する設計です。
     * 
     * static uint8_t control_mode = 0; // 0:Speed, 1:Torque
     * control_mode = !control_mode;
     * // モードに応じた制御を別途実行
     */

    // 現在のモーターステートを取得
    MCI_State_t state = MC_GetSTMStateMotor1();

    // [REFACTOR] 定格電流7A未満に基づく過電流・脱調等の保護 (FAULT_NOW監視)
    if (state == FAULT_NOW || state == FAULT_OVER)
    {
      // 異常発生時（過電流・過電圧・脱調など）は安全のためコマンドを受け付けない
      // ※MCSDK内部で自動的にハードウェアPWM出力停止が働きます
      return;
    }

    // モーターが停止（IDLE）状態なら起動、駆動状態なら停止
    if (state == IDLE)
    {
      // [REFACTOR] 起動前の目標速度設定と上限ガード（4000rpm超えを制限）
      float target_rpm = 3000.0f;
      if (target_rpm > MOTOR_MAX_RPM)
      {
        target_rpm = MOTOR_MAX_RPM;
      }

      // 起動前に目標速度(例: 3000RPMに1000msでランプ)を設定しておく
      MC_ProgramSpeedRampMotor1_F(target_rpm, 1000); 
      MC_StartMotor1();
    }
    else if (state == RUN || state == START)
    {
      // 停止指令
      MC_StopMotor1();
    }
  }
}

#endif
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
