# main.c USER CODE archive

This file preserves non-empty `USER CODE` blocks from each experimental `main.c` before generated MCSDK files are removed.
Values and implementations are historical test snapshots and must not be treated as the current specification.

## BLDC0421

Source before cleanup: projects/BLDC0421/Src/main.c

### USER CODE 2

```c
// モーターを起動します（開始直後はデフォルト設定速度でRev-Upします）
  MC_StartMotor1();

  // 起動後、1000ms（1秒）かけて 3000 RPM まで加速する指令を出す
  // ※ RPM_2_SPEED_UNIT() マクロで RPM -> 内部単位(0.1Hz) に自動変換されます
  MC_ProgramSpeedRampMotor1(RPM_2_SPEED_UNIT(3000), 1000);
```

### USER CODE 3

```c
// 5秒間 3000 RPM で回転を維持
    HAL_Delay(5000);

    // 2000ms（2秒）かけて 1500 RPM まで減速する
    // ※ float版のAPIを使う場合は直接RPMを指定できます
    MC_ProgramSpeedRampMotor1_F(1500.0f, 2000);

    // 5秒間 1500 RPM で回転を維持
    HAL_Delay(5000);

    // 再び 2000ms（2秒）かけて 3000 RPM まで加速する
    MC_ProgramSpeedRampMotor1(RPM_2_SPEED_UNIT(3000), 2000);

  }
```

## BLDC0428

Source before cleanup: projects/BLDC0428/Src/main.c

### USER CODE Includes

```c
#include <stdio.h>
#include <string.h>
#include "regular_conversion_manager.h"
```

### USER CODE PD

```c
// [REFACTOR] モーター「57DMWH75-2440 (ZGC製)」の実機スペックに基づく定数定義
#define MOTOR_POLE_PAIRS        2       // 極対数 (POLE_PAIRS)
#define MOTOR_MAX_RPM           4000.0f // 定格回転数上限 (rpm)
#define MOTOR_RATED_CURRENT_A   7.0f    // 定格電流(最大) 7A未満
#define BATTERY_NOMINAL_VOLTAGE 24.0f   // バッテリ定格電圧 24V (14Ah)
#define BATTERY_LOW_VOLTAGE     21.0f   // 過放電警告閾値の目安 (24V系で約21V)

// 実行するゴールを選択 (1, 2, 3, 4 のいずれかを指定してください)
// 1: ホールセンサ計算, 2: GPIOポーリングRPM切替, 3: EXTIボタン制御, 4: CN6可変抵抗制御
#define ACTIVE_GOAL 4
```

### USER CODE PV

```c
#if ACTIVE_GOAL == 4
RegConv_t potConv; // Goal 4: 可変抵抗読み取り用のRCM構造体
#endif
```

### USER CODE PFP

```c
#if ACTIVE_GOAL == 1
int16_t Get_Hall_ElectricalAngle(void);
#endif
```

### USER CODE 2

```c
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
```

### USER CODE 3

```c
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
```

### USER CODE MX_GPIO_Init_2

```c
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
```

### USER CODE 4

```c
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
```

## BLDC0512

Source before cleanup: projects/BLDC0512/Src/main.c

### USER CODE Includes

```c
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "regular_conversion_manager.h"
```

### USER CODE PD

```c
// [REFACTOR] モーター「57DMWH75-2440 (ZGC製)」の実機スペックに基づく定数定義
#define MOTOR_POLE_PAIRS        2       // 極対数 (POLE_PAIRS)
#define MOTOR_MAX_RPM           4000.0f // 定格回転数上限 (rpm)
#define MOTOR_RATED_CURRENT_A   7.0f    // 定格電流(最大) 7A未満
#define BATTERY_NOMINAL_VOLTAGE 24.0f   // バッテリ定格電圧 24V (14Ah)
#define BATTERY_LOW_VOLTAGE     21.0f   // 過放電警告閾値の目安 (24V系で約21V)
```

### USER CODE PV

```c
volatile bool is_system_active = false;
volatile bool is_fault_state = false;
volatile uint32_t last_button_press_time = 0; // チャタリング防止用

/* ここで目標回転数を設定します（コンパイル時に変更可能） */
#define DEFAULT_TARGET_RPM 3000
```

### USER CODE 2

```c
// 初期状態としてモーターを停止状態で待機
  MC_StopMotor1();
```

### USER CODE 3

```c
/* BLDC0512 統合ロジック: ボタンスイッチON/OFF + 固定目標RPM + UART出力 */

    static int16_t current_rpm_cmd = 0;
    static uint8_t led_counter = 0;
    int16_t target_rpm = is_system_active ? DEFAULT_TARGET_RPM : 0;

    /* 1. モータ状態の監視と指令値の更新 */
    MCI_State_t state = MC_GetSTMStateMotor1();

    if (state == FAULT_OVER || state == FAULT_NOW) {
        /* 異常時の安全処理 */
        current_rpm_cmd = 0;
        is_system_active = false;
        is_fault_state = true;
    }
    else if (!is_system_active) {
        /* 停止状態 */
        if (state == RUN || state == START) {
            MC_StopMotor1();
            current_rpm_cmd = 0;
        }
    }
    else if (is_fault_state) {
        /* エラー状態 (ボタンが押されてクリアされるのを待つ) */
        current_rpm_cmd = 0;
    }
    else {
        /* 稼働状態 */
        if (state == IDLE) {
            /* 停止状態から再起動 */
            MC_StartMotor1();
            current_rpm_cmd = 0;
        } else if (state == RUN) {
            /* 目標RPMが変わった場合のみAPIを発行 (2000msかけて加速・減速) */
            if (target_rpm != current_rpm_cmd || current_rpm_cmd == 0) {
                MC_ProgramSpeedRampMotor1_F((float)target_rpm, 2000);
                current_rpm_cmd = target_rpm;
            }
        }
    }

    /* 2. LEDの点滅制御 (50msループを基準) */
    led_counter++;
    if (is_fault_state) {
        // FAULT状態: 10Hz高速点滅 (50ms ON / 50ms OFF) => 1カウント毎にトグル
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    } else if (!is_system_active) {
        // OFF: 消灯
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    } else {
        // ON: 2Hz点滅 (250ms ON / 250ms OFF) => 5カウント毎にトグル
        if (led_counter % 5 == 0) HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }

    /* 3. デバッグ用にUARTで現在の状態を出力 */
    char msg[64];
    snprintf(msg, sizeof(msg), "[Sys: %s] Target: %4d | MotorState: %d\r\n",
             is_system_active ? "ON " : "OFF", target_rpm, state);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    HAL_Delay(50); /* サンプリングレートの制御 (50ms/loop) */
  }
```

### USER CODE 4

```c
/**
  * @brief  User Button (PC13) callback overriding MCSDK weak function
  * @retval None
  */
void UI_HandleStartStopButton_cb(void)
{
  /* 簡易的なチャタリング防止 (200ms以内は無視) */
  uint32_t current_time = HAL_GetTick();
  if (current_time - last_button_press_time > 200) {
    if (is_fault_state) {
      // エラー状態ならエラーをクリアして停止状態へ
      MC_AcknowledgeFaultMotor1();
      is_fault_state = false;
      is_system_active = false;
    } else {
      // ON/OFFを反転させる
      is_system_active = !is_system_active;
    }
    last_button_press_time = current_time;
  }
}
```

## BLDC0518

Source before cleanup: projects/BLDC0518/Src/main.c

### USER CODE 3

```c
}
```

## BLDC0519

Source before cleanup: projects/BLDC0519/Src/main.c

### USER CODE PD

```c
#define MAX_RPM       4000
#define MIN_RPM       500
#define RAMP_TIME_MS  500
#define RPM_TO_SPEED_UNIT(rpm) ((int16_t)(((rpm) * SPEED_UNIT) / U_RPM))
```

### USER CODE PV

```c
/* ADC変換結果格納変数 */
uint32_t adcValue = 0;
int16_t  targetRpm = 0;
int16_t  prevRpm   = 0;
```

### USER CODE 2

```c
/* ADCキャリブレーション・開始 */

  /* Fault解除・モーター起動 */
```

### USER CODE 3

```c
HAL_Delay(100);
```

## BLDC0616

Source before cleanup: projects/BLDC0616/Src/main.c

### USER CODE Includes

```c
#include "mc_api.h"
#include <stdio.h>
```

### USER CODE PD

```c
#define IQ_START_S16A   1070   // ≈1.0A（1068 s16A/A換算）まず控えめ
#define IQ_MAX_S16A     6000   // ≈5.6A 安全上限
#define OVERSPEED_RPM   4000   // まずは余裕を持って4000。後で下げてキャップ動作確認
#define REVERSE_GUARD_RPM (-50) // これより逆転側に振れたら異常とみなす閾値（ノイズ耐性のため0より少し余裕を持たせる）
#ifndef RPM_TO_SPEED_UNIT
#define RPM_TO_SPEED_UNIT(rpm) ((int16_t)(((rpm) * SPEED_UNIT) / U_RPM))
#endif
```

### USER CODE PV

```c
static uint8_t state = 0;
static int16_t lastIq = 0;
static uint32_t printTick = 0;
```

### USER CODE 3

```c
switch (state) {
	    case 0:  // 起動（1回だけ）
	      MC_AcknowledgeFaultMotor1();
	      HAL_Delay(200);
	      MC_StartMotor1();
	      state = 1;
	      break;

	    case 1:  // 閉ループRUN到達を待ってトルクモードへ
	      if (MC_GetSTMStateMotor1() == RUN) {
	          MC_ProgramTorqueRampMotor1(IQ_START_S16A, 500);
	          lastIq = IQ_START_S16A;
	          state = 2;
	      } else if (MC_GetSTMStateMotor1() == FAULT_NOW ||
	                 MC_GetSTMStateMotor1() == FAULT_OVER) {
	          HAL_Delay(500);
	          state = 0;   // フォルトなら再起動
	      }
	      break;

	    case 2: {  // トルクモード＋速度キャップ（トルク制御からは抜けない）
	      int16_t spd = MC_GetMecSpeedAverageMotor1();
	      bool speedOk = MC_GetSpeedSensorReliabilityMotor1() &&
	                     (spd >= RPM_TO_SPEED_UNIT(REVERSE_GUARD_RPM));
	      // 信頼性低下/逆回転/オーバースピードのいずれかでIqを0に。
	      // MC_StopMotor1()は呼ばずRUN・トルク制御のままにして、
	      // 条件が消えたら次ループで自動的にIQ_START_S16Aへ復帰させる。
	      int16_t iqRef = (!speedOk || (spd > RPM_TO_SPEED_UNIT(OVERSPEED_RPM))) ? 0 : IQ_START_S16A;
	      if (iqRef != lastIq) {                       // 変化時だけ再指令
	          MC_ProgramTorqueRampMotor1(iqRef, 100);
	          lastIq = iqRef;
	      }
	      break;
	    }
	  }

	  if (HAL_GetTick() - printTick >= 200) {
	      printTick = HAL_GetTick();
	      qd_f_t iqd = MC_GetIqdMotor1_F();
	      printf("Iq=%.3fA Id=%.3fA\r\n", iqd.q, iqd.d);
	  }

	  HAL_Delay(5);
  }
```

### USER CODE 4

```c
/* Retarget printf to the debugger console via semihosting (SYS_WRITEC),
 * guarded by C_DEBUGEN so it's a no-op (not a HardFault) when no debugger
 * is attached. Keeps USART2 free for MCP and avoids SWO wiring entirely. */
int __io_putchar(int ch)
{
  if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0UL)
  {
    static uint8_t c;
    c = (uint8_t)ch;
    register uint32_t r0 __asm__("r0") = 0x03; /* SYS_WRITEC */
    register void    *r1 __asm__("r1") = &c;
    __asm__ volatile ("bkpt #0xAB" : "+r"(r0) : "r"(r1) : "memory");
  }
  return ch;
}
```
