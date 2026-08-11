# センサレスFOCトルク制御

## 前提

- 停止・低速では観測器が位置を推定できない。
- 0rpmでのトルク保持はできない。
- 無負荷トルクモードでは5000rpm付近まで加速し得る。
- RUN到達後にトルクモードへ切り替え、3500rpmの速度キャップを設ける。

## `USER CODE BEGIN Includes`

```c
#include "mc_api.h"
```

## `USER CODE BEGIN PD`

```c
/* Goal2: Sensorless torque test */
#define IQ_TEST_S16A      1000  /* 初回は約1A相当から */
#define IQ_MAX_S16A       6000  /* 約5.6A */
#define TORQUE_RAMP_MS    1000
#define OVERSPEED_RPM     3500
#define CAP_HYST_RPM       300
#define REVERSE_GUARD_RPM  (-50)
#define STARTUP_DELAY_MS   200

#ifndef RPM_TO_SPEED_UNIT
#define RPM_TO_SPEED_UNIT(rpm) ((int16_t)(((rpm) * SPEED_UNIT) / U_RPM))
#endif
```

## `USER CODE BEGIN PV`

```c
static bool torque_mode = false;
static bool speed_capped = false;
static int16_t last_iq = 0;
```

## `USER CODE BEGIN 2`

```c
/* Goal2: Fixed startup sequence */
MC_AcknowledgeFaultMotor1();
HAL_Delay(STARTUP_DELAY_MS);
MC_StartMotor1();
```

## `USER CODE BEGIN 3`

```c
/* Goal2: Switch to torque mode only after observer lock and RUN */
if (!torque_mode && MC_GetSTMStateMotor1() == RUN) {
    MC_ProgramTorqueRampMotor1(IQ_TEST_S16A, TORQUE_RAMP_MS);
    last_iq = IQ_TEST_S16A;
    torque_mode = true;
}

/* Goal3: Torque cut with hysteresis */
if (torque_mode) {
    int16_t speed = MC_GetMecSpeedAverageMotor1();
    bool speed_ok = MC_GetSpeedSensorReliabilityMotor1() &&
                    speed >= RPM_TO_SPEED_UNIT(REVERSE_GUARD_RPM);

    if (!speed_ok) {
        speed_capped = true;
    } else if (!speed_capped && speed >= RPM_TO_SPEED_UNIT(OVERSPEED_RPM)) {
        speed_capped = true;
    } else if (speed_capped &&
               speed <= RPM_TO_SPEED_UNIT(OVERSPEED_RPM - CAP_HYST_RPM)) {
        speed_capped = false;
    }

    int16_t requested_iq = (!speed_ok || speed_capped) ? 0 : IQ_TEST_S16A;
    if (requested_iq != last_iq) {
        MC_ProgramTorqueRampMotor1(requested_iq,
                                   requested_iq == 0 ? 50 : TORQUE_RAMP_MS);
        last_iq = requested_iq;
    }
}

HAL_Delay(5);
```

実機では速度推定の信頼性と逆回転も監視する。関数名は生成済み `mc_api.h` で確認する。

## 試験順序

1. 無負荷で速度キャップの作動を先に確認する。
2. `IQ_TEST_S16A` は1000程度から開始する。
3. Motor PilotでIqと回転数を監視する。
4. Over Current、Speed Feedback、Under Voltageの有無を記録する。
5. 外力を加える試験では緊急停止手段を準備する。
