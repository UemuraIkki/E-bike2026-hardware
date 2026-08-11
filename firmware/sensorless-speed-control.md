# センサレスFOC速度制御

対象はMCSDK 6.4.2で生成した `main.c`。まず速度制御で確実に回し、Motor Pilotで電流換算を確認してからトルク制御へ進む。

## `USER CODE BEGIN Includes`

```c
#include "mc_api.h"
```

## `USER CODE BEGIN PD`

```c
/* Goal1: Sensorless FOC speed test */
#define TARGET_RPM        1200
#define SPEED_RAMP_MS     2000
#define STARTUP_DELAY_MS  200

#ifndef RPM_TO_SPEED_UNIT
#define RPM_TO_SPEED_UNIT(rpm) ((int16_t)(((rpm) * SPEED_UNIT) / U_RPM))
#endif
```

このSDKには `RPM_2_SPEED_UNIT` が存在しないため使用しない。

## `USER CODE BEGIN 2`

```c
/* Goal1: Fixed startup sequence */
MC_AcknowledgeFaultMotor1();
HAL_Delay(STARTUP_DELAY_MS);
MC_StartMotor1();
```

## `USER CODE BEGIN 3`

```c
/* Goal1: Program speed after RUN */
static bool speed_programmed = false;

if (!speed_programmed && MC_GetSTMStateMotor1() == RUN) {
    MC_ProgramSpeedRampMotor1(RPM_TO_SPEED_UNIT(TARGET_RPM), SPEED_RAMP_MS);
    speed_programmed = true;
}

HAL_Delay(10);
```

## 試験条件

- ベンチ電源の電流制限を3A以上にする。
- DebugはSWDとする。
- 800〜1500rpm程度から開始する。
- Motor PilotはST-Link VCP、1843200baudで接続する。
- `Iq / 1068 ≈ 電流[A]` を実電流と比較する。

API名は生成済み `mc_api.h` で確認する。
