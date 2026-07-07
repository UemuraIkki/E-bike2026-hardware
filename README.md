# ST Workbench Projects

このリポジトリは、STM32 ST Workbench (X-CUBE-MCSDK) を用いたモータ制御開発の記録です。
内容を以下の3部門に分けて管理します。

- **モータ部門** — FOC/センサレス制御などモータ駆動そのものに関する開発
- **ホールセンサ部門** — ホールセンサを用いた位置/速度検出に関する開発
- **トルクセンサ部門** — トルクセンサを用いた検出・制御に関する開発

現時点ではモータ部門のみ記載しています。他部門は追って追記します。

---

## モータ部門

### 現状 (最新: BLDC0616)

[projects/BLDC0616](projects/BLDC0616) が最新のモータ制御プロジェクトです。STM32L476RGTx +
MCSDK v6.4.2-Full をベースに、**センサレスFOC制御**でモータを駆動しています。

| 項目 | 内容 |
|---|---|
| MCU | STM32L476RGTx (LQFP64) |
| MCSDK | v6.4.2-Full |
| 制御方式 | FOC (Field Oriented Control) |
| 位置/速度推定 | センサレス — State Observer + PLL (`STO_PLL_M1`) |
| 電流検出 | 三相シャント (THREE_SHUNT, Rshunt = 0.01Ω) |
| PWM周波数 | 16 kHz |
| 対象モータ | `56JXE` (E-bike用 SM-PMSM, 2 pole pairs, 定格電流 14.175A, 最大回転数 4998rpm) |
| 起動方式 | Start/Stopボタン (PC13, EXTI) → `MC_StartMotor1()` → RUN到達後トルクランプ制御 |
| 保護 | オーバースピードキャップ (4000rpm)、FAULTステートでの自動再起動 |

制御パラメータは [BLDC0616/Inc/drive_parameters.h](projects/BLDC0616/Inc/drive_parameters.h)、
モータ電気定数は [BLDC0616/Inc/pmsm_motor_parameters.h](projects/BLDC0616/Inc/pmsm_motor_parameters.h)
に定義されています。アプリケーションロジック(起動シーケンス、トルクランプ、オーバースピード監視)は
[BLDC0616/Src/main.c](projects/BLDC0616/Src/main.c) に実装されています。

モータの物理定数(定格電流・BEMF定数・慣性など)は [hardware/motor/56JXE.json](hardware/motor/56JXE.json)
としてST Workbench用に切り出して管理しています。

### なぜセンサレス制御なのか

当初は搭載モータのホールセンサ(3相中のHallB)を使った120°通電/ホールFOC制御を狙っていましたが、
**HallB信号が実際には出力されていない(センサ故障)** ことが判明しました。この故障は開発初期には気づけず、
ホール信号を使った位置推定が安定しない原因の切り分けに時間を要しました。

最終的に「HallBが物理的に死んでいる」ことを特定し、ホールセンサに依存しない
**State Observer + PLL によるセンサレスBEMF推定**へ制御方式を切り替えることで、
このモータでも安定した速度/位置推定とFOC駆動を実現しています。

ホールセンサを使った制御の実装・検証自体は [projects/BLDC0512](projects/BLDC0512) 系列
(`hall_speed_pos_fdbk.c/.h`, TIM2 Hall入力など)に残しており、詳細はホールセンサ部門で扱います。

### 経緯 (BLDC05xx → BLDC0616)

プロジェクトは `BLDCMMDD` (月日) の命名で日々のイテレーションを重ねています。モータ部門に関連する
主な変遷は以下の通りです。

1. **BLDC0512 系列** — Potentiometer(可変抵抗)によるトルコ指令から、Step-Switching + LED点滅への置き換え、
   ボタン割り込みハンドラの修正 (MCSDK weak関数の上書き)、FAULT状態のハンドリングと診断出力の追加、
   単一の固定目標RPM + ON/OFFトグルへのリファクタなど、Hall入力(TIM2)を使う前提で機能を積み上げました。
   その過程でHallBの異常に気づき、手動Hall初期化コードを一旦作り込んだ後、
   ST Workbench側がTIM2 Hallセンサの初期化を正しく生成できるようになったため、手動実装を撤去しています。
2. **BLDC0616** — Hall依存をやめ、センサレス(STO+PLL)FOCを前提とした新規プロジェクトとして構成。
   起動→トルクモード→オーバースピード監視のシンプルな状態機械で駆動を確認する段階です。

### 既知の課題: チェーンにブレーキ/抵抗を掛けると逆回転方向に暴走する

**現象**: 駆動中のチェーン(ドライブトレイン)側にブレーキや抵抗を掛けると、モータが逆回転方向に
急激に高速回転し始める。

**考えられる原因**

このモータはBEMF(逆起電力)を利用したセンサレス位置/速度推定 (`STO_PLL_M1`) で駆動しており、
外部からチェーン経由で急激なトルク変化(ブレーキ/抵抗)を与えられると、以下のように推定と実回転が
ズレることで発生していると考えられる。

1. **推定角度と実回転子角度の同期外れ**
   State Observer + PLLは回転速度・BEMFの連続性を前提にしており、外部から急に回転数を変化させられると
   速度推定の分散(variance)が急増し、推定角度が実際の回転子角度から外れる
   ([sto_pll_speed_pos_fdbk.c:426-437](projects/BLDC0616/MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/Any/Src/sto_pll_speed_pos_fdbk.c#L426-L437))。
2. **フォールト検知までのタイムラグ**
   速度推定/BEMF整合性の異常は `ReliabilityCounter` としてカウントされるが、実際にフォールト
   (`bSpeedErrorNumber`)として確定するまでには
   [`M1_SS_MEAS_ERRORS_BEFORE_FAULTS = 100`](projects/BLDC0616/Inc/drive_parameters.h#L36)
   回連続の異常を要する設計になっている
   ([sto_pll_speed_pos_fdbk.c:511-524](projects/BLDC0616/MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/Any/Src/sto_pll_speed_pos_fdbk.c#L511-L524))。
   このため、角度推定がズレ始めてからフォールト停止するまでの間、ズレた角度のままFOC電流指令が
   投入され続ける猶予期間が存在する。
3. **アプリ側に逆回転/信頼性低下を監視するロジックが無い**
   [main.c](projects/BLDC0616/Src/main.c#L149-L157) のトルクモード制御 (state 2) は、
   順回転方向のオーバースピード (`spd > 4000rpm`) のみを監視しており、回転方向の反転や
   速度推定の信頼性低下 (`IsSpeedReliable` / `IsBemfConsistent`) を見て `Iq` 指令を落とす、
   といった保護が実装されていない。ズレた推定角度に対して固定のIq指令を投入し続けると、
   実回転子から見て意図しない方向にトルクが発生し得るため、ブレーキ/抵抗をきっかけに
   「逆回転方向へ加速し続ける」ような挙動として現れていると推測される。

センサレスBEMF方式は原理的に、低速域や外部からの強制回転(オーバーホール/回生的な負荷)に弱いという
既知の制約があり、今回の現象もその弱点が顕在化したものと考えられる。

**対策**

- [x] `main.c` のトルク制御ループに、回転方向の反転や `STO` の信頼性低下を検知した際に即座に
  `MC_StopMotor1()` を呼んで再起動シーケンスへ戻す、というガードを実装済み
  ([main.c:151-166](projects/BLDC0616/Src/main.c#L151-L166))。
  `MC_GetSpeedSensorReliabilityMotor1()` が false、または推定速度が `REVERSE_GUARD_RPM`
  (-50rpm)を下回った時点でトルク指令を打ち切り、`state=0` から再アライン/再起動する。
- [ ] `M1_SS_MEAS_ERRORS_BEFORE_FAULTS` を下げてフォールト確定までのタイムラグを縮める
  (誤フォールトとのトレードオフに注意)。
- [ ] 低速域 (`OBS_MINIMUM_SPEED_RPM` 付近以下)では推定信頼性が下がりやすいため、
  この領域でのトルク指令を強制的に絞る/停止するロジックを追加する。
- [ ] チェーン側からの急激な負荷変動を常時受ける用途であれば、センサレスのみに頼らず、
  ホールセンサ(修理・交換後)やエンコーダによる位置フィードバックの併用を検討する
  (ホールセンサ部門で対応予定)。

---

*(以下、ホールセンサ部門・トルクセンサ部門は追記予定)*
