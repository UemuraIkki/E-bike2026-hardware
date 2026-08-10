# ファームウェア資料

このリポジトリはMCSDK生成物を配布しません。ローカルでプロジェクトを再生成し、必要なUSER CODEを挿入します。

## 再生成

1. ST公式サイトからX-CUBE-MCSDK 6.4.2を取得する。
2. Motor Control Workbenchで `projects/BLDC0616.stwb6` を開く。
3. モータと電流設定を [../docs/02_specifications.md](../docs/02_specifications.md) と照合する。
4. STM32CubeIDEプロジェクトを生成する。
5. DebugをSWDに設定する。
6. まず速度制御を挿入し、実機で回転を確認する。
7. 電流換算を確認してからトルク制御へ進む。

## 資料

- [sensorless-speed-control.md](sensorless-speed-control.md): 最初の速度制御試験
- [sensorless-torque-control.md](sensorless-torque-control.md): RUN後のトルク制御と速度キャップ
- [archive/main-user-code-history.md](archive/main-user-code-history.md): 過去の実験コード

過去資料には廃止したAPI名、ADC1使用、過大電流値などが含まれます。現行実装へコピーする前に、生成済み `mc_api.h` と仕様書を確認してください。
