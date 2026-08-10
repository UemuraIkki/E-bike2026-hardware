# E-bike 2026 ハードウェア開発資料

古い自転車を電動アシスト化し、人力に応じてモータ出力を変えるE-bikeの開発記録です。レポート作成に必要な仕様、開発経過、担当範囲、試験結果、考察材料を、個人情報を含めずに集約します。

## 重要方針

- 氏名、学籍番号、個人メールアドレス、顔写真などは登録しません。
- 担当者は `担当A`〜`担当D` の匿名ラベルで管理します。
- 実名対応表、個人の感想、表紙、最終提出PDFはローカルだけで管理します。
- X-CUBE-MCSDK本体、CMSIS、HALドライバなどの第三者配布物は登録しません。
- Workbench設定スナップショットには過去の実験値も含まれます。現行仕様は必ず [仕様書](docs/02_specifications.md) を基準にします。

詳細は [PRIVACY.md](PRIVACY.md) と [THIRD_PARTY.md](THIRD_PARTY.md) を参照してください。

## レポート資料

| 章 | 資料 |
|---|---|
| 1. 装置・ソフトウェアの概要 | [docs/01_overview.md](docs/01_overview.md) |
| 2. 装置・ソフトウェアの仕様 | [docs/02_specifications.md](docs/02_specifications.md) |
| 3. 開発計画・役割分担 | [docs/03_plan_and_roles.md](docs/03_plan_and_roles.md) |
| 4. 開発 | [docs/04_development.md](docs/04_development.md) |
| 5. 動作試験 | [docs/05_testing.md](docs/05_testing.md) |
| 6. 考察 | [docs/06_discussion.md](docs/06_discussion.md) |
| 7. 感想 | [docs/07_reflection_template.md](docs/07_reflection_template.md) |

提出条件と資料の不足状況は [docs/README.md](docs/README.md)、写真や測定結果の追加方法は [docs/evidence/README.md](docs/evidence/README.md) にまとめています。

## 現在の技術構成

- 制御ボード: NUCLEO-L476RG（STM32L476RG、bxCAN内蔵）
- パワーボード: X-NUCLEO-IHM08M1（3相インバータ、最大出力電流10A）
- モータ: ZGC 57DMWH75-2440、24V BLDC、極対数2、定格4000rpm、無負荷5000rpm、定格0.28N·m / 7A
- 制御: X-CUBE-MCSDK 6.4.2、センサレスFOC（State Observer + PLL）
- トルクセンサ: BF SR系、CAN出力
- CANトランシーバ: SN65HVD230、3.3V動作
- デバッグ: SWD固定

現在はセンサレスFOCでモータを回し、トルクモードと安全制限を実機調整している段階です。Hall方式は検証資料を残しますが、現行運用方式ではありません。

## ファームウェア資料

- [firmware/README.md](firmware/README.md): MCSDKでの再生成方法とコード資料の使い方
- [firmware/sensorless-speed-control.md](firmware/sensorless-speed-control.md): 速度制御の挿入箇所と試験手順
- [firmware/sensorless-torque-control.md](firmware/sensorless-torque-control.md): トルク制御、助走、速度キャップ
- [firmware/archive/main-user-code-history.md](firmware/archive/main-user-code-history.md): 過去の `main.c` USER CODEブロックの機械抽出
- [projects/README.md](projects/README.md): `.stwb6` スナップショットの位置付け

## 提出までの不足資料

- 4担当それぞれの作業日、作業時間、判断理由
- Hall回路図と実装写真
- CANモジュール回路図、ビットレート、CAN ID、フレーム定義、受信ログ
- 車体整備の実施前後写真と確認結果
- トルクセンサ用ワッシャの寸法図、材質、加工方法
- モータ試験の条件表、時系列ログ、速度キャップ試験
- 30km / 約2時間を想定した消費電力量と温度試験

最終提出期限は2026年8月31日 17:00です。
