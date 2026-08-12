# レポート資料索引

提出要件は次の7章です。本ディレクトリは章番号と対応させています。

| 章 | ファイル | 現在の状態 |
|---|---|---|
| 1. 概要 | [01_overview.md](01_overview.md) | 共通本文の下書きあり |
| 2. 仕様 | [02_specifications.md](02_specifications.md) | 確定値を集約済み |
| 3. 計画・役割 | [03_plan_and_roles.md](03_plan_and_roles.md) | 匿名担当と当初計画を整理済み |
| 4. 開発 | [04_development.md](04_development.md) | 既知の経過あり、各担当の詳細追記必要 |
| 5. 試験 | [05_testing.md](05_testing.md) | 既知結果あり、写真・測定表が不足 |
| 6. 考察 | [06_discussion.md](06_discussion.md) | 共通材料あり |
| 7. 感想 | [07_reflection_template.md](07_reflection_template.md) | ローカル記入用テンプレート |

## 補助資料

- [匿名担当別・開発日程まとめ](work-timeline-by-role.md): 担当A〜Dの作業を大まかな日程順に一覧化
- [根拠資料索引](evidence/source-index.md): コード、測定、写真、回路、図面の有無を管理

### 担当別の簡易説明

- [担当A: モータ制御・モータ位置Hall・全体管理](roles/role-a-motor-control.md)
- [担当B: 車速検出用Hallセンサ](roles/role-b-wheel-speed-hall.md)
- [担当C: 車体整備・機械加工](roles/role-c-vehicle-mechanics.md)
- [担当D: CAN通信モジュール](roles/role-d-can-communication.md)

## 使い方

1. 共通部分はこのリポジトリの本文を各自のレポートへ引用・再構成する。
2. 第4章では自分の担当ラベルの内容を特に詳しく書く。
3. 実名、学籍番号、感想はGitHub外で追記する。
4. 事実と計画を混同しない。未完了項目は「予定」「要確認」と明記する。
5. 写真、測定値、回路図を追加したら [evidence/source-index.md](evidence/source-index.md) に出典を記録する。

## 提出前の共通確認

- 数値が [02_specifications.md](02_specifications.md) と一致しているか。
- センサレスFOCを現行方式として記述しているか。
- Hall方式を「検証中または将来候補」と区別しているか。
- 人力最大2倍、24km/h以上でアシスト停止という制約を記述したか。
- 「できたこと」と「今後実装すること」を区別したか。
- 自身の担当箇所が明確か。
- 写真と図に番号、説明、撮影・作成条件があるか。
