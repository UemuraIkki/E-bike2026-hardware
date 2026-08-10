# Motor Control Workbench設定スナップショット

このディレクトリには、MCSDK 6.4.2 Motor Control Workbenchの `.stwb6` 設定ファイルだけを保存します。生成済みソース、CMSIS、HAL、MCSDK本体は保存しません。

## スナップショット

| ファイル | 位置付け |
|---|---|
| `BLDC0416.stwb6` | 初期設定の記録 |
| `BLDC0421.stwb6` | 速度制御初期段階 |
| `BLDC0428.stwb6` | Hall・ポテンショ等の検討段階 |
| `BLDC0512.stwb6` | Hall入力の検証段階 |
| `BLDC0518.stwb6` | 中間生成版 |
| `BLDC0519.stwb6` | 中間生成版 |
| `BLDC0616.stwb6` | 現在のセンサレスFOC基準版 |

日付はファイル名の `MMDD` に対応します。ただしGit上の履歴は1コミットから始まっているため、詳細な作業内容は [../docs/04_development.md](../docs/04_development.md) と実験ノートで補います。

## 注意

- スナップショットには当時の実験値が含まれる。
- `56JXE` 等のプロファイル名が表示されても、実機モータの基準仕様はZGC 57DMWH75-2440、定格7Aである。
- 14.175A設定をそのまま使用しない。
- 再生成前に [../docs/02_specifications.md](../docs/02_specifications.md) と照合する。
