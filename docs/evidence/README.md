# 根拠資料の追加方法

レポート本文の主張には、写真、回路図、測定ログ、コード、実験ノートのいずれかを対応付けます。

## 推奨構成

```text
docs/evidence/
├─ circuits/       # 匿名化した回路図
├─ photos/         # メタデータ削除・背景確認済み写真
├─ plots/          # 電流、速度、CAN値などのグラフ
├─ logs/           # 個人情報を除いたCSV・テキストログ
├─ test-log-template.md
├─ photo-register.md
└─ source-index.md
```

未処理ファイルは `raw-media/` に置き、コミットしません。

## ファイル名

`YYYYMMDD_対象_内容_番号.ext` とします。

例:

- `20260812_motor_speedcap_01.csv`
- `20260812_can_waveform_01.png`
- `20260813_torque_washer_drawing_01.svg`

氏名、学籍番号、PC名をファイル名へ入れません。
