# 第三者ソフトウェアとMCSDKの扱い

## 結論

X-CUBE-MCSDKは、パッケージ全体を単一のOSSライセンスで公開できるものではありません。ST公式資料では、SLA0048、Ultimate Liberty、OSS、第三者ライセンスが混在すると説明されています。

SLA0048は一定条件の下でソース・バイナリの再配布を認めていますが、著作権表示、条件、免責の保持、ST製マイコンとの組合せ、各第三者ライセンスなどの条件があります。さらに、MCSDK内の各コンポーネントは個別条件を持ちます。

このリポジトリでは条件の取り違えを避けるため、次を採用します。

- MCSDK本体、CMSIS、HAL、同梱ライブラリ、生成済みベンダーコードを登録しない。
- チームが作成したMarkdown、設定値、Workbenchの `.stwb6`、ユーザー記述コードの抜粋だけを保持する。
- MCSDK 6.4.2は利用者がST公式サイトから取得する。
- 元ファイルの著作権表示やライセンス文を削って再配布する運用はしない。

この判断は法的助言ではなく、公開範囲を最小化するためのプロジェクト運用判断です。

## ST公式資料

- [X-CUBE-MCSDK製品ページ](https://www.st.com/en/embedded-software/x-cube-mcsdk.html)
- [X-CUBE-MCSDK Data brief](https://www.st.com/resource/en/data_brief/x-cube-mcsdk.pdf)

Data briefでは、MCSDKがSLA0048の混合ライセンスで提供され、ST MC FOC firmwareがUltimate Libertyのsource releaseとして扱われること、コンポーネントごとの条件確認が必要であることが示されています。

## 再生成時の注意

1. ST公式サイトからX-CUBE-MCSDK 6.4.2を取得する。
2. ダウンロード時に表示される最新の利用条件を確認する。
3. `projects/*.stwb6` をWorkbenchで開いて生成する。
4. `firmware/` の資料を参照してUSER CODEセクションへチームコードを戻す。
5. 生成物はローカルで使用し、公開リポジトリへ再追加しない。
