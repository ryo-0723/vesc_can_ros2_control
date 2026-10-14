# メッセージ対応と移行

状態は [f1tenth の ROS 2 版 vesc_msgs](https://github.com/f1tenth/vesc/tree/153998df8545fe1781b975df88e411b4e71d4bfe/vesc_msgs)
から状態用メッセージ 2 個を、このリポジトリの `vesc_msgs/` に同梱します。これはコミュニティの
ROS パッケージで、VESC firmware の CAN パケットそのものとは別の定義です。
`VescState.msg` と `VescStateStamped.msg` の内容を変更せずコピーし、上流の BSD
ライセンスを保存しています。CMake と package.xml は、この 2 個だけを生成する
ローカルの最小構成です。LICENSE と UPSTREAM.md をインストールします。
取り込み元と commit、各ファイルの SHA-256 は [UPSTREAM.md](../vesc_msgs/UPSTREAM.md) に
記録しています。同名の `vesc_msgs` でも fork や revision により定義が異なるため、
他の source copy は同じワークスペースへ追加しません。

## 使用する型

| 用途 | 型 | 単位・識別 |
|---|---|---|
| CAN 境界 | can_msgs/msg/Frame | 元の CAN ID、フラグ、DLC、ペイロードを保持 |
| VESC 状態 | vesc_msgs/msg/VescStateStamped | モータ名ごとのトピックで識別、state.controller_id は VESC の機器 ID |
| モータ指令（通常ノード） | vesc_can_driver/msg/MotorCommands | モータ名と指令値の配列、設定したモードに応じて A / 関節 rad / rad/s |
| 関節状態（ros2_control） | sensor_msgs/msg/JointState | joint_state_broadcaster が state interface から配信 |
| 有効化・無効化 | std_srvs/srv/SetBool | 明示的な有効化と異常解除 |
| 接続・鮮度・ドライバ異常 | diagnostic_msgs/msg/DiagnosticArray | モータごとの hardware_id と key/value |

上流 `vesc_msgs` は状態用のパッケージです
通常ノードの独自指令は `vesc_can_driver/msg/MotorCommands.msg` に置きます
CAN フレームと有効化サービスには標準型を使います
VescImu/VescImuStamped はネイティブ STATUS に
IMU データがないため、ソースにも生成対象にも含めません。

## 通常ノードのモータ識別と指令

1 つの通常ノードが `motors` に登録した全台を管理します
ROS 側の識別子はモータ名で、数値の logical_id は追加しません
モータ名と CAN の機器 ID の対応は YAML の `controller_id` に集約します
指令は `~/commands`、状態は `~/motors/<motor_name>/state` を使用します
上位ノードは同じモータ名で送受信でき、CAN ID を知る必要はありません

```text
string[] names
float64[] values
```

names と values の長さを揃え、各指令値の単位はそのモータの設定から決まります
配列順は自由で、未指定モータの指令と期限は維持します
配列長の不一致、未登録名、重複名、不正な指令値は全台停止になります
モードは指令から切り替えません
JointState は関節状態用で、effort は電流ではなくトルクなので指令に流用しません
状態メッセージ内の controller_id は元の VESC 機器 ID を保持します

## コピーと vcs 取得の使い分け

このリポジトリでは **必要な msg 2 個のコピー** を採用します。定義が小さく、
上流ドライバを利用せず、IMU を除いた状態専用の構成を維持するためです。
取得元 commit、内容のハッシュ、BSD ライセンス、上流の著者を記録します。
更新は自動追従せず、定義の差分と統合テストを確認して取り込みます。

[vcstool](https://github.com/dirk-thomas/vcstool#exporting-and-importing-sets-of-repositories)
の `vcs import` で Git を取得する場合は、URL と version でリポジトリを指定します。
上流一式を使う場合や、複数リポジトリの revision を一括管理する場合に適しています。
今回の 2 個だけを使う構成に切り出すには、取得後の選別・ローカル変更が追加で必要です。
`vcs` に切り替える場合は同梱 `vesc_msgs` を外し、commit を固定して取り込みます。

この同梱版は上流 `vesc_msgs` 全体の代替ではありません。IMU を使う他のノードと
同じワークスペースで利用する場合は、同梱版を外して完全な上流パッケージに揃えます。
状態メッセージ 2 個は同じ定義なので、本ドライバの API はそのまま利用できます。

## VescState の対応範囲

CAN レイアウトは [VESC firmware 6.06 の送信処理](https://github.com/vedderb/bldc/blob/94b305ec124b67b0161f2d3d85c4d5b35e48edc5/comm/comm_can.c)
を基準にします。現在、通常モータ向けの周期状態 STATUS1〜6 をデコード・保存します。
設定・BMS・GNSS・ファームウェア更新・問い合わせ応答の全パケットには対応していません。

| 受信 | VescState のフィールド | 配信する値 |
|---|---|---|
| STATUS1 | speed / current_motor / duty_cycle | 生の ERPM、モータ電流 [A]、duty |
| STATUS2 | charge_drawn / charge_regen | 積算電荷 [Ah] |
| STATUS3 | energy_drawn / energy_regen | 積算電力量 [Wh] |
| STATUS4 | temp_fet / temp_motor / current_input / pid_pos_now | 温度 [℃]、入力電流 [A]、モータ PID 角度 [deg] |
| STATUS5 | displacement / voltage_input | 符号付きタコメータ [count]、入力電圧 [V] |
| STATUS6 | 対応するフィールドなし | ADC1〜3 [V] と PPM 値を /diagnostics に配信 |

`speed` は極対数・減速比・方向を適用する前の ERPM です。`pid_pos_now` も生の
モータ PID 角度です。ros2_control の state interface には変換後の関節単位を渡します。
JointState の関節状態は joint_state_broadcaster がこの interface から作成します
複数の STATUS をまとめた状態であり、header.stamp は ROS の配信時刻です。
すべてのフィールドが同時に測定されたことを表す時刻ではありません。

浮動小数点のフィールドは、その STATUS を未受信または期限切れなら NaN です。
STATUS2/3/6 を受信していなくても、制御に必要なフィードバックが揃っていれば
有効化できます。対応可能な STATUS をすべて高頻度で送信する必要はありません。

STATUS1〜6 から取得できない avg_id/avg_iq/ntc_temp_mos1〜3/avg_vd/avg_vq は
常に NaN、distance_traveled と fault_code は **-1 (未取得)** です。
fault_code=0 は実機の「故障なし」を意味するため、未取得値には使いません。
ドライバの通信途絶などを VESC の故障番号へ変換せず、/diagnostics に配信します。
全フィールドの実値が必要な場合は、別途 VESC の値取得コマンドと CAN 分割通信の
実装が必要です。上流 msg の流用だけで取得できるデータが増えるわけではありません。

整数型の displacement は NaN を表現できないため、最後に受信した値を保持します。
未受信時は 0 です。利用前に、同じモータの /diagnostics にある **status_5_fresh=true**
を確認してください。他の STATUS も status_1_fresh〜status_6_fresh で判定できます。
unavailable_vesc_fields には常に未取得となる上流フィールドを列挙します。

STATUS5 の公式形式は既知の 6 バイト + 予約 2 バイトの DLC=8 です。
全 STATUS を DLC=8 に統一し、以前の STATUS5 の DLC=6 互換受理は廃止しました。
模擬 VESC と既知値テストも公式の 8 バイト形式を使い、
予約領域が既知フィールドに影響しないことを確認します。

## VESC firmware 6 の STATUS 形式

参照は公式 `vedderb/bldc` の `release_6_06`、commit
`94b305ec124b67b0161f2d3d85c4d5b35e48edc5` です。
[comm_can.c](https://github.com/vedderb/bldc/blob/94b305ec124b67b0161f2d3d85c4d5b35e48edc5/comm/comm_can.c)
の送信・受信処理と
[datatypes.h](https://github.com/vedderb/bldc/blob/94b305ec124b67b0161f2d3d85c4d5b35e48edc5/datatypes.h)
の packet ID を照合しました。公式タグ `6.00` でも同じ STATUS1〜6 の形式を確認しています。
VESC のハードウェア世代ではなく、firmware のネイティブ CAN 形式の参照です。

全フレームは拡張 ID、DLC=8、big endian です。
`i16` / `i32` は符号付き 16 / 32 bit の整数で、下表はペイロード先頭からの順です。

| STATUS | packet ID | デコードする項目の順序 |
|---|---|---|
| 1 | 9 | ERPM (i32)、モータ電流 (i16 / 10 A)、duty (i16 / 1000) |
| 2 | 14 | 消費電荷 (i32 / 10000 Ah)、回生電荷 (i32 / 10000 Ah) |
| 3 | 15 | 消費電力量 (i32 / 10000 Wh)、回生電力量 (i32 / 10000 Wh) |
| 4 | 16 | FET 温度 (i16 / 10 ℃)、モータ温度 (i16 / 10 ℃)、入力電流 (i16 / 10 A)、PID 角度 (i16 / 50 deg) |
| 5 | 27 | タコメータ (i32 count)、入力電圧 (i16 / 10 V)、予約領域 (2 bytes、無視) |
| 6 | 58 | ADC1 / ADC2 / ADC3 (各 i16 / 1000 V)、PPM (i16 / 1000) |

`Status1`〜`Status6` の構造体、宣言、関数定義をこの順序に揃えています。
標準 ID・RTR・エラーフレーム・異なる packet ID・DLC≠8 は false を返し、出力を更新しません。

## 旧独自 API からの移行

パッケージ分割後の通常ノードと模擬 VESC は `vesc_can_driver` にあります。
起動は `ros2 launch vesc_can_driver vesc_standalone.launch.py`、設定ファイルは
`vesc_can_driver/config/standalone.yaml` を使用します。既存のトピック設定を保つため、
通常ノードの既定名は `vesc_can_ros2_control` のままです。
hardware plugin 名は `vesc_can_ros2_control/VescSystem` を指定します。
C++ の共通ヘッダは `vesc_can_driver/...`、namespace は `vesc_can_driver` へ移動しています。

開発段階の独自 `vesc_can_interfaces` パッケージと SetPosition.srv は削除しました。
旧型の publisher/subscriber は次の型・トピック・単位へ変更してください。

| 旧 API | 移行先 |
|---|---|
| ActuatorTarget / ActuatorTargetArray | ~/commands の MotorCommands、names にモータ名、values に指令値 |
| ~/motors/<motor_name>/command の Float64 | ~/commands の MotorCommands、names: [motor_name]、values: [旧 data] |
| ~/joint_commands の JointState | ~/commands の MotorCommands、モータ名ごとに position / velocity から values へ移す |
| ActuatorState / ActuatorStateArray | ~/motors/<motor_name>/state の VescStateStamped。関節単位は ros2_control の state interface / joint_state_broadcaster を使用 |
| logical_id | motors の関節名で指令先を選ぶ。実機の CAN ID は controller_id に指定 |
| state_array_publish_period_ms | state_publish_period_ms |
| target_topic / target_array_topic / state_topic / state_array_topic | 新トピックへの ROS remapping |

通常ノードの `~/joint_states` は廃止しました
通常ノードの状態は VescStateStamped と診断を使用します

旧駆動指令の機械 RPM を新しい関節 rad/s へ移すときは
`joint_rad_s = motor_rpm × 2π / (60 × gear_ratio)` で変換します。
速度指令は上流 UART ノードの commands/motor/speed (ERPM) とは単位が異なります。
CAN の配線、中継器の can_msgs 型、ros2_control の関節インターフェースは同じです。

## Lyrical と apt 公開

確認時の [Lyrical rosdistro 一覧](https://github.com/ros/rosdistro/blob/master/lyrical/distribution.yaml)
には vesc_msgs がありません。開発と CI は同梱した `vesc_msgs` をソースからビルドし、
別のリポジトリを取得せず依存を満たします。以前の dependencies.repos で取得した
`src/vesc_upstream` がある場合は、`src` の外へ移してからビルドしてください。

apt 公開時には対象 distro で `vesc_msgs` をリリース済みのパッケージとして解決する
必要があります。同梱パッケージもリリースするか、上流のリリース済みパッケージへ
依存するかを上流メンテナと調整します。同名のパッケージを別の release repository
から重複登録せず、メッセージの定義も独自に変更しません。
