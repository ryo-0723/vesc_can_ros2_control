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
| モータ指令（通常ノード） | vesc_can_msgs/msg/VescCommandArray | 生成時刻と論理 ID・float32 指令値の配列、単位は A / 関節 rad / rad/s |
| 関節状態（ros2_control） | sensor_msgs/msg/JointState | joint_state_broadcaster が state interface から配信 |
| VESC 再起動（通常ノード） | vesc_can_msgs/srv/Reboot | motor_id は論理 ID、対応する 1 台を再起動 |
| 接続・鮮度・ドライバ異常 | diagnostic_msgs/msg/DiagnosticArray | モータごとの hardware_id と key/value |
| 即時の異常通知 | diagnostic_msgs/msg/DiagnosticArray | /vesc/event に VESC 故障コードまたは PC 側の停止理由を配信 |

上流 `vesc_msgs` は状態用のパッケージです
通常ノードの独自指令は独立した `vesc_can_msgs/msg/VescCommand.msg` と `VescCommandArray.msg` に置きます
上位ノードは vesc_can_msgs に依存すれば使え、ドライバ全体への依存は不要です
CAN フレームには標準型を使い、再起動サービスは独立した vesc_can_msgs に置きます
VescImu/VescImuStamped はネイティブ STATUS に
IMU データがないため、ソースにも生成対象にも含めません。

## 通常ノードのモータ識別と指令

1 つの通常ノードが `motors` に登録した全台を管理します
ROS 側の数値 ID は `logical_id`、CAN の機器 ID は `controller_id` で管理します
モータ一覧と、モータ名ごとの設定は rox2026 の YAML と同じ形です
指令は `command_topic`（既定 `/vesc/commands`）、状態は `~/motors/<motor_name>/state` を使用します

```text
# VescCommand
uint8 motor_id
float32 value

# VescCommandArray
builtin_interfaces/Time stamp
VescCommand[] commands
```

motor_id は設定の logical_id に対応し、0〜255 の範囲で重複させません
モードは YAML の control_mode で起動時に固定し、指令に mode は持たせません
通常ノードは control_mode / command_interface パラメータの起動後の変更も拒否します
value は電流 [A]、関節速度 [rad/s]、関節位置 [rad] のいずれかです
配列順は自由で、未指定モータの指令と期限は維持します
未登録 ID、重複 ID、不正な指令値は全台停止になります
stamp は送信側の ROS 生成時刻で、タイムアウトには受信側の単調時刻を使用します
stamp が 0 の指令も受け付け、古い・未来の stamp によって指令期限を延長しません
状態メッセージ内の controller_id は元の VESC 機器 ID を保持します

異常は `error_topic`（既定 `/vesc/event`）へ DiagnosticArray で即時通知します
不正指令・CAN エラーフレームはコールバック、期限切れは CAN 周期で検出した直後に配信します
VESC 本体の異常は該当モータだけを status に入れ、values に source=vesc / controller_id / fault_code を格納します
PC 側の異常は source=driver として全台の停止理由を通知します
同じ本体の故障コードは繰り返さず、変化・正常応答後の再発を通知します
最後のイベントを TransientLocal で保持し、周期の VescState で現在のコードを確認できます
問い合わせの既定周期は 100 ms、異常応答を受信した処理から即時配信します

## 再起動サービス

通常ノードの ~/reboot は vesc_can_msgs/srv/Reboot です

```text
uint8 motor_id
---
bool success
string message
```

motor_id は指令と同じ YAML の logical_id です
success は要求の受理を表し、本体の再起動完了の確認ではありません
対応する CAN ID へ COMM_REBOOT を 1 回送り、保存した指令は全台分を破棄します
対象の受信状態をクリアし、500 ms 後の新鮮な状態と新しい指令から再開します
通常ノードの ~/enable は廃止し、起動後は指令の受信だけで動作を開始します
ros2_control の PC 側の開始・停止は lifecycle に従います

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
故障コードの selective 問い合わせにも対応します
設定・BMS・GNSS・ファームウェア更新・問い合わせ応答の全パケットには対応していません

| 受信 | VescState のフィールド | 配信する値 |
|---|---|---|
| STATUS1 | speed / current_motor / duty_cycle | 生の ERPM、モータ電流 [A]、duty |
| STATUS2 | charge_drawn / charge_regen | 積算電荷 [Ah] |
| STATUS3 | energy_drawn / energy_regen | 積算電力量 [Wh] |
| STATUS4 | temp_fet / temp_motor / current_input / pid_pos_now | 温度 [℃]、入力電流 [A]、モータ PID 角度 [deg] |
| STATUS5 | displacement / voltage_input | 符号付きタコメータ [count]、入力電圧 [V] |
| STATUS6 | 対応するフィールドなし | ADC1〜3 [V] と PPM 値を /diagnostics に配信 |
| COMM_GET_VALUES_SELECTIVE（bit 15） | fault_code | VESC 本体の故障コード、正常応答は 0 |

`speed` は極対数・減速比・方向を適用する前の ERPM です。`pid_pos_now` も生の
モータ PID 角度です。ros2_control の state interface には変換後の関節単位を渡します。
JointState の関節状態は joint_state_broadcaster がこの interface から作成します
複数の STATUS をまとめた状態であり、header.stamp は ROS の配信時刻です。
すべてのフィールドが同時に測定されたことを表す時刻ではありません。

浮動小数点のフィールドは、その STATUS を未受信または期限切れなら NaN です。
STATUS2/3/6 を受信していなくても、制御に必要なフィードバックが揃っていれば
有効化できます。対応可能な STATUS をすべて高頻度で送信する必要はありません。

STATUS1〜6 から取得できない avg_id/avg_iq/ntc_temp_mos1〜3/avg_vd/avg_vq は
常に NaN、distance_traveled は **-1 (未取得)** です
fault_code は別途 selective 問い合わせで取得し、未受信・期限切れは **-1** です
fault_code=0 は実機の「故障なし」を意味するため、未取得値には使いません。
ドライバの通信途絶などを VESC の故障番号へ変換せず、/diagnostics と即時の /vesc/event に配信します
イベントの fault_code は DiagnosticStatus.values の文字列なので、上位ノードで整数へ変換します
VescState.fault_code は従来どおり int32 です
故障コードの取得・応答先 ID・中継フィルタは [CAN 故障問い合わせ](../vesc_can_driver/README.md#即時の異常通知) を参照してください
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
| ActuatorTarget / ActuatorTargetArray | /vesc/commands の VescCommandArray、logical_id を motor_id へ移し、配列の stamp に生成時刻 |
| ~/motors/<motor_name>/command の Float64 | VescCommandArray の要素 1 個、設定の logical_id と旧 data を指定 |
| ~/joint_commands の JointState | VescCommandArray、関節名を logical_id に対応付け、position / velocity から value へ移す |
| ~/commands の MotorCommands | VescCommandArray、names を設定の logical_id に対応付け、values を各 value へ移す |
| vesc_can_driver/msg/VescCommandArray | vesc_can_msgs/msg/VescCommandArray に移動、各 mode を削除し、配列の stamp に生成時刻を入れる |
| ActuatorState / ActuatorStateArray | ~/motors/<motor_name>/state の VescStateStamped。関節単位は ros2_control の state interface / joint_state_broadcaster を使用 |
| logical_id | 通常ノードの YAML で復活、0〜255。ros2_control は関節名を使用 |
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
