# メッセージ対応と移行

独自 msg / srv はすべて `vesc_can_msgs` にまとめます
指令・状態・再起動の型を使う上位ノードは、このパッケージだけに依存できます
状態型は CAN で取得する項目から新しく定義し、F1TENTH の msg と同梱 `vesc_msgs` は廃止しました

## 使用する型

| 用途 | 型 | 単位・識別 |
|---|---|---|
| CAN 境界 | can_msgs/msg/Frame | 元の CAN ID、フラグ、DLC、ペイロードを保持 |
| モータ指令（通常ノード） | vesc_can_msgs/msg/VescCommandArray | 生成時刻と論理 ID・float32 指令値の配列、単位は A / 関節 rad / rad/s / duty 比 |
| VESC 状態 | vesc_can_msgs/msg/VescState | モータ名ごとのトピック、controller_id は CAN の機器 ID |
| VESC 再起動（通常ノード） | vesc_can_msgs/srv/Reboot | motor_id は論理 ID、対応する 1 台を再起動 |
| 関節状態（ros2_control） | sensor_msgs/msg/JointState | joint_state_broadcaster が state interface から配信 |
| 接続・鮮度・ドライバ異常 | diagnostic_msgs/msg/DiagnosticArray | モータごとの hardware_id と key/value |
| 即時の異常通知 | diagnostic_msgs/msg/DiagnosticArray | /vesc/event に VESC 故障コードまたは PC 側の停止理由を配信 |

CAN と関節状態、診断は既存の標準型を使用します
独自の IMU 型や、VescState を包む Stamped 型はありません

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
value は電流 [A]、関節速度 [rad/s]、関節位置 [rad]、duty 比のいずれかです
モードは current / velocity / position / duty_cycle / current_velocity から選びます
duty_cycle は関節方向の比率 [-1, 1] で、direction と max_duty を適用して送信します
current_velocity でも value は関節速度 [rad/s] です
配列順は自由で、未指定モータの指令と期限は維持します
未登録 ID、重複 ID、不正な指令値は全台停止になります
stamp は送信側の ROS 生成時刻で、タイムアウトには受信側の単調時刻を使用します
stamp が 0 の指令も受け付け、古い・未来の stamp によって指令期限を延長しません

## VescState の対応範囲

CAN レイアウトは [VESC firmware 6.06 の送信処理](https://github.com/vedderb/bldc/blob/94b305ec124b67b0161f2d3d85c4d5b35e48edc5/comm/comm_can.c) を基準にします
通常モータ向けの STATUS1〜6 をデコードし、状態と診断に反映します
故障コードは selective 問い合わせで取得します

| 取得元 | VescState のフィールド | 配信する値 |
|---|---|---|
| ROS | stamp | 配信時刻 |
| 設定 | controller_id | CAN の機器 ID、0〜254 |
| STATUS1 | erpm / motor_current_a / duty_cycle | 生の ERPM、モータ電流 [A]、duty |
| STATUS2 | charge_drawn_ah / charge_regen_ah | 積算電荷 [Ah] |
| STATUS3 | energy_drawn_wh / energy_regen_wh | 積算電力量 [Wh] |
| STATUS4 | temp_fet_c / temp_motor_c / input_current_a / pid_pos_deg | 温度 [℃]、入力電流 [A]、モータ PID 角度 [deg] |
| STATUS5 | tachometer_counts / tachometer_fresh / input_voltage_v | 符号付きタコメータ [count]、STATUS5 の鮮度、入力電圧 [V] |
| COMM_GET_VALUES_SELECTIVE（bit 15） | fault_code | 本体の故障コード、正常応答は 0 |

STATUS6 の ADC1〜3 [V] と PPM は /diagnostics に配信し、VescState には追加しません
erpm は極対数・減速比・方向を適用する前の値、pid_pos_deg も生のモータ PID 角度です
ros2_control の state interface には変換後の関節単位を渡します
JointState は joint_state_broadcaster がこの interface から作成します

### 時刻と欠測値

VescState は stamp を直接持ち、header や入れ子の state はありません
各 STATUS は独立して届くため、stamp は全項目の同時測定を表しません
各項目の鮮度は配信時の単調時刻とモータの feedback_timeout で判定します

- 浮動値は float64、未受信・期限切れは NaN
- tachometer_counts は int32、最終受信値を保持し、未受信時は 0
- tachometer_fresh は STATUS5 が受信済みで期限内なら true
- fault_code は int16、未受信・期限切れは -1、正常応答は 0、本体の故障は正のコード

float64 により、CAN の符号付き int32 の ERPM をそのまま表現できます
整数のタコメータを現在のフィードバックとして使う場合は tachometer_fresh=true を確認します
故障コードは uint8 の応答値 0〜255 と未取得値 -1 を保持できます
ドライバの通信途絶などは本体の故障番号へ変換せず、診断と /vesc/event に配信します

STATUS2/3/6 が未受信でも、モードに必要なフィードバックが揃えば制御を開始できます
全 STATUS を同じ高頻度で送信する必要はありません
他の STATUS の鮮度は診断の status_1_fresh〜status_6_fresh からも確認できます

### 削除した未取得項目

旧定義の avg_id、avg_iq、ntc_temp_mos1〜3、avg_vd、avg_vq、distance_traveled の 8 項目は、
このドライバが取得せず常に欠測値だったため削除しました
未使用の frame_id と、VescStateStamped の包みも廃止しました
取得できる電流・電圧・温度・電荷・電力量・タコメータ・故障コードは残しています

## 異常イベント

異常は `error_topic`（既定 `/vesc/event`）へ DiagnosticArray で即時通知します
不正指令・CAN エラーフレームはコールバック、期限切れは CAN 周期で検出した直後に配信します
本体の異常は該当モータだけを status に入れ、values に source=vesc / controller_id / fault_code を格納します
PC 側の異常は source=driver として全台の停止理由を通知します
イベントの fault_code は KeyValue の文字列なので、上位ノードで整数へ変換します
同じ本体の故障コードは繰り返さず、変化・正常応答後の再発を通知します
最後のイベントを TransientLocal で保持し、周期の VescState で現在のコードを確認できます
問い合わせの既定周期は 100 ms、故障応答を受信した処理から即時配信します
故障コードの応答先 ID・中継フィルタは [CAN 故障問い合わせ](../vesc_can_driver/README.md#即時の異常通知) を参照してください

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
起動後の通常ノードは、新しい指令と必要な STATUS が揃うと動作を開始します
ros2_control の PC 側の開始・停止は lifecycle に従います

## VESC firmware 6 の STATUS 形式

参照は公式 `vedderb/bldc` の `release_6_06`、commit
`94b305ec124b67b0161f2d3d85c4d5b35e48edc5` です
[comm_can.c](https://github.com/vedderb/bldc/blob/94b305ec124b67b0161f2d3d85c4d5b35e48edc5/comm/comm_can.c) の送信・受信処理と
[datatypes.h](https://github.com/vedderb/bldc/blob/94b305ec124b67b0161f2d3d85c4d5b35e48edc5/datatypes.h) の packet ID を照合しました
VESC のハードウェア世代ではなく、firmware のネイティブ CAN 形式の参照です

全フレームは拡張 ID、DLC=8、big endian です
i16 / i32 は符号付き 16 / 32 bit の整数で、下表はペイロード先頭からの順です

| STATUS | packet ID | デコードする項目の順序 |
|---|---|---|
| 1 | 9 | ERPM (i32)、モータ電流 (i16 / 10 A)、duty (i16 / 1000) |
| 2 | 14 | 消費電荷 (i32 / 10000 Ah)、回生電荷 (i32 / 10000 Ah) |
| 3 | 15 | 消費電力量 (i32 / 10000 Wh)、回生電力量 (i32 / 10000 Wh) |
| 4 | 16 | FET 温度 (i16 / 10 ℃)、モータ温度 (i16 / 10 ℃)、入力電流 (i16 / 10 A)、PID 角度 (i16 / 50 deg) |
| 5 | 27 | タコメータ (i32 count)、入力電圧 (i16 / 10 V)、予約領域 (2 bytes、無視) |
| 6 | 58 | ADC1 / ADC2 / ADC3 (各 i16 / 1000 V)、PPM (i16 / 1000) |

Status1〜Status6 の構造体、宣言、関数定義をこの順序に揃えています
標準 ID・RTR・エラーフレーム・異なる packet ID・DLC≠8 は false を返し、出力を更新しません

## 購読側の変更

状態トピック名は `~/motors/<motor_name>/state` のままで、型を `vesc_can_msgs/msg/VescState` へ変更します
旧型の互換エイリアスはありません

| 旧アクセス | 新アクセス |
|---|---|
| msg.header.stamp | msg.stamp |
| msg.state.controller_id | msg.controller_id |
| msg.state.speed | msg.erpm |
| msg.state.current_motor / current_input | msg.motor_current_a / input_current_a |
| msg.state.temp_fet / temp_motor | msg.temp_fet_c / temp_motor_c |
| msg.state.pid_pos_now | msg.pid_pos_deg |
| msg.state.voltage_input | msg.input_voltage_v |
| msg.state.displacement | msg.tachometer_counts、利用時に msg.tachometer_fresh を確認 |
| msg.state.charge_drawn / charge_regen | msg.charge_drawn_ah / charge_regen_ah |
| msg.state.energy_drawn / energy_regen | msg.energy_drawn_wh / energy_regen_wh |
| msg.state.fault_code | msg.fault_code、int16 に変更 |

通常ノードの指令、再起動サービス、CAN の形式、ros2_control の state / command interface は同じです
通常ノードの状態は VescState と診断を使用し、JointState は配信しません
