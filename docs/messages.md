# メッセージ対応と移行

独自 msg / srv はすべて `vesc_can_msgs` にまとめます
指令・状態・エラー・再起動の型を使う上位ノードは、このパッケージだけに依存できます
状態型は CAN で取得する項目から新しく定義し、F1TENTH の msg と同梱 `vesc_msgs` は廃止しました

## 使用する型

| 用途 | 型 | 単位・識別 |
|---|---|---|
| CAN 境界 | can_msgs/msg/Frame | 元の CAN ID、フラグ、DLC、ペイロードを保持 |
| モータ指令（通常ノード） | vesc_can_msgs/msg/VescCommandArray | 生成時刻と論理 ID・float32 指令値の配列、単位は A / 関節 rad / rad/s / duty 比 |
| VESC 状態（通常ノード） | vesc_can_msgs/msg/VescStateArray | /vesc/state、各要素の motor_id は指令と同じ論理 ID |
| VESC 状態（ros2_control） | vesc_can_msgs/msg/VescState | モータ別のトピック、motor_id は URDF の logical_id |
| VESC 再起動（通常ノード） | vesc_can_msgs/srv/Reboot | motor_id は論理 ID、対応する 1 台を再起動 |
| 関節状態（ros2_control） | sensor_msgs/msg/JointState | joint_state_broadcaster が state interface から配信 |
| 即時の異常通知 | vesc_can_msgs/msg/VescEvent | /vesc/event に motor_id とエラー文字列を配信 |

CAN と関節状態は既存の標準型を使用します
独自の IMU 型や、VescState を包む Stamped 型はありません

## 通常ノードのモータ識別と指令

1 つの通常ノードが `motors` に登録した全台を管理します
ROS 側の数値 ID は `logical_id`、CAN の機器 ID は `controller_id` で管理します
通常ノードは YAML、ros2_control は URDF の各 joint に両方を指定します
指令・状態・エラーの motor_id は logical_id に統一し、CAN ID の対応はドライバ内で解決します
モータ一覧は motors、各モータの設定はその名前の下に記述します
指令は `command_topic`（既定 `/vesc/commands`）、状態は `state_topic`（既定 `/vesc/state`）を使用します
通常ノードの状態は VescStateArray で全モータをまとめ、異常だけ `/vesc/event` に即時通知します
周期の `/diagnostics` は配信しません

```text
# VescCommand
uint8 motor_id
float32 value

# VescCommandArray
builtin_interfaces/Time stamp
VescCommand[] commands

# VescStateArray
builtin_interfaces/Time stamp
VescState[] states
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
配列の全要素は同じ受信時刻で一括保存し、同じモータの未送信値は最新値で上書きします
全要素を検査し、不正な要素があれば一部だけ送信することなく全台停止します

## VescState の対応範囲

CAN レイアウトは [VESC firmware 6.06 の送信処理](https://github.com/vedderb/bldc/blob/94b305ec124b67b0161f2d3d85c4d5b35e48edc5/comm/comm_can.c) を基準にします
通常モータ向けの STATUS1〜6 をデコードし、共通ドライバの状態に反映します
故障コードは selective 問い合わせで取得します

| 取得元 | VescState のフィールド | 配信する値 |
|---|---|---|
| ROS | stamp | 配信時刻 |
| 設定 | motor_id | 論理 ID、0〜255、指令・エラーと共通 |
| STATUS1 | velocity_rad_s / motor_current_a / duty_cycle | 関節速度 [rad/s]、モータ電流 [A]、duty |
| STATUS2 | charge_drawn_ah / charge_regen_ah | 積算電荷 [Ah] |
| STATUS3 | energy_drawn_wh / energy_regen_wh | 積算電力量 [Wh] |
| STATUS4 | temp_fet_c / temp_motor_c / input_current_a / pid_pos_rad | 温度 [℃]、入力電流 [A]、PID 関節角 [rad] |
| STATUS5 | position_rad / input_voltage_v | 積算関節角 [rad]、入力電圧 [V] |
| COMM_GET_VALUES_SELECTIVE（bit 15） | fault_code | 本体の故障コード、正常応答は 0 |

STATUS6 の ADC1〜3 [V] と PPM は共通ドライバ内で保持します
velocity_rad_s は極対数・減速比・方向を適用した関節速度です
pid_pos_rad は STATUS4 の PID 角度を減速比・方向・zero_offset_rad で変換した単回転の関節角です
この換算には、VESC の PID 角度が校正済みのモータ機械角を表す設定が必要です
ホールセンサだけの標準設定では PID 角度が電気角を表す場合があるため、設定との一致を確認します
電流と duty は VESC 生値の符号を保持し、監視値として配信します
position_rad は全モードで STATUS5 のカウントから計算する、変換後の積算関節角です

```text
velocity_rad_s = ERPM × 2π / (60 × pole_pairs × gear_ratio) × direction
position_rad = tachometer_counts × 2π / (6 × pole_pairs × gear_ratio) × direction
```

position_rad は 2π で折り返さず、PID 用の zero_offset_rad や起動時のゼロ合わせは適用しません
例えば pole_pairs=7、gear_ratio=2、direction=-1、tachometer_counts=-168 なら 4π rad です
ros2_control の state interface には変換後の関節単位を渡します
位置制御以外の position state はこの積算角、位置制御では STATUS4 の単回転角です
JointState は joint_state_broadcaster がこの interface から作成します
速度・角度は float64、電流・duty・電荷・電力量・温度・電圧の 10 項目は float32 です
制御・オドメトリに使う値の精度を維持し、監視用の値だけメッセージで小さくします
CAN の電流・温度・電圧は 0.1、duty は 0.001 刻みで、float32 で監視できます
電荷・電力量は積算値が大きくなると float32 の丸め幅も大きくなります
内部のデコード・計算と ros2_control の state interface は double を維持します

### 時刻と欠測値

VescState は stamp を直接持ち、header や入れ子の state はありません
各 STATUS は独立して届くため、stamp は全項目の同時測定を表しません
各項目の鮮度は配信時の単調時刻とモータの feedback_timeout で判定します

- 速度・角度は float64、監視値は float32、どちらも未受信・期限切れは NaN
- position_rad は STATUS5 の鮮度に従い、未受信・期限切れでは NaN
- fault_code は int16、未受信・期限切れは -1、正常応答は 0、本体の故障は正のコード

フィードバックを使う前に std::isfinite() / math.isfinite() で欠測値を検査します
故障コードは uint8 の応答値 0〜255 と未取得値 -1 を保持できます
ドライバの通信途絶などは本体の故障番号へ変換せず、/vesc/event に停止理由を配信します

STATUS2/3/6 が未受信でも、モードに必要なフィードバックが揃えば制御を開始できます
全 STATUS を同じ高頻度で送信する必要はありません
各 STATUS の浮動値は期限切れで NaN となり、STATUS5 が古ければ position_rad も NaN です

### 削除した未取得項目

旧定義の avg_id、avg_iq、ntc_temp_mos1〜3、avg_vd、avg_vq、distance_traveled の 8 項目は、
このドライバが取得せず常に欠測値だったため削除しました
未使用の frame_id と、VescStateStamped の包みも廃止しました
公開メッセージの erpm・tachometer_counts・tachometer_fresh・motor_name・controller_id は削除しました
速度と角度は関節単位、識別は motor_id にまとめ、PID 角度も pid_pos_deg から pid_pos_rad に変更しました
生の ERPM・タコメータ・CAN ID・名前は内部のプロトコル処理と設定に残しています

## 異常イベント

異常は `error_topic`（既定 `/vesc/event`）へ VescEvent で即時通知します
型は次の 2 項目だけです

```text
uint8 motor_id
string message
```

不正指令・CAN エラーフレームはコールバック、期限切れは CAN 周期で検出した直後に配信します
本体の異常は該当 motor_id へ `VESC fault_code=4 (ABS_OVER_CURRENT)` のような文字列を通知します
故障名は [VESC 6.06 の公式 enum](https://github.com/vedderb/bldc/blob/94b305ec124b67b0161f2d3d85c4d5b35e48edc5/datatypes.h#L132-L163) に対応し、未知のコードは数値と UNKNOWN を表示します
PC 側の異常は影響する全モータへ ID ごとに同じ停止理由を通知します
正常コード 0 はイベントを配信しません
同じ本体の故障コードは繰り返さず、変化・正常応答後の再発を通知します
QoS は Reliable / TransientLocal で、登録モータ数分の直近イベントを保持します
全台停止時は全 ID のイベントを後から取得でき、現在の本体コードは周期の VescState で確認できます
イベントは履歴であり、復旧完了を表す通知は配信しません
問い合わせの既定周期は 100 ms、故障応答を受信した処理から即時配信します
故障コードの応答先ID・中継フィルタは [CANの故障問い合わせと再起動](#canの故障問い合わせと再起動) を参照してください

## CANの故障問い合わせと再起動

故障番号は [COMM_GET_VALUES_SELECTIVE](https://github.com/vedderb/bldc/blob/94b305ec124b67b0161f2d3d85c4d5b35e48edc5/comm/commands.c#L358-L453) のbit 15を指定して取得します
要求と応答はCAN_PACKET_PROCESS_SHORT_BUFFER（8）の拡張フレームです
応答先は内部定数 kReplyCanId=254 に固定し、YAML・URDFでのホストID設定は不要です
通常ノードとhardware pluginではモータにCAN ID 0〜253を使い、254は応答先に予約します
通常の電流・速度・位置・duty指令は、宛先のcontroller_idだけで送信できます

| 処理 | CAN ID | DLC | 内容 |
|---|---|---|---|
| 故障問い合わせ | (8 << 8)とVESCのcontroller_idをOR | 7 | COMM_GET_VALUES_SELECTIVE=50、mask=0x00008000 |
| 故障応答 | (8 << 8)と254をOR | 8 | data[0]にVESCのID、data[7]に故障番号 |
| 再起動 | (8 << 8)とVESCのcontroller_idをOR | 3 | 254、2、COMM_REBOOT=29 |

故障応答のCAN IDは0x8FEです
IDの組み立ては [公式のcomm_can_send_buffer](https://github.com/vedderb/bldc/blob/94b305ec124b67b0161f2d3d85c4d5b35e48edc5/comm/comm_can.c#L389-L419) に対応しています
STATUSだけを通すブリッジには、故障応答の受信フィルタも追加します
再起動のコマンド番号は [公式のCOMM_REBOOT](https://github.com/vedderb/bldc/blob/94b305ec124b67b0161f2d3d85c4d5b35e48edc5/datatypes.h#L921-L925) です
再起動ペイロードは FE 02 1D です

fault_poll_period_ms（既定100）はfeedback_timeout_msより短く設定します
0にすると問い合わせを無効化します
故障応答がない場合はfault_code=-1とし、必要なSTATUSによる制御を継続します
現在の取得方法は、各CAN IDが独立したVESCに対応する構成を対象にしています
VESC 6.06のdual-motor基板は第2モータの応答にも基板の主IDが付くため、対象外です
CANのis_errorはブリッジからのバス異常で、本体の故障番号とは分けて扱います

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

通常ノードの状態購読は /vesc/state の vesc_can_msgs/msg/VescStateArray に変更します
states[] の各要素を指令と同じ motor_id で識別します
hardware のモータ別トピックは vesc_can_msgs/msg/VescState を使用します
旧型の互換エイリアスはありません

| 旧アクセス | 新アクセス |
|---|---|
| msg.header.stamp | msg.stamp |
| msg.state.controller_id / msg.controller_id / msg.motor_name | msg.motor_id、設定の logical_id に合わせる |
| msg.state.speed / msg.erpm | msg.velocity_rad_s、関節 rad/s に変換済み |
| msg.state.current_motor / current_input | msg.motor_current_a / input_current_a |
| msg.state.temp_fet / temp_motor | msg.temp_fet_c / temp_motor_c |
| msg.state.pid_pos_now / msg.pid_pos_deg | msg.pid_pos_rad、PID 関節 rad に変換済み |
| msg.state.voltage_input | msg.input_voltage_v |
| msg.state.displacement / msg.tachometer_counts | msg.position_rad、積算関節 rad に変換済み、NaN を検査 |
| msg.state.charge_drawn / charge_regen | msg.charge_drawn_ah / charge_regen_ah |
| msg.state.energy_drawn / energy_regen | msg.energy_drawn_wh / energy_regen_wh |
| msg.state.fault_code | msg.fault_code、int16 に変更 |

vesc_can_msgs と購読側を同じ定義で再ビルドし、/vesc/event の購読型を VescEvent に変更します
通常ノード名はvesc_can_driverで、再起動サービスは /vesc_can_driver/reboot です
ros2_control の URDF には全 joint に一意の logical_id を追加します
通常ノードと同じモータなら YAML と URDF の logical_id を揃えます
通常ノードの指令、再起動サービス、CAN の形式、ros2_control の state / command interface は維持します
通常ノードは VescStateArray を周期配信し、異常を検出したときだけ /vesc/event に通知します

## C++ヘッダの変更

共通publisherと状態変換は `vesc_can_driver/vesc_node.hpp` にまとめました
以前の `vesc_state.hpp` を使っている場合はincludeを変更し、CMakeでは `vesc_can_driver::node` にリンクします
トピックの既定値と設定例は相対名になり、namespaceを指定するとその配下に配置されます
namespaceなしの起動では従来の `/vesc/commands`・`/vesc/state`・`/vesc/event` を使用します
