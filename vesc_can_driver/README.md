# vesc_can_driver

VESCのCANフレーム処理、通信管理、通常ノードをまとめたパッケージです
通常ノードは複数モータの指令を受け取り、状態を配信します
ros2_controlを使う場合も、CAN通信と単位変換はこのパッケージを共用します

## 設定例

| ファイル | 内容 |
|---|---|
| [examples/standalone.yaml](examples/standalone.yaml) | 駆動の速度制御、操舵の位置制御 |
| [examples/five_modes.yaml](examples/five_modes.yaml) | 速度・位置・電流・速度＋電流・duty比の5台 |

設定例をコピーし、`motors` の一覧と各モータの設定を編集します
モードは起動時に固定されるので、変更したらノードを再起動します

| 設定 | 意味 |
|---|---|
| logical_id | 上位ノードが使うmotor_id、0〜255で一意にする |
| controller_id | VESC Toolで設定したCAN ID、0〜254で一意にする |
| control_mode | velocity / current / position / duty_cycle / current_velocity |
| pole_pairs | 極対数、14極なら7 |
| gear_ratio / direction | モータ回転数÷関節回転数、正方向は+1か-1 |
| max_rpm / rpm_slew_rate | 機械RPMの速度上限と変化率、ROS指令の単位はrad/s |
| max_current_a / max_duty | 電流[A]、duty比の指令上限 |
| require_tachometer | trueならSTATUS5の受信を動作条件にする |
| feedback_timeout_ms / command_timeout_ms | フィードバックと指令の期限 |

起動・指令の例は [ルートREADME](../README.md#通常ノードで使う)、全モードの設定は [モードの説明](../docs/motor-modes.md#設定例) にあります

```bash
ros2 launch vesc_can_driver vesc_standalone.launch.py config:=/path/to/motors.yaml
```

## トピックとサービス

| 名前 | 型 | 用途 |
|---|---|---|
| /vesc/commands | vesc_can_msgs/msg/VescCommandArray | motor_idと指令値の配列 |
| /vesc/state | vesc_can_msgs/msg/VescStateArray | 全モータの状態、既定50 Hz |
| /vesc/event | vesc_can_msgs/msg/VescEvent | motor_idとエラー文字列 |
| ~/reboot | vesc_can_msgs/srv/Reboot | 指定したmotor_idのVESCを再起動 |

指令は指定されたモータだけ更新し、未送信の値は最新値で上書きします
CAN送信は100 Hzのタイマーで行い、指令トピックの配信周期とは分けています
未指定モータの指令期限は更新しません

状態の速度・角度は関節単位で、未受信・期限切れはNaNです
`state_topic: ""` にすると、`~/motors/<モータ名>/state` へ個別配信できます
通常は全台を読むため、配列でまとめています
型・単位・欠測値の詳細は [メッセージの説明](../docs/messages.md) を参照してください

## 即時の異常通知

VESCの故障は、該当motor_idへ `VESC fault_code=4 (ABS_OVER_CURRENT)` のように通知します
PC側の全台停止は、影響する各motor_idへ同じ停止理由を通知します
同じ異常の繰り返しを抑え、正常コード0はイベントにしません
イベントはReliable / TransientLocalで、登録モータ数分の直近通知を保持します

```bash
ros2 topic echo /vesc/event vesc_can_msgs/msg/VescEvent --qos-durability transient_local
ros2 service call /vesc_can_ros2_control/reboot vesc_can_msgs/srv/Reboot '{motor_id: 1}'
```

再起動を受理すると全台の保存指令を破棄し、指定したVESCへ再起動フレームを1回送ります
500 msの待機後、新しい状態と全台への新しい指令から再開します
`success` は要求の受理を表します

## CANの接続

`can_tx_topic` と `can_rx_topic` をCANブリッジのトピックに合わせます
送信はReliable、受信はBestEffortです
中継器はCAN ID、拡張フラグ、DLC、ペイロードをそのまま通してください

VESC Toolで必要なSTATUSの周期送信を有効にします
速度・電流・duty比にはSTATUS1、位置制御にはSTATUS1と4、積算角にはSTATUS5が必要です
故障問い合わせも使うため、STATUSに加えてshort bufferの応答を中継します
`host_can_id`（既定254）は同じバスのVESCや他の問い合わせ元と重複させません
問い合わせの形式と受信IDは [CANの故障問い合わせ](../docs/messages.md#canの故障問い合わせと再起動) にまとめています

## C++ライブラリとして使う

| CMake target | ヘッダ | 役割 |
|---|---|---|
| vesc_can_driver::protocol | vesc_protocol.hpp | CANフレームの生成・デコード |
| vesc_can_driver::driver | vesc_driver.hpp | 設定・状態・制御・CAN通信 |
| vesc_can_driver::state | vesc_state.hpp | ROSの状態・エラー配信 |

フレーム処理だけなら `protocol` にリンクします

```cmake
find_package(vesc_can_driver REQUIRED)
target_link_libraries(my_can_node PRIVATE vesc_can_driver::protocol)
```

```cpp
#include "vesc_can_driver/vesc_protocol.hpp"

auto frame = vesc_can_driver::protocol::make_set_rpm_frame(11, 420);  // CAN ID 11、420 ERPM
```

`protocol` はROSノードを起動しません
処理の分担とスレッドの説明は [内部構成](../docs/architecture.md) を参照してください
