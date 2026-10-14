# vesc_can_msgs

VESC CAN ドライバの独自 msg と srv をまとめたインターフェースパッケージです
通常ノードは VescStateArray、ros2_control のモータ別トピックは VescState を使用します
配列の各要素も同じ VescState です
上位ノードは `vesc_can_msgs` に依存すれば指令・状態・エラー・再起動の型を使用でき、ドライバ本体への依存は不要です
このパッケージのメッセージ依存は `builtin_interfaces` だけです

| 型 | 内容 |
|---|---|
| [msg/VescCommand](msg/VescCommand.msg) | 論理 motor_id と float32 value |
| [msg/VescCommandArray](msg/VescCommandArray.msg) | 生成時刻 stamp と VescCommand[] commands |
| [msg/VescState](msg/VescState.msg) | 配信時刻 stamp、論理 motor_id、変換済みの速度・角度、監視値と故障コード |
| [msg/VescStateArray](msg/VescStateArray.msg) | 配信時刻 stamp と全モータの VescState[] states |
| [msg/VescEvent](msg/VescEvent.msg) | 論理 motor_id とエラー文字列 message |
| [srv/Reboot](srv/Reboot.srv) | 論理 motor_id → bool success、string message |

## 指令と再起動

motor_id は logical_id に対応し、指令・状態・エラーで共通です
通常ノードは YAML、ros2_control は URDF の各 joint に logical_id を指定します
0〜255 の範囲で一意にし、CAN の controller_id は別に設定します
モードは起動時に固定し、value の単位は電流 [A]、関節速度 [rad/s]、関節位置 [rad]、duty 比のいずれかです
モード名は current / velocity / position / duty_cycle / current_velocity です
duty_cycle は関節方向の比率 [-1, 1] で、0.5 は 50% を表します
current_velocity の value も関節速度 [rad/s] です
配列の stamp は送信側での生成時刻で、タイムアウトは受信側の単調時刻で判定します
stamp が 0 の指令も受け付けます

再起動は通常ノードの ~/reboot に motor_id を指定します
success は要求受理を表し、本体の再起動完了の確認ではありません

## 状態

VescState は CAN で取得する項目から作成した独自定義です
F1TENTH の VescState / VescStateStamped を使用しません
stamp を直接持つため、Stamped 型で包む必要はありません

| 取得元 | フィールド |
|---|---|
| STATUS1 | velocity_rad_s、motor_current_a、duty_cycle |
| STATUS2 | charge_drawn_ah、charge_regen_ah |
| STATUS3 | energy_drawn_wh、energy_regen_wh |
| STATUS4 | temp_fet_c、temp_motor_c、input_current_a、pid_pos_rad |
| STATUS5 | position_rad、input_voltage_v |
| selective 問い合わせ | fault_code |

stamp は配信時刻で、すべての値の同時測定を表しません
通常ノードでは `/vesc/state` の states[] を motor_id で識別します
ros2_control のモータ別トピックにも同じ motor_id を入れます
motor_name、CAN の controller_id、生の erpm と tachometer_counts は公開メッセージから削除しました
velocity_rad_s は極対数・減速比・方向を適用した関節速度 [rad/s] です
pid_pos_rad は減速比・方向・zero_offset_rad を適用した PID 関節角 [rad] です
PID 角度には VESC 側の機械角設定・校正が必要です
position_rad は全モード共通の積算関節角度 [rad] です
tachometer_counts × 2π / (6 × pole_pairs × gear_ratio) × direction で計算し、2π で折り返しません
起動時の自動ゼロ合わせや、PID 用の zero_offset_rad は適用しません
未受信・期限切れでは position_rad も NaN です
ros2_control では位置制御以外の position state interface も同じ積算角を返します
位置制御の position state interface は STATUS4 の PID 角度を使います

未受信・期限切れの浮動値は NaN、fault_code は -1 です
fault_code=0 は VESC が応答した「故障なし」を表します
velocity_rad_s、pid_pos_rad、position_rad は float64 を使います
電流・duty・電荷・電力量・温度・電圧の 10 項目は監視用の float32 です
電荷・電力量の大きな積算値は float32 で丸め幅が増えるため、精密な積算には内部の double 値を使います
ros2_control の state interface と内部計算は double を維持します
現在のフィードバックとして使う値は isfinite() で NaN を検査します
STATUS6 の ADC / PPM は共通ドライバ内で保持します
詳細は [メッセージ対応表](../docs/messages.md) を参照してください

## エラー

/vesc/event は VescEvent の motor_id と message だけを配信します
本体の故障は `VESC fault_code=4 (ABS_OVER_CURRENT)` のような名称付き文字列です
PC 側の全台停止は対象となる全 ID へ停止理由を通知します
正常コード 0 はイベントを配信せず、同じ故障の繰り返し通知を抑えます
Reliable / TransientLocal で登録モータ数分の直近イベントを保持します
現在の本体コードは周期の VescState.fault_code で確認します

## 利用例

利用側の package.xml に `<depend>vesc_can_msgs</depend>` を追加します
C++ の rclcpp ノードでは次を指定します

```cmake
find_package(vesc_can_msgs REQUIRED)
target_link_libraries(my_node PRIVATE ${vesc_can_msgs_TARGETS})
```

```cpp
#include "vesc_can_msgs/msg/vesc_command_array.hpp"
#include "vesc_can_msgs/msg/vesc_event.hpp"
#include "vesc_can_msgs/msg/vesc_state.hpp"
#include "vesc_can_msgs/msg/vesc_state_array.hpp"
#include "vesc_can_msgs/srv/reboot.hpp"
```

```python
from vesc_can_msgs.msg import VescCommand, VescCommandArray, VescEvent, VescStateArray

cmd = VescCommandArray(
    stamp=node.get_clock().now().to_msg(),
    commands=[VescCommand(motor_id=10, value=2.0)],
)

def on_state(msg: VescStateArray):
    for state in msg.states:
        node.get_logger().info(
            f"{state.motor_id}: velocity={state.velocity_rad_s} rad/s, "
            f"position={state.position_rad} rad, fault={state.fault_code}"
        )

state_sub = node.create_subscription(
    VescStateArray, "/vesc/state", on_state, 10
)

def on_event(msg: VescEvent):
    node.get_logger().error(f"motor_id={msg.motor_id}: {msg.message}")

event_sub = node.create_subscription(VescEvent, "/vesc/event", on_event, 10)
```

独自 msg / srv とドライバはルートの MIT ライセンスを使用します
パッケージにはルートのライセンス原本をインストールします
