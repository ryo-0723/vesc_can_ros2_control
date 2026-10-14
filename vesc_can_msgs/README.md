# vesc_can_msgs

VESC CAN ドライバの独自 msg と srv をまとめたインターフェースパッケージです
通常ノードと ros2_control は同じ状態型を使用します
上位ノードは `vesc_can_msgs` に依存すれば指令・状態・再起動の型を使用でき、ドライバ本体への依存は不要です
このパッケージのメッセージ依存は `builtin_interfaces` だけです

| 型 | 内容 |
|---|---|
| [msg/VescCommand](msg/VescCommand.msg) | 論理 motor_id と float32 value |
| [msg/VescCommandArray](msg/VescCommandArray.msg) | 生成時刻 stamp と VescCommand[] commands |
| [msg/VescState](msg/VescState.msg) | 配信時刻 stamp、CAN の controller_id、取得した状態値と故障コード |
| [srv/Reboot](srv/Reboot.srv) | 論理 motor_id → bool success、string message |

## 指令と再起動

motor_id は通常ノードの YAML の logical_id に対応します
モードは起動時に固定し、value の単位は電流 [A]、関節速度 [rad/s]、関節位置 [rad] のいずれかです
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
| STATUS1 | erpm、motor_current_a、duty_cycle |
| STATUS2 | charge_drawn_ah、charge_regen_ah |
| STATUS3 | energy_drawn_wh、energy_regen_wh |
| STATUS4 | temp_fet_c、temp_motor_c、input_current_a、pid_pos_deg |
| STATUS5 | tachometer_counts、tachometer_fresh、input_voltage_v |
| selective 問い合わせ | fault_code |

stamp は配信時刻で、すべての値の同時測定を表しません
controller_id は CAN の機器 ID です
購読先はモータ名ごとのトピックなので、上位ノードが CAN ID を指定する必要はありません
erpm と pid_pos_deg は極対数・減速比・方向を適用する前のモータ生値です
関節単位の状態は ros2_control の state interface から取得します

未受信・期限切れの浮動値は NaN、fault_code は -1 です
fault_code=0 は VESC が応答した「故障なし」を表します
tachometer_counts は符号付き int32 の最終受信値を保持し、未受信時は 0 です
現在のフィードバックとして使う場合は tachometer_fresh=true を確認します
STATUS6 の ADC / PPM は診断に配信します
詳細は [メッセージ対応表](../docs/messages.md) を参照してください

## 利用例

利用側の package.xml に `<depend>vesc_can_msgs</depend>` を追加します
C++ の rclcpp ノードでは次を指定します

```cmake
find_package(vesc_can_msgs REQUIRED)
target_link_libraries(my_node PRIVATE ${vesc_can_msgs_TARGETS})
```

```cpp
#include "vesc_can_msgs/msg/vesc_command_array.hpp"
#include "vesc_can_msgs/msg/vesc_state.hpp"
#include "vesc_can_msgs/srv/reboot.hpp"
```

```python
from vesc_can_msgs.msg import VescCommand, VescCommandArray, VescState

cmd = VescCommandArray(
    stamp=node.get_clock().now().to_msg(),
    commands=[VescCommand(motor_id=10, value=2.0)],
)

def on_state(msg: VescState):
    node.get_logger().info(f"ERPM={msg.erpm}, fault={msg.fault_code}")

state_sub = node.create_subscription(
    VescState, "/vesc_can_ros2_control/motors/drive_joint/state", on_state, 10
)
```

独自 msg / srv とドライバはルートの MIT ライセンスを使用します
パッケージにはルートのライセンス原本をインストールします
