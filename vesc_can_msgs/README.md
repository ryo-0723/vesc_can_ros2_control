# vesc_can_msgs

上位ノードと通常ドライバで共用する指令メッセージと再起動サービスです
CAN ドライバ・ros2_control・VescState への依存を持ちません
上位パッケージは `vesc_can_msgs` に依存すれば指令型を使用できます

| 型 | フィールド |
|---|---|
| VescCommand | uint8 motor_id、float32 value |
| VescCommandArray | builtin_interfaces/Time stamp、VescCommand[] commands |
| srv/Reboot | uint8 motor_id → bool success、string message |

motor_id はドライバ YAML の logical_id に対応します
モードはドライバの起動時に固定し、value の単位は電流 [A]、関節速度 [rad/s]、関節位置 [rad] のいずれかです
stamp は送信側での生成時刻で、ドライバのタイムアウトは受信側の単調時刻で判定します
stamp が 0 の指令も受け付けます

利用側の package.xml に `<depend>vesc_can_msgs</depend>` を追加します
C++ の rclcpp ノードでは、通常の rclcpp の依存に加えて次を指定します

```cmake
find_package(vesc_can_msgs REQUIRED)
target_link_libraries(my_node PRIVATE ${vesc_can_msgs_TARGETS})
```

```cpp
#include "vesc_can_msgs/msg/vesc_command_array.hpp"
// cmd.stamp = node->now();
```

```python
from vesc_can_msgs.msg import VescCommand, VescCommandArray

cmd = VescCommandArray(
    stamp=node.get_clock().now().to_msg(),
    commands=[VescCommand(motor_id=10, value=2.0)],
)
```

状態は上流由来の `vesc_msgs/msg/VescStateStamped` を使用します
再起動サービスは通常ノードの ~/reboot に論理 motor_id を指定します
success は要求受理を表し、本体の再起動完了の確認ではありません
このパッケージは独自の指令型と再起動サービスを生成し、ルートの MIT ライセンス原本をインストールします
