# vesc_can_msgs

指令・状態・エラーのmsgと、再起動のsrvをまとめたパッケージです
上位ノードはこのパッケージだけに依存して通信できます

## メッセージ一覧

| 型 | 内容 |
|---|---|
| [VescCommand](msg/VescCommand.msg) | motor_idとfloat32の指令値value |
| [VescCommandArray](msg/VescCommandArray.msg) | 生成時刻stampとcommands配列 |
| [VescState](msg/VescState.msg) | 配信時刻stamp、motor_id、状態値、fault_code |
| [VescStateArray](msg/VescStateArray.msg) | 配信時刻stampと全モータのstates配列 |
| [VescEvent](msg/VescEvent.msg) | motor_idとエラー文字列message |
| [Reboot](srv/Reboot.srv) | motor_idを指定、successとmessageを返す |

motor_idは論理IDです
通常ノードはYAML、ros2_controlはURDFの `logical_id` に対応し、CAN IDはドライバ内で管理します
指令値の単位は起動時のモードに従います → [モードと単位](../docs/motor-modes.md)

状態の速度は `velocity_rad_s`、積算角は `position_rad`、PID角は `pid_pos_rad` です
速度・角度はfloat64、監視値はfloat32、欠測・期限切れはNaNです
fault_codeは未取得・期限切れが-1、正常が0、故障が正の番号です
stampは配信時刻で、全STATUSの同時測定を表すものではありません → [各項目の説明](../docs/messages.md)

## 利用例

利用側のpackage.xmlに `<depend>vesc_can_msgs</depend>` を追加します

C++の場合は次のtargetにリンクします

```cmake
find_package(vesc_can_msgs REQUIRED)
target_link_libraries(my_node PRIVATE ${vesc_can_msgs_TARGETS})
```

```cpp
#include "vesc_can_msgs/msg/vesc_command_array.hpp"
#include "vesc_can_msgs/msg/vesc_state_array.hpp"
#include "vesc_can_msgs/msg/vesc_event.hpp"
#include "vesc_can_msgs/srv/reboot.hpp"
```

Pythonでは次のように読み書きできます

```python
import math
from vesc_can_msgs.msg import VescCommand, VescCommandArray, VescEvent, VescStateArray

cmd = VescCommandArray(
    stamp=node.get_clock().now().to_msg(),
    commands=[VescCommand(motor_id=1, value=1.0)],
)


def on_state(msg: VescStateArray):
    for state in msg.states:
        if math.isfinite(state.position_rad):
            node.get_logger().info(f"motor_id={state.motor_id}: {state.position_rad} rad")


def on_event(msg: VescEvent):
    node.get_logger().error(f"motor_id={msg.motor_id}: {msg.message}")


command_pub = node.create_publisher(VescCommandArray, "/vesc/commands", 10)
state_sub = node.create_subscription(VescStateArray, "/vesc/state", on_state, 10)
event_sub = node.create_subscription(VescEvent, "/vesc/event", on_event, 10)
command_pub.publish(cmd)
```

実際に動かすときは、登録した全モータに指令を期限内で送り続けます
配列に含まれないモータの指令期限は更新されません
再起動サービスのsuccessは要求の受理を表します
