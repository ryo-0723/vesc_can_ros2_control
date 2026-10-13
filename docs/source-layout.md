# ソースの構成

公開用のソースは 3 つの ROS パッケージに分かれています。ワークスペースの
`src` にリポジトリを配置すると、その下の各 `package.xml` が検出されます。
リポジトリ直下には `src` はなく、各パッケージの `src` が実装を持ちます。

| ファイル | 役割 | 使用先 |
|---|---|---|
| `vesc_can_driver/src/vesc_protocol.cpp` | CAN 指令の encode と STATUS1〜6 の decode | `vesc_can_driver::protocol` |
| `vesc_can_driver/src/driver_core.cpp` | 関節単位変換、制御モード、範囲・期限の検査、全台停止 | `vesc_can_driver::core` |
| `vesc_can_driver/src/can_transport.cpp` | ROS の CAN トピックと専用 executor、指令の受け渡し | `vesc_can_driver::transport` |
| `vesc_can_driver/src/vesc_state_conversion.cpp` | 受信した生値から `vesc_msgs` への変換 | `vesc_can_driver::state_conversion` |
| `vesc_can_driver/src/vesc_node.cpp` | トピック・サービスで動かす通常ノード | `vesc_node` |
| `vesc_can_ros2_control/src/vesc_system.cpp` | ros2_control の設定、lifecycle、read/write | `vesc_hardware` |
| `vesc_can_driver/test/support/fake_vesc_node.cpp` | CAN 指令に応答する模擬 VESC | `fake_vesc_node`、デモと統合テスト |

現在の `src` に未使用の `.cpp` はありません。protocol、core、transport、state_conversion は
別々のライブラリで、単独利用と通常ノード・hardware plugin 間の共通化に使います。
通常ノードは controller_manager を使わない利用者向けの入口です。
4 モードの選択・指令範囲の検査・電流と速度の送信処理は `driver_core.cpp` に集約し、
通常ノードと hardware plugin が共用します。

## ファイルを分ける基準

現在の分割は責務と依存関係に沿っています。`vesc_protocol` は CAN の形式だけを扱い、
`driver_core` はモータの設定・状態・4 モードの制御判断をまとめます。
`can_transport` はその共通ロジックへ ROS の通信と executor を接続し、
`vesc_node` と `vesc_system` はそれぞれ通常ノードと ros2_control の入口になります。
設定の読み込みは YAML と URDF で入口が異なりますが、範囲検査と制御判断は共用します。

`vesc_state_conversion` を独立させることで、上流 `vesc_msgs` への変換を
制御ロジックから切り離して検証できます。変換処理は小さいものの、独立した役割があります。
通常ノードは状態配信で利用し、hardware plugin は transport を利用します。
今回はこの分割を維持し、モードごとにファイルを増やしてはいません。

## C++ 変数の命名

関節へ変換済みの位置・速度には `joint_` と単位を、VESC の生値には
`motor_` または `electrical_` と単位を付けます。例えば、
`joint_velocity_rad_s` は関節の rad/s、`electrical_rpm` は VESC の ERPM です。
電流の `joint_current_a` は関節方向で符号を定めたモータ電流 [A] を表し、トルクではありません。
時間長は `std::chrono::milliseconds`、受信時刻は steady clock の `TimePoint` を使用します。
受信済みの `has_status_1` と、鮮度判定後の `status_1_fresh` は区別します。

公開前の整理として、C++ メンバ名を次のように変更しました。

| 対象 | 以前の名前 | 現在の名前 |
|---|---|---|
| MotorConfig | `mode` | `control_mode` |
| MotorConfig | `max_velocity` | `max_joint_velocity_rad_s` |
| MotorConfig | `velocity_slew_rate` | `joint_velocity_slew_rate_rad_s2` |
| MotorConfig | `min_position` / `max_position` | `min_joint_position_rad` / `max_joint_position_rad` |
| MotorState | `position` / `velocity` | `joint_position_rad` / `joint_velocity_rad_s` |
| MotorState / Status1 | `current_a` | `motor_current_a` |
| MotorState / Status1 | `electrical_rpm` / `erpm` | `electrical_rpm` |
| MotorState / Status4 | `pid_position_deg` / `position_deg` | `motor_pid_position_deg` |
| Status5 | `tachometer` | `tachometer_counts` |
| MotorState | `have_status1` / `status1_time` | `has_status_1` / `status_1_received_at`（2〜6 も同様） |
| TransportOptions | `tx_topic` / `rx_topic` / `period` | `can_tx_topic` / `can_rx_topic` / `send_period` |

C++ の公開ヘッダを直接使う利用者はメンバ参照の更新が必要です。
ROS トピック・サービス・パラメータ、URDF の interface 名、上流メッセージの項目名は維持します。
例えば URDF の `max_velocity` は引き続き受け付け、内部の `max_joint_velocity_rad_s` へ読み込みます。

## 起動例とテスト用ファイル

模擬ノードは実機用実装と区別するため `src` から `test/support` へ移しました。
`use_mock:=true` と統合テストが利用するため削除していません。
デモを利用できるよう、`BUILD_TESTING=OFF` でもビルド・インストールします。

`vesc_can_driver/test` はプロトコル・共通ドライバ・通信・状態メッセージ変換のテスト、
`vesc_can_ros2_control/test` は設定と実プロセスを起動する統合テストです。
`config`、`launch`、`urdf` はインストール後の起動例に必要です。
`vesc_msgs` は `VescState.msg` と `VescStateStamped.msg` の 2 個と、それを生成する
ローカルの CMake / package.xml、BSD ライセンス・出典を持ちます。IMU は含めません。
ライセンス原本はルートの MIT と `vesc_msgs/LICENSE` の BSD の計 2 個です。
ドライバの各パッケージにはルートの MIT 原本をインストールします。

ビルド出力の `build`、`install`、`log` と Python キャッシュは `.gitignore` で除外します。
ソース配布時には含めません。
