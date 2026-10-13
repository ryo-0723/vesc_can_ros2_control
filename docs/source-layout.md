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

## driver_core の命名と関数の分担

局所変数には `cfg`（設定）、`state`（状態）、`cmd`（指令）、`idx`（添字）、
`pos`（位置）、`vel`（速度）を使います。型と関数の文脈で分かる `motor_` / `joint_` の
重複を減らし、物理量の単位は名前または構造体のコメントで明記します。
`MotorState::pos_rad` / `vel_rad_s` は関節へ変換済みの値、
`electrical_rpm` / `motor_current_a` / `pid_pos_deg` は VESC の生値です。
protocol 側のフィールド名とデコード処理は維持しています。

| 対象 | 整理前 | 現在 |
|---|---|---|
| MotorConfig | `control_mode` | `mode` |
| MotorConfig | `max_joint_velocity_rad_s` | `max_vel_rad_s` |
| MotorConfig | `joint_velocity_slew_rate_rad_s2` | `vel_slew_rad_s2` |
| MotorConfig | `min_joint_position_rad` / `max_joint_position_rad` | `min_pos_rad` / `max_pos_rad` |
| MotorConfig | `rpm_control_threshold_rpm` | `rpm_threshold`（モータの機械 RPM） |
| MotorState | `joint_position_rad` / `joint_velocity_rad_s` | `pos_rad` / `vel_rad_s` |
| MotorState | `motor_pid_position_deg` | `pid_pos_deg` |
| MotorState | `has_status_1` と `status_1_received_at` | `status1_at`（2〜6 も同様） |
| DriverCore 内の指令 | `has_command` と `received_at` | `received_at` |

受信時刻は `std::optional<TimePoint>` にまとめました。未受信は `nullopt`、
受信済みはその時刻を持ちます。これによりフラグと時刻の更新が一箇所になり、
`TimePoint{}` で受信した場合も未受信と混同しません。
`feedback_fresh()` は未受信・未来の時刻・期限切れを検査し、期限ちょうどは有効です。

| 関数 | 役割 |
|---|---|
| `parse_control_mode` / `command_interface` / `resolve_control_mode` | モード名の解釈、interface の対応、両者の整合性検査 |
| `vel_to_erpm` / `erpm_to_vel`、`pos_to_deg` / `deg_to_pos` | 関節単位と VESC 単位の変換 |
| `apply_direction` | 電流の符号変換。`direction` が ±1 なので両方向で共用 |
| `validate_configs` | モータ一覧の存在・名前と ID の重複検査。各モータの範囲・単位・時間の検査は内部 `validate_config` に委譲 |
| `state_ready` / `command_is_valid` | モードごとの必要なフィードバックと指令範囲の検査 |
| `check_commands`（内部） | 全台のフィードバックと指令期限を指令生成前に検査 |
| `make_cmd`（内部） | 4 モードの指令生成へ振り分け |
| `make_current_cmd` / `make_velocity_cmd`（内部） | 電流指令と速度のランプを共用 |
| `make_hybrid_cmd`（内部） | 電流起動から速度制御への移行 |
| `disable` / `stop_on_error` | 指令状態の初期化を共用し、異常停止は理由とラッチを保持 |

通常ノードの起動時の設定検査は `DriverCore` のコンストラクタに集約しました。
ros2_control は configure 前に設定を検査する必要があるため、on_init でも共通検査を呼びます。
浮動小数点の NaN、CAN 指令の整数範囲、単回転の位置範囲、フィードバック・指令・起動の
期限検査は、4 モードの動作に必要な検査として維持しています。

C++ の公開ヘッダを直接使う利用者はメンバ名・関数名と受信時刻の参照の更新が必要です。
ROS トピック・サービス・パラメータ、URDF の interface 名、上流メッセージの項目名は維持します。
例えば URDF の `max_velocity` は引き続き受け付け、内部の `max_vel_rad_s` へ読み込みます。

## 通信・ROS・起動設定の変数名

他のファイルも `cfg` / `state` / `cmd` の方針に揃えました。通信設定には `opts`、
ROS メッセージには `msg`、publisher / subscription には `pub` / `sub`、
パラメータには `params`、添字には `motor_idx` / `joint_idx` を使います。
配列を持つメンバは `configs_` / `states_` / `cmds_`、送受信は `tx` / `rx` で表します。
mutex と停止・異常フラグは、同期やラッチの意味が分かる名前を使います。

| 整理前 | 現在 | 意味 |
|---|---|---|
| `motor_config` / `transport_options` | `cfg` / `opts` | モータ設定 / 通信設定 |
| `can_tx_publisher_` / `can_rx_subscription_` | `tx_pub_` / `rx_sub_` | CAN 指令の配信 / 状態の受信 |
| `pending_commands_` / `command_frames_` | `pending_cmds_` / `cmd_frames_` | 未処理の指令 / 送信する CAN フレーム |
| `motor_command_subscriptions_` / `motor_state_publishers_` | `cmd_subs_` / `state_pubs_` | モータごとの指令購読 / 状態配信 |
| `hardware_parameters` / `hardware_info`（局所変数） | `hw_params` / `hw_info` | ros2_control の設定 / hardware 情報 |
| `robot_description_tree` / `can_topic_parameters` | `urdf_tree` / `mock_params` | URDF の木 / 模擬ノードの設定 |
| `package_share_dir` | `pkg_dir` | 起動するパッケージの share ディレクトリ |

通常ノードでは `steady_now` を期限判定用の steady clock、`stamp` をメッセージに付ける
ROS 時刻に使います。模擬ノードの `pos_rad` はモータ角度 [rad] で、
`MotorState::pos_rad` は関節角度 [rad] です。それぞれの型と変換箇所で区別します。
Python のテストでも `vel_pub` / `pos_pub` / `current_pub` / `hybrid_pub` を使い、
指令値と publisher を区別します。

公開 C++ 構造体 `TransportOptions` の `can_tx_topic` / `can_rx_topic` は
`tx_topic` / `rx_topic` に短縮しました。直接使う C++ コードはメンバ参照の更新が必要です。
ROS 設定のキーは引き続き `can_tx_topic` / `can_rx_topic`、送信周期の C++ メンバは `send_period` です。
設定キー、トピック名、標準・上流メッセージの項目名は維持しています。

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
