# ソースの構成

ドライバ本体は、プロトコル・CAN 管理・通常ノード・共通状態出力の 4 単位です。
通常ノードと ros2_control は同じ CAN 管理と状態出力を使います。

| ファイル | 役割 | 利用先 |
|---|---|---|
| `vesc_can_driver/src/vesc_protocol.cpp` | VESC 6 の CAN ID・ペイロード形式、動作指令・故障問い合わせの encode、STATUS1〜6・故障応答の decode | `vesc_can_driver::protocol` |
| `vesc_can_driver/src/vesc_driver.cpp` | モータ設定、単位変換、状態更新、5 モードの指令、CAN の周期送受信・停止管理 | `vesc_can_driver::driver` |
| `vesc_can_driver/src/vesc_node.cpp` | 通常ノードのパラメータ、指令トピック、再起動サービス、モータ状態と診断 | `vesc_node` |
| `vesc_can_driver/src/vesc_state.cpp` | VescState への変換、モータごとの状態配信、即時の異常通知 | `vesc_can_driver::state` |
| `vesc_can_ros2_control/src/vesc_system.cpp` | ros2_control の設定、lifecycle、state / command interface の受け渡し | `vesc_hardware` |

公開ヘッダは `vesc_protocol.hpp`、`vesc_driver.hpp`、`vesc_state.hpp` の 3 個です。
通常ノードは実行ファイルなので公開ヘッダを持ちません。
`vesc_system.hpp` は別パッケージの hardware plugin のヘッダです。
独自 msg / srv は独立した `vesc_can_msgs` にまとめます
`msg/` は VescCommand、VescCommandArray、VescState、`srv/` は Reboot の定義です
上位ノードはドライバ本体に依存せず、指令・状態・再起動の型を使えます

## 共用する範囲

`vesc_driver.hpp/.cpp` に `MotorConfig`、`MotorState`、共通の変換・検査、
`DriverCore`、`CanTransport` をまとめています。
`DriverCore` は状態更新とモードごとの制御判断、`CanTransport` は ROS の CAN トピック、
専用 executor、最新指令の受け渡し、周期送信を担当します。
通常ノードと hardware plugin はどちらも `CanTransport` を使います。

`vesc_state.hpp/.cpp` は `make_vesc_state()` と `MotorStatePublisher` を持ち、
通常ノードと hardware plugin が同じ項目・単位・未取得値の扱いで状態を配信します。
ros2_control の read/write には ROS 配信を入れません。
`vesc_system` は通常ノードを使わず、共通の driver / state ライブラリへ接続します。

hardware component 使用時は controller_manager が `VescSystem` を読み込みます
`VescSystem` が内部の CAN I/O ノードを持ち、通常ノード `vesc_node` は起動しません
関節状態のトピックは joint_state_broadcaster が state interface から作成します

`vesc_protocol` は形式の処理だけを扱い、ROS ノードや executor を起動しません。
protocol target は rclcpp / hardware_interface へリンクしません。
driver target は CAN 通信のため rclcpp を使いますが、ros2_control への依存はありません。

## vesc_driver の構成と関数

`vesc_driver.cpp` は次の順に並べています。

1. モードの選択と単位変換。
2. 設定・指令範囲・フィードバック鮮度の検査。
3. STATUS1〜6 の受信と状態更新。
4. 有効化・停止・指令期限と 5 モードのフレーム生成。
5. ROS の CAN 通信、指令の受け渡し、周期送信。

5 モードは `DriverCore::make_frame()` の一つの分岐で扱います。
電流起動後は速度モードと同じ速度ランプへ進み、電流モード・電流起動・停止は
内部の `make_current_frame()` を共用します。全台の事前検査は `check_commands()`、
送信時刻の付与と停止要求の反映は `CanTransport::publish_frames()` が担当します。

## ready / activate とパケット処理の違い

`ready()` と `activate()` は PC 側の制御管理で、VESC に送る CAN コマンドではありません。
`ready()` は全モータの必要な STATUS が新鮮で、位置モードなら位置が範囲内かを確認します。
`activate()` は動作指令を許可し、異常と以前の指令をリセットします。
通常ノードは新しい指令の受信時に内部で開始し、ros2_control は lifecycle の activate で開始します

パケットの作成・デコードだけなら `vesc_protocol` を直接使い、ready / activate は不要です。
`DriverCore::receive()` も有効化を要求せず、無効・異常停止中にも状態を更新します。
通信断・指令期限切れ・不正指令・電流起動失敗は全台のゼロ電流送信に切り替え、
通常ノードでは reboot サービスによる再起動後の新しい指令で復旧します
CanTransport::reboot() は CAN executor に要求を渡し、protocol::make_reboot_frame() で指定した 1 台を再起動します

このドライバの動作指令は SET_DUTY / SET_CURRENT / SET_RPM / SET_POS の 4 種類です
SET_DUTY の比率は direction で符号を合わせ、protocol が 100000 倍して格納します
`current_velocity` は SET_CURRENT で起動して SET_RPM へ移るホスト側の制御モードです。

## 通信・ROS・起動設定の変数名

局所変数は `cfg`（設定）、`opts`（通信設定）、`state`（状態）、`cmd`（指令）、
`idx`（添字）、`pos`（位置）、`vel`（速度）を使い、物理量の単位を名前に残します。
`MotorState::pos_rad` / `vel_rad_s` は関節へ変換済みの値、
`electrical_rpm` / `motor_current_a` / `pid_pos_deg` は VESC の生値です。

STATUS と指令の受信時刻は `std::optional<TimePoint>` です。
未受信は `nullopt` とし、時刻ゼロでの受信と区別します。
`steady_now` は期限判定、`stamp` はメッセージに付ける ROS 時刻です。
通常ノードの指令は `/vesc/commands` の VescCommandArray に統一しています
logical_id は通常ノードで保持し、共通の MotorConfig には CAN の controller_id だけを渡します
ROS 設定キー、state トピック名、interface 名は維持しています
VescState は取得する CAN 生値から独自定義にし、項目名に単位を付けています
通常ノードの control_mode / command_interface は起動後の変更を拒否します

## 周期と終了処理

CAN 指令は各モータ 100 Hz、VescState は両入口で 50 Hz です
通常ノードは VescState と診断を配信します
JointState は ros2_control の joint_state_broadcaster が 50 Hz で配信します
ros2_control の state interface は controller_manager の 100 Hz で更新します。
CAN と ROS 状態は独立したタイマーで扱い、モータごとのスレッドは追加しません。

`CanTransport::shutdown()` は専用 executor を停止して join した後に停止指令を送ります。
hardware plugin はその後に状態タイマーと publisher を破棄します。
異常通知コールバックも終了してから publisher を破棄し、read/write から ROS 通知を配信しません

## C++ 利用側の更新

共通型・制御・通信は `vesc_driver.hpp`、VescState の変換・配信は `vesc_state.hpp` を include します。
旧 core / transport target は `vesc_can_driver::driver`、
旧 state_conversion / state_publisher target は `vesc_can_driver::state` に統合しました。
`MotorControl` の処理は `DriverCore` に統合しています。
`CanTransport` と `MotorStatePublisher` の公開メソッドはそのまま使えます。

## テスト・設定・ライセンス

模擬 VESC は `test/support/fake_vesc_node.cpp` に置き、`use_mock:=true` と統合テストで使います。
デモ用に `BUILD_TESTING=OFF` でもインストールします。
ドライバのテストはプロトコル、制御、通信、状態変換を検査し、
ros2_control 側のテストは hardware 設定と両入口の統合動作を検査します。

`config`、`launch`、`urdf` はインストール後の起動例に必要です。
独自インターフェースの `vesc_can_msgs` と両ドライバの計 3 パッケージです
F1TENTH の msg と同梱 `vesc_msgs` は廃止しました
ライセンス原本はルートの MIT 1 個で、3 パッケージの share へインストールします
ビルド出力と Python キャッシュは `.gitignore` で除外します。
