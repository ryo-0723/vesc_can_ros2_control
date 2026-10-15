# 公開準備の検証記録

各節は検証当時の構成を記録しています
旧 vesc_msgs と上流定義への言及は過去の記録で、現在は独自の vesc_can_msgs に統合しています
最新結果は [5 モードの検証](#5-モードの検証) を参照してください

## 初回の公開準備 — 2026-10-13

既存の CAN ドライバ / ros2_control 分割を保持し、模擬 VESC のソースを
`vesc_can_driver/test/support` へ移動しました。現在の `src` の各ファイルはすべて
CMake のビルド対象です。[ソースの役割](architecture.md) と
[モータの制御モード](motor-modes.md) を追加しました。
ドライバ 2 パッケージの maintainer、repository URL、bugtracker URL も設定しました。

## 環境

- Docker: `osrf/ros:lyrical-desktop`、ROS パッケージの APT 更新後
- ROS 2 Lyrical、ros2_control 6.10.1、GNU C++ 15.2.0
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`ROS_DOMAIN_ID=193`
- ソースは読み取り専用でマウントし、ビルド出力をリポジトリ外へ配置
- 模擬 VESC を使用し、稼働中のロボットとは別の ROS domain で検証

## 結果

| 検証 | 結果 |
|---|---|
| 全 C++ ファイルの clang-format チェック（test/support を含む） | 成功 |
| launch と統合テストの Black チェック（100文字） | 成功 |
| `vesc_msgs` と `vesc_can_driver` のビルド（control パッケージを除外） | 成功 |
| `vesc_can_ros2_control` までのビルド | 成功 |
| protocol / core の単体テスト | 11 ケース成功 |
| transport の単体テスト | 1 ケース成功 |
| state_conversion の単体テスト | 3 ケース成功 |
| hardware 設定の単体テスト | 4 ケース成功 |
| 通常ノード・controller_manager の統合テストと終了コード | 3 ケース成功 |
| インストール済み `vesc_can_driver::protocol` を外部 CMake プロジェクトから利用 | ビルド・実行成功 |
| `BUILD_TESTING=OFF` での CAN ドライバビルド | 成功、模擬ノードの実行ファイルもインストール済み |
| package.xml の XML 構文、追加文書のローカルリンク、git diff --check | 成功 |

**22 テストケース、エラー・失敗・スキップは 0** です。
`colcon test-result` の表示は `27 tests, 0 errors, 0 failures, 0 skipped` です。
この 27 件には CTest の 5 エントリも含まれるため、個々のケース数と区別します。

追加した 7 ケースは、速度ゼロでの電流解放と位置ゼロへの移動の違い、
速度の変化率・減速比・回転方向、実測値による電流始動から速度制御への移行、
位置範囲・必須タコメータ・全台停止、未対応または複数の指令インターフェースの拒否、
必須モータ設定と状態インターフェースの検査、不正な数値設定の拒否を確認します。

外部 CMake プロジェクトは `find_package(vesc_can_driver REQUIRED)` と
`target_link_libraries(... PRIVATE vesc_can_driver::protocol)` を使用しました。
`SET_RPM` のフレーム ID とペイロードを実行時に確認し、`ldd` では
`rclcpp` と `hardware_interface` への動的リンクがないことも確認しました。

## 検証中に対応した環境の問題

既存 Docker イメージに新しい CAN / ros2_control パッケージを追加した直後は、
古いメッセージライブラリとの組み合わせで `has_buffer_fields_*` のシンボル不一致が
発生しました。ROS パッケージの更新と新しい出力ディレクトリでのビルド後に
実行テストが成功しました。CI にも更新可能な既存 ROS パッケージを揃える処理を
追加し、配布対象から外れたパッケージは候補なしとして除外しています。

検証環境では生成ファイルの時刻に数秒のずれがあり、`gmake` の `Clock skew`
警告も発生しました。コンテナ内部の別ディレクトリで再ビルド・全テストを行い、
再び 22 ケースが成功しました。再ビルド時には同梱 `vesc_msgs` のメッセージ生成部分に
時刻警告が残っています。また、上流由来の `cmake_minimum_required(VERSION 3.5)` に
新しい CMake の非推奨警告があります。これらは残存警告として記録します。

## 再実行

ワークスペースのルートから実行します。

```bash
source /opt/ros/lyrical/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=193
colcon build --packages-up-to vesc_can_ros2_control --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select vesc_can_driver vesc_can_ros2_control \
  --executor sequential --return-code-on-test-failure --event-handlers console_direct+
colcon test-result --verbose
```

## 未検証の範囲

今回の検証はローカル Docker 上です。GitHub Actions 上での実行、実 VESC、
物理 CAN バス、中継 MCU / Zenoh pico、実機の位置校正・制動・電流制限、
8 台以上の負荷、プロセス停止時の実機 watchdog は検証していません。
模擬ノードはプロトコル応答の確認用で、実機の力学や PID 動作を保証しません。
ROS apt リリースに必要な実機確認・権利確認・手続きは
[公開準備](releasing.md) を参照してください。

## IMU 除外・ライセンス整理後の再検証

同日、`VescImu.msg` / `VescImuStamped.msg` と `geometry_msgs` 依存を外しました。
上流の CHANGELOG は削除し、状態 msg 2 個を生成するローカルの CMake / package.xml
へ整理しました。ドライバの MIT 原本はルートの 1 個に集約し、F1TENTH の BSD 原本と
出典は `vesc_msgs` に保持しています。

`abu2027-tr:robot` の別コンテナで、Lyrical / ros2_control 6.10.1 /
`rmw_fastrtps_cpp` / ROS domain 193 を使用して新規ビルドを行いました。
全 22 テストケースが再び成功しました。
`colcon test-result` は `27 tests, 0 errors, 0 failures, 0 skipped` です。

インストール済みパッケージから Python の状態 msg 2 個を生成でき、
`ros2 interface package vesc_msgs` が `VescState` と `VescStateStamped` だけを列挙することを
確認しました。生成・インストール対象に IMU はありません。
状態 msg の SHA-256 は整理前と同一です。ドライバ 2 パッケージへインストールした
MIT ライセンスと、メッセージパッケージの BSD ライセンス・出典も原本と一致しました。

`Clock skew` 警告は検証環境のメッセージ生成時に残っています。
ローカル CMake を 3.16 に更新したため、以前の CMake 3.5 に関する非推奨警告は
今回のビルドでは発生していません。

## 4 モード対応後の検証

同日、電流・速度・位置・電流＋速度を明示的な `control_mode` にしました。
電流＋速度は電流始動から速度制御への移行です。通常ノードと hardware plugin の
両方から同じ共通ドライバを利用します。モードと指令インターフェースの整合性、
指令の範囲検査、方向変換、電流送信・全台停止、速度の変化率制限を共通化しました。

Lyrical / ros2_control 6.10.1 / `rmw_fastrtps_cpp` / ROS domain 193 の別 Docker
コンテナでビルド・テストを実行しました。**32 ケースが成功**し、
`colcon test-result` は `37 tests, 0 errors, 0 failures, 0 skipped` です。
個々のケースは protocol/core 18、transport 1、state_conversion 3、hardware 設定 6、
統合テストと終了コード 4 です。clang-format と Black のチェックも成功しました。

新たに確認した動作:

- 電流の正負・方向・A 単位・上限と、NaN/Inf の拒否
- モードと command/state interface の組み合わせ、不完全な始動設定の拒否
- 4 モード混在、指令期限切れ、異常保持と新しい指令による明示的復旧
- 電流始動から速度への移行、逆方向の実測値の扱い、反転とゼロ後の再始動
- 速度低下だけで始動へ戻らないこと、移行時の速度指令上限
- 直接速度制御の反転時も変化率制限を維持すること
- mailbox に入る不正指令を直後の正常指令で上書きして隠せないこと
- controller_manager の 4 モード混在制御、電流・速度・位置の CAN 指令と全台停止
- 通常ノードの 4 モード混在、Float64 / JointState 入力、NaN 電流時の全台停止

4 モードの設定例を `four_modes.yaml`、`vesc_four_modes.urdf`、
`controllers_four_modes.yaml` に追加し、統合テストでも使用しています。
電流の状態は custom `current` state interface [A] です。使用した Lyrical の
[joint_state_broadcaster 6.9.0](https://github.com/ros-controls/ros2_controllers/blob/6.9.0/joint_state_broadcaster/src/joint_state_broadcaster.cpp)
は JointState を配信するため、例の broadcaster には position / velocity を指定し、
電流を effort [Nm] に割り当てていません。

実機と Zenoh pico 経路の検証は引き続き別途必要です。

## VESC firmware 6 の STATUS デコード整理後の検証

同日、`Status1`〜`Status6` の構造体と `decode_status_1`〜`decode_status_6` の
関数定義を番号順に揃えました。公式 firmware 6.06 の commit
`94b305ec124b67b0161f2d3d85c4d5b35e48edc5` と、タグ `6.00` の
STATUS 送信・受信処理を照合しました。項目順・符号・倍率は既存実装と一致します。
全 STATUS のフレーム検査を DLC=8 に統一し、STATUS5 の DLC=6 互換受理を廃止しました。

全 6 種の decoder に対する不正 DLC・ID・フラグと出力保持、符号付き 16 / 32 bit
境界値、温度・電流・PID 角度・積算値・ADC・PPM の倍率を確認する 3 ケースを追加しました。
Lyrical で再ビルドし、4 モードの統合テストも含む **35 ケースが成功**しました。
`colcon test-result` は `40 tests, 0 errors, 0 failures, 0 skipped` です。

## 状態メッセージ変換の名前変更後の検証

`vesc_telemetry.hpp/.cpp` を `vesc_state_conversion.hpp/.cpp` に、公開 CMake target を
`vesc_can_driver::state_conversion` に変更しました。変換テストも `test_state_conversion` に
揃え、変換処理の内容は維持しています。

Lyrical で全 3 パッケージを再ビルドし、既存の **35 ケースが成功**しました。
`colcon test-result` は `40 tests, 0 errors, 0 failures, 0 skipped` です。
変更した C++ ファイルの clang-format 検査と、インストール後のヘッダ・共有ライブラリ・
公開 CMake target の名前も確認しました。

## C++ 変数名の整理後の検証

関節の位置・速度と VESC の生値を変数名で区別し、単位、STATUS の受信済みフラグと
受信時刻、CAN の送受信方向を明記しました。変更一覧と C++ 利用者向けの更新点は
[ソースの構成](architecture.md#処理の流れ) に記載しています。
ROS パラメータ、トピック、interface、メッセージ項目名は維持しています。

変更前のコピーと照合し、C++ 16 ファイルの文字列・数値・演算子が同じであることを
確認しました。Lyrical で全 3 パッケージを再ビルドし、4 モードの単体・統合テストを含む
**35 ケースが成功**しました。`colcon test-result` は
`40 tests, 0 errors, 0 failures, 0 skipped` です。
変更した C++ ファイルの clang-format と `git diff --check` も成功しました。

## CAN ドライバのパッケージ名変更後の検証

CAN パッケージとディレクトリを `vesc_can_driver` に揃えました。
ヘッダの include パス、C++ namespace、公開 CMake target、依存パッケージ、
launch とテスト、CI のパスとコマンドも更新しています。

Lyrical でドライバ単体をビルドした後、ros2_control plugin までビルドしました。
既存の **35 ケースが成功**し、`colcon test-result` は
`40 tests, 0 errors, 0 failures, 0 skipped` でした。
外部 CMake プロジェクトから `find_package(vesc_can_driver REQUIRED)` を使用し、
公開 target の protocol / core / transport / state_conversion をリンクした
C++ コードのビルド・実行も成功しました。

`ros2 pkg prefix vesc_can_driver` と `ros2 pkg executables vesc_can_driver` で
インストールされたパッケージと通常・模擬ノードを確認し、
`ros2 launch vesc_can_driver vesc_standalone.launch.py --show-args` で launch の解決も確認しました。
Python 構文、C++ の clang-format、`git diff --check` は成功しています。

## driver_core の短縮命名・重複整理後の検証

`cfg` / `cmd` / `idx` / `pos` / `vel` を使い、単位を保って長い識別子を短縮しました。
電流の符号変換を `apply_direction()` に統一し、モータ一覧と個別設定の検査、
全台の事前検査とモード別指令生成を分けました。通常ノードの重複した起動時設定検査を
除去し、異常停止時の指令リセットは `disable()` と共用しています。

STATUS と指令の受信フラグ・時刻を `std::optional<TimePoint>` にまとめました。
未受信、時刻ゼロでの受信、未来の受信時刻、期限ちょうど・期限切れを区別する検証と、
後ろのモータの起動失敗で先に生成した指令も全台停止へ置き換える検証の 2 ケースを追加しました。
既存 35 ケースと合わせた **37 ケースが成功**し、`colcon test-result` は
`42 tests, 0 errors, 0 failures, 0 skipped` でした。

ドライバ単体、ros2_control plugin、外部 CMake 利用者のビルド・実行も成功しました。
`driver_core` とその更新に伴うコード・テストの clang-format、`git diff --check` を確認し、
既存の通常ノード・通信の手元の整形を維持しました。
`vesc_protocol.hpp/.cpp` は編集前のコピーとバイト単位で一致しています。
`driver_core.cpp` は 473 行から 420 行になりました。

## 通信・ROS・起動設定の変数名整理後の検証

2026-10-14、通信・通常ノード・状態メッセージ変換・ros2_control・模擬ノード・
起動設定・テストの 15 ファイルで、`cfg` / `opts` / `cmd` / `pub` / `sub` / `params` に
命名を揃えました。通常ノードの期限判定には `steady_now`、メッセージの ROS 時刻には
`stamp` を使っています。公開 C++ メンバの変更点は
[通信・ROS・起動設定の変数名](architecture.md#設定と状態) に記載しています。

C++ の文字列・数値・演算子と、Python の変数名以外の AST が編集前と同じであることを
確認しました。`driver_core.hpp/.cpp` と `vesc_protocol.hpp/.cpp` も編集前とバイト単位で一致します。
Lyrical でドライバ単体と ros2_control plugin を再ビルドし、既存の **37 ケースが成功**しました。
`colcon test-result` は `42 tests, 0 errors, 0 failures, 0 skipped` でした。
外部 CMake 利用者のビルド・実行、起動設定の解決、変更箇所の clang-format、
Python 3 ファイルの Black と `git diff --check` も成功しています。

## 周期送信・共通状態配信・終了処理の整理後の検証

2026-10-14、CAN 指令を各モータ 100 Hz、通常ノードと ros2_control の
`JointState` / `VescStateStamped` を 50 Hz に揃えました。
controller_manager の `update_rate` は 100 のままで、state interface の取得・
オドメトリ controller は ROS 状態トピックの周期と独立しています。
状態配信は共通の `MotorStatePublisher` で行い、hardware read/write では配信しません。

CAN の時刻付与・停止要求の反映・配信を `publish_frames()` にまとめ、終了時にも共用しました。
停止中に毎周期繰り返していた初期化、通常ノードの終了用ラッパー、模擬ノードの固定 DLC 引数を
省略しました。参照調査で少数だった関数は ROS が呼ぶ lifecycle callback で、維持しています。

1 回の指令入力から 100 Hz の送信が続く単体テストと、無効な周期設定を拒否する
hardware テストを追加しました。統合テストでは指令入力を 25 Hz にし、
CAN 指令・モータ状態・関節状態の周期を測定しています。受信キューの滞留と区別するため、
周期の計算は各メッセージの送信時刻を使用します。

| 模擬構成 | CAN 指令（各モータ） | VescStateStamped（各モータ） | JointState |
|---|---|---|---|
| ros2_control、4 モード混在 | 100.0 Hz（4 台とも） | 50.0 Hz（4 台とも） | 50.0 Hz |
| 通常ノード、駆動・操舵 | 100.0 Hz（2 台とも） | 50.0 Hz（2 台とも） | 50.0 Hz |

controller_manager の実パラメータが `update_rate=100` であることも確認しました。
元の 37 ケースと追加 2 ケースを合わせた **39 ケースが成功**しました。
最終の `colcon test-result` は `44 tests, 0 errors, 0 failures, 0 skipped` です。
ドライバ単体と hardware plugin のビルド、新しい state_publisher target の外部 CMake 利用、
clang-format、Black、`git diff --check` も成功しています。
上表は模擬環境での ROS メッセージの実測で、実 CAN バスや Zenoh pico 経路の測定ではありません。

## STATUS 受信と制御管理の分離後の検証

2026-10-14、`driver_core.cpp` を STATUS1〜6 の受信と状態更新に絞り、
設定・状態の型と共通変換・検査を `motor_types`、有効化・期限検査・停止と
4 モードの指令生成を `MotorControl` に分けました。
電流・速度・位置・電流＋速度の処理は `make_frame()` の一つの分岐で扱い、
電流起動後の速度ランプを速度モードと共用します。
公開 CMake target は引き続き `vesc_can_driver::core` です。

`vesc_protocol.hpp/.cpp` は編集前とバイト単位で一致しています。
STATUS の受信関数と、共通変換・設定検査の関数群も移動前と一致することを確認しました。
DriverCore は制御の無効・異常停止中でも状態を受信し、フィードバックの復旧だけでは
動作を再開しないことを確認するテストを追加しました。

ドライバ単体と ros2_control plugin を Lyrical でビルドし、元の 39 ケースと追加 1 ケースを
合わせた **40 ケースが成功**しました。`colcon test-result` は
`45 tests, 0 errors, 0 failures, 0 skipped` です。
4 モードの指令、速度ランプ、電流起動・反転、指令・フィードバック・起動のタイムアウト、
全台停止と明示的な復帰を確認しています。
統合テストの実測は ros2_control の 4 台と通常ノードの 2 台とも
CAN 指令 100.0 Hz、VescStateStamped 50.0 Hz、JointState 50.0 Hz でした。

インストール済みパッケージを外部 CMake から利用するビルド・実行も成功しました。
全公開 target を利用する例に加え、core target だけにリンクして ROS ノードを起動せずに
`DriverCore` と `MotorControl` を利用する例を確認しています。
パッケージ・実行ファイル・launch の解決、全 C++ の clang-format と
`git diff --check` も成功しています。実機の CAN バス・モータ・Zenoh pico は未検証です。

## ドライバ本体を 4 単位に統合した後の検証

2026-10-14、ドライバ本体の実装を `vesc_protocol.cpp`、`vesc_driver.cpp`、
`vesc_node.cpp`、`vesc_state.cpp` の 4 ファイルへ統合しました。
公開ヘッダは protocol / driver / state の 3 個、公開ライブラリも
`vesc_can_driver::protocol` / `::driver` / `::state` の 3 個です。
ros2_control の `vesc_system` は別パッケージに残し、通常ノードと共通の driver / state を使います。

設定・状態の型と共通関数、STATUS 受信、4 モードの制御、ROS の CAN 送受信は
`vesc_driver` にまとめました。`MotorControl` は `DriverCore` に統合し、
状態を別オブジェクトから参照する構成を解消しています。
VescState の変換と配信は `vesc_state` にまとめました。
旧ヘッダと旧ライブラリ target は新しい構成へ置き換えています。

protocol の 2 ファイルは統合前とバイト単位で一致しています。
移動した変換・検査・STATUS 受信・指令生成・通信・状態配信の関数も、
統合に伴う参照名・クラス名と整形以外の処理が一致することを確認しました。
Lyrical でドライバ単体と hardware plugin をビルドし、既存の **40 ケースが成功**しました。
最終の `colcon test-result` は `45 tests, 0 errors, 0 failures, 0 skipped` です。

周期テストでは、ROS の送信時刻が約 9.5 秒逆行するケースを観測しました。
publisher の重複を除外しても発生したため、周期の測定は単調な受信時刻へ変更しました。
開始前に 0.25 秒間キューを処理し、その後 1 秒間を測定します。
指令入力の 25 Hz、必要サンプル数、期待周期の ±25% の判定は維持しています。
ドライバが付ける ROS header stamp と、steady clock による制御期限の処理は変更していません。

| 模擬構成 | CAN 受信（各モータ） | VescStateStamped 受信（各モータ） | JointState 受信 |
|---|---|---|---|
| ros2_control、4 モード混在 | 100.1 Hz（4 台とも） | 50.0 Hz（4 台とも） | 50.0 Hz |
| 通常ノード、駆動・操舵 | 100.1 / 100.4 Hz | 50.0 Hz（2 台とも） | 50.0 Hz |

外部 CMake から新しい 3 target を利用するビルド・実行と、
driver target だけをリンクして DriverCore を利用する例も成功しました。
パッケージ・実行ファイル・launch の解決、全 C++ の clang-format、
Python の Black、`git diff --check` も成功しています。
上表は ROS メッセージの受信周期で、実機 CAN バスや Zenoh pico の測定ではありません。

## 通常ノードの状態出力を整理した後の検証

2026-10-14、通常ノードの `~/joint_states` publisher とメッセージ生成を削除しました
通常ノードは VescStateStamped と診断を配信し、JointState の指令入力は維持しています
hardware component 使用時は controller_manager が VescSystem を読み込み、
通常ノード vesc_node は起動しません
関節状態は state interface から取得し、joint_state_broadcaster が `/joint_states` を配信します

統合テストの通常ノードの判定を VescState と CAN 指令に変更し、
通常ノードの joint_states publisher が存在しないことも確認しています
ドライバ単体と hardware plugin のビルド、既存の **40 ケースが成功**しました
`colcon test-result` は `45 tests, 0 errors, 0 failures, 0 skipped` です

| 模擬構成 | CAN 指令（各モータ） | VescStateStamped（各モータ） | JointState |
|---|---|---|---|
| ros2_control、4 モード混在 | 100.0 Hz | 50.0 Hz | 50.0 Hz、joint_state_broadcaster |
| 通常ノード、駆動・操舵 | 100.0 Hz | 50.0 Hz | 配信なし |

関数コメントの句点を整理し、通常ノード以外の 9 C++ ファイルは
コメント・整形を除いたコードが編集前と一致することも確認しました
全 C++ の clang-format、Python の Black、外部 CMake 利用、`git diff --check` は成功しています
上表は模擬環境での ROS メッセージ受信周期で、実 CAN バス・モータ・Zenoh pico は未検証です

## 通常ノードのモータ名による指令へ統一した後の検証

2026-10-14、通常ノードの指令を `~/commands` の MotorCommands に統一しました
names にモータ名、values に設定したモードの指令値を入れ、CAN ID との対応は設定に集約します
上流 VescState / VescStateStamped の内容と SHA-256 は変更していません
状態はモータ名ごとのトピックで配信し、上位ノードに CAN ID の知識を要求しません

ドライバ単体と hardware plugin を Lyrical でビルドし、**42 ケースが成功**しました
`colcon test-result` は `47 tests, 0 errors, 0 failures, 0 skipped` です
既存の統合テストを MotorCommands 入力へ変更し、次を確認しています

- 設定順と異なる名前の配列でも正しい VESC へ送ること
- 電流・速度・位置・電流＋速度を 1 つの指令メッセージで扱えること
- 配列長の不一致、未登録名、重複名、NaN 電流で全台停止し、異常を保持すること
- 一部のモータの指令更新が、未指定モータの指令期限を延長しないこと
- CAN 指令は各モータ 100 Hz、VescState は両入口で 50 Hz、JointState は ros2_control のみ 50 Hz

外部 CMake 利用者からのライブラリのビルド・実行と、MotorCommands の C++ / Python
シリアライズ・デシリアライズも成功しました
全 C++ の clang-format、Python 3 ファイルの Black、`git diff --check` は成功しています
検証は模擬環境で、実 CAN バス・モータ・Zenoh pico は未検証です

## 論理 ID 指令と即時の異常通知へ変更した後の検証

2026-10-14、共有チャットの軽量指令に合わせ、VescCommand と VescCommandArray を追加しました
各要素は motor_id / mode / float32 value で、motor_id は通常ノードの YAML の logical_id に対応します
YAML でモードを固定し、指令の mode が一致しなければ全台停止します
command_topic / error_topic を設定でき、rox2026 と同じ3モータの割り当てを持つ設定例を追加しました

Lyrical でドライバ単体と hardware plugin をビルドし、**45 ケースが成功**しました
最終の `colcon test-result` は `50 tests, 0 errors, 0 failures, 0 skipped` です
protocol/core 24、transport 3、state_conversion 3、hardware 設定 7、統合 7、終了コード 1 の内訳です

- 論理 ID と CAN ID の分離、配列順によらない振り分け、4 モードの混在
- 未登録 ID、重複 ID、モード不一致、不正な指令値での全台停止と異常保持
- rox2026 の論理 ID 10 / 11 / 12 から CAN ID 51 / 52 / 50 への指令と状態配信
- CAN エラーフレームと通常ノードの不正指令が、状態周期を待たずに通知されること
- 異常通知が状態ロック外で実行され、shutdown 時の未処理の異常も通知されること
- ros2_control のフィードバック途絶時にも、publisher の破棄前に異常通知が届くこと
- 通常ノードの TransientLocal 購読者が、最後の異常イベントを取得できること

状態周期を 2000 ms にした模擬構成で、不正指令の通知を 1.5 ms、CAN エラーの通知を 1.6 ms で受信しました
これはこの検証での受信遅延で、実機・DDS・OS を含む遅延保証ではありません
通常の状態・指令は従来どおり VescState 50 Hz、各モータの CAN 指令 100 Hz です
ros2_control の state interface は 100 Hz、JointState は 50 Hz です

上流 VescState / VescStateStamped の SHA-256 と、vesc_protocol.hpp/.cpp は編集前と一致しています
インストール後の外部 CMake 利用、指令メッセージの C++ / Python シリアライズ、
全 C++ の clang-format、Python 3 ファイルの Black、`git diff --check` も成功しました
実 CAN バス・モータ・Zenoh pico は未検証で、STATUS1〜6 から VESC 本体の故障コードは取得できません


## 指令型の独立と CAN 故障コード取得後の検証

2026-10-14、指令型を独立した vesc_can_msgs パッケージに移動しました
VescCommand は motor_id / float32 value のみで、モードは起動時の設定で固定します
VescCommandArray に builtin_interfaces/Time stamp を追加しました

VESC firmware 6.06 の commands.c / comm_can.c / datatypes.h を確認し、
COMM_GET_VALUES_SELECTIVE の bit 15 による故障コード問い合わせを追加しました
要求は CAN_PACKET_PROCESS_SHORT_BUFFER の DLC=7、応答は DLC=8 です
既定 100 ms ごとに問い合わせ、非ゼロの応答を受信した処理から /vesc/event へ配信します
DiagnosticArray の values に source=vesc / controller_id / fault_code を持たせ、
PC 側の異常は source=driver として区別します

Lyrical / rmw_fastrtps_cpp で **53 ケースが成功** しました
colcon test-result の集計は、テスト実行単位 5 件を含む **58 tests, 0 errors, 0 failures** です

- vesc_can_msgs 単体を先にビルドし、ドライバをビルドする前に外部 C++ / Python から指令型のシリアライズを確認
- 通常ノードの mode 変更拒否と、古い・未来の stamp が受信側の指令期限を変えないことを確認
- 公式の既知バイト列で故障の要求・応答を確認し、異なる宛先・mask・コマンド・DLC・フラグを拒否
- 本体の故障で全台停止し、正常応答が来るまで再有効化できず、故障情報の期限切れでも停止が解除されないことを確認
- 同一の故障コードの重複抑制、停止中のコード変更通知、取得値の鮮度、未取得値 -1 を確認
- 問い合わせ無効時に要求していない故障応答を受理しないことを確認
- 状態周期 2000 ms の通常ノードで、周期配信を待たず故障イベントを受信し、周期の診断にも本体の故障理由が出ることを確認

CAN 動作指令は各モータ 100 Hz、VescState / JointState は 50 Hz を維持しています
故障問い合わせの 10 Hz は動作指令とは別の周期です
追加の ros2_control 模擬確認でも、CAN ID 12 の故障コード 4 が publisher の終了前に届き、
4 モータが停止して全コントローラが無効化されました

外部 CMake 利用、ros2 topic pub の stamp: now、全 C++ の clang-format、
Python の Black、git diff --check も成功しました
上流 VescState / VescStateStamped の内容は変更していません
検証対象は独立した VESC ID を持つ模擬構成で、dual-motor 基板の第2モータの応答は対象外です
実 CAN バス・モータ・Zenoh pico での動作は未検証です


## rox2026 の故障応答フィルタを追加した検証

2026-10-15、rox2026 の robot_bringup/launch/hardware/ros2_socketcan.launch.py に、
PC の CAN ID 254 宛ての故障応答（拡張 ID 0x8FE）を通すフィルタを追加しました
STATUS1 の既存フィルタは保持し、設定値は 00000900:9FFFFF00,000008FE:9FFFFFFF です

rox2026 の ros2_socketcan の実際の C++ フィルタ解析を使い、524 条件を検査しました
すべての STATUS1 ID が引き続き通り、short buffer 応答は宛先 254 のみを通すこと、
標準 ID・STM32・EduLite のフレームがこの VESC フィルタを通らないことを確認しています
launch の Python 構文と、フィルタ以外のノード設定が変わっていないことも確認しました
両リポジトリの git diff --check は成功しました
実 CAN バスでの受信とブリッジの再起動は実施していません

再起動については公式 6.06 の COMM_REBOOT=29 と NVIC_SystemReset() を確認し、
short buffer に包む形式と、PC 側の enable による停止保持解除を README へ記録しました


## 有効化サービスを再起動サービスへ置き換えた検証

2026-10-15、通常ノードの ~/enable と auto_enable_once を廃止し、
新鮮な必要 STATUS と新しい指令が揃うと動作を開始する構成に変更しました
~/reboot は独立した vesc_can_msgs/srv/Reboot を使い、uint8 motor_id に論理 ID を指定します
対応する 1 台へ公式の COMM_REBOOT=29 を short buffer / DLC=3 / send=2 で 1 回送ります
要求受理は success=true とし、再起動の完了を保証する応答ではありません

再起動処理は CAN executor で停止フレームを送った後に実行します
保存した指令は全台分を破棄し、対象の受信状態をクリアします
500 ms は対象の指令と受信状態を受理せず、その後の新鮮な状態と新しい指令を使います
PC 内部の開始・停止 API は activate / deactivate / active に整理し、
ros2_control の lifecycle ではこの API を使用します

Lyrical / rmw_fastrtps_cpp で **57 ケースが成功** しました
colcon test-result は実行単位 5 件を含む **62 tests, 0 errors, 0 failures** です

- 独立した指令・再起動インターフェースのビルドと、C++ / Python でのシリアライズを確認
- enable サービスなしで通常ノードの指令から 4 モードの動作を開始することを確認
- フィードバックより先に届いた新しい指令を保持し、状態が揃うまで動作指令を出さないことを確認
- COMM_REBOOT の既知バイト列、ID 範囲、broadcast 拒否を確認
- reboot の論理 ID 11 が CAN ID 52 に変換され、対象の 1 台に 1 回だけ送信されることを確認
- CAN ID を論理 ID として渡した不正要求、再起動待機中の重複要求を拒否することを確認
- 対象の旧 STATUS を使わず、保存済み指令を再送せず、新しい指令で復旧することを確認
- 不正指令・通信途絶・VESC 本体故障の即時通知と停止を維持することを確認

実測の CAN 動作指令は各モータ約 100 Hz、VescState / JointState は約 50 Hz でした
インストール後の外部 CMake 利用は新しい activate API で成功しています
全 C++ の clang-format、Python の Black、git diff --check も成功しました
上流 VescState / VescStateStamped の定義は変更していません
再起動の検証は模擬 CAN で、実機への再起動命令は送信していません


## 独自状態 msg と統合後の検証

2026-10-15、状態を CAN で取得する項目から独自定義にし、msg / srv を vesc_can_msgs に統合しました
F1TENTH からコピーした vesc_msgs と VescStateStamped は廃止しました
VescState は stamp を直接持ち、値の名前に単位を付けています
常に未取得だった avg_id / avg_iq / ntc_temp_mos1〜3 / avg_vd / avg_vq / distance_traveled を削除しました
整数のタコメータには tachometer_fresh を追加し、故障コードは応答値 0〜255 と未取得値 -1 を保持する int16 です

ROS 2 Lyrical / rmw_fastrtps_cpp、隔離した ROS_DOMAIN_ID=179 で **58 ケースが成功** しました
colcon test-result は実行単位 5 件を含む **63 tests, 0 errors, 0 failures, 0 skipped** です

| 検証 | 結果 |
|---|---|
| vesc_can_msgs だけを先にビルド | 成功、ドライバへの依存なし |
| 外部 C++ / Python から指令・状態・再起動型をシリアライズ | 成功 |
| 生成インターフェースの列挙 | VescCommand / VescCommandArray / VescState / Reboot の 4 型 |
| ドライバ単体の依存ビルド | 成功、vesc_can_msgs と vesc_can_driver の 2 パッケージ |
| ros2_control までのビルド | 成功、計 3 パッケージ |
| STATUS1〜5 の状態変換・欠測値・各 STATUS の期限切れ | 成功 |
| int32 最大 ERPM と最小タコメータ値、故障コード 255 の保持 | 成功 |
| 通常ノードと hardware plugin の新しい VescState 受信 | 成功、配信時刻も確認 |
| 即時の異常通知、故障コードの反映、再起動と旧指令の破棄 | 成功 |
| インストール済み公開 CMake target の外部利用 | ビルド・実行成功 |

模擬構成で、CAN 動作指令は各モータ約 100 Hz、VescState と JointState は約 50 Hz でした
ros2_control の state interface は controller_manager の 100 Hz 設定を維持しています
故障イベントのテストは、サービス応答待ち中も指令を更新し、指令期限切れが先に起きない形にしました
状態配信を 2000 ms にしたテストでも、故障イベントは周期配信を待たずに届きます

全 C++ の clang-format、Python の Black、package.xml と文書のローカルリンク、git diff --check も確認しました
ソースは読み取り専用でマウントし、ビルド・検証用の出力はリポジトリ外に置きました
今回の動作検証は模擬 CAN で、実 CAN バス・モータ・Zenoh pico での確認は実施していません


## 5 モードの検証

2026-10-15、velocity / current / position / duty_cycle / current_velocity の 5 モードに対応しました
duty 比は direction で符号を合わせ、公式 VESC 6.06 の SET_DUTY=0 / 比率×100000 / DLC=4 で送信します
max_duty は 0 < max_duty ≤ 1 の指令上限で、通常ノードの YAML と hardware joint param から読み込みます
control_mode と command_interface は起動時に決まり、通常ノードの変更要求は拒否します

設定例を five_modes.yaml、controllers_five_modes.yaml、vesc_five_modes.urdf に更新しました
旧 four_modes の設定例はこの 5 モード版へ置き換えています
通常ノードと hardware plugin は同じ DriverCore の変換・範囲検査・フレーム生成を使用します

ROS 2 Lyrical / rmw_fastrtps_cpp、ROS_DOMAIN_ID=181 で **63 ケースが成功** しました
colcon test-result は実行単位 5 件を含む **68 tests, 0 errors, 0 failures, 0 skipped** です

- 公式の既知バイト列で duty の正負・倍率・拡張 ID・DLC を確認
- duty ±1 の境界、NaN/Inf、比率の範囲外、broadcast 宛先を検査
- duty の方向変換と、減速比・極対数を掛けないことを確認
- duty の上限とゼロ指令、5 モード混在時の指令期限・停止・復旧を確認
- hardware の duty_cycle command / state interface と設定上限を検査
- 通常ノードと controller_manager の 5 台構成で、duty の送信値と VescState の生値を確認
- 実行中の duty モードと command interface の変更拒否を確認
- 故障通知、論理 ID を指定した再起動、旧指令を再利用しない復旧を確認

5 台の hardware 模擬構成で、各モータの CAN 動作指令は約 100 Hz、VescState は約 50 Hz でした
JointState は約 50 Hz、controller_manager は 100 Hz の設定を維持しています
ドライバ単体と hardware plugin のビルド、公開 CMake target の外部利用も成功しました
Python の書式を Black で統一し、C++ / Python の書式と文書の参照を検査しました
検証は模擬 CAN で、実機へ動作指令は送信していません

## STATUS5 由来の積算角の検証

2026-10-15、VescState に position_rad [rad] を追加しました
全モードで STATUS5 の符号付きタコメータから極対数・減速比・direction を適用して算出します
2π では折り返さず、PID 用の zero_offset_rad は適用しません
未受信・期限切れでは NaN を配信します
位置制御以外の ros2_control position state interface も同じ tacho_to_pos() を使います
位置制御の position state interface は STATUS4 の PID 角度を維持します

ROS 2 Lyrical / rmw_fastrtps_cpp、ROS_DOMAIN_ID=182 で **66 ケースが成功** しました
colcon test-result は実行単位 5 件を含む **71 tests, 0 errors, 0 failures, 0 skipped** です
内訳は protocol/core 34、transport 6、state_conversion 6、hardware 設定 8、統合 11、終了コード 1 です

- 14 極のモータで 42 count = 2π rad、正負・複数回転・減速比・回転方向を確認
- activate / deactivate と STATUS4 の受信で車輪の積算角がリセットされないことを確認
- 全 5 モードで STATUS5 の積算角を配信し、PID の原点と角度から独立していることを確認
- STATUS5 の欠測・期限境界・期限切れ・未来の受信時刻で NaN と鮮度を確認
- 通常ノードの状態トピックでカウントと position_rad の対応を確認
- 実際の controller_manager と joint_state_broadcaster で、車輪の position state が 2π を超えて保持されることを確認
- 逆回転で積算角が減少し、停止後は hardware の position state と VescState.position_rad が一致することを確認
- インストール済み CMake target の外部利用と、新しい状態 msg の C++ / Python シリアライズを確認

5 台の hardware 模擬構成で CAN 指令は 100.1〜100.2 Hz、VescState は 49.9〜50.0 Hz、JointState は 50.0 Hz でした
通常ノードの 2 台構成でも CAN 指令は約 100 Hz、VescState は約 50 Hz です
ドライバ単体と hardware plugin をクリーンビルドし、C++ / Python の書式検査も通過しました
ソースは読み取り専用でマウントし、ビルド・検証出力はリポジトリ外に置きました
VESC の再起動や int32 カウンタの折り返しを連続化する処理は今回も追加していません
検証は模擬 CAN で、実機の測定精度・実 CAN バス・Zenoh pico は未検証です

## 通常ノードの最新指令と状態配列の検証

2026-10-15、通常ノードの指令を配列単位で一括保存する set_commands() に変更しました
論理 ID は起動時に作る表でモータ添字へ変換し、指令・状態の vector は再利用します
1 配列の全要素を同じ受信時刻と 1 回のロックで保存し、同じモータの未送信値を上書きします
値が同じでも受信時刻を更新し、未指定モータの期限を更新しません
指令値の検査は transport に集約し、通常ノードの同じ範囲検査を削除しました

通常ノードは /vesc/state に VescStateArray を周期配信します
各要素は motor_name を持つ VescState で、配列と各要素に同じ配信時刻を付けます
作業用の状態と出力配列は事前確保し、try-lock で前回の snapshot と鮮度判定を使えます
周期の /diagnostics は削除し、異常の検出時だけ /vesc/event に通知します
hardware のモータ別 VescState 配信と state interface は同じ方式を維持しています

ROS 2 Lyrical / rmw_fastrtps_cpp、ROS_DOMAIN_ID=183 で **68 ケースが成功** しました
colcon test-result は実行単位 5 件を含む **73 tests, 0 errors, 0 failures, 0 skipped** です
内訳は protocol/core 34、transport 8、state_conversion 6、hardware 設定 8、統合 11、終了コード 1 です

- CAN executor 起動前の複数の更新が、初回送信時には最新値だけになることを確認
- 一部のモータの更新が、他のモータの保持値を維持することを確認
- 別スレッドが指令配列を繰り返し更新しても、送信する 2 台の値が別の配列から混ざらないことを確認
- 配列後半の NaN・範囲外・不正添字・重複添字を検査し、全台停止を確認
- 通常ノードの状態配列でモータ名・順序・時刻・積算角・故障コードを確認
- 通常ノードから周期の /diagnostics とモータ別状態を同時配信しないことを確認
- 状態周期を 2000 ms にした構成でも、指令エラーと CAN エラーを 1.5 ms / 1.1 ms で通知
- 本体の故障コードの即時通知・同じ故障の重複抑制・再起動・停止保持を確認
- 外部 CMake target と新しい状態配列の Python シリアライズを確認

通常ノードの状態配列は 50.0 Hz、各モータの CAN 指令は 100.0 Hz でした
追加で state_publish_period_ms=2 を設定し、2 台の状態配列を 500.3 Hz で受信しました
このときも 2 台それぞれの CAN 指令は 100.0 Hz でした
これは模擬 CAN の動作確認で、従来の個別配信に対する CPU・帯域の比較測定ではありません
ビルド中に build.make の時刻が 1.7 秒先を示す Clock skew 警告がありましたが、全ターゲットのコンパイル・リンクと検証は完了しました
実 CAN バス・実機・Zenoh pico の性能は未検証です

## 状態・エラーの ID 統一とメッセージ整理の検証

2026-10-15、指令・状態・エラーの識別を論理 motor_id に統一しました
通常ノードは既存 YAML の logical_id、hardware は URDF の各 joint の必須 logical_id を使います
論理 ID は 0〜255 で一意にし、CAN の controller_id は別に管理します
/vesc/event は VescEvent（uint8 motor_id / string message）の 2 項目へ変更しました
本体故障は名称と番号を持つ文字列、PC 側の停止は影響する全 ID に停止理由を通知します
正常コード 0 はイベントを配信しません

VescState の速度は velocity_rad_s、PID 角度は pid_pos_rad、積算角は position_rad にまとめました
速度・角度は関節単位の float64、電流・duty・電荷・電力量・温度・電圧は float32 です
公開する生の ERPM・タコメータ・鮮度 bool・モータ名・CAN ID は削除しました
未受信・期限切れの浮動値は NaN を使い、内部計算と hardware state interface の double は維持します

ROS 2 Lyrical / rmw_fastrtps_cpp、ROS_DOMAIN_ID=184 で **70 ケースが成功** しました
colcon test-result は実行単位 5 件を含む **75 tests, 0 errors, 0 failures, 0 skipped** です
内訳は protocol/core 34、transport 8、state_conversion/publisher 7、hardware 設定 9、統合 11、終了コード 1 です

- 通常ノードと hardware で CAN ID と異なる論理 ID を配信し、指令と状態・エラーが同じ ID になることを確認
- URDF の logical_id の欠落・重複・範囲外・負数・小数を拒否し、255 は論理 ID として使用できることを確認
- 速度・PID 角度・積算角の方向と減速比、欠測・STATUS ごとの期限切れ、int32 境界の変換精度を確認
- 監視値の float32 化と、内部・hardware state の double の保持を確認
- 故障番号 4 / 5 / 6 / 29 の名称、未知の 255 の UNKNOWN 表示、正常応答の通知抑制を確認
- 本体故障は該当 ID、PC 側の全台停止は全 ID に通知し、遅れて購読しても全台停止の ID を取得できることを確認
- 状態周期が 2000 ms の構成で、指令エラー 1.8 ms、CAN エラー 1.4 ms、本体故障 81.1 ms の通知を確認
- インストール済み CMake target の外部利用、状態・配列・エラー msg の C++ / Python シリアライズを確認

本体の故障は 100 ms 周期の問い合わせで取得するため、受信後の通知時間に加えて問い合わせ待ちが生じます
周期の状態と JointState は独立したタイマーなので、統合テストでは両方の更新後の値を待って比較します
2 台の VescStateArray の CDR シリアライズは 186 bytes でした（DDS 等の通信ヘッダは含みません）
通常ノードは状態配列 50.0 Hz、CAN 指令 100.0〜100.1 Hz、5 台の hardware も状態約 50 Hz・CAN 約 100 Hz でした
追加の state_publish_period_ms=2 の検証では、2 台の状態配列 499.9 Hz、各モータの CAN 指令 100.0 Hz を確認しました
共有ホストで controller_manager に約 29.5 ms の overrun が 1 回あり、周期の保証や実機性能の測定ではありません
ドライバ単体と hardware plugin のビルド、C++ / Python の書式検査も通過しました
実 CAN バス・実機・Zenoh pico の性能は未検証です

## 設定例とREADMEの整理

2026-10-15、YAMLとURDFを各パッケージの examples/ にまとめました
通常ノードは駆動・操舵の基本例と5モードの例、hardware側は対応するURDFとコントローラ設定を残しています
rox2026固有のYAMLは削除し、論理IDとCAN IDの対応はテスト内の汎用設定で確認します
ソースの説明は architecture.md に統合し、4つのREADMEを日本語で書き直しました

ROS 2 Lyrical / rmw_fastrtps_cpp、ROS_DOMAIN_ID=185で70ケースの成功を確認しました
最終集計は74 tests、0 errors、0 failures、0 skippedです（ケース70件と実行単位4件）
始動フレームの検査は、CAN受信を確認してから指令を送るようにしました

- ドライバ単体とhardware pluginを空のビルド先からビルド
- 移動後の設定例を使って、通常ノード・5モード・ros2_control・再起動の統合テストを実行
- インストール後の標準launchで、基本例の2台へ指令を送り、通常ノードとros2_controlの両方でフィードバックを確認
- launchの既定パスがexamples/を指し、旧config/・urdf/がインストールされないことを確認
- YAML・URDF、文書のローカルリンク・見出し、C++・Pythonの書式を確認

検証は模擬CANで、実機への動作指令は送信していません
