# 公開準備の検証記録 — 2026-10-13

既存の CAN ドライバ / ros2_control 分割を保持し、模擬 VESC のソースを
`vesc_can_driver/test/support` へ移動しました。現在の `src` の各ファイルはすべて
CMake のビルド対象です。[ソースの役割](source-layout.md) と
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
[ソースの構成](source-layout.md#c-変数の命名) に記載しています。
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
