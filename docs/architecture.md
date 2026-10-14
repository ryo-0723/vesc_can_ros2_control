# 構成・スレッド・複数台

## パッケージの分担

| パッケージ | 内容 | 他のパッケージとの関係 |
|---|---|---|
| vesc_can_msgs | 指令 msg、時刻付きの配列指令とモータ状態 msg、エラー msg、再起動 srv | ドライバ・ros2_control への依存なし |
| vesc_can_driver | CAN encode/decode、共通状態・単位変換、通信、通常ノード、模擬 VESC、異常通知 | vesc_can_msgs を使用、ros2_control への依存なし |
| vesc_can_ros2_control | SystemInterface、control 用 launch、URDF と controller 設定例 | vesc_can_driver の driver / state を使用 |

`vesc_can_driver::protocol` は encode/decode だけの公開 CMake target です。
`vesc_can_driver::driver` はモータ設定・単位変換・状態更新・5 モードの指令処理と
CAN トピック通信をまとめます。`vesc_can_driver::state` は VescState の変換と配信です。
通常ノードと hardware plugin は同じ driver / state をリンクします。
protocol は rclcpp を使わず、driver / state は ROS 通信に rclcpp を使います。
いずれも ros2_control には依存せず、hardware plugin が ros2_control の入口を担当します。
パッケージを分割しても、ライブラリ間の呼び出しにトピック中継や追加スレッドは
発生しません。実行時の CAN executor と mailbox の構成は以下のとおりです。

```mermaid
flowchart LR
  C[ros2_control controllers] --> H[VescSystem: read / write]
  N[vesc_node: topic / service] --> M[latest command mailbox]
  H --> M
  M --> T[CanTransport: private executor thread]
  T --> D[DriverCore: STATUS / modes / deadlines]
  D --> P[vesc_protocol: encode / decode]
  P --> F[can_msgs/msg/Frame]
  F --> Z[Zenoh pico gateway]
  Z --> B[physical CAN bus]
  B --> V[multiple VESCs]
  V --> B
  B --> Z
  Z --> T
  T --> H
  T --> N
  T --> E["/vesc/event: VescEvent"]
```

`DriverCore::receive()` は制御の無効・異常停止中にも状態を更新します。
`ready` / `activate` はドライバのホスト側の制御状態で、CAN 有効化パケットではありません。
パケットの encode/decode は protocol を直接呼び出せます。

通常ノードとプラグインはどちらかを選びます。同じ ID を両方から制御しません。
`vesc_control.launch.py` は controller_manager に hardware component を読み込ませ、
通常ノード `vesc_node` は起動しません
VescSystem は実行ファイルではなく、controller_manager のプロセスへロードする
共有ライブラリです。System 1 つで複数の VESC を管理し、CAN 送受信ノードと
専用 executor スレッドを各 1 つ持ちます。モータごとの ROS ノードやスレッドは
作りません。controller_manager、コントローラ、ROS ミドルウェアのスレッドは別に存在します。

通常ノードは 1 プロセスで、トピック API のノードと CAN I/O の内部ノードの
2 つを持ちます。メイン executor と CAN executor がそれぞれ動きます。
「ROS グラフ上のノードが 1 つ」という構成ではありません。

上位ノード向けの指令入口は `/vesc/commands` の VescCommandArray 1 つです
各要素に論理 motor_id と float32 の value、配列全体に生成時刻 stamp を入れます
モードは起動時の YAML から決まり、指令や起動後のパラメータ変更で切り替えません
上位ノードは vesc_can_msgs にだけ依存して指令・状態・エラー・再起動の型を使えます
通常ノードの状態は `/vesc/state` の VescStateArray から購読し、motor_id で識別します
hardware の状態は `~/motors/<joint名>/state` の VescState で配信します
モータ名・logical_id・VESC の controller_id の対応は設定に集約し、上位ノードには CAN ID を要求しません
通常ノードは YAML、hardware は URDF の joint に logical_id を指定します
同じモータの logical_id を揃えれば、両入口の状態・エラーを同じ motor_id で識別できます
速度と角度は関節単位の float64、監視値は float32 で配信し、内部の double 計算は維持します
全モードの VescState.position_rad は STATUS5 から換算した積算関節角 [rad] です
車輪など位置制御以外の position state interface も同じ tacho_to_pos() を使います
位置制御の position state interface は STATUS4 の PID 角度を維持します

## 制御周期との境界

hardware read/write は、起動時に確保した配列とインターフェースハンドルを使用します。
read は状態のコピー、write は最新指令を共有領域へ渡す処理で、ROS publish や
ネットワーク待機を行いません。共有領域と ros2_control ハンドルのアクセスは
待機しない try-lock 方式です。競合した周期は以前の状態を使う／書き込みを次周期へ
延期します。鮮度期限の検査は続けるため、無期限に古い指令を使い続けません。

CAN executor が一定周期で最新指令を取り出し、変換・制限・異常検査・Frame 送信を
行います。通常ノードでは VescCommandArray の論理 ID を内部添字に対応付け、指定したモータの
最新指令を配列ごとに受け渡します。hardware では全関節の指令をまとめて受け渡します。

通常ノードは起動時に論理 ID からモータ添字への表と、指令・状態の作業領域を確保します
set_commands() は配列全体を検査し、1 回のロックと共通の受信時刻で指定モータだけを更新します
同じモータの未送信値は上書きし、指令の履歴をキューへ蓄積しません
未指定モータの値と受信時刻は維持し、同じ値の再受信でも指定モータの期限を更新します
CAN の周期処理が最新の送信待ち指令を取り出し、制御用の保持値へ移してフレームを生成します
フレーム生成と CAN トピックへの publish は同じ CAN executor で順に行います
状態配列の publish は通常ノードの executor です
状態は事前確保した領域へ try-lock でコピーし、競合時は前回の状態に鮮度判定を適用します
異常は検出した処理から /vesc/event へ通知し、状態の周期を待ちません
周期の /diagnostics は生成・配信しません
mailbox はモータごとに最新指令だけを保持します。
配列の stamp は送信側の生成時刻で、指令期限は受信側の単調時刻から判定します
受信コールバックは同じ executor で STATUS と故障コードを状態へ反映します
故障問い合わせは fault_poll_period_ms（既定 100 ms）の別タイマーで実行します
要求・応答は short buffer の 1 フレームずつで、動作指令の 100 Hz 周期とは独立しています

異常通知は CanTransport の on_fault を状態ロック外で呼び、共通の MotorStatePublisher が
VescEvent を error_topic（既定 /vesc/event）へ配信します
通常のコールバックからの不正指令・CAN エラーフレーム・本体の故障応答は、検出した処理から通知します
DriverFault に本体の ID と故障番号を持たせ、PC 側の異常と区別します
公開する項目は論理 motor_id とエラー文字列 message のみです
本体故障は該当 ID、PC 側の全台停止は全 ID に 1 件ずつ通知します
本体の故障コードが変化したときは、既に停止中でも通知します
ros2_control の read/write は通知を配信せず、異常停止を atomic に要求します
CAN 周期が要求を処理し、状態や期限から検出した異常も同じ処理の直後に通知します
通常ノードの異常停止は reboot 要求で解除し、同じ異常の毎周期の通知は行いません
通常の動作開始は fresh な指令と必要な STATUS が揃った時に CAN executor が処理します
再起動は論理 ID の指定を内部添字へ変換し、CAN executor で停止フレームの後に 1 回送信します
対象の受信状態はクリアし、500 ms の待機後に届く状態を使用します
登録モータ数分の直近イベントを TransientLocal で保持しますが、イベントは現在の有効状態ではありません
transport の shutdown / join を完了してから通知 publisher を破棄します
CAN 周期を待つ間に hardware が error lifecycle へ移った場合も、終了処理で未処理の異常を通知します

std::thread は join してから状態を破棄します。自分の executor だけを cancel し、
controller_manager の executor を停止しません。C++ の例外は起動・設定・専用
executor の境界で処理し、デストラクタから例外を出しません。

この方式は soft real-time です。mutex の共有、ROS ミドルウェア、OS スケジューラ、
ネットワークの遅延があるため hard real-time は保証しません。
専用スレッドを追加するだけで制御の決定性が保証されるわけではありません。
重い最適化やスレッドの増設は、実機で周期・CPU・CAN 負荷を測定してから判断します。

## 4 輪独立駆動・操舵の例

1 CAN バスなら System を 1 つ作り、駆動 4 関節 + 操舵 4 関節の計 8 関節を登録します。
同じ CAN Tx/Rx にまとめ、各関節へ固有の controller_id を割り当てます。
別バスの場合は System を分け、Tx/Rx のトピックも分けます。必要なフィードバックを
1 台でも失うと、その System 内の全 VESC を停止します。

8 台へ 100 Hz で送信すると指令は 800 frames/s。STATUS1/4/5 を各 50 Hz で
全台から受信するなら 1200 frames/s、合計 2000 frames/s です。
1 フレーム約 160 bit として余裕を見た概算は 320 kbit/s。
1 Mbit/s CAN の約 32%、500 kbit/s の約 64% に相当します。
これは DLC・bit stuffing・他ノード・再送などで変わる概算で、測定値ではありません。
必要な STATUS と周期を選び、バス負荷を測定してください。

検証は模擬 2 台と 5 モード混在の模擬 5 台での機能確認です。8 台以上での負荷試験、Zenoh pico を含む
途絶・遅延試験、実 VESC での角度・電流・停止の確認は今後の実機導入作業です。

## 外部パッケージとの関係

[sbgisen/vesc](https://github.com/sbgisen/vesc/tree/jazzy-devel) には
ros2_control の ActuatorInterface があり、USB/UART 接続では有用な候補です。
確認した Jazzy ブランチはシリアルポートと VESC パケットを使用します。
本パッケージの native CAN Frame + Zenoh pico 経路へそのまま接続できるものではありません。
通信の置き換え、CAN STATUS の状態取得、複数関節の System 化が必要です。
本実装は同プロジェクトのコードをコピーせず、CAN 境界を共有する構成を採用します。

通常ノードと hardware plugin は、共通の `MotorStatePublisher` でモータごとの
`vesc_can_msgs/msg/VescState` を 50 Hz で配信します。1 つの状態タイマーで全台を処理し、
通常ノードではメイン executor、hardware plugin では既存の CAN executor が実行します。
モータごとのノードやスレッドは追加しません。ros2_control の read/write は 100 Hz で
state interface と指令を受け渡し、メッセージの生成・配信は行いません。
関節状態は state interface から取得し、joint_state_broadcaster が `/joint_states` を
50 Hz で配信します
通常ノードの周期出力は VescStateArray、異常時の通知は /vesc/event です
[controller_manager の公式仕様](https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html#parameters)
でも、manager の update_rate は hardware の read / controller の update / hardware の write の周期です。
CAN 送信は両方の入口で各モータ 100 Hz です。例えば 4 台の指令は合計 400 frames/s になります。
STATUS1〜6 を解釈できる実装でも、
全 STATUS を常時送信する必要はありません。制御に必要な種類と診断用の低頻度データを
VESC Tool 側で選び、CAN と Zenoh の負荷を調整してください。
