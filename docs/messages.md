# メッセージ対応と移行

状態は [f1tenth の ROS 2 版 vesc_msgs](https://github.com/f1tenth/vesc/tree/153998df8545fe1781b975df88e411b4e71d4bfe/vesc_msgs)
のパッケージを、このリポジトリの `vesc_msgs/` に同梱します。これはコミュニティの
ROS パッケージで、VESC firmware の CAN パケットそのものとは別の定義です。
上流の 4 種類の msg と元のメタデータを変更せずコピーし、上流の LICENSE を
保存しています。ビルド設定には LICENSE と UPSTREAM.md をインストールする処理を
追加しています。取り込み元と commit は [UPSTREAM.md](../vesc_msgs/UPSTREAM.md) に
記録しています。同名の `vesc_msgs` でも fork や revision により定義が異なるため、
他の source copy は同じワークスペースへ追加しません。

## 使用する型

| 用途 | 型 | 単位・識別 |
|---|---|---|
| CAN 境界 | can_msgs/msg/Frame | 元の CAN ID、フラグ、DLC、ペイロードを保持 |
| VESC 状態 | vesc_msgs/msg/VescStateStamped | モータごとのトピック、state.controller_id で CAN ID を識別 |
| 関節状態・複数関節指令 | sensor_msgs/msg/JointState | 関節名、関節 rad / rad/s |
| 単一関節指令 | std_msgs/msg/Float64 | 選択したモードに応じて関節 rad または rad/s |
| 有効化・無効化 | std_srvs/srv/SetBool | 明示的な有効化と異常解除 |
| 接続・鮮度・ドライバ異常 | diagnostic_msgs/msg/DiagnosticArray | モータごとの hardware_id と key/value |

上流 `vesc_msgs` は状態用のパッケージです。指令・CAN フレーム・有効化サービスは
それぞれの標準型を使います。VescImu/VescImuStamped はネイティブ STATUS に
IMU データがないため、このドライバでは配信しません。

## VescState の対応範囲

CAN レイアウトは [VESC firmware の送信処理](https://github.com/vedderb/bldc/blob/master/comm/comm_can.c)
を基準にします。現在、通常モータ向けの周期状態 STATUS1〜6 をデコード・保存します。
設定・BMS・GNSS・ファームウェア更新・問い合わせ応答の全パケットには対応していません。

| 受信 | VescState のフィールド | 配信する値 |
|---|---|---|
| STATUS1 | speed / current_motor / duty_cycle | 生の ERPM、モータ電流 [A]、duty |
| STATUS2 | charge_drawn / charge_regen | 積算電荷 [Ah] |
| STATUS3 | energy_drawn / energy_regen | 積算電力量 [Wh] |
| STATUS4 | temp_fet / temp_motor / current_input / pid_pos_now | 温度 [℃]、入力電流 [A]、モータ PID 角度 [deg] |
| STATUS5 | displacement / voltage_input | 符号付きタコメータ [count]、入力電圧 [V] |
| STATUS6 | 対応するフィールドなし | ADC1〜3 [V] と PPM 値を /diagnostics に配信 |

`speed` は極対数・減速比・方向を適用する前の ERPM です。`pid_pos_now` も生の
モータ PID 角度です。JointState と ros2_control には変換後の関節単位を渡します。
複数の STATUS をまとめた状態であり、header.stamp は ROS の配信時刻です。
すべてのフィールドが同時に測定されたことを表す時刻ではありません。

浮動小数点のフィールドは、その STATUS を未受信または期限切れなら NaN です。
STATUS2/3/6 を受信していなくても、制御に必要なフィードバックが揃っていれば
有効化できます。対応可能な STATUS をすべて高頻度で送信する必要はありません。

STATUS1〜6 から取得できない avg_id/avg_iq/ntc_temp_mos1〜3/avg_vd/avg_vq は
常に NaN、distance_traveled と fault_code は **-1 (未取得)** です。
fault_code=0 は実機の「故障なし」を意味するため、未取得値には使いません。
ドライバの通信途絶などを VESC の故障番号へ変換せず、/diagnostics に配信します。
全フィールドの実値が必要な場合は、別途 VESC の値取得コマンドと CAN 分割通信の
実装が必要です。上流 msg の流用だけで取得できるデータが増えるわけではありません。

整数型の displacement は NaN を表現できないため、最後に受信した値を保持します。
未受信時は 0 です。利用前に、同じモータの /diagnostics にある **status_5_fresh=true**
を確認してください。他の STATUS も status_1_fresh〜status_6_fresh で判定できます。
unavailable_vesc_fields には常に未取得となる上流フィールドを列挙します。

STATUS5 の公式形式は既知の 6 バイト + 予約 2 バイトの DLC=8 です。
既存の DLC=6 も受け付けます。模擬 VESC と既知値テストは公式の 8 バイト形式を使い、
予約領域が既知フィールドに影響しないことを確認します。

## 旧独自 API からの移行

開発段階の独自 `vesc_can_interfaces` パッケージと SetPosition.srv は削除しました。
旧型の publisher/subscriber は次の型・トピック・単位へ変更してください。

| 旧 API | 移行先 |
|---|---|
| ActuatorTarget / ActuatorTargetArray | ~/motors/<motor_name>/command の Float64、または ~/joint_commands の JointState |
| ActuatorState / ActuatorStateArray | ~/motors/<motor_name>/state の VescStateStamped。関節単位には ~/joint_states を使用 |
| logical_id | motors の関節名で指令先を選ぶ。実機の CAN ID は controller_id に指定 |
| state_array_publish_period_ms | state_publish_period_ms |
| target_topic / target_array_topic / state_topic / state_array_topic | 新トピックへの ROS remapping |

旧駆動指令の機械 RPM を新しい関節 rad/s へ移すときは
`joint_rad_s = motor_rpm × 2π / (60 × gear_ratio)` で変換します。
Float64 指令は上流 UART ノードの commands/motor/speed (ERPM) とは単位が異なります。
CAN の配線、中継器の can_msgs 型、ros2_control の関節インターフェースは同じです。

## Lyrical と apt 公開

確認時の [Lyrical rosdistro 一覧](https://github.com/ros/rosdistro/blob/master/lyrical/distribution.yaml)
には vesc_msgs がありません。開発と CI は同梱した `vesc_msgs` をソースからビルドし、
別のリポジトリを取得せず依存を満たします。以前の dependencies.repos で取得した
`src/vesc_upstream` がある場合は、`src` の外へ移してからビルドしてください。

apt 公開時には対象 distro で `vesc_msgs` をリリース済みのパッケージとして解決する
必要があります。同梱パッケージもリリースするか、上流のリリース済みパッケージへ
依存するかを上流メンテナと調整します。同名のパッケージを別の release repository
から重複登録せず、メッセージの定義も独自に変更しません。
