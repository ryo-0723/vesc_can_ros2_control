# 公開と ROS apt リリースまで

現在は ROS 2 Lyrical を対象にした開発用の実装と自動テストです
GitHub へのソース公開と、ROS apt で配布するためのリリース手続きは別です

## パッケージと依存

| パッケージ | 内容 | このリポジトリ内の依存 |
|---|---|---|
| vesc_can_msgs | 独自の指令・状態 msg と再起動 srv | なし |
| vesc_can_driver | CAN encode/decode、通信管理、通常ノード、状態出力 | vesc_can_msgs |
| vesc_can_ros2_control | hardware plugin、control 用の起動・設定例 | vesc_can_driver |

メッセージパッケージを先に、ドライバ、hardware plugin の順でビルドします
同梱した F1TENTH の vesc_msgs は廃止し、インターフェースはこのリポジトリで独立して管理します
上位ノードも vesc_can_msgs に依存すれば指令・状態・再起動の型を使用できます

## 公開準備

1. 実 VESC と CAN 中継器を使い、単位・位置範囲・指令途絶・通信途絶・再起動・再接続を検証する
   対象 firmware と、ROS バージョン、ゲートウェイ条件を記録する
2. 3 パッケージの maintainer と repository / bugtracker URL を確認する
   maintainer は `ryo-0723 <tengbenliang742@gmail.com>` に設定済み
   初期コードの出典はルート README に記録している
3. README、設定例、CHANGELOG、バージョン、依存関係を整え、GitHub 上で CI を通す
   独自 VescState は旧 VescStateStamped と型・フィールドが異なるため、購読側の更新を案内する
4. source release/tag と release repository を用意し、対象 distro の bloom / rosdistro 手続きを行う
   依存の解決と ROS build farm のビルドを確認する
   具体的な手続きはリリース時の公式文書に従う

対応を確認済みの ROS 環境は Lyrical です
別の distro を追加する場合は hardware API の互換性とビルド・テストを確認します
検証済みの内容は [検証記録](validation.md)、型・単位・欠測値は [メッセージ対応表](messages.md) に記載しています

## ライセンスの配置

3 パッケージはルートの [LICENSE](../LICENSE) を使用します
ソース中の原本はこの MIT ライセンス 1 個に集約し、各パッケージの share へインストールします
CMake は `../LICENSE` を参照するため、このリポジトリの配置でビルドします
個別の release archive へ切り出す場合は原本を含め、install パスを合わせます

参考: [rosdistro の公開手順](https://github.com/ros/rosdistro/blob/master/CONTRIBUTING.md)、[bloom](https://github.com/ros-infrastructure/bloom)
