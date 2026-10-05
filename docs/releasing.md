# 公開と ROS apt リリースまで

通常ノード、hardware plugin、テストプログラムを全部用意しなければ ROS 公開が
できない、というルールはありません。本リポジトリでは用途に合わせて提供します。
C++ の特殊な機能を増やすことも公開条件ではありません。

現在は Lyrical を対象にした開発用の実装と自動テストです。公開用メタデータ、
実機検証、リリース手続きは残っています。GitHub の public 化だけでは ROS apt に入りません。

1. VESC の実機、CAN 中継器、Zenoh pico を含めて単位・位置範囲・指令途絶・通信途絶・
   再接続を検証する。利用できる VESC firmware、ROS バージョン、ゲートウェイ条件を記録する。
2. package.xml の maintainer `root@todo.todo` を開発者の実際の名前・連絡先へ更新する。
   初期コードの権利・出典を確認する。リポジトリの LICENSE は MIT だが、コピー元の権利確認を省略しない。
   同梱 `vesc_msgs` のライセンスは BSD-3-Clause で、同ディレクトリの LICENSE と
   上流の著者・メタデータを保持している。実際に自分でリリースを管理する場合は、
   著者情報を残したまま maintainer、repository URL、バージョンを実態に合わせる。
3. `vesc_msgs` を対象 distro へリリース済みの依存として解決できるようにする。
   現在の Lyrical rosdistro 一覧には `vesc_msgs` がないため、開発・CI は
   リポジトリ内へ同梱した f1tenth ROS 2 版をビルドする。取り込み元は
   [vesc_msgs/UPSTREAM.md](../vesc_msgs/UPSTREAM.md) に記録している。
   同梱だけでは apt 公開は完了しない。メッセージの上流メンテナと調整し、
   同梱 `vesc_msgs` も対象 distro へリリースするか、上流が対象 distro へ
   リリースした `vesc_msgs` に依存するかを決める。同名パッケージの release 元を
   重複させず、同名で異なる定義の fork も混在させない。
   README、設定例、CHANGELOG、バージョン、依存関係を整える。
   コードをコピーして sbgisen/vesc を利用する場合は同プロジェクトのライセンスと著作権表記を保持する。
4. CI のビルド・テストを GitHub 上で通し、対象 ROS バージョンを明示する。
   Jazzy/Humble を対応表に追加するなら API の互換実装と各 distro のビルド・テストを行う。
5. GitHub へ source release/tag を公開し、release repository と bloom のリリースを準備する。
   対象 distro の rosdistro 登録、ROS build farm の依存解決とビルドが通り、配布同期された後に
   apt で利用できる状態になる。具体的な手続きはリリース時の公式文書に従う。

参考: [rosdistro の公開手順](https://github.com/ros/rosdistro/blob/master/CONTRIBUTING.md)、
[bloom](https://github.com/ros-infrastructure/bloom)

公開 API は安定化が必要です。独自メッセージは廃止し、通常ノードの状態は
同梱した上流 `vesc_msgs`、指令は標準 ROS msg を使います。[移行案内](messages.md) に
型・単位・未取得値を記録しています。`vesc_msgs` は f1tenth 管理のコミュニティパッケージです。

abu2027 側へ導入するときは独立リポジトリを vcs の .repos ファイルや submodule などで
特定の tag/commit に固定すると、修正の追跡が容易です。コピーして取り込む場合も
元 commit を記録し、独立リポジトリを改修の基準にすると二重管理を抑えられます。
