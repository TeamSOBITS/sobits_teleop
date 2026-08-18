<a name="readme-top"></a>

[EN](README.md) | [JA](README_ja.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBITS TELEOP

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li><a href="#概要">概要</a></li>
    <li><a href="#環境構築">環境構築</a></li>
    <li>
      <a href="#実行・操作方法">実行・操作方法</a>
      <ul>
        <li><a href="#configファイル作成">configファイル作成</a></li>
        <li><a href="#起動引数">起動引数</a></li>
        <li><a href="#テレオペノード実行">テレオペノード実行</a></li>
        <li><a href="#アーム追跡バックエンドmeta-quest">アーム追跡バックエンド（Meta Quest）</a></li>
        <li><a href="#新しいロボットへの移植">新しいロボットへの移植</a></li>
        <li><a href="#アームスケールキャリブレーションmeta-quest">アームスケールキャリブレーション（Meta Quest）</a></li>
        <li><a href="#シミュレーションモード">シミュレーションモード</a></li>
      </ul>
    </li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
  </ol>
</details>



<!-- 概要 -->
## 概要

ロボットをジョイスティック（PS4, PS5）、Meta Quest、またはキーボードで遠隔操作するためのパッケージ．\
Meta Questのセットアップについては[こちら](https://github.com/TeamSOBITS/meta_quest_teleoperation)を参照．

本パッケージは**ロボット非依存**です．ロボット固有の設定はすべて1つのconfigディレクトリ
（`config/{robot_name}/`）にまとまっており，起動時に`robot_name:=<名前>`で選択します．
SOBITSロボット（`sobit_home`、`sobit_pro`、`sobit_edu`、`sobit_mini`、`sobit_light`）用の
configは同梱済みで，新しいロボットへの移植はこのディレクトリを作成するだけです —
コードやlaunchの変更は不要です（[新しいロボットへの移植](#新しいロボットへの移植)参照）．

対応入力デバイス：`ps4`、`ps5`、`quest`、`keyboard`

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 環境構築 -->
## 環境構築

ここで，本レポジトリのセットアップ方法について説明します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 環境条件

まず，以下の環境を整えてから，次のインストール段階に進んでください．

| System  | Version |
| --- | --- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS    | Jazzy Jalisco |
| Python | 3.12+ |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### インストール方法

1. ROSの`src`フォルダに移動します．
    ```sh
    cd ~/colcon_ws/src/
    ```

2. 本レポジトリをcloneします．
    ```sh
    git clone -b jazzy-devel https://github.com/TeamSOBITS/sobits_teleop
    ```

3. レポジトリの中へ移動します．
    ```sh
    cd sobits_teleop/
    ```

4. 依存パッケージをインストールします．
    ```sh
    bash install.sh
    ```

5. パッケージをビルドします．
    ```sh
    cd ~/colcon_ws/
    colcon build --symlink-install
    source ~/colcon_ws/install/setup.bash
    ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 実行・操作方法 -->
## 実行・操作方法

基本的な流れ：

1. ロボットと入力デバイスに対応するconfigファイルを確認・作成する．
2. 入力デバイスをPCに接続する．
3. 適切な引数を指定してテレオペノードを起動する．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

---

### configファイル作成

各ロボットのconfigは`config/{robot_name}/`以下に配置します．
このディレクトリが本パッケージ内で唯一のロボット固有部分です：

| ファイル | 役割 |
|---|---|
| `robot.yaml` | ジョイントコントローラとcmd_velのトピック名 |
| `{device}.yaml` | 使用するデバイスのボタン・軸マッピング |
| `arm_backend_plan.yaml` | Plan-and-replaceアーム追跡バックエンドのチューニング（Quest使用時のみ） |
| `arm_backend_servo.yaml` | MoveIt Servoアーム追跡バックエンドのチューニング（Quest使用時のみ） |
| `arm_scale_calibrator.yaml` | アームリーチキャリブレーションパラメータ（Quest使用時のみ） |

**アームの構成情報**（アームの一覧・planning group・目標/エンドエフェクタのフレーム・
コントローラトピック）は`{device}.yaml`（アームごとの`arm:`ブロック）と`robot.yaml`
（trajectoryトピックのマップ）に一度だけ定義します．launcherが起動するバックエンドへ
注入するため，2つの`arm_backend_*.yaml`はチューニングのみを持ち，ほぼロボット非依存です．

<details>
<summary>robot.yaml（例）</summary>

```yaml
/**:
  ros__parameters:
    robot_topic_name:
      joint_states_topic: joint_states
      joint_trajectory_topic:
        head:      head_position_controller/joint_trajectory
        body:      body_position_controller/joint_trajectory
        arm_left:  arm_left_position_controller/joint_trajectory
        arm_right: arm_right_position_controller/joint_trajectory
      cmd_vel_topic: cmd_vel
```

</details>

<details>
<summary>{device}.yaml（例）</summary>

```yaml
/**:
  ros__parameters:

    control_joints:       # 操作するjoint_trajectory_controllerを定義
      groups:
        - head
        - arm_left
      head:
        names:
          - head_tilt_joint
          - head_pan_joint
        head_tilt_joint:
          button:      2    # 有効化ボタン
          fast_button: 6    # 押している間、高速モード
          axis:        1    # ジョイスティック軸
          axis_sign:   1    # 正負を反転する場合は-1
          speed:       0.1  # 通常速度
          fast_speed:  0.5  # 高速モード時の速度

    control_poses:        # 定義済みポーズへの移動
      trigger: 8            # 任意の修飾ボタン．不要なら省略
      time_from_start: 3.0  # ポーズ到達までの既定秒数
      pose_list:
        - initial_pose
        - pre_manipulation_pose
      initial_pose:
        button: 2
        # `groups`を書くとポーズをこの場で定義し，関節軌道として配信する．
        # 省略した場合はpose_nameをMoveToPoseアクションで解決する．
        groups:
          - head
          - arm_left
        head:
          joints:    [ head_pan_joint, head_tilt_joint ]
          positions: [ 0.0,            0.0             ]
        arm_left:
          joints:    [ arm_left_elbow_joint ]
          positions: [ 2.5 ]
      pre_manipulation_pose:
        button: 3           # `groups`なし -> MoveToPoseアクションを使用

    control_velocity:     # 台車制御
      button:             5
      fast_button:        7
      linear_x_axis:      1
      linear_y_axis:      0
      angular_axis:       3
      axis_sign:          1
      linear_scale:       0.1
      angular_scale:      0.3
      fast_linear_scale:  0.2
      fast_angular_scale: 0.6
```

</details>

#### ポーズのバックエンド

`control_poses`の各エントリは，ポーズごとに次の2通りのいずれかで解決されます．

| `groups`の記述 | バックエンド | 動作 |
|---|---|---|
| あり | 関節軌道トピック | YAMLに書いた関節と目標値をそのまま各グループのコントローラへ配信する．アクションサーバは不要． |
| なし | `MoveToPose`アクション | `pose_name`のみを送り，ポーズはサーバ側で解決される． |

`groups`に並べる名前は`robot.yaml`で定義した関節グループである必要があり，
そこから軌道トピックが決まります（グループごとに`joint_trajectory_topic`で
上書き可）．`joints`と`positions`は同じ要素数でなければならず，不一致の
グループはロボットを動かす前に起動時エラーとしてスキップされます．記載しない
関節には指令を出さないため，コントローラが最後に保持した値のままになります．
単一グループのポーズなら`groups`を省略し，`joints`/`positions`をポーズ直下に
書くこともできます．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

---

### 起動引数

すべての設定はCLI引数で指定します．launchファイルを直接編集する必要はありません．

| 引数 | デフォルト | 説明 |
|---|---|---|
| `robot_name` | `sobit_home` | ロボット名（configディレクトリの選択に使用） |
| `device` | `ps4` | 入力デバイス：`ps4`、`ps5`、`quest`、`keyboard` |
| `joystick_device` | `/dev/input/js0` | ジョイスティックデバイスパス（PS4/PS5のみ） |
| `ros_ip` | `0.0.0.0` | Quest TCP接続用のPC IPアドレス |
| `use_ds4drv` | `True` | `ds4drv`を同時起動する（PS4のみ） |
| `use_moveit` | `false` | アーム追跡バックエンドを起動する（Questのみ） |
| `use_servo` | `false` | Plan-and-replaceの代わりにMoveIt Servoバックエンドを使用する（`use_moveit:=true`が必要） |
| `use_sim_time` | `false` | シミュレーション時刻を使用する（Gazebo） |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

---

### テレオペノード実行

#### PS4 / PS5

1. コントローラをBluetoothでPCとペアリングする．
2. `jstest-gtk`コマンドで接続を確認する．
3. 起動する：
    ```sh
    ros2 launch sobits_teleop sobits_teleop.launch.py \
      robot_name:=sobit_home \
      device:=ps4
    ```

> [!Note]
> Dockerコンテナ内で`ds4drv`を使用する場合は、`/dev/input/`と`/run/udev`をマウントすること．

#### Meta Quest

1. QuestとPCが同一Wi-Fiネットワークに接続されていることを確認する．
2. PCのIPアドレスを確認する：
    ```sh
    hostname -I
    ```
3. テレオペノードを起動する：
    ```sh
    ros2 launch sobits_teleop sobits_teleop.launch.py \
      robot_name:=sobit_home \
      device:=quest \
      ros_ip:=<PC_IPアドレス>
    ```
4. Quest側でUnityプロジェクトを起動し、左コントローラの**メニューボタン**でROS_IPを入力・設定する．

Meta Questのセットアップは[meta_quest_teleoperation](https://github.com/TeamSOBITS/meta_quest_teleoperation)を参照．

#### キーボード

```sh
ros2 launch sobits_teleop sobits_teleop.launch.py \
  robot_name:=sobit_home \
  device:=keyboard
```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

---

### アーム追跡バックエンド（Meta Quest）

ハンドコントローラのTFポーズをリアルタイムで追従するバックエンドが2種類あります．
どちらも同じインタフェース — `{side}_target_link` TFと
`/{robot_name}/{arm_name}/moveit_track_enabled`（`std_msgs/Bool`，グリップボタンが
自動でパブリッシュ）— を使うため，launch引数1つで切り替えられます．

| バックエンド | 選択方法 | 追従方式 |
|---|---|---|
| Plan-and-replace（`moveit_arm_controller`） | `use_moveit:=true` | MoveIt2で短いCartesian計画をストリーミング；特異点付近はOMPLへフォールバック |
| MoveIt Servo（`servo_node` + `servo_target_bridge`） | `use_moveit:=true use_servo:=true` | アームごとの`moveit_servo`インスタンスが50Hzで微分IK |

```sh
ros2 launch sobits_teleop sobits_teleop.launch.py \
  robot_name:=<robot_name> \
  device:=quest \
  ros_ip:=<PC_IPアドレス> \
  use_moveit:=true \
  use_servo:=true        # 省略するとPlan-and-replaceバックエンド
```

両launcherは`launch/include/`にあり，アーム構成を`quest.yaml`/`robot.yaml`から
注入するため，バックエンドのconfigはチューニングのみです．

#### Plan-and-replaceのチューニング — `config/{robot_name}/arm_backend_plan.yaml`

```yaml
arm_teleop:
  update_rate_hz:          50.0   # 追従ループ周波数
  max_cartesian_step_m:    0.10   # 1サイクルの最大ステップ
  eef_step_m:              0.02   # Cartesian補間解像度
  min_cartesian_fraction:   0.2   # この値を下回るとOMPLにフォールバック
  replan_threshold_m:      0.03   # 目標がこの距離動いたら再計画
  preempt_threshold_m:     0.30   # 実行中の軌道をキャンセルする距離
  arrival_threshold_m:     0.03   # EEがこの距離以内なら計画をスキップ
  velocity_scaling:        0.90
  acceleration_scaling:    0.80
  publish_mode: topic             # 軌道ストリーミング（テレオペに最適）
```

#### Servoのチューニング — `config/{robot_name}/arm_backend_servo.yaml`

Servoスタック全ノードで1ファイル；共有の`/**:`セクションがチューニングを持ちます．
移植時に確認すべきロボット依存の項目：

```yaml
moveit_servo:
  scale: {linear: 1.5, rotational: 3.0}  # EE速度上限 [m/s, rad/s]
  publish_joint_velocities: false  # アームのJTCが終端速度非ゼロの軌道を
                                   # 拒否する場合はfalseのまま
  lower_singularity_threshold: 50.0
  hard_stop_singularity_threshold: 200.0  # 無効化しないこと（関節ワインドアップ）
  joint_limit_margins: [0.02]

servo_bridge:
  pose_rate_hz: 100.0
  max_reach: 1.10   # 各アームの肩を中心としたリーチクランプ球の半径 [m]
                    # — 対象ロボットの肩→EEチェーン長の約90〜95%に設定
```

bridgeは目標を`max_reach`球にクランプするため，届かない位置の手をアームが追いかけて
完全伸展の特異点に陥ることを防ぎます．

##### 特異点ハルトからの復帰

`hard_stop_singularity_threshold`を超えるとServoは非常停止をラッチし
（`HALT_FOR_SINGULARITY`），以降はポーズ指令を無視します．そのため目標を
与え直すだけでは復帰できず，本来はグリップを離して特異点から離れた位置で
握り直す必要があります．bridgeは各Servoの`~/status`を監視し，自動で復帰します．

```yaml
servo_bridge:
  reset_on_halt: true           # ラッチされたハルトで下記の復帰処理を実行
  reset_cooldown_s: 2.0         # 復帰試行の最小間隔
  joint_escape_time_s: 1.0      # 脱出軌道の所要時間．0で無効
  joint_escape_lookback_s: 1.0  # この秒数だけ遡った姿勢へ脱出する
  escape_step: 0.005            # 1周期あたりのデカルト移動量 [m]．0で無効
  escape_timeout_s: 2.0         # これを超えたら諦めて握り直しを促す
```

復帰処理はServoを一時停止し，**関節空間で**アームを脱出させてから，脱出軌道が
実行される時間を待ってServoを再開します．関節空間の指令はヤコビアンを使わない
ため，特異点に阻まれません．脱出先は最新の姿勢ではなく`joint_escape_lookback_s`
だけ遡った関節配置です — ハルト直前の指令は特異点のすぐ隣にあり，そこへ戻ると
再びハルトするためです．`escape_step`はデカルト指令を最後に正常だったEE姿勢へ
向けて戻しますが，これはアームが特異点上に停止する前にしか効果がありません．
ハルトが`escape_timeout_s`を超えて続く場合は上書きを解除して握り直しを促すため，
復帰不能なハルトがアームを永久に拘束することはありません．

##### アームごとの命名

アームごとのフレームやトピックはテンプレートからアーム名を使って導出されるため，
規約に従うアームは個別の設定を一切必要としません．`{arm}`はアーム名そのもの
（`arm_right`），`{side}`は`arm_`接頭辞を除いた名前（`right`）に展開されます．

```yaml
servo_bridge:
  naming:
    reach_origin_frame:      "{arm}_shoulder_tilt_link"
    servo_node:              "servo_{arm}"
    enable_topic:            "{arm}/moveit_track_enabled"
    joint_traj_topic:        "{arm}_position_controller/joint_trajectory"
    status_topic:            "{servo_node}/status"
```

ロボットのリンク名が異なる場合はテンプレートを書き換えてください．再ビルドは
不要です．`status_topic`は解決後のノード名から`{servo_node}`を展開するため，
`servo_node`を上書きすればそれに追従します．特定のアームだけ変えたい場合は
`servo_bridge.{arm_name}.*`に直接キーを書けば，テンプレートより優先されます．

ターゲットフレームとエンドエフェクタフレームはここに含めません．`quest.yaml`が
それらの定義元であり，servoのlauncherがbridgeへ転送するため，定義は一箇所です．
bridgeを単体起動した場合はC++側の既定値が使われます．

Servoバックエンドには`ros-$ROS_DISTRO-moveit-servo`（`install.sh`でインストール）と，
`/{robot_name}`名前空間で動作中の`move_group`が必要です — launcherが起動時に
ロボットモデルをそこから取得します．

#### グリッパ操作（Quest，両バックエンド共通）

| 入力 | 動作 |
|---|---|
| グリップボタン（`enable_axis`） | 押している間アームを追従 |
| ポーズボタン（`pose_button`） | `pose_open`/`pose_close`をトグル |
| トリガ + スティック左右 | アダプティブ開閉 |
| トリガ + スティック上下 | 把持タイプ関節（`single_joint.name`）を回転 |

#### 頭部操作（Quest）

`quest_control.head.head_mode`を押している間，頭部追従がラッチされ，頭部がHMDの
姿勢に追従します．ラッチはHMDのTFではなく`/joy`のトリガ状態で駆動されるため，
QuestのTFが途切れている間でも離せば必ず追従が停止します．TFが復帰した際は
現在の姿勢で再アンカーするため，途切れていた分の差分が一度にジャンプとして
現れることはありません．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

---

### 新しいロボットへの移植

1. `config/{robot_name}/`を作成し，`robot.yaml`（コントローラトピック）と使用する
   デバイスごとの`{device}.yaml`を用意する．
2. Questでアームテレオペを行う場合，`quest.yaml`に`arm_<side>`/`hand_<side>`
   グループ（`target_frame_name:`、`end_effector_frame_name:`、グリッパのマッピング）を
   追加する — これが両バックエンド共通のアーム構成の単一定義源になります．
3. 既存ロボットの`arm_backend_plan.yaml`/`arm_backend_servo.yaml`をコピーし，
   チューニングを調整する（`max_reach`をアーム長に合わせる；速度上限；しきい値）．
4. `robot_name:={robot_name}`で起動する — それ以外の変更は不要です．

前提条件：アームが`joint_trajectory_controller`のトピックインタフェースで駆動される
こと（`robot.yaml`参照），アームごとに1つのplanning groupを持ち，SRDFの最後のリンクが
エンドエフェクタであるMoveIt configがあること，`/{robot_name}`名前空間で`move_group`
が動作していること．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

---

### アームスケールキャリブレーション（Meta Quest）

`quest.yaml`の`motion_scale`パラメータは人間のアームリーチとロボットのアームリーチを対応付けます．servo自身のEE速度上限である`moveit_servo.scale`とは別物のため，名前を分けています．`arm_scale_calibrator`ツールを使うと，適切な`motion_scale`値を自動的に計算できます．

#### 実行タイミング

オペレータごとに一度実行する（ロボットURDFのアームリーチが変わった場合も再実行）．結果を`config/{robot_name}/quest.yaml`の`arm_right.motion_scale`と`arm_left.motion_scale`に設定する．

#### 設定ファイル

`config/{robot_name}/arm_scale_calibrator.yaml`：

```yaml
robot_arm_reach_m: 1.2926      # 肩からEEまでのフルキネマティックチェーン（URDFより）

right_frame:  "right_controller_odom"
left_frame:   "left_controller_odom"
parent_frame: "base_footprint"

grip_axis: 7                   # Joyメッセージにおける右グリップの軸インデックス
```

**片腕ロボット**の場合、使用しない側のフレームを`""`に設定する：

```yaml
right_frame: "right_controller_odom"
left_frame:  ""   # 無効化 — 右アームのみ計測
```

#### 起動方法

```sh
ros2 run sobits_teleop arm_scale_calibrator --ros-args \
  --params-file install/sobits_teleop/share/sobits_teleop/config/{robot_name}/arm_scale_calibrator.yaml \
  -p joy_topic:=/{robot_name}/joy
```

> [!Note]
> `{robot_name}`は使用するロボット名に置き換える（例：`sobit_home`）．

#### 手順

**ステップ1 — 開始位置の設定**
1. 両コントローラーを持ち、自然な立ち姿勢をとる．
2. **両腕をまっすぐ前方に伸ばす**（ロボットの方向を向くように）．
3. **右グリップボタン**を押して離し、開始位置を記録する．

**ステップ2 — T字ポーズへのスウィープ**
1. 両腕をゆっくりと**左右真横に開いていく**（肘を伸ばしたまま）．
2. T字ポーズで完全に腕が伸びたら、**右グリップボタン**を押して離す．

> [!Note]
> スウィープは2秒以上かける必要がある．短すぎる場合は警告が表示され、再度試みることができる．

#### 結果の確認

```
=== RESULTS ===
  Human arm reach used   : 0.9150 m
  Robot arm reach        : 1.2926 m

  Recommended motion_scale = 1.2926 / 0.9150 = 1.4126

Update config/sobit_home/quest.yaml:
    motion_scale: 1.4126   # (both right and left)
```

表示された`scale`値を`quest.yaml`に設定する：

```yaml
arm_right:
  motion_scale: 1.4126
arm_left:
  motion_scale: 1.4126
```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

---

### シミュレーションモード

Gazeboシミュレーションと組み合わせて使用する場合は`use_sim_time:=true`を指定する：

```sh
ros2 launch sobits_teleop sobits_teleop.launch.py \
  robot_name:=sobit_home \
  device:=keyboard \
  use_sim_time:=true
```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- マイルストーン -->
## マイルストーン

- [ ] 疑似逆運動学の追加
- [ ] パラメータ読み込みを `generate_parameter_library` に移行（設定ミスを起動時エラーに）
- [ ] 左腕ハード復帰後に `check_collisions: true` へ戻す（必要なら ACM でペア除外）

現時点のバグや新規機能の依頼は[Issueページ][issues-url]をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/sobits_teleop.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/sobits_teleop/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/sobits_teleop.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/sobits_teleop/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/sobits_teleop.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/sobits_teleop/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/sobits_teleop.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/sobits_teleop/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/sobits_teleop.svg?style=for-the-badge
[license-url]: LICENSE
