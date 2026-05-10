<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

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
        <li><a href="#moveitアームコントローラmeta-quest">MoveItアームコントローラ（Meta Quest）</a></li>
        <li><a href="#アームスケールキャリブレーションmeta-quest">アームスケールキャリブレーション（Meta Quest）</a></li>
        <li><a href="#シミュレーションモード">シミュレーションモード</a></li>
      </ul>
    </li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
  </ol>
</details>



<!-- 概要 -->
## 概要

SOBITSのロボットをジョイスティック（PS4, PS5）、Meta Quest、またはキーボードで遠隔操作するためのパッケージ．\
Meta Questのセットアップについては[こちら](https://github.com/TeamSOBITS/meta_quest_teleoperation)を参照．

対応ロボット：`sobit_home`、`sobit_pro`、`sobit_edu`、`sobit_mini`、`sobit_light`\
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

各ロボットのconfigは`config/{robot_name}/`以下に配置します：

| ファイル | 役割 |
|---|---|
| `robot.yaml` | ジョイントコントローラとcmd_velのトピック名 |
| `{device}.yaml` | 使用するデバイスのボタン・軸マッピング |
| `moveit_arm_controller.yaml` | MoveItアーム追跡パラメータ（Quest使用時のみ） |
| `arm_scale_calibrator.yaml` | アームリーチキャリブレーションパラメータ（Quest使用時のみ） |

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
      trigger: 8
      pose_list:
        - initial_pose
        - pre_manipulation_pose
      initial_pose:
        button: 2

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
| `use_moveit` | `false` | MoveItアームコントローラを起動する（Questのみ） |
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

### MoveItアームコントローラ（Meta Quest）

Quest使用時にアームのテレオペを行う場合、`moveit_arm_controller`ノードがMoveIt2のCartesian経路計画を用いてコントローラのTFポーズをリアルタイムで追従します．

`use_moveit:=true`を指定することで有効になります：

```sh
ros2 launch sobits_teleop sobits_teleop.launch.py \
  robot_name:=sobit_home \
  device:=quest \
  ros_ip:=<PC_IPアドレス> \
  use_moveit:=true
```

設定ファイルは`config/{robot_name}/moveit_arm_controller.yaml`です：

```yaml
arm_teleop:
  arms: [arm_left, arm_right]   # SRDFのplanning group名と一致させること

  arm_left:
    planning_group:    "arm_left"
    target_frame:      "left_target_link"   # sobits_teleopがブロードキャストするTFフレーム
    base_frame:        "base_footprint"
    trajectory_topic:  "arm_left_position_controller/joint_trajectory"

  update_rate_hz:          25.0   # 追従ループ周波数
  max_cartesian_step_m:     0.5   # 1サイクルの最大ステップ
  eef_step_m:              0.03   # Cartesian補間解像度
  min_cartesian_fraction:   0.2   # この値を下回るとOMPLにフォールバック
  replan_threshold_m:      0.02   # アイドル時に目標がこの距離動いたら再計画
  preempt_threshold_m:     0.15   # 実行中の軌道をキャンセルして再計画する距離
  arrival_threshold_m:     0.03   # EEがこの距離以内なら計画をスキップ
  velocity_scaling:         0.6
  acceleration_scaling:     0.6
  traj_lookahead_ms:          40
  ompl_planning_timeout_s:   0.5
```

各アームの追従は`/{robot_name}/{arm_name}/moveit_track_enabled`（`std_msgs/Bool`）のパブリッシュで有効・無効を切り替えます．Questコントローラのグリップボタンが`sobits_teleop`を通じて自動的にパブリッシュします．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

---

### アームスケールキャリブレーション（Meta Quest）

`quest.yaml`の`scale`パラメータは人間のアームリーチとロボットのアームリーチを対応付けます．`arm_scale_calibrator`ツールを使うと、適切な`scale`値を自動的に計算できます．

#### 実行タイミング

オペレータごとに一度実行する（ロボットURDFのアームリーチが変わった場合も再実行）．結果を`config/{robot_name}/quest.yaml`の`right.scale`と`left.scale`に設定する．

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

  Recommended scale = 1.2926 / 0.9150 = 1.4126

Update config/sobit_home/quest.yaml:
    scale: 1.4126   # (both right and left)
```

表示された`scale`値を`quest.yaml`に設定する：

```yaml
right:
  scale: 1.4126
left:
  scale: 1.4126
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
