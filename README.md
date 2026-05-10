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
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#環境構築">環境構築</a>
    </li>
    <li>
    　<a href="#実行・操作方法">実行・操作方法</a>
      <ul>
        <li><a href="#configファイル作成">configファイル作成</a></li>
        <li><a href="#テレオペノード実行">テレオペノード実行</a></li>
        <li><a href="#アームスケールキャリブレーション">アームスケールキャリブレーション（Meta Quest）</a></li>
      </ul>
    </li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
  </ol>
</details>



<!-- 概要 -->
## 概要

<!-- ![SOBITS TELEOP](sobits_teleop/docs/img/sobits_teleop.png) -->

SOBITSのロボットをjoystick(PS4, PS5), Meta Quest, Keyboardで遠隔操作するためのパッケージ．\
Meta Questのセットアップに関しては[こちら](https://github.com/TeamSOBITS/meta_quest_teleoperation)をクリック．

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
| ROS    | Jazzy Jalisco|
| Python | 3.12~ |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### インストール方法

1. ROSの`src`フォルダに移動します．
    ```sh
    $ cd ~/colcon_ws/src/
    ```

2. 本レポジトリをcloneします．
    ```sh
    $ git clone https://github.com/TeamSOBITS/sobits_teleop
    ```

3. レポジトリの中へ移動します．
    ```sh
    $ cd sobits_teleop/
    ```

4. 依存パッケージをインストールします．
    ```sh
    $ bash install.sh
    ```

5. パッケージをコンパイルします．
   ```bash
   $ cd ~/colcon_ws/
   $ colcon build --symlink-install
   $ source ~/colcon_ws/install/setup.sh
   ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 実行・操作方法 -->
## 実行・操作方法
sobits_teleopを使う上での基本的な流れ

1. configファイル作成
   - ロボットと，デバイスに対応するconfigファイルを作成する．
2. テレオペノード実行
   - デバイスがPCと接続されていることを確認し、テレオペノードを実行する．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### configファイル作成
ロボットとテレオペで使用するデバイス（ps4, Meta Questなど）を選定する．

`sobits_teleop/config/{使用するロボット}/{使用するロボット}.yaml`にロボットの設定、`sobits_teleop/config/{使用するロボット}/{使用するデバイス}.yaml`にコントローラの設定をする．

<details>
<summary>robot.yaml(例) </summary>

```yaml

/**:
  ros__parameters:
  
    robot_topic_name:
      joint_states_topic : joint_states　# joint_statesのトピック名を指定
      joint_trajectory_topic: # 各ポジションのjoint_trajecotyのトピック名を指定
        head : head_position_controller/joint_trajectory
        body : body_position_controller/joint_trajectory
        arm_left : arm_left_position_controller/joint_trajectory
        arm_right : arm_right_position_controller/joint_trajectory
      cmd_vel_topic : cmd_vel　# command_velocityのトピック名を指定
```

</details>

<details>
<summary>{使用するデバイス}.yaml(例) </summary>

```yaml

/**:
  ros__parameters:

    control_joints: # 操作するjoint_trajectory_controllerを定義する
      groups:
      - head
      - body
      - arm_left
      - arm_right
      head:
        names: # joint_trajectory_controllerのjointを定義する
        - head_tilt_joint
        - head_pan_joint
        head_tilt_joint:
          button : 2           # トリガーの定義
          fast_button : 6      # トリガーを押している間、動きを早める
          axis : 1             # ジョイスティックの傾きで動かす
          axis_sign : 1        # ジョイスティックの正負を変更
          speed : 0.1          # トリガー押下時のスピード
          fast_speed : 0.5     # fast_button押下時のスピード
      ...

    control_poses: # 定義済のポーズにロボットを動かす
      trigger : 8        # トリガーの定義
      pose_list:         # 定義済のポーズリストを定義
        - initial_pose
        - ninja_pose
        - detecting_high_pose
        - pre_manipulation_pose
      initial_pose:
        button : 2            # トリガー押下状態で押されたとき、initial_poseにする
      ...

    control_velocity: # 車輪を動かす
      button : 5               # トリガーの定義
      fast_button : 7          # トリガーを押している間、動きを早める
      linear_x_axis : 1        # ジョイスティックの傾きにより前進後退する
      linear_y_axis : 0        # ジョイスティックの傾きにより左右移動する
      angular_axis : 3         # ジョイスティックの傾きにより旋回する
      axis_sign : 1            # 旋回時のジョイスティックの正負を変更
      linear_scale : 0.1       # linearのスピード
      angular_scale : 0.3      # angularのスピード
      fast_linear_scale : 0.2  # fast_button押下時のlinearスピード
      fast_angular_scale : 0.6 # fast_button押下時のangularスピード
```

</details>

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### テレオペノード実行
使用するデバイスとPCをBluetooth経由などで接続し、**sobits_teleop.launch.py**の**device**を使用するデバイスに合わせて書き換えて、実行する．

例
```
declare_device_cmd = DeclareLaunchArgument(
        'device',
        default_value='ps4', 
        # default_value='ps5',
        # default_value='quest',
        # default_value='keyboard',
        description='Input device type: ps4, quest, keyboard'
    )
```
<details>
<summary>DUALSHOCKの場合</summary>

1. DUALSHOCKをBluetoothでPCと接続し、`jstest-gtk`コマンドでデバイスと接続していることを確認する．
> [!Note]
> ds4drvをDocker container内で使用する場合、`/dev/input/`と`/run/udev`をマウントする．
2. その後、**sobits_teleop.launch.py**を起動し、yamlファイルで設定した操作方法を参考にロボットを操作する．

</details>

<details>
<summary>Meta Questの場合</summary>


1. 使用するMeta QuestとPCが同一WIFIに接続しているか確認する．

2. PC側で`hostname -I`コマンドなどを使用して**ROS_IP**を確認する．

3. Meta Questを起動し、事前にセットアップした**Unity Project**を起動する.
（Meta Questのセットアップに関しては[こちら](https://github.com/TeamSOBITS/meta_quest_teleoperation)を参考）

4. Unity Projectを起動したら、画面上にある**ROS_IP**を確認し、左側コントローラの**メニューボタン**を押してROS_IPを入力し、変更する.

5. その後、**sobits_teleop.launch.py**を起動し、yamlファイルで設定した操作方法を参考にロボットを操作する．

</details>

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

---

## アームスケールキャリブレーション（Meta Quest）

Meta Questコントローラでアームのテレオペを行う場合、`quest.yaml`の`scale`パラメータがオペレータのアームリーチとロボットのアームリーチを対応付ける．`arm_scale_calibrator`ツールを使うと、自分のアームリーチを実測し、適切な`scale`値を自動的に計算できる．

### 実行タイミング

オペレータごとに一度実行する（ロボットのURDFのアームリーチが変わった場合も再実行）．結果として得られた値を`config/{ロボット名}/quest.yaml`の`right.scale`と`left.scale`に設定する．

### 設定ファイル

各ロボットのキャリブレーション設定は`config/{ロボット名}/arm_scale_calibrator.yaml`にある：

```yaml
# 肩からEEまでのフルキネマティックチェーン（メートル）— ロボットURDFより
robot_arm_reach_m: 1.2926

# コントローラ位置のルックアップに使用するTFフレーム．
# 片腕ロボットの場合、使用しないアームのフレームを "" に設定して無効化できる．
right_frame:  "right_controller_odom"
left_frame:   "left_controller_odom"
parent_frame: "base_footprint"

# Joyメッセージにおける右グリップの軸インデックス
grip_axis: 7
```

**片腕ロボット**の場合、使用しない側のフレームを空文字列に設定する：

```yaml
right_frame: "right_controller_odom"
left_frame:  ""   # 無効化 — 右アームのみ計測する
```

### 起動方法

Questが接続されTFをパブリッシュしている状態で実行する：

```sh
ros2 run sobits_teleop arm_scale_calibrator --ros-args \
  --params-file install/sobits_teleop/share/sobits_teleop/config/{ロボット名}/arm_scale_calibrator.yaml \
  -p joy_topic:=/{ロボット名}/joy
```

> [!Note]
> `{ロボット名}`は使用するロボット名に置き換える（例：`sobit_home`）．

### 手順

**右グリップボタン**を使って2ステップで計測する：

**ステップ1 — 開始位置の設定**
1. 両コントローラーを持ち、自然な立ち姿勢をとる．
2. **両腕をまっすぐ前方に伸ばす**（ロボットの方向を向くように）．
3. **右グリップボタン**を押して離し、開始位置を記録する．

**ステップ2 — T字ポーズへのスウィープ**
1. 両腕をゆっくりと**左右真横に開いていく**（肘を伸ばしたまま）．
2. T字ポーズで完全に腕が伸びたら、**右グリップボタン**を押して離す．

> [!Note]
> スウィープは2秒以上かける必要がある．短すぎる場合は警告が表示され、再度試みることができる．

### 結果の確認

計測終了後、以下のような結果がコンソールに表示される：

```
=== RESULTS ===
  Human arm reach used   : 0.9150 m
  Robot arm reach        : 1.2926 m

  Recommended scale = 1.2926 / 0.9150 = 1.4126

Update config/sobit_home/quest.yaml:
    scale: 1.4126   # (both right and left)
```

表示された`scale`値を使用するロボットの`quest.yaml`に設定する：

```yaml
right:
  scale: 1.4126
left:
  scale: 1.4126
```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<!-- マイルストーン -->
## マイルストーン

- [ ] 疑似逆運動学の追加
- [ ] Meta Questでの逆運動学の追加

現時点のバッグや新規機能の依頼を確認するために[Issueページ][issues-url] をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
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
