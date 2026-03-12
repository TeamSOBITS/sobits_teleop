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
使用するデバイスとPCをBluetooth経由などで接続，dualshockを使う場合、`jstest-gtk`コマンドでデバイスと接続していることを確認する．
launchファイルを設定し，実行する．

> [!Note]
> ds4drvをDocker container内で使用する場合、`/dev/input/`と`/run/udev`をマウントする．

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
