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
| Ubuntu | 22.04 (Noble Numbat) |
| ROS    | Humble Hawksbill|
| Python | 3.10~ |

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

`sobits_teleop/config/{使用するロボット}/{使用するデバイス}.yaml`に設定ファイルを作成する．
その後、動かしたいジョイントや`cmd_vel`の設定をする．

<details>
<summary>作成例： </summary>

```yaml

joints:
  head_tilt_joint:
    joint_trajectory_topic: /sobit_home/head_position_controller/joint_trajectory
    mode_button: 2           # △
    fast_mode_button: 6      # L2
    axis: 1                  # 左スティック上下
    axis_sign: 1
    speed: 0.1
    fast_speed: 0.5
    min_pos: -0.7853
    max_pos: 0.52
  head_pan_joint:
    joint_trajectory_topic: /sobit_home/head_position_controller/joint_trajectory
    mode_button: 2           # △
    fast_mode_button: 6      # L2
    axis: 0                  # 左スティック左右
    axis_sign: 1
    speed: 0.1
    fast_speed: 0.5
    min_pos: -0.8726
    max_pos: 0.8726

cmd_vel:
  cmd_vel_topic: "/sobit_home/cmd_vel"
  mode_button: 5            # R1
  fast_mode_button: 7       # R2
  linear_x_axis: 1          # 左スティック上下
  linear_y_axis: 0          # 左スティック左右
  angular_axis: 3           # 右スティック左右
  linear_scale: 0.1
  angular_scale: 0.1
  fast_linear_scale: 0.2
  fast_angular_scale: 0.2
```

</details>

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### テレオペノード実行
使用するデバイスとPCをBluetooth経由などで接続し，`jstest-gtk`コマンドでジョイスティックコントローラと接続していることを確認する．
launchファイルを設定し，実行する．

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
