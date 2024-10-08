 <a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# bag handle estimator

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#紙袋の取っ手を推定するパッケージ">紙袋の取っ手を推定するパッケージ</a>
    </li>
    <li>
      <a href="#セットアップ">セットアップ</a>
      <ul>
        <li><a href="#環境条件">環境条件</a></li>
        <li><a href="#インストール方法">インストール方法</a></li>
      </ul>
    </li>
    <li>
    　<a href="#実行操作方法">実行・操作方法</a>
    </li>
  </ol>
</details>


<!-- レポジトリの概要 -->
## 紙袋の取っ手を推定するパッケージ

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

![Execute Result](img/estimate.png)

紙袋の取っ手の中心を”handle_point”のTFで出力．
一度検出した場所のTFを常に出し続けます．

後述するexecute_ctrlのフラグに関わらず，ノードが生存する間はTFを出し続けます．

検出の開始，停止はsobits_msgsのRunCtrl型のServiceで制御できます．


<!-- セットアップ -->
## セットアップ

ここで，本リポジトリのセットアップ方法について説明します．

### 環境条件

まず，以下の環境を整えてから，次のインストール段階に進んでください．

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 20.04 (Focal Fossa) |
| ROS | Noetic Ninjemys |
| Python | 3.8 |

> [!NOTE]
> `Ubuntu`や`ROS`のインストール方法に関しては，[SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6)を参照してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### インストール方法

1. ROSの`src`フォルダに移動します．
   ```sh
   $ roscd
   # もしくは，"cd ~/catkin_ws/"へ移動．
   $ cd src/
   ```
2. 本リポジトリをcloneします．
   ```sh
   $ git clone https://github.com/TeamSOBITS/bag_handle_estimator
   ```
3. リポジトリの中へ移動します．
   ```sh
   $ cd bag_handle_estimator/
   ```
4. パッケージをコンパイルします．
   ```sh
   $ roscd
   # もしくは，"cd ~/catkin_ws/"へ移動．
   $ catkin_make
   ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 実行・操作方法 -->
## 実行・操作方法
※先に[realsense_ros](https://github.com/TeamSOBITS/realsense_ros)をcloneしてinstall.shを実行しておいてください．



1. [handle_estimator.launch](launch/handle_estimator.launch)のパラメータを設定します．
   ```xml
   <!-- rvizを起動するかどうか  -->
    <arg name="rviz"                    default="false"/>
    <!-- 起動時に実行するかどうか -->
    <param name="execute_default" type="bool" value="true"/>
    <!-- 点群を出力するかどうか -->
	 <param name="pub_plane_cloud" type="bool" value="true"/>
    <!-- subscribeするtopic名 -->
    <param name="sub_point_topic_name" type="str" value="/hand_camera/depth_registered/points"/>
    <!-- base_frameの名前 -->
    <param name="base_frame_name" type="str" value="base_footprint"/>
    <!-- depthの範囲 -->
    <param name="depth_range_min_x" type="double" value="0.0"/>
    <param name="depth_range_max_x" type="double" value="1.2"/>
    <!-- widthの範囲 -->
    <param name="depth_range_min_y" type="double" value="-0.35"/>
    <param name="depth_range_max_y" type="double" value="0.35"/>
    <!-- heightの範囲 -->
    <param name="depth_range_min_z" type="double" value="0.5"/>
    <param name="depth_range_max_z" type="double" value="1.0"/>
   ```

2. RGB-Dカメラを起動します
   ```sh
   $ roslaunch realsense2_camera rs_rgbd.launch
   ```


2. [handle_estimator.launch](launch/handle_estimator.launch)というlaunchファイルを実行します．
   ```sh
   $ roslaunch bag_handle_estimator handle_estimator.launch
   ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### 検出の実行を切り替えるService型
```bash
/bag_handle_estimator/run_ctr [sobits_msgs/RunCtrl]
#Trueを送って検出開始 Falseを送って検出終了(defaultはTrue)
```

### Publications:
 * /bag_handle_estimater/cloud_plane [sensor_msgs/PointCloud2]
 * /rosout [rosgraph_msgs/Log]
 * /tf2 [tf2_msgs/TFMessage]


<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/bag_handle_estimator.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/bag_handle_estimator/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/bag_handle_estimator.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/bag_handle_estimator/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/bag_handle_estimator.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/bag_handle_estimator/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/bag_handle_estimator.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/bag_handle_estimator/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/bag_handle_estimator.svg?style=for-the-badge
[license-url]: LICENSE
