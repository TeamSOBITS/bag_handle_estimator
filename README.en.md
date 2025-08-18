 <a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# Bag Handle Estimator

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#Introduction">Introduction</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li>
    　<a href="#launch-and-usage">Launch and Usage</a>
    </li>
  </ol>
</details>


<!-- INTRODUCTION -->
## Introduction
This repository provides a package for estimating paper bag handles.

![Execute Result](img/estimate.png)

The center of the handle of a paper bag is output as a "handle_point".
Once detected, the TF of the location is always output.

The start and stop of detection can be controlled by a service of type RunCtrl in sobits_msgs.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- GETTING STARTED -->
## Getting Started

This section describes how to set up this repository.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Prerequisites

First, please set up the following environment before proceeding to the next installation stage.

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS | Humble Hawksbill |
| Python | ~3.0 |

<!-- > [!NOTE]
> If you need to install `Ubuntu` or `ROS`, please check our [SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6). -->

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Installation

1. Go to the `src` folder of ROS2.
   ```sh
   $ cd ~/colcon_ws/src/
   ```
2. Clone this repository.
   ```sh
   $ git clone -b humble-devel https://github.com/TeamSOBITS/bag_handle_estimator
   ```
3. Navigate into the repository.
   ```sh
   $ cd bag_handle_estimator/
   ```
4. Install the dependent packages.
   ```sh
   $ bash install.sh
   ```
5. Compile the package.
   ```sh
   $ cd ~/colcon_ws/
   $ colcon build --symlink-install
   $ source ~/colcon_ws/install/setup.sh
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- LAUNCH AND USAGE EXAMPLES -->
## Launch and Usage
<!-- ※Please clone [realsense_ros](https://github.com/TeamSOBITS/realsense_ros) and run install.sh first. -->



1. Set the parameters inside [handle_estimator.launch.py](launch/handle_estimator.launch.py)．
   ```python
   rviz_arg = DeclareLaunchArgument(
      'rviz',
      # rvizを起動するかどうか
      default_value='true',
      description='Launch RViz'
    )

   handle_estimator_node = Node(
      package='bag_handle_estimator',
      executable='handle_estimator',
      name='handle_estimator',
      output='screen',
      parameters=[{
      # 起動時に実行するかどうか
      'execute_default': True,
      # 点群を出力するかどうか
      'pub_plane_cloud': True,
      # subscribeするtopic名
      'sub_point_topic_name': '/camera/camera/depth/color/points',
      # base_frameの名前
      'base_frame_name': 'base_footprint',
      # depthの範囲
      'depth_range_min_x': 0.0,
      'depth_range_max_x': 0.5,
      # widthの範囲
      'depth_range_min_y': -0.3,
      'depth_range_max_y': 0.3,
      # heightの範囲
      'depth_range_min_z': 0.0,
      'depth_range_max_z': 0.5
         }]
      )
   ``` 

2. Activate the RGB-D camera

3. Execute the launch file[handle_estimator.launch.py](launch/handle_estimator.launch.py)
   ```sh
   $ ros2 launch bag_handle_estimator handle_estimator.launch.py
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Toggle detection execution
```bash
/bag_handle_estimator/run_ctr [sobits_msgs/RunCtrl]
#Send True to start detection,Send False to end detection(default:True)
```

### Publications:
 * /bag_handle_estimater/cloud_plane [sensor_msgs/PointCloud2]
 * /rosout [rosgraph_msgs/Log]
 * /tf2 [tf2_msgs/TFMessage]


<p align="right">(<a href="#readme-top">back to top</a>)</p>


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
