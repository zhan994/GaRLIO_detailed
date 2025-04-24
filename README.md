<p align="center">

  <h1 align="center"> 🌏 GaRLIO: Gravity enhanced Radar-LiDAR-Inertial Odometry [ICRA 2025]</h1>

  <p align="center">
    <a href="https://github.com/ChiyunNoh/GaRLIO"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/ChiyunNoh/GaRLIO"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://arxiv.org/abs/2502.07703"><img src="https://img.shields.io/badge/Paper-pdf-<COLOR>.svg?style=flat-square" /></a>
    </a>
  </p>
  
  <p align="center">
    <a href="https://rpm.snu.ac.kr"><strong>Chiyun Noh</strong></a>
    ·
    <a href="https://rpm.snu.ac.kr"><strong>Wooseong Yang</strong></a>
    ·
    <a href="https://minwoo0611.github.io/about/"><strong>Minwoo Jung</strong></a>
    ·
    <a href="https://sangwoojung98.github.io"><strong>Sangwoo Jung</strong></a>
    ·
    <a href="https://ayoungk.github.io/"><strong>Ayoung Kim</strong></a><sup>†</sup>
    <br/>
    <a href="https://rpm.snu.ac.kr/"><strong>Robust Perception and Mobile Robotics Lab (RPM)</strong></a>
  </p>

  <h3 align="center"><a href="https://arxiv.org/abs/2502.07703">Paper</a> | <a href="https://youtu.be/zeH3RQdIviw?si=aZg_WZfn4ErqkNu8">Video</a></h3>
  <div align="center"></div>
</p>


This repository contains the code for **GaRLIO: Gravity enhanced Radar-LiDAR-Inertial Odometry**, which is accepted by ICRA 2025. 

<!-- TABLE OF CONTENTS -->
<details open="open" style='padding: 10px; border-radius:5px 30px 30px 5px; border-style: solid; border-width: 1px;'>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#abstract">Abstract</a>
    </li>
    <li>
      <a href="#prerequisites">Prerequisites</a>
    </li>
    <li>
      <a href="#dataset">Dataset</a>
    </li>
    <li>
      <a href="#build">Build</a>
    </li>
    <li>
      <a href="#launch">Launch</a>
    </li>
    <li>
      <a href="#acknowledgments">Acknowledgements</a>
    </li>
    <li>
      <a href="#citation">Citation</a>
    </li>
    <li>
      <a href="#contact">Contact</a>
    </li>
  </ol>
</details>

## Update
[24/04/2025]: Full code of GaRLIO released.

[28/01/2025]: GaRLIO is accepted to IROS 2024.

## Abstract

<details>
  <summary>click to expand</summary>
Recently, gravity has been highlighted as a crucial constraint for state estimation to alleviate potential vertical drift. 
Existing online gravity estimation methods rely on pose estimation combined with IMU measurements, which is considered best practice when direct velocity measurements are unavailable. However, with radar sensors providing direct velocity data—a measurement not yet utilized for gravity estimation—we found a significant opportunity to improve gravity estimation accuracy substantially. GaRLIO, the proposed gravity-enhanced Radar-LiDAR-Inertial Odometry, can robustly predict gravity to reduce vertical drift while simultaneously enhancing state estimation performance using pointwise velocity measurements. Furthermore, GaRLIO ensures robustness in dynamic environments by utilizing radar to remove dynamic objects from LiDAR point clouds. Our method is validated through experiments in various environments prone to vertical drift, demonstrating superior performance compared to traditional LiDAR-Inertial Odometry methods. We make our source code publicly available to encourage further research and development.
</details>

## Prerequisites

* Livox_ros_driver : To install livox_ros_driver, please follow the [Livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver) guide. 

## Dataset
I tested **GaRLIO** on two dataset [NTU4DRadLM](https://github.com/junzhang2016/NTU4DRadLM) and [Snail-Radar](https://snail-radar.github.io/).

## Build
The code is tested on:

* Linux 20.04 LTS
* ROS Noetic
* PCL version 1.10.0
* Eigen version 3.1.0

To download and compile the package, use the following commands:

```bash
cd ~/catkin_ws/src
git clone https://github.com/ChiyunNoh/GaRLIO
cd ..
catkin build
```

## Launch
```bash
roslaunch garlio xxx.launch
rosbag play xxx.bag 
```

## Acknowledgments
Thanks for [Inv-LIO](https://ieeexplore.ieee.org/document/10013761), [FAST-LIO](https://github.com/hku-mars/FAST_LIO), [LINS](https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM), [SR-LIO](https://github.com/ZikangYuan/sr_lio), [REVE](https://github.com/christopherdoer/reve) and [VINS-MONO](https://github.com/HKUST-Aerial-Robotics/VINS-Mono). Furtheremore, a very big thank you to [Pengcheng Shi](https://github.com/spc2) for his great help in developing the algorithm.



## Citation

```
@INPROCEEDINGS { cynoh-2025-icra,
    AUTHOR = { Chiyun Noh and Wooseong Yang and Minwoo Jung and Sangwoo Jung and Ayoung Kim },
    TITLE = { GaRLIO: Gravity enhanced Radar-LiDAR-Inertial Odometry },
    BOOKTITLE = { Proceedings of the IEEE International Conference on Robotics and Automation (ICRA) },
    YEAR = { 2025 },
    MONTH = { May. },
    ADDRESS = { Atlanta },
}
```

## Contact
If you have any questions, please contact:

- Chiyun Noh {[gch06208@snu.ac.kr]()}
