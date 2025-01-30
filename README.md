<p align="center">

  <h1 align="center"> 🌏 GaRLIO: Gravity enhanced Radar-LiDAR-Inertial Odometry [ICRA 2025]</h1>

  <p align="center">
    <a href="https://github.com/ChiyunNoh/GaRLIO"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/ChiyunNoh/GaRLIO"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://github.com/ChiyunNoh/GaRLIO"><img src="https://img.shields.io/badge/Paper-pdf-<COLOR>.svg?style=flat-square" /></a>
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

  <h3 align="center"><a href="">Paper</a> | <a href="https://youtu.be/zeH3RQdIviw?si=aZg_WZfn4ErqkNu8">Video</a></h3>
  <div align="center"></div>
</p>


This repository contains the code for **GaRLIO: Gravity enhanced Radar-LiDAR-Inertial Odometry**, which is accepted by ICRA 2025. 

Full paper and code will be available in a few weeks. Stay tuned!

<!-- TABLE OF CONTENTS -->
<details open="open" style='padding: 10px; border-radius:5px 30px 30px 5px; border-style: solid; border-width: 1px;'>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#abstract">Abstract</a>
    </li>
    <li>
      <a href="#installation">Installation</a>
    </li>
    <li>
      <a href="#citation">Citation</a>
    </li>
    <li>
      <a href="#contact">Contact</a>
    </li>
  </ol>
</details>


## Abstract

<details>
  <summary>click to expand</summary>
Recently, gravity has been highlighted as a crucial constraint for state estimation to alleviate potential vertical drift. 
Existing online gravity estimation methods rely on pose estimation combined with IMU measurements, which is considered best practice when direct velocity measurements are unavailable. However, with radar sensors providing direct velocity data—a measurement not yet utilized for gravity estimation—we found a significant opportunity to improve gravity estimation accuracy substantially. GaRLIO, the proposed gravity-enhanced Radar-LiDAR-Inertial Odometry, can robustly predict gravity to reduce vertical drift while simultaneously enhancing state estimation performance using pointwise velocity measurements. Furthermore, GaRLIO ensures robustness in dynamic environments by utilizing radar to remove dynamic objects from LiDAR point clouds. Our method is validated through experiments in various environments prone to vertical drift, demonstrating superior performance compared to traditional LiDAR-Inertial Odometry methods. We make our source code publicly available to encourage further research and development.
</details>



## Installation


## Citation



```

```

## Contact
If you have any questions, please contact:

- Chiyun Noh {[gch06208@snu.ac.kr]()}
