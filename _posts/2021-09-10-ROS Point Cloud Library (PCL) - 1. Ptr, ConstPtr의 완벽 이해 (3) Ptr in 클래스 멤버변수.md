---
layout: post
title: ROS Point Cloud Library (PCL) - 1. Ptr, ConstPtr의 완벽 이해 (3) Ptr in 클래스 멤버변수
subtitle: PCL의 Ptr를 Class member 변수로 쓸 때 주의점
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true
---


## PCL Ptr in Class

이번 시간에는 마지막으로, Class 내에서 멤버 변수로 사용할 때의 주의점에 대해서 말씀 드리고, Ptr에 대한 얘기를 끝내겠습니다.

<script src="https://gist.github.com/LimHyungTae/4b5a1085162b0851a9429b017ab9b7ef.js"></script>


---

Point Cloud Library Tutorial 시리즈입니다.

0. [ROS Point Cloud Library (PCL) - 0. Tutorial 및 기본 사용법](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-0.-Tutorial-%EB%B0%8F-%EA%B8%B0%EB%B3%B8-%EC%82%AC%EC%9A%A9%EB%B2%95/)

1. **ROS Point Cloud Library (PCL) - 1. Ptr, ConstPtr의 완벽 이해**

2. [ROS Point Cloud Library (PCL) - 2. 형변환 - toROSMsg, fromROSMsg](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-2.-%ED%98%95%EB%B3%80%ED%99%98-toROSMsg,-fromROSMsg/)

3. [ROS Point Cloud Library (PCL) - 3. Transformation](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-3.-Transformation/)

4. [ROS Point Cloud Library (PCL) - 4. Voxelization](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-4.-Voxelization/)

5. [ROS Point Cloud Library (PCL) - 5. PassThrough](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-5.-PassThrough/)

6. [ROS Point Cloud Library (PCL) - 6. Statistical Outlier Removal](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-6.-Statistical-Outlier-Removal/)
