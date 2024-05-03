---
layout: post
title: ROS Point Cloud Library (PCL) - 1. Ptr, ConstPtr의 완벽 이해 (2) Ptr in PCL
subtitle: PCL에서 Ptr을 활용해보기
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true
---


## PCL pointer Ptr 예제

이번 시간에는 아래의 예시를 통해 어떻게 쓰면 될지 하나하나 알아가보도록 하겠습니다.

Line-by-line으로 주석을 달아두었으니 한번 살펴보시길 바랍니다 :)

<script src="https://gist.github.com/LimHyungTae/8a1f2259aadd7a7d96aa672259a80788.js"></script>



## 스스로 잘 이해했는지 테스트

Q1. `Ptr`을 new를 통해 선언한 `pcl::PointCloud<pcl::PointXYZ>::Ptr`, `Cloud`를 `pcl::PointCloud<pcl::PointXYZ>`로 선언한 pointcloud라 간략히 표현했을 때, 
* `Ptr` = `Ptr`
* `*Ptr` = `*Ptr`
* `*Ptr` = `Cloud`
* `Cloud` = `*Ptr`

이 각각 어떤 의미인가?

![img_file](/img/pcl_ptr_viz.png)

Q2. 위의 그림을 보고 어떤 상황인지 설명이 가능한가?

의미에 답을 내리지 못하시겠으면, 위의 예제 코드를 좀더 차근차근 읽어보시길 바랍니다. :)


---

Point Cloud Library Tutorial 시리즈입니다.

모든 코드는 이 [**레포지토리**](https://github.com/LimHyungTae/pcl_tutorial)에 있고, ros package로 구성되어 있어 build하여 직접 돌려보실 수 있습니다

{% include post_links_pcl.html %}