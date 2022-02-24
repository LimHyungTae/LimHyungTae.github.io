---
layout: post
title: ROS Point Cloud Library (PCL) - 4. Viewer로 visualization하는 법
subtitle: PCL Transformation
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true
---

# Debugging의 필수, visualization

오늘은 pcl pointcloud를 visualization하는 방법에 대해 알아보겠습니다.

대부분 ROS의 RViz로 visualization을 하실텐데, 간단히 몇개의 포인트를 볼 때는 저의 개인적인 소견으로는 pcl viewer가 훨씬 간편한 것 같습니다.

방법은 크게 `PCLVisualizer`를 사용하는 법과 `CloudViewer`룰 사용하는 법으로 크게 나뉘어져 있습니다.

<script src="https://gist.github.com/LimHyungTae/193f9662d4d11f9117acc0a81a16eeb0.js"></script>

실행시키면 다음과 같이 앞으로 5m transformation된 pointcloud와 raw한 데이터가 함께 visualization되는 것을 볼 수 있습니다.

![pcl_color](/img/pcl_viz.png)


#### 주의점 1

주의하실 점은, 본인이 원하고자 하는 색상으로 visualization을 하고 싶으실 때, `pcl::PointXYZGRB` type으로 색을 입혀주셔야 원하는 색으로 visualization이 됩니다.

(colorize 함수 참고)

```cpp
viewer1.addPointCloud<pcl::PointXYZ>(src, "src_red");
viewer1.addPointCloud<pcl::PointXYZ>(tgt, "tgt_green");
```
만약 이렇게 raw한 PointXYZ를 넣으면 어떻게 될까요?

![pcl_raw](/img/pcl_viz_raw.png)

칼라가 없기 때문에 둘 다 그냥 흰색으로 visualization이 됩니다.

#### 주의점 2

id에는 꼭 각각 다른 이름을 넣어야 합니다! 만약 id가 똑같으면 visualizer가 다음과 같은 경고를 하면서 가장 먼저 그 id를 할당 받은 pointcloud만 visualization을 해줍니다.

만약 실수로 아래와 같이 썼으면 어떻게 될까요?

```cpp
viewer1.addPointCloud<pcl::PointXYZRGB>(src_colored, "src_red");
viewer1.addPointCloud<pcl::PointXYZRGB>(tgt_colored, "src_red");
```

> [addPointCloud] The id <src_red> already exists! Please choose a different id and retry.

라는 warning message가 뜨면서 빨간 `src_colored` 밖에 viz가 되지 않는 것을 볼 수 있습니다.

---

Point Cloud Library Tutorial 시리즈입니다.

모든 코드는 이 [**레포지토리**](https://github.com/LimHyungTae/pcl_tutorial)에 있고, ros package로 구성되어 있어 build하여 직접 돌려보실 수 있습니다

0. [ROS Point Cloud Library (PCL) - 0. Tutorial 및 기본 사용법](https://limhyungtae.github.io/2021-09-09-ROS-Point-Cloud-Library-(PCL)-0.-Tutorial-%EB%B0%8F-%EA%B8%B0%EB%B3%B8-%EC%82%AC%EC%9A%A9%EB%B2%95/)
1. [ROS Point Cloud Library (PCL) - 1. Ptr, ConstPtr의 완벽 이해 (1) shared_ptr](https://limhyungtae.github.io/2021-09-09-ROS-Point-Cloud-Library-(PCL)-1.-Ptr,-ConstPtr%EC%9D%98-%EC%99%84%EB%B2%BD-%EC%9D%B4%ED%95%B4-(1)-shared_ptr/)
2. [ROS Point Cloud Library (PCL) - 1. Ptr, ConstPtr의 완벽 이해 (2) Ptr in PCL](https://limhyungtae.github.io/2021-09-10-ROS-Point-Cloud-Library-(PCL)-1.-Ptr,-ConstPtr%EC%9D%98-%EC%99%84%EB%B2%BD-%EC%9D%B4%ED%95%B4-(2)-Ptr-in-PCL/)
3. [ROS Point Cloud Library (PCL) - 1. Ptr, ConstPtr의 완벽 이해 (3) Ptr in 클래스 멤버변수](https://limhyungtae.github.io/2021-09-10-ROS-Point-Cloud-Library-(PCL)-1.-Ptr,-ConstPtr%EC%9D%98-%EC%99%84%EB%B2%BD-%EC%9D%B4%ED%95%B4-(3)-Ptr-in-%ED%81%B4%EB%9E%98%EC%8A%A4-%EB%A9%A4%EB%B2%84%EB%B3%80%EC%88%98/)
4. [ROS Point Cloud Library (PCL) - 2. 형변환 - toROSMsg, fromROSMsg](https://limhyungtae.github.io/2021-09-10-ROS-Point-Cloud-Library-(PCL)-2.-%ED%98%95%EB%B3%80%ED%99%98-toROSMsg,-fromROSMsg/)
5. [ROS Point Cloud Library (PCL) - 3. Transformation](https://limhyungtae.github.io/2021-09-10-ROS-Point-Cloud-Library-(PCL)-3.-Transformation/)
6. [ROS Point Cloud Library (PCL) - 4. Viewer로 visualization하는 법](https://limhyungtae.github.io/2021-09-10-ROS-Point-Cloud-Library-(PCL)-4.-Viewer%EB%A1%9C-visualization%ED%95%98%EB%8A%94-%EB%B2%95/)
7. [ROS Point Cloud Library (PCL) - 5. Voxelization](https://limhyungtae.github.io/2021-09-12-ROS-Point-Cloud-Library-(PCL)-5.-Voxelization/)
8. [ROS Point Cloud Library (PCL) - 6. PassThrough](https://limhyungtae.github.io/2021-09-12-ROS-Point-Cloud-Library-(PCL)-6.-PassThrough/)
9. [ROS Point Cloud Library (PCL) - 7. Statistical Outlier Removal](https://limhyungtae.github.io/2021-09-12-ROS-Point-Cloud-Library-(PCL)-7.-Statistical-Outlier-Removal/)
10. [ROS Point Cloud Library (PCL) - 8. KdTree를 활용한 Radius Search](https://limhyungtae.github.io/2021-09-12-ROS-Point-Cloud-Library-(PCL)-8.-KdTree%EB%A5%BC-%ED%99%9C%EC%9A%A9%ED%95%9C-Radius-Search/)
11. [ROS Point Cloud Library (PCL) - 9. KdTree를 활용한 K-nearest Neighbor Search (KNN)](https://limhyungtae.github.io/2021-09-12-ROS-Point-Cloud-Library-(PCL)-9.-KdTree%EB%A5%BC-%ED%99%9C%EC%9A%A9%ED%95%9C-K-nearest-Neighbor-Search-(KNN)/)
12. [ROS Point Cloud Library (PCL) - 10. Normal Estimation](https://limhyungtae.github.io/2021-09-13-ROS-Point-Cloud-Library-(PCL)-10.-Normal-Estimation/)
13. [ROS Point Cloud Library (PCL) - 11. Iterative Closest Point (ICP)](https://limhyungtae.github.io/2021-09-14-ROS-Point-Cloud-Library-(PCL)-11.-Iterative-Closest-Point-(ICP)/)
14. [ROS Point Cloud Library (PCL) - 12. Generalized Iterative Closest Point (G-ICP)](https://limhyungtae.github.io/2021-09-14-ROS-Point-Cloud-Library-(PCL)-12.-Generalized-Iterative-Closest-Point-(G-ICP)/)


