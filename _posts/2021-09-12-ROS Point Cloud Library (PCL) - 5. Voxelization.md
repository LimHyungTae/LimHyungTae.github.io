---
layout: post
title: ROS Point Cloud Library (PCL) - 5. Voxelization
subtitle: Reduction of Computation
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true
---

# Voxelization이 필요한 이유

![centroid](/img/pcl_centroid.PNG)

Voxelization은 pointcloud를 말그대로 voxel화해주는 기능입니다. 위의 그림처럼 여러 개의 point들이 있을 때 공간을 `Leaf` (혹은 Cell, grid 등 여러 표현이 있으나 PCL에서는 `setLeafSize`를 통해 voxel size를 지정해주기 때문에 편의 상 Leaf이라고 기술하였습니다)의 크기로 나누어서 그 크기 내의 점들의 평균을 내어 pointcloud를 재 기술해줍니다. 따라서 원래 한 `Leaf`에 N개의 point가 있었으면 이 N개의 point를 1개의 point로 나타내주기 때문에 효율적으로 pointcloud를 관리할 수 있습니다. 정리하자면 Voxelization을 하면 아래와 같은 두 효과를 볼 수 있습니다.

* memory-efficient하게 pointcloud를 저장 가능
* Registration, Filtering 등을 할 때 연산 대상(point)의 수가 줄어들기 때문에, 연산 속도가 굉장히 올라감

![ndt](/img/according_to_voxel2.png)

실제로 50x50m 크기의 map과 현재 pointcloud를 Normal Distribution Transform (NDT)로 registration을 할 때 voxel 크기를 어떻게 주냐에 따라서 연산 속도가 어떻게 변하는 지 알 수 있습니다. 하지만 주의하실 점은 ***Voxel size를 적절히 세팅해주어야 한다***는 것입니다. 왜냐하면 voxel size가 크다는 것은 그만큼 map을 간단히 표현하는 것이기 때문에, registration 자체가 부정확해질 수 있기 때문입니다.

# How to Downsample via VoxelGrid header

Voxelization을 하는 방법은 아래와 같습니다. (77번째 줄 부터)


<script src="https://gist.github.com/LimHyungTae/1235dcdbe293133079c359f11906be24.js"></script>

# 결과

결과를 보시면 아래와 같습니다. 왼쪽(빨강)이 raw data이고, 오른쪽(초록) voxelization된 pointcloud입니다.

![voxel](/img/voxel_example.png)

실외에서 3D pointcloud로 측정할 수 있는 거리가 약 80m 정도 되기 때문에, 0.2m로 voxelization을 하면 밖에서 보면 티가 잘 나지 않을 수 있습니다.

하지만 확대해서 살펴보면 그 차이를 확인할 수 있습니다.

![voxel_zoom](/img/voxel_zoom.png)

오른쪽(초록)이 좀 더 소(sparse)해진 것을 확인할 수 있습니다.

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

