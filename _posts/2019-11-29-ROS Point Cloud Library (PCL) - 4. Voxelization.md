---
layout: post
title: ROS Point Cloud Library (PCL) - 4. Voxelization
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

Voxelization을 하는 방법은 아래와 같습니다.


<script src="https://gist.github.com/LimHyungTae/1235dcdbe293133079c359f11906be24.js"></script>

---

Point Cloud Library Tutorial 시리즈입니다.

1. [ROS Point Cloud Library (PCL) - 1. Tutorial 및 기본 사용법](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-1.-Tutorial-%EB%B0%8F-%EA%B8%B0%EB%B3%B8-%EC%82%AC%EC%9A%A9%EB%B2%95/)

2. [ROS Point Cloud Library (PCL) - 2. 형변환 - toROSMsg, fromROSMsg](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-2.-%ED%98%95%EB%B3%80%ED%99%98-toROSMsg,-fromROSMsg/)

3. [ROS Point Cloud Library (PCL) - 3. Transformation](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-3.-Transformation/)

4. **ROS Point Cloud Library (PCL) - 4. Voxelization**

5. [ROS Point Cloud Library (PCL) - 5. PassThrough](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-5.-PassThrough/)

6. [ROS Point Cloud Library (PCL) - 6. Statistical Outlier Removal](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-6.-Statistical-Outlier-Removal/)

