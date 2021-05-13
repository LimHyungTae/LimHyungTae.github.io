---
layout: post
title: ROS Point Cloud Library (PCL) - 7. Normal Estimation
subtitle: Estimation of normal vectors
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true

---

# Normal Estimation

`pcl::NormalEstimation`은 말그대로 normal vector를 estimation해주는 함수입니다.

예시는 [여기](https://pointclouds.org/documentation/tutorials/normal_estimation.html)를 보시면 확인하실 수 있습니다.
---

# 궁금증

Normal Estimation은 인접한 point들과 해당 point 간의 관계를 구하는 것이기 때문에 알고리즘을 사용할 때 SearchMethod을 지정해주어야 합니다.

그리고 `setRadiusSearch(double radius)` 멤버함수를 통해서 radius를 지정해주는데, 그러면 만약 radius 이내에 인접한 point가 없을 때는 어떻게 되는지 궁금해서 테스트 해보았습니다.

아래 코드가 그 예시입니다.

<script src="https://gist.github.com/LimHyungTae/90cbbdd87727ee8bd3cb795005b5474f.js"></script>

```
(0,0,1 - 0)
(0,0,1 - 0)
(0,0,1 - 0)
(0,0,1 - 0)
(0,0,1 - 0)
(nan,nan,nan - nan)
```

`nan`으로 뜨는 것을 확인할 수 있습니다.

---

Point Cloud Library Tutorial 시리즈입니다.

0. [ROS Point Cloud Library (PCL) - 0. Tutorial 및 기본 사용법](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-0.-Tutorial-%EB%B0%8F-%EA%B8%B0%EB%B3%B8-%EC%82%AC%EC%9A%A9%EB%B2%95/)

1. [ROS Point Cloud Library (PCL) - 1. Ptr, ConstPtr의 완벽 이해](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-1.-Ptr,-ConstPtr%EC%9D%98-%EC%99%84%EB%B2%BD-%EC%9D%B4%ED%95%B4/)

2. [ROS Point Cloud Library (PCL) - 2. 형변환 - toROSMsg, fromROSMsg](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-2.-%ED%98%95%EB%B3%80%ED%99%98-toROSMsg,-fromROSMsg/)

3. [ROS Point Cloud Library (PCL) - 3. Transformation](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-3.-Transformation/)

4. [ROS Point Cloud Library (PCL) - 4. Voxelization](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-4.-Voxelization/)

5. [ROS Point Cloud Library (PCL) - 5. PassThrough](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-5.-PassThrough/)

6. **ROS Point Cloud Library (PCL) - 6. Statistical Outlier Removal**

7. **ROS Point Cloud Library (PCL) - 7. Normal Estimation**
