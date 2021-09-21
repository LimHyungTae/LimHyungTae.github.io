---
layout: post
title: ROS Point Cloud Library (PCL) - 11. Iterative Closest Point (ICP)
subtitle: Low-level Normal Estimation of PointCloud 
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true

---

# Iterative Closest Point (ICP)
이번에는 pointcloud에서의 normal vector를 추출하는 방법에 대해 알아보겠습니다.

Normal vector를 추출하는 일은 pointcloud의 geometry 정보를 활용할 때 다방면으로 활용됩니다.

큰 예제로는 G-ICP에서 각 point마다의 uncertainty(covariance)를 추정할 때, 혹은 벽면이나 바닥면을 추출할 때 등이 있습니다.  

PCL 상에 `pcl::NormalEstimation`이라는 end-to-end function이 있으나,

내부가 어떻게 구성돼있는지 알아보기 위해 직접 짜보았습니다.


# How to Use

<script src="https://gist.github.com/LimHyungTae/f93910f855b7b9981485fc2c95916279.js"></script>

Normal vector를 visualization하면 아래와 같습니다.

# 결과

![normal1](/img/normal_vector1.png)

![normal2](/img/normal_vector2.png)

보시는 것과 같이 벽면들, 바닥들의 normal vector가 수직으로 잘 뽑히는 것을 확인할 수 있습니다.

---

Point Cloud Library Tutorial 시리즈입니다.

