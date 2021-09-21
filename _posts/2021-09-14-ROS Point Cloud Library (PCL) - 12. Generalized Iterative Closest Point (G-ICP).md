---
layout: post
title: ROS Point Cloud Library (PCL) - 12. Generalized Iterative Closest Point (G-ICP)
subtitle: G-ICP 내부 설명 
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true

---

# Generalized Iterative Closest Point (ICP)

이번에는 tutorial의 마지막인, G-ICP에 대해서 알아보겠습니다.

기존의 ICP는 내부적으로 point-to-point로 상대적 거리를 줄여나갔다면 G-ICP에서는 바로 해당 point의 target point의 uncertainty를 활용하여 registration을 하는 기법입니다.



큰 예제로는 G-ICP에서 각 point마다의 uncertainty(covariance)를 추정할 때, 혹은 벽면이나 바닥면을 추출할 때 등이 있습니다.  


# How to Use

<script src="https://gist.github.com/LimHyungTae/4e0738316126fbe2d5cbe6cded1c2f2a.js"></script>

# 결과

![img](/img/gicp_result.png)

---

Point Cloud Library Tutorial 시리즈입니다.

