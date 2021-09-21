---
layout: post
title: ROS Point Cloud Library (PCL) - 11. Iterative Closest Point
subtitle: Hello ICP
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true

---

# Iterative Closest Point (ICP)

SLAM 분야에서 흔히 registration == Iterative Closest Point (ICP)라고 취급할 만큼, local registrtaion 중에서 가장 유명한 알고리즘입니다. 

ICP는 한 번 loop마다 두 개의 step으로 구성되어 있습니다. 그리고 registration에서는 통상적으로 주어진 두 point cloud를 source와 target으로 부르는데, 이 source가 target에 다가가는 transformation matrix를 구해주는 함수입니다.

* 1. 현재 주어진 source 각 point 별 가장 가까운 target point를 찾는다.  
* 2. 그 포인트 쌍들을 기반으로 transformation matrix를 incremental하게 추정한다.

이 1->2->1->2...하는 식으로 반복하여 optimization이 됩니다.


# How to Use

<script src="https://gist.github.com/LimHyungTae/639e39853fe465ffe941417821cc87e0.js"></script>

# 결과


**헉,** 근2데 움직이지 않으시는 것을 확인할 수 있습니다.

이는 
---

Point Cloud Library Tutorial 시리즈입니다.

