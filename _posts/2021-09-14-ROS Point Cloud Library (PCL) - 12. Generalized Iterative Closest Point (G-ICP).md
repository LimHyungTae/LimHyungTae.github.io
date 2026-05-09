---
layout: post
title: ROS Point Cloud Library (PCL) - 12. Generalized Iterative Closest Point (G-ICP)
subtitle: G-ICP 내부 설명 
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true
description: PCL Generalized ICP가 point-to-point ICP와 달리 source/target의 normal로 추정한 uncertainty를 활용해 정합 정확도를 높이는 원리와, setInputSource/Target 내부에서 일어나는 일을 정리한다.
permalink: /2021/09/14/pcl-tutorial-12-generalized-icp/
redirect_from:
  - '/2021-09-13-ROS Point Cloud Library (PCL) - 12. Generalized Iterative Closest Point (G-ICP)/'
  - '/2021-09-13-ROS-Point-Cloud-Library-(PCL)-12.-Generalized-Iterative-Closest-Point-(G-ICP)/'
  - '/2021-09-14-ROS-Point-Cloud-Library-(PCL)-12.-Generalized-Iterative-Closest-Point-(G-ICP)/'
---

<div style="background:#f0f7ff; border-left:3px solid #1B2C8F; padding:10px 16px; margin:16px 0 28px; border-radius:4px; font-size:14px;">
🚀 <strong>Interactive Demo (26.May.07)</strong>: 본 포스트의 PCL 코드들을 브라우저에서 바로 실행해볼 수 있는 <a href="https://limhyungtae.github.io/pcl_tutorial/#/" target="_blank">interactive demo 사이트</a>를 추가했습니다.
</div>

# Generalized Iterative Closest Point (G-ICP)

이번에는 tutorial의 마지막인, G-ICP에 대해서 알아보겠습니다.

기존의 ICP는 내부적으로 point-to-point로 상대적 거리를 줄여나갔다면 G-ICP에서는 바로 해당 point의 source/target point의 uncertainty를 활용하여 registration을 하는 기법입니다.

따라서 이러한 uncertainty를 추정하기 위해 source/target point cloud에 대해 normal vector를 추출합니다.

하지만 아래 코드에서 보이는 것처럼 `setInputSource()`, `setInputTarget()` 내부에서 normal vector를 추출하기 때문에, 사용자 관점에서는 아래와 같이 

```cpp
gicp.setInputSource(src);
gicp.setInputTarget(tgt);
gicp.align(*align);
```

를 하게 되면 gicp class 내부에서 우리가 구하고자 하는 source와 target의 transformation matrix를 구하게 됩니다.

# How to Use

<script src="https://gist.github.com/LimHyungTae/4e0738316126fbe2d5cbe6cded1c2f2a.js"></script>


G-ICP 코드는 사실 ICP를 상속 받아서 구현되어 있기 때문에, ICP를 사용할 때와 별반 다를바 없습니다.

끗!

---

Point Cloud Library Tutorial 시리즈입니다.

모든 코드는 이 [**레포지토리**](https://github.com/LimHyungTae/pcl_tutorial)에 있고, ros package로 구성되어 있어 build하여 직접 돌려보실 수 있습니다


{% include post_links_pcl.html %}

