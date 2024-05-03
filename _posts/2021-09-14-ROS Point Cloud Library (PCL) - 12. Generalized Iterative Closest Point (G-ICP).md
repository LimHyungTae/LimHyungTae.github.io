---
layout: post
title: ROS Point Cloud Library (PCL) - 12. Generalized Iterative Closest Point (G-ICP)
subtitle: G-ICP 내부 설명 
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true

---

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

