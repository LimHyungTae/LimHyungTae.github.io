---
layout: post
title: ROS Point Cloud Library (PCL) - 11. Iterative Closest Point
subtitle: Hello ICP
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true

---

# Iterative Closest Point (ICP)

SLAM 분야에서 흔히 registration == Iterative Closest Point (ICP)라고 일반적으로 취급할 만큼, ICP는 local registrtaion 중에서 가장 유명한 알고리즘입니다. 

ICP는 말 그대로 *iterative하게 가장 가까운 point를 찾는것을 반복*하면서 optimization이 시행되는데, 한 iteration 마다 두 개의 step으로 구성되어 있습니다. Registration에서는 통상적으로 주어진 두 point cloud pair를 각각 *source*와 *target*으로 부르는데, 이 source가 target에 다가가는 transformation matrix를 구해주는 함수입니다.

앞서 말한 두 step은 아래와 같습니다.

* 1. 현재 주어진 source 각 point 별 가장 가까운 target point를 찾는다 (knn에서 k=1로 해서 가장 가까운 것을 찾음).  
* 2. 그 포인트 쌍들을 기반으로 transformation matrix를 incremental하게 추정한다.

이 1->2->1->2...하는 식으로 반복하여 더 이상 transformation matrix의 변화(epsilon)가 0에 근접하면 optimization을 종료합니다.


# How to Use (77번 째 줄부터)

<script src="https://gist.github.com/LimHyungTae/639e39853fe465ffe941417821cc87e0.js"></script>

### 파라미터 세팅

ICP를 할 때 크게 주의해야할 파라미터 세팅은 아래와 같이 세개가 있습니다.

* `setMaxCorrespondenceDistance()`
* `setTransformationEpsilon()`
* `setMaximumIterations()`

### 결과값 return 받기

`getFinalTransformation()`
`getFitnessScore()`
`hasConverged()`

# 결과

![img](/img/icp_result.png)

보시는 것처럼, 앞으로 2m transformation을 인위적으로 한것을 따라간 것을 확인할 수 있습니다.

하지만 생각보다 저의 개인적으로는 이런 도심환경이나 실내 환경에서 registration을 할 때는 ICP보다 G-ICP를 사용하는 것이 **훨씬 낫습니다.** (이 부분은 단언코 말씀드릴 수 있습니다)



---

Point Cloud Library Tutorial 시리즈입니다.

