---
layout: post
title: ROS Point Cloud Library (PCL) - 9. KdTree를 활용한 K-nearest Neighbor Search (KNN)
subtitle: K-nearest Neighbor Search
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true

---

# Normal Estimation

![img_kd](/img/3dtree.png)

오늘은 pointcloud 상에서 기준 point(query point)를 기준 search 하는 법 중 두 번째 방법인 k-nearest Neighbor search에 대해 알아보겠습니다.

`radiusSearch()`는 pointcloud 상 주어진 query point의 거리 r 안에 있는 모든 point들의 index를 리턴해주는 함수라면, 

`nearestKSearch()`는 query point로부터 가장 가까운 K개의 point의 index를 리턴해주는 함수입니다.

주로 KNN은 point cloud 상의 normal vector를 추출할 때 많이 사용됩니다.

---

# How to use

아래는 차량 근처에 query point `query`를 주었을 때, 가장 가까운 1,200개의 pointcloud를 추출하는 예제입니다. (50번 째 줄 부터)

<script src="https://gist.github.com/LimHyungTae/8ebd587dd5fdbdbd54706150b7da437e.js"></script>

![img](/img/kdtree_knn_v2.png)

위의 그림처럼, input cloud가 주어지고(빨강) 차량 근처의 query point를 주었을 때(파랑) 그 근처의 1,200개의 cloud points(초록)이 추출되는 것을 볼 수 있습니다.

마찬가지로, radius search때와 같이 KdTree는 `setInputCloud()`할 때 내부적으로 input pointcloud를 트리화하여 저장하기 때문에, time cost가 다소 존재합니다.

따라서 자신이 적용할 task가 KdTree를 사용하기 전에 inputcloud를 한 번 등록해두고 계속 인접한 k개의 point를 뽑아야 하는 task인가를 검토하는 것이 중요합니다.

---

Point Cloud Library Tutorial 시리즈입니다.

모든 코드는 이 [**레포지토리**](https://github.com/LimHyungTae/pcl_tutorial)에 있고, ros package로 구성되어 있어 build하여 직접 돌려보실 수 있습니다

{% include post_links_pcl.html %}

