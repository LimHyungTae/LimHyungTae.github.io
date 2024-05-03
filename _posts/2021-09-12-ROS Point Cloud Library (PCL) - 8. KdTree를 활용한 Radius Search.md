---
layout: post
title: ROS Point Cloud Library (PCL) - 8. KdTree를 활용한 Radius Search
subtitle: KdTree FLANN 기반 Radius Search
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true

---

# Search in Pointcloud

![img_kd](/img/3dtree.png)

pointcloud 상에서 기준 point(query point)를 기준으로 search 하는 법은 크게

* Radius search
* k-nearest Neighbor search

가 있습니다. 먼저, Radius search는 pointcloud 상 주어진 query point의 거리 r 안에 있는 모든 point들의 index를 리턴해주는 함수입니다. 활용처는 크게 두가지가 있는데,

* 여기처럼 모바일 플랫폼의 sensor frame 주변의 noise의 index를 추출해서 제거할 때 (하지만, 엄밀히 따지면 연산적으로 비효율적임)
* 전체 map 상에서 global한 좌표를 기점으로 인근의 submap을 추출할 때 (예시는 [여기](https://github.com/LimHyungTae/ERASOR))

등입니다.

---

# How to use

아래는 두 query points `query1`과 `query2`에 대해 8m 이내로 인접해 있는 pointcloud를 추출하는 예제입니다.(52번 째 줄 부터)

<script src="https://gist.github.com/LimHyungTae/a0f16eb19b90899e6f7012eee257130c.js"></script>

![img](/img/kdtree_radius.png)

위의 그림처럼, input cloud(빨강)가 주어졌을 때 
* `query1`의 위치인 (0, 0, 0) 이내의 8m 영역(초록색)
* `query2`의 위치인 (20, 0, 0) 이내의 8m 영역(파란색)

을 추출해낼 수 있습니다.

주의하실 점은 KdTree는 `setInputCloud()`할 때 내부적으로 input pointcloud를 트리화하여 저장하기 때문에, time cost가 다소 존재한다는 것입니다. 하지만 한번 등록해두면 `radiusSearch()`를 통해 찾을 때는 단순히 for문을 돌리는 것보다는 훨씬 빠릅니다.

따라서 자신이 적용할 task가 KdTree를 사용하기 전에 inputcloud를 한 번 등록해두고 계속 search를 해야할 task인가를 검토하는 것이 중요합니다.

---

Point Cloud Library Tutorial 시리즈입니다.

모든 코드는 이 [**레포지토리**](https://github.com/LimHyungTae/pcl_tutorial)에 있고, ros package로 구성되어 있어 build하여 직접 돌려보실 수 있습니다

{% include post_links_pcl.html %}
