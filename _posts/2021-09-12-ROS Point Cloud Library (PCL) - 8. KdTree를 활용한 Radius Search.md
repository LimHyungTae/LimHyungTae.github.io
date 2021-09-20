---
layout: post
title: ROS Point Cloud Library (PCL) - 8. KdTree를 활용한 Radius Search
subtitle: KdTree FLANN 기반 Radius Search
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true

---

# Search in Pointcloud

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

![img](/img/sor.png)

위의 그림처럼, 왼쪽(빨강)에서의 noise 부분들이 상당히 제거됩니다. 특히, 이 LiDAR pointcloud는 실제 판교 백화점에서 측정된 data인데, 환경적 특징으로 인해 난반사 등이 발생하여 상당한 noise가 껴있어도 SOR을 통해 noise를 제거할 수 있다는 것을 확인할 수 있습니다.

그로 인해 오른쪽(초록)에서 dense하게 측정된 부분만 남게 됩니다.


---

Point Cloud Library Tutorial 시리즈입니다.

0. [ROS Point Cloud Library (PCL) - 0. Tutorial 및 기본 사용법](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-0.-Tutorial-%EB%B0%8F-%EA%B8%B0%EB%B3%B8-%EC%82%AC%EC%9A%A9%EB%B2%95/)

1. [ROS Point Cloud Library (PCL) - 1. Ptr, ConstPtr의 완벽 이해](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-1.-Ptr,-ConstPtr%EC%9D%98-%EC%99%84%EB%B2%BD-%EC%9D%B4%ED%95%B4/)

2. [ROS Point Cloud Library (PCL) - 2. 형변환 - toROSMsg, fromROSMsg](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-2.-%ED%98%95%EB%B3%80%ED%99%98-toROSMsg,-fromROSMsg/)

3. [ROS Point Cloud Library (PCL) - 3. Transformation](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-3.-Transformation/)

4. [ROS Point Cloud Library (PCL) - 4. Voxelization](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-4.-Voxelization/)

5. [ROS Point Cloud Library (PCL) - 5. PassThrough](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-5.-PassThrough/)

6. **ROS Point Cloud Library (PCL) - 6. Statistical Outlier Removal**
