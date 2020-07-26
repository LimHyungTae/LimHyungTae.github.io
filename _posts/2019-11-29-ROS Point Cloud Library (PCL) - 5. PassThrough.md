---
layout: post
title: ROS Point Cloud Library (PCL) - 5. PassThrough
subtitle: 축을 기준으로하는 pointcloud filtering
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true
---

# PassThrough의 활용 예시

PassThrough 함수는 말 그래도 range 기반으로 filtering을 해주는 함수입니다. PassThrough는 주로 pointcloud를 받은 후 의도치 않게 측정이 되는 부분들을 제거하기 위해 사용됩니다 (i.e. 로봇에 부착했는데 센서의 측정 영역에 로봇의 몸체 부분이 있어서 불필요한 point가 찍힌다던가 등등). 사용 순서는 아래와 같습니다.

1. Filtering하고자 하는 축을 `setFilterFieldName` 함수를 통해 지정해 줌 
2. 원하는 범위를 `setFilterLimits` 함수를 통해 지정해 줌
3. (Option) 원하는 범위의 내의 point를 통과 시킬 것인지 (`setFilterLimitsNegative(false)`), 원하는 범위 외의 point를 통과시킬 것인지 결정(`setFilterLimitsNegative(true)`)

# How to use PassThrough Filter

<script src="https://gist.github.com/LimHyungTae/e64164994be190b6a3638f6b770f9485.js"></script>

<script src="https://gist.github.com/LimHyungTae/aa538935ec8a5c8a482a8eb3002b6407.js"></script>

