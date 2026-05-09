---
layout: post
title: LeGO-LOAM Line by Line - 1. Introduction
subtitle: Preview and Preliminaries
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL, LeGO-LOAM]
comments: true
description: LIO-SAM 초기화 단계에서 사용되는 imuAvailable과 odomAvailable 플래그의 의미를 정리하고, IMU pre-integration 결과가 pose guess에 어떻게 활용되는지 살펴본다.
permalink: /2022/04/07/lio-sam-line-by-line-02-initialization/
  - '/LIO-SAM-Line-by-Line-2.-Initialization/'
redirect_from:
  - '/2022-04-07-LIO-SAM Line by Line - 2. Initialization/'
  - '/2022-04-07-LIO-SAM-Line-by-Line-2.-Initialization/'
---

# LIO-SAM
`imuAvailable`: use imu incremental estimation for pose guess (only rotation)

`odomAvailable`: use imu pre-integration estimation for pose guess

---

참고자료
* [SLAM KR Study: LOAM and LeGO-LOAM](https://www.youtube.com/watch?v=snPzNmcbCCQ&t=564s)
* [LeGO-LOAM-code-review-imageProjection (Chinese)](https://wykxwyc.github.io/2019/01/23/LeGO-LOAM-code-review-imageProjection/)
* [LeGO-LOAM-code-review-featureAssociation (Chinese)](https://wykxwyc.github.io/2019/01/24/LeGO-LOAM-code-review-featureAssociation/)
* [Interpreting LEGO-LOAM Source Code: FeatureAssociation (1) (Chinese)](https://papago.naver.com/?sk=zh-CN&tk=en&st=LEGO-LOAM%E6%BA%90%E7%A0%81%E8%A7%A3%E6%9E%90%20---%20FeatureAssociation%E8%8A%82%E7%82%B9(1))
* [Interpreting LEGO-LOAM Source Code: FeatureAssociation (3) (Chinese)](https://zhuanlan.zhihu.com/p/245603082)
