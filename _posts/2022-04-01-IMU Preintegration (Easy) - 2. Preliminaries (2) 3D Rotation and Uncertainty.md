---
layout: post
title: IMU Preintegration (Easy) - 2. Preliminaries (2) 3D Rotation and Uncertainty
subtitle: Why is the IMU Preintegration important
tags: [SLAM, LIO, VIO, IMU, Preintegration]
comments: true

---

# Preliminaries of IMU Preintegration 

원 논문의 이름이 "On-Manifold Preintegration for Real-Time
Visual-Inertial Odometry"인만큼, 사실 
먼저 preintegration을 이해하기 위해서는 manifold의 이해를 필요로 한다.

특히 3

![](/img/rotation/overview.png)


---

IMU Preintegration Derivation 설명 시리즈입니다.

TBU
---


### 원 논문

* [Foster *et al.*, "IMU Preintegration on Manifold for Efficient
Visual-Inertial Maximum-a-Posteriori Estimation," in *Robotics: Science and Systems (RSS), 2011](http://www.roboticsproceedings.org/rss11/p06.pdf).
* [Foster *et al.*, "On-Manifold Preintegration for Real-Time
Visual-Inertial Odometry," *Transaction on Robotics*, 2016](https://rpg.ifi.uzh.ch/docs/TRO16_forster.pdf)