---
layout: post
title: IMU Preintegration (Easy) - 4. Derivation of Preintegrated IMU Measurements
subtitle: Why is the IMU Preintegration important
tags: [SLAM, LIO, VIO, IMU, Preintegration]
comments: true

---

# Derivation of Preintegrated IMU Measurements



## Preintegrated IMU Measurements

![](/img/preintegration/preinteg_rot.png)

![](/img/preintegration/preinteg_vel.png)

![](/img/preintegration/preinteg_pos.png)

![](/img/preintegration/rel_motion_increments.png)




---

IMU Preintegration Derivation 설명 시리즈입니다.

TBU
---


### 원 논문

* [Foster *et al.*, "IMU Preintegration on Manifold for Efficient
Visual-Inertial Maximum-a-Posteriori Estimation," in *Robotics: Science and Systems (RSS), 2011](http://www.roboticsproceedings.org/rss11/p06.pdf).
* [Foster *et al.*, "On-Manifold Preintegration for Real-Time
Visual-Inertial Odometry," *Transaction on Robotics*, 2016](https://rpg.ifi.uzh.ch/docs/TRO16_forster.pdf)