---
layout: post
title: IMU Preintegration Derivation - 2. Preliminaries
subtitle: Why is the IMU Preintegration important
tags: [SLAM, LiDAR, Pointcloud, ROS, IMU, Preintegration]
comments: true

---

# Preliminaries of IMU Preintegration (작성 중)

먼저 preintegration을 



## Keyframe 

## Quaternion vs SO(3)

![](/img/preintegration/dt_equation.png)

![](/img/preintegration/physical_meaning.png)

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