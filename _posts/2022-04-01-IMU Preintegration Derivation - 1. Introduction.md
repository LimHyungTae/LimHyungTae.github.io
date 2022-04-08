---
layout: post
title: IMU Preintegration Derivation - 1. Introduction
subtitle: Why is the IMU Preintegration important
tags: [SLAM, LiDAR, Pointcloud, ROS, IMU, Preintegration]
comments: true

---

# IMU Preintegration 


VINS (Visual-Inertial System)이나 LIO (LiDAR-Inertial Odometry)에 관심이 많은 분이라면 *IMU preintegration*에 대해서 한 번쯤은 들어보았을 겁니다. IMU Preintegration는 *Hz가 매우 빠른 IMU data를 어떻게 camera나 LiDAR sensor로부터 취득한 data와 함께 효율적으로 factor graph SLAM을 할 수 있을까*하는 문제를 풀기 위해서 고안된 방법이라고 할 수 있습니다. 

## IMU Preintegration가 중요한 이유

IMU sensor는 주로 Hz가 높은데 (e.g. LIO-SAM 저자가 사용하는 IMU는 약 330 Hz로 data 측정), 이는 continuous-time system인 실제 우리 주변의 환경에서의 움직임을 discrete-time system 상에서 잘 묘사하기 위함이다. 하지만 그 결과, graph SLAM 상에서 IMU 관련 factor의 수가 늘어나게 된다. 

[여기](https://limhyungtae.github.io/2022-03-27-LeGO-LOAM-Line-by-Line-3.-FeatureAssociation-(3)/)에서 간략히 non-linear equation을 optimization하는 방법을 소개했듯이, SLAM에서도 $\mathcal{X}$라는 state가 주어졌을 때 $\mathbf{J} \Delta=-\epsilon$ 꼴의 식을 통해 반복적으로 optimal한 $\Delta$를 구해 inital 값을 지닌 state $\mathcal{X}$를 업데이트해나가는 과정이라고 볼 수 있다. 여기서 문제는, IMU에 대한 모든 상대적 pose 또한 graph SLAM의 parameter로 여기게 되면 $\mathbf{J}$와 $\mathcal{X}$의 행렬의 크기가 굉장히 커지게 된다는 것이다. 

따라서 이러한 문제를 해결하기 위해 IMU preintegration을 도입한 것이다.

![](/img/preintegration/overview.png)


## Objective

이번 series에서는 LIO-SAM Line by Line 글을 작성하기 이전에 IMU Preintegration에 대해 먼저 자세히 알아보고자 이 글을 작성하게 되었습니다. 그래서 논문 상의 수식 전개를 어떻게 해서 preintegration이 가능하게 된 건지에 대해 equation-by-equation으로 설명하려 합니다. 그리고 최종적으로는 LIO-SAM 상의 `imuPreintegration.cpp`에서 코드가 어떻게 작성되어 있는지 간략히 설명하고자 합니다.

목표는 왜 preintegration이 가능한지에 대한 이해를 돕는 것까지입니다. Jacobian을 계산해서 optimization하는 것은 이미 `gtsam::PreintegratedCombinedMeasurements` class로 잘 구현되어 있기 때문입니다 ([여기](https://gtsam.org/doxygen/4.0.0/a03435.html) 참조)

**P.S.** 글을 작성하는 데에 preintegration 고수인 [유병호님](https://urobot.kaist.ac.kr/url_teams/byeonghoyu/)의 큰 도움이 있었습니다,,,^^7

**P.P.S.** 이 series를 이해하기 위해서는 factor graph optimization과 3D rotation 에 대한 사전적 지식 (e.g., SO(3), skew-symmetric matrix, exponential/logarithm map 등등)이 필요로 합니다. 
* Graph SLAM에 대한 자료는 기섭이가 정리해둔 [SLAM Back-end 공부자료](https://gisbi-kim.github.io/blog/2021/10/03/slam-textbooks.html?fbclid=IwAR0NRvyHQhy6HaIIGMPudq4Mq34b1tQ_l5k3fEijHdk6KECTFsSHDBGhD8E)를 참고하시는 걸 추천드립니다.
* SO(3), skew-symmetric matrix, exponential/logarithm map 등의 용어가 친숙하지 않고 어떤 의미인지 헷갈리시는 분은 Joan Sola의 님의 [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508)의 Chapter 4까지 세 번 정독하는 것을 추천드립니다 '^'.


---

IMU Preintegration Derivation 설명 시리즈입니다.

TBU
---


### 원 논문

* [Foster *et al.*, "IMU Preintegration on Manifold for Efficient
Visual-Inertial Maximum-a-Posteriori Estimation," in *Robotics: Science and Systems (RSS), 2011](http://www.roboticsproceedings.org/rss11/p06.pdf).
* [Foster *et al.*, "On-Manifold Preintegration for Real-Time
Visual-Inertial Odometry," *Transaction on Robotics*, 2016](https://rpg.ifi.uzh.ch/docs/TRO16_forster.pdf)