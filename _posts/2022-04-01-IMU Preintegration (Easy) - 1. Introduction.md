---
layout: post
title: IMU Preintegration (Easy) - 1. Introduction
subtitle: Easy explanations of "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry"
tags: [SLAM, LIO, VIO, IMU, Preintegration]
comments: true

---

# IMU Preintegration for Beginners


VINS (Visual-Inertial System)이나 LIO (LiDAR-Inertial Odometry)에 관심이 많은 분이라면 *IMU preintegration*에 대해서 한 번쯤은 들어보았을 겁니다. IMU Preintegration는 *"Hz가 매우 빠른 IMU data를 어떻게 camera나 LiDAR sensor로부터 취득한 data와 함께 효율적으로 factor graph SLAM을 할 수 있을까"*하는 문제를 풀기 위해서 고안된 방법이라고 할 수 있습니다. 

## IMU Preintegration가 중요한 이유

IMU sensor는 주로 Hz가 높은데 (e.g. LIO-SAM 저자가 사용하는 IMU는 약 330 Hz로 data 측정), 이는 continuous-time system인 실제 우리 주변의 환경에서의 움직임을 discrete-time system 상에서 잘 묘사하기 위해서입니다. 하지만 그 결과, graph SLAM 상에서 모든 IMU measurements로부터 pose를 추정하면 이러한 IMU factor를 표현하는 파라미터의 수가 너무 많아지게 됩니다.

[여기](https://limhyungtae.github.io/2022-03-27-LeGO-LOAM-Line-by-Line-3.-FeatureAssociation-(3)/)에서 간략히 non-linear equation을 optimization하는 방법을 소개했듯이, SLAM에서도 최적의 해를 구할 때 non-linear equation을 iterative하게 풀어나가게 됩니다. 이러한 과정은 행렬 연산을 필요로 하는데, 여기서 문제는 IMU로 추정한 estimated pose들을 모두 다 measurements로 사용하게 되면 추정해야 하는 matrix의 크기가 시간이 지남에 따라 기하급수적으로 커지게 된다는 것입니다.

예로 들어 아래와 같이 25분 정도의 ROS bag 파일을 취득한 상황이 있다고 가정해봅시다. 그러면 IMU sensor (`/gx5/imu/data`)의 Hz가 높다보니, 25분 동안 약 491,848 개의 데이터가 취득되는 것을 확인할 수 있습니다. 
![](/img/preintegration/imu_characteristics_cut2.png)

이러한 상황에서 각 IMU의 time step에 대한 pose를 묘사하기 위해서는 3D translation를 위한 파라미터 3개와 3D rotation을 위한 파라미터 3개로 총 6개가 필요하게 한데, 그러면 총 2,951,088 여개의 parameter가 필요한 상황이 닥칩니다. 심지어 LiDAR sensor (`/os_cloud_node/points`)로 추정한 estimated pose 또한 factor graph에 추가해주어야 하기 때문에, 파라미터는 저 수보다 더 많아지게 됩니다. 요약하자면, LiDAR나 camera의 keyframe의 pose 사이에 수백 개의 IMU factor가 생성되게 되고, 이는 CPU 연산량이나 RAM을 어어어어어어어어어ㅓ어ㅓㅓ엄청 잡아먹기 떄문에 factor graph기반 SLAM의 실시간성을 위협하게 됩니다.

따라서, 이러한 문제를 해결하기 위해 IMU preintegration이 도입되었습니다. 한 마디로 preintegration을 요약하자면, IMU data로 기인하는 모든 수백 여개의 measurments를 factor로 직접적으로 넣는 것이 아니라, factor graph에 measurement를 추가하기 이전에 (pre-) 수백여개의 IMU data를 단 하나의 factor로 취합(integration)하는 방법이라고 말씀드릴 수 있습니다 ([여기](www.naver.com)에서 더 엄밀한 problem definition에 대해 자세히 설명하고 있음).

![](/img/preintegration/overview.png)


## Objective

이번 series에서는 LIO-SAM Line by Line 글을 작성하기 이전에 IMU Preintegration에 대해 먼저 자세히 알아보고자 이 글을 작성하게 되었습니다. 그래서 논문 상의 수식 전개를 어떻게 해서 preintegration이 가능하게 된 건지에 대해 equation-by-equation으로 설명하려 합니다. 그리고 최종적으로는 LIO-SAM 상의 `imuPreintegration.cpp`에서 코드가 어떻게 작성되어 있는지 간략히 설명하고자 합니다.

목표는 왜 preintegration이 가능한지에 대한 이해를 돕는 것까지입니다. Jacobian을 계산해서 optimization하는 것은 이미 `gtsam::PreintegratedCombinedMeasurements` class로 잘 구현되어 있기 때문입니다 ([여기](https://gtsam.org/doxygen/4.0.0/a03435.html) 참조). 개인적으로 **[원 논문](https://rpg.ifi.uzh.ch/docs/TRO16_forster.pdf)과 이 글을 함께 읽으면서 원 논문의 보충자료로서 이 글을 읽는 것을 추천드립니다**.

**P.S.** 글을 작성하는 데에 preintegration 고수인 [유병호님](https://urobot.kaist.ac.kr/url_teams/byeonghoyu/)의 큰 도움이 있었습니다,,,^^7

**P.P.S.** 이 series를 이해하기 위해서는 factor graph optimization과 3D rotation 에 대한 사전적 지식 (e.g., SO(3), skew-symmetric matrix, exponential/logarithm map 등등)이 필요로 합니다. 
* Graph SLAM에 대한 자료는 기섭이가 정리해둔 [SLAM Back-end 공부자료](https://gisbi-kim.github.io/blog/2021/10/03/slam-textbooks.html?fbclid=IwAR0NRvyHQhy6HaIIGMPudq4Mq34b1tQ_l5k3fEijHdk6KECTFsSHDBGhD8E)를 참고하시는 걸 추천드립니다.
* SO(3), skew-symmetric matrix, exponential/logarithm map 등의 용어가 친숙하지 않고 어떤 의미인지 헷갈리시는 분은 Joan Sola의 님의 [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508)의 Chapter 4까지 세 번 정독하는 것을 추천드립니다.


무튼 preintegration의 overview에 대해 살펴봤으니, 순차적으로 좀 더 자세히 preinegration에 대해 살펴볼 예정입니다. 편의 상 설명을 할 때는 반말로 진행하겠습니다 :).

---

IMU Preintegration Derivation 설명 시리즈입니다.

TBU
---


### 원 논문

* [Foster *et al.*, "IMU Preintegration on Manifold for Efficient
Visual-Inertial Maximum-a-Posteriori Estimation," in *Robotics: Science and Systems (RSS), 2011](http://www.roboticsproceedings.org/rss11/p06.pdf).
* [Foster *et al.*, "On-Manifold Preintegration for Real-Time
Visual-Inertial Odometry," *Transaction on Robotics*, 2016](https://rpg.ifi.uzh.ch/docs/TRO16_forster.pdf)