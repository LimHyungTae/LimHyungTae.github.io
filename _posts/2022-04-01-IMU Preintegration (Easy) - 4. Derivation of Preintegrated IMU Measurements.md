---
layout: post
title: IMU Preintegration (Easy) - 4. Derivation of Preintegrated IMU Measurements
subtitle: Preintegrated IMU Measurements
tags: [SLAM, LIO, VIO, IMU, Preintegration]
comments: true

---



본 글에서는 이제 총 K개의 IMU measurement를 통해 표현한 $$i$$와 $$j$$ 번째의 Relative motion을 preintegration하는 방법에 대해 설명한다.


## Preintegrated IMU Measurements

앞서 유도한 아래의 수식을 다시 살펴보자.

![](/img/preintegration/final_i_j_v2.png)

위의 수식에서 $$\mathtt{R}_i$$, $$\mathbf{v}_i$$, $$\mathbf{p}_i$$를 좌변으로 이항해주면 $$i$$ 번째 keyframe과 $$j$$ 번째 keyframe 사이의 최종적인 relative motion에 대해 표현할 수 있다. Introduction에서 말했듯이, 위의 수식만을 이용해서 factor graph optimization을 하는 것은 이미 가능하다. 하지만 위의 수식을 이용해서 factor graph optimization을 하면 $$i+1, i+2, \cdots, j-1$$의 pose들 또한 모두 다 parameter로 지니고 있어야 한다. 이는 앞서 말한 것 처럼 연산적인 측면이나 메모리적인 측면이나 모두 다 부담이 된다 ([Intro](https://limhyungtae.github.io/2022-04-01-IMU-Preintegration-(Easy)-1.-Introduction/)에서 약 25분 동안 데이터를 취득하면 49만 여개의 데이터가 취득된 것을 상기하자).

이러한 문제를 해결하기 위해 논문의 저자는 수백 여개의 measurments를 factor로 직접적으로 넣는 것이 아니라, factor graph에 measurement를 추가하기 이전에 (pre-) 수백여개의 IMU data를 단 하나의 uncertainty term으로 취합(integration)하는 방법에 대해서 제안을 하는 것이다.

### Definition & Assumption

**Definition**: Preintegration의 과정을 유도하기 앞서 논문에서는 아래와 같이 "relative motion increments" $$\Delta \mathtt{R}_{ij}$$, $$\Delta \mathbf{v}_{ij}$$, $$\Delta \mathbf{p}_{ij}$$를 정의한다:

![](/img/preintegration/relative_motion_increments.png)


여기서 $$\Delta t_{ij}$$는 두 keyframe 간의 총 시간 차를 의미한다. 참고로 이 relative motion increments 중 $$\Delta \mathbf{v}_{ij}$$와 $$\Delta \mathbf{p}_{ij}$$는 물리적으로 incremental motion을 뜻하지는 않는다. 왜냐하면 전개의 편의성을 위해서 중력에 관련된 term도 좌변으로 이항했기 때문이다. 실제로 $$k$$ 번째와 $$k+1$$ frame 사이의 물리적인 relative motion (아래의 자주색으로 표시된 부분)은 아래와 같다는 것을 주의하자.

![](/img/preintegration/physical_meaning.png)


**Assumption**: 그 후, 수식을 전개하기 전 $$i$$ 번째와 $$j$$ keyframe 사이에서는 bias가 일정하다고 가정한다.

![](/img/preintegration/preint_bias.png)

실제로 bias는 주로 MEMS system 상에 전기적 신호의 offset으로 인해 발생하는 것이기 때문에 빠른 시간 내에 급격하게 변화하지 않는다. 따라서 위처럼 가정을 해도 괜찮다.

## Derivation of Preintegrated IMU Measurements

위의 정의와 가정 이후 아래와 같이 실제 relative motion increments와 noise와 bias의 영향을 받은 measured relative motion increments간의 상관관계를 유도한다 (아래 전개가 이해가 안 되시는 분은 Joan Sola의 님의 [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508),,,Chapter 4까지,,,꼭 읽어보시길 바랍니다,,,).

### Preintegrated Rotation Measurements

![](/img/preintegration/preinteg_rot.png)

### Preintegrated Velocity Measurements
![](/img/preintegration/preinteg_vel_v2.png)

### Preintegrated Position Measurements

![](/img/preintegration/preinteg_pos_v2.png)


## Final Results

위의 전개를 하다보면 '이걸 왜 하고 있는거지...?'하며 살짝 아득해지는데, 이제 거의 다 왔다! 위의 전개를 통한 증명의 의의는 아래와 같이 "IMU data로 measured된 relative motion increments는 추정해야하는 (to-be-estimated) relative motion increments과 gaussian distribution을 따르는 random vector의 합의 꼴로 표현이 가능하다"는 것이다:

![](/img/preintegration/behave_like.png)

Uncertainty가 gaussian distribution의 꼴이라는 말은 optimization의 결과가 Maximum a posteriori (MAP)의 최적값으로 estimate되는 것이 가능하다는 것을 뜻한다 (cf. Uncertainty가 gaussian의 꼴을 따르지 않으면 이는 단순히 non-linear optimization이라 부른다).

따라서 preintegration을 하려면 다음과 같은 값들이 주어지고, 이들을 통해 optimization을 하면 된다.


## Bias Update 


![](/img/preintegration/bias_description.png)

## 결론

* 주어져야 하는 것: $$i$$번째 keyframe과 $$j$$번째 keyframe의 pose. 이것들은 optimization을 할 때의 prior factor로 사용된다
* 계산해야 하는 것: K개의 IMU data로부터 $$\Delta \tilde{\mathtt{R}}_{ij}$$, $$\Delta \tilde{\mathbf{v}_{ij}}$$, $$\Delta \tilde{\mathbf{p}}_{ij}$$, $$\delta {\boldsymbol{\phi}}_{ij}$$, $$\delta {\mathbf{v}_{ij}}$$, $$\delta \tilde{\mathbf{p}}_{ij}$$.
* Optimization되는 것: IMU의 bias term들

---

IMU Preintegration Derivation 설명 시리즈입니다.

TBU

---


### 원 논문

* [Foster *et al.*, "IMU Preintegration on Manifold for Efficient
Visual-Inertial Maximum-a-Posteriori Estimation," in *Robotics: Science and Systems (RSS), 2011](http://www.roboticsproceedings.org/rss11/p06.pdf).
* [Foster *et al.*, "On-Manifold Preintegration for Real-Time
Visual-Inertial Odometry," *Transaction on Robotics*, 2016](https://rpg.ifi.uzh.ch/docs/TRO16_forster.pdf)