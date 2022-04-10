---
layout: post
title: IMU Preintegration (Easy) - 4. Derivation of Preintegrated IMU Measurements
subtitle: Why is the IMU Preintegration important
tags: [SLAM, LIO, VIO, IMU, Preintegration]
comments: true

---

# Derivation of Preintegrated IMU Measurements

본 글에서는 이제 K개의 IMU measurement를 통해 표현한 $$i$$와 $$j$$ 번째의 Relative motion을 하나의 요소로 미리 취합(preintegration)하는 방법에 대해 설명한다 (수학주의).


## Preintegrated IMU Measurements

앞서 유도한 아래의 수식을 다시 살펴보자.

![](/img/preintegration/final_i_j.png)

위의 수식에서 $$\mathtt{R}_i$$, $$\mathbf{v}_i$$, $$\mathbf{p}_i$$를 좌변으로 이항해주면 $$i$$ 번째 keyframe과 $$j$$ 번째 keyframe 사이의 최종적인 relative motion에 대해 표현할 수 있다 


위의 수식을 통해 $$i$$ 번째와 $$j$$의 pose 차이를 기술할 수 있다. 하지만 그렇게 되면 앞서 말한 것 처럼 두 keyframe 사이에서 변하는 pose에 대한 parameter를 factor graph 상에서 다 저장해야하기 때문에 비효율적이다. 

이러한 문제를 해결하기 위해 논문의 저자는 수백 여개의 measurments를 factor로 직접적으로 넣는 것이 아니라, factor graph에 measurement를 추가하기 이전에 (pre-) 수백여개의 IMU data를 단 하나의 factor로 취합(integration)하는 방법에 대해서 제안을 하는 것이다.

### Definition & Assumption

Preintegration의 과정을 유도하기 앞서 논문에서는 아래와 같이 *relative motion increments*에 대해서 정의한다:
![](/img/preintegration/relative_motion_increments.png)


여기서 주의할 것은, 이 *relative motion increments*는 물리적으로 incremental motion을 뜻하는 것은 아니다. 왜냐하면 전개의 편의성을 위해서 중력에 관련된 term도 좌변으로 이항했기 때문이다. 실제로 물리적인 motion (  아래의 자주색으로 표시된 부분)은 각 $$k$$ 번째와 $$k+1$$의 물리적인 미세한 relative motion을 의미한다.

![](/img/preintegration/physical_meaning.png)


그 후, 수식을 전개하기 전 $$i$$ 번째와 $$j$$ keyframe 사이에서는 bias가 일정하다고 가정한다.

![](/img/preintegration/preint_bias.png)

(실제로 bias는 주로 MEMS system 상에 전지적 신호의 offset으로 인해 발생하는 것이기 떄문에 위처럼 가정을 해도 괜찮다. 자세한 설명은  
[여기](http://www.canalgeomatics.com/knowledgebase/imu-accuracy-error-definitions/ 참조)




이러한 가정 이후 아래와 같이 rotation, velocity, position에 대한 preintegration term을 표현한다 (아래 rotation term의 전개가 이해가 안 되시는 분은 Joan Sola의 님의 [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508),,,Chapter 4까지,,,꼭 읽어보시길 바랍니다,,,).

### Preintegrated Rotation Measurements

![](/img/preintegration/preinteg_rot.png)

### Preintegrated Velocity Measurements
![](/img/preintegration/preinteg_vel.png)

### Preintegrated Position Measurements

![](/img/preintegration/preinteg_pos.png)


## Final Results


---

IMU Preintegration Derivation 설명 시리즈입니다.

TBU
---


### 원 논문

* [Foster *et al.*, "IMU Preintegration on Manifold for Efficient
Visual-Inertial Maximum-a-Posteriori Estimation," in *Robotics: Science and Systems (RSS), 2011](http://www.roboticsproceedings.org/rss11/p06.pdf).
* [Foster *et al.*, "On-Manifold Preintegration for Real-Time
Visual-Inertial Odometry," *Transaction on Robotics*, 2016](https://rpg.ifi.uzh.ch/docs/TRO16_forster.pdf)