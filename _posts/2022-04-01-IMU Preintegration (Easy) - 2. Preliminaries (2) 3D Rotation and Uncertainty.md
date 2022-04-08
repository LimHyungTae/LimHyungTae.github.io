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
먼저 preintegration을 이해하기 위해서는 manifold의 이해를 필요로 한다. 하지만 수학 위주의 엄밀한 증명보다는 본 글에서는 왜 SO(3)를 쓰는 게 용이한지에 좀 더 조첨을 맞춘다.

## Rotation from 2D to 3D

*3D 상에서 Rotation을 표현하는 방법*이라고 하면 크게 
* a) euler angle representation, 
* b) quaternion, 
* c) Lie Group, i.e. SO(3)
  
를 떠올릴 것이다. 이 세 가지에 대해 친숙하지 않더라도, 괜찮다. 사실 우리는 교육과정에서 이러한 여러 rotation representation의 개념에 대해 알게 모르게 배웠다. 2D의 개념과 3D의 개념의 상관관계를 하나의 표로 정리하면 아래와 같다.

![](/img/rotation/overview_v2.png)

Rotation에 대해 자세히 설명하면 글이 너무 길어지기 때문에 자세한 설명글은 **여기(TBU)** 에서 다룬다.


## Uncertainty Description in SO(3) (Section Ⅲ. *B* ) 

On-Manifold Preintegration를 이해하는 데 가장 핵심적인, base가 되는 개념은 Section Ⅲ. *B*에서 소개되는, SO(3)의 uncertainty에 관련된 내용이다. 겉보기에는 논문에서도 preliminaries에 있어서 별로 안 중요하겠거니 했는데 **이 부분의 의미를 깨닫는 것이 제일 중요하다**. SO(3)의 uncertainty를 표현하는 것에 대해 알지 못하면 뒤의 Section에서 전개를 통해 preintegrated measurements를 표현하는 과정을 왜 하고 있는지 이해할 수 없기 때문이다.

3차원 상에서의 회전에 대한 불확실성은 불확실한 정도의 크기가 작다는 가정하면, 회전에 대한 noise는 기존 noise-free인  rotation matrix * noise를 exponential map을 통해 투영한 rotation matrix의 곱으로 아래와 같이 표현이 가능하다 (SO(3) $$ \times $$ SO(3) → SO(3)):

$$\tilde{\mathtt{R}}=\mathtt{R} \operatorname{Exp}(\epsilon), \quad \epsilon \sim \mathcal{N}(0, \Sigma) \; \; \; \; \text{[1]}$$ 

즉, 위의 식은 우리 주변의 3차원을 표현하는 manifold 상의 uncertainty를 3x1 크기를 지니고 zero-mean인 gaussian distribution으로 표현할 수 있음을 뜻한다.  

위의 수식 [1]을 활용하면 $$ i $$ 번 째 world 좌표계 기준의 rotation과 $$ i $$ 번째와 인접하는 $$ j $$번째 world 좌표계 기준 사이의 relative rotation과 그에 대한 uncertainty 또한 아래와 같이 나타낼 수 있고: $$ x $$

$$\Delta \tilde{\mathtt{R}}_{ij}=\Delta \mathtt{R}_{ij} \operatorname{Exp}(\epsilon), \quad \epsilon \sim \mathcal{N}(0, \Sigma) \; \; \; \; \text{[2]}$$ 

Factor graph SLAM의 최종 목표가 아래의 optimization 수식을 푸는 것과 같은데:

$$\mathbf{x}^{*}=\operatorname{argmin} \sum_{\mathbf{x}} \mathbf{e}_{i j}^{T} {\Omega}_{i j} \mathbf{e}_{i j} \; \; \; \; \;  \; \text{[3]}$$

수식 [1]을 바탕으로 [2]의 omega에 대응되는 부분(어려운 말로는 information matrix라 부름)을 저 epsilon의 역수를 통해 modeling을 가능하게 한다는 궁극적인 의미가 있다 (epsilon이 크다 → measurements가 불확실하다는 의미 → 해당 measurements에 해당하는 error 크기의 중경도를 따질 때 덜 중요하다고 여김, i.e. 지닌 information의 중요한 정도가 낮음. 이는 기존의 graph SLAM과 동일하다). $$ 5 + 5 $$



이 개념을 잘 받아들이는 게 중요한데, 왜냐하면 이 논문의 Section Ⅵ에서 설명하는 preintegration on manifold가 최종적으로 나타내고 싶은 것이 keyframe i와 j의 사이의 수십~수백 여개의 IMU measurements를 사용해서 아래와 같이 i와 j 간의 relative rotation을 

$$\tilde{\mathtt{R}}_{ij}=\mathtt{R}_{ij} \operatorname{Exp}(\epsilon^\prime)$$


Joan Sola의 님의 [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508)

HELLO $$ \alpha $$ and$$ \beta $$ with you $$\gamma$$.
---

IMU Preintegration Derivation 설명 시리즈입니다.

TBU
---


### 원 논문

* [Foster *et al.*, "IMU Preintegration on Manifold for Efficient
Visual-Inertial Maximum-a-Posteriori Estimation," in *Robotics: Science and Systems (RSS), 2011](http://www.roboticsproceedings.org/rss11/p06.pdf).
* [Foster *et al.*, "On-Manifold Preintegration for Real-Time
Visual-Inertial Odometry," *Transaction on Robotics*, 2016](https://rpg.ifi.uzh.ch/docs/TRO16_forster.pdf)