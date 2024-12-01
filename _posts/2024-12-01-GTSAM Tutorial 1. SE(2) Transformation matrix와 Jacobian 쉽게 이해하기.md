---
layout: post
title: GTSAM Tutorial 1. SE(2) Transformation matrix와 Jacobian 쉽게 이해하기
subtitle: Skew Symmetric
tags: [Jacobian, GTSAM]
comments: true
---

## Introduction 

Jacobian matrix를 알잘딱으로 구해주는 Ceres solver와는 다르게, GTSAM을 잘 활용하기 위해서는 Jacobian matrix가 어떻게 도출되는 것인지에 대해서도 정확히 알아야 한다(i.e., 아래의 `H1`과 `H2`가 어떻게 구해지는 지에 대해서도 이해를 해야 함):

```angular2html
// See https://github.com/borglab/gtsam/blob/979fb93b98a03bbca310461e73dcb282f5866b6f/gtsam_unstable/slam/PoseBetweenFactor.h#L94
Vector evaluateError(const POSE& p1, const POSE& p2,
    OptionalMatrixType H1, OptionalMatrixType H2) const override {
  if(body_P_sensor_) {
    POSE hx;
    if(H1 || H2) {
      Matrix H3;
      hx = p1.compose(*body_P_sensor_,H3).between(p2.compose(*body_P_sensor_), H1, H2); // h(x)
      if(H1) (*H1) *= H3;
      if(H2) (*H2) *= H3;
    } else {
      hx = p1.compose(*body_P_sensor_).between(p2.compose(*body_P_sensor_)); // h(x)
    }
    // manifold equivalent of h(x)-z -> log(z,h(x))
    return measured_.localCoordinates(hx);
  } else {
    POSE hx = p1.between(p2, H1, H2); // h(x)
    // manifold equivalent of h(x)-z -> log(z,h(x))
    return measured_.localCoordinates(hx);
  }
}
```

사실 이러한 Jacobian을 이미 기존에 짜있는 factor를 이용할 때는 깊은 생각 없이 그냥 가져다 쓰면 된다. 하지만 자신이 새로운 factor를 만들어 optimization을 하고자 할 때는 사용자 입장에서는 GTSAM이 Ceres에 비해 진입 장벽이 좀 높다고 생각된다.

그래서 이번에는 pose의 Jacobian을 구할 때 알아두면 좋을 개념에 대해 설명하고자 한다.

## Jacobian of SE(2) Transformation


예로 들어 아래와 같은 SE(2)를 통한 transformation 표현 식이 있다고 하자.

$$ \left[\begin{array}{l}
x^{\prime} \\
y^{\prime} \\
1
\end{array}\right]=T\left(\theta, t_x, t_y\right)\left[\begin{array}{l}
x \\
y \\
1
\end{array}\right]=\left[\begin{array}{ccc}
\cos \theta & -\sin \theta & t_x \\
\sin \theta & \cos \theta & t_y \\
0 & 0 & 1
\end{array}\right]\left[\begin{array}{l}
x \\
y \\
1
\end{array}\right] \; \; \; \; \text{[1]}$$

위의 식을 살펴 보면 우리가 늘 봐왔듯이 $$\left[\begin{array}{cc}
\mathbf{R} & \mathbf{t} \\
0 & 1
\end{array}\right]$$의 꼴로 구성되어 있는 것을 볼 수 있다(note: IEEE format에서는 matrix나 vector는 bold체로 써야한다. 이해를 할 때 참고).

그리고 이를 전개하면 아래와 같은데: 


$$\left[\begin{array}{c}
x^{\prime} \\
y^{\prime} \\
1
\end{array}\right]=\left[\begin{array}{c}
x \cos \theta-y \sin \theta+t_x \\
x \sin \theta+y \cos \theta+t_y \\
1
\end{array}\right]$$

위의 식의 Jacobian을 구해 보자. Jacobian의 각 row는 equation, column은 각 변수에 대한 미분 값을 나타내므로, Jacobian $$J$$는 아래와 같이 쓰여지고:

$$J=\left[\begin{array}{lll}
\frac{\partial x^{\prime}}{\partial t_x} & \frac{\partial x^{\prime}}{\partial t_y} & \frac{\partial x^{\prime}}{\partial \theta} \\
\frac{\partial y^{\prime}}{\partial t_x} & \frac{\partial y^{\prime}}{\partial t_y} & \frac{\partial y^{\prime}}{\partial \theta}
\end{array}\right]$$

최종적으로 이를 풀면 아래와 같다:

$$J=\left[\begin{array}{ccc}
1 & 0 & -x \sin \theta-y \cos \theta \\
0 & 1 & x \cos \theta-y \sin \theta
\end{array}\right] \; \; \; \; \text{[2]}$$

