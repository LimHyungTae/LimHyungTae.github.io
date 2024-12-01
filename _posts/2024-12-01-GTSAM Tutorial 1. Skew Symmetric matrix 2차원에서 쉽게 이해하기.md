---
layout: post
title: GTSAM Tutorial 1. Skew Symmetric matrix 2차원에서 쉽게 이해하기
subtitle: Skew Symmetric
tags: [Jacobian, GTSAM]
comments: true
---

## Introduction 

Jacobian matrix를 알잘딱으로 구해주는 Ceres solver와는 다르게, GTSAM을 잘 활용하려면 Jacobian matrix를 일일이 구해주어야 한다.
그러다 보니, 사용자 입장에서는 진입 장벽이 좀 높은데, 오늘은 pose의 Jacobian을 구할 때 알아두면 좋을 개념에 대해 설명하고자 한다.

나는 3차원에서의 설명을 하기 앞서 저차원에서 현상에 대한 이해를 설명하는 것을 선호하는 편인데, 오늘은 rotation의 skew symmetric matrix에 대해 쉽게 설명하고자 한다.

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

---

## Block Operation

그런데 Frank Dellaert 교수님 자료를 보면 위의 식을 좀더 섹시(?)하게 block operation으로 기입하는 것을 볼 수 있다.
수식 [1]을 좀더 간결히 쓰면 아래와 같이 쓸 수 있는데: 

$$\mathbf{x}^{\prime} = T(\mathbf{x}) = \mathbf{R}\mathbf{x} + \mathbf{t} \; \; \; \; \text{[3]}$$

이를 통해 수식 [2]의 Jacobian을 block operation으로 간결히 적으면 아래와 같이 적을 수 있다:

$$J=\left[\begin{array}{ll}
\frac{\partial T(\mathbf{x})}{\partial \mathbf{t}} & \frac{\partial T(\mathbf{x})}{\partial \theta} \end{array}\right] \; \; \; \; \text{[4]}$$

위에서 $$\frac{\partial T(\mathbf{x})}{\partial \mathbf{t}}$$는 $$2\times1$$의 vector에 대한 partial derivative이므로 $$2\times2$$의 크기가 되고, $$\frac{\partial T}{\partial \theta}$$는 2개의 수식에 대한 1개의 변수의 derivative이기 때문에 $$2\times1$$의 matrix가 된다. 
이제 수식 [3]을 scalar로 구성된 equation이랑 동일하게 취급해보자. 그러면 $$\frac{\partial T(\mathbf{x})}{\partial \mathbf{t}}$$의 경우 $$T(\mathbf{x})$$ (i.e., [3])에서의 $$\mathbf{t}$$가 상수마냥 존재하기 때문에 원래 scalar의 세계에서는 1이 될 것이다.
하지만 우리는 현재 matrix의 세계에 있으므로, 이 값은 identity matrix $$\mathbf{I}_{2\times2}$$가 된다. 그리고 이는 [2]의 앞쪽 $$2\times2$$구간과 일치한다.

그리고 뒤의 $$2\times1$$ 구간은 $$\frac{\partial T(\mathbf{x})}{\partial \theta} = \frac{\partial \mathbf{R}}{\partial \theta}\mathbf{x}$$가 될 것이다. 그럼 여기서, $$\frac{\partial \mathbf{R}}{\partial \theta}$$는 어떻게 구할 수 있을까? 

## Derivative Rotation Matrix

나와 유사한 교과 과정을 겪은 이라면 2차원의 rotation은 각도 $$\theta$$에 대한 표현식으로 나타낼 수 있고, 이는 코마신신코(요즘에는 고등학교 때 행렬을 안 배운다고 하던데...)라는 것을 알고 있을 것이다:

$$\mathbf{R} = R(\theta)=\left[\begin{array}{cc}
\cos \theta & -\sin \theta \\
\sin \theta & \cos \theta
\end{array}\right]$$

이를 손으로 직접 미분하면 아래와 같은 값을 구할 수 있다:

$$\frac{\partial R(\theta)}{\partial \theta}=\left[\begin{array}{cc}
-\sin \theta & -\cos \theta \\
\cos \theta & -\sin \theta
\end{array}\right]$$

그리고 이를 다시 표현하면 아래와 같이 표현할 수 있는데, 

$$\frac{\partial R(\theta)}{\partial \theta}=R(\theta) \hat{\Omega}=\hat{\Omega}R(\theta)$$

$$\hat{\Omega}=\left[\begin{array}{cc}
0 & -1 \\
1 & 0
\end{array}\right]$$

https://blog.naver.com/spacebug/220102117054

## Derivative Rotation Matrix의 물리적 의미

<p align="center">
  <img src="/img/circular_motion.png" alt="Circular Motion">
</p>