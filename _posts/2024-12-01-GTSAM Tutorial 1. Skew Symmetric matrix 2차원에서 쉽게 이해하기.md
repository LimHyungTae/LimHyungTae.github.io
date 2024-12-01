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

## SE(2) Transformation


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
\end{array}\right] $$

위의 식을 살펴 보면 우리가 늘 봐왔듯이 $$\left[\begin{array}{cc}
\mathbf{R}(\theta) & \mathbf{t} \\
0 & 1
\end{array}\right]$$의 꼴로 구성되어 있는 것을 볼 수 있다.

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

위의 식의 Jacobian을 구하면 최종적으로 아래와 같이 풀 수 있다:

$$J=\left[\begin{array}{ccc}
1 & 0 & -x \sin \theta-y \cos \theta \\
0 & 1 & x \cos \theta-y \sin \theta
\end{array}\right]$$

---



$$\frac{\partial x^{\prime}}{\partial t_x}=1, \quad \frac{\partial y^{\prime}}{\partial t_x}=0, \frac{\partial x^{\prime}}{\partial t_y}=0, \quad \frac{\partial y^{\prime}}{\partial t_y}=1$$ 

$$\mathbf{x}^{\prime} = \mathbf{R}\mathbf{x} + \mathbf{t}$$

위의 식을 $$\mathbf{t}$$에 대해 미분하면 
