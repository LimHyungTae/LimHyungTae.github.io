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

## Derivative of Rotation

예로 들어 아래와 같은 vector 표현 식이 있다고 하자.

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

$$\left[\begin{array}{c}
x^{\prime} \\
y^{\prime} \\
1
\end{array}\right]=\left[\begin{array}{c}
x \cos \theta-y \sin \theta+t_x \\
x \sin \theta+y \cos \theta+t_y \\
1
\end{array}\right]$$


$$\mathbf{x}^{\prime} = \mathbf{R}\mathbf{x} + \mathbf{t}$$

위의 식을 $$\mathbf{t}$$에 대해 미분하면 

$$\tilde{\mathtt{R}}=\mathtt{R} \operatorname{Exp}(\epsilon), \quad \epsilon \sim \mathcal{N}(0, \Sigma) \; \; \; \; \text{[1]}$$ 


$$\mathbf{e}_{i j}$$