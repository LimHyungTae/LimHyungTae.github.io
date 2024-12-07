---
layout: post
title: GTSAM Tutorial 3. Skew Symmetric matrix 2차원에서 쉽게 이해하기
subtitle: Skew Symmetric
tags: [Jacobian, GTSAM]
comments: true
---

## Introduction 

나는 3차원에서의 설명을 하기 앞서 저차원에서 현상에 대한 이해를 설명하는 것을 선호하는 편인데, 오늘은 rotation의 skew symmetric matrix에 대해 쉽게 설명하고자 한다 ([이전 글](https://limhyungtae.github.io/2024-12-01-GTSAM-Tutorial-2.-SE(2)-Transformation-matrix%EC%99%80-Jacobian-Block-Operation%EC%9C%BC%EB%A1%9C-%ED%91%9C%ED%98%84%ED%95%98%EA%B8%B0/)과 이어짐).

## Derivative of Rotation Matrix

나와 유사한 교과 과정을 겪은 이라면 2차원의 rotation은 각도 $$\theta$$에 대한 표현식으로 나타낼 수 있고, 이는 코마신신코(요즘에는 고등학교 때 행렬을 안 배운다고 하던데...)라는 것을 알고 있을 것이다:

$$\mathbf{R} = {R}(\theta) =\left[\begin{array}{cc}
\cos \theta & -\sin \theta \\
\sin \theta & \cos \theta
\end{array}\right]$$

이를 손으로 직접 미분하면 아래와 같은 값을 구할 수 있다:

$$\frac{\partial R(\theta)}{\partial \theta}=\left[\begin{array}{cc}
-\sin \theta & -\cos \theta \\
\cos \theta & -\sin \theta
\end{array}\right]$$

그리고 이를 다시 rotation matrix $$R(\theta)$$로 표현하면 아래와 같이 표현할 수 있는데, 

$$\frac{\partial R(\theta)}{\partial \theta}=R(\theta) \hat{\Omega}=\hat{\Omega}R(\theta) \; \; \text{where} \; \; \hat{\Omega}=\left[\begin{array}{cc}
0 & -1 \\
1 & 0
\end{array}\right]   \; \; \; \; \text{(6)}$$

여기서 위의 $$\hat{\Omega}$$가 우리가 Lie algebra를 공부할 때 흔히 들었던 **skew-symmetric matrix**이다. 
즉, 최종적으로 이전 글에서 수식 (5) $$\frac{\partial \mathbf{R}}{\partial \theta}\mathbf{x}$$의  $$\frac{\partial \mathbf{R}}{\partial \theta}$$에 수식 (6)을 대입해보면, $$\hat{\Omega}R(\theta)\mathbf{x}$$가 된다. 이는 $$\mathbf{x}$$를 $$R(\theta)$$로 회전시킨 후 skew-symmetric matrix를 곱한 것이 최종적으로 rotation matrix의 미분에 해당되는 값이라는 것을 알 수 있다. 

## Derivative Rotation Matrix의 물리적 의미

<p align="center">
  <img src="/img/circular_motion.png" alt="Circular Motion">
</p>
(위의 그림은 [여기](https://simagebank.net/wp/5257/)에서 발췌)

그렇다면 이게 물리적으로 어떤 의미를 뜻할까?
크게 두 가지로 해석할 수 있을 것 같은데, 첫번 째로는 skew-symmetric matrix로 변형된 vector는 위의 그림의 검은 진한 화살표와 동일하다는 것을 알 수 있다.
즉, 회전에 접선 방향에 대한 움직임을 나타내는 것이라고 해석될 수 있다.
예로 들어서, rotation matrix로 회전된 값이 $$(\frac{1}{2}, \frac{1}{2})$$라고 할 때, 여기에 $$\hat{\Omega}$$을 곱하게 되면 $$(-\frac{1}{2}, \frac{1}{2})$$ 값이 되는데, 이를 그려보면 정확히 원의 접선 vector와 일치하는 것을 볼 수 있다(대강 위의 그림의 $$\mathbf{Q}$$ 의 화살표 방향 일치함).

두 번째로는, rotation의 회전의 경우 기존 값 $$x$$와 $$y$$의 영향을 받는다는 것인데, 이 또한 자명하다.
왜냐하면 동일한 회전을 하더라도 회전을 하고자하는 길이(위의 그림에서의 $$r$$ 부분)가 길어지게 되면 각도가 동일하게 변경되더라도 더 많은 움직임이 발생하기 때문이다. 

--- 


GTSAM Tutorial 시리즈입니다.

{% include post_links_gtsam.html %}