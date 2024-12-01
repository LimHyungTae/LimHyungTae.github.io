---
layout: post
title: GTSAM Tutorial 3. Pose2의 unrotate Jacobian 이해하기
subtitle: Understanding H matrices in GTSAM
tags: [Jacobian, GTSAM]
comments: true
---

## Jacobian을 이해하는 데에 활용

Skew-symmetric matrix를 잘 이해하면, 아래의 코드도 이제 잘 이해가 될 것이다.
예를 들어, GTSAM의 `Pose2.cpp` 파일에서는 아래와 같이 `unrotate` 함수를 살펴보자. 
이렇게 GTSAM에서는 Jacobian `H1`과 `H2` 또한 return하는데,

```cpp
Point2 Rot2::unrotate(const Point2& p,
    OptionalJacobian<2, 1> H1, OptionalJacobian<2, 2> H2) const {
  const Point2 q = Point2(c_ * p.x() + s_ * p.y(), -s_ * p.x() + c_ * p.y());
  if (H1) *H1 << q.y(), -q.x();
  if (H2) *H2 = transpose();
  return q;
}
```

현재 위의 `unroate`에서 일어나는 이 수식을 $$f = \mathbf{R}^{\intercal}\mathbf{p}$$라 표현한다면,

* `H1`: 회전 각 $$\theta$$에 대한 Jacobian

앞선 글에서 처럼 이번에는 transposed rotation을 $$\theta$$에 대해 미분하면 아래의 결과를 얻을 수 있다:

$$\frac{\partial {R(\theta)^\intercal}}{\partial \theta}=\left[\begin{array}{cc}
-\sin \theta & \cos \theta \\
-\cos \theta & -\sin \theta
\end{array}\right] = - \left[\begin{array}{cc}
0 & -1 \\
1 & 0
\end{array}\right] \mathbf{R}^{\intercal}$$

따라서 $$\frac{\partial {R(\theta)^\intercal}}{\partial \theta}\mathbf{p}$$는 위의 코드에서 `q`$$=\mathbf{R}^{\intercal}\mathbf{p}$$를 계산한 후, 음의 방향인 skew-symmetric matrix인 $$\left[\begin{array}{cc}
0 & 1 \\
-1 & 0
\end{array}\right]$$을 곱해서 최종적으로 `*H1 << q.y(), -q.x()`로 할당되는 것을 볼 수 있다.

* `H2`: $$\mathbf{p}$$에 대한 Jacobian이므로, $$\mathbf{R}^{\intercal}$$ (i.e., $$f$$를 미분하면 $$\mathbf{p}$$에 곱해져있던 계수와 대응되는 $$\mathbf{R}^{\intercal}$$이 남음) 

이처럼 block operation을 통한 미분에 친숙해져야지 GTSAM을 더 잘 이해할 수 있게 된다.

시간이 될때 다른 error term에 대한 Jacobian도 종종 업데이트할 예정...

---

GTSAM Tutorial 시리즈입니다.

{% include post_links_gtsam.html %}