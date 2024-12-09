---
layout: post
title: GTSAM Tutorial 5. Rot2의 unrotate 함수를 예제로 Jacobian 구해보기
subtitle: Understanding H matrices in GTSAM
tags: [Jacobian, GTSAM]
comments: true
---

## `unrotate` 함수의 Jacobian 값 유도를 통해 엿보는 skew-symmetric matrix 활용 방법

Unary factor에서는 rotation에 대한 수식이 없었으므로, rotation이 있을 때에 `H`를 구하는 방법에 대해 좀더 친숙해지기 위해 예시를 하나 더 들어보자.
만약 Skew-symmetric matrix의 특성을 잘 이해했다면, 이 글도 잘 이해가 될 것이다.

GTSAM의 `Rot2.cpp` 파일에서는 아래와 같이 `unrotate` 함수가 있다.
`unrotate` 함수에서는 아래와 Jacobian `H1`과 `H2` 또한 return하는데:

```cpp
Point2 Rot2::unrotate(const Point2& p,
    OptionalJacobian<2, 1> H1, OptionalJacobian<2, 2> H2) const {
  const Point2 q = Point2(c_ * p.x() + s_ * p.y(), -s_ * p.x() + c_ * p.y());
  if (H1) *H1 << q.y(), -q.x();
  if (H2) *H2 = transpose();
  return q;
}
```

GTSAM에서 `H1`는 해당 객체를 위한 Jacobian(e.g., 위의 예제에서는 `Rot2` 클래스를 위한 Jacobian이 유도되므로, 회전 각 $$\theta$$에 대한 $$2\times1$$ Jacobian을 나타냄), 
`H2`는 입력 값에 대한 Jacobian(e.g., 위의 `unrotate` 함수에서 입력은 `Point2` 객체이므로. `Point2`에 대한 $$2\times2$$ Jacobian이 리턴됨)을 각각 나타낸다.

--- 

## `unrotate`에서 `H1`과 `H2` 유도 과정

현재 위의 `unrotate`에서 일어나는 이 수식을 $$f(\theta, \mathbf{p}) = \mathbf{R}^{\intercal}\mathbf{p}$$라 표현한다면, 우리는 이전 글에서 했듯이 아래와 같은 관계식을 풀어야 한다:

$$f(\theta + \delta \theta,  \mathbf{p} + \delta \mathbf{p}) = \mathbf{H}_1 \delta \theta + \mathbf{H}_2 \delta \mathbf{p}\;\;\;\;(1)$$

그리고 $$f(\theta, \mathbf{p}) = \mathbf{R}^\intercal \mathbf{p}$$이므로, 

$$f(\theta + \delta \theta, \mathbf{p} + \delta \mathbf{p}) = R(\theta + \delta\theta)^\intercal \left(\mathbf{p} + \delta\mathbf{p}\right) \;\;\;\;(2)$$
(작성 중)

[앞선 글](https://limhyungtae.github.io/2024-12-01-GTSAM-Tutorial-3.-Skew-Symmetric-matrix-2차원에서-쉽게-이해하기/)에서 $$R(\theta)$$의 미분 값을 구했던 것 처럼, 이번에는 transposed rotation을 $$\theta$$에 대해 미분하면 아래의 결과를 얻을 수 있다:

$$\frac{\partial {R(\theta)^\intercal}}{\partial \theta}=
\left[\begin{array}{cc}
-\sin \theta & \cos \theta \\
-\cos \theta & -\sin \theta
\end{array}\right] = - \left[\begin{array}{cc}
0 & -1 \\
1 & 0
\end{array}\right] \mathbf{R}^{\intercal} = -\hat{\Omega}\mathbf{R}^{\intercal}$$

여기서 $$\hat{\Omega}$$는 skew-symmetric matrix이다 ([이전 글](https://limhyungtae.github.io/2024-12-01-GTSAM-Tutorial-3.-Skew-Symmetric-matrix-2차원에서-쉽게-이해하기/)에서 설명하였다). 

따라서 $$\frac{\partial \mathbf{f}}{\partial \theta} = \frac{\partial {R(\theta)^\intercal}}{\partial \theta}\mathbf{p}$$는 위의 코드에서 `const Point 2 q`$$=\mathbf{R}^{\intercal}\mathbf{p}$$를 계산한 후, 음의 방향인 skew-symmetric matrix인 $$\left[\begin{array}{cc}
0 & 1 \\
-1 & 0
\end{array}\right]$$을 곱해서 최종적으로 `*H1 << q.y(), -q.x()`로 할당되는 것을 볼 수 있다.

--- 

`H2`의 경우, $$\mathbf{p}$$에 대한 Jacobian, i.e., $$\frac{\partial \mathbf{f}}{\partial \mathbf{p}}$$, 인데 $$\mathbf{f}$$를 미분하면 $$\mathbf{p}$$에 곱해져있던 계수와 대응되는 $$\mathbf{R}^{\intercal}$$이 남기 때문에 최종적으로 $$\mathbf{R}^{\intercal}$$가 되는 것을 
---

## Conclusion

이처럼 block operation으로 표현된 수식에 대한 Jacobian을 구하는 방법에 친숙해져야지 GTSAM을 더 잘 이해할 수 있게 된다.

이제 skew-symmetric matrix에 대한 이해를 했으니, 다양한 factor에 대한 Jacobian을 유도해보자.

---

GTSAM Tutorial 시리즈입니다.

{% include post_links_gtsam.html %}