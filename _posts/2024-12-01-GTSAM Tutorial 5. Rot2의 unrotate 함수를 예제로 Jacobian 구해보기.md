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

현재 위의 `unrotate`에서 일어나는 이 수식을 $$f(\theta, \mathbf{p}) = \mathbf{R}^{\intercal}\mathbf{p}$$라 표현하자. 2D의 경우 rotation에 해당하는 vector는 1차원 값인 $$\theta$$로 표현이 가능하다. 우리는 이전 글에서 했듯이 최종적으로 아래와 같은 관계식을 전개해서 $$\mathbf{H}_1$$과 $$\mathbf{H}_2$$를 구해야 한다:

$$f(\theta + \delta \theta,  \mathbf{p} + \delta \mathbf{p}) = f(\theta,  \mathbf{p}) + \mathbf{H}_1 \delta \theta + \mathbf{H}_2 \delta \mathbf{p}\;\;\;\;(1)$$

그리고 $$f(\theta, \mathbf{p}) = \mathbf{R}^\intercal \mathbf{p}$$이므로, 

$$f(\theta + \delta \theta, \mathbf{p} + \delta \mathbf{p}) = R(\theta + \delta\theta)^\intercal \left(\mathbf{p} + \delta\mathbf{p}\right) = R(-\theta - \delta\theta)\left(\mathbf{p} + \delta\mathbf{p}\right)  \;\;\;\;(2)$$

이고, 

$$R(- \delta\theta)$$는 small angle approximation을 활용하여 아래와 같이 표현할 수 있다:

$$R\left(-\delta \theta \right)=\left[\begin{array}{cc}
\cos \left(-\delta \theta \right) & -\sin \left(-\delta \theta \right) \\
\sin \left(-\delta \theta \right) & \cos \left(-\delta \theta \right)
\end{array}\right] \approx\left[\begin{array}{cc}
1 & \delta \theta  \\
-\delta \theta  & 1
\end{array}\right]=\mathbf{I}-R(\pi / 2) \delta \theta  \;\;\;\;(3)$$

따라서, 2D에서는 $$R(-\theta - \delta\theta) = R(-\theta) R(- \delta\theta)$$임을 이용하면 $$\mathbf{R}^\intercal \left(\mathbf{I}-R(\pi / 2) \delta \theta\right)$$와 같이 표현할 수 있고, 이를 다시 수식 (2)에 대입해서 전개하면 아래와 같이 표현할 수 있다:

$$\begin{align}
R(-\theta - \delta\theta)\left(\mathbf{p} + \delta\mathbf{p}\right) 
&= \mathbf{R}^\intercal \left(\mathbf{I}-R(\pi / 2) \delta \theta\right)\left(\mathbf{p} + \delta\mathbf{p}\right) \\
&= \mathbf{R}^\intercal \mathbf{p} 
   + \mathbf{R}^\intercal \delta\mathbf{p} 
   - \mathbf{R}^\intercal R(\pi / 2) \delta \theta \mathbf{p} 
   - \mathbf{R}^\intercal R(\pi / 2) \delta \theta \delta \mathbf{p} \\
&\simeq f(\theta, \mathbf{p}) 
   + \mathbf{R}^\intercal \delta\mathbf{p} 
   - \mathbf{R}^\intercal R(\pi / 2) \delta \theta \mathbf{p}.  \;\;\;\;(4)
\end{align}$$

수식 (4)에서 두번 째 줄의 제일 마지막 항인 $$\mathbf{R}^\intercal R(\pi / 2) \delta \theta \delta \mathbf{p}$$가 생략된 이유는, approximation을 할 때 자주 활용하는 기법인데, 미소 값을 두 번 곱하게 되면(i.e., $$\delta \theta$$와 $$\delta \mathbf{p}$$를 곱하는 행위) 그 값이 다른 term들에 비해 월등히 작아지기 때문이다. 따라서, 이미 이 term이 위의 수식 (4)에 영향이 미미하기 때문에, 계산 효율성을 위해 생략이 가능해진다.  

위의 식을 다시 살펴보면 $$\delta\mathbf{p}$$에 대한 $$\mathbf{H}_2$$는 $$\mathbf{H}_2 = \mathbf{R}^\intercal$$로 구해진 것을 확인할 수 있다. $$\delta \theta$$는 현재 term의 중간에 있는데,$$\mathbb{R}^1$$인 scalar 값이므로, 세번 째 항의 제일 뒷쪽으로 보내는 것이 가능하다. 그리고 이전에 skew-symmetric matrix에 대해 설명할 때 아래 수식을 상기해보자:


$$\frac{\partial R(\theta)}{\partial \theta}=R(\theta) \hat{\Omega}=\hat{\Omega}R(\theta) \; \; \text{where} \; \; \hat{\Omega}=\left[\begin{array}{cc}
0 & -1 \\
1 & 0
\end{array}\right]   \; \; \; \; \text{(5)}$$

현재 $$\hat{\Omega} = R(\pi/2)$$로 표현만 다를 뿐 동일한 matrix를 가리키고 있다. 따라서 수식 (5)를 활용하면 교환 법칙에 의거하여 $$\mathbf{R}^\intercal$$와 $$R(\pi / 2)$$의 위치를 뒤바꿀 수 있다(i.e., $$\mathbf{R}^\intercal R(\pi / 2) = R(\pi / 2)\mathbf{R}^\intercal$$).

따라서 최종적으로 $$\mathbf{H}_1$$과 $$\mathbf{H}_2$$는 아래와 같게 된다:

$$\mathbf{H}_1 = - R(\pi / 2) \mathbf{R}^\intercal \mathbf{p}, \;\; \mathbf{H}_2 = \mathbf{R}^\intercal$$

아래 코드를 다시 살펴보면, `q`를 계산하는 부분이 $$\mathbf{R}^\intercal \mathbf{p}$$이고, 그 이후 $$- R(\pi / 2)$$가 곱해져 `*H1 << q.y(), -q.x();`가 되는 것을 볼 수 있다.

```cpp
Point2 Rot2::unrotate(const Point2& p,
    OptionalJacobian<2, 1> H1, OptionalJacobian<2, 2> H2) const {
  const Point2 q = Point2(c_ * p.x() + s_ * p.y(), -s_ * p.x() + c_ * p.y());
  if (H1) *H1 << q.y(), -q.x();
  if (H2) *H2 = transpose();
  return q;
}
```

---

## Conclusion

결국 GTSAM은 delta 값에 대한 이해를 잘 하고 있느냐가 굉장히 중요하다고 볼 수 있다.
참고로 여기서는 $$h(\cdot)$$이라고 안 쓰고 $$f(\cdot)$$라고 표현했는데, 여기서 $$f$$는 멋있는 말로 'action'이라고 부르고, 이는 $$N$$ 차원의 point를 $$(N + 1) \times (N + 1)$$ transformation matrix로 transform하는 것을 뜻하기에, measurement function $$h(\cdot)$$와 구분지어 쓰려 노력하였다. 

이제 어느 정도 `H` 값을 구하는 유도 방법에 친숙해졌으니, 첫 글에서 무시하고 지나왔던 `Pose2`와 `Pose3`를 입력으로 하는 `BetweenFactor`의 `H1`와 `H2`를 다음 글부터 유도해 보자.

---

GTSAM Tutorial 시리즈입니다.

{% include post_links_gtsam.html %}
