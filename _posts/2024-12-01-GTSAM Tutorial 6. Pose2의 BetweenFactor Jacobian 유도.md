---
layout: post
title: GTSAM Tutorial 6. Pose2의 BetweenFactor Jacobian 유도
subtitle: Understanding H matrices in GTSAM
tags: [Jacobian, GTSAM]
comments: true
---

## Pose2의 Jacobian 구하기

자, 그럼 Jacobian에 대해 많이 친숙해진 것 같으니 대망의, `Pose2` 상에서 아래 코드의 `H1`과 `H2`를 구해보자.

```cpp
 /// evaluate error, returns vector of errors size of tangent space
Vector evaluateError(const T& p1, const T& p2,
  OptionalMatrixType H1, OptionalMatrixType H2) const override {
  T hx = traits<T>::Between(p1, p2, H1, H2); // h(x)
  // manifold equivalent of h(x)-z -> log(z,h(x))
#ifdef GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR
  typename traits<T>::ChartJacobian::Jacobian Hlocal;
  Vector rval = traits<T>::Local(measured_, hx, OptionalNone, (H1 || H2) ? &Hlocal : 0);
  if (H1) *H1 = Hlocal * (*H1);
  if (H2) *H2 = Hlocal * (*H2);
  return rval;
#else
  return traits<T>::Local(measured_, hx);
#endif
}
```

Unary factor에서 유도했던 것을 좀 더 일반화하여, 세 단계로 나눠서 수식을 유도해 보자.

### Step 1. Update Function 정의

2D 상의 Lie Group인 SE(2)에서는 $$\boldsymbol{\delta}$$를 통한 update는 다음과 같이 정의된다:

$$\boldsymbol{\xi} \oplus \boldsymbol{\delta} = \left[\begin{array}{cc}
\mathbf{R} & \mathbf{t} \\
0 & 1
\end{array}\right]\left[\begin{array}{cc}
\mathrm{Rot}(\delta\theta) & {\delta\mathbf{t}} \\
0 & 1
\end{array}\right] = 
\left[\begin{array}{c}
\mathbf{t} + \mathrm{Rot}(\theta} \delta\mathbf{t} \\
\theta + \delta \theta
\end{array}\right] \in \mathbb{R}^3 \; \; \; \; \text{(1)}$$

완전 러키비키하게 2D차원에서의 rotation은 단순한 yaw 각의 덧셈으로 SE(2)의 회전을 표현할 수 있기 때문에, 위와 같이 $$(x, y, \theta)$$의 형태로 간략하게 표현할 수 있다 (3차원에서는 어림도 없다!).

### Step 2. Measurement Function $$h(\cdot)$$ 정의

Lie Group 상에서 두 pose간의 뺄셈과 대응되는 개념은 inverse() 함수를 곱하는 것이다. 따라서 아래 수식을 전개하면:

$$\left(\mathbf{T}^{w}_1\right)^{-1} \mathbf{T}^{w}_2 =
\left[\begin{array}{cc}
\mathbf{R}_1 & \mathbf{t}_1 \\
\mathbf{0}_{1\times2} & 1
\end{array}\right]^{-1}\left[\begin{array}{cc}
\mathbf{R}_2 & \mathbf{t}_2 \\
\mathbf{0}_{1\times2} & 1
\end{array}\right] = 
\left[\begin{array}{cc}
\mathbf{R}^{\intercal}_1 & -\mathbf{R}^{\intercal}_1\mathbf{t}_1 \\
\mathbf{0}_{1\times2} & 1
\end{array}\right]\left[\begin{array}{cc}
\mathbf{R}_2 & \mathbf{t}_2 \\
\mathbf{0}_{1\times2} & 1
\end{array}\right] = 
\left[\begin{array}{cc}
\mathbf{R}^{\intercal}_1\mathbf{R}_2 & \mathbf{R}^{\intercal}_1(\mathbf{t}_2 - \mathbf{t}_1) \\
\mathbf{0}_{1\times2} & 1
\end{array}\right] \; \; \; \; \text{(2)}$$

따라서 두 pose의 차이에 대한 함수는 $$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) = 
\left[\begin{array}{c}
\mathrm{Rot}(-\theta_1}(\mathbf{t}2 - \mathbf{t}_1) \\
\theta_2 - \theta_1
\end{array}\right]\; \; \; \; \text{(3)}$$

### Step 3. $$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) \oplus \boldsymbol{\delta}$$와 $$h(\boldsymbol{\xi}_1 \oplus \boldsymbol{\delta}_1, \boldsymbol{\xi}_2 \oplus \boldsymbol{\delta}_2)$$ 전개하기 

자, 다시 복습을 해보자:

![](/img/gtsam_solving.png)

우리가 이제 해야할 것은? 그림 상에 있는 $$h(\boldsymbol{\xi}_1 \oplus \boldsymbol{\delta}_1, \boldsymbol{\xi}_2 + \boldsymbol{\delta}_2) = h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) \oplus \boldsymbol{\delta}$$를 전개해서 $$\boldsymbol{\delta}$$를 $$\boldsymbol{\delta}_1$$와 $$$$\boldsymbol{\delta}_2$$의 선형 조합으로 표현할 수 있는 $$\mathbf{H}_1$$과 $$\mathbf{H}_2$$를 전개해서 구하면 된다.

**Step 3-1. $$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) \oplus \boldsymbol{\delta}$$ 전개하기**

$$h(\boldsymbol{\xi}_1 \oplus \boldsymbol{\delta}_1, \boldsymbol{\xi}_2 + \boldsymbol{\delta}_2)$$는
수식 (3)을 통해 업데이트된 값들을 (1)의 값을 대입하기만 하면 된다. 따라서 이를 계산하면 아래와 같다:

**Step 3-2. $$h(\boldsymbol{\xi}_1 \oplus \boldsymbol{\delta}_1, \boldsymbol{\xi}_2 \oplus \boldsymbol{\delta}_2)$$ 전개하기**

$$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) \oplus \boldsymbol{\delta}$$ 또한 수식 (1)의 $$\boldsymbol{\xi}$$을 $$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2)$$으로 대체해주면 된다.
따라서 이는 아래의 수식을 전개하면 된다:

### Step 4. 수식 전개해서 `H1`, `H2`에 대응되는 값 유도


---

## Conclusion



---

GTSAM Tutorial 시리즈입니다.

{% include post_links_gtsam.html %}