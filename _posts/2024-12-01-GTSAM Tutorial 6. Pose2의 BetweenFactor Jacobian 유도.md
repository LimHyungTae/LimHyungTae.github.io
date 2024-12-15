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

2D 상의 Lie Group인 SE(2)와 대응되는 vector $$\boldsymbol{\xi} \in \mathbb{R}^3$$를 $$\boldsymbol{\delta} = [\delta\mathbf{t}; \delta\theta]^\intercal$$를 통해 update하는 식을 먼저 유도해보자.
$$\left[\begin{array}{cc}
\mathbf{R} & \mathbf{t} \\
\mathbf{0}_{1\times2} & 1
\end{array}\right]\left[\begin{array}{cc}
\mathrm{Rot}(\delta\theta) & {\delta\mathbf{t}} \\
\mathbf{0}_{1\times2} & 1
\end{array}\right]$$를 통해서, translation 부분은 $$\mathbf{t} + \mathrm{Rot}(\theta) \delta\mathbf{t}$$, rotation matrix는 $$\mathbf{R}\mathrm{Rot}(\delta\theta) = \mathrm{Rot}(\theta)\mathrm{Rot}(\delta\theta)$$이므로, rotation matrix는에 대응되는 각도 값은 $$\theta + \delta \theta$$가 된다. 따라서, update function $$\boldsymbol{\xi} \oplus \boldsymbol{\delta}$$는 다음과 같이 정의된다:

$$\boldsymbol{\xi} \oplus \boldsymbol{\delta} =  
\left[\begin{array}{c}
\mathbf{t} + \mathrm{Rot}(\theta) \delta\mathbf{t} \\
\theta + \delta \theta
\end{array}\right] \in \mathbb{R}^3 \; \; \; \; \text{(1)}$$

2D차원에서의 rotation은 완전 러키비키하게 단순한 yaw 각의 덧셈으로 SE(2)의 회전을 표현할 수 있기 때문에, (1)과 같이 단순하게 표기할 수 있다 (3차원에서는 어림도 없다!). 앞으로는 편의 상 $$\mathbf{0}_{1\times2}$$는 그냥 $$\mathbf{0}$$라고 적겠다.

### Step 2. Measurement Function $$h(\cdot)$$ 정의

Lie Group 상에서 두 pose간의 뺄셈과 대응되는 개념은 위의 코드에서 inverse된 `p1`의 transformation matrix와 $$\left(\mathbf{T}^{w}_1\right)^{-1}$$ `p2`의 transformation matrix $$\mathbf{T}^{w}_2$$를 곱하는 것이다. 따라서 아래 수식을 전개하면:

$$\left(\mathbf{T}^{w}_1\right)^{-1} \mathbf{T}^{w}_2 =
\left[\begin{array}{cc}
\mathbf{R}_1 & \mathbf{t}_1 \\
\mathbf{0} & 1
\end{array}\right]^{-1}\left[\begin{array}{cc}
\mathbf{R}_2 & \mathbf{t}_2 \\
\mathbf{0} & 1
\end{array}\right] \\ = 
\left[\begin{array}{cc}
\mathbf{R}^{\intercal}_1 & -\mathbf{R}^{\intercal}_1\mathbf{t}_1 \\
\mathbf{0} & 1
\end{array}\right]\left[\begin{array}{cc}
\mathbf{R}_2 & \mathbf{t}_2 \\
\mathbf{0} & 1
\end{array}\right] = 
\left[\begin{array}{cc}
\mathbf{R}^{\intercal}_1\mathbf{R}_2 & \mathbf{R}^{\intercal}_1(\mathbf{t}_2 - \mathbf{t}_1) \\
\mathbf{0} & 1
\end{array}\right] \; \; \; \; \text{(2)}$$

따라서 두 pose의 차이에 대한 함수를 vector화 해서 나타내면 

$$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) = 
\left[\begin{array}{c}
\mathrm{Rot}(-\theta_1)(\mathbf{t}_2 - \mathbf{t}_1) \\
\theta_2 - \theta_1
\end{array}\right]\; \; \; \; \text{(3)}$$

로 표현할 수 있다 ($$\mathbf{R}^{\intercal}_1$$는 2D rotation에서 $$\mathrm{Rot}(-\theta_1)$$로 간소화해서 표현할 수 있고, $$\mathbf{R}^{\intercal}_1\mathbf{R}_2$$를 표현하는 rotation angle이 $$\theta_2 - \theta_1$$이기 때문). 

### Step 3. $$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) \oplus \boldsymbol{\delta}$$와 $$h(\boldsymbol{\xi}_1 \oplus \boldsymbol{\delta}_1, \boldsymbol{\xi}_2 \oplus \boldsymbol{\delta}_2)$$ 전개하기 

자, 다시 복습을 해보자:


---

![](/img/gtsam_solving.png)

(계속 remind되는 스크린샷...하지만 이만큼 잘 설명되어 있는 글이 없다.)

---

우리가 이제 해야할 것은? 그림 상에 있는 $$h(\boldsymbol{\xi}_1 \oplus \boldsymbol{\delta}_1, \boldsymbol{\xi}_2 \oplus \boldsymbol{\delta}_2) = h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) \oplus \boldsymbol{\delta}$$를 전개해서 $$\boldsymbol{\delta}$$를 $$\boldsymbol{\delta}_1$$와 $$\boldsymbol{\delta}_2$$의 선형 조합으로 표현할 수 있는 $$\mathbf{H}_1$$과 $$\mathbf{H}_2$$를 전개해서 구하면 된다.


**Step 3-1. $$h(\boldsymbol{\xi}_1 \oplus \boldsymbol{\delta}_1, \boldsymbol{\xi}_2 \oplus \boldsymbol{\delta}_2)$$ 전개하기**

$$h(\boldsymbol{\xi}_1 \oplus \boldsymbol{\delta}_1, \boldsymbol{\xi}_2 + \boldsymbol{\delta}_2)$$는
수식 (1)을 통해 업데이트된 값들을 (3)의 값을 대입하기만 하면 된다. 즉, $$\mathbf{t}_1 \leftarrow \mathbf{t}_1 + \mathrm{Rot}(\theta_1) \delta\mathbf{t}_1$$, $$\theta_1 \leftarrow \theta_1 + \delta \theta_1$$, $$\mathbf{t}_2 \leftarrow \mathbf{t}_2 + \mathrm{Rot}(\theta_2) \delta\mathbf{t}_2$$, $$\theta_2 \leftarrow \theta_2 + \delta \theta_2$$를 3에 대입하면 아래와 같이 되고:

$$h(\boldsymbol{\xi}_1 \oplus \boldsymbol{\delta}_1, \boldsymbol{\xi}_2 \oplus \boldsymbol{\delta}_2) = 
\left[\begin{array}{c}
\mathrm{Rot}\left(-\theta_1-\delta \theta_1\right)\left(\mathbf{t}_2+\mathbf{R}_2 \delta \mathbf{t}_2-\mathbf{t}_1-\mathbf{R}_1 \delta t_1\right) \\
\theta_2+\delta \theta_2-\theta_1-\delta \theta_1
\end{array}\right]\; \; \; \; \text{(4)}$$

미소 각도에 대한 rotation는 $$\mathrm{Rot}\left(\delta_\theta\right) \simeq\left[\begin{array}{cc}
1 & -\delta_\theta \\
\delta_\theta & 1
\end{array}\right]=\mathbf{I}_{2 \times 2}+\hat{\Omega} \delta_\theta$$라 표현 할 수 있다는 것을 이미 배웠으므로, 이를 풀어서 쓰면 아래와 같아진다:

$$h(\boldsymbol{\xi}_1 \oplus \boldsymbol{\delta}_1, \boldsymbol{\xi}_2 \oplus \boldsymbol{\delta}_2) = 
\left[\begin{array}{c}
\left(\mathbf{R}_1^\intercal - \mathbf{R}_1^\intercal \hat{\Omega} \delta_\theta \right)\left(\mathbf{t}_2+\mathbf{R}_2 \delta \mathbf{t}_2-\mathbf{t}_1-\mathbf{R}_1 \delta t_1\right) \\
\theta_2+\delta \theta_2-\theta_1-\delta \theta_1
\end{array}\right]\; \; \; \; \text{(5)}$$

이해를 돕기 위해 부연설명하자면, $$\mathrm{Rot}\left(-\theta_1-\delta \theta_1 \right) = \mathrm{Rot}\left(-\theta_1\right) \mathrm{Rot}\left(-\delta \theta_1 \right) = \mathbf{R}_1^\intercal \left( \mathbf{I}_{2 \times 2} - \hat{\Omega} \delta_\theta \right) = \mathbf{R}_1^\intercal - \mathbf{R}_1^\intercal \hat{\Omega} \delta_\theta$$.

**Step 3-2. $$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) \oplus \boldsymbol{\delta}$$ 전개하기**


$$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) \oplus \boldsymbol{\delta}$$ 또한 수식 (2)에 $$\boldsymbol{\delta}$$에 해당되는 transformation matrix를 곱해준 후:

$$\left[\begin{array}{cc}
\mathbf{R}^{\intercal}_1\mathbf{R}_2 & \mathbf{R}^{\intercal}_1(\mathbf{t}_2 - \mathbf{t}_1) \\
\mathbf{0} & 1
\end{array}\right]
\left[\begin{array}{cc}
\mathrm{Rot}(\delta \theta) & \delta \mathbf{t} \\
\mathbf{0} & 1
\end{array}\right] $$

다시 vector 꼴로 되돌리면 된다:

$$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) \oplus \boldsymbol{\delta} = 
\left[\begin{array}{c}
\mathbf{R}_1^\intercal \mathbf{R}_2 \delta \mathbf{t} + \mathbf{R}_1^T\left(\mathbf{t}_2-\mathbf{t}_1\right) \\
\theta_2-\theta_1+\delta \theta
\end{array}\right].\; \; \; \; \text{(6)}$$


### Step 4. 수식 전개해서 `H1`, `H2`에 대응되는 값 유도

따라서 최종적으로, 수식 (5)와 수식 (6)을 같다고 놓고 풀면 우리가 원하는 `H1`과 `H2`를 구할 수 있다. 미소 translation $$\delta \mathbf{t}$$는 아래의 수식을 전개하면 되고: 

$$\mathbf{R}_1^\intercal \left(\mathbf{t}_2+\mathbf{R}_2 \delta \mathbf{t}_2-\mathbf{t}_1-\mathbf{R}_1 \delta t_1\right)
- \mathbf{R}_1^\intercal \hat{\Omega} \delta_\theta \left(\mathbf{t}_2 - \mathbf{t}_1\right) \\
= \mathbf{R}_1^\intercal \mathbf{R}_2 \delta \mathbf{t} + \mathbf{R}_1^T\left(\mathbf{t}_2-\mathbf{t}_1\right)
$$

(위에서도 앞선 글에서와 마찬가지로 미소 값들이 두 번 곱해지는 term들은 무시되었다) 미소 rotation $$\delta \theta$$에 대한 표현식은: 

$$ \theta_2+\delta \theta_2-\theta_1-\delta \theta_1 = \theta_2-\theta_1+\delta \theta $$

을 풀면 된다. 따라서 이 두 식을 푼 후, 최종적으로 $$\boldsymbol{\delta} = \mathbf{H}_1 \boldsymbol{\delta}_1 + \mathbf{H}_2 \boldsymbol{\delta}_2$$로 두고 수식을 정리하면 아래와 같이 최종적으로 유도할 수 있다:

$$\boldsymbol{\delta}=\left[\begin{array}{l}
\delta \mathbf{t} \\
\delta \theta
\end{array}\right]=-\left[\begin{array}{cc}
\mathbf{R}_2^\intercal \mathbf{R}_1 & \hat{\Omega} \mathbf{R}_2^\intercal\left(\mathbf{t}_1-\mathbf{t}_2\right) \\
\mathbf{0} & 1
\end{array}\right]\left[\begin{array}{l}
\delta \mathbf{t}_1 \\
\delta \theta_1
\end{array}\right]+\left[\begin{array}{cc}
\mathbf{I}_{2 \times 2} & 0 \\
\mathbf{0} & 1
\end{array}\right]\left[\begin{array}{l}
\delta \mathbf{t}_2 \\
\delta \theta_2
\end{array}\right]\; \; \; \; \text{(7)}$$

---

## Advanced: Adjoint Map and Beauty of Mathematics

사실 위와 같이 `H1`과 `H2`를 구하면 끝나는데, `H1`이 무엇을 의미하는지 살펴보고 이 글을 마치고자 한다. 
먼저 한 가지 why를 묻지 말고 받아들여야 할 것이 있는데, `Pose2.cpp`나 ``Pose3.cpp`에는 아래와 같이 `AdjointMap`이라는 함수가 구현되어 있다:

```cpp
// See https://github.com/borglab/gtsam/blob/3af5360ad397422023160604de99d0de447b0a88/gtsam/geometry/Pose2.cpp#L127

// Calculate Adjoint map
// Ad_pose is 3*3 matrix that when applied to twist xi, returns Ad_pose(xi)
Matrix3 Pose2::AdjointMap() const {
  double c = r_.c(), s = r_.s(), x = t_.x(), y = t_.y();
  Matrix3 rvalue;
  rvalue <<
      c,  -s,   y,
      s,   c,  -x,
      0.0, 0.0, 1.0;
  return rvalue;
}
```

그리고 다시 수식 (7)을 살펴보자. 수식 (7)을 다시 보니, `H1` matrix는 

```cpp
// See https://github.com/borglab/gtsam/blob/3af5360ad397422023160604de99d0de447b0a88/gtsam/base/Lie.h#L63C3-L69C4

Class between(const Class& g, ChartJacobian H1,
    ChartJacobian H2 = {}) const {
  Class result = derived().inverse() * g;
  if (H1) *H1 = - result.inverse().AdjointMap();
  if (H2) *H2 = Eigen::Matrix<double, N, N>::Identity();
  return result;
}
```

---

## Conclusion

대망의 Pose2의 BetweenFactor의 `H1`과 `H2`를 구하는 방법에 알아 보았다. 다시금 강조하지만, 단순히 BetweenFactor를 사용하고자 하는 입장에서는 전혀 알 필요가 없는 부분이다. 다만, SLAM의 optimization을 굉장히 low-level로 다뤄야 할 일이 있을 때 이런 과정을 모르면 GTSAM이 어떻게 동작하는지 절대 이해할 수 없기 때문에, 저 `H` matrix를 어떻게 유도하는 지 정리해보았다.

최대한 수학적 엄밀성을 따지기 보다는 이해가 쉽게 풀어 써보았는데, 모쪼록 읽는 분께 도움이 되었으면 한다.
만약 이해가 잘 안되는 부분이 있다면 앞선 글에서 설명했던 skew-symmetric matrix 부분과 unary factor를 다시 한 번 차근차근 읽어본 후 재도전(?)하는 것을 추천한다. 

---

GTSAM Tutorial 시리즈입니다.

{% include post_links_gtsam.html %}