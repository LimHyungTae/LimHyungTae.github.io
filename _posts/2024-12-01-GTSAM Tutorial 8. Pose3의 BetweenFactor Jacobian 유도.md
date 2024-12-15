---
layout: post
title: GTSAM Tutorial 8. Pose3의 BetweenFactor Jacobian 유도
subtitle: Understanding H matrices of Pose3 as an exercise
tags: [Jacobian, GTSAM]
comments: true
---

## Pose3의 Jacobian 구하기

이제 마지막으로 Pose3의 `H1`과 `H2`를 구해보자. Pose2때와 마찬가지로, 아래 세 단계를 따르면 쉽게 수식을 유도해 볼 수 있다. 다만 연습 문제 삼아 자세한 과정은 생략하고 어떻게 푸는지 hint만 적어두도록 하겠다~~교수되면 homework로 내기 위한 빅픽쳐~~.

### Step 1. Update Function 정의

Translation-rotation 순으로 적었던 2D와는 다르게 GTSAM에서 3D pose를 vector화해서 표현할 때는 rotation-translation 순으로 적는 것에 유의하자. 따라서, vector꼴로 표현된 rotation/translation 변화량을 각각 $$\boldsymbol{w} \in \mathbb{R}^3$$, $$\boldsymbol{v} \in \mathbb{R}^3$$라 하면, $$\boldsymbol{\delta} = [\boldsymbol{w}; \boldsymbol{v}]^\intercal \in \mathbb{R}^6$$로 표현이 된다. 그리고 2D 상에서 $$\mathrm{Rot}(\delta\theta) \simeq \mathbf{I} + \hat{\Omega} \delta \theta$$라고 표현했던 것이 3차원에서는 $$\mathbf{I} + [\boldsymbol{w}]_\times$$와 대응되기 때문에(이해가 되지 않는다면 Skew Symmetric matrix 관련 글을 다시 읽어보자), 이를 풀어서 쓰면 아래와 같고:

$$\left[\begin{array}{cc}
\mathbf{R} & \mathbf{t} \\
\mathbf{0} & 1
\end{array}\right]\left[\begin{array}{cc}
\mathbf{I} + [\boldsymbol{w}]_\times & \boldsymbol{v} \\
\mathbf{0} & 1
\end{array}\right]$$

따라서, update function $$\boldsymbol{\xi} \oplus \boldsymbol{\delta}$$는 다음과 같이 정의된다:

$$\boldsymbol{\xi} \oplus \boldsymbol{\delta} =  
\left[\begin{array}{c}
\mathrm{Log}\left( \mathbf{R} \left(\mathbf{I} + [\boldsymbol{w}]_\times\right) \right) \\ 
\mathbf{t} + \mathbf{R} \boldsymbol{v} 
\end{array}\right] \in \mathbb{R}^6 \; \; \; \; \text{(1)}$$

여기서 $$\mathrm{Log}\left( \cdot \right)$$는 우리가 2차원에서 $$\mathrm{Rot}(\theta)$$을 $$\theta$$로 간단히 표현했던 것 처럼, 3차원 rotation matrix을 3차원 rotation vector로 변환해주는 함수라 보면 된다.
우리가 사용하고자 하는 성질은 $$\mathrm{Log}\left(A\right) = \mathrm{Log}\left(B\right)$$이면 $$A=B$$라는 것이기 때문에, 저 $$\mathrm{Log}\left( \cdot \right)$$ 함수가 어떻게 동작하는지는 수식 전개 시 알 필요는 없다.

### Step 2. Measurement Function $$h(\cdot)$$ 정의

2D의 transformation matrix와 같이 relative pose에 대응하는 measurement function은 아래와 같고:

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
\mathrm{Log}\left(\mathbf{R}^{\intercal}_1 \mathbf{R}_2) \right) \\
\mathbf{R}^{\intercal}_1(\mathbf{t}_2 - \mathbf{t}_1) 
\end{array}\right] \in \mathbb{R}^6 \; \; \; \; \text{(3)}$$

로 표현할 수 있다. 2D에서 $$\theta_2 - \theta_1$$로 뺄셈으로 손쉽게 표현할 수 있었던 rotation이 차원이 증가해서 $$\mathrm{Log}\left(\mathbf{R}^{\intercal}_1 \mathbf{R}_2)$$라는 조금 복잡한 형태가 되었을 뿐, rotation의 차이를 표현하고자 하는 기저 원리는 같다.

### Step 3. $$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) \oplus \boldsymbol{\delta}$$와 $$h(\boldsymbol{\xi}_1 \oplus \boldsymbol{\delta}_1, \boldsymbol{\xi}_2 \oplus \boldsymbol{\delta}_2)$$ 전개하기 

자, 여기서부터는 스스로 한 번 해보는 것으로 하자!

**Step 3-1. $$h(\boldsymbol{\xi}_1 \oplus \boldsymbol{\delta}_1, \boldsymbol{\xi}_2 \oplus \boldsymbol{\delta}_2)$$ 전개하기**

**Step 3-2. $$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) \oplus \boldsymbol{\delta}$$ 전개하기**

전개를 하는 것 자체에는 어려움이 없을 것이다.


### Step 4. 수식 전개해서 `H1`, `H2`에 대응되는 값 유도

이제 $$h(\boldsymbol{\xi}_1 \oplus \boldsymbol{\delta}_1, \boldsymbol{\xi}_2 \oplus \boldsymbol{\delta}_2)$$와 $$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) \oplus \boldsymbol{\delta}$$의 translation/rotation 요소가 각각 같다고 하고 풀면 된다. 

**Hints**

* 미소 변화량 두개가 곱해지면 해당 term이 무시 가능해진다는 것을 활용하자. 즉, $$[\boldsymbol{w}_1]_\times$$과 $$\boldsymbol{v}_1$$이나 $$\boldsymbol{v}_2$$가 곱해지면 해당 term은 크기가 다른 term보다 월등히 작아지므로 무시 가능하다.
* 

$$\mathbf{R}_1^\intercal \left(\mathbf{t}_2+\mathbf{R}_2 \delta \mathbf{t}_2-\mathbf{t}_1-\mathbf{R}_1 \delta \mathbf{t}_1\right)
- \mathbf{R}_1^\intercal \hat{\Omega} \delta \theta_1 \left(\mathbf{t}_2 - \mathbf{t}_1\right) \\
= \mathbf{R}_1^\intercal \mathbf{R}_2 \delta \mathbf{t} + \mathbf{R}_1^T\left(\mathbf{t}_2-\mathbf{t}_1\right)
$$

(위에서도 앞선 글에서와 마찬가지로 미소 값들이 두 번 곱해지는 term들은 무시되었다) 미소 rotation $$\delta \theta$$에 대한 표현식은: 

$$ \theta_2+\delta \theta_2-\theta_1-\delta \theta_1 = \theta_2-\theta_1+\delta \theta $$

을 풀면 된다. 따라서 이 두 식을 푼 후, 최종적으로 $$\boldsymbol{\delta} = \mathbf{H}_1 \boldsymbol{\delta}_1 + \mathbf{H}_2 \boldsymbol{\delta}_2$$로 두고 수식을 정리하면 아래와 같이 최종적으로 유도할 수 있다:

$$\boldsymbol{\delta}=\left[\begin{array}{l}
\delta \mathbf{t} \\
\delta \theta
\end{array}\right]=-\left[\begin{array}{cc}
\mathbf{R}_2^\intercal \mathbf{R}_1 & -\hat{\Omega} \mathbf{R}_2^\intercal\left(\mathbf{t}_1-\mathbf{t}_2\right) \\
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
먼저 왜?를 묻지 말고 받아들여야 할 것이 있는데, `Pose2.cpp`나 `Pose3.cpp`에는 아래와 같이 `AdjointMap`이라는 함수가 구현되어 있다 (adjoint map 자체에 대해서는 [다음 글](https://limhyungtae.github.io/2024-12-01-GTSAM-Tutorial-7.-Adjoint-Map-%EC%89%BD%EA%B2%8C-%EC%9D%B4%ED%95%B4%ED%95%98%EA%B8%B0/)에서 살펴보자):

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

그리고 다시 수식 (7)을 살펴보자. 수식 (7)을 다시 보니, `H1` matrix는 $$h(\boldsymbol{\xi}_2, \boldsymbol{\xi}_1)$$의(i.e., $$\left[\begin{array}{cc}
\mathbf{R}^{\intercal}_2\mathbf{R}_1 & \mathbf{R}^{\intercal}_2(\mathbf{t}_1 - \mathbf{t}_2) \\
\mathbf{0} & 1
\end{array}\right]$$와 대응됨. 이 경우 위의 `t_`가 $$\mathbf{R}^{\intercal}_2(\mathbf{t}_1 - \mathbf{t}_2)$$이 되고, 코드 상에서 $$(x, y)$$를 $$(y, -x)$$로 변경하는 것이 $$-\hat{\Omega}$$를 곱해주는 것을 의미함) AdjointMap() 함수라는 것을 알 수 있다.

그래서 실제로 `gtsam/base/Lie.h` 코드 내에 `between`을 계산하는 코드를 살펴보면(아래에서 `derived()`가 `p1`, `g`가 `p2`로 각각 대응된다고 생각하면 된다), `H1`의 값에 $$\left(\mathbf{T}^{w}_1\right)^{-1} \mathbf{T}^{w}_2$$의 inverse를 한 후(원래는 $$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2)$$와 대응되던 transformation matrix가 `inverse()` 함수를 통해 해당 transformation matrix이 $$h(\boldsymbol{\xi}_2, \boldsymbol{\xi}_1)$$와 대응됨), `AdjointMap()` 함수를 통해 `H1`를 손쉽게 구하는 것을 볼 수 있다. 그리고 이는 3D에서도 마찬가지로 적용이 가능하기 때문에, 차원에 관계 없이 `H1` matrix와 `H2` matrix를 손쉽게 구하는 것이 가능해지게 된다: 

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

깔끔하고 아름답다!

---

## Conclusion

대망의 Pose2의 BetweenFactor의 `H1`과 `H2`를 구하는 방법에 알아 보았다. 다시금 강조하지만, 단순히 BetweenFactor를 사용하고자 하는 입장에서는 전혀 알 필요가 없는 부분이다. 다만, SLAM의 optimization을 굉장히 low-level로 다뤄야 할 일이 있을 때 이런 과정을 모르면 GTSAM이 어떻게 동작하는지 절대 이해할 수 없기 때문에, 저 `H` matrix를 어떻게 유도하는 지 정리해보았다.

최대한 수학적 엄밀성을 따지기 보다는 이해가 쉽게 풀어 써보았는데, 모쪼록 읽는 분께 도움이 되었으면 한다.
만약 이해가 잘 안되는 부분이 있다면 앞선 글에서 설명했던 skew-symmetric matrix 부분과 unary factor를 다시 한 번 차근차근 읽어본 후 재도전(?)하는 것을 추천한다. 

---

GTSAM Tutorial 시리즈입니다.

{% include post_links_gtsam.html %}