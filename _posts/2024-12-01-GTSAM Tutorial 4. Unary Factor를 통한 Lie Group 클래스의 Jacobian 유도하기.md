---
layout: post
title: GTSAM Tutorial 4. Unary Factor를 통한 Lie Group 클래스의 Jacobian 유도하기
subtitle: Understanding H matrices in GTSAM
tags: [Jacobian, GTSAM]
comments: true
---

## Unary Factor를 통한 GTSAM 내 Lie Group 클래스의 Jacobian 유도하기

자, 여기까지 읽었으면 Jacobian 자체는 잘 이해했을 것이다.
하지만 GTSAM에서 `H` 행렬을 구할 때 많은 사람들이 혼란을 겪는 부분이 있다. 사실, 이 튜토리얼은 바로 이 점을 설명하기 위해 작성되었다. 아래 예시를 통해 이를 자세히 살펴보자.

```cpp
class UnaryFactor: public NoiseModelFactor1<Pose2> {
  double mx_, my_; ///< X and Y measurements

public:
  UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model):
    NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

  Vector evaluateError(const Pose2& q,
                       boost::optional<Matrix&> H = boost::none) const
  {
    const Rot2& R = q.rotation();
    if (H) (*H) = (gtsam::Matrix(2, 3) <<
            R.c(), -R.s(), 0.0,
            R.s(), R.c(), 0.0).finished();
    return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
  }
};
```

여기서 `H` 매트릭스에 왜 rotation matrix $$R$$이 할당되어 있을까? 

## `H`가 어떻게 구해지는 걸까?

리턴되는 measurement function을 각각 $$f_1=t_x - m_x$$(`q.x() - mx_`에 대응되는 부분), 
$$f_2=t_y - m_y$$(`q.y() - my_`에 대응되는 부분)라 정의해보자.
그러면 유도되는 미분값은 $$\frac{df_1}{dt_{x}}=1, \frac{df_1}{dt_{y}}=0, \frac{df_2}{dt_{x}}=0, \frac{df_2}{t_{y}}=1$$로
매트릭스의 앞의 $$2\times2$$ 부분이 $$\mathbf{I}_{2\times2}$$가 되어야 할 것 같은데, translation의 Jacobian에 해당하는 부분이 $$\mathbf{R}$$로 되어 있는 것을 볼 수 있다(오해를 방지하기 위해 미리 말하자면, 사실 이건 완전히 틀린 접근 방식이다).
그렇다면 이 `H`는 어떻게 유도되는 것일까?

## Prerequisite: Retract 엿보기


여기서 `H`가 우리가 생각한 것과 다른 형태로 유도되는 것은 바로 우리가 optimize하고자 하는 변수가 Lie Group에 속하는 값이기 때문이다(`H`의 의미는 글의 마지막에 최종적으로 언급하겠다).
Lie Group의 세계에서는 pose는 transformation matrix 꼴로 표현되어 있다. 
그리고 optimization 시에는 (i) pose를 업데이트할 미소 pose를 vector의 형태로 취득한 후 이를 (ii) transformation matrix의 꼴로 되돌려서 pose의 우측에 곱해주어 pose를 업데이트한다는 것을 기억하자.
즉, pose $$\mathbf{T}$$를 업데이트하는 것을

$$\mathbf{T} \leftarrow \mathbf{T}\Delta\mathbf{T}\;\;\;\;(1)$$

와 같이 표현할 수 있다. 
수식 (1)을 vector의 형태로 표현(이를 parameterize라 부른다)해보면 미소 vector $$\boldsymbol{\delta}$$에 의해 vector꼴로 표현된 pose $$\boldsymbol{\xi}$$가 증분되는 것을 아래와 같이 표현할 수 있었다:

$$\boldsymbol{\xi} \leftarrow \boldsymbol{\xi} \oplus \boldsymbol{\delta}\;\;\;\;(2)$$

Lie Group의 세계에서는 우리가 최적화하고자 하는 값의 변화량이 수식 (1)과 같은 비선형 함수(즉, 변수들의 단순한 덧셈으로 표현되지 않는 함수)로 나타난다.
특히, 최적화를 수행할 때 관측값과 이를 모델링한 함수인 measurement function 역시 비선형 함수로 표현되기 때문에, 이를 그대로 사용하면 계산 과정이 복잡해지고 최적화가 어려워진다.
따라서 최적화를 보다 효율적으로 수행하려면 이러한 수식을 선형화하는 과정이 필요하다.

Unary factor의 경우에는, [두 pose를 다루었던 between factor](https://limhyungtae.github.io/2024-12-01-GTSAM-Tutorial-1.-SLAM%EC%9D%84-%EC%9C%84%ED%95%9C-Between-Factor-%EC%89%BD%EA%B2%8C-%EC%9D%B4%ED%95%B4%ED%95%98%EA%B8%B0/)와 다르게 업데이트하고자 하는 변수가 하나이므로,
이는 아래와 같이 수식으로 표현할 수 있으며:

$$h(\boldsymbol{\xi} \oplus \boldsymbol{\delta}) \simeq h(\boldsymbol{\xi}) + \mathbf{H}\boldsymbol{\delta}\;\;\;\;(3)$$

여기서 unary factor의 measurement function $$h(\cdot)$$은 Lie Group pose의 translation 값만 return해주는 함수이다. 
수식 (3)과 같이 선형화를 해주어야, 처음 글에서 보았듯이, 아래의 objective function을 $$||\mathbf{A}\mathbf{x} - \mathbf{b}||_2$$꼴로 표현할 수 있게 된다 (첫 글에서 본 아래의 스크린샷을 다시 살펴보자):

---

![](/img/gtsam_solving.png)

---

이 unary factor의 경우에는 수식이 아래와 같아진다:

$$\frac{1}{2}||h(\boldsymbol{\xi} \oplus \boldsymbol{\delta}) - \mathbf{m}||_2 \simeq \frac{1}{2}||h(\boldsymbol{\xi}) + \mathbf{H}\boldsymbol{\delta} - \mathbf{m}||_2 = \frac{1}{2}||\mathbf{H}\boldsymbol{\delta} - \mathbf{b}||_2 $$

위와 같이 되어야 이제 각 iteration 별 최적의 $$\boldsymbol{\delta}$$를 구할 수 있게 되기 때문이다. 여기서 $$\mathbf{m}$$은 코드에서 `mx_`와 `my_`로 표현되어 있는 measurement에 대응되는 2D point이다.
즉, 선형화를 하기 위해서 필요한 $$\boldsymbol{\delta}$$에 대한 표현식이 바로 우리가 구하고자 하는 `H`인 것이다.
 
---

## 수식 유도 과정

자, 이제 어느 정도 이해를 했으니 직접 `H` 값을 유도해 보자.
수식 (2)의 변수들을 각각 $$\boldsymbol{\xi} = [t_x, t_y, \theta]^{\intercal}$$, $$\boldsymbol{\delta} = [\delta_x, \delta_y, \delta_\theta]^{\intercal}$$라 표현해보자.
그러면 수식 (1)을 활용하면 SE(2) 상에서의 증분은 아래와 같이 표현된다:

$$\left[\begin{array}{ccc}
\cos \theta & -\sin \theta & t_x \\
\sin \theta & \cos \theta & t_y \\
0 & 0 & 1
\end{array}\right]\left[\begin{array}{ccc}
\cos \delta_{\theta} & -\sin \delta_{\theta} & \delta_x \\
\sin \delta_{\theta} & \cos \delta_{\theta} & \delta_x \\
0 & 0 & 1
\end{array}\right]. \; \; \; \; \text{(4)}$$

수식 (4)에서 $$\delta_{\theta}$$는 굉장히 작은 각도 값이므로, $$\delta_{\theta} \simeq 0$$라고 가정할 수 있다(이를 small angle approximation이라 부름).
이를 통해 $$\cos \delta_{\theta} \simeq 1$$, $$\sin \delta_{\theta} \simeq \delta_{\theta}$$로 근사가 가능하므로,
(4)는 아래와 같이 간소화되어 표현이 가능하다:

$$\left[\begin{array}{ccc}
\cos \theta & -\sin \theta & t_x \\
\sin \theta & \cos \theta & t_y \\
0 & 0 & 1
\end{array}\right]\left[\begin{array}{ccc}
1 & -\delta_{\theta} & \delta_x \\
\delta_{\theta} & 1 & \delta_y \\
0 & 0 & 1
\end{array}\right].\; \; \; \; \text{(5)}$$

따라서 위의 수식을 전개한 후의 translation에 해당하는 값이 $$h(\boldsymbol{\xi} \oplus \boldsymbol{\delta})$$와 대응되는 값이다.
편의를 위해 $$\mathbf{t} = [t_x, \, t_y]^\intercal$$, $$\boldsymbol{\delta}_{\mathbf{t}} = [\delta_x, \, \delta_y]^\intercal$$라 표현하면, 수식 (5)는 아래와 같이 block operation으로 표현되고:

$$\left[\begin{array}{cc}
R(\theta)  & \mathbf{t} \\
\mathbf{0}_{1\times 2} & 1
\end{array}\right]\left[\begin{array}{cc}
 \mathbf{I}_{2 \times 2} + \hat{\Omega}\delta_{\theta} & \boldsymbol{\delta}_{\mathbf{t}} \\
\mathbf{0}_{1\times 2}  & 1
\end{array}\right],\; \; \; \; \text{(6)}$$

따라서 $$h(\boldsymbol{\xi} \oplus \boldsymbol{\delta})$$는 아래와 같이 표현된다:

$$h(\boldsymbol{\xi} \oplus \boldsymbol{\delta}) = \mathbf{t} + R(\theta)\boldsymbol{\delta}_{\mathbf{t}} = h(\boldsymbol{\xi}) + R(\theta)\boldsymbol{\delta}_{\mathbf{t}}\; \; \; \; \text{(7)}$$

우리가 최종적으로 구하고자 하는 $$\mathbf{H}$$는 $$\boldsymbol{\delta}$$($$\boldsymbol{\delta}_{\mathbf{t}}$$ 아니고 $$\boldsymbol{\delta}$$임 주의) 앞에 곱해지는 matrix이므로,
수식 (7)을 기반으로 하여 $$\mathbf{H} = [R(\theta) \,\; \mathbf{0}_{2\times1}] \in \mathbb{R}^{2\times3}$$로 나타낼 수 있다 (수식 (7)내에 $$\delta_{\theta}$$가 존재하지 않으므로, $$\delta_{\theta}$$에 대한 partial derivative, i.e., Jacobian에서 $$\delta_{\theta}$$에 대한 column,는 모두 다 0이 된다).

## Conclusion

따라서, GTSAM에서 어떤 factor를 새로 정의할 때 필요한 Jacobian matrix `H`는 단순히 $$\left[\begin{array}{ccc}
\frac{df_1}{dt_x} & \frac{df_1}{dt_y} & \frac{df_1}{d\theta} \\
\frac{df_2}{dt_x} & \frac{df_2}{dt_y} & \frac{df_2}{d\theta} \\
\end{array}\right]$$가 아닌, 미소 증분 vector인 $$\boldsymbol{\delta}$$의 앞에 곱해지는 계수 행렬임을 주의하자. 그래서 이전 글들을 보면 Jacobian 자체를 가리킬 때는 $$\mathbf{J}$$라고 칭한 반면, 이 $$\boldsymbol{\delta}$$를 위한 jacobian은 `H`라고 칭해 이를 구분하고자 하였다.

---

GTSAM Tutorial 시리즈입니다.

{% include post_links_gtsam.html %}