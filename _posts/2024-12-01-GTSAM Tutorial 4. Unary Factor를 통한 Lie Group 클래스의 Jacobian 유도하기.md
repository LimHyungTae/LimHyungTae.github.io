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

여기서 `H` 매트릭스를 살펴 보면 뭔가 의아한 부분이 있지 않은가? 

## `H`가 뭔가 이상한데요?

리턴되는 measurement function을 각각 $$f_1=t_x - m_x$$(`q.x() - mx_`에 대응되는 부분), 
$$f_2=t_y - m_y$$(`q.y() - my_`에 대응되는 부분)라 정의해보자.
그러면 분명히 유도되는 미분값은 $$\frac{df_1}{dt_{x}}=1, \frac{df_1}{dt_{y}}=0, \frac{df_2}{dt_{x}}=0, \frac{df_2}{t_{y}}=1$$로
매트릭스의 앞의 $$2\times2$$ 부분이 $$\mathbf{I}_{2\times2}$$가 되어야 할 것 같은데, translation의 Jacobian에 해당하는 부분이 $$\mathbf{R}$$로 되어 있는 것을 볼 수 있다.

## 정답: Retract 때문

여기서 아주 중요한 부분이 있는데, `H`를 구할 때에는 해당 객체가 Lie Group에 속하는지를 꼭! 확인해야 한다.
여기서 Lie Group의 세계에서는 pose를 업데이트할 미소 pose를 vector의 형태로 취득한 후, 이를 다시 SE(N)으로 되돌려서 pose를 업데이트한다는 것에 주의해야 한다.
Lie Group 세계에서는 SE(N)으로 표현된 pose 간의 변화는 $$\left(\mathbf{T}^{w}_1\right)^{-1} \mathbf{T}^{w}_2 = \mathbf{T}^{1}_2$$
와 같이 곱셈으로 표현한다는 것을 떠올려 보자. 그리고 SE(N)의 pose를 어떤 vector의 형태로 표현(이를 parameterize라 부른다)했을 때, 미소 vector에 의해 증분되는 것은 아래와 같이 표편할 수 있었다:

$$\boldsymbol{\xi}^{t+1}_i = \boldsymbol{\xi}^{t}_i \oplus \boldsymbol{\delta}_i\;\;\;\;(1)$$

그렇기 때문에 우리가 optimization하고자 하는 값의 상대 값을 표현하는 measurement function이 non-linear 함수(즉, 어떤 함수가 변수들의 덧셈만으로 표현되지 않는 꼴)이기 때문에, 
optimization을 쉽게 하기 위해서는 해당 수식을 선형화해주어야 한다.
Unary factor의 경우에는, 두 pose를 다루었던 between factor와 다르게 업데이트하고자 하는 변수가 하나이므로,
이는 아래와 같이 수식으로 표현할 수 있다:

$$h(\boldsymbol{\xi} \oplus \boldsymbol{\delta}) \simeq h(\boldsymbol{xi}) + \mathbf{H}\boldsymbol{\delta}$$

여기서 $$h(\cdot)$$은 Lie Group pose의 translation 값만 return해주는 함수이다. 
위와 같이 선형화를 해주어야, 처음 글에서 보았듯이, 아래의 objective function을 $$||\mathbf{A}\mathbf{x} - \mathbf{b}||_2$$꼴로 표현할 수 있게 된다 (첫 글에서 본 아래의 스크린샷을 다시 살펴보자):

![](img/gtsam_solving.png)

이 unary factor의 경우에는 수식이 아래와 같아진다:

$$\frac{1}{2}||h(\boldsymbol{xi} \oplus \boldsymbol{\delta}) - \mathbf{m}||_2 \simeq \frac{1}{2}||h(\boldsymbol{xi}) + \mathbf{H}\boldsymbol{\delta} - \mathbf{m}||_2 = \frac{1}{2}||\mathbf{H}\boldsymbol{\delta} - \mathbf{b}||_2 $$

위와 같이 되어야 이제 각 iteration 별 최적의 $$\delta$$를 구할 수 있게 되기 때문이다.
즉, optimization을 하기 위해서 근사하는 행위에 필요한 게 바로 저 `H` matrix의 역할이다.
 
---

## 수식 유도

수식 (1)의 변수들을 각각 $$\boldsymbol{\xi}^{t} = [t_x, t_y, \theta]^{\intercal}$$, $$\boldsymbol{\delta}^{i} = [\delta_x, \delta_y, \delta_\theta]^{\intercal}$$라 표현해보자.
그러면 SE(2)에서의 증분은 아래와 같이 표현된다:

$$\left[\begin{array}{ccc}
\cos \theta & -\sin \theta & t_x \\
\sin \theta & \cos \theta & t_y \\
0 & 0 & 1
\end{array}\right]\left[\begin{array}{ccc}
\cos \delta_{theta} & -\sin \delta_{theta} & \delta_x \\
\sin \delta_{theta} & \cos \delta_{theta} & \delta_x \\
0 & 0 & 1
\end{array}\right] \; \; \; \; \text{(2)}$$

(2)에서 $$\delta_{theta} \sim 0$$이면 $$\cos \delta_{theta} \sim 1$$, $$\sin \delta_{theta} \sim \delta_{theta}$$로 근사가 가능하므로(small angle approximation),
(2)는 아래와 같이 간소화되어 표현이 가능하다:

$$\left[\begin{array}{ccc}
\cos \theta & -\sin \theta & t_x \\
\sin \theta & \cos \theta & t_y \\
0 & 0 & 1
\end{array}\right]\left[\begin{array}{ccc}
1 & -\delta_{theta} & \delta_x \\
\delta_{theta} & 1 & \delta_y \\
0 & 0 & 1
\end{array}\right].\; \; \; \; \text{(3)}$$

따라서 위의 수식을 전개한 후의 translation 값이 $$h(\boldsymbol{xi} \oplus \boldsymbol{\delta})$$와 대응되는 값이다.
편의를 위해 $$\mathbf{t} = [t_x, t_y]^\intercal$$, $$\delta_{\mathbf{t}} = [\delta_x, \delta_y]^\intercal$$라 표현하면,  
$$h(\boldsymbol{xi} \oplus \boldsymbol{\delta})$$에 해당하는 값은 

$$\mathbf{t} + R(\theta)\delta_{\mathbf{t}} = h(\boldsymbol{xi}) + R(\theta)\delta_{\mathbf{t}}\; \; \; \; \text{(4)}$$

이 된다.
따라서 우리가 최종적으로 구하고자 하는 $$\mathbf{H}$$는 위의 (4)를 표현하기 위해 $$\boldsymbol{\delta}$$ 앞에 곱해지는 matrix이므로,
$$\mathbf{H} = [\mathbf{R} \mathbf{0}_{2\times1}] \in \mathbb{R}^{2\times3}$$이 되는 것을 확인할 수 있다 (위의 수식 내에 $$\delta_{theta}$$가 존재하지 않으므로, $$\delta_{theta}$$에 대한 partial derivative는 모두 다 0이 되는 것은 자명하다).

## Conclusion

따라서, GTSAM에서 optimization을 할 때 필요한 Jacobian matrix `H`는 단순히 $$\left[\begin{array}{ccc}
\frac{df_1}{t_x} & \frac{df_1}{t_y} & \frac{df_1}{\theta} \\
\frac{df_2}{t_x} & \frac{df_2}{t_y} & \frac{df_2}{\theta} \\
\end{array}\right]$$가 아님을 주의하자! `H`는 미소 증분 vector인 $$\boldsymbol{\delta}$$를 표현하기 위한 matrix임을 잊지 말자.

---

GTSAM Tutorial 시리즈입니다.

{% include post_links_gtsam.html %}