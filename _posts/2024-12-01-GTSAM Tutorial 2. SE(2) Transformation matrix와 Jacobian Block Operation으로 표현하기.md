---
layout: post
title: GTSAM Tutorial 2. SE(2) Transformation matrix와 Jacobian Block Operation으로 표현하기
subtitle: Transformation matrix and Jacobian 
tags: [Jacobian, GTSAM]
comments: true
---

## Introduction 

Jacobian matrix를 알잘딱으로 구해주는 Ceres solver와는 다르게, GTSAM을 잘 활용하기 위해서는 Jacobian matrix가 어떻게 도출되는 것인지에 대해서도 정확히 알아야 한다(i.e., 즉, [이전 글](https://limhyungtae.github.io/2024-12-01-GTSAM-Tutorial-1.-SLAM%EC%9D%84-%EC%9C%84%ED%95%9C-Between-Factor-%EC%89%BD%EA%B2%8C-%EC%9D%B4%ED%95%B4%ED%95%98%EA%B8%B0/)에서 제대로 설명하지 않고 넘어간, 아래의 `H1`과 `H2`가 어떻게 구해지는 지에 대해서도 이해를 해야 함):

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

사실 이러한 Jacobian 매트릭스는 이미 기존에 짜있는 factor를 이용할 경우에는 깊은 생각 없이 그냥 가져다 쓰면 된다. 하지만 자신이 새로운 factor를 만들어 optimization을 하고자 할 때는 이 Jacobian들도 적절히 유도를 해주어야 한다(개인적으로 이러한 이유 때문에 사용자 입장에서는 GTSAM이 Ceres에 비해 진입 장벽이 좀 높다고 생각된다~~옆집 Ceres Solver는 아묻따 `autodiff`로 Jacobian 알아서 구해줌~~)).

그래서 이번에는 pose를 표현할 때와 Jacobian을 구할 때 알아두면 좋을 개념에 대해 미리 설명하고자 한다.

## Jacobian of SE(2) Transformation?

예로 들어 아래와 같은 SE(2)를 통한 transformation 표현 식이 있다고 하자.

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
\end{array}\right]. \; \; \; \; \text{(1)}$$

위의 식을 살펴 보면, $$\mathbf{R}$$을 rotation matrix, $$\mathbf{t}$$라 정의했을 때, 위의 $$T\left(\theta, t_x, t_y\right)$$가 $$\left[\begin{array}{cc}
\mathbf{R} & \mathbf{t} \\
0 & 1
\end{array}\right]$$의 꼴로 구성되어 있는 것을 볼 수 있다(note: IEEE format에서는 matrix나 vector는 bold체로 써야한다. 이해를 할 때 참고).

그리고 이를 전개하면 아래와 같은데: 


$$\left[\begin{array}{c}
x^{\prime} \\
y^{\prime} \\
1
\end{array}\right]=\left[\begin{array}{c}
x \cos \theta-y \sin \theta+t_x \\
x \sin \theta+y \cos \theta+t_y \\
1
\end{array}\right]$$

위의 식의 Jacobian을 구해 보자. Jacobian의 각 row는 equation, column은 각 변수에 대한 미분 값을 나타내므로, Jacobian $$J$$는 아래와 같이 쓰여지고:

$$J=\left[\begin{array}{lll}
\frac{\partial x^{\prime}}{\partial t_x} & \frac{\partial x^{\prime}}{\partial t_y} & \frac{\partial x^{\prime}}{\partial \theta} \\
\frac{\partial y^{\prime}}{\partial t_x} & \frac{\partial y^{\prime}}{\partial t_y} & \frac{\partial y^{\prime}}{\partial \theta}
\end{array}\right]$$

최종적으로 이를 풀면 아래와 같다:

$$J=\left[\begin{array}{ccc}
1 & 0 & -x \sin \theta-y \cos \theta \\
0 & 1 & x \cos \theta-y \sin \theta
\end{array}\right] \; \; \; \; \text{(2)}$$

$$J$$의 가로축(row)에는 관계에 대한 표현식의 갯수, 세로축(column)에는 우리가 풀고자 하는 다 변수를 구성하는 요소의 개수로 구성되어 있는 것을 알 수 있다.

--- 

## Block Operation

그런데 Frank Dellaert 교수님 자료를 보면 위의 식을 좀더 섹시(?)하게 block operation으로 기입하는 것을 볼 수 있다.
수식 (1)을 좀더 간결히 쓰면 아래와 같이 쓸 수 있는데: 

$$\mathbf{x}^{\prime} = T(\mathbf{x}) = \mathbf{R}\mathbf{x} + \mathbf{t} \; \; \; \; \text{(3)}$$

이를 통해 수식 (2)의 Jacobian을 block operation으로 간결히 적으면 아래와 같이 적을 수 있다:

$$J=\left[\begin{array}{ll}
\frac{\partial T(\mathbf{x})}{\partial \mathbf{t}} & \frac{\partial T(\mathbf{x})}{\partial \theta} \end{array}\right] \; \; \; \; \text{(4)}$$

위에서 $$\frac{\partial T(\mathbf{x})}{\partial \mathbf{t}}$$는 크기가 2인 vector($$t_x$$와 $$t_y$$로 구성되어 있으므로)에 대한 partial derivative이므로 $$2\times2$$의 크기가 되고, $$\frac{\partial T}{\partial \theta}$$는 2개의 수식에 대한 1개의 변수의 partial derivative이기 때문에 $$2\times1$$의 matrix가 된다. 

이제 수식 (3)을 scalar로 구성된 equation처럼 여겨서 수식 (4)를 풀어 보자. 그러면 $$\frac{\partial T(\mathbf{x})}{\partial \mathbf{t}}$$의 경우 $$T(\mathbf{x})$$ (i.e., (3))에서의 $$\mathbf{t}$$가 상수마냥 존재하기 때문에 원래 scalar의 세계에서는 1이 될 것이다.
하지만 우리는 현재 matrix의 세계에 있으므로, 이 partial derivative $$\frac{\partial T(\mathbf{x})}{\partial \mathbf{t}}$$는 identity matrix $$\mathbf{I}_{2\times2}$$가 된다. 그리고 이는 수식 (2)의 앞쪽 $$2\times2$$ 구간과 일치한다.

그리고 뒤의 $$2\times1$$ 구간은 

$$\frac{\partial T(\mathbf{x})}{\partial \theta} = \frac{\partial \mathbf{R}}{\partial \theta}\mathbf{x}  \; \; \; \; \text{(5)}$$

가 될 것이다. 그럼 여기서, $$\frac{\partial \mathbf{R}}{\partial \theta}$$는 어떻게 구할 수 있을까? 
이를 위해서는 skew symmetric에 대한 이해가 필요한데, 이는 다음 글에서 살펴보자. 

---

GTSAM Tutorial 시리즈입니다.

{% include post_links_gtsam.html %}