---
layout: post
title: GTSAM Tutorial 2. Block Operation in Matrix
subtitle: Block Operation
tags: [Jacobian, GTSAM]
comments: true
---

## Block Operation

그런데 Frank Dellaert 교수님 자료를 보면 위의 식을 좀더 섹시(?)하게 block operation으로 기입하는 것을 볼 수 있다.
수식 [1]을 좀더 간결히 쓰면 아래와 같이 쓸 수 있는데: 

$$\mathbf{x}^{\prime} = T(\mathbf{x}) = \mathbf{R}\mathbf{x} + \mathbf{t} \; \; \; \; \text{[3]}$$

이를 통해 수식 [2]의 Jacobian을 block operation으로 간결히 적으면 아래와 같이 적을 수 있다:

$$J=\left[\begin{array}{ll}
\frac{\partial T(\mathbf{x})}{\partial \mathbf{t}} & \frac{\partial T(\mathbf{x})}{\partial \theta} \end{array}\right] \; \; \; \; \text{[4]}$$

위에서 $$\frac{\partial T(\mathbf{x})}{\partial \mathbf{t}}$$는 $$2\times1$$의 vector에 대한 partial derivative이므로 $$2\times2$$의 크기가 되고, $$\frac{\partial T}{\partial \theta}$$는 2개의 수식에 대한 1개의 변수의 derivative이기 때문에 $$2\times1$$의 matrix가 된다. 
이제 수식 [3]을 scalar로 구성된 equation이랑 동일하게 취급해보자. 그러면 $$\frac{\partial T(\mathbf{x})}{\partial \mathbf{t}}$$의 경우 $$T(\mathbf{x})$$ (i.e., [3])에서의 $$\mathbf{t}$$가 상수마냥 존재하기 때문에 원래 scalar의 세계에서는 1이 될 것이다.
하지만 우리는 현재 matrix의 세계에 있으므로, 이 값은 identity matrix $$\mathbf{I}_{2\times2}$$가 된다. 그리고 이는 [2]의 앞쪽 $$2\times2$$구간과 일치한다.

그리고 뒤의 $$2\times1$$ 구간은 

$$\frac{\partial T(\mathbf{x})}{\partial \theta} = \frac{\partial \mathbf{R}}{\partial \theta}\mathbf{x}  \; \; \; \; \text{[5]}$$

가 될 것이다. 그럼 여기서, $$\frac{\partial \mathbf{R}}{\partial \theta}$$는 어떻게 구할 수 있을까? 
이는 다음 글에서 살펴보자. 

---

GTSAM Tutorial 시리즈입니다.

{% include post_links_gtsam.html %}