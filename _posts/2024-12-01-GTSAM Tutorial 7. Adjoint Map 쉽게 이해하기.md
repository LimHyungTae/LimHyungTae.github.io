---
layout: post
title: GTSAM Tutorial 7. Adjoint Map 쉽게 이해하기
subtitle: Understanding the Concepts of Adjoint Map
tags: [Jacobian, GTSAM]
comments: true
---

### Adjoint Map의 의미?

그렇다면, 이전 글에서 $$\boldsymbol{\delta}_1$$ 앞에 $$ `AdjointMap()`을 곱해주는 행위가 물리적으로 무슨 의미일까? 

결론적으로 말하자면, 이 adjoint matrix는 `p1` 관점 좌표계 상의 오차 $$\boldsymbol{\delta}_1$$를 `p2`의 좌표계로 변환하는 역할을 한다.

예를 들어서 설명을 해보자.
우리가 기차에 타있고, 기차의 창문을 통해 새가 날아다니는 걸 관측했다고 치자.
새의 관점에서 살펴보면, 새는 주로 +$$x$$ 방향으로 병진 운동 + 약간의 rotation을 취할 것이다.
하지만 기차에 타있는 우리와 새의 그 시각의 pose의 차이가 어떻냐에 따라 해당 움직임이 기차 안의 우리에게는 창문으로부터 멀어지면서 회전하게끔 관측될 수도 있고, 창문으로부터 가까워지면서 회전하게끔 관측될 수도 있다.

따라서, 우리가 이전 `BetweenFactor`를 통해 도출해낸 $$\boldsymbol{\delta}_1$$와 $$\boldsymbol{\delta}_2$$는 각기 다른 축에서의 local한 좌표계 기준으로 묘사되어 있기 때문에 바로 덧셈을 하는 것이 불가능하다. 
하지만 이 `p1` 관점의 pose의 미소 변화량 $$\boldsymbol{\delta}_1$$을 `p2` 관점에서 볼 수 있게끔 transform을 해주어, 덧셈이 가능해지는 것이다. 

이 사실을 기반으로 이전 글의 수식 (7)을 다시 살펴보면: 

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

왜 $$\boldsymbol{\delta}_2$$에서는 단순히 Identity matrix가 곱해졌는지 이제 알 수 있다. 왜냐하면 우리가 하고자 했던 것은 각기 다른 좌표계 상의 두 pose에서 optimization을 통해 변화되는 미소 pose 변화량 $$\boldsymbol{\delta}$$를 `p2` 좌표계로 통일하고 싶기 때문이다. 그러니 $$\boldsymbol{\delta}_2$$는 이미 `p2` 좌표계로 기술되어 있기 때문에 변화할 필요가 없고, `p1`의 $$\boldsymbol{\delta}_1$$만 `p2` 좌표계로 옮겨주면 된다.

지금까지 깊은 수학 없이 최대한 학부생-석사 1년차 초 학생들이 이해할 수 있게끔 쉽게 써봤는데, 진또배기 맛을 보고 싶은 이는 GTSAM 레포지토리의 [doc/math.pdf](https://github.com/borglab/gtsam/blob/develop/doc/math.pdf)를 읽어보면 된다(추천하진 않는다...).

---

GTSAM Tutorial 시리즈입니다.

{% include post_links_gtsam.html %}