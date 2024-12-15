---
layout: post
title: GTSAM Tutorial 7. Adjoint Map 쉽게 이해하기
subtitle: Understanding H matrices in GTSAM
tags: [Jacobian, GTSAM]
comments: true
---

### Adjoint Map의 의미?

그렇다면, 저 `AdjointMap()`을 곱해주는 행위가 물리적으로 무슨 의미일까? 

결론적으로 말하자면, 이 adjoint matrix는 `p1` 관점 좌표계 상의 오차 $$\boldsymbol{\delta}_1$$를 `p2`의 좌표계로 변환하는 역할을 한다.
즉, pose1에서의 오차를 pose2 기준으로 어떻게 보이는지 계산합니다.
이 때 이 두 twist는 다른 축에서의 local한 좌표계 기준으로 묘사되어 있는 twist이기 때문에, 바로 덧셈을 하는 것이 불가능하다. 하지만 이 p1 관점의 twist를 p2 관점에서 볼 수 있게끔 tangent space 상에서 transform을 해주는 것이다. 계속해서 우리는 두 pose의 상태 pose를 다룰 때 현재 vector에 대응되는 $$(N+1) \times (N+1)$$ matrix를 통해 표현을 했는데, adjoint matrix를 

(쉽게 풀어서 쓰느라 수학적으로 엄밀하지 않을 수 있는데, 혹시 나의 설명에서 틀린 설명이 있다면 언제든 메일로 정리해서 보내주시면 감사하겠습니다,,,)


결국, 두 pose의 초기 오차를 고려해서 최종적으로 두 pose의 상대적인 오차를 하나의 좌표계 기준으로 표현하는 것입니다.
이 과정은 SLAM이나 Pose Graph Optimization 등에서 상대 오차를 계산하는 데 매우 중요한 역할을 합니다.


---

GTSAM Tutorial 시리즈입니다.

{% include post_links_gtsam.html %}