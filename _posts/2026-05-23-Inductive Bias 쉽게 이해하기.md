---
layout: post
title: Inductive Bias 쉽게 이해하기
subtitle: 모델이 세상을 어떤 방식으로 보겠다고 미리 가정하는가
tags: [Machine Learning, Deep Learning, Inductive Bias, Research Note]
comments: true
description: Inductive bias의 의미를 머신러닝 관점에서 정리한다. 모델 구조, loss, data augmentation, optimizer가 어떤 가정을 넣는지와 왜 bias가 일반화에 필요한지 설명한다.
permalink: /2026/05/23/what-is-inductive-bias/
redirect_from:
  - '/2026-05-23-Inductive Bias 쉽게 이해하기/'
  - '/2026-05-23-Inductive-Bias-쉽게-이해하기/'
  - '/2026-05-23-Inductive Bias란 무엇인가/'
  - '/2026-05-23-Inductive-Bias란-무엇인가/'
---

## 들어가며

Machine learning 글을 읽다 보면 **inductive bias**(귀납적 편향, 즉 모델에 미리 넣어둔 가정)라는 말을 자주 만난다.

처음에는 대충 "모델이 가진 가정" 정도로 이해하고 넘어가도 된다.
그런데 논문을 읽다 보면 이 단어가 꽤 중요한 위치에 놓인다.

```text
CNN은 translation equivariance라는 inductive bias를 가진다.
Transformer는 CNN보다 inductive bias가 약하다.
Data-driven method는 hand-crafted inductive bias를 줄인다.
```

이런 문장을 제대로 이해하려면, inductive bias를 단순히 "편향"이라고 번역하면 조금 부족하다.

내가 이해한 결론부터 말하면 이렇다.

> Inductive bias는 모델이 학습 데이터를 넘어 일반화하기 위해 미리 넣고 들어가는 가정이다.

즉 모델은 데이터를 그냥 외우는 것이 아니라,
"세상이 아마 이런 구조를 가질 것이다"라는 가정을 들고 학습을 시작한다.

---

## 왜 Bias가 필요한가

머신러닝의 목표는 training data를 맞히는 것이 아니다.
정말 중요한 것은 **보지 못한 데이터에 잘 동작하는 것**, 즉 generalization이다.
그런데 관측한 데이터만 놓고 보면 가능한 설명은 무한히 많다.
예를 들어 점 몇 개가 주어졌다고 하자.

```text
(1, 2), (2, 4), (3, 6)
```

우리는 자연스럽게 `y = 2x`를 떠올린다.
하지만 사실 이 세 점을 통과하는 이상한 고차 다항식도 무한히 만들 수 있다.
training point만 보면 둘 다 맞다.

그럼 왜 우리는 단순한 직선을 더 그럴듯하다고 느끼는가?

이미 "세상은 너무 이상하게 꼬인 함수보다 단순한 함수일 가능성이 높다"는 bias를 가지고 있기 때문이다.
이것이 inductive bias의 가장 기본적인 형태이다.

Inductive bias가 없으면, 모델은 관측하지 않은 지점에서 무엇을 해야 할지 결정할 수 없다.
그래서 일반화는 결국 bias 위에서만 가능하다.

---

## 어디에 숨어 있나

Inductive bias는 꼭 모델 architecture에만 있는 것이 아니다.
생각보다 여러 곳에 숨어 있다.

| 위치 | 들어가는 가정 |
|---|---|
| Model architecture | 입력의 구조가 특정 방식으로 처리되어야 한다 |
| Loss function | 어떤 오차를 더 중요하게 볼 것인가 |
| Regularization | 단순하거나 작은 해를 더 선호한다 |
| Data augmentation | 어떤 변환이 label을 바꾸지 않는다고 본다 |
| Optimizer | 어떤 경로로 parameter space를 탐색할 것인가 |
| Preprocessing | 어떤 정보는 중요하고 어떤 정보는 버려도 된다고 본다 |

예를 들어 CNN을 보자.
CNN은 이미지에서 nearby pixel들이 서로 관련이 있고,
같은 pattern이 위치를 바꿔도 비슷한 의미를 가진다고 가정한다.

그래서 convolution kernel을 local하게 적용하고,
같은 kernel을 전체 image에 공유한다.
이건 단순한 구현 trick이 아니다.

```text
이미지는 locality와 translation structure를 가진다.
```

라는 강한 inductive bias를 넣은 것이다.

반대로 MLP는 모든 pixel을 그냥 긴 vector로 본다.
locality라는 가정을 거의 쓰지 않는다.
즉 MLP가 이미지 task에 안 맞는다는 뜻은 아니다.
다만 CNN이 architecture에 미리 넣어둔 locality와 weight sharing을 MLP는 데이터에서 직접 배워야 하므로,
같은 조건에서는 CNN보다 data-efficient하지 않을 수 있다.

---

## Strong Bias와 Weak Bias

Inductive bias는 좋고 나쁨의 문제가 아니라,
**문제에 잘 맞는가**의 문제이다.

강한 bias는 search space를 많이 줄여준다.
그래서 데이터가 적을 때 유리하고, 학습도 안정적일 수 있다.
하지만 bias가 문제와 안 맞으면 성능의 ceiling을 낮출 수 있다.

약한 bias는 더 유연하다.
대신 모델이 스스로 구조를 배워야 하므로 보통 더 많은 데이터와 계산이 필요하다.

대충 이렇게 볼 수 있다.

| Bias | 장점 | 단점 |
|---|---|---|
| Strong inductive bias | data-efficient, 안정적, 해석 쉬움 | 가정이 틀리면 한계가 뚜렷함 |
| Weak inductive bias | flexible, scale하면 강력함 | 데이터/연산이 많이 필요함 |

Transformer가 자주 이 관점에서 설명된다.
CNN은 이미지에 대한 locality, weight sharing 같은 bias가 강하다.
반면 Transformer는 attention으로 token 간 관계를 더 자유롭게 보므로 architecture 자체의 bias는 상대적으로 약하다.

그래서 작은 데이터에서는 CNN 계열이 더 잘 버티는 경우가 많고,
큰 데이터와 큰 compute가 있으면 Transformer가 더 넓은 구조를 학습할 수 있다.

---

## Research Code 관점에서 보기

연구 코드를 읽을 때 inductive bias를 찾는 습관은 꽤 유용하다.

논문이 새로운 module을 제안했다면, 단순히 "성능이 오른다"가 아니라 이런 질문을 던져볼 수 있다.

```text
이 module은 어떤 가정을 넣는가?
그 가정은 이 task의 data와 맞는가?
이 bias는 architecture에 들어갔나, loss에 들어갔나, preprocessing에 들어갔나?
데이터가 커지면 이 bias는 여전히 이득인가, 아니면 제약인가?
```

예를 들어 SLAM이나 3D vision 쪽에서는 이런 bias가 자주 나온다.

- 주변 point는 같은 surface에 있을 가능성이 높다.
- 공간은 대부분 smooth하다.
- camera나 LiDAR pose는 시간적으로 갑자기 튀지 않는다.
- rigid body motion을 따른다.
- scene geometry는 low-dimensional structure를 가진다.

이런 가정들이 factor graph, smoothness loss, voxel grid, plane fitting, motion prior 같은 형태로 코드에 들어간다.
겉으로는 그냥 engineering choice처럼 보이지만, 사실은 모델이 세상을 바라보는 방식이다.

---

## 정리

Inductive bias는 모델이 데이터를 보기 전에 이미 가지고 들어가는 가정이다.

핵심만 정리하면 다음과 같다.

- 일반화는 bias 없이 불가능하다.
- inductive bias는 architecture, loss, regularization, augmentation, optimizer 등 여러 곳에 들어간다.
- 강한 bias는 data efficiency를 주지만, 틀린 가정이면 한계가 된다.
- 약한 bias는 flexible하지만, 보통 더 많은 data와 compute가 필요하다.
- 좋은 모델 설계는 bias를 없애는 것이 아니라, task에 맞는 bias를 고르는 일에 가깝다.

그래서 논문이나 코드를 볼 때 "무슨 module을 썼나"에서 멈추지 말고,
한 단계 더 들어가서 이렇게 물어보면 좋다.

> 이 방법은 세상에 대해 무엇을 미리 믿고 있는가?

이 질문에 답할 수 있으면, inductive bias를 꽤 제대로 이해하고 있는 것이다.
