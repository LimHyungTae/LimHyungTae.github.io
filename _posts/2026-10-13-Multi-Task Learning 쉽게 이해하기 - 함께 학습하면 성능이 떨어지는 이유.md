---
layout: post
title: Multi-Task Learning 쉽게 이해하기 - 함께 학습하면 성능이 떨어지는 이유
subtitle: Shared latent에서의 gradient 간섭과 이를 줄이는 방법
tags: [Multi-Task Learning, Gradient, PCGrad, GradNorm, Deep Learning, Research Note]
comments: true
description: 관련된 두 task를 하나의 latent vector로 학습해도 성능이 떨어지는 이유를 설명한다. Gradient 크기와 방향, 표현 용량을 구분하고 loss weighting, GradNorm, PCGrad, task-specific branch의 적용 기준을 정리한다.
permalink: /2026/10/13/multi-task-learning-gradient-interference/
---

## Introduction

하나의 latent vector에 여러 head를 붙여 학습했는데, 각각 따로 학습할 때보다 성능이 떨어지는 경우가 있다.
심지어 두 task가 서로 관련되어 있는데도 이런 일이 생긴다.
같은 정보를 활용하는 문제라면 같이 배우는 것이 더 유리해야 하지 않을까?

내가 이해한 핵심은 다음과 같다.

> Task가 서로 관련되어 있다는 것과, 현재 모델에서 두 task가 같은 방향의 업데이트를 요구한다는 것은 별개의 문제이다.

이 글에서는 하나의 encoder와 latent를 공유하는 **multi-task learning**을 기준으로,
왜 이런 일이 생기고 무엇부터 확인하면 좋을지 정리한다.

---

## 사전 지식: Shared Latent와 두 Head

Latent vector는 encoder가 입력을 변환한 내부 표현이다.
Head는 그 표현을 받아 각 task의 예측을 만드는 부분이다.

```text
Input -> Shared encoder -> Latent z -> Head A -> Prediction A
                                    -> Head B -> Prediction B
```

두 loss를 더해서 학습한다면 다음처럼 쓸 수 있다.

$$
L = \lambda_A L_A + \lambda_B L_B
$$

여기서 $$\lambda_A$$와 $$\lambda_B$$는 각 loss에 주는 가중치이다.
Head A와 Head B의 parameter가 분리되어 있어도, **encoder는 두 loss의 영향을 함께 받는다.**
따라서 head를 두 개로 나눴다고 학습 전체가 독립적이 되는 것은 아니다.

## Gradient의 방향과 크기

Gradient는 parameter를 조금 바꿨을 때 loss가 어떻게 변하는지 나타내는 벡터이다.
일반적인 gradient descent는 그 반대 방향으로 parameter를 움직여 loss를 줄인다.

공유 encoder의 parameter를 $$\theta$$라고 하면, 두 task가 만드는 gradient는 다음과 같다.

$$
g_A = \nabla_\theta L_A, \qquad g_B = \nabla_\theta L_B
$$

Encoder를 업데이트할 때는 $$\lambda_A g_A + \lambda_B g_B$$가 사용된다.
중요한 것은 **gradient가 두 개라는 사실보다, 두 벡터의 방향과 크기**이다.

| 관계 | 작은 gradient descent step에서의 의미 |
|---|---|
| 비슷한 방향 | 한 task의 업데이트가 다른 task에도 도움이 될 수 있음 |
| 직교하는 방향 | 다른 task에 대한 1차적인 영향이 없음 |
| 반대 성분이 있는 방향 | 한 task의 업데이트가 다른 task의 loss를 늘릴 수 있음 |

즉 '서로 독립적인 gradient라서 학습이 나빠진다'는 표현은 조금 모호하다.
서로 직교한다는 의미라면, 그것만으로 충돌이라고 보지는 않는다.

이를 확인할 때 자주 보는 값이 **gradient cosine similarity**이다.

$$
\cos(g_A,g_B) = \frac{g_A^\top g_B}{\|g_A\|\|g_B\|}
$$

음수이면 두 gradient에 서로 반대되는 성분이 있다는 뜻이다.
예를 들어 B의 gradient만 따라 작은 step을 움직일 때 A의 loss 변화는 대략 다음과 같다.

$$
\Delta L_A \approx -\eta g_A^\top g_B
$$

$$\eta$$는 learning rate이다.
내적이 음수이면 위 값은 양수가 되어 A의 loss가 증가하는 방향이다.
이것이 **gradient conflict**를 이해하는 출발점이다.
([PCGrad 논문](https://arxiv.org/abs/2001.06782))

다만 이것은 현재 parameter와 batch에서의 국소적인 설명이다.
Cosine이 한 번 음수였다고 최종 성능 저하의 원인이 확정되는 것은 아니고,
합쳐진 gradient가 반드시 두 loss를 모두 악화시키는 것도 아니다.
Adam처럼 gradient를 변환하는 optimizer에서는 실제 update 방향도 함께 고려해야 한다.

---

## 관련된 Task 사이에서도 생기는 간섭

예를 들어 같은 물체의 종류와 자세를 예측한다고 하자.
둘 다 물체를 이해하는 문제이므로 서로 관련되어 있다.
하지만 분류에는 자세가 달라져도 비슷한 표현이 유리할 수 있고,
자세 추정에는 그 차이가 표현에 남아 있어야 한다.

충분한 용량과 적절한 구조가 있다면 둘 다 담을 수 있다.
하지만 작은 latent 하나로 모든 정보를 압축하면 어떤 정보를 남길지 경쟁할 수 있다.
이 예시는 **task의 관련성과 필요한 표현의 일치가 항상 같지는 않다**는 뜻이다.

성능 저하의 후보는 크게 세 가지로 나누어 볼 수 있다.

| 원인 후보 | 확인할 내용 |
|---|---|
| Gradient 크기 불균형 | 한 task가 encoder 업데이트를 대부분 차지하는가? |
| Gradient 방향 충돌 | 같은 shared parameter에서 gradient가 반복적으로 충돌하는가? |
| 표현 용량 또는 공유 구조 | 하나의 latent가 두 task에 필요한 정보를 충분히 담는가? |

Multi-task 학습 때문에 특정 task의 성능이 나빠지는 현상을 **negative transfer**라고 부른다.
다만 현재 관찰만으로 위 셋 중 무엇이 원인인지는 알 수 없다.
데이터, label 품질, 학습량 차이도 먼저 확인해야 한다.

## 진단: Loss 값보다 Task별 학습 상태

먼저 A만, B만, A+B를 학습하는 세 실험을 비교한다.
Task별 데이터 노출 횟수, validation split, tuning 조건을 맞추고 여러 seed에서 차이가 반복되는지 본다.
같은 epoch 수라도 task를 번갈아 sampling하면 각 task의 학습량은 달라질 수 있다.

그다음 아래 항목을 기록한다.

- Task별 training loss와 validation metric.
- 같은 shared parameter에 대한 task별 gradient norm과 cosine similarity.
- 실제 loss weight를 곱한 뒤의 gradient norm.

전체 encoder가 부담스럽다면 마지막 shared block부터 측정할 수 있다.
이 경우 그 block에 대한 진단이라는 점을 기억해야 한다.
또한 A와 B의 별도 head parameter를 비교하는 것이 아니라, **같은 공유 parameter에 대한 gradient를 비교**해야 한다.

Loss가 100과 1이라고 해서 encoder에 미치는 영향도 100배 차이 나는 것은 아니다.
Loss의 값과 미분값의 크기는 다르기 때문이다.
한 batch의 측정값보다 여러 batch와 학습 구간의 추세가 더 중요하다.

---

## Technique 1: Loss Weighting과 GradNorm

첫 단계는 loss의 단위와 reduction을 확인하는 것이다.
한 loss는 sample별 평균이고 다른 loss는 모든 pixel의 합이라면, 의도하지 않은 불균형이 생길 수 있다.
이를 정리한 뒤 $$\lambda_A$$와 $$\lambda_B$$를 바꿔 validation 성능을 비교한다.

자동 조정 기법으로는 **GradNorm**이 있다.
Shared parameter에 도달하는 gradient 크기와 task별 상대적인 학습 속도를 보고 loss weight를 조절한다.
단순히 모든 gradient norm을 같게 만드는 방법으로만 이해하면 부정확하다.
상대적으로 학습이 느린 task도 고려한다.
([GradNorm, ICML 2018](https://proceedings.mlr.press/v80/chen18a.html))

또 다른 방법은 **uncertainty weighting**이다.
Task별로 학습하는 uncertainty parameter를 이용해 loss의 비중을 조절한다.
여기서 uncertainty는 sample마다 별도로 예측하는 confidence가 아니라 task 단위의 값이다.
가중치를 무조건 낮추는 해를 막는 항도 함께 사용하며, 구체적인 식은 loss의 확률 모델에 따라 달라진다.
([Kendall et al., CVPR 2018](https://arxiv.org/abs/1705.07115))

이 방법들은 loss 균형을 잡는 후보이다.
방향이 충돌하는 gradient를 직접 수정하는 기법과는 구분해서 보는 것이 좋다.

## Technique 2: PCGrad

**PCGrad**는 gradient 방향 충돌을 직접 다루는 방법이다.
두 task의 gradient가 충돌하면, 한 gradient에서 다른 gradient와 충돌하는 성분을 projection으로 제거한 뒤 합친다.

직관적으로는 다음과 같다.

```text
기존:   Task별 gradient를 그대로 합침
PCGrad: 충돌하는 성분을 조정한 뒤 합침
```

Task별 gradient를 따로 구해야 하므로 학습 계산과 구현 부담은 늘어난다.
현재 batch에서의 충돌을 완화하는 방법이지, 모든 task의 validation 성능 향상을 보장하지는 않는다.
Latent에 필요한 정보가 없거나 label이 잘못된 문제도 별도로 해결해야 한다.
([PCGrad, NeurIPS 2020](https://arxiv.org/abs/2001.06782))

## Technique 3: Task별 표현 공간 분리

하나의 latent를 끝까지 공유하는 것이 부담이라면, 어디까지 공유할지를 바꿀 수 있다.

```text
Input -> Shared early blocks -> Branch A -> Latent A -> Head A
                            -> Branch B -> Latent B -> Head B
```

앞부분에서는 공통 정보를 배우고, 뒷부분에서는 task별 표현을 만든다.
간단하게는 shared latent 뒤에 task별 MLP를 추가해 볼 수 있다.
다만 이미 latent에서 사라진 정보는 MLP만으로 복원하기 어려우므로,
이 경우에는 더 이른 layer에서 분기하거나 latent 용량을 늘리는 실험이 필요하다.

더 유연한 구조로는 **MMoE(Multi-gate Mixture-of-Experts)**가 있다.
여러 expert network를 공유하되, 각 task의 gate가 expert 출력을 어떤 비율로 사용할지 학습한다.
모든 task가 동일한 표현 조합을 쓰도록 강제하는 정도를 줄이는 방식이다.
구조와 비용이 늘어나므로 간단한 branch 분리와 먼저 비교할 만하다.
([MMoE, KDD 2018](https://research.google/pubs/modeling-task-relationships-in-multi-task-learning-with-multi-gate-mixture-of-experts/))

## 실험 순서: 관찰에 따른 선택

실제로 적용한다면 나는 다음 순서로 확인할 것 같다.

1. Single-task baseline과 학습량, loss reduction, validation 조건을 맞춘다.
2. Loss weight를 조절하며 두 task의 성능 변화를 함께 본다.
3. Gradient 크기가 불균형하면 GradNorm, 방향 충돌이 반복되면 PCGrad를 비교한다.
4. 개선이 부족하면 task별 branch와 latent 용량을 바꿔 공유 구조를 점검한다.

A가 주목적이고 B가 auxiliary task라면 B의 비중을 줄이거나 천천히 높이는 실험도 가능하다.
B 경로에서 latent를 `detach()`하면 B의 loss가 encoder를 바꾸지 못하게 할 수도 있다.
이 경우 B head는 학습되지만, **B가 encoder의 표현 학습에 도움을 주는 경로도 차단된다.**
따라서 원인을 분리하는 실험이나 주 task를 우선하는 설계로 해석해야 한다.

두 task를 번갈아 업데이트하는 것만으로는 간섭이 사라지지 않는다.
여전히 같은 encoder를 서로 다른 방향으로 바꾸기 때문이다.

## 정리

관련된 task를 함께 학습하는 것은 도움이 될 가능성이 있다는 뜻이지, 성능 향상을 보장하는 조건은 아니다.
중요한 것은 **gradient의 크기, 방향, 그리고 표현을 어디까지 공유할지**이다.

Weighting은 task의 영향력을, PCGrad는 충돌하는 방향을, branch 분리는 공유 구조를 조정한다.
이 역할을 구분해 두면, 성능이 떨어졌을 때 기법을 무작정 추가하기보다 관찰에 맞는 실험을 선택할 수 있다.

---

## 최근 방법론: PCGrad 이후의 접근

그렇다면 PCGrad나 GradNorm 이후에는 어떤 방법들이 나왔을까?
여기서는 2023~2025년에 발표된 세 가지를 소개한다.
확인한 논문들의 결과만으로 모든 task 조합에서 통하는 하나의 표준 해법을 고르기는 어렵다.
아래 방법들은 각 논문의 benchmark에서 좋은 결과를 보고한 비교 후보로 이해하면 된다.

### FAMO: 계산 부담을 줄인 Task 균형 조정

**FAMO(Fast Adaptive Multitask Optimization, NeurIPS 2023)**는 task별 loss가 균형 있게 감소하도록 가중치를 조절한다.
핵심은 모든 task의 parameter gradient를 별도로 계산하고 저장하는 부담을 줄인다는 점이다.

PCGrad처럼 task별 gradient를 직접 다루면 task 수가 많아질수록 비용이 커진다.
FAMO는 loss의 변화 정보를 활용해 weight를 업데이트하므로, 이런 비용이 부담인 상황에서 비교할 만하다.
논문은 supervised learning과 multi-task RL에서 기존 gradient 조정 기법과 비슷하거나 더 좋은 성능을 보고한다.
물론 전체 모델의 계산량이 task 수와 무관해진다는 뜻은 아니다.
Task별 head와 loss 계산은 여전히 필요하다.
([FAMO 논문](https://arxiv.org/abs/2306.03792), [공식 구현](https://github.com/Cranial-XIX/FAMO))

두 head만 쓰는 현재 상황에서는 task가 수십 개인 경우보다 비용상의 이점이 작을 수 있다.
그래도 **구조 변경 없이 loss 균형을 조절하는 비교 실험**으로 고려할 만하다.

### FairGrad: Task별 개선량을 고려한 Gradient 조합

**FairGrad(ICML 2024)**는 task 사이에 업데이트의 이득을 어떻게 배분할지에 주목한다.
PCGrad가 충돌하는 성분을 제거하는 방식이라면, FairGrad는 task별 loss 감소량을 바탕으로 전체 update 방향을 결정하는 최적화 문제를 만든다.

직관적으로는 다음과 같은 질문이다.

> 전체 성능을 개선하면서, 일부 task만 계속 뒤처지지 않도록 업데이트를 배분할 수 있을까?

여기서 fairness는 서로 단위가 다른 accuracy와 depth error를 같은 숫자로 맞춘다는 뜻이 아니다.
Task별 개선량을 어떤 기준으로 평가하고 배분할지에 관한 것이다.
논문은 supervised learning과 RL benchmark에서 gradient 조정 방법들 대비 우수한 결과를 보고한다.
([FairGrad 논문](https://proceedings.mlr.press/v235/ban24a.html), [공식 구현](https://github.com/OptMN-Lab/fairgrad))

현재 실험에서 **한 task는 좋아지는데 다른 task가 계속 희생되는 양상**이라면 비교 후보가 된다.
Task별 gradient와 가중치 계산이 필요하므로 성능뿐 아니라 학습 비용도 함께 봐야 한다.

### ConsMTL: Head가 만드는 Gradient 자체의 조정

**ConsMTL(CVPR 2025)**은 이 글의 질문과 특히 잘 연결된다.
출발점은 **task-specific head도 shared encoder로 전달되는 gradient의 방향에 영향을 준다**는 것이다.

지금까지의 설명에서는 head가 gradient를 보내면 shared parameter 쪽에서 이를 조정했다.
ConsMTL은 한 단계 더 나아가, head를 학습하는 방식에도 추가적인 목표를 준다.
Shared parameter에서는 공동 update 방향을 구하고,
task-specific parameter에는 각 task가 보내는 gradient가 그 방향에 가까워지도록 추가 loss를 적용한다.
논문은 이를 두 수준의 최적화, 즉 bi-level optimization으로 구성한다.
([ConsMTL 논문](https://openaccess.thecvf.com/content/CVPR2025/papers/Qin_Towards_Consistent_Multi-Task_Learning_Unlocking_the_Potential_of_Task-Specific_Parameters_CVPR_2025_paper.pdf), [공식 구현](https://github.com/jianke0604/MTLLib))

쉽게 말하면 **도착한 gradient를 조정하는 것에 더해, 서로 협력하기 좋은 gradient를 보내도록 head도 학습하는 방식**이다.
따라서 단순히 head를 더 깊게 만드는 것과는 다르며, 추가 loss와 학습 절차를 구현해야 한다.
Shared latent에 여러 head를 붙인 상황에서 gradient 간섭이 반복적으로 관찰된다면 읽어볼 만하다.

### 현재 문제에서의 비교 후보

| 관찰 또는 제약 | 비교해 볼 방법 |
|---|---|
| Task별 gradient 계산 비용이 부담 | FAMO |
| 일부 task만 좋아지고 다른 task가 뒤처짐 | FairGrad |
| Head가 encoder로 보내는 gradient의 간섭이 반복됨 | ConsMTL |

이 표는 각 방법의 목적을 바탕으로 한 실험 제안이며, 성능 순위는 아니다.
지금처럼 두 task를 다루는 상황이라면 기존 weighted-sum과 PCGrad를 기준으로 두고,
관찰된 문제에 맞는 방법을 하나씩 추가해 비교하는 편이 결과를 해석하기 좋다.
평균 점수뿐 아니라 **각 task가 single-task baseline 대비 얼마나 좋아지거나 나빠졌는지**까지 확인해야 한다.
