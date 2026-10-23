---
layout: post
title: Multi-Task Learning 최적화 비교 - GradNorm, PCGrad, CAGrad, UPGrad 선택 기준
subtitle: Gradient의 크기와 방향을 직접 바꿔보며 이해하는 loss balancing과 gradient surgery
tags: [Multi-Task Learning, GradNorm, PCGrad, CAGrad, UPGrad, Optimization, Research Note]
comments: true
description: GradNorm, PCGrad, CAGrad, UPGrad의 원리와 차이를 interactive gradient 예제로 설명한다. 음수 cosine의 의미, task별 Jacobian 계산 비용, 두 task에서 PCGrad와 UPGrad의 관계와 방법 선택 기준을 정리한다.
permalink: /2026/10/23/gradnorm-pcgrad-cagrad-upgrad-comparison/
---

## Introduction

Multi-task learning을 공부하다 보면 GradNorm, PCGrad, CAGrad, UPGrad 같은 이름을 만나게 된다.
설명을 읽어보면 모두 여러 task를 함께 잘 학습하기 위한 방법인데, 막상 무엇부터 적용해야 할지는 조금 애매하다.

흔히 'gradient가 충돌하면 PCGrad 같은 방법을, 학습 속도가 다르면 GradNorm을 사용한다'고 한다.
직관적으로는 이해가 되지만, 왜 그런지 궁금해서 각 방법이 실제로 무엇을 바꾸는지 정리해봤다.

내가 이해한 핵심은 다음과 같다.

> GradNorm은 각 task의 발언권을 조절하고, PCGrad와 UPGrad는 서로 방해하는 gradient 성분을 수정한다. CAGrad는 평균 loss를 줄이는 방향 근처에서 가장 불리한 task의 개선량을 고려한다.

이 차이를 이해하려면, 먼저 gradient의 방향과 실제 parameter의 이동 방향을 구분해야 한다.

---

## 사전 지식: Gradient와 실제 업데이트 방향

공유 parameter를 $$\theta$$, task별 loss를 $$L_i$$라고 하자.
각 task의 gradient는 $$g_i = \nabla_\theta L_i$$이다.
여러 gradient를 합쳐 최종 벡터 $$d$$를 만들면, 일반적인 gradient descent는 다음처럼 이동한다.

$$
\theta_{\mathrm{new}} = \theta - \eta d
$$

즉 **실제 이동 방향은 $$d$$의 반대 방향**이다.
이 글에서는 $$d$$를 'parameter에서 빼줄 벡터'라는 의미로 사용한다.
작은 step에서 task별 loss 변화는 다음과 같이 근사할 수 있다.

$$
\Delta L_i \approx -\eta g_i^\top d
$$

따라서 $$g_i^\top d$$가 양수이면 해당 task의 loss가 감소하고, 음수이면 증가하는 방향이다.
이 값을 아래 예제에서는 **1차 근사 개선량**이라고 부르겠다. Learning rate를 곱하기 전의 값이다.

여기서 중요한 구분이 나온다.

- $$g_1^\top g_2 < 0$$: 두 task의 gradient에 서로 반대되는 성분이 있다.
- $$g_i^\top d < 0$$: 최종 업데이트가 task $$i$$를 악화시키는 방향이다.

**두 gradient의 cosine이 음수라는 사실만으로, 최종 업데이트에서 어느 task의 loss가 반드시 증가하는 것은 아니다.**
다른 task의 영향뿐 아니라 자기 gradient의 기여도 함께 있기 때문이다.
이 설명은 현재 gradient에 대한 국소적인 1차 근사이며, 큰 learning rate나 Adam의 실제 이동까지 그대로 보장하지는 않는다.

## Interactive 예제: 네 가지 Case

Case와 방법을 바꾸면 원본 gradient, 수정된 gradient, 최종 $$d$$를 비교할 수 있다.
실선은 원본, 점선은 수정된 task gradient이며, 굵은 화살표는 최종 $$d$$이다.
표의 개선량이 음수라면 해당 task가 손해를 보는 방향이다.

<style>
.mtl-gradient-frame { width: 100%; height: 780px; border: 0; }
@media (max-width: 600px) { .mtl-gradient-frame { height: 1150px; } }
</style>
<iframe class="mtl-gradient-frame" src="{{ '/assets/interactive/gradient-methods.html' | relative_url }}" title="GradNorm, PCGrad, CAGrad, UPGrad interactive 비교" loading="lazy"></iframe>

[Interactive 예제 크게 보기]({{ '/assets/interactive/gradient-methods.html' | relative_url }})

예제는 원리를 분리해서 보기 위한 **2차원 gradient 실험**이며, 실제 모델을 학습시키는 simulation은 아니다.
비교를 위해 기본 aggregation을 평균으로 통일했고, CAGrad는 추가적인 rescaling 없이 원래의 제약식으로 계산한다.
GradNorm은 gradient와 loss 비율을 고정한 상태에서 weight만 반복 업데이트한다. Alpha와 업데이트 횟수를 바꿔볼 수 있으며, 양수 weight를 유지하고 합을 task 수로 정규화한다.
표의 Weight는 명시적인 loss weight이며, projection이나 CAGrad 내부 최적화에서 유도되는 결합 계수를 뜻하지 않는다.
세 task의 PCGrad는 결과를 재현할 수 있도록 projection 순서를 task 번호순으로 고정했다. 원래 알고리즘은 순서를 무작위로 선택한다.

---

## GradNorm: 학습 속도에 따른 Loss Weight 조정

GradNorm은 task별 loss에 곱하는 양수 weight $$w_i$$를 학습한다.
Task gradient 자체를 다른 방향으로 꺾는 대신, **각 gradient가 최종 업데이트에 기여하는 크기**를 바꾼다.

학습 속도는 순간적인 loss 기울기가 아니라, 초기 loss 대비 현재 loss의 비율로 판단한다.

$$
\widetilde L_i(t)=\frac{L_i(t)}{L_i(0)},\qquad
r_i(t)=\frac{\widetilde L_i(t)}{\operatorname{mean}_j\widetilde L_j(t)}
$$

예를 들어 task 1의 loss가 처음의 30%까지 내려왔고, task 2는 90% 남아 있다면 task 2가 상대적으로 느리게 학습한 것이다.
GradNorm은 이런 task에 더 큰 gradient norm을 배정하도록 weight를 조절한다.

선택한 shared parameter 집합 $$W$$에서 측정한 norm을 $$G_i=\|\nabla_W(w_iL_i)\|$$라고 하면, 목표 norm은 다음과 같다.

$$
G_i^{\mathrm{target}}=\overline G\,r_i^\alpha
$$

실제 norm과 목표 norm의 차이를 줄이도록 weight를 업데이트한다. 이때 목표값은 weight를 미분하는 과정에서 상수로 취급한다.
**Alpha가 0이면 norm을 비슷하게 맞추고, 양수이면 느린 task에 더 큰 norm을 요구한다.**
그러므로 단순히 모든 gradient를 unit vector로 만드는 방법은 아니다.
([GradNorm 원 논문](https://proceedings.mlr.press/v80/chen18a.html))

Case A에서는 두 gradient가 충돌하지 않지만 task 2의 loss 감소가 느리다.
PCGrad와 UPGrad는 제거할 충돌이 없어 평균과 같은 결과를 내는 반면, GradNorm은 task 2의 상대적인 기여를 키울 수 있다.

다만 양수 weight로 길이를 바꾸어도 **두 원본 gradient 사이의 cosine은 바뀌지 않는다.**
최종 합의 방향은 달라질 수 있지만, 서로 반대되는 성분을 직접 제거하는 것은 아니다.
또 noisy label 때문에 loss가 줄지 않는 task에 더 큰 weight를 주는 것이 항상 좋은 선택은 아니다.

## PCGrad: 다른 Task를 방해하는 성분 제거

PCGrad는 task별 gradient를 복사한 뒤, 다른 task의 원본 gradient와 충돌하는지 확인한다.
현재 수정 중인 gradient를 $$\widetilde g_i$$라고 할 때, $$\widetilde g_i^\top g_j<0$$이면 다음 projection을 적용한다.

$$
\widetilde g_i \leftarrow \widetilde g_i
-\frac{\widetilde g_i^\top g_j}{\|g_j\|^2}g_j
$$

말로 풀면 **task $$j$$의 gradient와 반대되는 성분을 빼는 것**이다.
수정 직후에는 $$\widetilde g_i$$가 $$g_j$$에 수직이 된다. Zero gradient는 projection 대상에서 제외한다.
각 task에 대해 이 과정을 수행한 뒤 결과를 합친다.
([PCGrad 원 논문](https://arxiv.org/abs/2001.06782))

Case B의 gradient는 다음과 같다.

$$
g_1=(3,0),\qquad g_2=(-1,1)
$$

단순 평균은 $$d=(1,0.5)$$이다.
이때 $$g_2^\top d=-0.5$$이므로 task 2가 손해를 본다.
PCGrad로 수정하면 $$\widetilde g_1=(1.5,1.5)$$, $$\widetilde g_2=(0,1)$$이고,
평균은 $$d=(0.75,1.25)$$가 된다. 이제 두 task의 1차 근사 개선량은 각각 $$2.25$$와 $$0.5$$로 양수다.

단, 세 task 이상에서는 한 번 제거한 충돌이 다음 projection에서 다시 생길 수 있다.
PCGrad를 '모든 task와 충돌하지 않는 방향을 항상 찾아주는 방법'으로 이해하면 안 된다.

## CAGrad: 평균 Loss와 최악의 개선량 사이의 절충

CAGrad는 task별 gradient를 차례로 수정하는 대신, 최종 $$d$$를 직접 찾는다.
평균 gradient를 $$g_0=\operatorname{mean}_i g_i$$라고 하면 핵심 문제는 다음과 같다.

$$
\max_d\;\min_i g_i^\top d
\quad\text{subject to}\quad
\|d-g_0\|\le c\|g_0\|
$$

두 부분으로 나누면 이해하기 쉽다.

- **목적:** 가장 적게 개선되는 task의 개선량을 가능한 크게 만든다.
- **제약:** 평균 gradient에서 너무 멀리 벗어나지 않는다.

$$c=0$$이면 평균 gradient 그대로이고, $$c$$를 키우면 불리한 task를 고려할 여지가 커진다.
논문은 적절한 smoothness와 step-size 조건 아래 $$0\le c<1$$에서 평균 loss의 stationary point로 수렴하는 성질을 분석한다. Deep network의 global optimum을 보장한다는 뜻은 아니다.
([CAGrad 원 논문](https://arxiv.org/abs/2110.14048))

Case B에서 CAGrad의 $$c$$를 움직여보면, 평균 방향에서 조금씩 벗어나며 task 2의 손해를 줄이는 모습을 볼 수 있다.
다만 허용 범위가 좁으면 모든 task의 개선량을 양수로 만들지 못할 수 있다.
**CAGrad는 모든 충돌을 금지하는 방법이 아니라, 평균 loss 최적화와 task별 개선 사이의 절충**에 가깝다.

## UPGrad: 모든 Task와 충돌하지 않는 영역으로 Projection

UPGrad는 각 gradient를 다음 영역 안으로 projection한다.

$$
\mathcal C=\{v\mid g_j^\top v\ge0\ \text{for every }j\}
$$

어느 task에 대해서도 1차 근사 개선량이 음수가 되지 않는 벡터들의 집합이다.
각 원본 gradient와 가장 가까운 $$\mathcal C$$ 안의 벡터를 구한 뒤 평균한다.
PCGrad가 다른 gradient를 하나씩 고려한다면, **UPGrad는 모든 task의 제약을 동시에 만족시키도록 projection**한다.
([UPGrad: Jacobian Descent 원 논문](https://arxiv.org/html/2406.16232v2))

하지만 '충돌하지 않음'과 '모든 task가 빠르게 개선됨'은 다르다.
Case D에서는 공동으로 개선할 수 있는 방향이 없고, 제약을 만족하는 벡터가 원점밖에 없다.
그래서 UPGrad 결과는 $$d=0$$이다. 새로운 학습 신호를 만들어서 문제를 해결하는 것은 아니다.

여기서 두 head를 사용하는 경우에 중요한 사실이 있다.

> Task가 정확히 두 개라면, 같은 gradient와 aggregation scale을 사용한 PCGrad와 UPGrad는 같은 결과를 낸다.

원 논문 표기에서는 PCGrad가 합, UPGrad가 평균이라 크기 차이가 있지만, 방향은 같다.
위 예제는 둘 다 평균을 사용하므로 Case A~C에서 결과가 일치한다.
따라서 vector loss와 segmentation loss를 **두 objective로 묶어** 학습한다면, UPGrad로 바꾼다고 다른 방향이 생기는 것은 아니다.
반면 classification, point regression, segmentation 등을 각각 별도 objective로 취급하면 task 수가 달라진다.
([원 논문 Appendix B.4](https://arxiv.org/html/2406.16232v2))

---

## 반례: Cosine이 음수여도 둘 다 개선되는 경우

Case C는 $$g_1=(1,0)$$, $$g_2=(-0.5,\sqrt{3}/2)$$이다.
두 gradient의 cosine은 $$-0.5$$이지만, 평균 gradient와 각 task gradient의 내적은 모두 $$0.25$$이다.
즉 단순 평균으로도 두 task가 개선되는 방향이다.

이 case에서는 projection 후 최종 벡터의 방향도 평균과 같고, 크기만 달라진다.
이런 차이를 비교할 때는 learning rate 효과까지 살펴봐야 한다.
**음수 cosine의 빈도만으로 gradient surgery가 필요하다고 결론 내릴 수 없는 이유**다.

## 계산 비용: Jacobian과 GradNorm의 차이

Task가 $$T$$개이고 shared parameter가 $$P$$개라면, task별 gradient를 쌓은 행렬은 다음과 같다.

$$
J=\begin{bmatrix}g_1^\top\\ \vdots\\g_T^\top\end{bmatrix}
\in\mathbb R^{T\times P}
$$

이것이 여기서 말하는 **loss vector의 Jacobian**이다. Hessian이나 2차 미분을 구한다는 뜻은 아니다.
합산 loss를 한 번 backward하면 합쳐진 gradient만 얻으므로, task별 gradient를 구분하려면 추가적인 미분 계산이 필요하다.

PCGrad, CAGrad, UPGrad를 전체 shared parameter에 적용하면 task별 gradient 계산과 저장 비용이 커질 수 있다.
단순히 FP32로 $$J$$ 전체를 저장한다고 할 때, $$P=10^8$$이고 $$T=2$$이면 gradient 행렬만 약 0.8 GB다.
이는 **행렬 자체의 크기**이며 baseline 대비 추가 메모리나 전체 peak memory를 뜻하지 않는다.

**GradNorm도 task별 gradient norm을 계산해야 하므로 공짜는 아니다.**
원 논문은 보통 마지막 shared layer에서 norm을 측정해 비용을 줄인다.
이렇게 작은 parameter 집합에서 norm만 사용하는 설정과, 전체 encoder의 task gradient를 보관하는 설정은 비용이 크게 다를 수 있다.
([GradNorm, Section 3.1](https://proceedings.mlr.press/v80/chen18a/chen18a.pdf))

CAGrad와 UPGrad에는 추가적인 최적화 문제를 푸는 비용도 있다.
다만 task 수가 적다면 solver보다 task별 backward가 지배적일 수 있다.
또 $$JJ^\top$$라는 task 간 내적 행렬을 이용하는 구현 등으로 전체 Jacobian을 명시적으로 저장하는 비용을 줄일 여지도 있다.
그렇다고 task별 미분 정보를 얻는 비용까지 사라지는 것은 아니다.
([CAGrad](https://arxiv.org/abs/2110.14048), [Gramian-based Jacobian descent](https://arxiv.org/html/2406.16232v2))

따라서 '뒤의 세 방법은 무조건 몇 배 느리다'보다, **어느 parameter에 적용하고 gradient와 graph를 어떻게 보관하는지**를 확인해야 한다.
Shared BEV feature에서만 gradient를 조작하는 것은 parameter 공간에서 적용하는 것과도 다르다.
특히 feature 공간에서 확보한 내적 관계가 encoder parameter 공간에서도 그대로 유지된다고 볼 수는 없다.

## 선택 기준: 무엇부터 적용할 것인가

| 관찰 또는 목표 | 먼저 비교할 후보 | 이유와 주의점 |
|---|---|---|
| 충돌보다 task별 norm·loss 감소율 차이가 두드러짐 | GradNorm | 기여 크기를 조절. 느린 이유가 label noise인지도 확인 |
| 합산 업데이트에서 특정 task의 손해가 반복됨 | PCGrad | 충돌 성분을 제거하는 비교적 단순한 baseline |
| 평균 loss를 중심에 두면서 불리한 task도 고려하고 싶음 | CAGrad | 평균 gradient 근처에서 worst improvement를 최적화 |
| 여러 task의 1차적 비충돌 조건을 동시에 요구함 | UPGrad | 모든 제약을 동시에 고려. 업데이트가 작아지거나 0이 될 수 있음 |
| 두 objective에서 PCGrad와 UPGrad 중 고민함 | 우선 PCGrad | 동일한 입력과 scale에서는 같은 결과 |
| 음수 cosine만 있고 실제 성능 저하는 불명확함 | 기존 weighted sum 유지·진단 | 음수 cosine 자체는 교체 근거가 아님 |

내가 BEV vector head와 segmentation head를 함께 학습한다면, 먼저 loss normalization과 GT 정합성을 확인하고,
shared block에서 task별 norm, cosine, $$g_i^\top d$$를 기록할 것 같다.
그다음 loss 감소율 차이가 문제인지, 업데이트 방향이 문제인지에 따라 GradNorm과 PCGrad를 각각 비교하겠다.

또한 raw gradient에서의 비충돌 조건은 Adam의 preconditioning이나 momentum을 거친 **실제 parameter 이동**에 그대로 보장되지 않는다.
진단값이 좋아졌다고 vector mAP와 segmentation mIoU가 반드시 좋아지는 것도 아니다.
동일한 학습 조건에서 validation 성능, 실행 시간, peak GPU memory를 함께 비교해야 한다.

결국 '충돌이면 gradient surgery, 속도 차이면 GradNorm'은 유용한 출발점이다.
다만 최종 선택은 **cosine의 부호 하나가 아니라, 어떤 task가 왜 손해를 보고 있으며 무엇을 최적화하려는가**에 달려 있다.
