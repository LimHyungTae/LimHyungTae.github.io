---
layout: post
title: CLS Token이란? ViT 토큰의 의미와 차원 쉽게 이해하기
subtitle: Patch token과의 차이부터 DINOv3를 활용한 GOOSE Challenge 1위 사례까지
tags: [CLS Token, Vision Transformer, ViT, DINOv3, Semantic Segmentation, GOOSE]
comments: true
description: CLS token의 의미와 학습 방식, token 개수와 embedding 차원의 차이를 설명한다. DINOv3의 1024차원 CLS feature를 11개 category 예측에 활용한 ICRA 2026 GOOSE Challenge 1위 사례도 소개한다.
permalink: /2026/10/12/cls-token-vit-dimension/
---

## Introduction

Vision Transformer 논문을 읽다 보면 **CLS token**, **patch token**, **register token**이라는 말이 나온다.
그런데 token 하나라고 하면 숫자가 하나라는 것인지, 벡터 하나라는 것인지 처음에는 헷갈린다.
특히 CLS token에 classifier를 붙인다는 설명만 보면, 그 안에 어떤 정보가 들어 있는지 감이 잘 오지 않는다.

내가 이해한 핵심은 다음과 같다.

> ViT에서 token 하나는 여러 숫자로 이루어진 벡터이고, CLS token은 그중 이미지 전체를 대표하는 표현을 읽어내기 위해 마련한 자리이다.

이 글에서는 token의 의미와 차원부터 설명하고, 내가 참여한 GOOSE Challenge 1위 솔루션에서 CLS token을 어떻게 활용했는지 연결해 본다.

---

## 사전 지식: Token과 Embedding

Token은 모델이 다루는 입력 단위이다.
텍스트 모델에서는 단어나 단어 조각이 token에 대응하고, ViT에서는 이미지를 나눈 작은 **patch**가 token에 대응한다.
이 단위를 모델이 계산할 수 있는 벡터로 바꾼 것이 **embedding**이다.

ViT에서 'patch token을 Transformer에 넣는다'고 말할 때는 보통 이 벡터를 뜻한다.

```text
Image patch -> Embedding -> [0.12, -0.37, 0.08, ..., 0.51]
                            숫자 D개로 이루어진 token 하나
```

예를 들어 RGB 이미지의 16 x 16 patch에는 16 x 16 x 3 = 768개의 픽셀 값이 있다.
이를 projection으로 1024차원 벡터로 바꿀 수 있다.
**원본 patch의 크기와 token의 embedding 차원은 서로 다른 값**이다.

또한 각 차원이 '하늘', '차량', '나무'처럼 미리 정해진 의미를 하나씩 담당하는 것은 아니다.
학습된 벡터의 여러 성분이 함께 정보를 표현한다고 보는 편이 정확하다.

## CLS Token의 역할: 이미지 전체를 읽는 자리

CLS는 classification에서 온 이름이다.
ViT에서는 patch에서 만든 token들 외에, 학습 가능한 벡터 하나를 추가한다.
이것이 **CLS token**이다.

```text
Transformer 입력:
[CLS] [Patch 1] [Patch 2] ... [Patch N]
```

처음 넣는 CLS embedding은 이미지마다 새로 추출하는 값이 아니라, 학습되는 공통 parameter이다.
하지만 Transformer를 통과하면서 attention으로 다른 token들의 정보를 받아들이므로,
**출력의 CLS feature는 입력 이미지에 따라 달라진다.**

입력에서는 같은 출발점을 사용하고, 출력에서는 각 이미지에 대한 표현을 얻는 셈이다.
이 출력 벡터에 classification head를 붙이고 학습하면, 예측에 필요한 정보를 모으도록 학습 신호가 전달된다.
([ViT 논문](https://arxiv.org/abs/2010.11929))

따라서 CLS라는 이름을 붙였다고 자동으로 완벽한 요약이 만들어지는 것은 아니다.
어떤 정보를 담는지는 architecture와 학습 목표에 영향을 받는다.
Patch token들의 단순 평균과도 다르다.
평균은 정해진 pooling 연산이고, CLS는 Transformer 안에서 학습되는 정보 교환을 거쳐 얻는 표현이다.

---

## Token 개수와 벡터 차원

**Token 하나라고 해서 1차원 정보인 것은 아니다.**
예를 들어 DINOv3 ViT-L/16의 embedding dimension은 1024이다.
CLS 하나도 1024개의 숫자이며, patch token 하나도 1024개의 숫자이다.
`/16`은 patch의 한 변 길이를 나타내며, embedding dimension을 뜻하지 않는다.
([DINOv3 공식 구현](https://github.com/facebookresearch/dinov3/blob/main/dinov3/models/vision_transformer.py))

차원을 구체적으로 세어 보자.
아래는 대회 입력 크기가 아니라, 계산을 쉽게 하기 위한 **224 x 224 이미지 예시**이다.

```text
Patch 크기:             16 x 16
Patch 개수:             (224 / 16) x (224 / 16) = 196
Token 하나의 벡터 길이:  D = 1024
```

DINOv3의 ViT는 CLS 하나와 register 네 개도 사용하므로, 총 201개의 token이 된다.
([DINOv3 Model Card](https://github.com/facebookresearch/dinov3/blob/main/MODEL_CARD.md))

| 대상 | Shape |
|---|---|
| 이미지 하나의 CLS feature | `[1024]` |
| 이미지 하나의 patch features | `[196, 1024]` |
| 이미지 하나의 전체 token sequence | `[201, 1024]` |
| B개 이미지를 묶은 전체 sequence | `[B, 201, 1024]` |
| Batch에서 CLS만 추출한 결과 | `[B, 1024]` |

여기서 **201은 token 개수, 1024는 각 token의 벡터 차원, B는 batch size**이다.
코드에서 CLS의 token 축을 유지하면 `[B, 1, 1024]`로 표현할 수도 있다.
이는 같은 정보를 축 하나를 유지한 채 나타낸 것이다.

이미지가 커지면 patch token 수는 늘어나지만, 같은 모델의 embedding dimension은 그대로이다.
즉 더 큰 이미지라고 CLS가 자동으로 2048차원으로 늘어나는 것은 아니다.

## Patch Token과 Register Token의 차이

| Token | 입력에서의 출처 | 주된 용도 |
|---|---|---|
| Patch token | 이미지 patch를 embedding한 벡터 | 위치별 feature 활용 |
| CLS token | 학습 가능한 추가 벡터 | 이미지 전체를 대표하는 feature 활용 |
| Register token | 학습 가능한 추가 벡터 | 내부 계산을 위한 여분의 작업 공간 |

같은 Transformer 안에서는 같은 embedding dimension을 쓰더라도 역할이 다를 수 있다.
또한 출력 patch token도 attention을 통해 주변 정보를 받으므로, 해당 patch의 픽셀 정보만 담는 것은 아니다.

Register token은 보통 최종 task의 출력으로 직접 읽지 않는다.
자세한 배경은 [Register Token 글](/2026/10/13/what-is-register-token/)에서 다뤘다.
CLS는 이와 달리 이미지 단위의 예측에 사용할 대표 벡터라는 점을 기억하면 된다.

---

## 활용 사례: GOOSE Challenge 1위 솔루션

내가 공동 저자로 참여한 [ICRA 2026 GOOSE Challenge 기술 보고서](https://arxiv.org/abs/2606.18582)에서는 CLS를 **보조 학습 목표**에 사용했다.
주 task는 64개 fine class에 대한 pixel별 segmentation이다.

```text
DINOv3 ViT-L/16
  |
  +-- Patch features -> ViT-Adapter -> Mask2Former -> Segmentation
  |
  +-- CLS feature [1024] -> Linear head -> Category logits [11]
```

CLS head는 이미지에 11개 coarse category가 각각 존재하는지 예측한다.
정답 mask에서 각 category가 한 pixel이라도 나타나면 1로 표시하므로 추가 annotation은 필요 없다.
여러 category가 동시에 존재할 수 있어 **multi-hot target과 sigmoid 기반 binary cross-entropy**를 사용했다.
위치나 면적을 예측하는 head는 아니다.

여기서 **1024는 feature 차원이고, 11은 예측할 category 수**이다.
Head가 1024차원 표현을 11개 logit으로 변환한다.
학습 목표는 다음과 같으며, 보조 loss의 가중치는 0.1이었다.

$$
L = L_{\mathrm{segmentation}} + 0.1 L_{\mathrm{CLS}}
$$

의도는 위치별 예측과 함께 장면 전체의 category 정보도 encoder가 유지하도록 돕는 것이다.
Development composite mIoU는 보조 loss 추가 전 69.80%에서 71.20%로 **1.40%p** 높아졌다.

최종 제출은 TTA와 세 checkpoint의 ensemble까지 포함해 composite 76.57%, fine-class 69.32%, category-level 83.81%로 1위를 기록했다.
이는 CLS만의 효과가 아니라 전체 구성의 결과이다.
([보고서의 방법 및 실험 결과](https://arxiv.org/html/2606.18582v1))

## 정리

CLS token은 이미지 전체의 표현을 읽어내는 **벡터 하나**이다.
그 벡터에 들어 있는 숫자의 개수는 모델의 embedding dimension으로 정해지고,
최종 예측값의 개수는 그 뒤에 붙이는 head로 정해진다.

그래서 코드를 읽을 때는 세 가지를 따로 보면 된다.

- **Token 수**: 몇 개의 벡터를 처리하는가.
- **Embedding dimension**: 각 벡터에 숫자가 몇 개 있는가.
- **Head output dimension**: 최종적으로 무엇을 몇 개 예측하는가.

이 구분이 잡히면 CLS를 분류나 보조 loss에 활용한다는 설명도 훨씬 구체적으로 이해할 수 있다.
