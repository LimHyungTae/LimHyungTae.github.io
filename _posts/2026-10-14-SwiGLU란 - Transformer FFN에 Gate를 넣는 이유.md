---
layout: post
title: SwiGLU와 GELU FFN 비교 - Transformer의 Gate 구조와 파라미터 수
subtitle: LLM에서 익숙한 gated FFN을 ViT로 가져올 때의 판단 기준
tags: [SwiGLU, FFN, Transformer, Vision Transformer, LLM]
comments: true
description: SwiGLU의 두 branch와 elementwise gating을 설명한다. GELU FFN과의 parameter 수 비교, DINOv3의 실제 사용 variant, pretrained ViT 교체 시 고려할 점을 정리한다.
permalink: /2026/10/14/swiglu-transformer-ffn/
---

## Introduction

Transformer를 개선한다고 하면 attention부터 떠올리기 쉽다.
하지만 각 token의 정보를 변환하는 FFN도 상당한 parameter와 계산을 차지한다.
SwiGLU는 이 FFN의 구조를 바꾸는 방법이다.

내가 이해한 핵심은 다음과 같다.

> SwiGLU는 두 갈래로 변환한 feature를 곱해, 입력에 따라 각 성분이 전달되는 정도를 조절하는 FFN이다.

## 사전 지식: Attention과 FFN

Attention은 token 사이의 정보를 섞는다.
반면 일반적인 FFN은 같은 network를 각 token에 독립적으로 적용한다.

```text
Attention: 다른 token에서 어떤 정보를 가져올 것인가
FFN:       가져온 정보를 각 token 안에서 어떻게 변환할 것인가
```

기본적인 FFN은 차원을 늘리고, GELU 같은 activation을 적용하고, 원래 차원으로 돌아온다.

```text
x [D] -> Linear [M] -> GELU -> Linear [D]
```

## SwiGLU의 구조: 두 Branch의 곱

Bias를 생략하면 SwiGLU는 다음처럼 쓸 수 있다.

$$
\mathrm{SwiGLU}(x)=\left[\mathrm{SiLU}(xW_g)\odot(xW_u)\right]W_d
$$

$$\odot$$는 같은 위치의 성분끼리 곱한다는 뜻이다.
$$\mathrm{SiLU}(a)=a\,\mathrm{sigmoid}(a)$$이며 Swish의 한 형태이다.

```text
        +-> Linear -> SiLU --+
x [D] --+                   +-> 성분별 곱 [M] -> Linear -> y [D]
        +-> Linear ---------+
```

한쪽을 gate, 다른 쪽을 전달할 feature로 이해하면 편하다.
다만 SiLU 출력은 0~1 사이의 확률이 아니다.
SwiGLU의 gate는 feature를 단순히 켜고 끄는 binary switch도 아니다.
([GLU Variants Improve Transformer, 2020](https://arxiv.org/abs/2002.05202))

입력이 `[B, N, D]`라면 두 branch는 각각 `[B, N, M]`, 최종 출력은 `[B, N, D]`이다.
Token 수는 그대로 유지된다.

## 공정한 비교: Hidden Dimension

GELU FFN의 hidden dimension을 그대로 쓰고 SwiGLU로 바꾸면 parameter가 늘어난다.
Bias를 제외한 weight 수를 세면 차이가 보인다.

| FFN | 대략적인 parameter 수 |
|---|---|
| 두 Linear를 사용하는 일반 FFN | $$2DM$$ |
| 세 Linear를 사용하는 SwiGLU | $$3DM$$ |

따라서 기존 FFN이 $$M=4D$$였다면, 비슷한 parameter 수의 SwiGLU는 $$M\approx 8D/3$$이 된다.
실제 구현에서는 GPU 연산에 맞춰 배수 단위로 조정하기도 한다.

즉 'SwiGLU가 더 좋았다'는 결과를 읽을 때는 **같은 hidden dimension인지, 같은 parameter budget인지**를 먼저 봐야 한다.
학습 메모리와 latency 역시 별도로 측정해야 한다.

## 실제 채택: LLM과 Vision

[Qwen3](https://arxiv.org/html/2505.09388v1)는 SwiGLU를 사용하는 LLM 사례이다.
Vision에서도 DINOv3의 ViT-S+, ViT-H+ 등 일부 variant가 SwiGLU를 사용한다.
그러나 기본 ViT-L/16은 공식 설정에서 `mlp`를 사용한다.
**같은 DINOv3 계열이라고 FFN 구조까지 모두 같은 것은 아니다.**
([DINOv3 backbone 설정](https://github.com/facebookresearch/dinov3/blob/main/dinov3/hub/backbones.py))

이런 사례는 SwiGLU가 vision에서도 사용 가능한 설계라는 근거이다.
모든 ViT에서 GELU보다 항상 낫다는 근거로 확대하면 안 된다.

## ViT 적용: 무엇을 바꿔야 하는가

GELU를 SiLU로 바꾸는 것만으로 SwiGLU가 되지는 않는다.
Gate branch와 elementwise product가 필요하므로 Linear layer의 구성도 바뀐다.
기존 FFN weight를 그대로 옮겨서 같은 함수가 되리라고 기대하기 어렵다.

처음부터 학습하는 모델이라면 동일 budget으로 GELU FFN과 비교하기 좋다.
Pretrained encoder를 쓴다면 기존 모델을 유지한 baseline과 재학습 조건을 맞춰야 한다.
관심 task가 segmentation이라면 classification 성능뿐 아니라 dense feature의 품질도 확인해야 한다.

## 정리

SwiGLU는 **token 사이의 연결이 아니라, token 내부의 feature 변환**을 개선하는 후보이다.
Attention을 그대로 둔 채 FFN의 표현 방식을 바꿔 볼 수 있다는 점에서 실험의 범위가 명확하다.

[Transformer 구조 개선 시리즈 전체 목록](/2026/10/14/transformer-architecture-techniques/)
