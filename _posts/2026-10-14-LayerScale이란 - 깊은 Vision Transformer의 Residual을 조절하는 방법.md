---
layout: post
title: LayerScale이란? 깊은 Vision Transformer의 Residual을 조절하는 방법
subtitle: DINOv3 코드에 숨어 있는 channel별 learnable scale
tags: [LayerScale, Residual, Vision Transformer, DINOv3, Training Stability]
comments: true
description: LayerScale의 residual scaling과 작은 초기값의 역할을 설명한다. LayerNorm 및 DropPath와의 차이, DINOv3 채택 사례, pretrained backbone에서의 주의점을 정리한다.
permalink: /2026/10/14/layerscale-vision-transformer/
---

## Introduction

ViT block 코드를 읽으면 attention 출력에 작은 벡터를 곱한 뒤 입력에 더하는 부분이 보인다.
계산은 단순해서 지나치기 쉽지만, 깊은 network를 학습하는 관점에서는 의미가 있다.
이 장치가 **LayerScale**이다.

> LayerScale은 residual branch가 기존 표현을 얼마나 바꿀지 channel별로 학습하는 배율이다.

2021년 CaiT 연구에서 제안된 방법이며, 최신 발명은 아니다.
다만 DINOv3 같은 최근 vision backbone에도 쓰이므로, 새로운 gating 기법을 이해할 때 비교 기준으로 알아두면 좋다.

## 사전 지식: Residual Connection

Residual block은 입력에 새 계산 결과를 더한다.

$$
y=x+F(x)
$$

여기서 $$F$$가 attention 또는 FFN branch라고 생각하면 된다.
입력을 유지하는 경로와, 이를 수정하는 경로가 함께 있는 구조이다.
LayerScale은 다음과 같이 바꾼다.

$$
y=x+\gamma\odot F(x)
$$

$$\gamma$$는 학습 가능한 벡터이고, $$\odot$$는 성분별 곱이다.
입력이 `[B, N, D]`라면 gamma는 보통 `[D]`이며 batch와 token 축으로 공유된다.

```text
x ----------------------------+-> 더하기 -> y
  -> Norm -> Attention -> gamma 곱 -+
```

Attention과 FFN에 각각 별도의 gamma를 사용할 수 있다.
이 경우 block 하나에 추가되는 parameter는 2D개이다.
([CaiT / Going deeper with Image Transformers](https://arxiv.org/abs/2103.17239))

## 작은 초기값의 의미

Gamma를 작은 양수로 초기화하면 학습 초기에 각 branch의 기여가 작아진다.
처음부터 여러 layer가 표현을 크게 바꾸기보다, 학습하면서 branch의 기여도를 조절하는 방식이다.
공식 DINOv3 backbone 설정에서는 LayerScale 초기값으로 `1e-5`를 사용한다.
([DINOv3 설정](https://github.com/facebookresearch/dinov3/blob/main/dinov3/hub/backbones.py))

그러나 작은 값이 모든 상황에서 좋은 것은 아니다.
너무 작으면 branch 쪽의 학습 신호도 약해질 수 있다.
또한 학습되는 gamma는 sigmoid gate처럼 0~1로 제한되지 않으며, 입력마다 별도로 계산되는 값도 아니다.

## LayerNorm과 DropPath의 차이

| 기법 | 하는 일 |
|---|---|
| LayerNorm | Feature의 통계량을 이용해 정규화 |
| LayerScale | Residual 출력에 학습 가능한 channel별 배율 적용 |
| DropPath | 학습 중 residual branch를 확률적으로 생략 |

이름에 Layer가 들어가지만 LayerScale은 normalization이 아니다.
Feature 평균이나 분산을 계산하지 않고 scale을 곱한다.
([LayerScale 구현](https://github.com/facebookresearch/dinov3/blob/main/dinov3/layers/layer_scale.py))

DropPath는 stochastic depth 계열의 regularization이고, LayerScale은 연속적인 learnable scale이다.
둘은 함께 쓸 수 있지만 같은 기능은 아니다.
([Stochastic Depth](https://arxiv.org/abs/1603.09382))

## 적용 범위: 새 Block과 Pretrained Block

깊은 ViT를 처음부터 학습하거나 새로운 residual module을 추가한다면 작은 초기 scale을 비교해 볼 수 있다.
이는 기존 표현에 대한 초기 교란을 줄이려는 설계 선택이다.

반면 pretrained DINOv3를 fine-tuning할 때 이미 학습된 gamma를 다시 `1e-5`로 초기화하면,
기존 branch의 기여도를 크게 바꾸게 된다.
**작은 값으로 시작한 학습 방법과, 학습된 값을 작게 되돌리는 것은 전혀 다른 작업**이다.

실험에서는 layer별 gamma와 branch 출력 norm을 함께 확인하면 좋다.
Gamma 자체의 크기만으로 중요도를 판단하기보다, 실제로 residual에 더해지는 값의 크기를 봐야 한다.

## 정리

LayerScale은 **각 channel에서 새 정보를 얼마나 더할지 학습하는 장치**이다.
Vision에서 이미 검증된 채택 사례가 있지만, 모든 LLM의 표준 구성이라고 일반화할 필요는 없다.
입력에 따라 gate를 새로 계산하는 [Gated Attention](/2026/10/14/gated-attention-vision-transformer/)과 비교하면 차이가 더 명확해진다.

[Transformer 구조 개선 시리즈 전체 목록](/2026/10/14/transformer-architecture-techniques/)
