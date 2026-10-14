---
layout: post
title: QK-Norm이란? Transformer Attention 학습을 안정화하는 방법
subtitle: LLM과 ViT에서 Query와 Key를 한 번 더 정규화하는 이유
tags: [Transformer, QK-Norm, Attention, Vision Transformer, LLM]
comments: true
description: QK-Norm의 역할과 Pre-Norm과의 차이를 설명한다. ViT-22B, Qwen3, Gemma 3의 채택 사례를 통해 vision에서 적용할 상황과 확인할 지표를 정리한다.
permalink: /2026/10/14/qk-norm-transformer/
---

## Introduction

Transformer 코드에는 이미 LayerNorm이 있는데, Query와 Key에 다시 norm을 적용하는 코드가 보일 때가 있다.
같은 정규화를 두 번 하는 것처럼 보이지만, 적용하는 위치가 다르다.

내가 이해한 핵심은 다음과 같다.

> QK-Norm은 attention이 정보를 비교하기 직전, Query와 Key의 크기를 조절하는 장치이다.

## 사전 지식: Attention Logit

Attention은 Query(Q)와 Key(K)를 비교해 weight를 만들고, 그 비중으로 Value(V)를 모은다.
Query 하나와 Key 하나의 비교 점수는 보통 다음처럼 계산한다.

$$
s_{ij}=\frac{q_i^\top k_j}{\sqrt{d_h}}
$$

$$d_h$$는 head 하나의 벡터 차원이다.
이 점수에 softmax를 적용하기 전의 값을 **attention logit**이라고 부른다.

내적은 방향뿐 아니라 벡터의 크기에도 영향을 받는다.
Q와 K의 norm이 과도하게 커지면 점수 차이가 커지고, softmax가 특정 token에 거의 모든 비중을 줄 수 있다.
정보를 선별하는 것 자체는 필요하지만, logit의 통제되지 않은 증가가 학습 불안정성으로 이어지는 상황은 별개의 문제이다.

## 구조: Projection 이후의 Normalization

```text
Input -> Pre-Norm -> Q projection -> Q-Norm --+
                  -> K projection -> K-Norm --+-> Attention
                  -> V projection -----------+
```

Pre-Norm은 Q/K/V를 만들기 전의 입력을 정규화한다.
그 뒤의 projection weight가 학습되면서 Q와 K의 크기는 다시 달라질 수 있다.
QK-Norm은 이 **projection 이후의 Q와 K**를 다룬다.

Qwen3 구현에서는 head dimension에 RMSNorm을 적용한다.
RMSNorm은 벡터의 제곱평균제곱근을 이용해 크기를 정규화하며, LayerNorm과 달리 평균을 빼는 과정은 없다.
Q와 K의 shape가 `[B, H, N, d_h]`라면 마지막 축을 정규화하고, token 수와 차원은 유지한다.
([Qwen3 구현](https://github.com/huggingface/transformers/blob/main/src/transformers/models/qwen3/modeling_qwen3.py))

## 같은 이름 안의 구현 차이

QK-Norm이 항상 같은 수식인 것은 아니다.
초기의 QKNorm 연구는 head dimension에 L2 normalization과 학습 가능한 scale을 사용했다.
ViT-22B는 Q와 K에 LayerNorm을 적용한다.
따라서 논문을 재현할 때는 normalization 종류, 적용 축, scale과 RoPE의 적용 순서를 확인해야 한다.
([QKNorm 원 논문, 2020](https://arxiv.org/abs/2010.04245), [ViT-22B, 2023](https://arxiv.org/html/2302.05442v1))

## 실제 채택: Vision과 LLM

ViT-22B 논문은 모델을 키우는 과정에서 큰 attention logit과 학습 발산을 관찰하고, QK normalization으로 이를 완화했다.
이 기법은 vision에 적용해 볼 수 있다는 추측을 넘어, 실제 대형 ViT의 학습 설계에 포함된 사례이다.

최근 LLM 쪽에서도 [Qwen3](https://arxiv.org/html/2505.09388v1)가 학습 안정성을 위해 채택했고,
[Gemma 3](https://developers.googleblog.com/gemma-explained-whats-new-in-gemma-3/)도 QK-Norm을 사용한다.
처음 나온 아이디어는 오래되었지만, 최근 모델을 읽을 때 계속 만나는 이유이다.

## Vision 실험에서의 확인 항목

Loss spike가 생긴다고 바로 QK-Norm을 추가하기보다는, attention에서 문제가 시작하는지 확인하는 편이 좋다.

- Layer별 Q/K norm과 attention logit의 크기.
- Attention entropy: weight가 몇 개 token에 지나치게 몰리는지 나타내는 지표.
- Learning rate, mixed precision, gradient norm과 loss spike의 관계.

낮은 entropy 자체가 오류는 아니다. 물체 경계나 대응점을 찾으려면 뚜렷한 attention이 유리할 수도 있다.
중요한 것은 학습 중 크기가 폭주하는지, 그 변화가 성능 저하와 함께 나타나는지이다.

Pretrained ViT에 새 norm을 넣으면 attention 분포가 바뀐다.
기존 weight를 그대로 불러온 뒤 평가만 하는 실험보다는, 원래 구조의 fine-tuning과 새 구조의 fine-tuning을 같은 조건에서 비교해야 한다.

## 정리

QK-Norm은 **무엇을 볼지 결정하는 비교 점수의 크기**를 다룬다.
새 token을 추가하거나 FFN을 키우는 방법은 아니다.
ViT 학습이 불안정하고 그 원인이 attention logit에 있다면 먼저 검토할 만한 설계이다.

[Transformer 구조 개선 시리즈 전체 목록](/2026/10/14/transformer-architecture-techniques/)
