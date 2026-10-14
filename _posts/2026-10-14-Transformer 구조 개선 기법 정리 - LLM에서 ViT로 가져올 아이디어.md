---
layout: post
title: Transformer 구조 개선 기법 정리 - LLM에서 ViT로 가져올 아이디어
subtitle: QK-Norm, 2D RoPE, SwiGLU부터 Gated Attention까지
tags: [Transformer, Vision Transformer, LLM, Architecture, Research Note]
comments: true
description: 최근 주요 Transformer에서 채택한 구조 개선 기법 여섯 가지를 비교한다. 실제 vision 적용 근거와 LLM에서 가져와 실험할 후보를 구분하고 개별 설명 글로 연결한다.
permalink: /2026/10/14/transformer-architecture-techniques/
---

## Introduction

Register token을 공부하고 나면 비슷한 질문이 생긴다.
Transformer의 큰 틀을 유지하면서, 내부를 조금 바꾸어 성능이나 학습 안정성을 개선할 방법은 또 없을까?
특히 LLM에서 잘 쓰는 기법이라면 vision에도 가져올 수 있지 않을까?

내가 이해한 핵심은 다음과 같다.

> Attention만 새로 설계할 필요는 없다. Q와 K를 정규화하는 위치, FFN의 구조, residual을 더하는 방식도 중요한 설계 선택이다.

이 시리즈는 **2026년 10월 14일 기준 확인한 원 논문과 공식 구현**을 바탕으로 작성했다.
최근 주요 모델의 구성 요소를 살펴보되, 모든 기법이 최근에 처음 제안된 것은 아니라는 점을 먼저 구분한다.

## 전체 목록: 무엇을 바꾸는가

| 개별 글 | 바꾸는 부분 | 근거와 적용 범위 |
|---|---|---|
| [QK-Norm](/2026/10/14/qk-norm-transformer/) | Attention score를 만드는 Q와 K | ViT-22B, Qwen3, Gemma 3에서 채택 |
| [2D RoPE](/2026/10/14/rope-2d-vision-transformer/) | Attention에 위치를 넣는 방식 | RoPE-ViT 연구와 DINOv3에서 vision 적용 확인 |
| [SwiGLU](/2026/10/14/swiglu-transformer-ffn/) | Token별 FFN | Qwen3와 DINOv3의 일부 variant에서 채택 |
| [GQA](/2026/10/14/gqa-vision-transformer/) | Query head 사이의 K/V 공유 | LLM에서 널리 채택, 일반 ViT에서는 이점 재검증 필요 |
| [LayerScale](/2026/10/14/layerscale-vision-transformer/) | Residual branch의 channel별 배율 | CaiT에서 제안, DINOv3에도 사용되는 vision 측 비교 기준 |
| [Gated Attention](/2026/10/14/gated-attention-vision-transformer/) | Attention output을 통과시키는 양 | Qwen3-Next와 Qwen3.5 계열에서 채택, vision 이식은 연구 후보 |

채택 근거는 [Qwen3 보고서](https://arxiv.org/html/2505.09388v1), [Gemma 3 설명](https://developers.googleblog.com/gemma-explained-whats-new-in-gemma-3/), [DINOv3 구현](https://github.com/facebookresearch/dinov3/blob/main/dinov3/hub/backbones.py), [Qwen3.5 model card](https://huggingface.co/Qwen/Qwen3.5-35B-A3B)에서 확인했다.

LayerScale은 LLM에서 넘어온 최신 기법으로 소개하는 것이 아니다.
Vision에서 이미 쓰는 residual 조절 방식으로서, 입력에 따라 조절하는 Gated Attention과 비교하기 좋아 함께 넣었다.

## 최근 모델의 채택과 성능 향상의 구분

어떤 SOTA 모델에 들어 있다는 사실만으로, 그 기법 하나가 SOTA를 만들었다고 말할 수는 없다.
데이터와 학습량, 모델 크기, 다른 구성 요소도 함께 바뀌기 때문이다.

예를 들어 GQA는 주로 inference 효율을 위한 선택이다.
정확도를 무조건 높이는 장치로 보면 목적을 잘못 이해하게 된다.
반대로 QK-Norm은 큰 attention logit 때문에 학습이 불안정한 상황에서 의미가 있다.
이미 안정적인 작은 모델에서 같은 개선 폭이 나올지는 별도 실험이 필요하다.

## Vision 연구에서의 적용 순서

내가 ViT를 개선하는 실험을 시작한다면 다음처럼 문제에 맞춰 선택할 것 같다.

| 현재 관찰 | 먼저 볼 후보 |
|---|---|
| Attention logit이 커지고 loss spike가 반복됨 | QK-Norm |
| 학습과 평가의 이미지 해상도가 다름 | 2D RoPE와 좌표 처리 |
| FFN 표현력을 바꾸고 싶음 | 동일 parameter budget의 SwiGLU |
| 깊은 encoder나 새 residual module의 학습이 불안정함 | LayerScale |
| K/V 저장이나 projection 비용이 병목임 | GQA |
| Attention 결과를 입력에 따라 조절하고 싶음 | Gated Attention |

이 순서는 benchmark 순위가 아니라, 각 기법이 바꾸는 부분을 기준으로 한 실험 제안이다.
지금 관심사에는 **QK-Norm → 2D RoPE → SwiGLU → Gated Attention** 순서로 읽는 것이 가장 직접적이다.

## 실험 전 확인: Pretrained Model의 구조

DINOv3 같은 pretrained backbone을 쓴다면 먼저 어떤 기법이 이미 들어 있는지 확인해야 한다.
기존 모델에 같은 기능을 중복 추가하는 것은 새로운 개선이 아니다.

또한 GELU FFN을 SwiGLU로 교체하거나 positional encoding을 바꾸면 pretrained model의 함수가 달라진다.
Weight를 일부 불러올 수 있다는 것과 기존 표현을 유지한다는 것은 다르다.
새 구조를 처음부터 학습하는 실험과 pretrained backbone을 수정하는 실험은 구분해 기록하는 편이 좋다.

각 실험에서는 parameter 수, FLOPs, 실제 latency, peak memory와 task metric을 함께 비교한다.
Segmentation이라면 image-level 성능뿐 아니라 작은 물체와 경계의 성능도 봐야 한다.
기법 하나씩 바꾼 뒤 조합해야 어떤 변경이 도움이 되었는지 해석할 수 있다.

## 범위: 구조 변경과 실행 최적화

이 시리즈는 Transformer의 계산 구조를 바꾸는 방법에 집중한다.
FlashAttention처럼 같은 attention을 더 효율적으로 실행하는 kernel 기법, LoRA처럼 적은 parameter만 학습하는 fine-tuning 방법도 중요하지만 질문의 축이 조금 다르다.
Register token 역시 이 목록과 함께 읽을 수 있는 [별도의 구조 변경 사례](/2026/10/13/what-is-register-token/)이다.
