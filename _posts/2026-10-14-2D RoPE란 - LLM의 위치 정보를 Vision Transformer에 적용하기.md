---
layout: post
title: 2D RoPE란? LLM의 위치 정보를 Vision Transformer에 적용하기
subtitle: Token의 위치를 Q와 K의 회전으로 표현하는 방법
tags: [RoPE, Positional Encoding, Vision Transformer, DINOv3, LLM]
comments: true
description: RoPE의 상대 위치 표현과 2D 확장을 설명한다. RoPE-ViT와 DINOv3의 사례를 통해 해상도 변화, 좌표 처리, special token 적용 시 확인할 점을 정리한다.
permalink: /2026/10/14/rope-2d-vision-transformer/
---

## Introduction

이미지를 patch로 나누고 token으로 바꾸면 각 patch에 무엇이 있는지는 표현할 수 있다.
하지만 어느 patch가 위쪽에 있고, 어느 patch가 바로 옆에 있는지도 모델이 알아야 한다.
그래서 Transformer에는 위치를 전달하는 장치가 필요하다.

내가 이해한 RoPE의 핵심은 다음과 같다.

> RoPE는 위치에 따라 Q와 K를 회전시켜, 두 token을 비교할 때 상대적인 위치 차이도 반영하게 한다.

## 사전 지식: 위치를 더하는 방식

기본적인 learned positional embedding은 각 위치의 벡터를 token에 더한다.

```text
Patch embedding + 해당 위치의 embedding -> Transformer
```

특정 크기의 patch grid로 학습했다면, 평가 해상도를 바꿀 때 위치 embedding을 보간하는 등의 처리가 필요하다.
RoPE는 Q와 K를 만든 뒤 위치에 따라 변환하는 방식이다.

```text
Token -> Q, K projection -> 위치에 따른 회전 -> Attention score
```

## RoPE의 직관: 회전 각도의 차이

RoPE는 벡터의 두 성분씩을 한 쌍으로 보고 회전시킨다.
회전 각도는 token 위치와 주파수에 따라 정해진다.
표준 1D 형태에서는 위치 $$m$$, $$n$$의 Q와 K 사이에 다음 관계가 성립한다.

$$
(R_mq)^\top(R_nk)=q^\top R_{n-m}k
$$

수식을 모두 외울 필요는 없다.
**각 token은 자신의 위치로 회전하지만, 두 벡터를 비교한 결과에는 위치 차이가 들어간다**는 뜻이다.
원래 벡터의 내용도 남아 있으므로 attention이 거리만 보고 결정되는 것은 아니다.
([RoFormer, 2021](https://arxiv.org/abs/2104.09864))

RoPE를 적용해도 token 개수와 벡터 차원은 변하지 않는다.
새 정보를 뒤에 이어 붙여 차원을 늘리는 것이 아니라, 기존 성분들을 변환한다.

## 2D 확장: 가로와 세로의 구분

텍스트에는 주로 순서 하나가 있지만, 이미지 patch에는 행과 열이 있다.
단순히 이미지를 길게 펼친 순번만 사용하면 행 경계에서 공간 관계를 설명하기가 어색해진다.

```text
2D patch 좌표:
(0,0) (0,1) (0,2)
(1,0) (1,1) (1,2)
```

2D RoPE는 가로·세로 좌표를 회전에 반영한다.
Axial 방식은 벡터 성분을 나누어 각 축을 처리하고, mixed 방식은 회전 주파수에 두 축의 영향을 함께 넣을 수 있다.
**2D RoPE라는 이름 아래에도 좌표와 주파수를 설계하는 여러 방식이 있다.**
([RoPE-ViT, ECCV 2024](https://arxiv.org/abs/2403.13298))

## 실제 채택: DINOv3

RoPE는 LLM뿐 아니라 DINOv3의 ViT에도 들어간다.
DINOv3는 서로 다른 해상도의 이미지를 처리하는 데 RoPE를 활용한다.
공식 구현에는 좌표 정규화와 rescaling 설정도 있으므로, '행 번호와 열 번호를 그냥 넣는다'고만 이해하면 실제 모델과 차이가 생긴다.
([DINOv3 보고서](https://arxiv.org/html/2508.10104v1), [좌표 처리 구현](https://github.com/facebookresearch/dinov3/blob/main/dinov3/layers/rope_position_encoding.py))

여기서 일반화할 수 있는 것은 **위치 테이블 크기에 묶이지 않는 설계가 해상도 변경을 다루기 편하게 한다**는 점이다.
RoPE를 넣으면 학습하지 않은 모든 해상도에서 성능이 유지된다는 뜻은 아니다.
물체 크기, patch 수, 좌표 범위가 함께 달라지므로 실제 평가가 필요하다.

## Vision 적용 시의 확인 항목

| 항목 | 확인할 내용 |
|---|---|
| 좌표 기준 | Pixel 좌표인지 patch 좌표인지, 정규화 여부 |
| 축 처리 | Axial인지 mixed인지, 각 축에 배정한 차원 |
| Special token | CLS와 register처럼 공간 위치가 없는 token의 처리 |
| 평가 해상도 | 정사각형뿐 아니라 종횡비가 다른 이미지의 성능 |

CLS와 register에 임의의 patch 좌표를 부여하는 것은 모델의 원래 설계와 다를 수 있다.
Pretrained model을 수정한다면 기존 positional encoding과 어떤 관계로 사용할지도 정해야 한다.
DINOv3처럼 이미 RoPE를 사용하는 모델에 다시 추가하는 것은 개선 실험이 아니다.

## 정리

2D RoPE는 **어느 token을 볼지 계산하는 과정에 공간 관계를 반영하는 방법**이다.
해상도 변경이 잦은 segmentation이나 여러 크기의 이미지를 다루는 ViT를 공부할 때 특히 유용하다.

[Transformer 구조 개선 시리즈 전체 목록](/2026/10/14/transformer-architecture-techniques/)
