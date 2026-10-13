---
layout: post
title: TensorRT IPluginV3 쉽게 이해하기 - BEV Pooling을 예시로
subtitle: 직접 만든 GPU 연산을 TensorRT의 layer로 연결하는 방법
tags: [TensorRT, IPluginV3, BEV, CUDA, Deployment, Research Note]
comments: true
description: NVIDIA의 BEV pooling 최적화 사례를 통해 TensorRT plugin과 IPluginV3의 역할을 쉽게 설명한다. GPU kernel과 plugin의 차이, build와 runtime, plugin creator의 의미를 정리한다.
permalink: /2026/10/13/tensorrt-ipluginv3-bev-pooling/
---

## Introduction

NVIDIA의 [BEV pooling 최적화 글](https://developer.nvidia.com/blog/accelerating-bev-pooling-on-nvidia-gpus-for-physical-ai-applications/)을 보면 **IPluginV3**라는 이름이 나온다.
BEV나 perception은 익숙해도, TensorRT 내부를 잘 모르면 여기서부터 조금 막힌다.
CUDA로 연산을 빠르게 만들었다는 것은 알겠는데, plugin은 왜 또 필요한 것일까?

내가 이해한 핵심은 다음과 같다.

> IPluginV3는 직접 구현한 연산을 TensorRT가 하나의 layer로 사용할 수 있도록 연결하는 인터페이스이다.

즉 GPU에서 계산하는 코드와, 그 계산을 모델 안에 넣는 약속을 구분하면 이해하기 쉽다.

---

## 사전 지식: TensorRT와 Engine

학습이 끝난 모델을 실제 서비스나 로봇에서 실행하는 과정을 **inference**라고 한다.
TensorRT는 이 inference를 NVIDIA GPU에서 효율적으로 실행하기 위한 도구이다.

모델을 받으면 연산을 연결하고 실행 방법을 선택해 **engine**을 만든다.
Engine은 배포 환경에서 모델을 실행하기 위한 준비된 결과물이라고 생각하면 된다.

여기에는 두 단계가 있다.

```text
Build:   모델을 분석하고 실행할 engine을 준비
Runtime: 준비된 engine에 실제 입력을 넣어 추론
```

그런데 개발자가 직접 만든 특수 연산까지 TensorRT가 모두 알고 있을 수는 없다.
이런 연산을 연결하는 확장 기능이 **plugin**이다.
기본 연산으로 표현할 수 있더라도, 특정 작업에 맞춘 구현을 사용하기 위해 plugin을 선택할 수도 있다.

---

## 예시: BEV Pooling의 역할

BEV는 **Bird's-Eye View**, 즉 위에서 내려다본 공간 표현이다.
여러 카메라에서 추출한 feature를 공통 BEV grid에 모으면, downstream module이 같은 공간 기준으로 정보를 사용할 수 있다.

NVIDIA 글에서 다루는 BEV pooling은 image feature에 depth 정보를 가중치로 적용하고, 해당 BEV cell에 모아 합산하는 연산이다.
예를 들어 서로 다른 카메라의 feature가 같은 cell에 기여한다면, 그 기여를 한곳에 누적한다.

```text
여러 카메라의 image feature + depth 정보
                  |
             BEV pooling
                  |
           공통 BEV feature map
                  |
         Detection / Occupancy 등
```

개념은 단순하지만 GPU에서는 흩어진 데이터를 읽고 같은 위치로 모으는 과정이 병목이 될 수 있다.
NVIDIA 글은 중복된 메모리 접근을 줄이고, GPU의 cache 특성에 맞게 kernel을 구성한다.
그리고 이 구현을 **IPluginV3 기반 plugin으로 TensorRT에 연결**한다.
([NVIDIA 원문](https://developer.nvidia.com/blog/accelerating-bev-pooling-on-nvidia-gpus-for-physical-ai-applications/))

---

## CUDA Kernel과 IPluginV3의 역할 차이

**CUDA kernel**은 GPU에서 실제 계산을 수행하는 함수이다.
하지만 계산 함수만 있다고 TensorRT가 그 함수를 언제, 어떤 입력으로 호출해야 하는지 알 수 있는 것은 아니다.

| 구성 요소 | 담당 역할 |
|---|---|
| CUDA kernel | Feature를 읽고 BEV cell별로 누적하는 실제 계산 |
| IPluginV3 기반 plugin | 입력 조건, 출력 형태, 실행 방법을 TensorRT에 제공 |
| TensorRT engine | Plugin을 앞뒤 layer와 연결해 전체 추론 실행 |

비유하자면 kernel은 작업을 수행하는 장비이고, plugin은 그 장비의 연결 규격과 사용 절차이다.
어떤 재료를 넣고 어떤 결과가 나오는지 알아야 전체 공정에 넣을 수 있다.

따라서 **IPluginV3를 사용한다는 사실만으로 연산이 빨라지는 것은 아니다.**
BEVPoolV3의 성능 개선은 kernel과 데이터 접근 방식의 최적화에서 나온다.
Plugin은 그 구현을 TensorRT 안에서 사용할 수 있게 한다.
또한 **BEVPoolV3의 V3와 IPluginV3의 V3는 각각 다른 대상의 버전**이다.

---

## IPluginV3의 세 가지 역할

공식 문서는 IPluginV3를 core, build, runtime capability를 묶는 인터페이스로 설명한다.
이름은 길지만, TensorRT가 plugin에 묻는 질문으로 바꾸면 간단하다.

| Capability | TensorRT가 확인하는 내용 |
|---|---|
| `IPluginV3OneCore` | 이 plugin의 이름과 버전은 무엇인가? |
| `IPluginV3OneBuild` | 어떤 입력 형식을 지원하고, 출력 tensor는 어떤 형태인가? |
| `IPluginV3OneRuntime` | 실제 입력이 준비되었을 때 어떻게 실행하는가? |

Build 단계에서는 앞뒤 layer와 연결 가능한지 확인해야 한다.
예를 들어 앞 layer가 FP16 tensor를 출력한다면, plugin도 그 입력을 처리할 수 있어야 한다.

실행할 때는 TensorRT가 입력·출력 메모리와 CUDA stream을 전달하고, plugin의 **`enqueue()`**를 호출한다.
Stream은 GPU 작업을 제출할 실행 흐름이라고 보면 된다.
Plugin은 이 흐름에 kernel 실행을 넣어 결과를 출력 메모리에 기록한다.

```text
앞 layer의 output
      |
TensorRT가 plugin.enqueue() 호출
      |
Plugin이 CUDA kernel 실행 요청
      |
결과를 다음 layer에서 사용
```

Runtime capability는 engine을 만드는 중 성능을 측정할 때도 쓰일 수 있다.
따라서 '배포 후에만 쓰는 기능'이라고 이해하면 조금 부정확하다.
([공식 API 설명](https://docs.nvidia.com/deeplearning/tensorrt/latest/inference-library/plugins-api-migration.html))

---

## 배포 관점: Plugin Creator와 Registry

Engine을 저장했다가 다시 불러올 때도 plugin을 복원해야 한다.
이때 **creator**는 plugin 객체를 만드는 역할이고, **registry**는 해당 creator를 찾아주는 목록이다.

TensorRT는 저장된 plugin 이름·버전·namespace와 설정을 이용해 적절한 creator를 찾는다.
따라서 배포 환경에서도 필요한 plugin 구현을 사용할 수 있어야 한다.
Engine을 저장했다고 임의의 custom CUDA 코드까지 자동으로 모두 해결된다고 생각하면 안 된다.
([공식 Plugin 문서](https://docs.nvidia.com/deeplearning/tensorrt/latest/inference-library/plugins-cpp.html))

## 정리

BEV pooling 사례에서 기억할 것은 세 가지이다.

- **BEV pooling**: 어떤 계산이 필요한가.
- **CUDA kernel**: 그 계산을 GPU에서 어떻게 수행하는가.
- **IPluginV3**: 그 구현을 TensorRT가 어떻게 연결하고 호출하는가.

이렇게 나누어 보면 IPluginV3는 새로운 perception 알고리즘이 아니라,
연구 코드의 특수 연산을 배포용 inference engine으로 가져오기 위한 접점이라고 이해할 수 있다.
