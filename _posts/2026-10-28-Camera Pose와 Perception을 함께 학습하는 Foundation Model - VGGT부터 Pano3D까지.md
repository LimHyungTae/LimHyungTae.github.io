---
layout: post
title: Camera Pose와 Perception을 함께 학습하는 Foundation Model - VGGT부터 Pano3D까지
subtitle: VGGT, Pi3, CUT3R, Pano3D, GAGeo, SenseNova-Vision의 구조와 공동 학습 방식 비교
tags: [Foundation Model, Camera Pose Estimation, Multi-Task Learning, VGGT, Pano3D, 3D Vision, Research Note]
comments: true
description: Camera pose와 perception을 함께 다루는 foundation model 및 확장 연구 6편을 정리한다. Geometry와 semantics, joint training과 joint inference를 구분하고 VGGT, Pi3, CUT3R, Pano3D, GAGeo, SenseNova-Vision의 핵심 구조와 읽는 순서를 설명한다.
permalink: /2026/10/28/camera-pose-perception-foundation-models/
---

## Introduction

여러 이미지에서 카메라의 위치를 추정하는 것과 장면의 3D 구조를 복원하는 것은 서로 밀접한 문제다.
카메라가 어디에서 장면을 보았는지 알면 공간을 복원하기 쉽고, 공간 구조를 알면 카메라의 위치를 찾는 데 도움이 된다.

최근에는 이런 관계를 하나의 모델 안에서 학습하는 연구가 나오고 있다.
여기서 한 단계 더 나아가, 복원한 공간에 어떤 물체가 있는지까지 함께 예측하는 연구도 있다.

이 글에서는 **camera pose와 perception을 함께 다루는 모델 6편**을 정리한다.
핵심은 이름이나 성능 순위보다, 무엇을 공유하고 어떤 출력을 함께 학습하는지에 있다.

> Pose와 depth를 함께 예측하는 모델, geometry에 semantics를 추가한 모델, 여러 vision task를 생성 문제로 통합한 모델은 서로 구분해서 읽어야 한다.

## 사전 지식: Pose, Geometry, Semantics

여기서 pose는 사람의 관절이나 물체의 자세가 아니라 **camera pose**, 즉 카메라의 위치와 방향이다.
연속 영상에서 카메라의 움직임을 추정하는 ego-motion과도 관련되지만, 이미지 집합의 상대 pose를 추정하는 것과 장시간 odometry를 수행하는 것은 같은 평가 문제가 아니다.

Perception 역시 범위가 넓으므로 두 가지로 나누면 이해하기 쉽다.

| 구분 | 답하려는 질문 | 출력 예시 |
|---|---|---|
| Geometric perception | 공간이 어떤 형태인가? | Depth, point map, 3D reconstruction |
| Semantic perception | 공간에 무엇이 있는가? | Class label, segmentation mask, object detection |

**Point map**은 이미지의 각 pixel에 3D 좌표를 대응시킨 표현이다.
Depth map이 pixel별 깊이를 나타낸다면, point map은 정해진 좌표계에서의 위치를 나타낸다.
또 panoptic segmentation은 semantic class뿐 아니라 개별 object instance까지 구분하는 문제다.

아래 논문은 모두 새 foundation model을 처음부터 학습하는 연구는 아니다.
범용 geometric representation을 학습하는 모델과, 그 모델을 다른 task로 확장한 연구를 함께 다룬다.

---

## VGGT: Pose와 Dense Geometry의 공동 예측

**VGGT: Visual Geometry Grounded Transformer, CVPR 2025**

VGGT는 여러 RGB 이미지를 받아 camera parameter, depth map, point map, tracking에 필요한 정보를 예측한다.
입력 이미지의 token을 공유 Transformer로 처리하고, frame 내부 attention과 frame 간 정보를 교환하는 global attention을 번갈아 적용한다.

```text
여러 RGB 이미지
      |
Shared Transformer
      |-- Camera token --> Camera head --> Extrinsics / Intrinsics
      |-- Image tokens --> Dense heads --> Depth / Point map
      +-- Image tokens --> Tracking features
```

Camera extrinsics는 pose를, intrinsics는 초점거리 등 카메라의 투영 특성을 나타낸다.
장면 전체에 대한 camera parameter와 pixel별 dense prediction이 서로 다른 방식으로 feature를 읽는 구조다.

이 논문에서 볼 부분은 **관련된 기하학적 출력을 하나의 표현에서 함께 학습하는 설계**다.
다만 원래 VGGT가 semantic segmentation이나 detection까지 함께 학습하는 모델은 아니다.
'여러 3D task를 다룬다'는 설명을 'geometry와 semantics를 모두 다룬다'로 받아들이면 범위가 달라진다.
([논문](https://arxiv.org/abs/2503.11651), [공식 프로젝트](https://vgg-t.github.io/))

## Pi3: Reference View에 대한 의존성 완화

**Pi3: Permutation-Equivariant Visual Geometry Learning, ICLR 2026**

Pi3는 camera pose와 local point map을 예측하면서, 특정 이미지를 reference view로 지정하는 구조에서 벗어나려는 접근이다.
여기서 local point map은 각 이미지의 카메라 좌표계에서 표현한 3D 점이다.

핵심인 **permutation equivariance**는 입력 이미지의 순서를 바꾸면 출력도 그에 맞게 순서만 바뀌는 성질이다.
어떤 이미지를 첫 번째에 놓았는지에 따라 reconstruction 결과가 불안정해지는 문제를 줄이려는 것이다.

이 논문은 multi-task loss 조정법보다는 **공유 표현의 좌표계와 대칭성을 어떻게 설계할 것인가**에 초점이 있다.
또 뒤에서 소개할 Pano3D와 GAGeo의 기반 모델이기도 하므로, 후속 연구를 읽기 위한 연결 고리로 유용하다.
([공식 논문·코드 안내](https://github.com/yyfz/Pi3))

## CUT3R: Persistent State를 통한 연속 관측의 통합

**Continuous 3D Perception Model with Persistent State, CVPR 2025**

CUT3R는 새 이미지가 들어올 때마다 내부 state를 갱신한다.
현재 이미지의 token과 이전까지 누적한 state token이 정보를 교환하고, 그 결과로 point map과 camera parameter를 예측한다.

```text
이전 State + 현재 Image tokens
                |
       State update / Readout
                |-- 현재 Camera parameter / Point map
                +-- 다음 관측으로 전달할 State
```

핵심은 **pose와 공간 복원이 시간에 걸쳐 누적한 표현을 함께 사용한다**는 점이다.
매번 관측을 독립적으로 처리하는 대신, 이전에 본 장면 정보가 현재 예측에 반영된다.

따라서 online perception과 temporal representation을 공부할 때 읽기 좋다.
다만 이것도 기본적으로 geometric perception 모델이며, 원래 모델이 semantic segmentation까지 제공하는 것은 아니다.
또 online으로 처리한다는 사실만으로 loop closure나 장시간 drift 관리까지 갖춘 완성형 SLAM이라고 볼 수는 없다.
([논문](https://arxiv.org/abs/2501.12387), [공식 프로젝트](https://cut3r.github.io/))

---

## Pano3D: Geometric Backbone에 Semantic 이해 추가

**Pano3D: Unified 3D Reconstruction and Panoptic Segmentation, ECCV 2026**

앞선 모델이 '공간이 어떻게 생겼는가'를 다루었다면, Pano3D는 여기에 '무엇이 있는가'를 더한다.
기존 reconstruction backbone에 **query-based mask decoder**를 추가하고, geometric loss와 semantic loss를 함께 사용해 fine-tuning한다.

Encoder feature와 최종 cross-view decoder feature를 결합하며, learnable object query로 여러 view에 걸친 instance를 표현한다.
MUSt3R와 Pi3에 적용해, online 방식과 all-to-all attention 방식 모두에서 접근을 검증한다.

이 논문에서 특히 볼 부분은 **기존 geometry feature를 고정해서 쓰는 것과 semantic task까지 함께 학습하는 것의 차이**다.
저자들은 joint training이 reconstruction과 panoptic segmentation에 상호 이득을 주는 결과를 ablation으로 제시한다.

다만 핵심 기여는 reconstruction과 panoptic segmentation의 결합이다.
이를 곧바로 '독립적인 camera pose loss와 segmentation loss의 충돌을 해결한 연구'로 읽으면 안 된다.
어떤 geometry loss를 유지하고 어느 module을 업데이트하는지까지 살펴보는 것이 좋다.
([논문](https://arxiv.org/abs/2606.14307), [공식 프로젝트](https://victorbbt.github.io/Pano3D/))

## GAGeo: Pose, Box, Mask의 명시적인 결합

**Beyond 2D Matching: A Unified Single-Stage Framework for Geometry-Aware Cross-View Object Geo-Localization, ECCV 2026**

GAGeo는 camera pose와 semantic perception을 명시적으로 함께 예측하는 사례다.
Pi3를 기반으로 visual token, referring prompt, learnable task token을 결합하여 **bounding box, segmentation mask, camera position과 rotation**을 한 번의 forward pass에서 예측한다.

여기서 referring prompt는 이미지에서 찾으려는 대상을 지정하는 point, box, mask 등의 입력이다.
Task token은 출력 task를 위한 학습 가능한 token이며, 이 둘의 역할은 다르다.

다만 적용 문제는 일반적인 연속 영상의 odometry가 아니다.
지상이나 드론에서 본 특정 객체를 위성 영상과 같은 reference image에서 찾는 **cross-view object geo-localization**이다.

따라서 새 범용 foundation model보다는, 기존 geometric foundation model을 여러 출력에 맞게 확장한 연구로 보는 것이 정확하다.
읽을 때는 **task token이 공유 backbone과 어떻게 상호작용하며, 서로 다른 출력이 어떤 supervision으로 연결되는지**에 주목하면 좋다.
([논문](https://arxiv.org/abs/2606.30576), [공식 프로젝트](https://cipual.github.io/GAGeo-project-page/))

## SenseNova-Vision: 여러 Vision Task의 생성 문제 통합

**Vision as Unified Multimodal Generation, 2026년 7월 기술 보고서**

SenseNova-Vision은 task별 head를 추가하는 방식과 다른 방향을 택한다.
Pretrained multimodal model을 기반으로 detection, segmentation, depth, normal, point map, camera pose 등의 annotation을 instruction-response 형태로 학습한다.

Symbolic output은 text로, dense spatial output은 image 형태로 표현한다.
이렇게 출력 형식을 통일해 별도의 task-specific prediction head 없이 다양한 vision task를 다룬다.

다만 **하나의 checkpoint가 여러 task를 지원한다는 것과, 한 번의 forward에서 모든 task를 동시에 예측한다는 것은 다르다.**
SenseNova-Vision은 instruction으로 수행할 task와 출력 형식을 지정하는 접근이다.

이 논문은 '어디에 head를 붙일까'보다 **서로 다른 supervision을 하나의 학습·출력 인터페이스로 통합할 수 있을까**라는 관점에서 읽을 만하다.
모든 task에서 specialist보다 우수하거나, 여러 head를 병렬 실행하는 것보다 지연시간이 짧다는 뜻은 아니다.
([논문](https://arxiv.org/abs/2607.06560), [공식 코드](https://github.com/OpenSenseNova/SenseNova-Vision))

---

## 비교: 각 논문의 학습 초점

| 모델 | 중심 출력·기능 | 읽을 때 볼 부분 |
|---|---|---|
| VGGT | Camera, depth, point map, tracking | Global output과 dense output의 feature 공유 |
| Pi3 | Camera pose, local point map | Reference view와 입력 순서에 대한 구조적 성질 |
| CUT3R | 연속 관측의 camera parameter와 point map | State를 통한 시간적 정보 공유 |
| Pano3D | Reconstruction + panoptic segmentation | Geometry와 semantics의 joint fine-tuning |
| GAGeo | Cross-view box, mask, camera pose | Task token을 이용한 공동 예측 |
| SenseNova-Vision | Pose를 포함한 다양한 vision task | Instruction 기반의 출력 형식 통합 |

이 표를 하나의 성능 순위로 읽으면 안 된다.
입력 조건, 출력 범위, 학습 데이터와 평가 문제가 다르기 때문이다.
Camera pose도 상대 pose인지, 기준 지도에 대한 pose인지, metric scale이 필요한지에 따라 요구가 달라진다.

## 읽는 순서와 확인할 실험

기본 구조를 익히려면 **VGGT와 Pi3**부터 읽고, geometry와 semantics의 공동 학습으로 관심을 넓히려면 **Pano3D와 GAGeo**를 이어서 읽는 순서가 자연스럽다.
연속 영상의 state가 궁금하면 CUT3R, 범용 모델의 task 통합이 궁금하면 SenseNova-Vision으로 확장하면 된다.

Multi-task learning 관점에서는 다음을 확인하는 것이 중요하다.

- **공유 범위:** Encoder, cross-view decoder, task head 중 무엇을 함께 업데이트하는가.
- **Supervision:** 여러 출력을 지원하는 것과 실제로 여러 loss로 공동 학습하는 것이 일치하는가.
- **데이터 구성:** 같은 sample에 모든 label이 있는가, task별 dataset을 섞는가.
- **Ablation:** Frozen backbone, joint fine-tuning, single-task 설정을 비교했는가.
- **평가 조건:** Pose와 perception의 validation 성능을 각각 보고하는가.

Task A와 task B의 loss를 더했다는 사실만으로 서로 도움을 준다고 결론 내릴 수는 없다.
반대로 jointly trained model의 성능이 높다고 해서, 그 이유가 특정 gradient balancing 기법에 있다고 추정해서도 안 된다.
구조, pretraining, 데이터, loss의 기여를 구분해서 봐야 한다.

정리하면 이 분야는 **geometry를 함께 배우는 단계에서, semantics와 시간적 정보까지 공유하는 단계로 확장**되고 있다.
어떤 모델이 가장 최신인가보다, 어떤 정보를 공유해야 여러 task에 실제로 도움이 되는지 살펴보면 논문 사이의 연결이 더 잘 보인다.
