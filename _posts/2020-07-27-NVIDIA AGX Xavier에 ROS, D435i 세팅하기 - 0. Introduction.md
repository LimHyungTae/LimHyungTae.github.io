---
layout: post
title: NVIDIA AGX Xavier에 ROS, D435i 세팅하기 - 0. Introduction
subtitle: NVIDIA AGX Xavier with Interl Realsense D435i
tags: [NVIDIA, Jetson, AGX, Xavier, ROS, D435i]
comments: true

---

# Introduction

최근 모바일 플랫폼에 NVIDIA AGX Xavier(이하 Xavier)에 Intel Realsense D435i를 부착하여 RGB+D data를 얻게 세팅을 하다 알게된 저의 삽질들과 어떻게 세팅하는 지를 공유하기 위해 이렇게 글을 작성하게 되었습니다. 잘 정리된 글들이 이미 많긴하지만, 다들 뿔뿔이 흩어져 있어서 한번 모아서 정리해 보았습니다. NVIDIA의 platform들이 프로세서가 ARM 기반이어서 특히 Intel Realsense D 시리즈와 충돌이 나서 인스톨하기 어려운데, 그러한 방법들을 해결할 수 있는 꿀팁도 이것저것 적어보았습니다. 그리고 마지막으로 이러한 충돌을 뚫고(?) ROS로 데이터를 받는 법까지 정리해보겠습니다.

Step은 크게 3가지로 구성되어 있습니다.

1. Jetpack을 활용한 NVIDIA AGX Xavier에 적절한 L4T 버전 인스톨하기

2. NVIDIA AGX Xavier 커널 재빌드하기

3. D435i 데이터를 받기 위한 realsense2_camera 패키지 설치 및 데이터를 "잘" 받기 위한 기타 Fine Tuning들

사족이라면, 딥러닝 모델을 활용해서 inference를 하기 위해 구매하시는 게 아니면 **Intel NUC가 저는 개인적으로 훨씬 더 사용하기 용이**한 것 같습니다 (ARM 기반 프로세서는 뭔가 하다보면 예기치 못한 에러를 야기해서 정신적으로(?) 힘드네요). 혹시 폼팩터 컴퓨터를 고민 중이신 분들은 참고하시면 좋을 것 같습니다.


# Reference

레퍼런스만 잘 읽어도 누구나 쉽게 설치하실 수 있습니다 :)

1. [용산동 쌍가락지의 NVIDIA 플랫폼 Setting 바이블](https://github.com/engcang/vins-application)

2. [https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html](https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html)
