---
layout: post
title: NVIDIA AGX Xavier에 ROS, D435i 세팅하기 - 1. Jetpack을 활용한 NVIDIA AGX Xavier에 적절한 L4T 버전 인스톨하기
subtitle: NVIDIA AGX Xavier with Interl Realsense D435i
tags: [NVIDIA, Jetson, AGX, Xavier, ROS, D435i]
comments: true

---
1. Developer Kit 16GB여야 함! Jetpack 8GB에는 Jetpack 4.2.1못 까는 듯

# Jetpack을 통한 설치

1. [Jetpack 공식 사이트](https://developer.nvidia.com/embedded/jetpack)에 들어가서 

[Jetpack Archive](https://developer.nvidia.com/embedded/jetpack-archive)

[이 블로그](https://m.blog.naver.com/PostView.nhn?blogId=ambidext&logNo=221560535815&categoryNo=23&proxyReferer=https:%2F%2Fwww.google.com%2F)


Ubuntu 18.04 NVIDIA Jetson AGX Xavier [16GB]

```
L4T 32.1.0 [Jetpack 4.2]
```


1. Jetpack을 활용한 NVIDIA AGX Xavier에 적절한 L4T 버전 인스톨하기

2. NVIDIA AGX Xavier 커널 재빌드하기

3. D435i 데이터를 받기 위한 realsense2_camera 패키지 설치 및 데이터를 "잘" 받기 위한 기타 Fine Tuning들

사족이라면, 딥러닝 모델을 활용해서 inference를 하기 위해 구매하시는 게 아니면 **Intel NUC가 저는 개인적으로 훨씬 더 사용하기 용이**한 것 같습니다 (ARM 기반 프로세서는 뭔가 하다보면 예기치 못한 에러를 야기해서 정신적으로(?) 힘드네요). 혹시 폼팩터 컴퓨터를 고민 중이신 분들은 참고하시면 좋을 것 같습니다.

