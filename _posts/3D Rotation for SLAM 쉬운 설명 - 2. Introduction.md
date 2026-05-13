---
layout: post
title: 3D Rotation for SLAM 쉬운 설명 - 2. Introduction
subtitle: Preview and Preliminaries
tags: [SLAM, Pose Estimation, 3D Geometry, Rotation]
comments: true
description: SLAM과 robotics 관점에서 Euler angle과 quaternion을 비교하고 gimbal lock에 대한 흔한 오해를 짚으며 회전 표현의 직관을 잡는다.
image: /img/rotation/gimbal_lock.jpeg
permalink: /2022/04/10/3d-rotation-for-slam-02-introduction/
  - '/3D-Rotation-for-SLAM-쉬운-설명-2.-Introduction/'
redirect_from:
  - '/2022-04-10-3D Rotation for SLAM 쉬운 설명 - 2. Introduction/'
  - '/2022-04-10-3D-Rotation-for-SLAM-쉬운-설명-2.-Introduction/'
---

# Rotation의 for Robotics 쉬운 설명

Robotics 분야에 종사하는 사람의 관점에서 이러한 회전은 아래와 같이 해석될 수 있다.

* Q. Euler angle → Quaternion을 쓰는 이유가 짐벌락 (gimbal lock)을 피하기 위해서이다?
  * 이건 반은 맞고 반은 틀렸다고 생각한다 (100% 개인적인 의견). 왜냐하면, 실제로 짐벌락이 일어나는 robot의 motion이 거의 없기 때문이다. 그래픽스에서는 종종 일어날 수 있는 문제이긴 하지만, 아래의 그림처럼 robot이 roll로 90도, pitch로 회전해야 하는 일이 일어날까?
  * 오히려 그것보다는

![짐벌락 예시](/img/rotation/gimbal_lock.jpeg)

여러분이 오일러 앵글을 사용한다면 여러분은 짐벌락 문제를 항상 염두에 두어야만 합니다.
오일러 앵글을 사용하면서 얻어지는 장점은 오일러 앵글을 사용하면 쿼터니언 회전보다 그래픽 아티스트들이 이해하기가 매우 쉽다는 점입니다.

쿼터니언 회전은 훨씬 더 강력하며 튼튼합니다.
Max에서 TCB 컨트롤러는 방향을 구하기 위해서 쿼터니언 수학을 사용합니다.
쿼터니언 회전은 회전량을 나타내는 하나의 값과 회전축을 나타내는 세 개의 값으로 구성됩니다.
