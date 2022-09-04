---
layout: post
title: SLAM, Robotics 프로그래밍을 위한 3가지 코딩꿀팁
subtitle: 이웃에게 친절한 SLAM 개발자가 되자
tags: [SLAM, OpenCV, Ubuntu, ROS]
comments: true
---

# 서론

요즘은 SLAM open source 코드도 많고, SLAM 및 mapping 관련 업무를 다루는 회사도 늘어나다보니 다른 동료들과 함께 코드를 짜야할 일이 생긴다. 그러다 보면 소통이 원활하게 되지 않아서 의도치 않게 서로에게 스트레스를 주는 몇가지 부분들이 있다.

사실 제목은 거창하게 지었으나, 더 정확히 말하자면 '동료 SLAM 개발자들과 함께 코딩할 때 그들의 편의성을 올려주는 방법'이 더 부합한 거 같다. 하지만 어그로(?)를 위해 제목을 3가지 코딩꿀팁이라 남겨둔다.

이 분야에 오래 종사한 사람어도 깊게 생각해보지 않은 부분들이 있을 수 있기 때문에, 생산성 향상을 위해 한 번쯤 읽어보면 도움이 될 글이라고 생각된다.


### 1. Pose가 무엇을 기준인지 명시 



`T_LiDAR_TO_CAM`라는 변수가 있다고 가정하자 (transformation matrix가 친숙하지 않은 분은 [여기](https://limhyungtae.github.io/2021-09-10-ROS-Point-Cloud-Library-(PCL)-3.-Transformation/) 참조).

하지만, 여기서 `to`의 의미가 SLAM / Robotics 분야에서 혼동해서 사용하고 있다는 것을 알게 되었다.

i) 

ii)

### 2. Quaternion, Eular angle의 순서를 잘 명시하자


### 3. 위의 두 사항을 기반으로 Extrinsic을 어딘가에 잘 기록해두자

