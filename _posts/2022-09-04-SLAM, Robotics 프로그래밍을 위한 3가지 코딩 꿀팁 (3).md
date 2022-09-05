---
layout: post
title: SLAM, Robotics 프로그래밍을 위한 3가지 코딩 꿀팁 (3)
subtitle: 3. Extrinsic 및 pose 추정의 기준 frame을 명시하자
tags: [SLAM, Robotics, ROS]
comments: true
---

(Cont'd)


### 3. Extrinsic 및 pose 추정의 기준 frame을 분명하게 명시하자

마지막 꿀팁은, pose의 기준 frame을 README나 class 내부에 잘 명시해두자는 것이다. 왜냐하면 알고리즘마다 기준으로 잡는 축이 다 다르기 때문이다. 얘기로 하면 잘 와닿지 않아서 바로 예제를 준비했다. 대표적인 예시로 아래 LIO-SAM의 `/config/params.yaml` 일부분과 FAST-LIO의 `config/avia.yaml`의 일부분을 살펴보자 (extrinsic에 주목해서 살펴보자):

* `/config/params.yaml`의 일부분 in LIO-SAM
  
```
# Extrinsics: T_lb (lidar -> imu)
  extrinsicTrans: [0.0, 0.0, 0.0]
  extrinsicRot: [-1, 0, 0,
                  0, 1, 0,
                  0, 0, -1]
  extrinsicRPY: [0, -1, 0,
                 1, 0, 0,
                 0, 0, 1]
```

* `config/avia.yaml`의 일부분 in FAST-LIO
  
```
mapping:
    ...     
    extrinsic_T: [ 0.04165, 0.02326, -0.0284 ]
    extrinsic_R: [ 1, 0, 0,
                   0, 1, 0,
                   0, 0, 1]

```

차이점을 느꼈는가? **느끼긴 무슨!** config 파일만 보면 두 라이브러리 모두 extrinsic을 할당해줄 뿐 extrinsic이 무엇을 의미하는지 명시해두지 않았다. **그.런.데 두 알고리즘에서 추정하는 pose의 기준 좌표계가 정반대여서, extrinsic 또한 정반대의 의미를 지닌다**. 
코드 내부를 깊게 살펴 보면 LIO-SAM에서는 world frame 기준 LiDAR 좌표계의 pose를 estimation하는 반면, FAST-LIO에서는 IMU 좌표계의 pose를 estimation하는 것을 확인할 수 있다. **그럼에도 불구하고 두 알고리즘의 extrinsic이 둘다 `extrinsic`이라고 덩그러니 되어있다(!)**. 


이러한 문제는 사용자를 상당히 귀찮게 하는데, 자기 자신이 취득한 데이터로 각각 LIO-SAM과 FAST-LIO를 돌려봐야할 때 (즉, 적절한 extrinsic을 데이터를 취득할 때 세팅한 센서 셋업에 맞게 수정해줘야 할 때), 해당 파라미터가 사용되는 부분의 코드를 한 번 살펴봐야 하기 때문이다. 그로 인해 그냥 한 번 돌려보려고 한 사용자 입장에서는, 코드 내부까지 꼼꼼히 살펴본 후 파라미터를 설정해줘야 하기 때문에 꽤나 피로감을 느끼게 된다.

---
**참고:** 코드를 예시로 살펴보자면, FAST-LIO의 `laser_mapping.cpp` 내부 코드를 살펴보면 config에서 설정한 `extrinsic_T`와 `extrinsic_R`이 LiDAR point cloud를 IMU 기준으로(즉, with respect to the IMU frame)으로 변환하는 변수였다는 것을 알 수 있다 ([꿀팁1]()에서 언급했던 것처럼, 이렇게 `wrt`로 적으면 해당 코드를 읽는 사람이 transformation matrix 변수의 역할에 대해 좀 더 명확히 이해할 수 있다).  

```cpp
nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
.
.
.
Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
```


### (3)의 결론 및 해결책

그렇기 때문에 extrinsic을 표현할 때와 다중 센서를 활용하여 pose를 추정할 때에는 알고리즘의 기준 frame이 뭔지 명시하는 습관을 들이면 좋을 듯 하다. 지금의 나라면 위의 config를 다음과 같이 작성하지 않았을까 싶다. 주석에 "HT:"라 되어 있는 부분 참고. 

* 주석이 추가된 `/config/params.yaml`의 일부분 in LIO-SAM
  
```
# Extrinsics: T_lb (lidar -> imu)
# HT: Transform imu data into the lidar frame
# HT: s.t. T_lb * p_wrt_imu -> p_raw_lidar 
  extrinsicTrans: [0.0, 0.0, 0.0]
  extrinsicRot: [-1, 0, 0,
                  0, 1, 0,
                  0, 0, -1]
  extrinsicRPY: [0, -1, 0,
                 1, 0, 0,
                 0, 0, 1]
```

* 주석이 추가된 `config/avia.yaml`의 일부분 in FAST-LIO
  
```
mapping:
    ...   
    # HT: Transform cloud points from the lidar frame into the imu frame
    # HT: s.t. extrinsic * p_raw_lidar -> p_wrt_imu 
    extrinsic_T: [ 0.04165, 0.02326, -0.0284 ]
    extrinsic_R: [ 1, 0, 0,
                   0, 1, 0,
                   0, 0, 1]
```

# 최종 결론

이 세가지 팁은 말그대로 '팁'일 뿐, 누군가가 따르지 않는다고 해서 틀린 것은 아니다 (즉, wrong-and-right를 판별할 수는 없다). 하지만 코드를 나타낼 때 더 좋고 나쁘고의 영역(worse-and-better)은 분명히 있기에, 코드를 작성할 때 앞서 얘기한 세가지 꿀팁을 잘 참고하면 좋겠다. 사실, 이 세 꿀팁은 출처가 없고, 필자가 SLAM 분야를 연구를 하면서 불편하다고 느낀 것들을 카테고리화한 것이다. 모쪼록 끝까지 읽은 독자분들은 이웃에게 친절한 SLAM 개발자가 되길 응원한다. 끗.

---


SLAM, Robotics 프로그래밍을 위한 3가지 코딩 꿀팁 시리즈입니다.

1. [SLAM, Robotics 프로그래밍을 위한 3가지 코딩꿀팁 (1)](https://limhyungtae.github.io/2022-09-04-SLAM,-Robotics-%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D%EC%9D%84-%EC%9C%84%ED%95%9C-3%EA%B0%80%EC%A7%80-%EC%BD%94%EB%94%A9-%EA%BF%80%ED%8C%81-(1)/)
2. [SLAM, Robotics 프로그래밍을 위한 3가지 코딩꿀팁 (2)](https://limhyungtae.github.io/2022-09-04-SLAM,-Robotics-%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D%EC%9D%84-%EC%9C%84%ED%95%9C-3%EA%B0%80%EC%A7%80-%EC%BD%94%EB%94%A9-%EA%BF%80%ED%8C%81-(2)/)
3. [SLAM, Robotics 프로그래밍을 위한 3가지 코딩꿀팁 (3)](https://limhyungtae.github.io/2022-09-04-SLAM,-Robotics-%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D%EC%9D%84-%EC%9C%84%ED%95%9C-3%EA%B0%80%EC%A7%80-%EC%BD%94%EB%94%A9-%EA%BF%80%ED%8C%81-(3)/)





