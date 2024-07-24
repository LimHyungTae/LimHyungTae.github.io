---
layout: post
title: ROS tf_echo 활용한 extrinsic 구하기
subtitle: ROS tf_echo 활용한 extrinsic 구하기
tags: [ROS, C++, RViz, tf]
comments: true
---

이번 글에서는 ROS tf_echo를 활용하여 extrinsic을 구하는 방법에 대해 알아보겠다. 각 센서 frame의 TF가 존재한다면, 이를 활용하여 두 frame 사이의 extrinsic을 구할 수 있다.

## How to use

```angular2html
rosrun tf tf_echo ${FRAME_A} ${FRAME_B}
```

라고 입력하면 T_AB를 구할 수 있다. 즉, B frame을 A frame으로 변환하는 행렬(Tranformation of B w.r.t. A)이다.

## Example 1

Robot에 Ouster LiDAR를 장착했다고 가정하고, 아래와 같이 입력을 하면 아래와 같은 값을 얻을 수 있다 (참고: ROS에서는 (qx, qy, qz, qw) 순서로 quaternion을 표기한다. 그러나 몇몇 library에서는 (qw, qx, qy, qz) 순서인 경우도 있기 때문에 quaternion의 순서를 늘 유심히 살펴봐야 함).
```angular2html
rosrun tf tf_echo apis/base_link apis/os_lidar
# Output
At time 0.000
- Translation: [0.120, 0.000, 0.351]
- Rotation: in Quaternion [0.000, 0.000, 1.000, 0.000]
            in RPY (radian) [0.000, -0.000, 3.142] 
            in RPY (degree) [0.000, -0.000, 180.000]
```

이는 합당한데, LiDAR가 base_link에서 0.12m 앞, 0.351m 위에 위치하고 있음을 나타내기 때문이다. 그리고 Ouster는 Velodyne과 다르게 센서 기준 앞쪽 방향이 -X이다. 따라서 180도 회전(pi만큼 회전해서 radian에서 3.142가 찍힌 것을 볼 수 있음)한 것도 나타나진다.
이처럼 extrsinsic을 구할 때에는, 센서의 설치 방향을 기반으로 출력된 값이 물리적으로 make sense한지 한 번 확인해보는 게 중요하다. 

## Example 2

만약 Ouster OS1-64 내부의 LiDAR와 IMU를 이용해서 Fast-LIO2를 돌리려면 어떻게 해야할까? 아래와 같이 입력하면 된다.

```angular2html
rosrun tf tf_echo apis/os_lidar apis/os_imu
# Output
At time 0.000
- Translation: [-0.006, 0.012, -0.029]
- Rotation: in Quaternion [0.000, 0.000, 1.000, 0.000]
            in RPY (radian) [0.000, 0.000, 3.142]
            in RPY (degree) [0.000, 0.000, 180.000]
```

그런데 이는 IMU 정보를 LiDAR frame으로 옮기는 transformation이므로, LiDAR frame을 IMU frame으로 옮기는 것이 필요하다. 따라서 위의 출력된 값의 inverse matrix를 extrinsic으로 넣어주면 된다.