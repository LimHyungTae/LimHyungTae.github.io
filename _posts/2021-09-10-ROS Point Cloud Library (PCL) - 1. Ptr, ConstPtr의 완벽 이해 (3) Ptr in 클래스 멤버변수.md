---
layout: post
title: ROS Point Cloud Library (PCL) - 1. Ptr, ConstPtr의 완벽 이해 (3) Ptr in 클래스 멤버변수
subtitle: PCL의 Ptr를 Class member 변수로 쓸 때 주의점
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true
---


## PCL Ptr in Class

이번 시간에는 마지막으로, Class 내에서 멤버 변수로 사용할 때의 주의점에 대해서 말씀 드리고, Ptr에 대한 얘기를 끝내겠습니다.

<script src="https://gist.github.com/LimHyungTae/4b5a1085162b0851a9429b017ab9b7ef.js"></script>

#### Error Case 1

위의 코드에서 보이는 것처럼, 멤버 변수에서 사용해야할 때는 선언을 따로 해주고, 생성자(costructor)에서 동적할당을 해서 써야 합니다.

위의 Error Case 1처럼, 내부에서 new를 통해 힙 영역에 동적할당을 해주게 되면 에러가 나는 것을 확인할 수 있습니다.


#### Error Case 2

만약 선언 후에 생성자 내에서 동적 할당을 해주지 않으면 어떻게 될까요?

![wo_init](img/pcl_class_wo_init.png)

![wo_init_output](img/pcl_class_wo_init_result.png)

다음과 같이 exit code 139가 뜨는데, 이 의미는 다음과 같습니다.


> exit(139): It indicates Segmentation Fault which means that the program was trying to access a memory location not allocated to it. This mostly occurs while using pointers or trying to access an out-of-bounds array index.

즉, 조세호 님마냥 '할당 안 받았는데 어떻게 데이터를 넣어요?!' 하는 error가 일어나는 것을 엿볼 수 있습니다.

여기까지 하면 Ptr의 개념을 어느정도 다 익히셨으리라 보고, 다음 lecture부터는 실제로 많이 사용하는 함수들을 위주로 살펴보겠습니다.


---

Point Cloud Library Tutorial 시리즈입니다.

0. [ROS Point Cloud Library (PCL) - 0. Tutorial 및 기본 사용법](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-0.-Tutorial-%EB%B0%8F-%EA%B8%B0%EB%B3%B8-%EC%82%AC%EC%9A%A9%EB%B2%95/)

1. **ROS Point Cloud Library (PCL) - 1. Ptr, ConstPtr의 완벽 이해**

2. [ROS Point Cloud Library (PCL) - 2. 형변환 - toROSMsg, fromROSMsg](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-2.-%ED%98%95%EB%B3%80%ED%99%98-toROSMsg,-fromROSMsg/)

3. [ROS Point Cloud Library (PCL) - 3. Transformation](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-3.-Transformation/)

4. [ROS Point Cloud Library (PCL) - 4. Voxelization](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-4.-Voxelization/)

5. [ROS Point Cloud Library (PCL) - 5. PassThrough](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-5.-PassThrough/)

6. [ROS Point Cloud Library (PCL) - 6. Statistical Outlier Removal](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-6.-Statistical-Outlier-Removal/)
