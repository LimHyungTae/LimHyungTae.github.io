---
layout: post
title: ROS Point Cloud Library (PCL) - 6. PassThrough
subtitle: 축을 기준으로하는 pointcloud filtering
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true
---

# PassThrough의 활용 예시

PassThrough 함수는 말 그래도 range 기반으로 filtering을 해주는 함수입니다. PassThrough는 주로 pointcloud를 받은 후 의도치 않게 측정이 되는 부분들을 제거하기 위해 사용됩니다 (i.e. 로봇에 부착했는데 센서의 측정 영역에 로봇의 몸체 부분이 있어서 불필요한 point가 찍힌다던가 등등). 사용 순서는 아래와 같습니다.

1. Filtering하고자 하는 축을 `setFilterFieldName` 함수를 통해 지정해 줌 
2. 원하는 범위를 `setFilterLimits` 함수를 통해 지정해 줌
3. (Option) 원하는 범위의 내의 point를 통과 시킬 것인지 (`setFilterLimitsNegative(false)`), 원하는 범위 외의 point를 통과시킬 것인지 결정(`setFilterLimitsNegative(true)`)

# How to use PassThrough Filter

아래의 snippet은 로봇 pointcloud로 x축 방향과 y축 방향으로 3.0m 주변 공간을 filtering해주는 예제입니다. (67번 째 줄부터)

만약 `setFilterLimitsNegative(true)`로 설정을 하게 되면 지정된 range 이외(e.g. -3.0~3.0m로 지정해두었으면 축 기준 -Inf~-3.0과 3.0~Inf) 영억이 filtering됩니다.

<script src="https://gist.github.com/LimHyungTae/e64164994be190b6a3638f6b770f9485.js"></script>

![img](/img/passthrough_noise_filter.png)

---
추가로, 제가 실제로 짰던 코드 snippet도 공유드립니다. 주로 Robot에 3D LiDAR를 부착하게 되면 Sensor로 취득한 뒷 부분을 아래와 같이 filtering해야 합니다. 이 때 `setFilterLimitsNegative(true)`를 사용하면 손쉽게 filtering할 수 있습니다. (물론 for문으로 포인트마다 영역을 확인해줘서 filtering해주는 방법도 가능합니다 :)

![real](/img/passthrough_real_case.JPG){: .center-block :}


<script src="https://gist.github.com/LimHyungTae/aa538935ec8a5c8a482a8eb3002b6407.js"></script>


---

Point Cloud Library Tutorial 시리즈입니다.

0. [ROS Point Cloud Library (PCL) - 0. Tutorial 및 기본 사용법](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-0.-Tutorial-%EB%B0%8F-%EA%B8%B0%EB%B3%B8-%EC%82%AC%EC%9A%A9%EB%B2%95/)

1. [ROS Point Cloud Library (PCL) - 1. Ptr, ConstPtr의 완벽 이해](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-1.-Ptr,-ConstPtr%EC%9D%98-%EC%99%84%EB%B2%BD-%EC%9D%B4%ED%95%B4/)

2. [ROS Point Cloud Library (PCL) - 2. 형변환 - toROSMsg, fromROSMsg](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-2.-%ED%98%95%EB%B3%80%ED%99%98-toROSMsg,-fromROSMsg/)

3. [ROS Point Cloud Library (PCL) - 3. Transformation](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-3.-Transformation/)

4. [ROS Point Cloud Library (PCL) - 4. Voxelization](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-4.-Voxelization/)

5. **ROS Point Cloud Library (PCL) - 5. PassThrough**

6. [ROS Point Cloud Library (PCL) - 6. Statistical Outlier Removal](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-6.-Statistical-Outlier-Removal/)
