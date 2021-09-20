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

만약 `setFilterLimitsNegative(true)`로 설정을 하게 되면 지정된 range 이외(e.g. -3.0 to 3.0m로 지정해두었으면 축 기준 -Inf to-3.0과 3.0 to Inf가 filter된 결과로 나옵니다) 영억이 filtering됩니다.

<script src="https://gist.github.com/LimHyungTae/e64164994be190b6a3638f6b770f9485.js"></script>

![img](/img/pass_through_v2.svg)

보시는 것과 같이 왼쪽(빨강) 주변부에 차량 위쪽이 찍혀서 noise가 있었는데, sensor frame 인근의 pointcloud를 지움으로써 오른쪽(초록)처럼 주변부가 깔끔해지는 것을 확인할 수 있습니다.



---
추가로, 제가 실제로 짰던 코드 snippet도 공유드립니다. 주로 mobile robot에 3D LiDAR를 로봇 위쪽이 아닌 앞 쪽에 부착하게 되면 Sensor로 취득한 뒷 부분을 아래와 같이 filtering해야 합니다. 이 때 `setFilterLimitsNegative(true)`를 사용하면 손쉽게 filtering할 수 있습니다. (사실 for문으로 포인트마다 영역을 확인해줘서 filtering해주는 방법도 가능합니다 :)

![real](/img/passthrough_real_case.JPG){: .center-block :}


<script src="https://gist.github.com/LimHyungTae/aa538935ec8a5c8a482a8eb3002b6407.js"></script>

`애걔, 겨우 저걸 지우려고 이 노력을?`이라 생각하실 수도 있으나, 저 차 윗면에 반사된 pointcloud가 몇개 안돼보여도 반사면으로 인해 몇 천 개가 있는 경우도 있고, 또한 mapping을 할 때 저 noise를 지우지 않으면 차가 지나간 부분에 noise처럼 pointcloud가 찍혀있기 때문에 navigation 상에서 문제를 일으킬 수도 있습니다 (i.e. 실제 공간은 비어있으나 noise가 있어서 공간이 occupied되어 있다고 여겨질 수도 있기 때문입니다)

따라서, 모든 pointcloud data를 얻은 후, sensor frame 주변의 noise pointcloud를 잘 제거해주는 것은 필수입니다.

---

Point Cloud Library Tutorial 시리즈입니다.

0. [ROS Point Cloud Library (PCL) - 0. Tutorial 및 기본 사용법](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-0.-Tutorial-%EB%B0%8F-%EA%B8%B0%EB%B3%B8-%EC%82%AC%EC%9A%A9%EB%B2%95/)

1. [ROS Point Cloud Library (PCL) - 1. Ptr, ConstPtr의 완벽 이해](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-1.-Ptr,-ConstPtr%EC%9D%98-%EC%99%84%EB%B2%BD-%EC%9D%B4%ED%95%B4/)

2. [ROS Point Cloud Library (PCL) - 2. 형변환 - toROSMsg, fromROSMsg](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-2.-%ED%98%95%EB%B3%80%ED%99%98-toROSMsg,-fromROSMsg/)

3. [ROS Point Cloud Library (PCL) - 3. Transformation](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-3.-Transformation/)

4. [ROS Point Cloud Library (PCL) - 4. Voxelization](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-4.-Voxelization/)

5. **ROS Point Cloud Library (PCL) - 5. PassThrough**

6. [ROS Point Cloud Library (PCL) - 6. Statistical Outlier Removal](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-6.-Statistical-Outlier-Removal/)
