---
layout: post
title: ROS Point Cloud Library (PCL) - 6. Statistical Outlier Removal
subtitle: Outlier Rejection
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true

---

# Statistical Outlier Removal

![sor](/img/pcl_sor.PNG)

Statistical Outlier Removal는 outlier를 제거하는 알고리즘입니다. n=1, 2, ..., N 개의 point가 있을 때 n 번째 point를 기준으로 인접한 k개의 point들과 평균과 분산을 구해서 분산이 세팅한 parameter보다 큰 값이면 outlier로 간주하고 제거하는 알고리즘입니다. 이 알고리즘을 통해서는 위 그림같이 부정확하게 측정된 point를 제거할 수 있다는 장점이 있습니다. 3D LiDAR 센서라고 해서 100% 정확하게 거리를 측정하지는 못 합니다. 물체의 겉이 반사가 심하게 코딩되어 있거나(e.g. 뻔뜩뻔뜩한 대리석 바닥) 면이 둥글어서 난반사가 일어나거나 유리로 되어 있는 경우에는 굴절이 일어나면서 측정값이 부정확해질 수 있기 떄문입니다. 그럴 떄 Satisticcal Outlier Removal을 사용하면 outlier를 손쉽게 제거할 수 있습니다.    

---

# How to use

아래는 인접한 10개를 통해 std가 1.0 이상인 outlier를 제거하는 코드 예제입니다.

<script src="https://gist.github.com/LimHyungTae/180795d280fdc091d2798c2b7e215fa6.js"></script>

---

하지만, 저의 경험으로는 **3D LiDAR로 얻은 pointcloud에서는 SOR을 잘 안 씁니다.** 왜냐하면 인접한 k개의 point를 찾는 것도 연산이 너무 오래 걸리기 떄문입니다. 그리고 3D Pointcloud는 필연적으로 메모리를 효율적으로 사용하기 위해 [voxelization](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-4.-Voxelization/)을 사용하는데, voxelization은 Leaf 내부의 여러 point의 평균을 내기 떄문에, 이 과정에서 outlier의 영향을 줄일 수 있습니다. 따라서 SOR을 굳이 사용하지 않아도 SLAM을 하는데 큰 무리가 없습니다. (100% 저의 견해입니다. 부족한 점이 있으면 언제든 코멘트 부탁드립니다. 감사합니다.)

![sor_real_case](/img/hitach_sor.JPG)

그래서 저는 주로 2D LiDAR의 outlier를 제거할 때 사용합니다. Hitach-LG에서 주관한 LiDAR 경진대회에 나갔을 때에도 2D LiDAR의 LaserScan의 outlier를 제거하는데 효과적임을 확인했었습니다. 2D LiDAR 같은 경우도 한 평면만 스캔을 하는데, LaserScan이 원기둥으로 기둥이나 매끄러운 평면에 맞게되면 range 값들이 종종 오측정됩니다. 따라서 당시에 경진대회를 했을 때 Clustering을 하기 전에 SOR을 통해 너무 심한 outlier를 제거해준 후 clustering을 하면 좀더 robust하게 clustering이 되는 것을 확인했습니다. 그리고 2D LiDAR같은 경우에는 point 수가 그렇게 많지 않으니 SOR을 해도 연산에 부담이 없었던 것으로 기억합니다 :)

---

Point Cloud Library Tutorial 시리즈입니다.

1. [ROS Point Cloud Library (PCL) - 1. Tutorial 및 기본 사용법](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-1.-Tutorial-%EB%B0%8F-%EA%B8%B0%EB%B3%B8-%EC%82%AC%EC%9A%A9%EB%B2%95/)

2. [ROS Point Cloud Library (PCL) - 2. 형변환 - toROSMsg, fromROSMsg](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-2.-%ED%98%95%EB%B3%80%ED%99%98-toROSMsg,-fromROSMsg/)

3. [ROS Point Cloud Library (PCL) - 3. Transformation](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-3.-Transformation/)

4. [ROS Point Cloud Library (PCL) - 4. Voxelization](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-4.-Voxelization/)

5. [ROS Point Cloud Library (PCL) - 5. PassThrough](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-5.-PassThrough/)

6. **ROS Point Cloud Library (PCL) - 6. Statistical Outlier Removal**
