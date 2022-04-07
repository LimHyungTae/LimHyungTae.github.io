---
layout: post
title: LeGO-LOAM Line by Line - 1. Introduction
subtitle: Preview and Preliminaries
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL, LeGO-LOAM]
comments: true

---

# LIO-SAM



최근 LiDAR sensor로 취득한 3D point cloud에서 feature extraction을 어떻게 뽑는가에 대한 관심이 생겨서, LeGO-LOAM을 자세히 보게 되었습니다.

그런데 찾아보는 도중, 크게 3가지 문제점을 느꼈습니다.
* Overview에 대한 설명 자료는 많으나, line-by-line으로 자세한 설명으로 되어있는 한글 문서가 없음 (주로 중국어(!)로 되어있음)
* LeGO-LOAM의 코드가 생각보다 휴리스틱한 부분과 하드코딩이 많아서, 처음 SLAM을 공부하는 이들이 LeGO-LOAM 코드를 바이블 삼아 공부하기에는 진입장벽이 다소 높음을 발견 (LiDAR SLAM framework 자체를 공부하는 것이 목적이라면 저는 [Faster-LIO](https://github.com/gaoxiang12/faster-lio)를 보는 것을 더 추천드립니다.)
* 논문 상에서는 Levenberg-Marquardt 방법으로 optimization을 했다고 간단히 설명되어 있으나, 실제 코드 내부는 jacobian term을 직접 하나하나 구한다거나 설명없이 하드코딩으로 되어 있는 부분이 많음. 따라서 수학적 배경없이 코드를 이해하기 상당히 어려움

그래서 제가 이해한 것을 바탕으로 line-by-line으로, 초심자도 LeGO-LOAM 코드를 잘 이해할 수 있게 정리해보고자 이렇게 글을 쓰게 되었습니다.

LOAM과 LeGO-LOAM의 지식이 전혀 없으신 분은 진용이 형이 예전에 발표하셨던 [SLAM KR youtube 영상](https://www.youtube.com/watch?v=snPzNmcbCCQ&t=1589s)을 한 번 보고 읽어보시는 걸 추천드립니다.
 
* 그림들 내부는 혹시 모를 다른 나라의 외국인분들을 위해 영어로 작성하기로 했습니다. 저도 중국인 분의 글을 구글 번역을 돌려서 참고 했는데, 그림은 번역이 안돼서 힘들었습니다 😅 이 점 양지 부탁드립니다. 
 
## Preview

LeGO-LOAM의 전반적인 파이프라인은 아래와 같고,

![](/img/lego_loam_w_robot.PNG)

코드 상에서는 그림 상의 각각의 block이 아래처럼 대응됩니다.

* `imageProjection.cpp` (Segmentation)
* `featureAssociation.cpp` (Feature Extraction & Lidar Odometry)
* `mapOptimization.cpp` (Lidar Mapping)
* `transformFusion.cpp` (Transform Integration)

그리고 코드 상의 pipeline을 ROS rqt로 visualization해보면 아래와 같습니다.

![](/img/lego_loam_overview.png)


향후 각각의 요소에 대해서 설명할 예정입니다 

(22.03.07: 현재 작성자의 interest는 `featureAssociation.cpp`까지입니다...헤헿...)

## Preliminaries

LeGO-LOAM을 설명하기 앞서, LOAM 계열 알고리즘을 더 잘 이해하기 위해서는 3D LiDAR sensor의 하드웨어적인 특성에 대한 사전지식이 필요합니다. 3D omnidirectional LiDAR sensor의 경우 여러 ring을 가지고 있고 (channel이라고도 부름), 인접한 ring 간의 사이각이 주로 uniform하게 구성되어 있습니다.

Velodyne Puck-16으로 LeGO-LOAM을 돌려야 하는 상황에서는 아래와 같이 파라미터 세팅이 되는데:

```cpp
extern const int N_SCAN = 16;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0+0.1;
extern const int groundScanInd = 7;
```

이를 잘 이해하려면 LiDAR Odometry를 돌리기 이전에 **센서의 datasheet를 꼭 확인**해야 합니다. 예를 들어 Velodyne Puck같은 경우에는 [스펙](https://www.amtechs.co.jp/product/VLP-16-Puck.pdf)과 [메뉴얼](https://velodynelidar.com/wp-content/uploads/2019/12/63-9243-Rev-E-VLP-16-User-Manual.pdf)을 확인하시면 변수의 의미를 파악할 수 있습니다. 아래는 코드의 변수에 대응되는 부분을 제가 간단히 표현해보았습니다.

![](/img/preintegration/overview.png)



만약 다른 센서(e.g Ouster OS1-64)로 LOAM 계열 코드를 돌려야하는 경우에는 하드웨어의 특성에 맞게 아래와 같이 파라미터들이 수정되어야 합니다.
(**중요**:Ouster 같은 경우는 Velodyne 사의 3D LiDAR와 측정 방식이 달라 추가적으로 코드 내부를 더 수정해주어야 할 수도 있습니다. 간략히 예시를 들자면 아래와 같이 하드웨어에 맞게 파라미터들을 잘 수정해주어야 합니다. 자세한 것은 기섭이가 정리해둔 [레포지토리](https://github.com/irapkaist/SC-LeGO-LOAM) 참조)

```cpp
// Ouster OS1-64
extern const int N_SCAN = 64;
extern const int Horizon_SCAN = 1024;
extern const float ang_res_x = 360.0/float(Horizon_SCAN);
extern const float ang_res_y = 33.2/float(N_SCAN-1);
extern const float ang_bottom = 16.6+0.1;
extern const int groundScanInd = 15;
```

LiDAR sensor의 하드웨어에 대해 살펴봤으니, 순차적으로 LeGO-LOAM의 각 모듈에 대해 살펴볼 예정입니다. 편의 상 모듈 별 설명을 할 때는 반말로 진행하겠습니다 :).


---

LeGO-LOAM의 line-by-line 설명 시리즈입니다.


1. [LeGO-LOAM-Line-by-Line-1.-Introduction: Preview and Preliminaries](https://limhyungtae.github.io/2022-03-27-LeGO-LOAM-Line-by-Line-1.-Introduction/)
2. [LeGO-LOAM-Line-by-Line-2.-ImageProjection-(1): Range Image Projection & Ground Removal](https://limhyungtae.github.io/2022-03-27-LeGO-LOAM-Line-by-Line-2.-ImageProjection-(1)/)
3. [LeGO-LOAM-Line-by-Line-2.-ImageProjection-(2): Cloud Segmentation using Clustering](https://limhyungtae.github.io/2022-03-27-LeGO-LOAM-Line-by-Line-2.-ImageProjection-(2)/)
4. [LeGO-LOAM-Line-by-Line-3.-FeatureAssociation-(1): Ready for Feature Extraction](https://limhyungtae.github.io/2022-03-27-LeGO-LOAM-Line-by-Line-3.-FeatureAssociation-(1)/)
5. [LeGO-LOAM-Line-by-Line-3.-FeatureAssociation-(2): Corner and Planar Feature Extraction](https://limhyungtae.github.io/2022-03-27-LeGO-LOAM-Line-by-Line-3.-FeatureAssociation-(2)/)
6. [LeGO-LOAM-Line-by-Line-3.-FeatureAssociation-(3): Relative Pose Estimation via Two-stage Optimization](https://limhyungtae.github.io/2022-03-27-LeGO-LOAM-Line-by-Line-3.-FeatureAssociation-(3)/)
 
---

참고자료
* [SLAM KR Study: LOAM and LeGO-LOAM](https://www.youtube.com/watch?v=snPzNmcbCCQ&t=564s)
* [LeGO-LOAM-code-review-imageProjection (Chinese)](https://wykxwyc.github.io/2019/01/23/LeGO-LOAM-code-review-imageProjection/)
* [LeGO-LOAM-code-review-featureAssociation (Chinese)](https://wykxwyc.github.io/2019/01/24/LeGO-LOAM-code-review-featureAssociation/)
* [Interpreting LEGO-LOAM Source Code: FeatureAssociation (1) (Chinese)](https://papago.naver.com/?sk=zh-CN&tk=en&st=LEGO-LOAM%E6%BA%90%E7%A0%81%E8%A7%A3%E6%9E%90%20---%20FeatureAssociation%E8%8A%82%E7%82%B9(1))
* [Interpreting LEGO-LOAM Source Code: FeatureAssociation (3) (Chinese)](https://zhuanlan.zhihu.com/p/245603082)
