---
layout: post
title: IMU Preintegration Derivation - 1. Introduction
subtitle: Preview and Preliminaries
tags: [SLAM, LiDAR, Pointcloud, ROS, IMU, Preintegration]
comments: true

---

# IMU Preintegration (작성중)

$$ x = y^2 $$

![](/img/preintegration/overview.png)


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
