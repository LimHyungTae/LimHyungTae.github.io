---
layout: post
title: ROS Point Cloud Library (PCL) - 10. Normal Estimation
subtitle: Low-level Normal Estimation of PointCloud 
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true

---

# Normal Estimation

이번에는 pointcloud에서의 normal vector를 추출하는 방법에 대해 알아보겠습니다.

Normal vector를 추출하는 일은 pointcloud의 geometry 정보를 활용할 때 다방면으로 활용됩니다.

큰 예제로는 G-ICP에서 각 point마다의 uncertainty(covariance)를 추정할 때, 혹은 벽면이나 바닥면을 추출할 때 등이 있습니다.  

PCL 상에 `pcl::NormalEstimation`이라는 end-to-end function이 있으나,

내부가 어떻게 구성돼있는지 알아보기 위해 직접 짜보았습니다.


# How to use

<script src="https://gist.github.com/LimHyungTae/f93910f855b7b9981485fc2c95916279.js"></script>

Normal vector를 visualization하면 아래와 같습니다.

# 결과

![normal1](/img/normal_vector1.png)

![normal2](/img/normal_vector2.png)

보시는 것과 같이 벽면들, 바닥들의 normal vector가 수직으로 잘 뽑히는 것을 확인할 수 있습니다.


# 궁금증

위에서 본 것과 같이, normal estimation은 인접한 point들과 해당 point 간의 관계를 구하는 것이기 때문에 알고리즘을 사용할 때 SearchMethod을 지정해주어야 합니다.

그리고 `setRadiusSearch(double radius)` 멤버함수를 통해서 radius를 지정해주는데, 그러면 만약 radius 이내에 인접한 point가 없을 때는 어떻게 되는지 궁금해서 테스트 해보았습니다.

아래 코드가 그 예시입니다.

<script src="https://gist.github.com/LimHyungTae/90cbbdd87727ee8bd3cb795005b5474f.js"></script>

```
(0,0,1 - 0)
(0,0,1 - 0)
(0,0,1 - 0)
(0,0,1 - 0)
(0,0,1 - 0)
(nan,nan,nan - nan)
```

`nan`으로 뜨는 것을 확인할 수 있습니다.


---

Point Cloud Library Tutorial 시리즈입니다.

{% include post_links_pcl.html %}
