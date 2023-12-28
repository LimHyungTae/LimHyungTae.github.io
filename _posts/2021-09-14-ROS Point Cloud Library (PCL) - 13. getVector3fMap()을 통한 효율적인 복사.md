---
layout: post
title: ROS Point Cloud Library (PCL) - 13. getVector3fMap()을 통한 효율적인 복사
subtitle: The Faster, the better
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true

---

# getVector3fMap()을 통한 Speed Up

이번에는 getVector3fMap()의 사용에 대해 알아보겠습니다.

굉장히 간단하지만, 알아 두면 굉장히 좋은, point cloud를 다룰 때 알고리즘의 속도를 implementation level에서 어느정도 올릴 수 있는 techinque입니다.

아래와 같이 `tmp`와 `tmp2`에 각각 x, y, z 값을 Eigen::Vector3f를 넣을 때,

```cpp
// Assume that `src` point cloud is given

vector<Eigen::Vector3f> tmp;
vector<Eigen::Vector3f> tmp2;

std::chrono::system_clock::time_point start = std::chrono::high_resolution_clock::now();
for (const auto &pt : src->points) {
    tmp.emplace_back(Eigen::Vector3f(pt.x, pt.y, pt.z));
}
std::chrono::system_clock::time_point mid = std::chrono::high_resolution_clock::now();

for (const auto &pt : src->points) {
    tmp2.emplace_back(pt.getVector3fMap());
}
std::chrono::system_clock::time_point end = std::chrono::high_resolution_clock::now();

const double sec1 = std::chrono::duration_cast<std::chrono::duration<double>>(
    mid - start).count();
const double sec2 = std::chrono::duration_cast<std::chrono::duration<double>>(
    end - mid).count();
std::cout << "Taken: " << sec1 << " sec (naive version)" << std::endl;
std::cout << "Taken: " << sec2 << " sec (via `getVector3fMap()`)" << std::endl;
```


출력되는 결과를 살펴보면 다음과 같습니다:

```commandline
Taken: 0.00210314 sec (naive version)
Taken: 0.00142921 sec (via `getVector3fMap()`)
```

즉, 속도가 1.5배 정도 빨라진 것을 확인할 수 있습니다. 성능 차이의 주요 이유는 다음과 같은데, 

**1. 객체 생성 비용 감소:** 첫 번째 방법에서는 각 반복마다 새로운 `Eigen::Vector3f` 객체를 생성합니다. 이는 메모리 할당과 초기화에 추가적인 시간을 필요로 합니다.
반면, `getVector3fMap()`은 기존 객체에 대한 참조만을 반환하기 때문에 이러한 추가 비용이 발생하지 않습니다.

**2. 메모리 접근 최적화:** `getVector3fMap()`는 원본 데이터에 대한 직접적인 접근을 제공합니다. 이는 메모리 접근이 최적화되어 있으며, 캐시 효율성이 높아집니다. 반면에 새 객체를 만들면 메모리 접근 패턴이 덜 효율적일 수 있습니다.

**3. 복사 연산 최소화:** 첫 번째 방법에서는 각 포인트의 x, y, z 값을 복사하여 새 Eigen::Vector3f 객체를 만듭니다. getVector3fMap()을 사용할 때는 이러한 복사가 없으므로 성능이 향상됩니다.

결론적으로, getVector3fMap()을 사용하는 것은 기존 데이터 구조를 직접 활용함으로써 메모리 할당과 객체 생성의 오버헤드를 줄이고, 데이터 접근을 최적화하여 전체적인 성능을 향상시킵니다.

위의 `tmp`와 `tmp2`를 `src`의 크기만큼 reserve한 후에 다시 돌려보면 다음과 같은 결과를 얻을 수 있습니다.

```commandline
Taken: 0.000885237 sec (naive version)
Taken: 0.000787708 sec (via `getVector3fMap()`)
```

~~reserve(src->points.size() 짱짱~~ 속도의 차이가 많이 줄기는 하지만, 그래도 속도가 약 1.12배 정도 빨라지는 것을 볼 수 있습니. 

결론: 동일한 행위를 하는데 속도가 빨라지는 것이니, 꼭 활용하면 좋을 듯하다.

---

Point Cloud Library Tutorial 시리즈입니다.

모든 코드는 이 [**레포지토리**](https://github.com/LimHyungTae/pcl_tutorial)에 있고, ros package로 구성되어 있어 build하여 직접 돌려보실 수 있습니다

0. [ROS Point Cloud Library (PCL) - 0. Tutorial 및 기본 사용법](https://limhyungtae.github.io/2021-09-09-ROS-Point-Cloud-Library-(PCL)-0.-Tutorial-%EB%B0%8F-%EA%B8%B0%EB%B3%B8-%EC%82%AC%EC%9A%A9%EB%B2%95/)
1. [ROS Point Cloud Library (PCL) - 1. Ptr, ConstPtr의 완벽 이해 (1) shared_ptr](https://limhyungtae.github.io/2021-09-09-ROS-Point-Cloud-Library-(PCL)-1.-Ptr,-ConstPtr%EC%9D%98-%EC%99%84%EB%B2%BD-%EC%9D%B4%ED%95%B4-(1)-shared_ptr/)
2. [ROS Point Cloud Library (PCL) - 1. Ptr, ConstPtr의 완벽 이해 (2) Ptr in PCL](https://limhyungtae.github.io/2021-09-10-ROS-Point-Cloud-Library-(PCL)-1.-Ptr,-ConstPtr%EC%9D%98-%EC%99%84%EB%B2%BD-%EC%9D%B4%ED%95%B4-(2)-Ptr-in-PCL/)
3. [ROS Point Cloud Library (PCL) - 1. Ptr, ConstPtr의 완벽 이해 (3) Ptr in 클래스 멤버변수](https://limhyungtae.github.io/2021-09-10-ROS-Point-Cloud-Library-(PCL)-1.-Ptr,-ConstPtr%EC%9D%98-%EC%99%84%EB%B2%BD-%EC%9D%B4%ED%95%B4-(3)-Ptr-in-%ED%81%B4%EB%9E%98%EC%8A%A4-%EB%A9%A4%EB%B2%84%EB%B3%80%EC%88%98/)
4. [ROS Point Cloud Library (PCL) - 2. 형변환 - toROSMsg, fromROSMsg](https://limhyungtae.github.io/2021-09-10-ROS-Point-Cloud-Library-(PCL)-2.-%ED%98%95%EB%B3%80%ED%99%98-toROSMsg,-fromROSMsg/)
5. [ROS Point Cloud Library (PCL) - 3. Transformation](https://limhyungtae.github.io/2021-09-10-ROS-Point-Cloud-Library-(PCL)-3.-Transformation/)
6. [ROS Point Cloud Library (PCL) - 4. Viewer로 visualization하는 법](https://limhyungtae.github.io/2021-09-10-ROS-Point-Cloud-Library-(PCL)-4.-Viewer%EB%A1%9C-visualization%ED%95%98%EB%8A%94-%EB%B2%95/)
7. [ROS Point Cloud Library (PCL) - 5. Voxelization](https://limhyungtae.github.io/2021-09-12-ROS-Point-Cloud-Library-(PCL)-5.-Voxelization/)
8. [ROS Point Cloud Library (PCL) - 6. PassThrough](https://limhyungtae.github.io/2021-09-12-ROS-Point-Cloud-Library-(PCL)-6.-PassThrough/)
9. [ROS Point Cloud Library (PCL) - 7. Statistical Outlier Removal](https://limhyungtae.github.io/2021-09-12-ROS-Point-Cloud-Library-(PCL)-7.-Statistical-Outlier-Removal/)
10. [ROS Point Cloud Library (PCL) - 8. KdTree를 활용한 Radius Search](https://limhyungtae.github.io/2021-09-12-ROS-Point-Cloud-Library-(PCL)-8.-KdTree%EB%A5%BC-%ED%99%9C%EC%9A%A9%ED%95%9C-Radius-Search/)
11. [ROS Point Cloud Library (PCL) - 9. KdTree를 활용한 K-nearest Neighbor Search (KNN)](https://limhyungtae.github.io/2021-09-12-ROS-Point-Cloud-Library-(PCL)-9.-KdTree%EB%A5%BC-%ED%99%9C%EC%9A%A9%ED%95%9C-K-nearest-Neighbor-Search-(KNN)/)
12. [ROS Point Cloud Library (PCL) - 10. Normal Estimation](https://limhyungtae.github.io/2021-09-13-ROS-Point-Cloud-Library-(PCL)-10.-Normal-Estimation/)
13. [ROS Point Cloud Library (PCL) - 11. Iterative Closest Point (ICP)](https://limhyungtae.github.io/2021-09-14-ROS-Point-Cloud-Library-(PCL)-11.-Iterative-Closest-Point-(ICP)/)
14. [ROS Point Cloud Library (PCL) - 12. Generalized Iterative Closest Point (G-ICP)](https://limhyungtae.github.io/2021-09-14-ROS-Point-Cloud-Library-(PCL)-12.-Generalized-Iterative-Closest-Point-(G-ICP)/)
14. [ROS Point Cloud Library (PCL) - 13. getVector3fMap()을 통한 효율적인 복사](https://limhyungtae.github.io/2021-09-14-ROS-Point-Cloud-Library-(PCL)-13.%20getVector3fMap()%EC%9D%84-%ED%86%B5%ED%95%9C-%ED%9A%A8%EC%9C%A8%EC%A0%81%EC%9D%B8-%EB%B3%B5%EC%82%AC/)

