---
layout: post
title: ROS Point Cloud Library (PCL) - 1. Ptr, ConstPtr의 완벽 이해
subtitle: PCL의 Ptr, ConstPtr 사용법 및 정리
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true
---


## PCL pointer Ptr 선언

### Pointer를 사용하는 이유

pcl::PointCloud의 pointer는 아래와 같이 선언할 수 있습니다.

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
```

우선 `pcl::PointCloud<T>::Ptr`를 사용하는 이유는 제가 생각하기에 아래 두 요인이 큰 것 같습니다.

1. `pcl::PointCloud<T>::Ptr`는 typedef로 안에 boost::shared_ptr로 되어 있는데, 소멸할 때 pointcloud data를 자동적으로 delete해주기 때문
2. 2의 내용은 std::vector를 사용할 때 발생하는 memory leak과 관련이 있습니다. 관심 있으신 분은 google에 **vector memory leak**이라고 검색하시거나 [여기](https://stackoverflow.com/questions/1361139/how-to-avoid-memory-leaks-when-using-a-vector-of-pointers-to-dynamically-allocat)글을 보시면, 이러한 현상을 해결하기 위해 boost::shared_ptr을 사용한다고 합니다.
    * 한 줄 요약: std::vector의 `clear()`나 `resize()`는 vector의 iterator의 위치만 이동시켜주고, 해당하는 메모리를 delete하지 않는 문제가 있음. 근데 boost::shared_ptr을 쓰면 자동적으로 메모리 해제를 해 줌

그럼, 기존의 reference 기반의 pointer처럼 `pcl::PointCloud`의 주소를 Ptr로 할당시키려면 아래와 같이 하면 될까요?

```cpp
ptr_cloud = &cloud2; # Wrong :(
```

정답은 *틀렸습니다!!* 자세히 wiki를 살펴보시면 boost::shared_ptr로 구성되어 있기때문에, 아래와 같이 사용해야 pointcloud를 참조할 수 있습니다.

```cpp
*ptr_cloud = cloud2; # Right :)
```
## 예제

<script src="https://gist.github.com/LimHyungTae/8a1f2259aadd7a7d96aa672259a80788.js"></script>

### 사용법

#### pcl::PointCloud<T>::Ptr <-> pcl::PointCloud<T>
```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
*ptr_cloud = cloud2;

cout<<"Before: "<<endl;
print_pc(cloud3);
cloud3 = *ptr_cloud;
cout << "After: " << endl;
print_pc(cloud3);
```
##### Result: <br/>
Before: <br/>
0: 7, 8, 9 <br/>
1: 10, 11, 12 <br/>
After: <br/> 
0: 1, 2, 3 <br/>
1: 4, 5, 6 <br/>
2: 7, 8, 9 <br/>
3: 10, 11, 12

#### pcl::PointCloud<T>::Ptr 간의 할당

TBU

#### pcl::PointCloud<T>::ConstPtr

TBU 

---

Point Cloud Library Tutorial 시리즈입니다.

0. [ROS Point Cloud Library (PCL) - 0. Tutorial 및 기본 사용법](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-0.-Tutorial-%EB%B0%8F-%EA%B8%B0%EB%B3%B8-%EC%82%AC%EC%9A%A9%EB%B2%95/)

1. **ROS Point Cloud Library (PCL) - 1. Ptr, ConstPtr의 완벽 이해**

2. [ROS Point Cloud Library (PCL) - 2. 형변환 - toROSMsg, fromROSMsg](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-2.-%ED%98%95%EB%B3%80%ED%99%98-toROSMsg,-fromROSMsg/)

3. [ROS Point Cloud Library (PCL) - 3. Transformation](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-3.-Transformation/)

4. [ROS Point Cloud Library (PCL) - 4. Voxelization](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-4.-Voxelization/)

5. [ROS Point Cloud Library (PCL) - 5. PassThrough](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-5.-PassThrough/)

6. [ROS Point Cloud Library (PCL) - 6. Statistical Outlier Removal](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-6.-Statistical-Outlier-Removal/)
