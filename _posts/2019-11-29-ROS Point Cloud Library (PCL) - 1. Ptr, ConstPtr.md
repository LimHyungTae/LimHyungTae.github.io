---
layout: post
title: ROS Point Cloud Library (PCL) - 0. Tutorial 및 기본 사용법
subtitle: 기본 사용법 및 operation들 정리
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

1. PCL 상에서 poincloud를 처리할 때 값들을 통째로 복사한 후 처리하기에는 point들이 너무 많음 (3D LiDAR는 적어도 한 번 스캔에 10000개의 point가 측정되는데, 크기가 10000인 vector를 계속 메모리에 통째로 복사한다고 생각하면 이해가 쉽게 될 것 같습니다) 
2. Sensor data를 처리함에 있어서 그때그때 data를 받아서 처리해야 하므로, 동적할당을 통해 메모리를 좀 더 효율적으로 사용함

그럼, 기존의 reference 기반의 pointer처럼 `pcl::PointCloud`의 주소를 Ptr로 할당시키려면 아래와 같이 하면 될까요?

```cpp
ptr_cloud = &cloud2; # Wrong :(
```

정답은 아닙니다!! 자세히 wiki를 살펴보시면 boost::shared_ptr로 구성되어 있기때문에, 아래와 같이 사용해야 pointcloud를 참조할 수 있습니다.

```cpp
*ptr_cloud = cloud2; # Right :)
```

### 사용법

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
*ptr_cloud = cloud2;

cout<<"Original: "<<endl;
for (int i = 0 ; i < cloud3.size(); ++i){
cout << i << ": ";
cout << cloud3.points[i].x << ", ";
cout << cloud3.points[i].y << ", ";
cout << cloud3.points[i].z << endl;
}

cloud3 = *ptr_cloud;

cout << "After: " << endl;
//  cout<<"size: " << cloud3.size() << endl;
for (int i = 0 ; i < cloud3.size(); ++i){
cout << i << ": ";
cout << cloud3.points[i].x << ", ";
cout << cloud3.points[i].y << ", ";
cout << cloud3.points[i].z << endl;
}
```
##### Result: <br/>
Original: <br/>
0: 7, 8, 9 <br/>
1: 10, 11, 12 <br/>
After: <br/> 
0: 1, 2, 3 <br/>
1: 4, 5, 6 <br/>
2: 7, 8, 9 <br/>
3: 10, 11, 12

---

Point Cloud Library Tutorial 시리즈입니다.

1. **ROS Point Cloud Library (PCL) - 1. Tutorial 및 기본 사용법**

2. [ROS Point Cloud Library (PCL) - 2. 형변환 - toROSMsg, fromROSMsg](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-2.-%ED%98%95%EB%B3%80%ED%99%98-toROSMsg,-fromROSMsg/)

3. [ROS Point Cloud Library (PCL) - 3. Transformation](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-3.-Transformation/)

4. [ROS Point Cloud Library (PCL) - 4. Voxelization](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-4.-Voxelization/)

5. [ROS Point Cloud Library (PCL) - 5. PassThrough](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-5.-PassThrough/)

6. [ROS Point Cloud Library (PCL) - 6. Statistical Outlier Removal](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-6.-Statistical-Outlier-Removal/)
