---
layout: post
title: ROS Point Cloud Library (PCL) - 0. Tutorial 및 기본 사용법
subtitle: 기본 사용법 및 operation들 정리
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true
---

# Point Cloud Library를 사용해야 하는 이유 

Point Cloud Library(사이트는 [여기](https://pointclouds.org/)) LiDAR나 RGB-D 카메라를 통해 취득한 pointcloud를 후처리하는데 사용되는 알고리즘을 구현해둔 라이브러리입니다. 특히, SLAM등이나 navigation을 할 때 써야하는 filter 알고리즘, voxelization, registration (i.e. ICP나 NDT 등) 등이 이미 구현되어 있어서 pointcloud를 후처리하거나 SLAM 알고리즘을 개발할 때 편리합니다. 특히, 연구레벨에서도 PCL 자체가 깔끔한 상속으로 구성되어 있어서 자기만의 수정된 알고리즘을 구현할 때에도 PCL의 부분부분을 상속받아서 쉽게 수정하여 사용가능한 것으로 알고 있습니다.
![centroid](/img/pcl_contents.JPG)

---

# 기초적인 사용법

## PCL 선언하는 법 & T Type

### T Type

pcl에서 구현되어 있는 Pointcloud 타입 `pcl::PointCloud<T>`에는 다양한 type을 담을 수 있는데,

주로 **LiDAR**를 사용할 때는 `pcl::PointXYZ`, `pcl::PointXYZI`를 많이 사용합니다.

**RGB-D나 스테레오 카메라**는 depth를 image에 align시키면 point의 색깔도 알 수 있기 때문에 `pcl::PointXYZRGB`를 사용하기도 합니다.

더 다양한 type은 원래 [pcl tutorial 페이지](http://www.pointclouds.org/documentation/tutorials/adding_custom_ptype.php#adding-custom-ptype)에서 확인 가능합니다. 

```cpp
pcl::PointCloud<pcl::PointXYZ> cloud;
pcl::PointCloud<pcl::PointXYZI> cloud;
pcl::PointCloud<pcl::PointNormal> cloud;
```

```cpp
pcl::PointXYZ point_xyz;
point_xyz.x = 1;
point_xyz.y = 2;
point_xyz.z = 3;
```
혹은 아래와 같이 한 줄로 선언이 가능합니다.
```cpp 
pcl::PointXYZ point_xyz = {1, 2, 3}; // 1, 2, 3이 각각 x, y, z로 지정된다.
```

### pcl::PointCloud 선언해서 Points에 Point 넣는 법

전문 코드는 아래와 같습니다.

<script src="https://gist.github.com/LimHyungTae/4e620ac9195f63423ab206e5070811a1.js"></script>

기본적으로 pcl은 std::vector의 사용법과 유사합니다.

왜냐하면 pcl의 내부를 살펴보면 std::vector로 구성되어 있기 때문입니다 :) ([여기](http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html)를 참조하시면 알 수 있습니다)

```cpp
pcl::PointCloud<pcl::PointXYZ> cloud_init0;
cloud_init0.resize(3); //cloud의 size를 3으로 설정

cloud_init0.points[0].x = 1;    cloud_init0.points[0].y = 2;    cloud_init0.points[0].z = 3;
cloud_init0.points[1].x = 4;    cloud_init0.points[1].y = 5;    cloud_init0.points[1].z = 6;
cloud_init0.points[2].x = 7;    cloud_init0.points[2].y = 8;    cloud_init0.points[2].z = 9;

```

or

```cpp
pcl::PointCloud<pcl::PointXYZ> cloud_init1;
pcl::PointXYZ point_xyz; // pcl::PointXYZ이라는 type에 data를 담는다.

point_xyz.x = 1;    point_xyz.y = 2;    point_xyz.z = 3;
cloud_init1.push_back(point_xyz);
point_xyz.x = 4;    point_xyz.y = 5;    point_xyz.z = 6;
cloud_init1.push_back(point_xyz);
point_xyz.x = 7;    point_xyz.y = 8;    point_xyz.z = 9;
cloud_init1.push_back(point_xyz);
```
or
```cpp
pcl::PointCloud<pcl::PointXYZ> cloud_init2;
cloud_init2.push_back(pcl::PointXYZ(1, 2, 3));
cloud_init2.push_back(pcl::PointXYZ(4, 5, 6));
cloud_init2.push_back(pcl::PointXYZ(7, 8, 9));
```

출력을 하면 아래와 같은 결과가 출력됩니다.
```cpp
template <class T>
void print_pc(pcl::PointCloud<T>& cloud){
    int count = 0;
    for (const auto& pt: cloud.points){
        cout << count++ << ": ";
        cout <<pt.x << ", "<<pt.y << ", "<< pt.z << endl;
    }
}
```
##### Result: <br/>
0: 1, 2, 3 <br/> 
1: 4, 5, 6 <br/>
2: 7, 8, 9 

##  Functions

http://docs.pointclouds.org/1.8.1/classpcl_1_1_point_cloud.html

### begin(): PointCloud의 첫 부분을 가르키는 *iterator*를 반환함

```cpp
cout << "begin(): ";
cout << cloud.begin()->x << ", ";
cout << cloud.begin()->y << ", ";
cout << cloud.begin()->z << endl;
```

->의 의미는 [이 블로그 글](https://ianuarias.tistory.com/16)에 잘 설명돼있습니다. 

->의 의미 한 줄 요약: return 값이 주소값이기 때문에, 주소값이 가르키는 객체의 멤버변수/멤버함수는 ->로 지칭함.

##### Result::<br/>
begin(): 1, 2, 3

### end(): PointCloud의 끝 부분을 가르키는 *iterator*를 반환함

```cpp
cout << "end(): ";
cout << cloud.end()->x << ", ";
cout << cloud.end()->y << ", ";
cout << cloud.end()->z << endl;
```
##### Result: <br/>
end(): 0, 0, 0

왜 0, 0, 0?: vector의 end()처럼 마지막 요소의 다음 부분을 가르키는 iterator를 리턴하기 때문! 즉 (0, 0, 0)은 메모리 상 값이 할당되지 않은 부분을 가르켜 값을 가져왔기 때문에 뜨는 숫자들입니다.

따라서 `pcl::PointCloud`의 제일 뒷쪽의 요소를 가르키는 iterator를 뜻하려면 `std::vector`와 마찬가지로 -1을 빼주어 사용해야 합니다.

```cpp
cout << "end() -1: ";
cout << (cloud.end()-1)->x << ", ";
cout << (cloud.end()-1)->y << ", ";
cout << (cloud.end()-1)->z << endl;
```
##### Result: <br/>
end() - 1: 7, 8, 9

### front(): 첫번째 *원소*를 반환함
```cpp
cout << "front(): ";
cout << cloud.front().x << ", ";
cout << cloud.front().y << ", ";
cout << cloud.front().z << endl;
```
##### Result: <br/>
front(): 1, 2, 3

### back(): 마지막 *요소*를 반환함

```cpp
cout << "back(): ";
cout << cloud.back().x << ", ";
cout << cloud.back().y << ", ";
cout << cloud.back().z << endl;
```

##### Result: <br/>
back(): 7, 8, 9

### at(int index) pcl::PointCloud 상의 index에 해당하는 요소를 반환함.

```cpp
cout << "at(1): ";
cout << cloud.at(1).x << ", ";
cout << cloud.at(1).y << ", ";
cout << cloud.at(1).z << endl;
```
##### Result: <br/>
at(1): 4, 5, 6

### empty()
```cpp
if (cloud.empty()) cout << "True"; else cout << "False";
cout << " | size of pc: " << cloud.size() << endl;
```
##### Result: <br/>
False | size of pc: 3

### clear()
```cpp
cloud.empty();
if (cloud.empty()) cout << "True"; else cout << "False";
cout << " | size of pc: " << cloud.size() << endl;
```
##### Result: <br/>
True | size of pc: 0


## ★PointCloud 합치기
~~이런 pythonic한 문법이 되다니...갓PCL...~~
```cpp
pcl::PointCloud<pcl::PointXYZ> cloud2;
cloud2.push_back(pcl::PointXYZ(1, 2, 3));
cloud2.push_back(pcl::PointXYZ(4, 5, 6));

pcl::PointCloud<pcl::PointXYZ> cloud3;
cloud3.push_back(pcl::PointXYZ(7, 8, 9));
cloud3.push_back(pcl::PointXYZ(10, 11, 12));

cloud2 += cloud3;

cout <<"size: " << cloud2.size() << endl;
print_pc(cloud2);
```
##### Result: <br/>
size: 4 <br/>
0: 1, 2, 3 <br/>
1: 4, 5, 6 <br/>
2: 7, 8, 9 <br/>
3: 10, 11, 12

---

`std::vector`의 특성상 무조건 뒤쪽으로 쌓이는 것을 확인할 수 있습니다.

### +=는 값을 할당하는 걸까, 복사하는 걸까?

```cpp
cloud3.push_back(pcl::PointXYZ(12, 13, 14));
cout<<"After: "<<endl;
print_pc(cloud2);
cout<<"cloud3?: "<<endl;
print_pc(cloud3);
```
##### Result: <br/>
After: <br/> 
0: 1, 2, 3 <br/>
1: 4, 5, 6 <br/>
2: 7, 8, 9 <br/> 
3: 10, 11, 12 <br/>
cloud3?: <br/> 
0: 7, 8, 9 <br/>
1: 10, 11, 12 <br/>
2: 12, 13, 14

결과가 그대로임을 알 수 있습니다. 즉, **+=** operation은 주소를 할당받아 link되어 있지 않고, points들을 통째로 복사해온다는 것을 알 수 있습니다.

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
10. 
