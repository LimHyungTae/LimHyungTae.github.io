---
layout: post
title: ROS Point Cloud Library (PCL) - 3. Transformation
subtitle: PCL Transformation
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true
---

# Transformation을 하는 이유?

![tf](/img/pcl_robot_sensor.PNG)

Robotics 분야에서 개발/연구를 하다보면 밥먹듯이 해야하는 것이 transformation입니다. 왜냐하면 그 이유는 robot의 system에서 기술해야 하는 수많은 좌표축(frame)이 존재하기 때문입니다. 위의 그림과 같이 LiDAR sensor가 하나만 달려있는 로봇이어도 최소 3가지 축이 존재합니다.

* **Global Frame**: Navigation Frame, Map Frame이라고 불리기도 하며, 맵의 (0,0,0)과 방향을 나타냄 
* **Body Frame**: Robot이 움직일 때 움직인 정도의 기준이 되는 축. 로봇이 모터 등으로 움직였을 때, 그 움직임을 나타내는 위치를 지칭함 
* **Sensor Frame**: Sensor의 data가 들어올 때, 센서의 상이 맺히는 부분의 위치와 방향을 나타냄

따라서 sensor를 통해 기술되는 data를 로봇의 움직임을 기술하는 축으로 transform해주어야 하고(**Sensor Frame → Body Frame**), 최종적으로 전체 pointcloud를 map 기준으로 취합해야 하기 때문에 (**Body Frame → Global Frame**) transformation이 필요한 것입니다. 


[![](http://img.youtube.com/vi/Sn_Ot3TiCyQ/0.jpg)](http://www.youtube.com/watch?v=Sn_Ot3TiCyQ "pose_correction")

*위의 video처럼 map을 만들 때 transformation은 필수입니다!*

---

# How to use

만약 위의 그림에서 LiDAR 센서가 로봇 Body 기준 x축으로 +0.165m, z축으로 + 0.320m에 놓여져 있다고 하면 어떻게 transformation을 하면 될까요? 코드는 아래와 같습니다. 

<script src="https://gist.github.com/LimHyungTae/ddd6f5cd6c2507a86388bd1b703e0cbb.js"></script>

간혹 0.165와 0.320이 +인지 -인지 부호가 헷갈리는 경우가 있습니다. 그럴때는 저는 (0,0,0,1), (1,0,0,1), (0,1,0,1), (0,0,1,1) 등과 transformation matrix를 곱했을 때의 결과가 Body 기준으로 맞는 위치로 표현되었는지 확인해봅니다.  

* 수학적 표현에 대해서는 저는 개인적으로 [*Introduction to Robotics*의 chapter 2](http://www.mech.sharif.ir/c/document_library/get_file?uuid=5a4bb247-1430-4e46-942c-d692dead831f&groupId=14040)가 많이 도움되었습니다.

* ROS에서는 pose의 orientation이 quaternion으로 표현이 되는데, 이걸 어떻게 rotation matrix로 표현할 수 있을까요? 정답은 ROS의 `tf::Quaternion`을 사용하면 쉽게 변환할 수 있습니다. 코드는 [여기](https://gist.github.com/LimHyungTae/2499a68ea8ee4d8a876a149858a5b08e)를 참조하세요 ;)


---


Point Cloud Library Tutorial 시리즈입니다.

1. [ROS Point Cloud Library (PCL) - 1. Tutorial 및 기본 사용법](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-1.-Tutorial-%EB%B0%8F-%EA%B8%B0%EB%B3%B8-%EC%82%AC%EC%9A%A9%EB%B2%95/)

2. [ROS Point Cloud Library (PCL) - 2. 형변환 - toROSMsg, fromROSMsg](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-2.-%ED%98%95%EB%B3%80%ED%99%98-toROSMsg,-fromROSMsg/)

3. **ROS Point Cloud Library (PCL) - 3. Transformation**

4. [ROS Point Cloud Library (PCL) - 4. Voxelization](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-4.-Voxelization/)

5. [ROS Point Cloud Library (PCL) - 5. PassThrough](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-5.-PassThrough/)

6. [ROS Point Cloud Library (PCL) - 6. Statistical Outlier Removal](https://limhyungtae.github.io/2019-11-29-ROS-Point-Cloud-Library-(PCL)-6.-Statistical-Outlier-Removal/)
