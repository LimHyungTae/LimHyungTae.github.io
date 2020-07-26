---
layout: post
title: ROS Point Cloud Library (PCL) - 2. 형변환 - toROSMsg, fromROSMsg
subtitle: ROS와 PCL 간의 형 변환
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true
---

# 형변환을 해야 하는 이유

ROS를 처음 입문을 하면 형 변환을 하는 게 낯설어서 굉장히 힘듭니다. 그런데 이 형 변환을 왜 해야할까요? 정답은 ROS에서 master를 통해 통신할 때 주고 받는 `sensor_msgs::PointCloud2` 메세지를 사용하는데 그 data를 후처리할 때는 PCL의 pointcloud를 사용해야 하기 때문입니다. 비유를 하자면 ``sensor_msgs::PointCloud2`는 택배를 보내기 위해 상자에 담아둔 상태이고 `pcl::PointCloud`는 그 상자를 뜯어서 포장지를 뜯은 data라고 이해하면 될 것 같습니다. 

사족이지만 이렇게 분리해서 쓰는 이유는 -물론 ROS와 PCL을 개발한 사람들이 다른 이유도 있겠지만- 통신을 빠르게 하기 위함이라고 생각합니다 (저의 100% 추정입니다). 왜냐하면 [ROS sensors_msgs::PointCloud2 공식 안내 페이지](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html)를 살펴보면 range data가 uint8로 encoding되어 있는데, 이렇게 1차원의 range 정보를 `data`를 담아서 publish/subscribe하는 게 3개의 float64로 구성되있는 수 만개의 `pcl::PointXYZ`를 직접 publish/subscribe하는 것보다 훨씬 메모리를 절약할 수 있기 때문입니다. 

[![](http://img.youtube.com/vi/Sn_Ot3TiCyQ/0.jpg)](http://www.youtube.com/watch?v=Sn_Ot3TiCyQ "pose_correction")
*위의 video처럼 map을 만들 때 transformation은 필수입니다!*
---

# PointCloud 형 변환 

메세지 변환은 변환은 어렵지 않습니다 :) 아래와 같이 `pcl::fromROSMsg`와 `pcl::toROSMsg`를 사용하면 쉽게 변환할 수 있습니다. 

### sensor_msgs::PointCloud2 → pcl::PointCloud

```cpp
pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_dst;
    pcl::fromROSMsg(cloudmsg, cloud_dst);
    return cloud_dst;
  }
```
### pcl::PointCloud → sensor_msgs::PointCloud2
```cpp
sensor_msgs::PointCloud2 cloud2cloudmsg(pcl::PointCloud<pcl::PointXYZ> cloud_src)
  {
    sensor_msgs::PointCloud2 cloudmsg;
    pcl::toROSMsg(cloudsrc, cloudmsg);
    cloudmsg.header.frame_id = "map";
    return cloudmsg;
  }
```

---

2D LiDAR를 사용하시는 분도 계실텐데, 2D LiDAR는 `sensor_msgs::PointCloud2`가 아닌 `sensor_msgs::LaserScan`이기 때문에, 아래와 같이 laser_geometry header를 통해 변환해줄 수 있습니다.

### sensor_msgs::LaserScan → sensor_msgs::PointCloud2
```cpp

#include "laser_geometry/laser_geometry.h"

sensor_msgs::PointCloud2 laser2cloudmsg(sensor_msgs::LaserScan laser)
    {
      static laser_geometry::LaserProjection projector;
      sensor_msgs::PointCloud2 pc2_dst;
      projector.projectLaser(laser, pc2_dst,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
      pc2_dst.header.frame_id = "map";

      return pc2_dst;
    }
```

따라서 편의상 저는 `sensor_msgs::LaserScan` → `sensor_msgs::PointCloud2` → `pcl::PointCloud`로 변환해서 사용합니다. LaserScan 같은 경우는 3D LiDAR 보다 데이터 양이 훨씬 적기 떄문에 (약 1000개의 range data가 들어옵니다) `sensor_msgs::PointCloud2`를 거쳐서 변환해도 속도 이슈가 크게 발생하지 않습니다.

---


Point Cloud Library Tutorial 시리즈입니다.

1. **ROS Point Cloud Library (PCL) - 1. Tutorial 및 기본 사용법**

2. **ROS Point Cloud Library (PCL) - 2. 형변환 - toROSMsg, fromROSMsg**




