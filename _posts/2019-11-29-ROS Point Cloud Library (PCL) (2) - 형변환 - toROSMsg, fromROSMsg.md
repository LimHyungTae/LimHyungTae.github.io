---
layout: post
title: ROS Point Cloud Library (PCL) (2) - 형변환 - toROSMsg, fromROSMsg
subtitle: ROS와 PCL 간의 형 변환
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true
---

### 형변환을 해야 하는 이유

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




