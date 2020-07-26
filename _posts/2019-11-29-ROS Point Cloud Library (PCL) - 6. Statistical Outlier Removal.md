---
layout: post
title: ROS Point Cloud Library (PCL) - 6. Statistical Outlier Removal
subtitle: Outlier Rejection
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true
---


![sor](/img/pcl_sor.PNG)

The number of neighbors to analyze for each point is set to 10, and the standard deviation multiplier to 1.0
```cpp
#include <pcl/filters/statistical_outlier_removal.h>

//Input: pcl::PointCloud source, cloud_src
//Output: voxelized pcl::PointCloud, pc_sor_filtered 

pcl::PointCloud<pcl::PointXYZ> pc_sor_filtered;
pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_sor_filtered(new pcl::PointCloud<pcl::PointXYZ>);
*ptr_sor_filtered = cloud_src;

int num_neigbor_points = 10;
double std_multiplier = 1.0;

pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
sor.setInputCloud (ptr_sor_filtered);
sor.setMeanK (num_neigbor_points);
sor.setStddevMulThresh (std_multiplier);
sor.filter(*ptr_sor_filtered);

pc_sor_filtered = *ptr_sor_filtered;
```

<script src="https://gist.github.com/LimHyungTae/180795d280fdc091d2798c2b7e215fa6.js"></script>
