---
layout: post
title: ROS Point Cloud Library (PCL) - 5. PassThrough
subtitle: 축을 기준으로하는 pointcloud filtering
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true
---

## Filtering using a PassThrough Filter

```cpp
#include <pcl/filters/passthrough.h>

//Input: pcl::PointCloud source, cloud_src
//Output: Filtered pcl::PointCloud, pc_filtered along z axis, from 0.5m to 100.0m

pcl::PointCloud<pcl::PointXYZ> pc_filtered;
pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PassThrough<pcl::PointXYZ> filter;

double min_range = 0.5;
double max_range = 100.0;
*ptr_filtered = cloud_src;

filter.setInputCloud(ptr_filtered);
filter.setFilterFieldName("z");
filter.setFilterLimits(min_range, max_range);
// filter.setFilterLimitsNegative(true);
filter.filter(*ptr_filtered);

pc_filtered = *ptr_filtered;
```

<script src="https://gist.github.com/LimHyungTae/e64164994be190b6a3638f6b770f9485.js"></script>

