---
layout: post
title: ROS Point Cloud Library (PCL) - 6. Statistical Outlier Removal
subtitle: Reduction of Computation
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true
---

## Downsampling to a Voxel Grid
![centroid](/img/pcl_centroid.PNG)
```cpp
#include <pcl/filters/voxel_grid.h>

//Input: pcl::PointCloud source, cloud_src
//Output: voxelized pcl::PointCloud, pc_voxelized 

pcl::PointCloud<pcl::PointXYZ> pc_voxelized;
pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

double var_voxelsize = 0.05;

*ptr_filtered = cloud_src;
voxel_filter.setInputCloud(ptr_filtered);
voxel_filter.setLeafSize(var_voxelsize, var_voxelsize, var_voxelsize);
voxel_filter.filter(*ptr_filtered);

pc_voxelized = *ptr_filtered;
```
그런데 굳이 filter()함수에 ptr을 넣지 않고 직접적으로 pcl::PointCloud<pcl::PointXYZ>로 받아도 된다.
```cpp
void mapgen::voxelize(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_src, pcl::PointCloud<pcl::PointXYZ>& pc_dst, double var_voxel_size){

  static pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(pc_src);
  voxel_filter.setLeafSize(var_voxel_size, var_voxel_size, var_voxel_size);
  voxel_filter.filter(pc_dst);

}
```

<script src="https://gist.github.com/LimHyungTae/1235dcdbe293133079c359f11906be24.js"></script>

## Statistical Outlier Removal
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
