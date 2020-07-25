---
layout: post
title: ROS Point Cloud Library (PCL) - 3. Voxelization
subtitle: ROS와 PCL 간의 형 변환
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL]
comments: true
---


## Transformation
![tf](/img/pcl_robot_sensor.PNG)

```cpp
//Input: pcl::PointCloud source, cloud_src
//Output: Transformed pcl::PointCloud, pc_transformed via 4x4 transformation matrix

pcl::PointCloud<pcl::PointXYZ> pc_transformed;
pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZ>);

Eigen::Matrix4f trans;
trans<< 1,   0,  0, 0.165,
        0,   1,  0, 0.000,
        0,   0,  1, 0.320,
        0,   0,  0,     1;
pcl::transformPointCloud(cloud_src, *ptr_transformed, trans);

pc_transformed = *ptr_transformed
```

[1] [Using a matrix to transform a point cloud](http://pointclouds.org/documentation/tutorials/matrix_transform.php)

[2] [Filtering a PointCloud using a PassThrough filter](http://pointclouds.org/documentation/tutorials/passthrough.php)

[3] [Downsampling a PointCloud using a VoxelGrid filter](http://pointclouds.org/documentation/tutorials/voxel_grid.php)

[4] [Removing outliers using a StatisticalOutlierRemoval filter](http://pointclouds.org/documentation/tutorials/statistical_outlier.php)



