#PCL cheat sheet

## pcl
**input: foo**</br>
**output: var**
```cpp
var foo = function(x) {
  return(x + 5);
}
foo(3)
```

## sensor_msgs::Pointcloud2 in ROS to pcl::PointCloud

## pcl::PointCloud to sensor_msgs::Pointcloud2 in ROS 

## pcl
```cpp
pcl::PointCloud<pcl::PointXYZ> output;
   pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::fromROSMsg(msg->velodyne, output);
   // For debugging


   pcl::PassThrough<pcl::PointXYZ> filter;
   Eigen::Matrix4f trans;
   trans<< 0.0174524,  -0.9998477,  0, -0.001,
                   0,           0, -1, -0.095,
           0.9998477,   0.0174524,  0, -0.064,
                   0,           0,  0,     1;
//   trans<< 0,  -1,  0, 0.000,
//           0,   0, -1, -0.10,
//           1,   0,  0, -0.07,
//           0,   0,  0,     1;
   pcl::transformPointCloud(output,*filtered,trans);
   filter.setInputCloud(filtered);
   filter.setFilterFieldName("z");
   filter.setFilterLimits(0, m_max_range);
   filter.filter(*filtered);
```

## Test
```cpp
pcl::fromROSMsg(pose_vec.back().pc2, cloud_t);
  pcl::PointCloud<pcl::PointXYZ> cloud_t_cut_by_dist = cvt::cloud2cloudcut(cloud_t,pcl::PointXYZ(0,0,0),0.5,20);
  Eigen::Matrix4f tf_velodyne2base = cvt::geoPose2eigen(pose_vec.back().T_laser2rt);
  Eigen::Matrix4f tf_base2origin = cvt::geoPose2eigen(pose_vec.back().T_laser2rt);
  pcl::transformPointCloud(cloud_t_cut_by_dist, cloud_t, tf_base2origin * tf_velodyne2base);
```

## Hey
