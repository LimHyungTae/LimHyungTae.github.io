---
layout: post
title: Kimera-Multi 빌드 시 something wrong with flag 'logtostderr' 해결 방법
subtitle: 
tags: [Ubuntu, filesystem, library]
comments: true
---

현재 Kimera-Multi 레포지토리를 빌드하면서 다음과 같은 에러가 발생했다: 

```angular2html
ERROR: something wrong with flag 'logtostderr' in file '/workspace/lib/glog/src/logging.cc'.  One possibility: file '/workspace/lib/glog/src/logging.cc' is being linked both statically and dynamically into this executable.
================================================================================REQUIRED process [kimera_vio_ros/kimera_vio_ros_node-2] has died!
process has died [pid 3682959, exit code 1, cmd /home/shapelim/kimera_vio_ws/devel/lib/kimera_vio_ros/kimera_vio_ros_node --use_lcd=false --vocabulary_path=/home/shapelim/kimera_vio_ws/src/Kimera-VIO/vocabulary/ORBvoc.yml --flagfile=/home/shapelim/kimera_vio_ws/src/Kimera-VIO/params/Euroc/flags/Mesher.flags --flagfile=/home/shapelim/kimera_vio_ws/src/Kimera-VIO/params/Euroc/flags/VioBackend.flags --flagfile=/home/shapelim/kimera_vio_ws/src/Kimera-VIO/params/Euroc/flags/RegularVioBackend.flags --flagfile=/home/shapelim/kimera_vio_ws/src/Kimera-VIO/params/Euroc/flags/Visualizer3D.flags --use_external_odometry=false --do_coarse_imu_camera_temporal_sync=false --do_fine_imu_camera_temporal_sync=false --lcd_no_optimize=false --lcd_no_detection=false --depth_image_mask= --logtostderr=1 --colorlogtostderr=1 --log_prefix=1 --minloglevel=0 --v=0 --log_output=false --log_euroc_gt_data=false --output_path=/home/shapelim/kimera_vio_ws/src/Kimera-VIO-ROS/output_logs/ --lcd_disable_stereo_match_depth_check=false --no_incremental_pose=false --viz_type=0 --visualize=true reinit_flag:=reinit_flag reinit_pose:=reinit_pose odometry:=odometry resiliency:=resiliency imu_bias:=imu_bias optimized_trajectory:=optimized_trajectory pose_graph:=pose_graph mesh:=mesh frontend_stats:=frontend_stats debug_mesh_img/image_raw:=debug_mesh_img/image_raw feature_tracks/image_raw:=feature_tracks/image_raw time_horizon_pointcloud:=time_horizon_pointcloud __name:=kimera_vio_ros_node __log:=/home/shapelim/.ros/log/d62b4e54-5c22-11ef-8b10-89c894c2eed9/kimera_vio_ros-kimera_vio_ros_node-2.log].
log file: /home/shapelim/.ros/log/d62b4e54-5c22-11ef-8b10-89c894c2eed9/kimera_vio_ros-kimera_vio_ros_node-2*.log
Initiating shutdown!
================================================================================
```

## Solution

이는 높은 확률로 현재 환경에 여러 개의 glog 라이브러리가 설치되어 있어서 발생하는 문제이다.
따라서, 먼저 현재 시스템에 설치된 glog 라이브러리를 찾아서 제거해주어야 한다.
어디에 뭐가 깔려있는지는 아래와 같이 찾을 수 있다:^

```angular2html
> sudo find /usr/ -name "glog*"
/usr/local/include/glog
/usr/local/lib/cmake/glog
/usr/local/lib/cmake/glog/glog-config.cmake
/usr/local/lib/cmake/glog/glog-targets.cmake
/usr/local/lib/cmake/glog/glog-config-version.cmake
/usr/local/lib/cmake/glog/glog-targets-noconfig.cmake
/usr/include/glog
```

컴퓨터 내 glog 충돌을 해결하기 위해 위의 파일들을 제거해준다: 

```angular2html
sudo rm -rf /usr/local/include/glog /usr/local/lib/libglog* /usr/local/lib/cmake/glog
```

그리고 

```angular2html
sudo apt-get purge libgoogle-glog-dev   
```

를 통해 혹여나 남아있는 파일도 완전히 제거해준다.


그 후, 에러의 꼴이 바뀌었는데, 나의 컴퓨터에서는 vdb_fusion을 설치할 때 남아있던 glog가 또 잡혀서 에러가 일어났다.

```angular2html
ERROR: something wrong with flag 'logtostderr' in file '/home/shapelim/kimera_multi_lio_ws/src/vdbfusion_mapping/assets/scripts/glog/src/logging.cc'.  One possibility: file '/home/shapelim/kimera_multi_lio_ws/src/vdbfusion_mapping/assets/scripts/glog/src/logging.cc' is being linked both statically and dynamically into this executable.
================================================================================REQUIRED process [kimera_vio_ros/kimera_vio_ros_node-2] has died!
process has died [pid 3698846, exit code 1, cmd /home/shapelim/kimera_vio_ws/devel/lib/kimera_vio_ros/kimera_vio_ros_node --use_lcd=false --vocabulary_path=/home/shapelim/kimera_vio_ws/src/Kimera-VIO/vocabulary/ORBvoc.yml --flagfile=/home/shapelim/kimera_vio_ws/src/Kimera-VIO/params/Euroc/flags/Mesher.flags --flagfile=/home/shapelim/kimera_vio_ws/src/Kimera-VIO/params/Euroc/flags/VioBackend.flags --flagfile=/home/shapelim/kimera_vio_ws/src/Kimera-VIO/params/Euroc/flags/RegularVioBackend.flags --flagfile=/home/shapelim/kimera_vio_ws/src/Kimera-VIO/params/Euroc/flags/Visualizer3D.flags --use_external_odometry=false --do_coarse_imu_camera_temporal_sync=false --do_fine_imu_camera_temporal_sync=false --lcd_no_optimize=false --lcd_no_detection=false --depth_image_mask= --logtostderr=1 --colorlogtostderr=1 --log_prefix=1 --minloglevel=0 --v=0 --log_output=false --log_euroc_gt_data=false --output_path=/home/shapelim/kimera_vio_ws/src/Kimera-VIO-ROS/output_logs/ --lcd_disable_stereo_match_depth_check=false --no_incremental_pose=false --viz_type=0 --visualize=true reinit_flag:=reinit_flag reinit_pose:=reinit_pose odometry:=odometry resiliency:=resiliency imu_bias:=imu_bias optimized_trajectory:=optimized_trajectory pose_graph:=pose_graph mesh:=mesh frontend_stats:=frontend_stats debug_mesh_img/image_raw:=debug_mesh_img/image_raw feature_tracks/image_raw:=feature_tracks/image_raw time_horizon_pointcloud:=time_horizon_pointcloud __name:=kimera_vio_ros_node __log:=/home/shapelim/.ros/log/92efdc76-5c23-11ef-8b10-89c894c2eed9/kimera_vio_ros-kimera_vio_ros_node-2.log].
log file: /home/shapelim/.ros/log/92efdc76-5c23-11ef-8b10-89c894c2eed9/kimera_vio_ros-kimera_vio_ros_node-2*.log
Initiating shutdown!
================================================================================
```

이또한 완전히 제거해준 후, `catkin clean`으로 해당 workspace 상에서 잘못 link된 것들을 모두 제거한 후, 다시 빌드를 해서 실행시키면 정상적으로 실행된다.


