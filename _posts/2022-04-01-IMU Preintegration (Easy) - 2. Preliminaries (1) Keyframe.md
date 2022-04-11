---
layout: post
title: IMU Preintegration (Easy) - 2. Preliminaries (1) Keyframe
subtitle: Definition of Keyframe
tags: [SLAM, LIO, VIO, IMU, Preintegration]
comments: true

---

# Preliminaries of IMU Preintegration (1) Definition of Keyframe


먼저 preintegration을 이해하기 전에 keyframe가 무엇인지 알아야 한다. keyframe은 말그대로 "중요한 (key) frame"을 뜻하는데, 여기서 "중요하다"는 말은 주어진 두 frame 사이에 relative pose를 추정할 때 주변 환경의 기하학적 특성을 잘 묘사할 수 있는 reliable & repeatable features가 풍부하여 충분히 정확한 pose 추정이 가능하다는 것을 의미한다. 

 keyframe을 선정하는 주된 이유는 사용하는 sesnor의 모든 data를 전부 사용하여 SLAM을 하려 하면 많은 메모리를 필요로 하기 때문이라고 생각한다. 예를 들자면, 3D LiDAR sensor는 대부분 10 Hz로 3D point cloud를 취득하고, camera sensor의 같은 경우에는 약 30 Hz로 image를 취득한다. 그런데 LiDAR sensor를 활용해서 큰 도심 환경을 매핑하기 위해 약 1시간 가량 데이터를 취득했다고 가정하면, 1시간 동안 약 36,000개의 frame을 얻게 된다. 각 frame은 약 10만 여개의 points로 구성되어 있는데 (64 채널 기준. Velodyne HDL 64E의 경우 약 130,000 여개의 points를 매 frame마다 획득함), 이 raw range를 (x, y, z) format으로 파싱하면 각 point 당 3개의 float을 필요로 하게 된다. 그러면 어림잡아도 raw data를 저장하는 데에만 36,000 * 130,000 * 3 * 4하게 되면 약 56 GB 정도의 메모리가 필요하게 된다 (물론 이해를 돕기 위한 예시일 뿐, 정확하지 않음. 실제로 SLAM을 할 때는 voxelization 등을 활용해서 최소한 points만 저장하기 때문). 

 따라서 이러한 문제를 해결하기 위해서 대부분의 SLAM 알고리즘들에서는 a) 이전 keyframe 기준 로봇이 어느 정도 이상 움직였거나 (하지만 그 거리가 "적당히" 멀어야 함. Keyframe 사이의 거리가 너무 멀어지게 되면 pose 추정을 하는 것이 오히려 부정확해지기 때문) b) 이전 keyframe 기준 어느 정도 시간이 흘렀을 경우 다음 keyframe을 생성하고 있다. 
 
 이렇게 keyframe을 생성하는 행위는 sensor configuration과는 무관하게 LiDAR Inertial Odometry (LIO)든 Visual Inertial Odometry (VIO)이든 비슷하게 이루어진다. LIO-SAM와 VINS-Mono의 keyframe 생성 파트를 아래에 코드 상의 주석으로 표기해보았다 (case (a)와 case (b)로 표기해 둠).

 **In LIO-SAM** (`saveFrame()` function and `laserCloudInfoHandler` callback in `mapOptmization.cpp`)


 ```cpp
 bool saveFrame()
{
    if (cloudKeyPoses3D->points.empty())
        return true;

    if (sensor == SensorType::LIVOX)
    {
        if (timeLaserInfoCur - cloudKeyPoses6D->back().time > 1.0)
            return true;
    }

    Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
    Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                        transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);
    // - - - - - - - - - - - - - - - - - - - - - - -
    // HERE!!!! An example of the case (a)
    // Check the magnitude of the relative movement
    // - - - - - - - - - - - - - - - - - - - - - - -
    if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
        abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
        abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
        sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
        return false;

    return true;
}
 ```

```cpp
void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr& msgIn)
{
    // extract time stamp
    timeLaserInfoStamp = msgIn->header.stamp;
    timeLaserInfoCur = msgIn->header.stamp.toSec();

    // extract info and feature cloud
    cloudInfo = *msgIn;
    pcl::fromROSMsg(msgIn->cloud_corner,  *laserCloudCornerLast);
    pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);

    std::lock_guard<std::mutex> lock(mtx);
    // - - - - - - - - - - - - - - - - - - 
    // HERE!!!! An example of the case (b)
    // - - - - - - - - - - - - - - - - - - 
    static double timeLastProcessing = -1;
    if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
    {
        timeLastProcessing = timeLaserInfoCur;

        updateInitialGuess();

        extractSurroundingKeyFrames();

        downsampleCurrentScan();

        scan2MapOptimization();

        saveKeyFramesAndFactor();

        correctPoses();

        publishOdometry();

        publishFrames();
    }
}

```

**In VINS-Mono** (`process()` function in `pose_graph_node.cpp` )

```cpp
Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                        pose_msg->pose.pose.position.y,
                        pose_msg->pose.pose.position.z);
Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                            pose_msg->pose.pose.orientation.x,
                            pose_msg->pose.pose.orientation.y,
                            pose_msg->pose.pose.orientation.z).toRotationMatrix();
// - - - - - - - - - - - - - - - - - - - - - - -
// HERE!!!! An example of the case (a)
// Check the magnitude of the relative translation
// - - - - - - - - - - - - - - - - - - - - - - -
if((T - last_t).norm() > SKIP_DIS)
{
    vector<cv::Point3f> point_3d; 
    vector<cv::Point2f> point_2d_uv; 
    vector<cv::Point2f> point_2d_normal;
    vector<double> point_id;

    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        cv::Point3f p_3d;
        p_3d.x = point_msg->points[i].x;
        p_3d.y = point_msg->points[i].y;
        p_3d.z = point_msg->points[i].z;
        point_3d.push_back(p_3d);

        cv::Point2f p_2d_uv, p_2d_normal;
        double p_id;
        p_2d_normal.x = point_msg->channels[i].values[0];
        p_2d_normal.y = point_msg->channels[i].values[1];
        p_2d_uv.x = point_msg->channels[i].values[2];
        p_2d_uv.y = point_msg->channels[i].values[3];
        p_id = point_msg->channels[i].values[4];
        point_2d_normal.push_back(p_2d_normal);
        point_2d_uv.push_back(p_2d_uv);
        point_id.push_back(p_id);

        //printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
    }

    KeyFrame* keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), frame_index, T, R, image,
                        point_3d, point_2d_uv, point_2d_normal, point_id, sequence);   
    m_process.lock();
    start_flag = 1;
    posegraph.addKeyFrame(keyframe, 1);
    m_process.unlock();
    frame_index++;
    last_t = T;
}
```

```cpp
// - - - - - - - - - - - - - - - - - - 
// HERE!!!! An example of the case (b)
// - - - - - - - - - - - - - - - - - - 
if (skip_cnt < SKIP_CNT)
{
    skip_cnt++;
    continue;
}
else
{
    skip_cnt = 0;
}
```

---

## Goal of IMU Preintegration between two keyframes

결과적으로 keyframe은 아래의 그림과 같이 sensor로 매번 취득되는 frame 중에서 어느 정도의 간격을 두고 선별된다. [원 논문](https://rpg.ifi.uzh.ch/docs/TRO16_forster.pdf)에서는 인접한 두 keyframe을 $$i$$와 $$j$$로 표기하고 있다. 따라서 preintegration의 문제 정의는 "두 keyframe 사이의 수십~수백여개의 IMU data (그림 상의 x 표시)를 어떻게 하나의 factor (그림 상의 파란 ■)로 표현할 수 있는가"로 정리할 수 있다.


![](/img/preintegration/keyframe.png)



---

IMU Preintegration Derivation 설명 시리즈입니다.

1. [IMU Preintegration (Easy) - 1. Introduction](https://limhyungtae.github.io/2022-04-01-IMU-Preintegration-(Easy)-1.-Introduction/)
2. [IMU Preintegration (Easy) - 2. Preliminaries (1) Keyframe](https://limhyungtae.github.io/2022-04-01-IMU-Preintegration-(Easy)-2.-Preliminaries-(1)-Keyframe/)
3. [IMU Preintegration (Easy) - 2. Preliminaries (2) 3D Rotation and Uncertainty](https://limhyungtae.github.io/2022-04-01-IMU-Preintegration-(Easy)-2.-Preliminaries-(2)-3D-Rotation-and-Uncertainty/)
4. [IMU Preintegration (Easy) - 3. Derivation of IMU Model and Motion Integration](https://limhyungtae.github.io/2022-04-01-IMU-Preintegration-(Easy)-3.-Derivation-of-IMU-Model-and-Motion-Integration/)
5. [IMU Preintegration (Easy) - 4. Derivation of Preintegrated IMU Measurements](https://limhyungtae.github.io/2022-04-01-IMU-Preintegration-(Easy)-4.-Derivation-of-Preintegrated-IMU-Measurements/)
6. [IMU Preintegration (Easy) - 5. IMUPreintegration in LIO-SAM](https://limhyungtae.github.io/2022-04-01-IMU-Preintegration-(Easy)-5.-IMUPreintegration-in-LIO-SAM/)
 

---



### 원 논문

* [Foster *et al.*, "IMU Preintegration on Manifold for Efficient
Visual-Inertial Maximum-a-Posteriori Estimation," in *Robotics: Science and Systems (RSS), 2011](http://www.roboticsproceedings.org/rss11/p06.pdf).
* [Foster *et al.*, "On-Manifold Preintegration for Real-Time
Visual-Inertial Odometry," *Transaction on Robotics*, 2016](https://rpg.ifi.uzh.ch/docs/TRO16_forster.pdf)