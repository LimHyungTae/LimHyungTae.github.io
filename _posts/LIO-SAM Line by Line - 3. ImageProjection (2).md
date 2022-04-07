---
layout: post
title: LeGO-LOAM Line by Line - 1. Introduction
subtitle: Preview and Preliminaries
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL, LeGO-LOAM]
comments: true

---

# LIO-SAM

### Callbacks

### 3. projectPointCloud()

```cpp
void projectPointCloud()
{
    int cloudSize = laserCloudIn->points.size();
    // range image projection
    for (int i = 0; i < cloudSize; ++i)
    {
        PointType thisPoint;
        thisPoint.x = laserCloudIn->points[i].x;
        thisPoint.y = laserCloudIn->points[i].y;
        thisPoint.z = laserCloudIn->points[i].z;
        thisPoint.intensity = laserCloudIn->points[i].intensity;

        float range = pointDistance(thisPoint);
        if (range < lidarMinRange || range > lidarMaxRange)
            continue;

        int rowIdn = laserCloudIn->points[i].ring;
        if (rowIdn < 0 || rowIdn >= N_SCAN)
            continue;

        if (rowIdn % downsampleRate != 0)
            continue;

        int columnIdn = -1;
        if (sensor == SensorType::VELODYNE || sensor == SensorType::OUSTER)
        {
            float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
            static float ang_res_x = 360.0/float(Horizon_SCAN);
            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;
        }
        else if (sensor == SensorType::LIVOX)
        {
            columnIdn = columnIdnCountVec[rowIdn];
            columnIdnCountVec[rowIdn] += 1;
        }
        
        if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
            continue;

        if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
            continue;

        thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

        rangeMat.at<float>(rowIdn, columnIdn) = range;

        int index = columnIdn + rowIdn * Horizon_SCAN;
        fullCloud->points[index] = thisPoint;
    }
}
```

---

주의: timestamp가 가장 첫 point라는 사전 가정이 있음!
[Velodyne driver](https://github.com/ros-drivers/velodyne/blob/master/velodyne_driver/src/driver/driver.cc)

```cpp
<arg name="timestamp_first_packet" default="false" />
```

```cpp
// publish message using time of last packet read
        ROS_DEBUG("Publishing a full Velodyne scan.");
        if (config_.timestamp_first_packet){
        scan->header.stamp = scan->packets.front().stamp;
        }
        else{
        scan->header.stamp = scan->packets.back().stamp;
        }
        scan->header.frame_id = config_.frame_id;
output_.publish(scan);
```

---

**a) index 구하기**


**b) Point-wise deskewing**

```cpp
thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);
```

```cpp
PointType deskewPoint(PointType *point, double relTime)
{
    if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
        return *point;

    double pointTime = timeScanCur + relTime;

    float rotXCur, rotYCur, rotZCur;
    findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

    float posXCur, posYCur, posZCur;
    findPosition(relTime, &posXCur, &posYCur, &posZCur);

    if (firstPointFlag == true)
    {
        transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
        firstPointFlag = false;
    }

    // transform points to start
    Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
    Eigen::Affine3f transBt = transStartInverse * transFinal;

    PointType newPoint;
    newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
    newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
    newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
    newPoint.intensity = point->intensity;

    return newPoint;
}

```

`transStartInverse`는 늘 4x4 identity matrix임. posXCur, posYCur, posZCur는 늘 0이고 `imuRotX[0]`, `imuRotY[0]`, `imuRotZ[0]`도 0으로 initialize되기 때문

```cpp
void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
{
    *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

    int imuPointerFront = 0;
    while (imuPointerFront < imuPointerCur)
    {
        if (pointTime < imuTime[imuPointerFront])
            break;
        ++imuPointerFront;
    }

    if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
    {
        *rotXCur = imuRotX[imuPointerFront];
        *rotYCur = imuRotY[imuPointerFront];
        *rotZCur = imuRotZ[imuPointerFront];
    } else {
        int imuPointerBack = imuPointerFront - 1;
        double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
        double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
        *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
        *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
        *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
    }
}
```


```cpp
void cloudExtraction()
{
    int count = 0;
    // extract segmented cloud for lidar odometry
    for (int i = 0; i < N_SCAN; ++i)
    {
        cloudInfo.startRingIndex[i] = count - 1 + 5;

        for (int j = 0; j < Horizon_SCAN; ++j)
        {
            if (rangeMat.at<float>(i,j) != FLT_MAX)
            {
                // mark the points' column index for marking occlusion later
                cloudInfo.pointColInd[count] = j;
                // save range info
                cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
                // save extracted cloud
                extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                // size of extracted cloud
                ++count;
            }
        }
        cloudInfo.endRingIndex[i] = count -1 - 5;
    }
}
```

```cpp
void publishClouds()
{
    cloudInfo.header = cloudHeader;
    cloudInfo.cloud_deskewed  = publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
    pubLaserCloudInfo.publish(cloudInfo);
}
```

`cloudInfo.cloud_deskewed`에 deskewed points가 저장됨. 

```


참고자료
* [SLAM KR Study: LOAM and LeGO-LOAM](https://www.youtube.com/watch?v=snPzNmcbCCQ&t=564s)
* [LeGO-LOAM-code-review-imageProjection (Chinese)](https://wykxwyc.github.io/2019/01/23/LeGO-LOAM-code-review-imageProjection/)
* [LeGO-LOAM-code-review-featureAssociation (Chinese)](https://wykxwyc.github.io/2019/01/24/LeGO-LOAM-code-review-featureAssociation/)
* [Interpreting LEGO-LOAM Source Code: FeatureAssociation (1) (Chinese)](https://papago.naver.com/?sk=zh-CN&tk=en&st=LEGO-LOAM%E6%BA%90%E7%A0%81%E8%A7%A3%E6%9E%90%20---%20FeatureAssociation%E8%8A%82%E7%82%B9(1))
* [Interpreting LEGO-LOAM Source Code: FeatureAssociation (3) (Chinese)](https://zhuanlan.zhihu.com/p/245603082)
