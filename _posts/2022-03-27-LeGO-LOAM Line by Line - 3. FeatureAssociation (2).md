---
layout: post
title: LeGO-LOAM Line by Line - 3. FeatureAssociation (2)
subtitle: Corner and Planar Feature Extraction
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL, LeGO-LOAM]
comments: true
---

# FeatureAssociation in LeGO-LOAM (2) Edge and Planar Feature Extraction

(Cont'd)

![](/img/lego_loam_fa2.png)

Feature를 뽑을 준비가 끝나면 `extractFeatures()` 함수를 통해 feature를 추출한다. 그런데, relative pose를 추정하기 위해서는 연속적인 두 프레임의 feature가 필요하기 때문에, i.e. features on t-1 & features on t, initialization이 필요하다. 따라서 t=1인 경우에는 위와 같이 `checkSystemInitialization()` 함수까지만 진행이 되고 return 을 통해 종료된다.

### extractFeatures()

이 함수에서는 앞에 계산했던 `cloudCurvature[i] = diffRange*diffRange` (from `calculateSmoothness()`)와 `cloudNeighborPicked[i]`를 활용하여 최종적으로 feature extraction을 진행한다. 크게 feature extraction 과정은 edge feature과 corner feature를 나눠서 뽑는다.

```cpp
void extractFeatures()
{
    cornerPointsSharp->clear();
    cornerPointsLessSharp->clear();
    surfPointsFlat->clear();
    surfPointsLessFlat->clear();

    for (int i = 0; i < N_SCAN; i++) {

        surfPointsLessFlatScan->clear();

        for (int j = 0; j < 6; j++) {
            int sp = (segInfo.startRingIndex[i] * (6 - j)    + segInfo.endRingIndex[i] * j) / 6;
            int ep = (segInfo.startRingIndex[i] * (5 - j)    + segInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

            if (sp >= ep)
                continue;

            std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());
            // ---------------------------
            // 1. Extract corner features
            // ---------------------------
            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--) {
                int ind = cloudSmoothness[k].ind;
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > edgeThreshold &&
                    segInfo.segmentedCloudGroundFlag[ind] == false) {

                    largestPickedNum++;
                    if (largestPickedNum <= 2) {
                        cloudLabel[ind] = 2;
                        cornerPointsSharp->push_back(segmentedCloud->points[ind]);
                        cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
                    } else if (largestPickedNum <= 20) {
                        cloudLabel[ind] = 1;
                        cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
                    } else {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++) {
                        int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] 
                                            - segInfo.segmentedCloudColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l]
                                            - segInfo.segmentedCloudColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            // ---------------------------
            // 2. Extract planar features
            // ---------------------------
            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++) {
                int ind = cloudSmoothness[k].ind;
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < surfThreshold &&
                    segInfo.segmentedCloudGroundFlag[ind] == true) {

                    cloudLabel[ind] = -1;
                    surfPointsFlat->push_back(segmentedCloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4) {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++) {

                        int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] 
                                            - segInfo.segmentedCloudColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {

                        int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] 
                                            - segInfo.segmentedCloudColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            3. Set surfPointsLessFlatScan
            for (int k = sp; k <= ep; k++) {
                if (cloudLabel[k] <= 0) {
                    surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
                }
            }
        }

        surfPointsLessFlatScanDS->clear();
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.filter(*surfPointsLessFlatScanDS);

        *surfPointsLessFlat += *surfPointsLessFlatScanDS;
    }
}
```

a) 가장 먼저, 각 channal 별로 start point (`sp`)와 end point (`ep`)를 아래와 같이 6분할한다. 코드를 자세히 보기 전에는 단순히 이미지 plane을 6개의 subregion으로 쪼개서 feature를 고루 뽑는 줄 알았는데, 그게 아니라 아래와 같이 valid points 중 앞에서 5번째와 뒤에서 6번째 사이를 6분할한다 (기억이 안나면 [여기](https://limhyungtae.github.io/2022-03-27-LeGO-LOAM-Line-by-Line-2.-ImageProjection-(2)/)의 (b) 참조).

![](/img/lego_loam_sp_ep.png)
(그림은 간소하게 그렸으나, y 방향으로 주로 1,000 이상의 pixel이 있음을 양지 부탁드립니다...)

b) 그 후 해당 subregion을 sorting을 한다. 그래서 `sp`쪽에서는 curvature가 작은 값이 위치하게 되고, `ep` 쪽에는 반대로 curvature가 크게 위치하게 된다.

c) Sorting 후에 edge features를 선별하는데, 
    * `markOccludedPoints()` 함수로부터 마스킹이 안 되었고 (`cloudNeighborPicked[ind] == 0`)
    * curvature 값이 충분히 크고 (cloudCurvature[ind] > edgeThreshold, 클수록 양옆의 range 차이가 linear하지 않다는 뜻)
    * 해당 pixel이 ground 가 아닌 경우에만 corner feature를 뽑는다 (segInfo.segmentedCloudGroundFlag[ind] == false. 바닥이 아닌 곳에서 edge를 뽑는 것이 목표이기 때문. 애시당초 LeGO-LOAM은 험지에서 잘하기 위한 LOAM이어서 주로 이 corner는 벽의 모서리와 나무 줄기/가로등을 지칭한다고 생각하면 될듯).

d) 위의 조건을 만족하면 해당 pixel을 corner feature로 pixel을 뽑는다. 그 후, `cloudNeighborPicked[ind] = 1`로 설정하여 해당 pixel이 중복으로 뽑히는 걸 방지하고, feature를 고루 뽑기 위해 그 주변의 +-10 index 범위의 pixel은 feature를 뽑는데 사용하지 않는다.

e) 제일 sharp한 2개의 points를 뽑은 후, `cornerPointsLessSharp`를 할당한다.

f) 위의 과정과 비슷하게 planar feature를 뽑는 것을 반복하는데, planar feature는 우선적으로 ground에서만 `surfPointsFlat`으로 할당한다. 

g) 여기서 특이한건, `surfPointsLessFlat`를 할당하는 방법인데, `surfPointsLessFlat`는 planar feature + 할당되지 않은 모든 feature로 구성되어 있다. feature 뽑는 것을 진행하면 각 cloudLabel[i]이 다음과 같이 할당되어 있는데:

* `cloudLabel[i] == 2`: Sharp corner features
* `cloudLabel[i] == 1`: Less harp corner features
* `cloudLabel[i] == 0`: Not assigned
* `cloudLabel[i] == -1`: Planar features

`cloudLabel[i]`가 0이거나 -1인 경우에 대해서만 아래와 같이 `surfPointsLessFlatScan`에 다 때려 넣는다.

```cpp
// 3. Set surfPointsLessFlatScan
for (int k = sp; k <= ep; k++) {
    if (cloudLabel[k] <= 0) {
        surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
    }
}
```

마지막으로 각 channel 별로 voxelization을 시행한다. Voxelsize는 `initializationValue()` 함수에서 다음과 같이 미리 크기가 지정 되어있다. 

```cpp
downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
```

**NOTE**: `cornerPointsLessSharp`와 `surfPointsLessFlat`는 해당 frame이 t-1에서의 역할을 할 때 활용된다. 이 두 feature 집합을 편히 *less-feature*라 부를 때, 이 *less-feature*들은 t-1의 feature 입력값으로, distint feature (i.e. `cornerPointsSharp` & `surfPointsFlat`)들은 t의 입력값이 되어 서로 간에 correspondence를 찾는다. 


### checkSystemInitialization()

Feature를 추출한 후에는 FeatureAssociation이 처음 feature를 뽑았는지 아닌지를 확인한다. 초기에는 `systemInitedLM = false`로 되어 있는데, 이는 LiDAR odometry를 계산하려면 t-1과 t의 두 프레임의 feature set이 필요하기 때문에 체크하는 것이다. 코드는 아래와 같다.

```cpp
void checkSystemInitialization(){
    pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
    cornerPointsLessSharp = laserCloudCornerLast;
    laserCloudCornerLast = laserCloudTemp;

    laserCloudTemp = surfPointsLessFlat;
    surfPointsLessFlat = laserCloudSurfLast;
    laserCloudSurfLast = laserCloudTemp;

    kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
    kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

    laserCloudCornerLastNum = laserCloudCornerLast->points.size();
    laserCloudSurfLastNum = laserCloudSurfLast->points.size();
    
    // Just for visualization
    sensor_msgs::PointCloud2 laserCloudCornerLast2;
    pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
    laserCloudCornerLast2.header.stamp = cloudHeader.stamp;
    laserCloudCornerLast2.header.frame_id = "/camera";
    pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

    sensor_msgs::PointCloud2 laserCloudSurfLast2;
    pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
    laserCloudSurfLast2.header.stamp = cloudHeader.stamp;
    laserCloudSurfLast2.header.frame_id = "/camera";
    pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

    transformSum[0] += imuPitchStart;
    transformSum[2] += imuRollStart;

    systemInitedLM = true;
}
```

즉, t=1일 때는 다음과 같이 Kd Tree만 세팅한 후 `systemInitedLM`를 true로 세팅하여 종료한다. 여기서 t=1일 때 `laserCloudCornerLast`와 `laserCloudSurfLast`는 크기가 0인 point cloud이다 (swap을 하지만 사실상 별로 의미 없는 swap임).

![](/img/lego_loam_initialization_v2.png) 
 
 
이렇게 feature setting이 완료되면, 마지막으로 relative pose를 추정한다.
 
---

LeGO-LOAM의 line-by-line 설명 시리즈입니다.


1. [LeGO-LOAM-Line-by-Line-1.-Introduction: Preview and Preliminaries](https://limhyungtae.github.io/2022-03-27-LeGO-LOAM-Line-by-Line-1.-Introduction/)
2. [LeGO-LOAM-Line-by-Line-2.-ImageProjection-(1): Range Image Projection & Ground Removal](https://limhyungtae.github.io/2022-03-27-LeGO-LOAM-Line-by-Line-2.-ImageProjection-(1)/)
3. [LeGO-LOAM-Line-by-Line-2.-ImageProjection-(2): Cloud Segmentation using Clustering](https://limhyungtae.github.io/2022-03-27-LeGO-LOAM-Line-by-Line-2.-ImageProjection-(2)/)
4. [LeGO-LOAM-Line-by-Line-3.-FeatureAssociation-(1): Ready for Feature Extraction](https://limhyungtae.github.io/2022-03-27-LeGO-LOAM-Line-by-Line-3.-FeatureAssociation-(1)/)
5. [LeGO-LOAM-Line-by-Line-3.-FeatureAssociation-(2): Corner and Planar Feature Extraction](https://limhyungtae.github.io/2022-03-27-LeGO-LOAM-Line-by-Line-3.-FeatureAssociation-(2)/)
6. [LeGO-LOAM-Line-by-Line-3.-FeatureAssociation-(3): Relative Pose Estimation via Two-stage Optimization](https://limhyungtae.github.io/2022-03-27-LeGO-LOAM-Line-by-Line-3.-FeatureAssociation-(3)/)
 