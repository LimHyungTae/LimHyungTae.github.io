---
layout: post
title: LeGO-LOAM 상세 설명 - 3. FeatureAssociation (3)
subtitle: Relative Pose Estimation via Feature Association
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL, LeGO-LOAM]
comments: true
---

# FeatureAssociation in LeGO-LOAM (3) Relative Pose Estimation via Feature Association

(cont'd)

![](/img/lego_loam_fa3.png)


## Overview

최종적으로 t-1과 t의 feature를 구했으면, 주어진 feature들을 바탕으로 relative pose를 구한다. 흔히 알고 있듯이 planar feature를 통해 z, roll, pitch가 optimize하고 edge feature를 통해 x, y, yaw가 optimize하는, two-stage optimization 방식을 따른다.

![](/img/lego_loam_two_stage_optimization.PNG)


다른 section과 다르게 이 부분에서는 Levenberg-Marquardt (LM) optimization을 하는 부분의 설명이 필요하다보니 좀더 수학수학해졌다 :(. 아래의 링크들이 도움이 되리라 생각된다.
* [Least square, Iterative method, LM optimization](https://www.slideshare.net/phani279/lecture-5-15476418)
* [Optimization (KOR.)](https://darkpgmr.tistory.com/142)
* [Cross product](https://en.wikipedia.org/wiki/Cross_product)
* [Partial derivative](https://www.khanacademy.org/math/multivariable-calculus/multivariable-derivatives/partial-derivative-and-gradient-articles/a/introduction-to-partial-derivatives)


### updateInitialGuess()

`updateInitialGuess()` 함수에서는 IMU data가 들어오면 IMU data를 통해 구한 inital relative pose를 더해준다. 하지만 IMU를 쓰지 않으면 `imuShiftFromStartX`, `imuShiftFromStartY`, `imuShiftFromStartZ`, `imuVeloFromStartX`, `imuVeloFromStartY`, `imuVeloFromStartZ` 모두 다 계속 0이다. 즉, IMU 데이터가 들어오지 않으면 `updateInitialGuess()` 함수를 통해 `transformCur`가 업데이트되지는 않는다.

여기서 주의해야할점은 초기 transformCur의 값은 t-2와 t-1 프레임 간의 pose로 할당되어 있다는 것이다 (t=1과 t=2의 경우에는 [0, 0, 0, 0, 0, 0]으로 initialization되어 있음). 그 이유는 iterative하게 optimization을 할 때에는 초기값에 따라 성능이 많이 달라지게 되는데, 모바일 로봇의 경우 주로 앞-뒤로 움직이기 떄문에 (어려운 말로는 non-holonomic motion을 주로 하기 때문에) 이전에 추정한 pose와 현재 추정해야할 pose의 경향성이 비슷하다는 전제하에 initial pose를 이전 time step에 추정한 pose를 할당해둔다.

```cpp
void updateInitialGuess(){

    imuPitchLast = imuPitchCur;
    imuYawLast = imuYawCur;
    imuRollLast = imuRollCur;

    imuShiftFromStartX = imuShiftFromStartXCur;
    imuShiftFromStartY = imuShiftFromStartYCur;
    imuShiftFromStartZ = imuShiftFromStartZCur;

    imuVeloFromStartX = imuVeloFromStartXCur;
    imuVeloFromStartY = imuVeloFromStartYCur;
    imuVeloFromStartZ = imuVeloFromStartZCur;

    if (imuAngularFromStartX != 0 || imuAngularFromStartY != 0 || imuAngularFromStartZ != 0){
        transformCur[0] = - imuAngularFromStartY;
        transformCur[1] = - imuAngularFromStartZ;
        transformCur[2] = - imuAngularFromStartX;
    }

    if (imuVeloFromStartX != 0 || imuVeloFromStartY != 0 || imuVeloFromStartZ != 0){
        transformCur[3] -= imuVeloFromStartX * scanPeriod;
        transformCur[4] -= imuVeloFromStartY * scanPeriod;
        transformCur[5] -= imuVeloFromStartZ * scanPeriod;
    }
}
```

### updateTransformation()

Pose estimation하는 부분은 자명하다. 크게 **a) Correspondence 찾기**와 **b) non-linear optmizaton을 통한 parameter update하기**로 나눠지고, 이는 각각 planar feature과 edge feature에 대해 행해진다 (ICP와 동일하다).

```cpp
void updateTransformation(){
    if (laserCloudCornerLastNum < 10 || laserCloudSurfLastNum < 100)
        return;
    // ----------------------
    // Optimization of t_z, roll, and pitch by using planar (surface) features 
    // ----------------------
    for (int iterCount1 = 0; iterCount1 < 25; iterCount1++) {
        laserCloudOri->clear();
        coeffSel->clear();

        findCorrespondingSurfFeatures(iterCount1);

        if (laserCloudOri->points.size() < 10)
            continue;
        if (calculateTransformationSurf(iterCount1) == false)
            break;
    }
    
    // ----------------------
    // Optimization of t_x, t_y, and yaw by using corner (edge) features 
    // ----------------------
    for (int iterCount2 = 0; iterCount2 < 25; iterCount2++) {

        laserCloudOri->clear();
        coeffSel->clear();

        findCorrespondingCornerFeatures(iterCount2);

        if (laserCloudOri->points.size() < 10)
            continue;
        if (calculateTransformationCorner(iterCount2) == false)
            break;
    }
}
```

순서는 surface features -> corner features 순으로 optimization을 진행하지만, 둘의 구조와 알고리즘 상의 방법론이 동일하기 때문에, 수식이 좀 더 쉬운 corner feature에 대해 먼저 설명한다.

### findCorrespondingCornerFeatures(iterCount2)

```cpp
void findCorrespondingCornerFeatures(int iterCount){

    int cornerPointsSharpNum = cornerPointsSharp->points.size();

    for (int i = 0; i < cornerPointsSharpNum; i++) {

        TransformToStart(&cornerPointsSharp->points[i], &pointSel);

        if (iterCount % 5 == 0) {

            kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
            int closestPointInd = -1, minPointInd2 = -1;

            if (pointSearchSqDis[0] < nearestFeatureSearchSqDist) {
                closestPointInd = pointSearchInd[0];
                int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity);

                float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist;
                for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
                    if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan + 2.5) {
                        break;
                    }

                    pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) * 
                                 (laserCloudCornerLast->points[j].x - pointSel.x) + 
                                 (laserCloudCornerLast->points[j].y - pointSel.y) * 
                                 (laserCloudCornerLast->points[j].y - pointSel.y) + 
                                 (laserCloudCornerLast->points[j].z - pointSel.z) * 
                                 (laserCloudCornerLast->points[j].z - pointSel.z);

                    if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan) {
                        if (pointSqDis < minPointSqDis2) {
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                    }
                }
                for (int j = closestPointInd - 1; j >= 0; j--) {
                    if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5) {
                        break;
                    }

                    pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) * 
                                 (laserCloudCornerLast->points[j].x - pointSel.x) + 
                                 (laserCloudCornerLast->points[j].y - pointSel.y) * 
                                 (laserCloudCornerLast->points[j].y - pointSel.y) + 
                                 (laserCloudCornerLast->points[j].z - pointSel.z) * 
                                 (laserCloudCornerLast->points[j].z - pointSel.z);

                    if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan) {
                        if (pointSqDis < minPointSqDis2) {
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                    }
                }
            }

            pointSearchCornerInd1[i] = closestPointInd;
            pointSearchCornerInd2[i] = minPointInd2;
        }

        if (pointSearchCornerInd2[i] >= 0) {

            tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
            tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];

            float x0 = pointSel.x;
            float y0 = pointSel.y;
            float z0 = pointSel.z;
            float x1 = tripod1.x;
            float y1 = tripod1.y;
            float z1 = tripod1.z;
            float x2 = tripod2.x;
            float y2 = tripod2.y;
            float z2 = tripod2.z;

            float m11 = ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1));
            float m22 = ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1));
            float m33 = ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1));

            float a012 = sqrt(m11 * m11  + m22 * m22 + m33 * m33);

            float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

            float la =  ((y1 - y2)*m11 + (z1 - z2)*m22) / a012 / l12;

            float lb = -((x1 - x2)*m11 - (z1 - z2)*m33) / a012 / l12;

            float lc = -((x1 - x2)*m22 + (y1 - y2)*m33) / a012 / l12;

            float ld2 = a012 / l12;

            float s = 1;
            if (iterCount >= 5) {
                s = 1 - 1.8 * fabs(ld2);
            }

            if (s > 0.1 && ld2 != 0) {
                coeff.x = s * la; 
                coeff.y = s * lb;
                coeff.z = s * lc;
                coeff.intensity = s * ld2;

                laserCloudOri->push_back(cornerPointsSharp->points[i]);
                coeffSel->push_back(coeff);
            }
        }
    }
}
```

위 함수는 다시 크게 두 파트로 나뉘어진다.

**i) Correspondence 찾기**: 가장 먼저, time t 상의 sharp한 corner feature와 가장 거리가 가까운 t-1 두 corner feature를 찾는다. 여기서 `kdtreeCornerLast`의 입력 point cloud는 t-1의 `cornerPointsLessSharp`이다. t-1에서 가장 가까운 점 1개를 찾은 후, 그 점을 기점으로 +-2.5ms (확인 요망) 근처에 있는 점을 l점으로 취급한다. 근데 코드 상에서 보면 +- check를 위->아래 순으로 체크하는 것 같다. 

최종적으로는 가장 가까운 점과 그 다음 가까운 점이 아래와 같이 할당된다. 만약 두 점의 거리의 제곱이 주어진 파라미터 `nearestFeatureSearchSqDist`보다 크면 잘못된 pair라고 상정한다. 기본적으로 `nearestFeatureSearchSqDist`는 25m^2으로 설정되어 있다. 가장 가까운 포인트와 그 다음 가까운 포인트는 아래와 같이 index가 저장된다.

```cpp
pointSearchCornerInd1[i] = closestPointInd;
pointSearchCornerInd2[i] = minPointInd2;
```

만약 할당이 제대로 되지 않았으면 -1이 세팅된다.

**ii) Optimization에 필요한 values 세팅**: 성공적으로 correspondence를 찾았으면 이제 향후 optimization에 필요한 값들을 세팅한다. 크게 아래의 세가지를 설정한다 (※수학주의).


1. Point-to-line distance

먼저 point-to-line distacne가 계산된다. 엄밀히 말하자면 외적(cross product)를 통해 두 line이 일치하는 정도를 측정한다. 외적을 하게 되면 크기x크기xsin(사잇각)이 되는데, 따라서 time t 상의 타겟 포인트와 t-1 상에서 타겟 포인트와 가장 가까운점과 그 다음 가까운 점을 각각 이은 두 선이 일치하는 쪽으로 향후에 optimize되는 것이다. 수식을 전개하기 위해 아래 그림으로 대체한다.

![](/img/lego_loam_fa_point_to_line.png)


2.Jacobian term

그 후, optimization에 필요한 jacobian term을 미리 구해둔다. 자료들을 찾아보면 저 `la`, `lb`, `lc`가 무엇인지 설명을 하지 않는데, 저 term은 향후에 non-linear optimization을 할 때 필요하다. Interative estimation method 기반으로 non-linear optimization을 하면 아래와 같이 표현할 수 있다.

![](/img/lego_loam_fa_solve.png)

Corner feature 같은 경우에는 아래와 같이 yaw, x, y (하지만 좌표축이 ZXY로 변했음을 기억하자. 따라서 r_y (yaw 회전), x_rel (왼쪽), z_rel (앞쪽)이 optimize된다)의 변화량이 풀어야할 파라미터가 된다. 하지만 jacobian matrix의 각 term을 구할 때 함수 (아래의 1, 2, 3)에 각 타겟 포인트에 대한 point-to-line distance의 derivate가 필요하다. 이 값은 r_y, x_rel, z_rel에 invariant하기 때문에 미리 계산해둔다. 수식 전개는 아래와 같다.


![](/img/lego_loam_fa_la_lb_lc.png)


3. Alternative weighting

또한 각 distance에 대한 weight가 아래와 같이 매겨진다.

```cpp
float s = 1;
if (iterCount >= 5) {
    s = 1 - 1.8 * fabs(ld2);
}

if (s > 0.1 && ld2 != 0) {
    coeff.x = s * la; 
    coeff.y = s * lb;
    coeff.z = s * lc;
    coeff.intensity = s * ld2;

    laserCloudOri->push_back(cornerPointsSharp->points[i]);
    coeffSel->push_back(coeff);
}
```

해석하자면,

* 초기 5번 iteration 동안은 모든 distance들에 대해 `s=1`로 세팅하여 모든 measurements의 중요한 정도 (weight)를 균등하게 둔다.
* 5번 이후에는 너무 `ld2`의 크기에 따라 weight를 달리하는데, 특히 0.1이하의 경우에는 향후 optimization에 사용하지 않는다. 즉, `s`에 0.1을 넣고 전개하면 `ld2`가 0.5m 이하인 점들만 유효한 measurements로 여긴다고 해석할 수 있다. 이는 아주 reasonable한데, 왜냐하면 주로 모바일 로봇들이 1~3m/s로 움직이기 때문이다. 따라서 적어도 point-to-line distance가 0.5m 이내의 점들이 유효한 pair이고 그 이외의 점들은 extreme outliers일 가능성이 크다.


### calculateTransformationCorner(iterCount2)

Correspondences를 추정한 후, time t에서의 `cornerPointsSharp`에서 유효하다고 판단되어 선별된 `laserCloudOri`을 입력으로 하여 optimization을 진행한다. 크게 네 파트로 구성되어 있는데, i) jacobian matrix 계산, ii) solve the delta, iii) degeneracy check, iv) 수렴했는지 판단 으로 구성되어 있다.

```cpp
bool calculateTransformationCorner(int iterCount){

    int pointSelNum = laserCloudOri->points.size();

    cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

    float srx = sin(transformCur[0]);
    float crx = cos(transformCur[0]);
    float sry = sin(transformCur[1]);
    float cry = cos(transformCur[1]);
    float srz = sin(transformCur[2]);
    float crz = cos(transformCur[2]);
    float tx = transformCur[3];
    float ty = transformCur[4];
    float tz = transformCur[5];

    float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz; float b3 = crx*cry; float b4 = tx*-b1 + ty*-b2 + tz*b3;
    float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry; float b7 = crx*sry; float b8 = tz*b7 - ty*b6 - tx*b5;

    float c5 = crx*srz;

    for (int i = 0; i < pointSelNum; i++) {

        pointOri = laserCloudOri->points[i];
        coeff = coeffSel->points[i];

        float ary = (b1*pointOri.x + b2*pointOri.y - b3*pointOri.z + b4) * coeff.x
                  + (b5*pointOri.x + b6*pointOri.y - b7*pointOri.z + b8) * coeff.z;

        float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;

        float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;

        float d2 = coeff.intensity;

        matA.at<float>(i, 0) = ary;
        matA.at<float>(i, 1) = atx;
        matA.at<float>(i, 2) = atz;
        matB.at<float>(i, 0) = -0.05 * d2;
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (iterCount == 0) {
        cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        isDegenerate = false;
        float eignThre[3] = {10, 10, 10};
        for (int i = 2; i >= 0; i--) {
            if (matE.at<float>(0, i) < eignThre[i]) {
                for (int j = 0; j < 3; j++) {
                    matV2.at<float>(i, j) = 0;
                }
                isDegenerate = true;
            } else {
                break;
            }
        }
        matP = matV.inv() * matV2;
    }

    if (isDegenerate) {
        cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP * matX2;
    }

    transformCur[1] += matX.at<float>(0, 0);
    transformCur[3] += matX.at<float>(1, 0);
    transformCur[5] += matX.at<float>(2, 0);

    for(int i=0; i<6; i++){
        if(isnan(transformCur[i]))
            transformCur[i]=0;
    }

    float deltaR = sqrt(
                        pow(rad2deg(matX.at<float>(0, 0)), 2));
    float deltaT = sqrt(
                        pow(matX.at<float>(1, 0) * 100, 2) +
                        pow(matX.at<float>(2, 0) * 100, 2));

    if (deltaR < 0.1 && deltaT < 0.1) {
        return false;
    }
    return true;
}
```

 
 Plane feature도 위와 똑같은 process로 진행된다. 연습문제로 삼아서 스스로 전개해보길 추천한다.

---

LeGO-LOAM의 line-by-line 설명 시리즈입니다.
