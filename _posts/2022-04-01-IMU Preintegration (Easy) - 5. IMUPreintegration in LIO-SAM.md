---
layout: post
title: 2022-04-01-IMU Preintegration (Easy) - 5. IMUPreintegration in LIO-SAM
subtitle: Line-by-line explanations of IMU Preintegration in LIO-SAM
tags: [SLAM, LIO, VIO, IMU, Preintegration]
comments: true

---

# Derivation of Preintegrated IMU Measurements

본 글에서는 이제 K개의 IMU measurement를 통해 표현한 $$i$$와 $$j$$ 번째의 Relative motion을 하나의 요소로 미리 취합(preintegration)하는 방법에 대해 설명한다 (수학주의).


```cpp
void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
{
    std::lock_guard<std::mutex> lock(mtx);

    sensor_msgs::Imu thisImu = imuConverter(*imu_raw);

    imuQueOpt.push_back(thisImu);
    imuQueImu.push_back(thisImu);

    if (doneFirstOpt == false)
        return;

    double imuTime = ROS_TIME(&thisImu);
    double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
    lastImuT_imu = imuTime;

    // integrate this single imu message
    imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                            gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt);

    // predict odometry
    gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

    // publish odometry
    nav_msgs::Odometry odometry;
    odometry.header.stamp = thisImu.header.stamp;
    odometry.header.frame_id = odometryFrame;
    odometry.child_frame_id = "odom_imu";

    // transform imu pose to ldiar
    gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
    gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

    odometry.pose.pose.position.x = lidarPose.translation().x();
    odometry.pose.pose.position.y = lidarPose.translation().y();
    odometry.pose.pose.position.z = lidarPose.translation().z();
    odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
    odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
    odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
    odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
    
    odometry.twist.twist.linear.x = currentState.velocity().x();
    odometry.twist.twist.linear.y = currentState.velocity().y();
    odometry.twist.twist.linear.z = currentState.velocity().z();
    odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
    odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
    odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
    pubImuOdometry.publish(odometry);
}
```

```cpp
void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    std::lock_guard<std::mutex> lock(mtx);

    double currentCorrectionTime = ROS_TIME(odomMsg);

    // make sure we have imu data to integrate
    if (imuQueOpt.empty())
        return;

    float p_x = odomMsg->pose.pose.position.x;
    float p_y = odomMsg->pose.pose.position.y;
    float p_z = odomMsg->pose.pose.position.z;
    float r_x = odomMsg->pose.pose.orientation.x;
    float r_y = odomMsg->pose.pose.orientation.y;
    float r_z = odomMsg->pose.pose.orientation.z;
    float r_w = odomMsg->pose.pose.orientation.w;
    bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
    gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));


    // 0. initialize system
    if (systemInitialized == false)
    {
        resetOptimization();

        // pop old IMU message
        while (!imuQueOpt.empty())
        {
            if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
            {
                lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                imuQueOpt.pop_front();
            }
            else
                break;
        }
        // initial pose
        prevPose_ = lidarPose.compose(lidar2Imu);
        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
        graphFactors.add(priorPose);
        // initial velocity
        prevVel_ = gtsam::Vector3(0, 0, 0);
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
        graphFactors.add(priorVel);
        // initial bias
        prevBias_ = gtsam::imuBias::ConstantBias();
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
        graphFactors.add(priorBias);
        // add values
        graphValues.insert(X(0), prevPose_);
        graphValues.insert(V(0), prevVel_);
        graphValues.insert(B(0), prevBias_);
        // optimize once
        optimizer.update(graphFactors, graphValues);
        graphFactors.resize(0);
        graphValues.clear();

        imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        
        key = 1;
        systemInitialized = true;
        return;
    }


    // reset graph for speed
    if (key == 100)
    {
        // get updated noise before reset
        gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
        gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
        gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
        // reset graph
        resetOptimization();
        // add pose
        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
        graphFactors.add(priorPose);
        // add velocity
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
        graphFactors.add(priorVel);
        // add bias
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
        graphFactors.add(priorBias);
        // add values
        graphValues.insert(X(0), prevPose_);
        graphValues.insert(V(0), prevVel_);
        graphValues.insert(B(0), prevBias_);
        // optimize once
        optimizer.update(graphFactors, graphValues);
        graphFactors.resize(0);
        graphValues.clear();

        key = 1;
    }


    // 1. integrate imu data and optimize
    while (!imuQueOpt.empty())
    {
        // pop and integrate imu data that is between two optimizations
        sensor_msgs::Imu *thisImu = &imuQueOpt.front();
        double imuTime = ROS_TIME(thisImu);
        if (imuTime < currentCorrectionTime - delta_t)
        {
            double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
            imuIntegratorOpt_->integrateMeasurement(
                    gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                    gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
            
            lastImuT_opt = imuTime;
            imuQueOpt.pop_front();
        }
        else
            break;
    }
    // add imu factor to graph
    const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
    gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
    graphFactors.add(imu_factor);
    // add imu bias between factor
    graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                        gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
    // add pose factor
    gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
    gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
    graphFactors.add(pose_factor);
    // insert predicted values
    gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
    graphValues.insert(X(key), propState_.pose());
    graphValues.insert(V(key), propState_.v());
    graphValues.insert(B(key), prevBias_);
    // optimize
    optimizer.update(graphFactors, graphValues);
    optimizer.update();
    graphFactors.resize(0);
    graphValues.clear();
    // Overwrite the beginning of the preintegration for the next step.
    gtsam::Values result = optimizer.calculateEstimate();
    prevPose_  = result.at<gtsam::Pose3>(X(key));
    prevVel_   = result.at<gtsam::Vector3>(V(key));
    prevState_ = gtsam::NavState(prevPose_, prevVel_);
    prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
    // Reset the optimization preintegration object.
    imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
    // check optimization
    if (failureDetection(prevVel_, prevBias_))
    {
        resetParams();
        return;
    }


    // 2. after optiization, re-propagate imu odometry preintegration
    prevStateOdom = prevState_;
    prevBiasOdom  = prevBias_;
    // first pop imu message older than current correction data
    double lastImuQT = -1;
    while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
    {
        lastImuQT = ROS_TIME(&imuQueImu.front());
        imuQueImu.pop_front();
    }
    // repropogate
    if (!imuQueImu.empty())
    {
        // reset bias use the newly optimized bias
        imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
        // integrate imu message from the beginning of this optimization
        for (int i = 0; i < (int)imuQueImu.size(); ++i)
        {
            sensor_msgs::Imu *thisImu = &imuQueImu[i];
            double imuTime = ROS_TIME(thisImu);
            double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);

            imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                    gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
            lastImuQT = imuTime;
        }
    }

    ++key;
    doneFirstOpt = true;
}
```

## Preintegrated IMU Measurements

앞서 유도한 아래의 수식을 다시 살펴보자.

![](/img/preintegration/final_i_j.png)

위의 수식에서 $$\mathtt{R}_i$$, $$\mathbf{v}_i$$, $$\mathbf{p}_i$$를 좌변으로 이항해주면 $$i$$ 번째 keyframe과 $$j$$ 번째 keyframe 사이의 최종적인 relative motion에 대해 표현할 수 있다 


위의 수식을 통해 $$i$$ 번째와 $$j$$의 pose 차이를 기술할 수 있다. 하지만 그렇게 되면 앞서 말한 것 처럼 두 keyframe 사이에서 변하는 pose에 대한 parameter를 factor graph 상에서 다 저장해야하기 때문에 비효율적이다. 

이러한 문제를 해결하기 위해 논문의 저자는 수백 여개의 measurments를 factor로 직접적으로 넣는 것이 아니라, factor graph에 measurement를 추가하기 이전에 (pre-) 수백여개의 IMU data를 단 하나의 factor로 취합(integration)하는 방법에 대해서 제안을 하는 것이다.

### Definition & Assumption

Preintegration의 과정을 유도하기 앞서 논문에서는 아래와 같이 *relative motion increments*에 대해서 정의한다:
![](/img/preintegration/relative_motion_increments.png)


여기서 주의할 것은, 이 *relative motion increments*는 물리적으로 incremental motion을 뜻하는 것은 아니다. 왜냐하면 전개의 편의성을 위해서 중력에 관련된 term도 좌변으로 이항했기 때문이다. 실제로 물리적인 motion (  아래의 자주색으로 표시된 부분)은 각 $$k$$ 번째와 $$k+1$$의 물리적인 미세한 relative motion을 의미한다.

![](/img/preintegration/physical_meaning.png)


그 후, 수식을 전개하기 전 $$i$$ 번째와 $$j$$ keyframe 사이에서는 bias가 일정하다고 가정한다.

![](/img/preintegration/preint_bias.png)

(실제로 bias는 주로 MEMS system 상에 전지적 신호의 offset으로 인해 발생하는 것이기 떄문에 위처럼 가정을 해도 괜찮다. 자세한 설명은 [여기](http://www.canalgeomatics.com/knowledgebase/imu-accuracy-error-definitions/ 참조)


이러한 가정 이후 아래와 같이 rotation, velocity, position에 대한 preintegration term을 표현한다 (아래 rotation term의 전개가 이해가 안 되시는 분은 Joan Sola의 님의 [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508),,,Chapter 4까지,,,꼭 읽어보시길 바랍니다,,,).

### Preintegrated Rotation Measurements

![](/img/preintegration/preinteg_rot.png)

### Preintegrated Velocity Measurements
![](/img/preintegration/preinteg_vel.png)

### Preintegrated Position Measurements

![](/img/preintegration/preinteg_pos.png)


## Final Results

위의 전개를 하다보면 '이걸 왜 하고 있는거지...?'라는 의문이 들 것이다. 최종적으로는 아래와 같이 표현을 할 수 있고:

![](/img/preintegration/behave_like.png)

Rotation, velocity, position을 factor에 추가할 때 IMU를 통해 저 covariance term만 추정해주면 되는 것으로 과정이 simplify되는 것을 볼 수 있다.

즉, 위의 식은 우리 주변의 3차원을 표현하는 manifold 상의 uncertainty를 3x1 크기를 지니고 zero-mean인 gaussian distribution으로 표현할 수 있음을 뜻한다.  

위의 수식 [1]을 활용하면 $$ i $$ 번째 world 좌표계 기준의 rotation과 $$i$$ 번째와 인접하는 $$j$$ 번째 world 좌표계 기준 사이의 relative rotation과 그에 대한 uncertainty 또한 아래와 같이 나타낼 수 있고: 

$$\Delta \tilde{\mathtt{R}}_{ij}=\Delta \mathtt{R}_{i}^{\intercal} \mathtt{R}_{j} \operatorname{Exp}(\delta \boldsymbol{\phi}_{ij}) \; \;  \; \; \; \; \; \text{[2]}$$ 

Factor graph SLAM의 최종 목표가 아래의 optimization 수식을 푸는 것과 같은데:

$$\mathbf{x}^{*}=\operatorname{argmin} \sum_{\mathbf{x}} \mathbf{e}_{i j}^{T} {\Omega}_{i j} \mathbf{e}_{i j} \; \; \; \; \;  \; \text{[3]}$$

[2]의 $$\Omega_{ij}$$에 대응되는 부분 (어려운 말로는 information matrix라 부름)을 저 $$\delta \boldsymbol{\phi}_{ij}$$의 역수를 통해 modeling이 가능해진다. ($$\delta \boldsymbol{\phi}_{ij}$$이 크다 → relative rotation에 대한 measurements가 불확실하다는 의미 → 해당 measurements에 해당하는 error 크기의 중경도를 따질 때 덜 중요하다고 여김, i.e. 지닌 information의 중요한 정도가 낮다고 판단). **따라서 기존의 factor graph SLAM의 objective function에 loss term을 끼워넣는 것이 가능해진다!**



## Bias Update 


![](/img/preintegration/bias_description.png)

## 결론

* 

---

IMU Preintegration Derivation 설명 시리즈입니다.

TBU
---


### 원 논문

* [Foster *et al.*, "IMU Preintegration on Manifold for Efficient
Visual-Inertial Maximum-a-Posteriori Estimation," in *Robotics: Science and Systems (RSS), 2011](http://www.roboticsproceedings.org/rss11/p06.pdf).
* [Foster *et al.*, "On-Manifold Preintegration for Real-Time
Visual-Inertial Odometry," *Transaction on Robotics*, 2016](https://rpg.ifi.uzh.ch/docs/TRO16_forster.pdf)