---
layout: post
title: IMU Preintegration (Easy) - 5. IMUPreintegration in LIO-SAM
subtitle: Line-by-line explanations of IMU Preintegration in LIO-SAM
tags: [SLAM, LIO, VIO, IMU, Preintegration]
comments: true

---

최종적으로 LIO-SAM 내부의 `imuPreintegration.cpp` 코드가 어떻게 작동되는지를 알아본다. 코드가 꽤 긴데, 원 코드는 [여기](https://github.com/TixiaoShan/LIO-SAM/blob/master/src/imuPreintegration.cpp) 참조.

## Procedure

IMU preintegration 코드 구성은 크게 a) 생성자, b) `imuHandler` Callback, c) `odometryHandler` Callback으로 구성되어 있다.

**a) 생성자 (Constructor)**: 가장 먼저, 코드를 실행하면 주어진 Subscriber, Publisher, 파라미터들, `gtsam::PreintegratedImuMeasurements`을 세팅한다. 코드 원문은 아래와 같다:

```cpp
IMUPreintegration()
{
    subImu      = nh.subscribe<sensor_msgs::Imu>  (imuTopic, 2000, &IMUPreintegration::imuHandler, this, ros::TransportHints().tcpNoDelay());
    subOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 5,    &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());

    pubImuOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000);

    boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
    p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
    p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
    p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
    gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

    priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
    priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
    priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
    correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
    correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
    noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
    
    imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
    imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization        
}
```


**b) imuHandler Callback**: 클래스의 생성자가 실행된 이후, 가장 먼저 `imuHandler` 콜백이 실행된다. `imuHandler` 콜백과는 달리 `odomHandler` callback은 LIO-SAM을 실행시켰을 때 ImageProjection→FeatureExtraction→MapOptimization의 일련의 과정을 거친 후에 생성되는 world 좌표계 기준 pose를 받는 callback이다. 그렇기 때문에 처음 LIO-SAM을 실행시키면 raw IMU를 쌓는 `imuHandler` 콜백이 `odomHandler` 콜백보다 먼저 실행이 된다.

`imuHandler` 콜백의 역할은 크게 세 가지로 나뉘는데,

1. IMU 데이터를 `imuQueOpt`와 `imuQueImu`에 queue에 저장한다.
2. 저장을 한 후, 만약 `odomHandler`에서 optimization이 실행이 아직 되지 않았으면 return을 하고, preintegrationd을 활용한 factor graph optimization이 시행될 때까지 기다린다 (시행되면 `doneFirstOpt`가 true로 됨).
3. 그 후, motion integration을 통해, IMU 데이터들을 활용하여 계산한 odometry pose를 publish한다.

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

여기서 과정 2에서 왜 리턴하고 끝을 내는지에 대해서는 다시 생각해보면, **아직 bias term들이 optimization을 통해 추정되지 않았기 때문**이다. 다시 수식으로 돌아가서 살펴보자면, 처음 해당 코드가 실행됐을 때는 아래의 빨간 박스에 해당하는 값들이 update가 안 되어 있는 상태이다:

![](/img/preintegration/IMU_equation_for_lio_sam.png)


 따라서 biases가 부정확한 상황에서 `integrateMeasurement()` 함수를 통해 position을 prediction하게 되면 그 결과가 매우 부정확하다. 그렇기에 `odometryHandler` 콜백에서 처음 optimization을 하기 전까지 IMU data를 저장만할 뿐 prediction은 하지 않는다 (참고로 `if (doneFirstOpt == false)`를 주석처리하면 pose가 발산하여 우주로 날아가는 것을 확인할 수 있다).

참고로 위의 그림의 $$\boldsymbol{\eta}^{ad}(t)$$와 $$\boldsymbol{\eta}^{gd}(t)$$는 생성자에서 값을 할당해준 `p->accelerometerCovariance`와 `p->gyroscopeCovariance`에 대응된다.


**c) odometryHandler Callback**: `odomHandler` callback은 LIO-SAM을 실행시켰을 때 ImageProjection→FeatureExtraction→MapOptimization의 일련의 과정을 거친 후에 생성되는 world 좌표계 기준의 keyframe의 pose를 콜백으로 받아온다. 콜백을 통해 받아오는 첫 pose의 posiiton은 (0, 0, 0)이고, rotation은 `params.yaml`의 `/lio_sam/useImuHeadingInitialization`이 true면 절대 좌표계로, false이면 (0, 0, 0, 1)의 quaternion으로 표현된다. 이 콜백은 아래의 순서로 동작한다.

i) 가장 먼저, msg가 오면 keyframe의 pose의 timestamp를 `currentCorrectionTime`로 세팅하고, pose 값을 gtsam library의 pose format으로 변환시킨다:

```cpp
double currentCorrectionTime = ROS_TIME(odomMsg);

float p_x = odomMsg->pose.pose.position.x;
float p_y = odomMsg->pose.pose.position.y;
float p_z = odomMsg->pose.pose.position.z;
float r_x = odomMsg->pose.pose.orientation.x;
float r_y = odomMsg->pose.pose.orientation.y;
float r_z = odomMsg->pose.pose.orientation.z;
float r_w = odomMsg->pose.pose.orientation.w;
bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));
```

ii) 그 후, factor graph optimization을 위한 initialization을 시행하기 위해 세팅한다. 여기서 preintegration은 **$$i$$ 번째 keyframe과 $$j$$ 번째 keyframe 사이**의 measurments를 하나의 factor로 만든다는 것에 주목하자. 즉, optimization을 하기 위해서는 pose가 적어도 2개가 필요하는 것을 알 수 있다. 따라서 첫 pose가 왔을 때는 주어진 pose가 하나밖에 없기 때문에 optimization을 시행할 수 없다. 따라서 `if (systemInitialized == false)` 문 내부에 있는 코드를 실행하여 초기 값들을 세팅한 후, 콜백이 return된다. 

다시 이 부분은 크게 두 가지로 나뉘는데, 첫 번째로는 오래된 IMU data가 들어온 시간이 `currentCorrectionTime` 보다 작은 (즉, 현재 pose 보다 이전에 들어온 data라는 것을 의미) data들을 queue에서 아래와 같이 제거한다.

![](/img/preintegration/imu_queue.png)

(preintegration은 $$i$$ 번째 keyframe과 $$j$$ 번째 keyframe **사이**의 measurments를 하나의 factor로 만든다는 것을 다시 한 번 기억하자!!! 그러니 그 이전의 값들은 preintegrated measurements를 만드는데 필요 없다.)

두번 째로는, optimization에 사용되는 factor graph(`gtsam::ISAM2`)를 초기화하고, preintegration measurment (`gtsam::PreintegratedImuMeasurements`)들도 biases과 Jacobian term들을 초기화 한다. 이를 위해 `optimizer`에 optimization을 시행하기 위한 초기 값을 세팅하고, preintegration을 위해 `imuIntegratorImu_`과 `imuIntegratorOpt_`도 reset을 해둔다. 참고로, 아래 두 줄의 `resetIntegrationAndSetBias()` 함수는:

```cpp
imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
```

[여기](http://docs.ros.org/en/melodic/api/gtsam/html/PreintegrationBase_8cpp_source.html)를 살펴보면 

```cpp
void PreintegrationBase::resetIntegrationAndSetBias(const Bias& biasHat) {
    biasHat_ = biasHat;
    resetIntegration();
}
```

로 정의되어 있고, `resetIntegration()` 함수는 아래와 같이 정의되어 있다.

```cpp
void ManifoldPreintegration::resetIntegration() {
   deltaTij_ = 0.0;
   deltaXij_ = NavState();
   delRdelBiasOmega_.setZero();
   delPdelBiasAcc_.setZero();
   delPdelBiasOmega_.setZero();
   delVdelBiasAcc_.setZero();
   delVdelBiasOmega_.setZero();
}
```

위에서 `deltaTij_`는 i와 j 간의 시간차,`deltaXij_`는 우리가 최종적으로 계산해야하는 preintegrated measurements를 뜻한다, i.e. 아래의 초록색 박스들의 값. 

![](/img/preintegration/preinteg_set_bias.png)

NavState는 [여기](https://gtsam.org/doxygen/a04144.html)를 보면 Rot3, Point3, Velocity3의 멤버 변수를 지니는 것을 확인할 수 있다. `deltaXij_` 변수 아래의 5개 term은 bias를 optimization할 때 쓰는 Jacobian을 계산하기 위한 term이다. 요약하자면, 이를 통해 $$i$$와 $$j$$ 사이의 시간 차, preintegrated measurements, Jacobian terms들 모두 초기화된다는 것을 확인할 수 있다. 

iii) 그 다음, 이제 첫번 째 pose 이후의 pose가 들어오면 optimization을 시행하여 biases를 correction한다. 먼저, $$j$$ 번째 pose가 들어오면 $$j-1$$과 $$j$$ 번째 pose 사이의 IMU measurement를 이용해서 preintegrated measurements를 계산한다.

```cpp
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
```

그 다음, `imuIntegratorOpt_`에 저장되어 있는 preintegrated measurements를 입력으로 하여 IMU factor를 세팅한다.

```cpp
// add imu factor to graph
const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
graphFactors.add(imu_factor);
```

그리고 우리가 추정하고자 하는 $$j-1$$과 $$j$$ 번째 사이의 bias도 optimization에 추가해준다. 여기서 보면 앞서서 preintegrated measurements를 증명했을 때 사용했던, bias가 두 keyframe 사이에서는 constant하다는 가정을 `gtsam::imuBias::ConstantBias()`을 통해 나타낸 것을 확인할 수 있다.

```cpp
// add imu bias between factor
graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                    gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
```

그 후 코드 상 아래 부분은 `opimizer`에 factor graph들과 그에 해당하는 초기 값들은 대입해주는 과정이고, `optimizer`의 결과는 아래와 같이 멤버 변수에 할당한다.

```cpp
gtsam::Values result = optimizer.calculateEstimate();
prevPose_  = result.at<gtsam::Pose3>(X(key));
prevVel_   = result.at<gtsam::Vector3>(V(key));
prevState_ = gtsam::NavState(prevPose_, prevVel_);
prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
```

최종적으로 우리가 구하고자 한 biases는 `prevBias_`로 주어지는데, 이 값은 optimization을 통해 추정된 IMU의 biases이다. 따라서 update된 biases들을 이용하면 IMU의 좀 더 정확한 incremental한 움직임을 추정할 수 있게 된다. 그래서 아래의 코드에서는 $$j$$ 번째 pose가 들어온 데까지 IMU data들에 motion integration을 하여 IMU data 기준 pose를 motion integration을 통해 구한다.

```cpp
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
```

IMU data들이 `imuQueOpt`와 `imuQueImu`로 똑같은 값들이 복사되어 있다는 것을 주의하자. 이 두 변수들의 상관관계가 다소 헷갈리는데, 정리하자면 각각의 역할은 아래와 같다.

* `imuQueOpt`-`imuIntegratorOpt_`: Preintegration을 통해서 IMU factor를 생성하기 위함. 따라서 두 keyframe 사이의 IMU data들로부터 preintegration을 하여 preintegrated measurement를 계산하고, 이 계산한 값을 optimizer에 넘겨 줌.
* `imuQueImu`-`imuIntegratorImu_`: Optimization의 결과 값을 입력으로 받아 motion integration을 시행. 따라서 원래 LiDAR data의 Hz로 출력되던 pose를 IMU data의 Hz로 올려 줌.

위의 사항을 알고 다시 `imuHandler()` 함수((b)의 3번째 파트)를 살펴보자. Optimization이 한번이라도 시행되면 `odometryHandler()` 함수 내부에서 `doneFirstOpt`가 true로 세팅된다. 그렇게 되면 IMU data가 `imuHandler()`를 통해 입력으로 들어왔을 때 함수의 아래쪽도 시행이 된다. 아래 부분에서는 integration을 하면서 incremental한 relative pose를 `imuIntegratorImu_`가 계산해주는 것을 아래의 코드 파트에서 확인할 수 있다.

```cpp
// integrate this single imu message
imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                        gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt);

// predict odometry
gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);
```

P.S. `odometryHandler()` 내부의 `if (key == 100)` 문은 factor들을 계속 추가하면 요구하는 메모리가 계속 증가하기 때문에 그 것을 리셋해주는 부분이라고 이해하면 된다. 이를 **marginalization**이라 부르는데, VINS-Mono 내부에서도 sliding window를 할 때 marginalization을 시행한다. 이 부분에서는 100개 이상의 key pose가 쌓이면 이때까지의 모든 pose들과 IMU factor를 하나의 prior로 정보를 축약하고, 이를 다시 활용하여 optimization을 진행한다. 자세한 사항은 "schur complement"를 검색해보길...

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