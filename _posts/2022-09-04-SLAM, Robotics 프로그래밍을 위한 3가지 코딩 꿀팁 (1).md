---
layout: post
title: SLAM, Robotics 프로그래밍을 위한 3가지 코딩 꿀팁 (1)
subtitle: 1. Transformation matrix의 기준이 어디인지 "잘" 명시하자.
tags: [SLAM, Robotics, ROS]
comments: true
---

# 서론

근래에는 SLAM open source 코드들도 많고 SLAM 및 mapping 관련 업무를 다루는 회사도 늘어나다보니, 다른 동료들과 함께 코드를 짜야할 일이 생긴다. 그러다 보면 다 알아보겠거니 하고 크게 신경쓰지 않았으나, 의도치 않게 코드를 읽어봐야 하는 이(혹은 미래의 '나')에게 피로감을 주는 일들이 생긴다. 본 시리즈는 필자가 석박사 과정 동안 시행착오를 겪은 후 깨달은, 로봇의 3D landmark나 pose를 다룰 때 알아두고 있으면 좋을 세가지 꿀팁을 제시한다.


사실 제목은 거창하게 지었으나, 더 정확히 말하자면 '동료 SLAM 개발자들과 함께 코딩할 때 그들의 편의성을 올려주는 방법'이 더 부합한 거 같다. 대규모로 코딩을 하지 않는 나같은 연구자들에게는 '지금 짜고 있는 코드를 수 년 뒤에 다시 찾아본 후 사경(?)을 헤맬 미래의 '나'를 위해 코드를 좀 더 알아보기 쉽게 작성하는 법'이라고 이름을 붙이고 싶다. 하지만 어그로(?)를 위해 제목은 3가지 코딩꿀팁이라 남겨둔다.


![img](/img/wft_p_minute.jpeg)

*더 정확히는 SLAM 및 robotics 관련 개발을 할 때 코드를 읽어보는 사람의 'WTFs/minute'을 줄일 수 있는 방법에 대해 알아본다*


이 분야에 오래 종사한 사람어도, 다른 사람의 코드를 깊게, 또는 많이 읽어보지 않은 이들에게는 도움이 되리라 생각된다. 따라서 생산성 향상을 위해 한 번쯤 읽어보면 좋을 듯 싶다. 이해를 돕기 위해 오픈 소스들의 snippet도 첨부해가며 설명할 예정이다.

---

## 1. Transformation matrix의 기준이 어디인지 "잘" 명시하자. 

### Problem

`T_LiDAR_TO_CAM`라는 transformation matrix를 나타내는 변수가 있다고 가정하자 (transformation matrix가 친숙하지 않은 분은 [여기](https://limhyungtae.github.io/2021-09-10-ROS-Point-Cloud-Library-(PCL)-3.-Transformation/) 참조). 이를 어떻게 해석할 수 있을까? 정답은, 의미를 알기 위해서는(다시 말해 "to"의 의미를 해석하기 위해서는) 해당 변수의 사용처를 **꼭** 살펴봐야 한다. 따라서 변수 하나를 이해하기 위해 코드를 일일이 다 뜯어봐야 하기 때문에 코드를 읽는 동료를 상당히 귀찮게 만든다. 

어떻게 다르게 사용되는지에 대해서 자세히 살펴보자. 위처럼 표기되는 변수는 크게 두가지로 **완전히 다른 의미로 사용된다**는 것을 강조한다. 

---

### Case A. Point Cloud 관점에서의 Transformation Matrix

LiDAR 좌표계를 원점으로 한 point를 $$^\text{L}\mathbf{p}$$라 했을 때, point를 camera 좌표계를 기준으로 옮길 때의 matrix를 뜻함. 즉, *with respect to camera coorinate (카메라 좌표계를 기준점으로 바라봤을 때)*라는 의미로 쓰였으나 "with respect"를 생략된 경우로, 4x1의 point의 **왼 쪽**에 곱해져서 추정하고자 하는 point의 축을 camera 좌표계로 바꿀 때. 즉, `T_LiDAR_TO_CAM`을 $${^{\text{C}}_\text{L}\mathbf{T}}$$라고 표현하면, $${^{\text{C}}_\text{L}\mathbf{T}} * {^\text{L}\mathbf{p}} \rightarrow {^\text{C}\mathbf{p}}$$의 의미로 사용됨 ($$\text{L}$$: LiDAR 좌표계, $$\text{C}$$: 카메라 좌표계를 뜻함)
표기가 익숙치 않은 분들은 *Introduction to Robotics* textbook의 chapter 2 정독을 추천드립니다! 

**예시:** 아래의 [ERASOR](https://github.com/LimHyungTae/ERASOR)의 코드 내부에서 i) PointCloud2 msg를 `ptr_query`로 할당한 후, ii) voxelization을 하고, ii) `tf_lidar2body_`라는 변수를 통해 LiDAR 좌표계 기준인 point cloud를 world 좌표계 기준 좌표계로 옮기는 것을 볼 수 있다 (다시 보니 `tf_lidar2world_`가 좀 더 바람직한 변수명일듯 싶다. ~~이래서 리팩토링이 중요하다!~~).

```cpp
pcl::fromROSMsg(msg->lidar, *ptr_query);
erasor_utils::voxelize_preserving_labels(ptr_query, *ptr_query_voxel, query_voxel_size_);

pcl::transformPointCloud(*ptr_query_voxel, *ptr_query_body, tf_lidar2body_);
```

---

### Case B. Pose 관점에서의 Transformation Matrix

현재의 pose가 world frame 관점에서 바라본(*with respect to world frame*) LiDAR의 pose라 할때, transformation matrix **오른쪽에** 곱해져서 추정하고자 하는 pose의 축을 camera 좌표계로 바꿀 때. 
다시 말해, `T_LiDAR_TO_CAM`은 $${^{\text{L}}_\text{C}\mathbf{T}}$$로, $${^{\text{W}}_\text{L}\mathbf{T}} * {^{\text{L}}_\text{C}\mathbf{T}} \rightarrow {^{\text{W}}_\text{C}\mathbf{T}}$$ ($${^\text{W}_\text{L}\mathbf{T}}$$: World 좌표계 기준에서 본 LiDAR의 pose) 


**예시:** 대표적인 예시로, 아래와 같이 KITTI의 ground truth를 transformation할 때 사용된다. KITTI는 ground truth가 카메라 좌표계를 기준으로 작성되어 있기 때문에, LiDAR에 대한 ground truth가 필요하면 그 축을 옮겨주어야 한다. 따라서 아래 보이는 것 처럼 `KITTI_CAM2LIDAR`라는 변수를 `tf4x4_cam`의 오른쪽에 곱하여 원점 기준 LiDAR의 pose로 변환하는 것을 볼 수 있다.

```cpp
void loadAllKittiPoses(string txt, vector<Eigen::Matrix4f> &poses) {
    // Pose is transformed into lidar pose!
    Eigen::Matrix4f KITTI_CAM2LIDAR;
    KITTI_CAM2LIDAR << -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
            -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02,
            9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
            0, 0, 0, 1;

    Eigen::Matrix4f tf_origin;
    tf_origin << 0, 0, 1, 0,
            -1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, 0, 1;

    poses.clear();
    poses.reserve(2000);
    std::ifstream in(txt);
    std::string line;

    int count = 0;
    while (std::getline(in, line)) {
        vector<float> pose = pose_utils::splitLine(line, ' ');
        Eigen::Matrix4f tf4x4_cam = Eigen::Matrix4f::Identity();
        pose_utils::vec2tf4x4(pose, tf4x4_cam);
        Eigen::Matrix4f tf4x4_lidar = tf_origin * tf4x4_cam * KITTI_CAM2LIDAR;
        poses.emplace_back(tf4x4_lidar);
        count++;
    }
    in.close();
    std::cout << "Total " << count << " poses are loaded" << std::endl;
}
```

--

### 꿀팁 (1)의 결론 및 해결책

무심코 사용한 `tf_A_to_B`나 `T_A_to_B`라는 표현이 누군가에게는 다르게 해석되어서 코드를 잘못 이해하거나, 변수의 의미를 이해하기 위해서는 변수의 사용처를 찾아봐야 하다보니 큰 피로감을 야기할 수도 있다. 따라서 여럿이서 코드를 작성한다면, 이러한 네이밍 컨벤션(naming convention)을 처음에 한 번 토의를 한 후에 코드를 작성하면 좀 더 이런 귀찮은 이슈를 줄일 수 있을 것이다. 
예로 들어, case (a)를 표현하고자 할 때는 `A_wrt_B`(`B` 좌표계 관점에서 바라본 `A`)로 사용하고 case (b)의 경우에는 `A_to_C`(`A` 좌표계를 `C` 좌표계로 transformation)라고 표현하자던가 등등. 

개인적으로 [GTSAM convention](https://gtsam.org/gtsam.org/2020/06/28/gtsam-conventions.html)도 굉장히 깔끔하다고 생각한다, i.e., aTb와 같이 표기하기. 

---

SLAM, Robotics 프로그래밍을 위한 3가지 코딩 꿀팁 시리즈입니다.

{% include post_links_tips_for_robotics.html %}
