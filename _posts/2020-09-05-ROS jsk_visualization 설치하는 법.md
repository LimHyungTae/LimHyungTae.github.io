---
layout: post
title: ROS jsk_visualization 설치하는 법
subtitle: ROS Rviz Visualization 플러그인 설치
tags: [RViz, ROS, jsk, jsk_visaulization]
comments: true

---
## Introduction

본 페이지는 ROS-melodic에서 jsk_visualization를 사용하기 위해 세팅한 결과를 정리하기 위한 포스트입니다.

링크는 [여기](https://github.com/jsk-ros-pkg/jsk_visualization)를 확인해보시면 됩니다.

jsk_visualization을 설치하면 아래와 같이 기존에 없던 visualization msg들을 추가적으로 사용할 수 있습니다.

![example](/img/jsk_viz.png){: .center-block :}

*polygon array로 visualization한 것. Original Rviz에는 적합한 msg가 없어서 plugin을 추가해보았습니다 :) 이 외에도 3D bounding box등 여러가지 용이한 msg들이 있습니다*


## Install 

먼저 아래의 주소를 ros workspace에서 catkin_ws/src에 clone합니다.

```
git clone https://github.com/jsk-ros-pkg/jsk_visualization.git
```

그 후 build를 하면 되는데, 하다보면 dependency가 없다고 뜨므로, 아래 명령어로 dependency를 자동으로 설치할 수 있습니다.

```
rosdep install jsk_rviz_plugins
```

## Compile Error

그런데 jsk library가 좀 오래된 버전의 opencv으로 되어있어서, compile하는 데 에러가 나는 것을 확인할 수 있습니다.

해결하기 위해서는 video library과 관련된 코드를 조금 수정하면 되는데 아래와 같이 두 부분을 고치면 됩니다.

`video_capture_display.h`와 `video_capture_display.cpp`

* 1. 편의를 위해 namespace 설정

using namespace cv;

* 2. `video_capture_display.cpp`의 202번 줄

```
writer_.open(file_name_, CV_FOURCC_DEFAULT, fps_, cv::Size(width_, height_));
```

to

```
int videowriter = VideoWriter::fourcc('I', 'Y', 'U', 'V');
// writer_.open(file_name_, CV_FOURCC_DEFAULT, fps_, cv::Size(width_, height_));
writer_.open(file_name_, videowriter, fps_, cv::Size(width_, height_));
```

* 3. `video_capture_display.cpp`의 235번 줄

```
cv::cvtColor(image, image, CV_RGB2BGR);  // RGB -> BGR
```

to
```
cv::cvtColor(image, image, COLOR_RGB2BGR);  // RGB -> BGR
```

