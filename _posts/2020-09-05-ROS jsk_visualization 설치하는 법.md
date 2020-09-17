---
layout: post
title: ROS jsk_visualization 설치하는 법
subtitle: ROS Rviz Visualization 플러그인 설치
tags: [NVIDIA, RViz, ROS, jsk, jsk_visaulization]
comments: true

---

본 페이지는 ROS-melodic에서 jsk_visualization를 사용하기 위해 세팅한 결과를 정리하기 위한 포스트입니다.

링크는 [여기](https://github.com/jsk-ros-pkg/jsk_visualization)를 확인해보시면 됩니다.

```
git clone https://github.com/jsk-ros-pkg/jsk_visualization.git
```

```
rosdep install jsk_rviz_plugins
```

## 문제점
```
#include <opencv2/opencv.hpp>
```

using namespace cv

video_capture_display.h

### 202번 줄

int videowriter = VideoWriter::fourcc('I', 'Y', 'U', 'V');
// writer_.open(file_name_, CV_FOURCC_DEFAULT, fps_, cv::Size(width_, height_));
writer_.open(file_name_, videowriter, fps_, cv::Size(width_, height_));

### 237번 줄

cv::cvtColor(image, image, COLOR_RGB2BGR);  // RGB -> BGR
