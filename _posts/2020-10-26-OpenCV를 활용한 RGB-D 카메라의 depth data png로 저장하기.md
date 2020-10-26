---
layout: post
title: OpenCV를 활용한 RGB-D 카메라의 depth data png로 저장하기
subtitle: OpenCV로 RGB-D data parsing하기
tags: [OpenCV, RGB-D, png, imwrite]
comments: true

---
## Introduction

본 페이지는 StaticFusion 알고리즘을 돌리기 위해 RGB-D camera (Azure kinect)의 RGB 이미지와 depth 이미지를 png로 저장하는 방법에 대해 설명합니다. 

단순히 `cv::imwrite()` 함수를 이용하여 저장한 후, 이미지를 불러와서 사용하면 될 줄 알았는데, 잘 되지 않음을 확인했습니다.

full code는 [여기](https://github.com/LimHyungTae/rgbdsaver)에서 git clone을 하여 사용하시면 됩니다.

## RGB-D Data

ROS에서는 rosbag 내에 RGB data와 depth 이미지를 주로 CompressedImage msg로 제공합니다.

RGB 이미지야, `cv::imdecode()` 함수로 압축을 풀면 되지만, depth는 그렇지 않습니다.

## OpenCV에서 Depth data를 기술하는 방법

data를 `cv::Mat`으로 기술할 때는 CV_32FC1로 저장하는 방법, CV_16UC1로 저장하는 방법, 이렇게 두가지 방법이 있다.

* **CV_32FC1** (실제 값은 2): `float`으로, 미터 단위(m)로 기술함.
* **CV_16UC1** (실제 값은 5): `unsigned short (uint16_t)`로 미리미터 단위(mm)로 기술함. <= 저장공간을 좀 덜 쓰니 이 방식으로 저장된게 아닐까 생각됨

하지만, png 파일로 저장하기 위해서는 필수적으로 CV_16UC1로 기술해야한다 것을 경험적으로 확인했습니다.

[위 레포지토리](https://github.com/LimHyungTae/rgbdsaver)의 *node/src/RGBDSaver.cpp*의 65 번 째 줄을 보면, 처음 Azure kinect의 depth를 decode하면 type이 **CV_32FC1**로 되어 있습니다.

하지만, 이 depth를 `cv::write()` 함수를 통해 png로 저장한 후, `cv::imread()` 함수로 다시 불러와보면 data type이 강제로 **8UC1**로 되는 것을 확인할 수 있습니다.

따라서 depth data를 png로 저장하기 위해서는 **CV_16UC1**로 변환을 한 후, `cv::write()`를 해주어야 합니다.

## 정리

[위 레포지토리](https://github.com/LimHyungTae/rgbdsaver)의 *node/src/RGBDSaver.cpp* 코드 내부에 

* Azure Kinect로부터 얻은 Compressed depth image를 decode하여 `cv::Mat`의 **CV_32FC1**로 변환하는 법 (64~65번째 줄)
* **CV_32FC1** -> **CV_16UC1**로 변환하는 법 (70~81번째 줄)

등이 자세히 적혀 있습니다. :)
