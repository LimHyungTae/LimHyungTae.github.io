---
layout: post
title: OpenCV를 활용해서 RGB-D 카메라의 depth data png로 저장하기
subtitle: OpenCV로 RGB-D data parsing하기
tags: [OpenCV, RGB-D, png, imwrite]
comments: true

---
## Introduction

본 페이지는 SemanticKITTI 상에서 dynamic object 수를 plot한 결과입니다. 

## rgbd depth

1. CV_32FC1로 저장하는 방법, CV_16UC1로 저장하는 방법 두가지가 존재함
- CV_32FC1: `float`으로 m로 저장함.
- CV_16UC1: `unsigned short`로 mm로 저장함. <= 저장공간을 좀 덜 쓰니 이 방식으로 저장된게 아닐까 생각됨

출력하면 각각

- CV_16UC1 - 2
- CV_32FC1 - 5

Parse RGB-D data from Azure Kinect to png files.

Basically, this repository is to run *[StaticFusion](https://github.com/raluca-scona/staticfusion)-ImageSequenceAssoc* since they need offline png files, e.g. RGB png and depth png respectively.

Original author: Hyungtae Lim (shapelim@kaist.ac.kr)


## Objective

An RGB image and a depth whose types are `sensors_msgs::CompressedImage` is saved as a png file. 

## How To Use

1. Revise the rosparams in `launch/parse_rgbd.launch`.

2. Play your own rosbag file that contains corresponding RGB and depth data.

3. ```roslaunch rgbdsaver parse_rgbd.launch```

4. After finishing play the ros bag file, run `rostopic pub /savetext std_msgs/Float32 "data: 0.0"` to save **rgbd_assoc.txt** file. (line 29~39 in `node/src/RGBDSaver.cpp`)

5. Thus, the organisational structure of the dataset is as follows:

``` 
/$savedir$/rgb/   - folder containing all color images
/$savedir$/depth/ - folder containing all depth images
/$savedir$/rgbd_assoc.txt - Sequences for StaticFusion. If your goal is not to run StaticFusion, this file may not be necessary.
```

## Descriptions

***rgb***

* The decoded image is originally BGR, yet png for StaticFusion is RGB-order. 
* So, if you need a BGR-ordered png file, then annotate 52nd line in `node/src/RGBDSaver.cpp`

***depth***

* It is saved as 16 bit monochrome in PNG. That is, a depth image is saved to png file whose type is **CV_16UC1**, or `unsigned short` on millimeter scale. 
* A decoded depth data is originally **CV_32FC1** which is on meter scale (line 65 in `node/src/RGBDSaver.cpp`).
* However, **CV_32FC1** is automatically casted to **8UC1** when saving the data using `cv::imwrite()` :(. That's why we save the depth to **CV_16UC1**.
* `nan` data is substituted with 0 according to `std::isnan()`.
