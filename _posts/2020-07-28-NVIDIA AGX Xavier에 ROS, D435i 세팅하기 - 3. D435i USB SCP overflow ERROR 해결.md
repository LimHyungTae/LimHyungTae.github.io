---
layout: post
title: NVIDIA AGX Xavier에 ROS, D435i 세팅하기 - 3. D435i USB SCP overflow ERROR 해결
subtitle: NVIDIA AGX Xavier with Interl Realsense D435i
tags: [NVIDIA, Jetson, AGX, Xavier, ROS, D435i]
comments: true

---

USB SCP 에러가 뜨는 원인은 [여기](https://community.intel.com/t5/Items-with-no-label/D435-Windows-10-USB-SCP-overflow-depends-on-resolution-no/td-p/572017) 참조



## Solution

***"USB SCP overflow" appears at varying increasing rate, if I use 1280x720 for all sensors (rgb and stereo) at 30 fps. I reduce RGB camera resolution to 960 x 540 and I do not have overflow problem at all and I'm fine with it. It looks limited to USB3 bandwidth, not the CPU or memory.***

RGB-D sensor의 데이터 양이 많다보니 bandwidth 문제가 발생하는 것이었음. Resolution을 640x480으로 낮추니 해결되었음
