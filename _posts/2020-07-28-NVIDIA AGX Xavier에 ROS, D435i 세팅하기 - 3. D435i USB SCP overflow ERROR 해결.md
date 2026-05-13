---
layout: post
title: NVIDIA AGX Xavier에 ROS, D435i 세팅하기 - 3. D435i USB SCP overflow ERROR 해결
subtitle: NVIDIA AGX Xavier with Intel Realsense D435i
tags: [NVIDIA, Jetson, AGX, Xavier, ROS, D435i]
comments: true
description: D435i를 1280x720 30fps로 돌릴 때 발생하는 "USB SCP overflow" 에러가 USB3 bandwidth 한계 때문임을 짚고, RGB 해상도를 640x480으로 낮춰 해결하는 방법을 공유한다.
permalink: /2020/07/28/nvidia-xavier-d435i-setup-03-usb-overflow-fix/
redirect_from:
  - '/2020-07-28-NVIDIA AGX Xavier에 ROS, D435i 세팅하기 - 3. D435i USB SCP overflow ERROR 해결/'
  - '/2020-07-28-NVIDIA-AGX-Xavier에-ROS,-D435i-세팅하기-3.-D435i-USB-SCP-overflow-ERROR-해결/'
---

USB SCP 에러가 뜨는 원인은 [여기](https://community.intel.com/t5/Items-with-no-label/D435-Windows-10-USB-SCP-overflow-depends-on-resolution-no/td-p/572017) 참조



## Solution

***"USB SCP overflow" appears at varying increasing rate, if I use 1280x720 for all sensors (rgb and stereo) at 30 fps. I reduce RGB camera resolution to 960 x 540 and I do not have overflow problem at all and I'm fine with it. It looks limited to USB3 bandwidth, not the CPU or memory.***

RGB-D sensor의 데이터 양이 많다보니 bandwidth 문제가 발생하는 것이었음. Resolution을 640x480으로 낮추니 해결되었음
