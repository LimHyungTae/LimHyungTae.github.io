---
layout: post
title: NVIDIA AGX Xavier에 ROS, D435i 세팅하기 - 2. D435i Fine tuning
subtitle: NVIDIA AGX Xavier with Interl Realsense D435i
tags: [NVIDIA, Jetson, AGX, Xavier, ROS, D435i]
comments: true

---

realsense_viewer를 통해 보면 d435i의 데이터를 좀 

# 1. Jetson Xavier 세팅

Clock을 max로 사용하게 지정해주어야 함.

그리고 fan을 돌리기 위해 jetson_clocks shell 파일 실행하면 xavier의 fan이 도는 것 확인할 수 있음

```
$ sudo nvpmodel -m 0
$ sudo /usr/bin/jetson_clocks
```

# 2. Set Post-processing

# 3. Set High Accuracy mode


D435i는 중요한 세팅을 두 가지 해야 함. **post-processing** 모드와 **high accuracy mode**를 켜줘야 함!

rs_camera.launch 카메라 내부의 filters option에 다음과 같이 넣으면 된다.

```
 <arg name="filters" default="pointcloud,disparity,spatial,temporal,decimation"/>
```

그리고 

```
$ run rqt_reconfigure rqt_reconfigure
```
을 실행시켜서 GUI로 option들 조정이 가능하다.

혹은 아래의 service를 날려주면 high accuracy mode로 세팅됨!
```
rosservice call --wait /camera/stereo_module/set_parameters "config:
  ints: 
  - {name: 'visual_preset', value: 3}" 
```

![before](/img/d435i_before.gif){: .center-block :}

![after](/img/d435i_after.gif){: .center-block :}

![before2](/img/d435i_side_before.gif){: .center-block :}

![after2](/img/d435i_side_after.gif){: .center-block :}

