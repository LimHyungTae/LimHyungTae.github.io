---
layout: post
title: NVIDIA AGX Xavier에 ROS, D435i 세팅하기 - 2. D435i Fine tuning
subtitle: NVIDIA AGX Xavier with Interl Realsense D435i
tags: [NVIDIA, Jetson, AGX, Xavier, ROS, D435i]
comments: true

---

본 페이지는 모바일 플랫폼에서 D435i를 사용하기 위해 세팅한 결과를 정리하기 위한 포스트입니다.

realsense_viewer에서는 원클릭으로 설정가능한 **post-processing**이나 **High accuracy mode**를 roslaunch에서는 어떻게 사용하는지에 대한 방법을 설명합니다.

D435i보다는 Microsoft Azure가 더 정확하다고 의견이 모아지고 있는데, 가격이 D435i에 비해 쪼오금 더 비싸다보니 아직 많은 분들이 D435i를 선호하는 것 같습니다. :)

# 1. Jetson Xavier 세팅

가장 먼저, depth camera의 pointcloud를 CPU가 잘 처리할 수 있게 컴퓨터의 성능을 최대한으로 세팅해주어야 합니다.

Jetson Xavier는 여러 모드가 있는데, max clock으로 사용하기 위해선 아래와 같은 명령어를 입력합니다.

```
$ sudo nvpmodel -m 0
```

그리고 왜인지는 모르겠으나 Xavier를 껐다키면 fan이 간혹 돌지 않는데, /usr/bin/ 폴더 아래에 있는 jetson_clocks 파일 실행하면 xavier의 Fan이 동작하는 것 확인할 수 있습니다.

```
$ sudo /usr/bin/jetson_clocks
```

사족으로, NX나 TX2보드 같은 경우에는 온도가 어느 이상으로 올라가면 CPU 성능이 급격히 저하되는 현상을 겪으실 수 있는데, AGX는 그런 점에서 강인하다고 합니다. 하지만 만일의 상황에 대비하기 위해 저는 세팅을 보수적으로 하는 것을 추천드립니다.



# 2. Set Post-processing

Realsense2에서는 많은 post-processing option이 존재하는데, ROS에서도 똑같이 사용하기 위해서는 `rs_camera.launch`의  filters option에 다음과 같이 넣으면 됩니다.

```
 <arg name="filters" default="pointcloud,disparity,spatial,temporal,decimation"/>
```

참고 메뉴얼은 [여기](https://dev.intelrealsense.com/docs/tuning-depth-cameras-for-best-performance)와 [여기](https://dev.intelrealsense.com/docs/depth-post-processing)입니다.

혹시 UAV에 D435i를 세팅하신다면, temporal filter의 gain들을 좀 조정해주어야 할 것으로 보입니다. 

이름 그대로 temporal한 정보를 기반으로 filtering을 하는 것으로 보이는데, UAV처럼 빠르게 움직이는 상황에서도 필터링이 잘 될지 잘 모르겠습니다.(저는 모바일 로봇 플랫폼에서 사용했습니다)

메뉴얼에도 temporal filter가 D435i가 어느정도 static한 상황에서도 좋다고 명시되어 있습니다.


# 3. Set High Accuracy mode

`rs_camera.launch`를 실행한 후에 아래와 같이 rosservice를 날려주면 D435i의 모드가 high accuracy mode로 설정됩니다.

```
rosservice call --wait /camera/stereo_module/set_parameters "config:
  ints: 
  - {name: 'visual_preset', value: 3}" 
```

High accuracy mode 외에도 여러 모드가 있기 때문에, [여기](https://github.com/IntelRealSense/librealsense/wiki/D400-Series-Visual-Presets)를 참고하셔서 자신에게 가장 잘 맞는 mode를 택하시면 될 것 같습니다.

값이 잘 들어갔는지 해보고 싶으시면, 아래의 명령어를 입력하시면 GUI 단에서 각 filter와 mode 파라미터를 변경하실 수 있습니다.

```
$ run rqt_reconfigure rqt_reconfigure
```

# 결과 

#### Before1

![before](/img/d435i_before.gif){: .center-block :}

#### After1

![after](/img/d435i_after.gif){: .center-block :}

결과를 보면 확실히 덜 일렁거리게 되는 것을 확인하실 수 있습니다. 

#### Before2

![before2](/img/d435i_side_before.gif){: .center-block :}

#### After2

![after2](/img/d435i_side_after.gif){: .center-block :}

그리고 temporal filter의 gain들을 조정하면 *After2*처럼 좀 느려지는 느낌(?)이 들게 되는데, pointcloud들은 확실히 smooth해짐을 확인할 수 있습니다.
