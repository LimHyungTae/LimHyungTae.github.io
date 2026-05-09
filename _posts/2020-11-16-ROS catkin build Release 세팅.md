---
layout: post
title: ROS catkin build Release 세팅
subtitle: ROS catkin build Release
tags: [ROS, catkin, catkin-tools]
comments: true
description: ROS 패키지가 debug 모드로 빌드되어 속도가 떨어질 때, catkin build에 --save-config와 -DCMAKE_BUILD_TYPE=Release를 한 번 지정해 항상 Release로 컴파일되도록 설정하는 방법을 정리한다.
image: /img/catkinbuild_release.png
permalink: /2020/11/16/ros-catkin-build-release-setup/
redirect_from:
  - '/2020-11-16-ROS catkin build Release 세팅/'
  - '/2020-11-16-ROS-catkin-build-Release-세팅/'
---

# Release로 설정해야 하는 이유

Ouster OS0을 쓰던 도중, 패킷을 받는 속도가 너무 느려서 확인해보니 compile이 debug 모드로 되어 있음을 확인했다.

debug 모드로 컴파일 했을 때와 Release로 ROS를 컴파일했을 때의 속도가 많이 차이가 난다고 한다.

(Window에서 Visual Studio개발할 때 Release로 내보내는 것과 비슷한 느낌이지 않을까 싶다.)

# catkin build할 때 명령어
```
$ catkin build --save-config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

![cb](/img/catkinbuild_release.png)

# 결과

위의 명령어는 한 번 세팅해주면 BUILD_TYPE을 계속 Release로 유지해준다.

**Additional CMake Args**에 `-DCMAKE_BUILD_TYPE=Release`가 추가 된것을 볼 수 있다.

![cb_raw](/img/catkinbuild_release_raw.png)

### Update on 2024-02-01

만약 config를 지우고 싶다면 아래와 같은 명령어를 기입하면 저장된 config를 제거할 수 있다.

```
$ catkin config -r -DCMAKE_BUILD_TYPE=Release
```
