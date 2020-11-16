---
layout: post
title: ROS catkin build Release 세팅
subtitle: ROS catkin build Release
tags: [ROS, catkin, catkin-tools]
comments: true
---

# Release로 설정해야 하는 이유

Ouster OS0을 쓰던 도중, 패킷을 받는 속도가 너무 느려서 확인해보니 compile이 debug 모드로 되어 있음을 확인했다.

debug 모드로 컴파일 했을 때와 Release로 ROS를 컴파일했을 때의 속도가 많이 차이가 난다고 한다.

(Window에서 Visual Studio개발할 때 Release로 내보내는 것과 비슷한 느낌이지 않을까 싶다.)

```
$ catkin build --save-config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

![cb](/img/catkinbuild_release.png)


![cb_raw](/img/catkinbuild_release_raw.png)
