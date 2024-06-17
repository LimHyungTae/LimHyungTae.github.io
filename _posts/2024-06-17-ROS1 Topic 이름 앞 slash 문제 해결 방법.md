---
layout: post
title: ROS1 Topic 이름 앞 slash 문제 해결 방법
subtitle: How to Solve ROS1 Topic Name Forward Slash Problem
tags: [ROS, C++, RViz]
comments: true
---

ROS1에서 rviz를 사용하던 도중, 언제부터인가 '/robot/velodyne_link'라고 기입하면 'robot/velodyne_link'로 인식하는 문제가 발생했다. 
이 문제는 Ubutun 18.04에서는 있지 않았는데, Ubuntu 20.04로 오는 과정에서 뭔가 에러가 나는 것 같다.

그래서 비효율적으로 처음에는 topic을 받아서 forward slash `/`을 토픽 앞에 추가적으로 붙여주는 node를 만들었는데,
이 문제는 [여기](https://github.com/laboshinl/loam_velodyne/issues/157)에서 볼 수 있듯이, 간단히 해결할 수 있다고 한다:

```
Seems that’s a problem with recent version of RViz. Just change the camera_init frame on rviz from /camera_init to //camera_init. That solved my problem.
```

즉 slash를 두 개 놓으면 잘 동작한다(!). 이를 몰라서 비효율적으로 사용하고 있었는데, 메모를 위해 기록한다.