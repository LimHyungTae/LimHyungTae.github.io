---
layout: post
title: Kimera-Multi 빌드 시 glog 관련 에러 해결 방법
subtitle: 
tags: [Ubuntu, filesystem, library]
comments: true
description: Kimera-Multi 빌드 중 voxblox, dpgo_ros, kimera_distributed에서 발생하는 glog 관련 에러와 CHECK_NOTNULL 컴파일 에러를 해결한 과정을 정리한다.
permalink: /2024/08/18/kimera-multi-glog-error-fix/
redirect_from:
  - '/2024-08-18-Kimera-Multi 빌드 시 glog 관련 에러 해결 방법/'
  - '/2024-08-18-Kimera-Multi-빌드-시-glog-관련-에러-해결-방법/'
---

이전 글에서 무사히 glog 이슈를 해결한 줄 알았더니, 이번에는 `voxblox`나 `dpgo_ros`에서 glog를 찾을 수 없다고 에러가 발생했다.

## Case 1. glog를 지우고 다시 설치했을 때 (결국 실패함...아래로 이동)

### 1. Glog error in ROS-free package 해결하기

먼저 ETH-ASL 친구들은 glog를 catkin화해서 사용하는데, 문제는 이 glog를 ros-free인 package에서는 찾지를 못한다는 것이다.
그런데 여기 [PR](https://github.com/ethz-asl/glog_catkin/pull/17)에서 해당 문제를 해결했다.
따라서 아래와 같은 류의 에러가 발생한다면:

```angular2html
/home/shapelim/kimera_multi_ws/src/voxblox/voxblox/include/voxblox/integrator/esdf_occ_integrator.h:9:10: fatal error: glog/logging.h: No such file or directory
    9 | #include <glog/logging.h>
```

현재 ROS workspace에 있는 `glog_catkin`을 제거한 후, 저 TimeSchneider42님의 레포지토리를 설치하면 된다.

```bash
git clone https://github.com/TimSchneider42/glog_catkin.git
```


---

### 2. Glog error in `dpgo_ros` 해결하기 

그 후, `dpgo_ros`를 빌드하려고 하니 다음과 같은 에러가 발생했다:

```angular2html
/usr/bin/ld: cannot find -lglog::glog
collect2: error: ld returned 1 exit status
make[2]: *** [CMakeFiles/dpgo_ros.dir/build.make:156: /home/shapelim/kimera_multi_ws/devel/.private/dpgo_ros/lib/libdpgo_ros.so] Error 1
make[1]: *** [CMakeFiles/Makefile2:2220: CMakeFiles/dpgo_ros.dir/all] Error 2
make: *** [Makefile:146: all] Error 2
```

이는 `dpgo_ros`에서 `glog::glog`를 찾지 못해서 발생하는 문제인데, 다른 레포지토리에서는 `catkin_simple`을 사용해서 현재 workspace 내의 `glog_catkin`를 link한 반면,
이 레포지토리에서는 또 local 컴퓨터에 설치하고자 하는 glog를 사용하고 있는 것 같다 (그리고 안타깝게도 local에 glog가 깔려있으면 현재 MIT-SPARK 시스템은 어느 glog를 가르켜야하는지에 대한 에러가 발생한다...진퇴양난임).

그런데 그냥 `CMakeLists.txt`에 아래와 같이 한 줄 추가해줬더니 해결됐다:

```
find_package(glog REQUIRED)
```

---

### 3. `kimera_distributed에서의 에러 해결하기`

```angular2html
In file included from /home/shapelim/kimera_multi_ws/src/kimera_distributed/src/SubmapAtlas.cpp:6:
/home/shapelim/kimera_multi_ws/src/kimera_distributed/src/SubmapAtlas.cpp: In member function ‘std::shared_ptr<kimera_distributed::Keyframe> kimera_distributed::SubmapAtlas::getLatestKeyframe()’:
/home/shapelim/kimera_multi_ws/devel/.private/glog_catkin/include/glog/logging.h:776:80: error: no matching function for call to ‘CheckNotNull(const char [74], int, const char [44], std::shared_ptr<kimera_distributed::Keyframe>)’
  776 |   google::CheckNotNull(__FILE__, __LINE__, "'" #val "' Must be non NULL", (val))
      |                                                                                ^
/home/shapelim/kimera_multi_ws/src/kimera_distributed/src/SubmapAtlas.cpp:88:12: note: in expansion of macro ‘CHECK_NOTNULL’
   88 |     return CHECK_NOTNULL(getKeyframe(keyframe_id));
      |            ^~~~~~~~~~~~~
In file included from /home/shapelim/kimera_multi_ws/src/kimera_distributed/src/SubmapAtlas.cpp:6:
/home/shapelim/kimera_multi_ws/devel/.private/glog_catkin/include/glog/logging.h:1286:4: note: candidate: ‘template<class T> T* google::CheckNotNull(const char*, int, const char*, T*)’
 1286 | T* CheckNotNull(const char *file, int line, const char *names, T* t) {
      |    ^~~~~~~~~~~~
/home/shapelim/kimera_multi_ws/devel/.private/glog_catkin/include/glog/logging.h:1286:4: note:   template argument deduction/substitution failed:
In file included from /home/shapelim/kimera_multi_ws/src/kimera_distributed/src/SubmapAtlas.cpp:6:
/home/shapelim/kimera_multi_ws/devel/.private/glog_catkin/include/glog/logging.h:776:80: note:   mismatched types ‘T*’ and ‘std::shared_ptr<kimera_distributed::Keyframe>’
  776 |   google::CheckNotNull(__FILE__, __LINE__, "'" #val "' Must be non NULL", (val))
      |                                                                                ^
/home/shapelim/kimera_multi_ws/src/kimera_distributed/src/SubmapAtlas.cpp:88:12: note: in expansion of macro ‘CHECK_NOTNULL’
   88 |     return CHECK_NOTNULL(getKeyframe(keyframe_id));
      |            ^~~~~~~~~~~~~
/home/shapelim/kimera_multi_ws/src/kimera_distributed/src/SubmapAtlas.cpp: In member function ‘std::shared_ptr<kimera_distributed::Submap> kimera_distributed::SubmapAtlas::createSubmap(int, const gtsam::Pose3&, const uint64_t&)’:
/home/shapelim/kimera_multi_ws/devel/.private/glog_catkin/include/glog/logging.h:776:80: error: no matching function for call to ‘CheckNotNull(const char [74], int, const char [44], std::shared_ptr<kimera_distributed::Submap>)’
  776 |   google::CheckNotNull(__FILE__, __LINE__, "'" #val "' Must be non NULL", (val))
      |                                                                                ^
/home/shapelim/kimera_multi_ws/src/kimera_distributed/src/SubmapAtlas.cpp:104:24: note: in expansion of macro ‘CHECK_NOTNULL’
  104 |     auto prev_submap = CHECK_NOTNULL(getSubmap(submap_id - 1));
      |                        ^~~~~~~~~~~~~
.
.
.
```

`CHECK_NOTNULL` 에러는 왜 내 컴퓨터에서만 뜨는지는 모르겠으나, pointer 뒤에 `get()`을 추가해서 받으면 해결된다.

--- 

## Case 1. glog를 지우고 다시 설치했을 때 (성공)

처음에는 glog가 여러 개 설치되어 있어서 발생한 문제라고 생각했는데, [이전 글](https://limhyungtae.github.io/2024-08-17-Kimera-Multi-%EB%B9%8C%EB%93%9C-%EC%8B%9C-something-wrong-with-flag-'logtostderr'-%ED%95%B4%EA%B2%B0-%EB%B0%A9%EB%B2%95/)과 같이 아래와 같이 `glog`를 완전히 지우고 

```angular2html
sudo apt-get install libgoogle-glog-dev
```

다시 설치하니 대체로 glog 이슈는 해결되었다.

그럼에도 내 컴퓨터에서는 `kimera_distributed`, `kimera_vio`, `voxblox_ros`가 빌드가 실패했다...
(참고로 Kimera 계열을 build할 때는 꼭!!! `catkin build -DGTSAM_USE_SYSTEM_EIGEN=ON`와 같이 GTSAM 관련 명령어를 명시해줘야 한다고 한다.)

주로 `CHECK_NOTNULL` 관련 에러가 발생하는데:

```angular2html
rrors     << kimera_vio:make /home/shapelim/kimera_multi_ws/logs/kimera_vio/build.make.001.log                                                               
In file included from /home/shapelim/kimera_multi_ws/src/kimera_vio/include/kimera-vio/backend/RegularVioBackendParams.h:27,
                 from /home/shapelim/kimera_multi_ws/src/kimera_vio/include/kimera-vio/pipeline/Pipeline-definitions.h:19,
                 from /home/shapelim/kimera_multi_ws/src/kimera_vio/include/kimera-vio/visualizer/OpenCvDisplay.h:19,
                 from /home/shapelim/kimera_multi_ws/src/kimera_vio/src/visualizer/OpenCvDisplay.cpp:15:
/home/shapelim/kimera_multi_ws/src/kimera_vio/src/visualizer/OpenCvDisplay.cpp: In constructor ‘VIO::OpenCv3dDisplay::OpenCv3dDisplay(VIO::DisplayParams::Ptr, const ShutdownPipelineCallback&)’:
/home/shapelim/kimera_multi_ws/src/kimera_vio/src/visualizer/OpenCvDisplay.cpp:35:16: error: no matching function for call to ‘CheckNotNull(const char [79], int, const char [84], std::shared_ptr<VIO::OpenCv3dDisplayParams>)’
   35 |       params_(*CHECK_NOTNULL(
      |                ^~~~~~~~~~~~~
In file included from /home/shapelim/kimera_multi_ws/src/kimera_vio/include/kimera-vio/backend/RegularVioBackendParams.h:27,
                 from /home/shapelim/kimera_multi_ws/src/kimera_vio/include/kimera-vio/pipeline/Pipeline-definitions.h:19,
                 from /home/shapelim/kimera_multi_ws/src/kimera_vio/include/kimera-vio/visualizer/OpenCvDisplay.h:19,
                 from /home/shapelim/kimera_multi_ws/src/kimera_vio/src/visualizer/OpenCvDisplay.cpp:15:
/home/shapelim/kimera_multi_ws/devel/include/glog/logging.h:1286:4: note: candidate: ‘template<class T> T* google::CheckNotNull(const char*, int, const char*, T*)’
 1286 | T* CheckNotNull(const char *file, int line, const char *names, T* t) {
      |    ^~~~~~~~~~~~
/home/shapelim/kimera_multi_ws/devel/include/glog/logging.h:1286:4: note:   template argument deduction/substitution failed:
In file included from /home/shapelim/kimera_multi_ws/src/kimera_vio/include/kimera-vio/backend/RegularVioBackendParams.h:27,
                 from /home/shapelim/kimera_multi_ws/src/kimera_vio/include/kimera-vio/pipeline/Pipeline-definitions.h:19,
                 from /home/shapelim/kimera_multi_ws/src/kimera_vio/include/kimera-vio/visualizer/OpenCvDisplay.h:19,
                 from /home/shapelim/kimera_multi_ws/src/kimera_vio/src/visualizer/OpenCvDisplay.cpp:15:
/home/shapelim/kimera_multi_ws/src/kimera_vio/src/visualizer/OpenCvDisplay.cpp:35:16: note:   mismatched types ‘T*’ and ‘std::shared_ptr<VIO::OpenCv3dDisplayParams>’
   35 |       params_(*CHECK_NOTNULL(
      |                ^~~~~~~~~~~~~
make[2]: *** [CMakeFiles/kimera_vio.dir/build.make:1266: CMakeFiles/kimera_vio.dir/src/visualizer/OpenCvDisplay.cpp.o] Error 1
make[1]: *** [CMakeFiles/Makefile2:799: CMakeFiles/kimera_vio.dir/all] Error 2
make: *** [Makefile:146: all] Error 2
```

코드 내에 있는 `CHECK_NOTNULL`의 입력으로 들어가는 pointer를 `get()`으로 받아주면 해결된다.
이를 통해 `voxblox_ros`와 `kimera_vio`는 해결했는데, `kimera_distributed`에서는 여전히 `CHECK_NOTNULL` 관련 에러가 아래와 같이 발생한다:

```
Errors     << kimera_distributed:make /home/shapelim/kimera_multi_ws/logs/kimera_distributed/build.make.006.log                                               
In file included from /home/shapelim/kimera_multi_ws/src/kimera_distributed/src/SubmapAtlas.cpp:6:
/home/shapelim/kimera_multi_ws/src/kimera_distributed/src/SubmapAtlas.cpp: In member function ‘std::shared_ptr<kimera_distributed::Keyframe> kimera_distributed::SubmapAtlas::getLatestKeyframe()’:
/home/shapelim/kimera_multi_ws/devel/.private/glog_catkin/include/glog/logging.h:776:23: error: could not convert ‘google::CheckNotNull<kimera_distributed::Keyframe>(((const char*)"/home/shapelim/kimera_multi_ws/src/kimera_distributed/src/SubmapAtlas.cpp"), 88, ((const char*)"\'getKeyframe(keyframe_id).get()\' Must be non NULL"), kimera_distributed::SubmapAtlas::getKeyframe(int)(keyframe_id).std::shared_ptr<kimera_distributed::Keyframe>::<anonymous>.std::__shared_ptr<kimera_distributed::Keyframe, __gnu_cxx::_S_atomic>::get())’ from ‘kimera_distributed::Keyframe*’ to ‘std::shared_ptr<kimera_distributed::Keyframe>’
  776 |   google::CheckNotNull(__FILE__, __LINE__, "'" #val "' Must be non NULL", (val))
      |   ~~~~~~~~~~~~~~~~~~~~^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      |                       |
      |                       kimera_distributed::Keyframe*
/home/shapelim/kimera_multi_ws/src/kimera_distributed/src/SubmapAtlas.cpp:88:12: note: in expansion of macro ‘CHECK_NOTNULL’
   88 |     return CHECK_NOTNULL(getKeyframe(keyframe_id).get());
      |            ^~~~~~~~~~~~~
/home/shapelim/kimera_multi_ws/src/kimera_distributed/src/SubmapAtlas.cpp: In member function ‘std::shared_ptr<kimera_distributed::Submap> kimera_distributed::SubmapAtlas::getLatestSubmap()’:
/home/shapelim/kimera_multi_ws/devel/.private/glog_catkin/include/glog/logging.h:776:23: error: could not convert ‘google::CheckNotNull<kimera_distributed::Submap>(((const char*)"/home/shapelim/kimera_multi_ws/src/kimera_distributed/src/SubmapAtlas.cpp"), 134, ((const char*)"\'getSubmap(submap_id).get()\' Must be non NULL"), kimera_distributed::SubmapAtlas::getSubmap(int)(submap_id).std::shared_ptr<kimera_distributed::Submap>::<anonymous>.std::__shared_ptr<kimera_distributed::Submap, __gnu_cxx::_S_atomic>::get())’ from ‘kimera_distributed::Submap*’ to ‘std::shared_ptr<kimera_distributed::Submap>’
  776 |   google::CheckNotNull(__FILE__, __LINE__, "'" #val "' Must be non NULL", (val))
      |   ~~~~~~~~~~~~~~~~~~~~^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      |                       |
      |                       kimera_distributed::Submap*

```

이는 `CHECK_NOTNULL`의 입력으로 들어가는 pointer는 원시 포인터를 받아야하기 때문에 `get()`으로 받아주되, 변수를 CHECK_NOTNULL 밖에 선언해주면 된다:

```angular2html
@@ -85,7 +85,9 @@ std::shared_ptr<Keyframe> SubmapAtlas::getLatestKeyframe() {
   else {
     int keyframe_id = numKeyframes() - 1;
     CHECK_GE(keyframe_id, 0);
-    return CHECK_NOTNULL(getKeyframe(keyframe_id));
+    auto keyframe_ptr = getKeyframe(keyframe_id);
+    CHECK_NOTNULL(keyframe_ptr.get());
+    return keyframe_ptr;
   }
 }
```

이렇게 하면 Kimera-Multi를 돌리는 세팅을 완료하게 된다!
