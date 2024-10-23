---
layout: post
title: LunarVim LSP를 위한 compile_commands.json 파일 설정 방법
subtitle: How to install LunarVim
tags: [Ubuntu, vim]
comments: true
---

## Introduction

깡 LunarVim을 쓰다가, 어느 순간 '왜 file navigation을 안 쓰고 있지?'라는 생각이 문득 들어서, 파일 간의 네비게이션 단축키를 세팅하려고 한다.
그러기 위해서는 LSP에 대한 이해가 필요하다.

## What is LSP?

먼저 LSP는 Language Server Protocol의 약자로, IDE에서 파일 간의 관계를 바탕으로 변수 자동 완성이나 탐색 기능을 지원하는 역할을 한다. 
쉽게 말해, VSCode, CLion, Pycharm 같은 IDE에서 제공하는 다양한 기능의 기반이라고 생각할 수 있다. 
아래는 LSP의 주요 기능들입니다:

* 코드 자동 완성: LSP를 통해 사용 중인 언어의 자동 완성 기능을 제공
* 구문 오류 및 경고 표시: 문법 오류나 경고를 실시간으로 보여주며, 코드에 버그가 있는지 사용자에게 feedback을 해 줌
* 정의로 이동: 변수나 함수의 정의로 빠르게 이동
* 리팩토링 지원: 변수 이름 변경과 같은 리팩토링 작업 용이
* 코드 형식 자동 정리: 포맷팅 규칙에 따라 코드를 일관성 있게 정리해 주는 기능을 제공

내가 사용하고자 한 기능은 **정의로 이동**이다. 이 때까지는 파일명을 검색해서 일일이 찾아다녔기 때문이다...

## How to Set

C++의 경우 `clangd`가, Python의 경우에는 `pyright`가 깔려있어야 하는데, 이는 LunarVim에서 `:LspInfo` 명령어를 치면 확인할 수 있다:

![lspinfo](/img/lspinfo.png)

이게 깔렸다면, 이제 package 상에 `compile_commands.json`만 배치하면 된다. `CMakeLists.txt`를 통해 빌드를 한다면 이는 쉽게 생성할 수 있다.
아래는 ROS 상에서 catkin build로 build할 때 compile command를 내보내는 방법이다:

```angular2html
catkin build -DCMAKE_EXPORT_COMPILE_COMMANDS=1 
```

C++의 LSP인 clangd는 프로젝트의 빌드 환경에 맞는 컴파일 명령을 알아야 제대로 작동하는데, 이 정보는 보통 `compile_commands.json` 파일에 저장된다.
예시로, `compile_commands.json` 파일은 아래와 같이 구성되어 있다 (현재 나는 catkin workspace를 여러 개 사용하는데, 아래는 `dcist_ws`라는 catkin workspace의 `build/patchwork`의 `compile_commands.json` 파일임을 참고):

```angular2html
[
{
  "directory": "/home/shapelim/dcist_ws/build/patchwork",
  "command": "/usr/bin/c++ -DDISABLE_LIBUSB_1_0 -DDISABLE_PCAP -DDISABLE_PNG -DROSCONSOLE_BACKEND_LOG4CXX -DROS_BUILD_SHARED_LIBS=1 -DROS_PACKAGE_NAME=\\\"patchwork\\\" -Dqh_QHpointer -DvtkRenderingContext2D_AUTOINIT=\"1(vtkRenderingContextOpenGL2)\" -DvtkRenderingCore_AUTOINIT=\"3(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingOpenGL2)\" -I/home/shapelim/dcist_ws/devel/.private/patchwork/include -I/opt/ros/noetic/include -I/opt/ros/noetic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp -I/home/shapelim/dcist_ws/src/patchwork/include -I/home/shapelim/dcist_ws/build/patchwork/tbb/include -isystem /usr/include/vtk-7.1 -isystem /usr/include/freetype2 -isystem /usr/include/eigen3 -isystem /usr/include/pcl-1.10 -isystem /usr/include/ni -isystem /usr/include/openni2 -O3 -DNDEBUG -std=c++14 -std=c++17 -o CMakeFiles/ros_kitti.dir/nodes/ros_kitti.cpp.o -c /home/shapelim/dcist_ws/src/patchwork/nodes/ros_kitti.cpp",
  "file": "/home/shapelim/dcist_ws/src/patchwork/nodes/ros_kitti.cpp",
  "output": "CMakeFiles/ros_kitti.dir/nodes/ros_kitti.cpp.o"
},
{
  "directory": "/home/shapelim/dcist_ws/build/patchwork",
  "command": "/usr/bin/c++ -DDISABLE_LIBUSB_1_0 -DDISABLE_PCAP -DDISABLE_PNG -DROSCONSOLE_BACKEND_LOG4CXX -DROS_BUILD_SHARED_LIBS=1 -DROS_PACKAGE_NAME=\\\"patchwork\\\" -Dqh_QHpointer -DvtkRenderingContext2D_AUTOINIT=\"1(vtkRenderingContextOpenGL2)\" -DvtkRenderingCore_AUTOINIT=\"3(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingOpenGL2)\" -I/home/shapelim/dcist_ws/devel/.private/patchwork/include -I/opt/ros/noetic/include -I/opt/ros/noetic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp -I/home/shapelim/dcist_ws/src/patchwork/include -I/home/shapelim/dcist_ws/build/patchwork/tbb/include -isystem /usr/include/vtk-7.1 -isystem /usr/include/freetype2 -isystem /usr/include/eigen3 -isystem /usr/include/pcl-1.10 -isystem /usr/include/ni -isystem /usr/include/openni2 -O3 -DNDEBUG -std=c++14 -std=c++17 -o CMakeFiles/ros_kitti_publisher.dir/nodes/ros_kitti_publisher.cpp.o -c /home/shapelim/dcist_ws/src/patchwork/nodes/ros_kitti_publisher.cpp",
  "file": "/home/shapelim/dcist_ws/src/patchwork/nodes/ros_kitti_publisher.cpp",
  "output": "CMakeFiles/ros_kitti_publisher.dir/nodes/ros_kitti_publisher.cpp.o"
},
{
  "directory": "/home/shapelim/dcist_ws/build/patchwork",
  "command": "/usr/bin/c++ -DDISABLE_LIBUSB_1_0 -DDISABLE_PCAP -DDISABLE_PNG -DROSCONSOLE_BACKEND_LOG4CXX -DROS_BUILD_SHARED_LIBS=1 -DROS_PACKAGE_NAME=\\\"patchwork\\\" -Dqh_QHpointer -DvtkRenderingContext2D_AUTOINIT=\"1(vtkRenderingContextOpenGL2)\" -DvtkRenderingCore_AUTOINIT=\"3(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingOpenGL2)\" -I/home/shapelim/dcist_ws/devel/.private/patchwork/include -I/opt/ros/noetic/include -I/opt/ros/noetic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp -I/home/shapelim/dcist_ws/src/patchwork/include -I/home/shapelim/dcist_ws/build/patchwork/tbb/include -isystem /usr/include/vtk-7.1 -isystem /usr/include/freetype2 -isystem /usr/include/eigen3 -isystem /usr/include/pcl-1.10 -isystem /usr/include/ni -isystem /usr/include/openni2 -O3 -DNDEBUG -std=c++14 -std=c++17 -o CMakeFiles/pub_for_legoloam.dir/nodes/pub_for_legoloam.cpp.o -c /home/shapelim/dcist_ws/src/patchwork/nodes/pub_for_legoloam.cpp",
  "file": "/home/shapelim/dcist_ws/src/patchwork/nodes/pub_for_legoloam.cpp",
  "output": "CMakeFiles/pub_for_legoloam.dir/nodes/pub_for_legoloam.cpp.o"
},
{
  "directory": "/home/shapelim/dcist_ws/build/patchwork",
  "command": "/usr/bin/c++ -DDISABLE_LIBUSB_1_0 -DDISABLE_PCAP -DDISABLE_PNG -DROSCONSOLE_BACKEND_LOG4CXX -DROS_BUILD_SHARED_LIBS=1 -DROS_PACKAGE_NAME=\\\"patchwork\\\" -Dqh_QHpointer -DvtkRenderingContext2D_AUTOINIT=\"1(vtkRenderingContextOpenGL2)\" -DvtkRenderingCore_AUTOINIT=\"3(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingOpenGL2)\" -I/home/shapelim/dcist_ws/devel/.private/patchwork/include -I/opt/ros/noetic/include -I/opt/ros/noetic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp -I/home/shapelim/dcist_ws/src/patchwork/include -I/home/shapelim/dcist_ws/build/patchwork/tbb/include -isystem /usr/include/vtk-7.1 -isystem /usr/include/freetype2 -isystem /usr/include/eigen3 -isystem /usr/include/pcl-1.10 -isystem /usr/include/ni -isystem /usr/include/openni2 -O3 -DNDEBUG -std=c++14 -std=c++17 -o CMakeFiles/offline_kitti.dir/nodes/offline_kitti.cpp.o -c /home/shapelim/dcist_ws/src/patchwork/nodes/offline_kitti.cpp",
  "file": "/home/shapelim/dcist_ws/src/patchwork/nodes/offline_kitti.cpp",
  "output": "CMakeFiles/offline_kitti.dir/nodes/offline_kitti.cpp.o"
},
{
  "directory": "/home/shapelim/dcist_ws/build/patchwork",
  "command": "/usr/bin/c++ -DDISABLE_LIBUSB_1_0 -DDISABLE_PCAP -DDISABLE_PNG -DROSCONSOLE_BACKEND_LOG4CXX -DROS_BUILD_SHARED_LIBS=1 -DROS_PACKAGE_NAME=\\\"patchwork\\\" -Dqh_QHpointer -DvtkRenderingContext2D_AUTOINIT=\"1(vtkRenderingContextOpenGL2)\" -DvtkRenderingCore_AUTOINIT=\"3(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingOpenGL2)\" -I/home/shapelim/dcist_ws/devel/.private/patchwork/include -I/opt/ros/noetic/include -I/opt/ros/noetic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp -I/home/shapelim/dcist_ws/src/patchwork/include -I/home/shapelim/dcist_ws/build/patchwork/tbb/include -isystem /usr/include/vtk-7.1 -isystem /usr/include/freetype2 -isystem /usr/include/eigen3 -isystem /usr/include/pcl-1.10 -isystem /usr/include/ni -isystem /usr/include/openni2 -O3 -DNDEBUG -std=c++14 -std=c++17 -o CMakeFiles/offline_own_data.dir/nodes/offline_own_data.cpp.o -c /home/shapelim/dcist_ws/src/patchwork/nodes/offline_own_data.cpp",
  "file": "/home/shapelim/dcist_ws/src/patchwork/nodes/offline_own_data.cpp",
  "output": "CMakeFiles/offline_own_data.dir/nodes/offline_own_data.cpp.o"
},
{
  "directory": "/home/shapelim/dcist_ws/build/patchwork/gtest/googlemock",
  "command": "/usr/bin/c++ -DGTEST_CREATE_SHARED_LIBRARY=1 -Dgmock_EXPORTS -I/usr/src/googletest/googlemock/include -I/usr/src/googletest/googlemock -isystem /usr/src/googletest/googletest/include -isystem /usr/src/googletest/googletest -O3 -DNDEBUG -std=c++11 -fPIC -Wall -Wshadow -Wno-error=dangling-else -DGTEST_HAS_PTHREAD=1 -fexceptions -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -std=c++17 -DGTEST_HAS_PTHREAD=1 -o CMakeFiles/gmock.dir/src/gmock-all.cc.o -c /usr/src/googletest/googlemock/src/gmock-all.cc",
  "file": "/usr/src/googletest/googlemock/src/gmock-all.cc",
  "output": "gtest/googlemock/CMakeFiles/gmock.dir/src/gmock-all.cc.o"
},
{
  "directory": "/home/shapelim/dcist_ws/build/patchwork/gtest/googlemock",
  "command": "/usr/bin/c++ -DGTEST_CREATE_SHARED_LIBRARY=1 -Dgmock_main_EXPORTS -isystem /usr/src/googletest/googlemock/include -isystem /usr/src/googletest/googlemock -isystem /usr/src/googletest/googletest/include -isystem /usr/src/googletest/googletest -O3 -DNDEBUG -std=c++11 -fPIC -Wall -Wshadow -Wno-error=dangling-else -DGTEST_HAS_PTHREAD=1 -fexceptions -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -std=c++17 -DGTEST_HAS_PTHREAD=1 -o CMakeFiles/gmock_main.dir/src/gmock_main.cc.o -c /usr/src/googletest/googlemock/src/gmock_main.cc",
  "file": "/usr/src/googletest/googlemock/src/gmock_main.cc",
  "output": "gtest/googlemock/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o"
},
{
  "directory": "/home/shapelim/dcist_ws/build/patchwork/gtest/googletest",
  "command": "/usr/bin/c++ -DGTEST_CREATE_SHARED_LIBRARY=1 -Dgtest_EXPORTS -I/usr/src/googletest/googletest/include -I/usr/src/googletest/googletest -O3 -DNDEBUG -std=c++11 -fPIC -Wall -Wshadow -Wno-error=dangling-else -DGTEST_HAS_PTHREAD=1 -fexceptions -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -std=c++17 -o CMakeFiles/gtest.dir/src/gtest-all.cc.o -c /usr/src/googletest/googletest/src/gtest-all.cc",
  "file": "/usr/src/googletest/googletest/src/gtest-all.cc",
  "output": "gtest/googletest/CMakeFiles/gtest.dir/src/gtest-all.cc.o"
},
{
  "directory": "/home/shapelim/dcist_ws/build/patchwork/gtest/googletest",
  "command": "/usr/bin/c++ -DGTEST_CREATE_SHARED_LIBRARY=1 -Dgtest_main_EXPORTS -isystem /usr/src/googletest/googletest/include -isystem /usr/src/googletest/googletest -O3 -DNDEBUG -std=c++11 -fPIC -Wall -Wshadow -Wno-error=dangling-else -DGTEST_HAS_PTHREAD=1 -fexceptions -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -std=c++17 -DGTEST_HAS_PTHREAD=1 -o CMakeFiles/gtest_main.dir/src/gtest_main.cc.o -c /usr/src/googletest/googletest/src/gtest_main.cc",
  "file": "/usr/src/googletest/googletest/src/gtest_main.cc",
  "output": "gtest/googletest/CMakeFiles/gtest_main.dir/src/gtest_main.cc.o"
}
]
```

clangd는 이 파일을 찾지 못할 경우 기본적으로 "fallback" 모드로 동작하는데, 이는 일부 기능이 제한되거나 제대로 작동하지 않을 수 있음을 의미한다.

무튼, 위의 옵션으로 빌드하면 `build/${PACKAGE_NAME}`에 아래와 같이 `compile_commands.json`가 각 package 별로 생성된다.

![compile](/img/compile_commands.png)

이를 복사하거나 링크를 해서 `src/${PACKAGE_NAME}/compile_commands.json`에 배치하면 된다.
LSP가 잘 동작했다면 최종적으로 `.cache/clangd`라는 폴더가 생기고, 인덱싱 정보가 해당 폴더 내에 생성된다.

## **중요** ChatGPT가 잘 못 알려준 것들

ChatGPT가 아래와 같이 LunarVim의 config 파일에 세팅하면 된다고 알려 주었는데:

```angular2html
require'lspconfig'.clangd.setup{
    cmd = { "clangd", "--compile-commands-dir=/home/shapelim/dcist_ws/build/compile_commands" },
}
```

위의 라인은 `compile_commands`를 해당 주소에 배치해둬도 동작하지 않았다.

## 결론

귀찮기는 하지만, 각 package 별 `compile_commands.json` 파일을 `package.xml`(ROS의 경우)가 있는 위치에 두고 `lvim .`을 실행하니 네비게이션 기능도 잘 동작했다.
이제는 진짜로 JetBrain-free한 개발을 하게 될지도...?

---

여담 1. LaTex 같은 경우에도 `:MasonInstall texlab`을 설치하면 tex 파일을 위한 LSP가 지원이 가능한 것으로 보인다. 

여담 2. 계속 해서 `cpp` 실행 파일이 늘어날 수 있는 상황이라면 `ln -s` 옵션을 통해 파일을 링크해도 잘 동작함을 확인했다.  