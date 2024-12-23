---
layout: post
title: Pybind11 Line by Line - 3. pyproject.toml와 Github Actions을 통한 Pypi에 코드 업로드하기
subtitle: Understanding How pip3 works
tags: [Pybind11, Pybinding, Python, C++]
comments: true
---

이제 Pybinding이 잘 된다는 가정 하에, 어떻게 하면 우리의 연구 코드를 `pip3 install`로 설치하게 할 수 있을까?
사실 이 과정은 어어어엄청 엔지니어링이어서, 1) 주변에 이 C++ 코드 pybinding 후, 2) Pypi에 올려보기를 **해본 이가 없다면, 도전하지 않는 것을 추천**한다.
왜냐하면 생각보다 직관적이지 않고, 경험적인 부분이 많기 때문이다. 그리고 C++이 어떻게 빌드되어서 어떻게 동작하는 것인지에 대한 이해도 약간 필요하기 때문에, 
도움을 청할 수 있는 이가 있다면 힘껏 도움을 청해보자. 참고로 나도 이 과정을 100% 이해하진 못 해서, 틀린 설명이 있을 수도 있다(하지만 잘 돌아가긴 함). 

추천하는 예제 코드는 아래와 같다:

* [Patchwork++](https://github.com/url-kaist/patchwork-plusplus)
* [ROBIN](https://github.com/MIT-SPARK/ROBIN)

둘의 PR들을 살펴보면 내가 어디를 고쳤는지 대강 알 수 있을 것이다.

## Step 0. Prerequisites

### `CMakeLists.txt`의 변수들과 친해지기

사실 이 repository를 package화하는 과정을 이해하기 전에, C++ 코드를 로컬 컴퓨터에 설치해보는 경험을 해보는 게 중요하다.
내가 C++ 코드를 package화 하는 공부를 할 때 작성한 [cmake_make_install_study](https://github.com/LimHyungTae/cmake_make_install_study/tree/master/MyProject)의 README.md를 한 번 읽어보는 것을 추천한다.

CMakeLists.txt에서 주로 쓰이는 변수는 아래와 같은데:

```angular2html
CMAKE_CURRENT_LIST_DIR in CMakeLists.txt /home/shapelim/git/cmake_auto_include_study
CMAKE_SOURCE_DIR: /home/shapelim/git/cmake_auto_include_study
CMAKE_BINARY_DIR: /home/shapelim/git/cmake_auto_include_study/build
CMAKE_CURRENT_SOURCE_DIR: /home/shapelim/git/cmake_auto_include_study
CMAKE_CURRENT_BINARY_DIR: /home/shapelim/git/cmake_auto_include_study/build
CMAKE_CURRENT_LIST_DIR: /home/shapelim/git/cmake_auto_include_study
CMAKE_CURRENT_LIST_FILE: /home/shapelim/git/cmake_auto_include_study/CMakeLists.txt
CMAKE_INSTALL_PREFIX: /usr/local
CMAKE_INSTALL_LIBDIR: lib
CMAKE_INSTALL_INCLUDEDIR: include
PROJECT_SOURCE_DIR: /home/shapelim/git/cmake_auto_include_study
PROJECT_BINARY_DIR: /home/shapelim/git/cmake_auto_include_study/build
```

주로 우리가 쓰는 것은:

* `${CMAKE_CURRENT_SOURCE_DIR}`: 현재 CMakeLists.txt의 위치를 나타냄
* `${CMAKE_CURRENT_BINARY_DIR}`: 우리가 build하고자 하는 주소. 주로 `mkdir build && cd build && cmake .. && make -j 16` 해서 빌드를 할 때, `build` 디렉토리가 이 변수에 해당함

### Dependency

그리고 기초적으로, 아래의 package들이 필요하다: 

```bash
sudo apt-get install gcc g++ build-essential libeigen3-dev cmake python3-pip python3-dev git ninja-build -y
```



## Step 1. `pyproject.toml` 파일 구성

먼저, 앞서 `setup.py`를 쓴 것과는 다르게, `pyproject.toml` 파일을 구성해야 한다. 다른 블로그들([블로그1](https://gbdai.tistory.com/59), [블로그2](https://teddylee777.github.io/python/pypi/))에서는 `setup.py`를 설정해야 한다고 되어 있다.
하지만 이 [블로그 글](https://miintto.github.io/docs/python-pypi-packages)을 보면 이제는 `pyproject.toml` 파일로 표현하는 것으로 표준화가 되었다고 한다.
그리고 우리의 목표는 Pypi에 최종적으로 올리는 것인데, 이 과정에서 Github Action을 통해서 올려야 Window/Linux/Mac을 지원할 수 있게 wheel 파일이 생성된다.
실제로 나의 동료인 Nacho의 [KISS-ICP](https://github.com/PRBonn/kiss-icp/blob/main/python/pyproject.toml)에서도 `pyproject.toml`를 사용하는 것을 볼 수 있다.

현재 KISS-ICP 상의 `pyproject.toml`을 따라서 아래와 같이 Patchwork++ 레포지토리에서도 `pyproject.toml` 파일을 구성해 보았다:

```commandline
[build-system]
requires = ["scikit_build_core", "pybind11"]
build-backend = "scikit_build_core.build"

[project]
name = "pypatchworkpp"
version = "1.0.4"
requires-python = ">=3.8"
description = "ground segmentation"
dependencies = [
	"numpy>=1.23"
]

[project.optional-dependencies]
demo = [
	"open3d-cpu>=0.17"
]

[tool.scikit-build]
cmake.args = ["-DINCLUDE_PYTHON_WRAPPER=true"]
```

## Step 2. `CMakeLists.txt` 구성

현재 내가 쓰는 template은 아래와 같다 ([Patchwork++](https://github.com/url-kaist/patchwork-plusplus/blob/master/python/CMakeLists.txt)와 [ROBIN](https://github.com/MIT-SPARK/ROBIN) 코드를 비교해 가며 보자):

```commandline
cmake_minimum_required(VERSION 3.16...3.26)
project(patchworkpp_python_wrapper LANGUAGES CXX)

# Set build type
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Parameters used in `patchworkpp` subdirectory.
# Thus, link should be `patchworkpp::ground_seg_cores`
# See https://github.com/url-kaist/patchwork-plusplus/tree/master/cpp/CMakeLists.txt#L21
set(PARENT_PROJECT_NAME patchworkpp)
set(TARGET_NAME ground_seg_cores)

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# See our `pyproject.toml` file. We use `scikit_build_core`, which turns on `SKBUILD`
if (DEFINED SKBUILD)
    message(STATUS "Building with Scikit-Build")
endif ()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp ${CMAKE_CURRENT_BINARY_DIR}/patchworkpp_cpp)
else()
  cmake_minimum_required(VERSION 3.18)
  message(STATUS "Performing out-of-tree build, fetching Patchwork++ v${CMAKE_PROJECT_VERSION} Release from Github")
  include(FetchContent)
  FetchContent_Declare(
    ext_ground_seg_cores PREFIX ${PARENT_PROJECT_NAME}
    URL https://github.com/url-kaist/patchwork-plusplus/archive/refs/tags/v${CMAKE_PROJECT_VERSION}.tar.gz SOURCE_SUBDIR
        cpp/patchworkpp)
  FetchContent_MakeAvailable(ext_ground_seg_cores)
endif()

pybind11_add_module(pypatchworkpp patchworkpp/pybinding.cpp)

target_link_libraries(pypatchworkpp PUBLIC ${PARENT_PROJECT_NAME}::${TARGET_NAME})

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  target_compile_options(pypatchworkpp PUBLIC -fsized-deallocation)
endif()

install(TARGETS pypatchworkpp DESTINATION .)
```

위와 같이 `pyproject.toml`이 존재하는 곳에 `CMakeLists.txt`도 동시에 있어야 한다.

scikit-build-core는 Python 빌드 시스템(Python build 라이브러리)과 C++ 빌드 시스템(CMake) 사이의 다리 역할을 한다.
대부분 블로그 글을 보면 `hatchling`을 통해 Pypi에 올리는 법을 적어두었는데, 이 경우에는 **Pure Python 코드만 가능**하다.
우리같은 로봇쟁이들의 C++ 코드를 Pybinding한 후, Pypi에 올리려면 묻지도 따지지도 말고 반드시 `scikit-build-core`을 사용해야 한다.

## Step 3. Build 

위와 같이 코드를 작성한 Patchwork++ repository에서 

`pip3 install python/`을 해보면 빌드가 되는 것을 확인할 수 있다!

### How does scikit-build-core work?

`pip3 install python/`를 실행하면 우리가 ``mkdir build && cd build && cmake .. && make -j 16`를 통해 빌드했던 일련의 과정을 자동으로 실행한 후, wheel 파일을 생성해준다.
한 번 이해하고 싶다면, `pip3 install python/ --verbose`로 쳐보자. 아래는 ROBIN 레포지토리에서 해당 명령어로 설치하면 아래와 같이 출력되는데, 차근차근 살펴보자 (코드의 `***` 부분이 각각의 step을 나타낸다):

```commandline
-- Build files have been written to: /tmp/tmpvcokejo2/build
  *** Building project with Ninja...
  [1/33] Building CXX object pmc-build/CMakeFiles/pmc.dir/pmc_utils.cpp.o
  [2/33] Building CXX object pmc-build/CMakeFiles/pmc.dir/pmc_cores.cpp.o
  [3/33] Building CXX object pmc-build/CMakeFiles/pmc.dir/pmc_clique_utils.cpp.o
  [4/33] Building CXX object pmc-build/CMakeFiles/pmc_main.dir/pmc_driver.cpp.o
  [5/33] Building CXX object pmc-build/CMakeFiles/pmc.dir/pmc_maxclique.cpp.o
  [6/33] Building CXX object pmc-build/CMakeFiles/pmc.dir/pmc_heu.cpp.o
  [7/33] Building CXX object robin_cpp/CMakeFiles/robin.dir/src/utils.cpp.o
  [8/33] Building CXX object pmc-build/CMakeFiles/pmc.dir/pmcx_maxclique_basic.cpp.o
  [9/33] Building CXX object robin_cpp/CMakeFiles/robin.dir/src/graph.cpp.o
  [10/33] Building CXX object robin_cpp/CMakeFiles/robin.dir/src/core.cpp.o
  [11/33] Building CXX object pmc-build/CMakeFiles/pmc.dir/pmcx_maxclique.cpp.o
  [12/33] Building CXX object robin_cpp/CMakeFiles/robin.dir/src/pkc.cpp.o
  [13/33] Building CXX object robin_cpp/CMakeFiles/robin.dir/src/graph_solvers.cpp.o
  [14/33] Building CXX object robin_cpp/CMakeFiles/robin.dir/src/graph_io.cpp.o
  [15/33] Building CXX object robin_cpp/CMakeFiles/robin.dir/src/graph_core.cpp.o
  [16/33] Building CXX object robin_cpp/CMakeFiles/robin.dir/src/problems.cpp.o
  [17/33] Building CXX object pmc-build/CMakeFiles/pmc.dir/pmc_graph.cpp.o
  [18/33] Linking CXX shared library pmc-build/libpmc.so
  [19/33] Linking CXX executable pmc-build/pmc_main
  [20/33] Building CXX object robin_cpp/tests/CMakeFiles/robin_unit_tests.dir/problems_test.cpp.o
  [21/33] Building CXX object robin_cpp/tests/CMakeFiles/robin_unit_tests.dir/utils_test.cpp.o
  [22/33] Building CXX object robin_cpp/tests/CMakeFiles/robin_unit_tests.dir/math_test.cpp.o
  [23/33] Building CXX object robin_cpp/CMakeFiles/robin.dir/src/robin.cpp.o
...
```

먼저 우리가 `mkdir build && cd build`하던 과정을 이 scikit-build-core가 자동으로 시행해준다. 
그래서 위에 보면 `/tmp/tmpvcokejo2/build`라는 주소가 임시적으로 생성한 `build` directory의 주소인 것을 알 수 있다.

그리고 ROBIN에서는

`add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../ ${CMAKE_CURRENT_BINARY_DIR}/robin_cpp)`

라고 subdirectory를 선언했는데, 이 subdirectory 명령어를 사용하면

```angular2html
|-- build
|   |-- robin_cpp
...
```

와 같이 우리가 임의로 생성한 build 폴더의 하위 폴더에 raw 코드를 두고 out-of-the-box로 build해주겠다는 것을 의미한다.


```commandline
*** Installing project into wheel...
  -- Install configuration: "Release"
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/lib/cmake/xenium/xeniumTargets.cmake
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/parameter.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/kirsch_kfifo_queue.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/policy.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/utils.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/harris_michael_hash_map.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/michael_scott_queue.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/aligned_object.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/left_right.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/hash.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/impl
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/impl/vyukov_hash_map_traits.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/impl/vyukov_hash_map.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/seqlock.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/ramalhete_queue.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/backoff.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/detail
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/detail/pointer_queue_traits.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/detail/fixed_size_circular_array.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/detail/growing_circular_array.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/detail/hardware.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/detail/port.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/marked_ptr.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/kirsch_bounded_kfifo_queue.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/chase_work_stealing_deque.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/harris_michael_list_based_set.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/vyukov_hash_map.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/acquire_guard.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/vyukov_bounded_queue.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/quiescent_state_based.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/generic_epoch_based.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/impl
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/impl/quiescent_state_based.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/impl/generic_epoch_based.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/impl/hazard_pointer.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/impl/lock_free_ref_count.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/impl/hazard_eras.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/impl/stamp_it.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/detail
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/detail/retire_list.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/detail/deletable_object.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/detail/concurrent_ptr.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/detail/perf_counter.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/detail/guard_ptr.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/detail/allocation_tracker.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/detail/thread_block_list.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/detail/orphan.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/hazard_pointer.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/lock_free_ref_count.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/hazard_eras.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/xenium/reclamation/stamp_it.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/lib/libpmc.so
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/lib/cmake/pmc/pmcTargets.cmake
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/lib/cmake/pmc/pmcTargets-release.cmake
  -- Up-to-date: /tmp/tmpvcokejo2/wheel/platlib/include
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/pmc
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/pmc/pmc_utils.h
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/pmc/pmc_neigh_coloring.h
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/pmc/pmc_graph.h
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/pmc/pmcx_maxclique.h
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/pmc/pmc_neigh_cores.h
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/pmc/pmc.h
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/pmc/pmcx_maxclique_basic.h
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/pmc/pmc_debug_utils.h
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/pmc/pmc_vertex.h
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/pmc/pmc_maxclique.h
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/pmc/pmc_heu.h
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/pmc/pmc_headers.h
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/pmc/pmc_input.h
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/lib/cmake/pmc/pmcConfig.cmake
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/lib/librobin.a
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/lib/libpmc.so
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/lib/cmake/robin/robinTargets.cmake
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/lib/cmake/robin/robinTargets-release.cmake
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/lib/cmake/robin/robinConfig.cmake
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/lib/cmake/robin/robinConfigVersion.cmake
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/robin
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/robin/robin.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/robin/core.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/robin/utils.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/robin/graph_io.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/robin/views.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/robin/math.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/robin/problems.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/robin/graph_solvers.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/robin/graph.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/robin/pkc.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/robin/macros.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/include/robin/graph_core.hpp
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/lib/cmake/xenium/xeniumConfig.cmake
  -- Installing: /tmp/tmpvcokejo2/wheel/platlib/./spark_robin.cpython-310-x86_64-linux-gnu.so
  -- Set runtime path of "/tmp/tmpvcokejo2/wheel/platlib/./spark_robin.cpython-310-x86_64-linux-gnu.so" to ""
  *** Making wheel...
  *** Created spark_robin-1.2.0-cp310-cp310-linux_x86_64.whl
  Building wheel for spark_robin (pyproject.toml) ... done
  Created wheel for spark_robin: filename=spark_robin-1.2.0-cp310-cp310-linux_x86_64.whl size=453539 sha256=e61f4898fddd7f6867abdacb3f2e149abc3d1d43230e2277b2330a4f0a771419
  Stored in directory: /tmp/pip-ephem-wheel-cache-gxd_w6c_/wheels/0a/d0/d8/c1ca94966b9bcc315d87083f82cb67c729487a165eb5192760
Successfully built spark_robin
Installing collected packages: spark_robin
  Attempting uninstall: spark_robin
    Found existing installation: spark_robin 1.2.0
    Uninstalling spark_robin-1.2.0:
      Removing file or directory /usr/local/lib/python3.10/dist-packages/include/pmc/
      Removing file or directory /usr/local/lib/python3.10/dist-packages/include/robin/
      Removing file or directory /usr/local/lib/python3.10/dist-packages/include/xenium/
      Removing file or directory /usr/local/lib/python3.10/dist-packages/lib/cmake/pmc/
      Removing file or directory /usr/local/lib/python3.10/dist-packages/lib/cmake/robin/
      Removing file or directory /usr/local/lib/python3.10/dist-packages/lib/cmake/xenium/
      Removing file or directory /usr/local/lib/python3.10/dist-packages/lib/libpmc.so
      Removing file or directory /usr/local/lib/python3.10/dist-packages/lib/librobin.a
      Removing file or directory /usr/local/lib/python3.10/dist-packages/spark_robin-1.2.0.dist-info/
      Removing file or directory /usr/local/lib/python3.10/dist-packages/spark_robin.cpython-310-x86_64-linux-gnu.so
      Successfully uninstalled spark_robin-1.2.0
Successfully installed spark_robin-1.2.0

```

그 후, 위처럼 Installation 후 wheel 파일을 생성하는 것을 볼 수있다!
`CMakelist.txt`에서 `install(TARGETS pypatchworkpp DESTINATION .)`라고 하는 부분에서 저 . 때문에 `/tmp/tmpvcokejo2/wheel/platlib/./spark_robin.cpython-310-x86_64-linux-gnu.so`
사이에 .이 있는 것이다. 
저 주소를 통해 `.so` 파일이 처음 생성되는 위치를 지정할 수 있고, 그 줄 뒤의 ""라고 되어 있는 부분은 `pyproject.toml` 내에서

```commandline
[tool.scikit-build]
cmake.args = ["-DCMAKE_INSTALL_RPATH=${DESINATION}"]
```

과 같이 쓰면 저 "" 부분도 변경할 수 있다(그런데 둘 다 안 건드려도 잘 build된다). 즉 해당 줄이 아래와 같이 된다:

```commandline
Set runtime path of "/tmp/tmpvcokejo2/wheel/platlib/./spark_robin.cpython-310-x86_64-linux-gnu.so" to "${DESINATION}"
```

## Step 4. Git Actions 설정 

그 후, 여기 [pypi.yml](https://github.com/url-kaist/patchwork-plusplus/blob/master/.github/workflows/pypi.yml)처럼 Action을 설정하면 된다! 

```angular2html
name: Publish to PyPI.org

on:
  workflow_dispatch:
  release:
    types: [published]
  push:
    branches: ["master"]
  pull_request:
    branches: ["master"]

jobs:
  build_sdist:
    name: Build source distribution
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3

      - name: Build sdist
        run: pipx run build --sdist ${{github.workspace}}/python/
      - name: Move sdist to dist
        run: mkdir -p dist && mv ${{github.workspace}}/python/dist/*.tar.gz dist/

      - uses: actions/upload-artifact@v3
        with:
          path: dist/*.tar.gz

  cibuildwheel:
    name: Build wheels on ${{ matrix.os }}
    runs-on: ${{ matrix.os }}
    strategy:
      matrix:
        os: [ubuntu-22.04, windows-2022, macos-14]

    steps:
      - uses: actions/checkout@v3

      - name: Build test wheels (only PRs)
        if: github.event_name != 'release'
        uses: pypa/cibuildwheel@v2.16.5
        env: # build 1 build per platform just to make sure we can do it later when releasing
          CIBW_BUILD: "cp310-*"
        with:
          package-dir: ${{github.workspace}}/python/

      - name: Build all wheels
        if: github.event_name == 'release'
        uses: pypa/cibuildwheel@v2.16.5
        with:
          package-dir: ${{github.workspace}}/python/

      - uses: actions/upload-artifact@v3
        with:
          path: ./wheelhouse/*.whl

  pypi:
    if: github.event_name == 'release'
    needs: [cibuildwheel, build_sdist]
    runs-on: ubuntu-latest
    steps:
      - uses: actions/download-artifact@v3
        with:
          name: artifact
          path: dist

      - uses: pypa/gh-action-pypi-publish@release/v1
        with:
          password: ${{ secrets.PYPI_API_TOKEN }}
```

여기서 `on`은 Github의 branch의 조건을 나타내고, `jobs` 상의 `build_sdist`, `cibuildwheel`은 각각 실행되는 것들을 뜻한다.
만약 이 글을 읽는 당신이 이 글을 읽고 위의 글을 반영하고 싶다면:

* 메인 branch가 main인지 master인지 확인하고 on 부분 변경
* `pyproject.toml`이 있는 위치를 `${{github.workspace}}/python/`를 통해 가리켜야 함. 만약 그냥 폴더 내에 있지 않는다면 `$${{github.workspace}}`라고만 쓰면 된다.
* github.event_name == 'release'로 되어 있으므로, repository 우측에 보는 Tags를 눌러서 release인 태그를 생성한다
* 레포지토리의 `secrets.PYPI_API_TOKEN`을 설정

## Step 5. Pypi 가입하기 & PYPI_API_TOKEN 설정하기 

3. [Pypi.org](https://pypi.org/)에 가서 ID를 가입하자(가입 절차는 알잘딱 따라가기만 하면 되니 설명은 생략한다). 
4. 가입 후, 해야할 것이 있는데, GitHub의 SSH key를 등록하듯이, Pypi에도 토큰을 등록해야 한다. 등록하는 방법 절차는 아래와 같다 (ChatGPT 선생님이 알려줬다).

    1. 로그인 후 오른쪽 상단의 계정 아이콘을 클릭하여 **"Account settings"**로 이동
    2. "API tokens" 섹션에서 "Add API token" 버튼을 클릭
    3. 토큰 이름(자유롭게 지정하면 됨)과 권한 범위(마다 **Entire account** 밖에 없을 것이다. 그거 누루면 됨)를 설정
    4. **"Add token"**을 클릭하면, API 토큰이 생성됩

이때 표시되는 토큰은 한 번만 확인할 수 있으므로 반드시 안전한 곳에 저장해야 한다고 한다. 그리고 저 `pypi-`로 시작되는 토큰을 복사한 후, '~/.pypirc`에 아래와 같이 작성하면:

```
[pypi]
username = __token__
password = pypi-${FOLLOWING PASSWORD}
```

로컬 환경에서 Pypi로 파일을 업로드할 수 있게 된다. 
만약 로컬 컴퓨터에서 우리가 위의 과정을 거치지 않은채 manual로 wheel file을 Pypi로 업로드하면 `Enter your API token:` 라고 되물을 것이지만, 우리는 token을 컴퓨터 상에 등록했으므로 token을 치지 않아도 된다.

근데 우리는 **Actions이 대신해주기 때문**에, 우리의 repository로 가서 `Settings -> Secrets and variables -> Actions -> Repository secrets`에 `New repository secret`을 누른 후, `PYPI_API_TOKEN`라는 이름으로 변수 추가해주면 된다. 

---

## Step 6. Pypi Set up

그 후, pypi.org  첫 페이지 좌측 하단의 `Publishing`을 눌러서 우리가 pypi로 Actions을 통해 wheel 파일들을 업로드할 것임을 미리 고지(?)해줘야 한다.

![](/img/publishing_key.png)

자세한 사항은 [여기 블로그](https://velog.io/@bailando/github-actions-%EC%9C%BC%EB%A1%9C-pypi-%ED%8C%A8%ED%82%A4%EC%A7%80-%EB%B0%B0%ED%8F%AC-%EC%9E%90%EB%8F%99%ED%99%94%ED%95%98%EA%B8%B0)를 참고하면 된다(이 부분은 그리 어렵지 않음). 

위의 과정을 거치면 "Pending Publisher"과 같이 되어 있을텐데, 그 상태에서 Actions을 실행하면 코드들이 자동으로 wheel 파일로 변경되어서 Pypi에 업로드가 된다!


# 마치며 

처음에는 잘 몰라서 `python3 -m build`, `python3 -m twine upload dist/*` 등으로 수작업으로 한 번 진행해보려고 했는데, 이 것을 **불가능**하다.
Wheel 파일을 Ubuntu에서 생성하면 tag가 `-linux`와 같이 생기는데, Pypi에는 `-manilinux`와 같이 다양한 linux type에 지원할 수 있는 wheel 파일만 업로드 가능하기 때문.
그러나 이러한 번거로운 과정도 위의 Actions이 다 해결해준다. 

이 과정을 거치면서 '아직 ChatGPT가 멀었구나...'를 많이 느꼈다. 아무래도 소수의 개발자만 할 짓이다보니, 자료가 많이 없는 것 같다. 
모쪼록 이 글을 읽는 이는 자기 스스로 자신의 연구를 널리널리 스스로 알릴 수 있는 역량을 잘 기르길!
