---
layout: post
title: Pybind11 Line by Line - 1. Open Source Codes Analyses
subtitle: CMakeLists.txt setting for Pybind11
tags: [Pybind11, Pybinding, Python, C++]
comments: true
---

## Motivation

요즘 멋있는 library를 보면 다들 pybind를 지원하는 것을 볼 수 있다.
[TEASER++](https://github.com/MIT-SPARK/TEASER-plusplus)나 [KISS-ICP](https://github.com/PRBonn/kiss-icp)같이 robotics 분야에서 별을 1,000개 이상 받은 repository들을 보면 다 pybind 관련 CMakelists.txt 파일이나 코드도 포함하고 있는데,  
개인적인 생각으로 이는 굉장히 전략적으로 좋다고 생각한다.
왜냐하면 시대의 흐름 상 robotics 인들의 수 <<<< 딥러닝 연구자들의 수인 것이 당연하기 때문에, 
더 많은 사람들의 관심을 끌기 위해서는 python을 지원하는 것이 좋다고 생각한다. 

사실 Nacho가 KISS-ICP를 배포하는 것을 옆에서 지켜봤을 때, Nacho는 pybind를 씀과 동시에 PyPI에 코드를 배포해서 pip install로 코드를 설치하는 것도 지원하였는데, 진짜 대단한 거 같음...나중에 그 경지에 도달하고 싶지만 일단 차근차근 pybind에 대해 분석해본다.~~나도 개발 잘하는 연구자가 되고 싶다~~ 

이번 시간에는 주로 [cmake_example](https://github.com/pybind/cmake_example), [KISS-ICP](https://github.com/PRBonn/kiss-icp), [TEASER++](https://github.com/MIT-SPARK/TEASER-plusplus) 내부에 있는 pybinding 관련 코드들을 살펴보고 분석한다.

---

### `cmake_example` 빌드해보기

[https://github.com/pybind/cmake_example](https://github.com/pybind/cmake_example)에서 pybinding을 하는 좋은 예제를 이미 제공하고 있다. 
터미널 창에서 아래와 같이 치면:

```commandline
git clone --recursive https://github.com/pybind/cmake_example.git
pip3 install ./cmake_example
```

아래와 같이 빌드가 된다

![](/img/pybinding_cmake_example_results.png)

그리고 아무 터미널창을 다시 열어서 python을 실행시키고 아래와 같이 치면:

![](/img/pybinding_results.png)

위와 같이 `cmake_exmaple`이라는 패키지가 wheel 파일로 변환되어 잘 설치되었음을 확인할 수 있다.
아래는 이러한 pybinding이 어떻게 가능하게 된 것인지 코드레벨로 분석해본다.

---

## Pybind 코드 레벨 이해하기

### 1. Pybind11 레포지토리 복사/설치

Pybind를 하기 위해서는 우선 [pybind11](https://github.com/pybind/pybind11/tree/914c06fb252b6cc3727d0eedab6736e88a3fcb01) 레포지토리가 필요하다.
그래서 `cmake_exmple`에서는 `git submodule`을 이용해서 pybind11을 가져온다 (그래서 `git clone` 시 `--recursive`를 꼭 붙여줘야 함). 
마찬가지로 TEASER++에서도 pybind11을 가져오는데, 여기서는 modern CMake의 정신인 target-oriented CMake (> 3.13에서부터 지원)을 이용해서 pybind11을 가져오는 것을 볼 수 있다.

```commandline
# In `https://github.com/MIT-SPARK/TEASER-plusplus/blob/master/CMakeLists.txt` 
if (BUILD_PYTHON_BINDINGS)
    set(PYBIND11_PYTHON_VERSION ${TEASERPP_PYTHON_VERSION})

    # download the pybind11 repo
    configure_file(cmake/pybind11.CMakeLists.txt.in pybind11-download/CMakeLists.txt)
    execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
            WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/pybind11-download")
    execute_process(COMMAND "${CMAKE_COMMAND}" --build .
            WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/pybind11-download")
    add_subdirectory("${CMAKE_BINARY_DIR}/pybind11-src"
            "${CMAKE_BINARY_DIR}/pybind11-build")

    message(STATUS "TEASER++ Python binding will be built.")
    add_subdirectory(python)
endif ()
```

위의 CMakelist.txt를 보면 `pybind11-download`라는 폴더를 만들고 그 안에 `cmake/pybind11.CMakeLists.txt.in`가 가리키고 있는 레포지토리를 해당 폴더에 다운로드 받는다.
`cmake/pybind11.CMakeLists.txt.in` 파일을 보면 아래와 같이 되어있다.

```commandline
# `https://github.com/MIT-SPARK/TEASER-plusplus/blob/master/cmake/pybind11.CMakeLists.txt.in`
project(pybind11-download NONE)

include(ExternalProject)
ExternalProject_Add(pmc
        GIT_REPOSITORY    https://github.com/pybind/pybind11.git
        GIT_TAG           v2.11.1
        SOURCE_DIR        "${CMAKE_CURRENT_BINARY_DIR}/pybind11-src"
        BINARY_DIR        "${CMAKE_CURRENT_BINARY_DIR}/pybind11-build"
        CONFIGURE_COMMAND ""
        BUILD_COMMAND     ""
        INSTALL_COMMAND   ""
        TEST_COMMAND      ""
        )
```

즉, `ExternalProject_Add`를 이용해서 `pybind11` repository를 타겟삼아, 흔히 C++ 패키지를 빌드할 때 사용하는 `cmake .. →  make -j 8`를 하면 자동으로 pybind11을 다운로드해주는 것을 확인할 수 있다.

따라서 위의 두 행위는 동일한 행위를 하는 것이다.

### 2. PYBIND11_MODULE 설정하기

그 후, `cmake_example`의 [main.cpp](https://github.com/pybind/cmake_example/blob/master/src/main.cpp)를 보면 아래와 `PYBIND11_MODULE`이라는 것이 선언되어 있는 것을 볼 수 있다.
패턴을 살펴봤을 때, `PYBIND11_MODULE(${package_name}, m)`이라고 입력하는 듯하다.
그리고 함수의 경우에는 `m.def`를 이용해서 python에서 사용할 수 있도록 binding을 해주는 것을 볼 수 있다.

```cpp

```cpp
#include <pybind11/pybind11.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

int add(int i, int j) {
    return i + j;
}

namespace py = pybind11;

PYBIND11_MODULE(cmake_example, m) {
    m.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------

        .. currentmodule:: cmake_example

        .. autosummary::
           :toctree: _generate

           add
           subtract
    )pbdoc";
    /* 
    Hyungtae's comment: Python 상에 쓸 '"add"'라는 이름의 함수를 지정하고,
    이 함수를 C++의 'add' 함수와 binding해준다.
    */
    m.def("add", &add, R"pbdoc(
        Add two numbers

        Some other explanation about the add function.
    )pbdoc");

    /* 
    Hyungtae's comment: Python 상에 쓸 '"subtract"'라는 이름의 함수를 지정하고,
    이 함수를 C++의 lambda function을 이용해 binding해준다.
    */
    m.def("subtract", [](int i, int j) { return i - j; }, R"pbdoc(
        Subtract two numbers

        Some other explanation about the subtract function.
    )pbdoc");

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
```

반면, TEASER++에서는 class도 있고, 멤버 변수도 있고하다보니 좀더 복잡하다 [[여기]](https://github.com/MIT-SPARK/TEASER-plusplus/blob/master/python/teaserpp_python/teaserpp_python.cc).
클래스의 경우는 `py::class_<teaser::RegistrationSolution>(m, "RegistrationSolution")`와 같은 꼴로 선언되어 있고, 멤버 변수의 경우는 `def_readwrite`를 이용해서 선언되어 있다.
이해를 돕기 위해 정리하자면, `def`와 `def_readwrite`의 차이점은 아래와 같다:

- `def_readwrite`: Python에서 C++ 클래스의 멤버 변수에 접근할 수 있도록 함. 즉, `def_readwrite`를 사용하면 Python 코드에서 해당 멤버 변수를 읽고 쓸 수 있음 
- `def`: 클래스의 함수나 C++ 함수를 binding할 때 사용. 이를 통해 Python에서 해당 함수를 호출할 수 있게 
됨

### 3. CMakeLists.txt 설정하기

그 후, `cmake_exampl`의 `CMakeLists.txt`를 보면 아래와 같이 `pybind11_add_module`이라는 명령어를 통해 위에 작성된 `main.cpp` source code를 pybind11을 통해 모듈을 추가하는 것을 볼 수 있다. 

```commandline
cmake_minimum_required(VERSION 3.4...3.18)
project(cmake_example)

add_subdirectory(pybind11)
pybind11_add_module(cmake_example src/main.cpp)

# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.
target_compile_definitions(cmake_example
                           PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
```                           

즉, `cmake_example`이라는 module에 `main.cpp`를 pybind11을 통해 binding을 해주는 것을 볼 수 있다.
주의해야할 것은 `cmake_example`이라는 module 이름은 `PYBIND11_MODULE(${package_name}, m)`에서 `${package_name}`와 동일해야한다는 것이다.

---

### 4. P.S. C++ Library <-> Pybinding

여기서 궁금증은, 간단한 코드나 C++ library-free한 코드들의 경우는 ok인데, C++의 stl container나 Eigen을 사용해서 짠 코드들은 어떻게 저 `.def`에 작성하는 지 궁금할 수 있다.
이는 KISS-ICP의 [kiss_icp_pybind.cpp](https://github.com/PRBonn/kiss-icp/blob/main/python/kiss_icp/pybind/kiss_icp_pybind.cpp)를 보면 어떻게 처리해야하는 지 간접적으로 알 수 있다:

```cpp
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
```

이와 같이 `pybind11` 안에 있는 header 파일을 include해주면 `def`를 작성할 때 어떤 함수의 입력 type에 대해 작성해야하는 상황에서도 편하게 작성할 수 있다.
또한 pybinding을 보면 `#include <pybind11/numpy.h>`같이 numpy를 사용할 수 있도록 해주는 header 파일도 존재하는데, 이를 이용해서 C++을 짜서 binding을 하면 numpy를 입력으로 받는 함수를 선언할 수 있다.


---


Pybinding line-by-line 설명 시리즈입니다.

1. [Pybind11 Line by Line - 1. Open Source Codes Analyses](https://limhyungtae.github.io/2023-12-14-Pybind11-Line-by-Line-1.-Open-Source-Codes-Analyses/)
2. [Pybind11 Line by Line - 2. Package에 대한 이해](https://limhyungtae.github.io/2023-12-14-Pybind11-Line-by-Line-2.-Package%EC%97%90-%EB%8C%80%ED%95%9C-%EC%9D%B4%ED%95%B4/)






