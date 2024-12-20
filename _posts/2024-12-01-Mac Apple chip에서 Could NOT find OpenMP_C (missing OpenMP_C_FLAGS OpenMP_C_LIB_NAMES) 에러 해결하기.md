---
layout: post
title: Mac Apple chip에서 Could NOT find OpenMP_C (missing OpenMP_C_FLAGS OpenMP_C_LIB_NAMES) 에러 해결하기
subtitle: Run Algorithms on Mac
tags: [MacOS, C++]
comments: true
---

## 문제

요즘에는 Linux 뿐만 아니라 Mac에서도 코드가 동작하게끔하는 데에 관심이 많은데, 최근에 Linux system에서는 잘 동작하는 코드를 `cmake`를 통해 build를 했을 때 아래와 같은 에러가뜨는 것을 볼 수 있었다:

```
CMake Error at /opt/homebrew/share/cmake/Modules/FindPackageHandleStandardArgs.cmake:233 (message):
  Could NOT find OpenMP_C (missing: OpenMP_C_FLAGS OpenMP_C_LIB_NAMES)
Call Stack (most recent call first):
  /opt/homebrew/share/cmake/Modules/FindPackageHandleStandardArgs.cmake:603 (_FPHSA_FAILURE_MESSAGE)
  /opt/homebrew/share/cmake/Modules/FindOpenMP.cmake:616 (find_package_handle_standard_args)
  CMakeLists.txt:26 (find_package)
```

## 원인 

MacOS에서 OpenMP를 사용할 때 컴파일러의 지원 및 설정 문제에서 비롯된 것이라고 한다.
무슨 소리인가 하면, 원래 OpenMP는 C++과 C compiler가 알잘딱 라이브러리(`libomp`) 위치를 찾아내서 링크를 걸어주어야 하는데, 
Apple에서 제공하는 기본 clang (AppleClang)은 OpenMP를 제대로 지원하지 않기 때문이라고 한다.


## 해결책

사실 깊게 이유에 대해 알 필요는 없다. 해결책은 먼저 LLVM을 설치해주어야 한다.
LLVM은 Low-Level Virtual Machine의 약자로, 원래는 컴파일러와 관련된 저수준 가상 기계를 목표로 했지만, 현재는 컴파일러 인프라 프로젝트로 확장되어 다양한 언어와 플랫폼을 지원하는 도구 모음을 의미한다고 한다.

```
brew install llvm
```

그 후, 자신이 사용하고자 하는 `.bashrc`나 `.zshrc`에 아래 두 줄을 추가해주면 된다:

```
export CC=/opt/homebrew/opt/llvm/bin/clang
export CXX=/opt/homebrew/opt/llvm/bin/clang++
```
