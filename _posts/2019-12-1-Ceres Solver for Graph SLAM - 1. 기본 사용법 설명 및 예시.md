---
layout: post
title: Ceres Solver for Graph SLAM - 1. 기본 사용법 설명 및 예시
subtitle: C++에서 Ceres Solver 사용하기
tags: [SLAM, Optimization, Ceres Solver, C++]
comments: true
---

# Introduction

![ceresintro](/img/ceres_intro.png)

Ceres Solver는 Google에서 개발한 Non-linear Optimization Library입니다. 

처음 Graph SLAM을 시작하는 분들이라면 한번 쯤 [Cartographer](https://opensource.googleblog.com/2016/10/introducing-cartographer.html)를 들어봤으셨을텐데, Cartographer 내부를 보시면 Ceres Solver로 optimization을 하고 있는 것을 볼 수 있습니다. 이처럼 Non-linear cost function을 optimization하는 것을 Ceres Solver를 이용하면 간단히 최적화할 수 있기 때문에, Ceres Solver는 저희가 *어떻게* optimization을 하는가에 대한 노력을 쏳는 것을 줄이게 만들고 무엇을* optimization할지에 더 집중할 수 있게 해줍니다.

# 튜토리얼 자료 설치


![ceres](/img/ceres_cmake_CMakeLists.png)

![make](/img/ceres_make.png)

![hello_world](/img/ceres_hello_world.png)

# Ceres Solver Tutorial

