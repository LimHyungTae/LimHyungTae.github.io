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

처음 Graph SLAM을 시작하는 분들이라면 한번 쯤 [Cartographer](https://opensource.googleblog.com/2016/10/introducing-cartographer.html)를 들어봤으셨을텐데, Cartographer 내부를 보시면 Ceres Solver로 optimization을 하고 있는 것을 볼 수 있습니다. 이처럼 Non-linear cost function을 Ceres Solver를 이용하면 간단히 최적화할 수 있기 때문에, Ceres Solver는 저희가 **어떻게 optimization을 하는가**에 대한 노력을 쏟는 것을 줄이게 만들고 **무엇을 optimization할지**에 더 집중할 수 있게 해줍니다.

# 튜토리얼 자료 설치

튜토리얼 자료는 [여기](https://github.com/LimHyungTae/helloceres)에 github로 올려두었습니다. Step-by-step으로 설명드릴 예정입니다.

1. 먼저 기존의 Ceres 공식 홈페이지에서 Ceres Solver를 설치한다.

[http://ceres-solver.org/installation.html](http://ceres-solver.org/installation.html)

2. 위의 git 레포지토리를 clone한다.

<pre><code>$ git clone https://github.com/LimHyungTae/helloceres.git</code></pre>

3. 

그리고 helloceres 폴더로 이동한 후 cmake와 make를 해준다.

<pre><code>$ cd helloceres</code></pre>


<pre><code>$ cmake CMakeLists.txt</code></pre>

**==> 결과**

![ceres](/img/ceres_cmake_CMakeLists.png)

<pre><code>$ make</code></pre>

**==> 결과**

![make](/img/ceres_make.png)

그러면 폴더 상에 컴파일된 파일이 생성되는 것을 볼 수 있다. 

### 실행

실행은 아래와 같이 하면 결과가 출력된다.

<pre><code>$ ./${filename}</code></pre>

# 1. Hello World

![hello_world](/img/ceres_hello_world.png)


# 2 여러 Cost function 사용하기



![hello_world](/img/ceres_hello_world.png)

