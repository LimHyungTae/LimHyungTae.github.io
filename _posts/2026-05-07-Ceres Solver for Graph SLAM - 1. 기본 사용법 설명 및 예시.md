---
layout: post
title: Ceres Solver for Graph SLAM - 1. 기본 사용법 설명 및 예시
subtitle: Hello World와 Cost function 여러 개 다루기
tags: [SLAM, Optimization, Ceres Solver, C++]
comments: true
description: Ceres Solver의 가장 기본적인 사용법을 Hello World와 multiple residual 예제로 정리한다. Cost functor 정의, AddResidualBlock, Solve의 세 단계로 SLAM optimization의 출발선을 짚는다.
image: /img/ceres_intro.png
permalink: /2026/05/07/ceres-graph-slam-01-basic-usage/
redirect_from:
  - '/2019-11-14-Ceres Solver for SLAM (1) /'
  - '/2019-12-1-Ceres Solver for Graph SLAM - 1. 기본 사용법 설명 및 예시/'
  - '/2019-11-14-Ceres-Solver-for-SLAM-(1)-/'
  - '/2019-12-1-Ceres-Solver-for-Graph-SLAM-1.-기본-사용법-설명-및-예시/'
  - '/2026-05-07-Ceres-Solver-for-Graph-SLAM-1.-기본-사용법-설명-및-예시/'
---

## Introduction

![ceresintro](/img/ceres_intro.png)

Ceres Solver는 Google에서 개발한 non-linear optimization library이다. 처음 Graph SLAM을 시작하는 사람이라면 한 번쯤 [Cartographer](https://opensource.googleblog.com/2016/10/introducing-cartographer.html)를 들어보았을 텐데, Cartographer 내부를 들여다보면 결국 Ceres Solver로 optimization을 하고 있는 것을 볼 수 있다. 이처럼 비선형 cost function을 *Ceres에 던져 넣고 알아서 최적화시키는 형태*로 SLAM을 짤 수 있어서, *어떻게 optimization을 푸는가*보다는 *무엇을 optimize할 것인가*에 집중할 수 있게 만드는 게 Ceres의 가장 큰 장점이다.

이 글은 Ceres에 처음 입문하는 사람을 대상으로 한다. 즉 1편에서는 SLAM 같은 응용은 잠시 잊고, *Ceres가 어떤 모양의 입출력을 가진 library인지*만 깔끔하게 정리한다. SLAM에서의 본격적인 활용은 [3편](https://limhyungtae.github.io/2026/05/07/ceres-graph-slam-03-pose-graph-3d-overview/)부터 다룰 예정이다.

> 본래 이 시리즈는 2019년에 1편을 쓰고 멈춰있었다. 시간이 한참 지나는 동안 Ceres도 2.x로 올라가면서 API가 일부 바뀌었고 (대표적으로 `LocalParameterization` → `Manifold`), 모던 C++ 컴파일러 환경에서 빌드 방법도 달라졌다. 이 글은 그 변화들을 반영해 *2026년 기준*으로 다시 정리한 버전이다.

이번 sub-series 전체의 흐름은 다음과 같다.

* **1편 (현재 글)**: Ceres의 기본, Hello World와 cost function 여러 개 사용하기.
* **2편**: Pose error를 정의하는 법, 2D pose부터 시작해서 3D quaternion으로 가는 다리 놓기.
* **3편**: Pose Graph 3D example 한눈에 보기.
* **4편**: `PoseGraph3dErrorTerm` 깊게 이해하기, residual의 수학과 AutoDiff.
* **5편**: `BuildOptimizationProblem`과 `Manifold` 깊게 이해하기.

각설하고, 출발해보자.

---

## 튜토리얼 자료

이 시리즈에서 다룰 minimal example들은 [LimHyungTae/helloceres](https://github.com/LimHyungTae/helloceres) 레포에 모아두었다 (Ubuntu 22.04/24.04, macOS 15에서 동작 확인).

```bash
sudo apt install libceres-dev          # macOS는 brew install ceres-solver
git clone https://github.com/LimHyungTae/helloceres.git && cd helloceres
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

빌드가 끝나면 `build/`에 각 예제 실행파일이 생긴다. macOS Anaconda 사용자의 gflags/glog 충돌 회피 옵션이나 Docker 빌드는 [README](https://github.com/LimHyungTae/helloceres#hammer-build)에 정리해뒀다.

---

## 예제 1: Hello World

가장 간단한 NLLS 문제부터 풀어보자. *어떤 변수 $$x$$의 값을 10에 가깝게 만들어 보자*. 즉 다음 cost를 minimize한다.

$$
\min_{x} \frac{1}{2} (10 - x)^2
$$

당연히 답은 $$x = 10$$인데, 이걸 Ceres로 푸는 코드는 다음과 같다.

```cpp
#include <iostream>
#include "ceres/ceres.h"

// (i) Cost functor: residual = 10 - x
struct CostFunctor {
  template <typename T>
  bool operator()(const T* const x, T* residual) const {
    residual[0] = T(10.0) - x[0];
    return true;
  }
};

int main(int argc, char** argv) {
  // (ii) Optimization 변수 선언 및 초깃값
  double x = 0.5;
  const double initial_x = x;

  // (iii) Problem과 cost function 등록
  ceres::Problem problem;
  ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
  problem.AddResidualBlock(cost_function, /*loss=*/nullptr, &x);

  // (iv) Solver 옵션 세팅 후 solve
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << initial_x << " -> " << x << "\n";
  return 0;
}
```

코드는 짧지만 Ceres의 핵심 idea를 다 담고 있다. 크게 네 부분으로 나누어 보자.

### (i) Cost functor

```cpp
struct CostFunctor {
  template <typename T>
  bool operator()(const T* const x, T* residual) const {
    residual[0] = T(10.0) - x[0];
    return true;
  }
};
```

Residual을 계산하는 *함수 객체(functor)*를 정의한다. `operator()`가 입력 parameter `x`를 받아 residual을 채우면 된다. 핵심 디테일은 두 가지이다.

* **`template <typename T>`**: AutoDiff가 동작하려면 functor가 `double` 외에 `ceres::Jet<double, N>`도 받아들일 수 있어야 한다. 자세한 원리는 [4편](https://limhyungtae.github.io/2026/05/07/ceres-graph-slam-04-posegraph3d-error-term/)에서 다룬다.
* **`return true`**: residual 계산이 정상적으로 이뤄졌다는 신호. NaN/Inf가 나오는 등 실패한 경우엔 `false`를 리턴해서 LM이 step size를 줄이게 만들 수 있다.

### (ii) Optimization 변수

```cpp
double x = 0.5;
```

Ceres는 *raw double pointer*를 통해 parameter를 관리한다. 그래서 변수는 일반적인 C++ scalar 또는 array로 선언하고, *그 pointer를 그대로 Ceres에 넘긴다*. Ceres는 이 메모리를 *in-place로 직접 update*하기 때문에, optimization이 끝난 후에는 `x` 값이 자동으로 최적값으로 갱신되어 있다.

### (iii) Problem과 cost function 등록

```cpp
ceres::Problem problem;
ceres::CostFunction* cost_function =
    new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
problem.AddResidualBlock(cost_function, nullptr, &x);
```

`AutoDiffCostFunction<Functor, ResidualDim, Param1Dim>`의 template 인자는 다음과 같이 읽으면 된다.

* `CostFunctor`: residual을 계산하는 functor type
* `1`: residual의 dimension (스칼라이므로 1)
* `1`: parameter `x`의 dimension (스칼라이므로 1)

`AddResidualBlock`의 두 번째 인자는 *loss function*인데, `nullptr`이면 표준 L2 cost ($$\frac{1}{2} \|\mathbf{r}\|^2$$)를 그대로 쓴다는 의미이다. Outlier가 있을 때 robust loss를 쓰고 싶으면 여기에 `new ceres::HuberLoss(1.0)` 등을 넘기면 된다.

세 번째 이후 인자는 parameter block의 pointer들이다. 여기서는 `x` 하나뿐이라 `&x` 하나만 넘긴다.

### (iv) Solve

```cpp
ceres::Solver::Options options;
options.linear_solver_type = ceres::DENSE_QR;
options.minimizer_progress_to_stdout = true;

ceres::Solver::Summary summary;
ceres::Solve(options, &problem, &summary);
```

* `linear_solver_type = DENSE_QR`: 작은 dense 문제에는 QR이 가장 안정적이다. 큰 SLAM 문제에서는 `SPARSE_NORMAL_CHOLESKY`로 바꿔주면 빠르다.
* `minimizer_progress_to_stdout = true`: iteration마다 cost 변화가 콘솔에 찍힌다. 디버깅에 유용하다.

`ceres::Solve`를 호출하면 `summary`에 결과 정보가 들어가고, parameter `x`가 자동으로 최적값으로 갱신된다.

### 실행 결과

```
iter      cost      cost_change  ...
   0  4.512500e+01    0.00e+00   ...
   1  4.511505e-28    4.51e+01   ...

Ceres Solver Report: Iterations: 2, ...
x : 0.5 -> 10
```

초기값 `0.5`에서 출발해 단 한 번의 iteration으로 정확히 `10`으로 수렴한다. 사실 이 문제는 cost가 $$x$$에 대해 quadratic이라 LM이 한 step에 정답을 찾아간다.

---

## 예제 2: Cost function 여러 개 사용하기

실제 SLAM에서는 cost가 단일 residual이 아니라 *여러 residual의 합*으로 정의된다. Pose graph optimization을 예로 들면, graph의 각 edge가 하나의 residual을 만들고 전체 cost는 모든 edge residual의 합이다.

`add_residual_1.cc`에서 다루는 minimal 예제는 다음과 같다.

$$
\min_{x} \frac{1}{2} \bigl[ (10 - x)^2 + x^2 \bigr]
$$

gradient를 0으로 두면 $$x = 5$$가 답이다. 두 residual이 서로를 반대 방향으로 끌어당기다가 정확히 가운데에서 균형을 이루는 지점이다. 코드는 다음과 같다.

```cpp
struct CostA {
  template <typename T>
  bool operator()(const T* const x, T* residual) const {
    residual[0] = T(10.0) - x[0];
    return true;
  }
};

struct CostB {
  template <typename T>
  bool operator()(const T* const x, T* residual) const {
    residual[0] = x[0];
    return true;
  }
};

int main(int argc, char** argv) {
  double x = 0.5;

  ceres::Problem problem;

  // (1) 첫 번째 residual block: 10 - x
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CostA, 1, 1>(new CostA),
      nullptr, &x);

  // (2) 두 번째 residual block: x
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CostB, 1, 1>(new CostB),
      nullptr, &x);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << "x = " << x << "\n";  // 5가 출력
  return 0;
}
```

핵심은 *`AddResidualBlock`을 두 번 호출했다*는 점이다. Ceres는 등록된 모든 residual의 합을 cost로 묶어서 minimize한다. SLAM에서는 이 호출이 graph edge 수만큼(수천~수만 번) 일어난다.

지금은 두 residual의 가중치가 모두 1이라 정확히 가운데에서 멈춘다. 만약 `CostA`의 신뢰도가 더 높다면(= information이 크면) 답이 10 쪽으로 끌려간다. 이 *information weighting* 부분은 [4편](https://limhyungtae.github.io/2026/05/07/ceres-graph-slam-04-posegraph3d-error-term/)에서 sqrt-information matrix와 함께 다룬다.

---

## 결론

이 글에서는 Ceres의 가장 기본적인 사용법인 *Hello World*와 *cost function 여러 개 등록하기*를 살펴봤다. 한마디로 정리하면, Ceres에서 새로운 문제를 풀려면 다음 세 가지만 하면 된다.

1. **Cost functor 정의**: residual을 계산하는 `template <typename T> operator()` 짜기.
2. **`AddResidualBlock`**: cost functor를 problem에 등록하기 (residual block 단위).
3. **`Solve`**: solver option을 세팅하고 solve.

이게 전부이다. SLAM이든 curve fitting이든 robust regression이든 사용 패턴은 동일하다. *우리가 신경 쓸 부분은 (1) 단계의 residual을 어떻게 정의하느냐*뿐이다.

다음 [2편](https://limhyungtae.github.io/2026/05/07/ceres-graph-slam-02-pose-error/)에서는 SLAM의 본격적인 시작점인 *pose error*를 어떻게 정의하는지를 다룬다. 2D pose부터 시작해서 3D quaternion까지 천천히 살펴볼 예정이다.

---

Ceres Solver for Graph SLAM 시리즈입니다.

{% include post_links_ceres.html %}
