---
layout: post
title: Ceres Solver for Graph SLAM - 3. Pose Graph 3D Example 한눈에 보기
subtitle: How does pose_graph_3d.cc work?
tags: [SLAM, Optimization, Ceres Solver, Pose Graph, C++]
comments: true
---

## Introduction

[1편](https://limhyungtae.github.io/2026-05-07-Ceres-Solver-for-Graph-SLAM-1.-%EA%B8%B0%EB%B3%B8-%EC%82%AC%EC%9A%A9%EB%B2%95-%EC%84%A4%EB%AA%85-%EB%B0%8F-%EC%98%88%EC%8B%9C/)에서는 Ceres의 기본 사용법(Hello World, multiple residuals)을, [2편](https://limhyungtae.github.io/2026-05-07-Ceres-Solver-for-Graph-SLAM-2.-Pose-Error%EB%A5%BC-%EC%A0%95%EC%9D%98%ED%95%98%EB%8A%94-%EB%B2%95/)에서는 SLAM의 *pose error*가 2D에서 3D로 갈 때 갑자기 어려워지는 이유, *rotation은 manifold라서 multiplicative quaternion error로 정의해야 한다*는 것, 를 짚었다. 이번 sub-series (3~5편)에서는 그 idea가 실제 코드로 구현된 *Ceres pose_graph_3d 공식 example*을 line-by-line으로 깊게 분석한다.

우리가 분석할 [Pose Graph 3D 예제](https://github.com/ceres-solver/ceres-solver/tree/master/examples/slam/pose_graph_3d)는 코드 자체는 200줄도 안 되는 두 파일 ([`pose_graph_3d.cc`](https://github.com/ceres-solver/ceres-solver/blob/master/examples/slam/pose_graph_3d/pose_graph_3d.cc), [`pose_graph_3d_error_term.h`](https://github.com/ceres-solver/ceres-solver/blob/master/examples/slam/pose_graph_3d/pose_graph_3d_error_term.h)) 이지만, Ceres가 SLAM optimization을 푸는 방식의 모든 핵심이 이 안에 들어있다. 한 번에 읽으면 부담스러우니 *overview → residual의 수학 → API 활용*의 세 단계로 쪼갠다.

이 글은 Graph SLAM의 기본 개념은 이미 어느 정도 알고 있다는 가정하에 작성하였다. 만약 Graph SLAM이라는 개념 자체가 처음이라면, 이 시리즈를 읽기 전에 [수식없이 이해하는 Graph SLAM (1)](https://limhyungtae.github.io/2020-07-08-%EC%88%98%EC%8B%9D%EC%97%86%EC%9D%B4-%EC%9D%B4%ED%95%B4%ED%95%98%EB%8A%94-Graph-SLAM-(1)/) 시리즈를 먼저 읽어보길 권장한다. 또, Lie group / SE(3) / Hamilton quaternion에 대한 기본 지식도 어느 정도 있다는 전제로 진행한다 (만약 SE(3)가 익숙하지 않다면 [모두를 위한 SLAM](https://limhyungtae.github.io/slam/) Lecture 07에서 먼저 다지고 오는 걸 추천한다).

이번 sub-series는 총 세 편으로 진행한다.

* **3편 (현재 글)**: Pose Graph 3D Example 한눈에 보기, 두 파일이 어떻게 맞물려 돌아가는지에 대한 overview.
* **4편**: `PoseGraph3dErrorTerm` 깊게 이해하기, `.h` 파일의 6차원 residual이 수학적으로 정확히 무엇을 의미하는지.
* **5편**: `BuildOptimizationProblem`과 `Manifold` 깊게 이해하기, `.cc` 파일이 Ceres API를 어떻게 활용하는지.

각설하고, 출발해보자.

---

## Motivation: 왜 굳이 이 example을 깊게 보는가

Ceres를 막상 처음 써본 사람들이 자주 하는 질문이 있다. *"Hello world까진 따라쳐봤는데, 그래서 이걸 SLAM에 어떻게 쓰는 거야?"* 사실 [Ceres 공식 페이지의 tutorial](http://ceres-solver.org/tutorial.html)은 curve fitting과 robust regression 같이 일반적인 NLLS(Nonlinear Least Squares) 문제 위주로 설명되어 있어서, SLAM에서 흔히 쓰는 pose graph optimization으로 가는 다리가 살짝 비어있다.

그 다리를 정확히 채워주는 게 바로 `examples/slam/pose_graph_3d` 디렉토리이다. 이 코드는 다음 네 가지를 모두 보여준다.

* Ceres에서 **manifold 위의 optimization**을 어떻게 처리하는지 (quaternion이 4차원이지만 자유도는 3인 문제)
* **Multi-block parameter** (한 residual이 두 pose에 동시에 의존하는 상황)을 어떻게 등록하는지
* **AutoDiff**로 Jacobian을 손으로 짜지 않고도 어떻게 구할 수 있는지
* Pose graph의 **gauge freedom** (전체 graph를 평행이동/회전해도 cost가 바뀌지 않는 자유도)을 어떻게 fix하는지

개인적으로 Ceres example 중에서 가장 SLAM-friendly하고 또 가장 짧은 코드라고 생각한다. 그래서 깊게 분석할 가치가 충분하다.

---

## 본 시리즈의 목표

코드를 그냥 위에서부터 아래로 읽어내려가며 "여기는 이런 일을 합니다"라고 한 줄씩 설명하는 건 Ceres 공식 문서나 GitHub 주석으로도 가능하다. 그 정도로는 ~~ChatGPT한테 물어봐도 답해주는 시대~~인 만큼, 이 시리즈에서는 다음 세 가지를 명확히 하는 데 집중한다.

1. **수학과 코드의 1:1 매핑**: residual의 수학적 정의가 코드의 어느 line과 매칭되는지를 명시적으로 추적한다.
2. **API 호출의 *이유*에 대한 설명**: `SetManifold`, `SetParameterBlockConstant`, `LLT decomposition` 같은 호출이 *왜* 거기 있어야 하는지를 짚는다.
3. **AutoDiff의 동작 원리**: `template <typename T>`로 시작하는 한 줄이 어떻게 Jacobian을 자동으로 구해주는지 그 mechanism을 짚어본다.

---

## 출발점: `pose_graph_3d.cc`의 `main()` 흐름

먼저 전체 그림부터 보자. `pose_graph_3d.cc`의 `main()`은 굉장히 깔끔하다 (인자 파싱과 logging 부분은 가독성을 위해 생략).

```cpp
int main(int argc, char** argv) {
  // ... command-line flag 파싱 ...

  ceres::examples::MapOfPoses poses;
  ceres::examples::VectorOfConstraints constraints;

  // (1) g2o 파일에서 pose와 constraint를 읽음
  CHECK(ceres::examples::ReadG2oFile(
      absl::GetFlag(FLAGS_input), &poses, &constraints));

  // (2) Optimization 전 상태 저장
  CHECK(ceres::examples::OutputPoses("poses_original.txt", poses));

  // (3) Ceres problem 구성
  ceres::Problem problem;
  ceres::examples::BuildOptimizationProblem(constraints, &poses, &problem);

  // (4) Solve
  CHECK(ceres::examples::SolveOptimizationProblem(&problem));

  // (5) Optimization 후 상태 저장
  CHECK(ceres::examples::OutputPoses("poses_optimized.txt", poses));

  return 0;
}
```

흐름이 정확히 다섯 줄로 요약된다.

1. `g2o` format의 input file을 읽어 **pose 집합**과 **constraint 집합**을 얻는다.
2. 최적화 전 pose를 `poses_original.txt`로 출력한다.
3. `ceres::Problem` 객체를 만들고, `BuildOptimizationProblem()`이 cost function들을 모두 등록한다.
4. `SolveOptimizationProblem()`이 Levenberg–Marquardt iteration을 돌려 solve한다.
5. 최적화된 pose를 `poses_optimized.txt`로 출력한다.

핵심은 (3)과 (4)이다. (1)/(2)/(5)는 file I/O이고 사실상 boilerplate이다. 그래서 이 시리즈에서 깊게 들여다볼 함수는 `BuildOptimizationProblem()`(5편)과 그 안에서 호출되는 `PoseGraph3dErrorTerm`(4편) 두 개로 좁혀진다.

---

## 핵심 데이터 구조: `types.h`

코드를 이해하기 위해 알아야 할 type은 단 두 개이다 (Ceres example의 `types.h`에 정의되어 있다).

```cpp
struct Pose3d {
  Eigen::Vector3d p;          // position (3D translation)
  Eigen::Quaterniond q;       // orientation (Hamilton quaternion)
};

struct Constraint3d {
  int id_begin;                                  // pose A의 ID
  int id_end;                                    // pose B의 ID
  Pose3d t_be;                                   // A frame에서 본 B의 measurement
  Eigen::Matrix<double, 6, 6> information;       // measurement uncertainty의 inverse
};
```

이를 담는 컨테이너는 다음과 같다.

```cpp
using MapOfPoses = std::map<int, Pose3d, std::less<int>, ...>;
using VectorOfConstraints = std::vector<Constraint3d, ...>;
```

이 정도가 전부이다. 우리가 가진 정보를 한 줄로 요약하면:

* **Pose 집합**: 각 pose는 ID와 (position, quaternion) pair.
* **Constraint 집합**: 각 constraint는 두 pose ID, A frame에서 본 B의 상대 pose measurement, 그리고 그 measurement의 information matrix.

Pose graph의 정의 그대로이다. **Node = Pose, Edge = Constraint**.

여기서 한 가지 주목할 점은 `information` matrix가 6×6이라는 것이다. Pose의 자유도가 SE(3)의 6 DoF (translation 3 + rotation 3)이기 때문이다. 즉 measurement는 6차원 공간에서의 uncertainty를 갖고, residual도 6차원이 될 것이라는 힌트가 여기에 이미 들어있다. (Residual이 정확히 6차원인 이유는 4편에서 자세히 다룬다.)

---

## 두 파일의 역할 분담

`pose_graph_3d_error_term.h`와 `pose_graph_3d.cc`의 역할은 명확히 나뉜다.

| 파일 | 역할 | 핵심 entity |
|------|------|------------|
| `pose_graph_3d_error_term.h` | **수학적 residual의 정의** | `PoseGraph3dErrorTerm` functor |
| `pose_graph_3d.cc` | **Ceres problem의 구성과 solve** | `BuildOptimizationProblem()` 함수 |

`.h`는 "이 measurement가 주어졌을 때 두 pose 사이의 residual을 어떻게 계산할 것인가"라는 **수학적 정의** 그 자체만을 담고 있다. 따라서 `.h`는 Ceres와 사실 큰 상관이 없다, 그냥 Eigen으로 작성된 6차원 vector-valued function이다 (단, AutoDiff와 호환되도록 template으로 짜여있다는 점만 다르다).

`.cc`는 그 residual functor를 받아서 Ceres `Problem` 객체에 등록하고, parameterization과 gauge fixing을 처리한 뒤 solver를 돌리는 **engineering 부분**을 담고 있다.

이 분리는 Ceres의 일반적인 패턴이기도 하다. 새로운 cost를 추가하고 싶다면 `.h` 같은 곳에 functor 하나만 새로 만들고, `.cc`에서 `AddResidualBlock`만 부르면 된다. 결국에는 Ceres에서 새로운 SLAM 문제를 풀려면 우리는 **functor 하나만 잘 짜면 된다**는 뜻이고, 이게 Ceres가 SLAM 커뮤니티에서 인기있는 이유 중 하나이기도 하다.

GTSAM과 비교해보면 이 차이가 더 와닿는다. GTSAM에서 새 factor를 추가하려면 `evaluateError()`를 짜면서 Jacobian `H1`, `H2`를 직접 구해서 채워줘야 한다 (관련 내용은 [GTSAM Tutorial](https://limhyungtae.github.io/2024-12-01-GTSAM-Tutorial-1.-SLAM%EC%9D%84-%EC%9C%84%ED%95%9C-Between-Factor-%EC%89%BD%EA%B2%8C-%EC%9D%B4%ED%95%B4%ED%95%98%EA%B8%B0/)에서 자세히 다뤘다). 반면 Ceres는 ~~아묻따~~ AutoDiff가 알아서 처리해주기 때문에, 우리는 residual의 forward pass만 짜면 된다. 코드량은 줄어들지만, 그 대신 AutoDiff가 어떻게 동작하는지를 알아야 디버깅이나 성능 분석을 할 수 있다. 이 부분은 4편에서 짚을 예정이다.

---

## 예제 데이터: `sphere.g2o`

Ceres가 함께 제공하는 예제 데이터는 `sphere.g2o`인데, 약 2,500개의 pose와 ~10,000개의 constraint로 구성되어 있다. 이게 어떤 데이터냐 하면, robot이 sphere 표면을 따라 이동하면서 측정한 pose graph이다. Optimization 전에는 odometry drift 때문에 sphere가 뭉그러진 형태이지만, optimization 후에는 깔끔한 sphere가 복원되어야 한다 (g2o repository의 README나 Google 검색 첫 결과 이미지를 참고하면 어떤 모양인지 직관이 잡힌다).

이 시리즈에서는 algorithm의 원리에 집중할 것이고, visualization은 본 글의 scope를 벗어나므로 생략한다.

---

## 결론

이 글에서는 Ceres의 pose graph 3D example 두 파일이 어떻게 맞물려 돌아가는지, 즉, `main()`이 *g2o read → problem build → solve → output*의 다섯 단계 흐름을 따르고, 두 파일의 역할이 *residual의 수학 정의 (`.h`)* vs *Ceres API engineering (`.cc`)* 으로 깔끔하게 나뉘어져 있다, 를 살펴봤다.

다음 [4편](https://limhyungtae.github.io/2026-05-07-Ceres-Solver-for-Graph-SLAM-4.-PoseGraph3dErrorTerm-%EA%B9%8A%EA%B2%8C-%EC%9D%B4%ED%95%B4%ED%95%98%EA%B8%B0/)에서는 `pose_graph_3d_error_term.h`를 한 줄 한 줄 뜯어보면서, 6차원 residual이 수학적으로 정확히 무엇을 의미하는지, 그리고 `template <typename T>` 한 줄이 어떻게 AutoDiff로 Jacobian을 만들어주는지를 깊게 다룰 것이다.

---

Ceres Solver for Graph SLAM 시리즈입니다.

{% include post_links_ceres.html %}
