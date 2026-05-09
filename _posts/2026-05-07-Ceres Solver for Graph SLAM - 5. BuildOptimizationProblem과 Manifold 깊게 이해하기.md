---
layout: post
title: Ceres Solver for Graph SLAM - 5. BuildOptimizationProblem과 Manifold 깊게 이해하기
subtitle: How Ceres glues your residual into a real optimizer
tags: [SLAM, Optimization, Ceres Solver, Pose Graph, Manifold, Quaternion]
comments: true
description: Ceres pose_graph_3d.cc의 BuildOptimizationProblem을 분석한다. AddResidualBlock, EigenQuaternionManifold, SetParameterBlockConstant로 quaternion 자유도와 gauge freedom을 어떻게 처리하는지 다룬다.
permalink: /2026/05/07/ceres-graph-slam-05-optimization-problem-manifold/
redirect_from:
  - '/2026-05-07-Ceres Solver for Graph SLAM - 5. BuildOptimizationProblem과 Manifold 깊게 이해하기/'
---

## Introduction

[3편](https://limhyungtae.github.io/2026/05/07/ceres-graph-slam-03-pose-graph-3d-overview/)에서 두 파일의 역할 분담을 잡았고, [4편](https://limhyungtae.github.io/2026/05/07/ceres-graph-slam-04-posegraph3d-error-term/)에서는 `.h` 파일의 residual 수학을 깊게 봤다. 이번 5편에서는 `.cc` 쪽, 즉 `BuildOptimizationProblem()`을 다룬다. 이 함수가 사실상 *우리가 정의한 residual functor를 어떻게 Ceres `Problem` 객체에 끼워넣어 실제 optimizer가 돌아가게 만드는지*를 담고 있다.

언뜻 보면 이 함수는 단순히 *for loop으로 constraint마다 residual block을 추가하는 게 전부*인 것 같지만, 안을 들여다보면 4가지 디테일이 모두 들어있다.

1. Information matrix를 Cholesky로 쪼개 sqrt-information을 만드는 것
2. 한 residual을 *4개의 parameter block*에 동시에 매다는 것
3. Quaternion이 4차원이지만 자유도는 3이라는 사실을 Ceres에 알려주기 위한 **Manifold** 등록
4. Pose graph의 gauge freedom을 없애기 위한 **첫 pose 고정**

이 중 (3)이 SLAM optimization에서 가장 자주 헷갈리는 부분이라, 이번 글의 절반 정도는 manifold 이야기에 할애할 것이다. 각설하고 출발해보자.

---

## 한눈에 보기: `BuildOptimizationProblem()`의 4단계

전체 코드는 다음과 같다 (CHECK문과 logging은 가독성을 위해 생략).

```cpp
void BuildOptimizationProblem(const VectorOfConstraints& constraints,
                              MapOfPoses* poses,
                              ceres::Problem* problem) {
  ceres::LossFunction* loss_function = nullptr;
  ceres::Manifold* quaternion_manifold = new EigenQuaternionManifold;

  for (const auto& constraint : constraints) {
    auto pose_begin_iter = poses->find(constraint.id_begin);
    auto pose_end_iter   = poses->find(constraint.id_end);

    // (1) Information matrix를 LLT로 쪼개기
    const Eigen::Matrix<double, 6, 6> sqrt_information =
        constraint.information.llt().matrixL();

    // (2) Cost function 만들기
    ceres::CostFunction* cost_function =
        PoseGraph3dErrorTerm::Create(constraint.t_be, sqrt_information);

    // (3) AddResidualBlock, 4개의 parameter block에 한 번에 매기
    problem->AddResidualBlock(cost_function,
                              loss_function,
                              pose_begin_iter->second.p.data(),
                              pose_begin_iter->second.q.coeffs().data(),
                              pose_end_iter->second.p.data(),
                              pose_end_iter->second.q.coeffs().data());

    // (4) Quaternion manifold 등록
    problem->SetManifold(pose_begin_iter->second.q.coeffs().data(),
                         quaternion_manifold);
    problem->SetManifold(pose_end_iter->second.q.coeffs().data(),
                         quaternion_manifold);
  }

  // (5) Gauge freedom 처리, 첫 pose 고정
  auto pose_start_iter = poses->begin();
  problem->SetParameterBlockConstant(pose_start_iter->second.p.data());
  problem->SetParameterBlockConstant(pose_start_iter->second.q.coeffs().data());
}
```

이 함수가 하는 일을 4단계로 요약하면:

1. **Cost function 생성**: 각 constraint마다 residual functor를 wrapping한 `AutoDiffCostFunction`을 만든다. 이때 information matrix를 LLT로 쪼개 sqrt-information으로 변환한다.
2. **Residual block 등록**: 만든 cost function을 두 pose의 4개 parameter block (각 pose의 p와 q)에 매단다.
3. **Manifold 등록**: 각 quaternion parameter block에 `EigenQuaternionManifold`를 붙인다.
4. **Gauge fix**: `poses->begin()`이 가리키는 첫 pose의 p와 q를 둘 다 constant로 만든다.

각 단계의 의미를 하나씩 짚어보자.

---

## (1) `LLT` decomposition: information matrix를 sqrt로

Constraint마다 들어있는 `information` 멤버는 6×6 information matrix $$\boldsymbol{\Omega}$$ (= covariance의 inverse)이다. 그런데 4편에서 살펴봤듯, Ceres는 cost를 *L2 norm 제곱* 형태로만 받아들이기 때문에 Mahalanobis form은 미리 변환해주어야 한다. 이게 line 78-79이다.

```cpp
const Eigen::Matrix<double, 6, 6> sqrt_information =
    constraint.information.llt().matrixL();
```

`llt()`는 Eigen의 Cholesky decomposition을 수행한다. 결과는 lower triangular matrix $$\mathbf{L}$$로, 다음을 만족한다.

$$
\boldsymbol{\Omega} = \mathbf{L} \mathbf{L}^T
$$

이 $$\mathbf{L}$$이 `sqrt_information`이고, 이게 functor의 `applyOnTheLeft`로 전달되어 raw error를 곱하면 cost가 자동으로 Mahalanobis 형태가 된다 (이 부분의 수학은 4편에서 다뤘다).

> 개인적으로는 *`sqrt`라는 이름이 헷갈린다*고 생각한다. 행렬의 "square root"는 정의가 여러 가지인데(LLT의 `L`, eigen-decomposition 기반 등), Ceres example에서는 `LLT`의 `L` 자체를 sqrt-information이라고 부른다. 즉 strict한 의미의 matrix square root는 아니다. 이런 부분은 익숙해지는 수밖에 없다.

---

## (2) `AddResidualBlock`: 한 residual을 4개의 block에 매기

이제 핵심이다. Ceres에서는 *parameter*가 *block 단위*로 관리된다. 즉, optimization 변수를 한 덩어리씩 묶어 다루며, 각 block은 raw `double*` pointer로 식별된다.

Pose 하나는 (position, quaternion) = (3D, 4D) 두 vector로 나뉘므로, 한 pose가 자연스럽게 *2개의 parameter block*을 차지한다. Pose graph constraint 하나는 두 개의 pose에 의존하므로, 한 residual block은 *4개의 parameter block*에 동시에 매달리게 된다.

```cpp
problem->AddResidualBlock(cost_function,
                          loss_function,
                          pose_begin_iter->second.p.data(),         // p_a (3-D)
                          pose_begin_iter->second.q.coeffs().data(),// q_a (4-D)
                          pose_end_iter->second.p.data(),           // p_b (3-D)
                          pose_end_iter->second.q.coeffs().data()); // q_b (4-D)
```

각 인자를 차례로 보면:

* `cost_function`: 4편에서 만든 `AutoDiffCostFunction<PoseGraph3dErrorTerm, 6, 3, 4, 3, 4>`. 6차원 residual을 내고, 4개의 parameter block에 의존한다.
* `loss_function`: 여기서는 `nullptr`. *outlier에 대한 robustification*을 안 한다는 뜻, pure L2 cost를 그대로 쓴다는 의미이다. Outlier가 있을 수 있는 실제 SLAM에서는 `new ceres::HuberLoss(1.0)` 같이 robust loss를 넣어준다 (Huber, Cauchy 등은 Ceres가 기본 제공).
* `p_a.data()`, `q_a.coeffs().data()`, ...: 각 parameter block의 raw pointer. 여기서 중요한 건 **Ceres가 이 pointer가 가리키는 메모리를 *직접 in-place로 수정한다*** 는 점이다. 즉 optimization이 끝나고 나면 `poses` map의 각 `Pose3d`가 자동으로 optimized 값으로 갱신되어 있다. `MapOfPoses`를 그대로 들고 있으면 결과가 자동으로 반영되는 구조이다.

> 사실 Ceres가 in-place로 동작하는 이 부분이 처음에는 살짝 위험해 보일 수 있다. 그렇지만 이 디자인 덕분에 "원본 데이터 → solver"가 별도의 복사 없이 바로 연결되어 매우 효율적이다. 결국 우리가 신경쓸 것은 *parameter block의 lifetime이 `Solve()` 호출까지 살아있도록 보장하는 것* 뿐이다.

`p.data()`는 Eigen `Vector3d`의 내부 raw pointer를 그대로 노출하고, `q.coeffs().data()`는 `[x, y, z, w]` 4개의 raw pointer를 노출한다. 이 4개 데이터 array가 Ceres가 직접 update할 메모리이다.

---

## (3) Quaternion Manifold: 왜 필요한가, 어떻게 동작하는가

이 글의 핵심이다. Quaternion은 4D인데 회전의 자유도는 3이다. 이 사실을 Ceres에 *명시적으로 알려주지 않으면* optimization은 잘못된 방향으로 흘러간다.

### 왜 naïve update가 깨지는가

Ceres의 LM은 매 iteration마다 normal equation을 풀어 *update vector* $$\Delta \mathbf{x}$$를 얻은 뒤, parameter를 다음과 같이 갱신한다.

$$
\mathbf{x}_{\text{new}} = \mathbf{x}_{\text{old}} + \Delta \mathbf{x} \quad (\text{additive update})
$$

이 update가 Eucliden vector space에서는 잘 동작한다. 하지만 quaternion에 대해 단순 덧셈을 하면 다음 두 가지 문제가 생긴다.

1. **Unit norm이 깨진다**: $$\mathbf{q} + \Delta\mathbf{q}$$는 일반적으로 $$\|\cdot\| = 1$$이 아니다. 그러면 더 이상 회전을 표현하는 unit quaternion이 아니게 된다.
2. **자유도 mismatch**: $$\Delta \mathbf{q}$$가 4D에서 자유롭게 움직이게 되면, 실제 회전 자유도(3D) 외에 *rotation에 영향을 안 주는 방향* (norm을 늘리거나 줄이는 방향)으로도 update가 일어난다. 그러면 Hessian이 rank-deficient해지고, optimization이 불안정해진다.

이 두 문제를 *동시에* 해결하는 방법이 바로 **manifold**이다.

### Manifold의 두 함수: Plus와 Minus

Ceres의 `Manifold` interface는 정확히 두 가지 연산을 정의한다 (이름은 단순화).

* `Plus(x, delta) → x_new`: tangent space의 update vector $$\boldsymbol{\delta}$$를 받아, manifold 위의 새 point $$\mathbf{x}_{\text{new}}$$를 반환.
* `Minus(y, x) → delta`: manifold 위 두 point의 차이를 tangent space vector로 표현.

여기서 *tangent space의 차원*과 *ambient space의 차원*이 다를 수 있다. Quaternion의 경우, ambient space는 4D ($$\mathbb{R}^4$$), tangent space는 3D ($$\mathfrak{so}(3)$$의 axis-angle vector)이다.

`EigenQuaternionManifold`의 `Plus` 연산은 다음을 수행한다.

$$
\mathbf{q}_{\text{new}} = \mathbf{q} \otimes \exp\bigl( \boldsymbol{\delta} \bigr)
$$

* $$\boldsymbol{\delta} \in \mathbb{R}^3$$: tangent space vector (axis-angle 형태의 small rotation)
* $$\exp(\boldsymbol{\delta})$$: $$\mathfrak{so}(3) \to S^3$$의 exponential map (unit quaternion 반환)
* $$\otimes$$: quaternion 곱

이 연산은 다음 두 가지 성질을 자동으로 만족시킨다.

1. **Unit norm 보존**: $$\exp(\boldsymbol{\delta})$$가 unit quaternion이고, unit quaternion의 곱은 다시 unit quaternion이다.
2. **자유도 정확**: $$\boldsymbol{\delta}$$가 3차원이므로 update space가 정확히 회전의 자유도와 같다.

### `SetManifold()` 호출

이 manifold를 Ceres에 등록하는 게 line 90-93이다.

```cpp
problem->SetManifold(pose_begin_iter->second.q.coeffs().data(),
                     quaternion_manifold);
problem->SetManifold(pose_end_iter->second.q.coeffs().data(),
                     quaternion_manifold);
```

이 호출 이후 Ceres는 quaternion parameter block에 대해 다음과 같이 동작한다.

* Solver가 update를 계산할 때는 *3차원 vector* $$\boldsymbol{\delta}$$로 계산한다 (4차원 `q`가 아니라).
* Update를 적용할 때는 `Plus(q, delta)`를 호출해서 manifold-aware하게 갱신한다.
* Hessian의 dimension도 자동으로 *tangent space dimension(3)*으로 맞춰진다.

결국 Ceres는 *내부적으로는 3차원에서 optimization*을 하면서, *외부적으로는 4차원 quaternion을 노출*하는 셈이다. 이게 manifold의 진가이다.

> Ceres 2.1 이전 버전에는 `LocalParameterization`이라는 비슷한 interface가 있었다. 새 `Manifold` API는 그것의 일반화/refactoring 결과로 등장했고, 동작 원리는 거의 동일하다. 옛 코드에서 `EigenQuaternionParameterization`을 썼다면 동일한 역할이다, 인터페이스만 정리되었다고 보면 된다.

### 한 manifold 객체를 두 block에 공유

코드에서 눈여겨볼 작은 디테일은, `quaternion_manifold` 객체를 한 번만 만들어 *모든* quaternion parameter block에 공유한다는 점이다.

```cpp
ceres::Manifold* quaternion_manifold = new EigenQuaternionManifold;
// ... for loop 안에서 매 iteration ...
problem->SetManifold(pose_begin_iter->second.q.coeffs().data(),
                     quaternion_manifold);
problem->SetManifold(pose_end_iter->second.q.coeffs().data(),
                     quaternion_manifold);
```

`EigenQuaternionManifold`는 stateless하기 때문에 여러 block이 공유해도 문제가 없다. Ceres는 ownership 규칙에 따라 manifold pointer를 알아서 관리한다 (default로 problem이 ownership을 가져감, `Problem::Options`로 변경 가능).

---

## (4) Gauge freedom과 `SetParameterBlockConstant`

Pose graph optimization은 **6 DoF의 gauge freedom**을 가진다. 이게 무슨 말이냐 하면, 우리가 추정한 모든 pose에 동일한 rigid body transformation $$\mathbf{T}$$를 곱해도 *cost가 정확히 같다*는 것이다.

$$
\mathbf{x}_i' = \mathbf{T} \cdot \mathbf{x}_i \quad \forall i \implies \text{cost}(\{\mathbf{x}'_i\}) = \text{cost}(\{\mathbf{x}_i\})
$$

직관적으로도 자명하다, pose graph의 cost는 *상대 pose* $$\tilde{\mathbf{T}}_{ab} = \mathbf{x}_a^{-1} \mathbf{x}_b$$에만 의존하는데, 모든 pose에 같은 $$\mathbf{T}$$를 곱하면:

$$
\mathbf{x}_a'^{-1} \mathbf{x}_b' = (\mathbf{T} \mathbf{x}_a)^{-1} (\mathbf{T} \mathbf{x}_b) = \mathbf{x}_a^{-1} \mathbf{x}_b
$$

즉 상대 pose는 그대로이다. 그래서 cost가 안 바뀐다. 이게 SE(3)의 6 DoF (translation 3 + rotation 3) 만큼의 자유도가 cost에 영향을 안 준다는 의미이고, *Hessian이 그만큼 rank-deficient*하다는 의미이다.

Rank-deficient Hessian은 normal equation을 풀 수 없다 (혹은 풀려도 update가 그 null space 방향으로 자유롭게 움직인다). LM은 internal damping($$\lambda \mathbf{I}$$)을 더해 mitigate하지만, *명시적으로 제거하는 게 깔끔하다*. 그 방법이 다음과 같이 *어느 한 pose를 fix*하는 것이다.

```cpp
auto pose_start_iter = poses->begin();
problem->SetParameterBlockConstant(pose_start_iter->second.p.data());
problem->SetParameterBlockConstant(pose_start_iter->second.q.coeffs().data());
```

* `pose_start_iter` = `poses->begin()`: ID가 가장 작은 pose (보통 ID 0).
* `SetParameterBlockConstant(ptr)`: 해당 parameter block을 optimization 변수에서 제외 (값이 그대로 유지됨).
* `p`와 `q.coeffs()`를 둘 다 fix해야 *6 DoF 모두*를 묶는 효과가 난다, translation 3 + rotation 3.

이렇게 하면 첫 pose가 *anchor* 역할을 하고, 나머지 모든 pose는 그 anchor에 *상대적으로* 결정된다. Hessian은 rank가 정확히 채워지고, optimization은 깔끔하게 수렴한다.

> 실제 SLAM system에서는 종종 첫 pose를 fix하는 대신 *prior factor* (e.g., GPS prior, IMU 초기 자세 prior)를 추가해서 gauge를 깨기도 한다. 두 방법 다 결국엔 6 DoF 자유도를 어떻게 채울 것인지의 선택이다, Ceres example에서는 simplicity를 위해 hard fix를 택한 것이다.

---

## SolveOptimizationProblem: 마무리

`BuildOptimizationProblem`에서 만든 `Problem`은 이제 `SolveOptimizationProblem`이 받아서 푼다.

```cpp
bool SolveOptimizationProblem(ceres::Problem* problem) {
  ceres::Solver::Options options;
  options.max_num_iterations = 200;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);
  std::cout << summary.FullReport() << '\n';

  return summary.IsSolutionUsable();
}
```

* `max_num_iterations = 200`: 최대 200번의 LM iteration.
* `SPARSE_NORMAL_CHOLESKY`: pose graph는 Hessian이 *sparse*하다. 한 residual block이 4개의 parameter block에만 의존하기 때문에, 전체 Hessian의 non-zero pattern이 graph의 connectivity와 같다. Sparse Cholesky가 이런 구조에서 가장 빠르다.
* `summary.FullReport()`: 사람이 읽기 좋은 진단 정보 (iteration 수, 각 step의 cost, total time 등).
* `IsSolutionUsable()`: solve가 NaN/Inf 등으로 실패하지 않았는지 체크.

Solver type 선택은 graph 크기에 따라 다른데, sphere.g2o처럼 수천 개 정도의 pose에서는 SPARSE_NORMAL_CHOLESKY가 거의 항상 좋은 선택이다. 더 큰 문제에서는 `ITERATIVE_SCHUR`나 다른 옵션도 고려할 수 있지만, 그건 이 글의 scope를 벗어나므로 생략한다.

---

## 마무리: 두 파일의 분담 다시 보기

이제 sub-series의 마지막이니, 처음 3편에서 그렸던 큰 그림을 다시 한 번 환기해본다.

| 파일 | 역할 | 4편/5편 정리 |
|------|------|-------------|
| `pose_graph_3d_error_term.h` | residual의 *수학적 정의* | 4편: 6차원 residual = (translation diff, $$2 \cdot \text{Vec}(\delta \mathbf{q})$$); LLT-derived sqrt-information으로 Mahalanobis 변환; AutoDiff로 Jacobian 자동 |
| `pose_graph_3d.cc` | residual을 *Ceres에 끼워넣기* | 5편: 4개 block에 AddResidualBlock; quaternion에 Manifold 등록; 첫 pose는 SetParameterBlockConstant로 gauge fix |

각 파일이 200줄도 안 되지만, 그 안에 다음과 같이 SLAM optimization의 핵심 idea가 모두 녹아 있다.

* Manifold-aware optimization (quaternion의 4D vs 3D 처리)
* Multi-block parameter (한 residual이 여러 변수에 의존)
* Mahalanobis cost를 L2 cost로 변환 (sqrt-information)
* Multiplicative quaternion error로 SO(3) Lie algebra residual
* AutoDiff로 Jacobian 자동 계산
* Gauge freedom 처리

---

## 결론

이 sub-series에서는 Ceres의 pose graph 3D example을 두 파일에 걸쳐 깊게 분석했다. 다음을 강조하며 마무리하고 싶다.

> *우리가 매일 쓰는 60줄짜리 functor 안에는 Lie algebra, Mahalanobis distance, AutoDiff, manifold optimization이 모두 녹아 있다.*

ChatGPT나 Cursor가 코드를 잘 짜주는 시대일수록, 이런 짧은 코드를 *line-by-line으로 따라가며 그 뒤의 수학과 idea를 정확히 이해하는 능력*이 오히려 더 중요해지는 게 아닐까 싶다. 이 시리즈가 SLAM 입문자에게 그 한 걸음을 떼게 해주었으면 하는 바람이다.

마지막으로, 이 example에서 나온 idea를 가지고 다음과 같이 응용을 시도해보면 좋을 것이다.

* 같은 구조로 *2D pose graph* (`pose_graph_2d`)도 분석해보기, Ceres example에 이미 있음
* GPS prior factor를 추가해 SetParameterBlockConstant 없이 gauge를 깨보기
* HuberLoss를 넣어 outlier-robust pose graph를 만들어보기

언젠가 시간이 되면 위 응용들도 글로 정리해보고 싶다 ^^7

---

Ceres Solver for Graph SLAM 시리즈입니다.

{% include post_links_ceres.html %}
