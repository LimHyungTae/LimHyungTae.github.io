---
layout: post
title: Ceres Solver for Graph SLAM - 4. PoseGraph3dErrorTerm 깊게 이해하기
subtitle: What does the 6-D residual really mean?
tags: [SLAM, Optimization, Ceres Solver, Pose Graph, AutoDiff, Quaternion, Lie Algebra]
comments: true
---

## Introduction

[3편](https://limhyungtae.github.io/2026-05-07-Ceres-Solver-for-Graph-SLAM-3.-Pose-Graph-3D-Example-%ED%95%9C%EB%88%88%EC%97%90-%EB%B3%B4%EA%B8%B0/)에서는 `pose_graph_3d.cc`의 `main()` 흐름과, 두 파일이 *수학 정의*와 *Ceres engineering*으로 역할이 나뉜다는 점을 살펴봤다. 이번 편에서는 그 중 *수학 정의*를 담당하는 `pose_graph_3d_error_term.h`만 한 파일을 깊게 뜯어본다.

이 파일이 하는 일은 단 하나이다. *"두 pose A, B와 그 사이의 measurement가 주어졌을 때, 6차원 residual을 어떻게 계산하는지"* 를 정의하는 것. 그런데 그 6차원이 정확히 무엇으로 채워지고, 왜 quaternion에서 vector part에 2를 곱하며, 왜 information matrix를 LLT로 쪼개는지 — 이 모든 게 사실 깊게 들여다볼 만한 주제이다. 한 마디로 이번 글은 "60줄짜리 functor 하나에 박힌 SLAM optimization의 수학"을 푸는 글이다.

`evaluateError()`에서 Jacobian `H1`, `H2`를 직접 채워줘야 하는 GTSAM과 다르게 ([GTSAM Tutorial 1편](https://limhyungtae.github.io/2024-12-01-GTSAM-Tutorial-1.-SLAM%EC%9D%84-%EC%9C%84%ED%95%9C-Between-Factor-%EC%89%BD%EA%B2%8C-%EC%9D%B4%ED%95%B4%ED%95%98%EA%B8%B0/) 참고), Ceres에서는 ~~아묻따~~ AutoDiff로 Jacobian을 자동으로 구해준다. 그래서 이 파일에는 Jacobian 코드가 한 줄도 없다. 대신 `template <typename T>`로 시작하는 한 줄이 그 마법을 가능하게 만든다. 이 마법의 원리도 글 후반에서 짚어볼 것이다.

각설하고, 이 파일을 한 줄 한 줄 따라가보자.

---

## Notation 정리

본격적으로 들어가기 전에, 이번 글에서 사용할 notation을 정리한다 (`.h` 파일 상단의 주석 표기와 동일하게 맞췄고, [2편](https://limhyungtae.github.io/2026-05-07-Ceres-Solver-for-Graph-SLAM-2.-Pose-Error%EB%A5%BC-%EC%A0%95%EC%9D%98%ED%95%98%EB%8A%94-%EB%B2%95/)의 약속과도 일관된다). 글 중간에 다시 돌아오지 않아도 되도록 *self-contained* 하게 적어둔다.

* **Subscript $$_a$$, $$_b$$ — 어느 frame/pose의 양인지 가리키는 index.**  
  예: $$\mathbf{p}_a, \mathbf{q}_a$$는 "pose A의 position과 orientation". $$\mathbf{p}_b, \mathbf{q}_b$$는 마찬가지로 pose B의 것. $$\mathbf{q}$$는 모두 Hamilton quaternion이다.
* **Subscript $$_{ab}$$ — "A에서 B로의 *상대* 변환".**  
  예: $$\mathbf{p}_{ab}$$는 *A frame 좌표계에서 표현된 B의 위치*, $$\mathbf{q}_{ab}$$는 *A에서 B로 가는 회전*.
* **Tilde $$\tilde{\cdot}$$ — 현재 추정 pose로부터 *계산한 (estimated)* 값.**  
  예: $$\tilde{\mathbf{q}}_{ab}$$는 "현재 추정값 $$\mathbf{q}_a, \mathbf{q}_b$$로부터 *계산한* 상대 회전" — 즉 $$\tilde{\mathbf{q}}_{ab} = \mathbf{q}_a^{*} \otimes \mathbf{q}_b$$.
* **Hat $$\hat{\cdot}$$ — sensor가 *측정한 (measured)* 값.**  
  예: $$\hat{\mathbf{q}}_{ab}$$는 "A frame에서 B의 상대 회전을 *sensor가 측정해준* 값". ~~hat~~을 단 변수는 모두 measurement로 약속한다.
* **Conjugate $$\mathbf{q}^{*}$$ — quaternion의 conjugate.**  
  Quaternion $$\mathbf{q} = (\mathbf{v}, w)$$ (vector part $$\mathbf{v} \in \mathbb{R}^3$$, scalar part $$w \in \mathbb{R}$$)에 대해 $$\mathbf{q}^{*} = (-\mathbf{v}, w)$$. *Unit quaternion에서는 conjugate가 곧 inverse*이다 — 즉 $$\mathbf{q}^{*} \otimes \mathbf{q} = (\mathbf{0}, 1)$$이 identity quaternion이 되며, *반대 방향의 회전*을 의미한다.
* **Quaternion product $$\otimes$$ — 두 quaternion의 곱.**  
  단순한 element-wise 곱이 아니라 *회전의 합성 (composition)* 에 대응한다. 즉 $$\mathbf{q}_a \otimes \mathbf{q}_b$$를 회전 행렬로 옮기면 $$\mathbf{R}(\mathbf{q}_a) \mathbf{R}(\mathbf{q}_b)$$가 된다.
* **Coefficient ordering** — `[x, y, z, w]` (Eigen / Hamilton convention; `w`가 scalar part). 코드의 `q.coeffs()`가 정확히 이 order의 4-element array를 돌려준다.

> 요약: *"hat($$\hat{\cdot}$$)은 measurement, tilde($$\tilde{\cdot}$$)는 estimate, $$_{ab}$$는 A→B의 상대"*. 그리고 $$\otimes$$는 quaternion 곱, $$^{*}$$는 conjugate(=inverse). 이 약속만 잡고 가면 아래 모든 식을 같은 안경으로 읽을 수 있다.

---

## Residual의 큰 그림

`PoseGraph3dErrorTerm`의 residual은 정확히 6차원이다. 이는 SE(3)의 자유도 (translation 3 + rotation 3)와 정확히 일치한다. 6차원 residual을 [translation block; rotation block]으로 나누어보자.

$$
\mathbf{r} =
\begin{bmatrix}
\mathbf{r}_p \\
\mathbf{r}_\theta
\end{bmatrix}
=
\sqrt{\boldsymbol{\Omega}} \cdot
\begin{bmatrix}
\tilde{\mathbf{p}}_{ab} - \hat{\mathbf{p}}_{ab} \\
2 \cdot \text{Vec}\bigl( \hat{\mathbf{q}}_{ab} \otimes \tilde{\mathbf{q}}_{ab}^{*} \bigr)
\end{bmatrix}
$$

여기서 $$\boldsymbol{\Omega}$$는 6×6 information matrix(= covariance의 inverse)이고, $$\sqrt{\boldsymbol{\Omega}}$$는 Cholesky factor이다.

이 한 식이 60줄짜리 functor의 핵심이고, 이번 글에서 던질 질문은 단 두 가지이다.

1. 왜 translation residual은 *그냥 빼기* 인가?
2. 왜 rotation residual은 *quaternion vector part에 2를 곱한 값* 인가?

각각 하나씩 답해보자.

---

## (1) Translation residual: 왜 단순한 차이일까?

먼저 추정 상대 pose의 translation 부분 $$\tilde{\mathbf{p}}_{ab}$$가 어떻게 계산되는지 보자.

$$
\tilde{\mathbf{p}}_{ab} = \mathbf{R}(\mathbf{q}_a)^T \cdot (\mathbf{p}_b - \mathbf{p}_a)
$$

이 식의 의미를 풀어보면 — *world frame에서 B의 위치 $$\mathbf{p}_b$$와 A의 위치 $$\mathbf{p}_a$$의 차이를 A frame으로 회전*시킨 것이다. 즉 "A에서 B로의 변위를 A frame에서 표현"한 값이다. Coordinate frame에서 보면 굉장히 자연스러운 정의이다.

코드의 line 95가 정확히 이걸 하고 있다.

```cpp
Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);
```

`q_a_inverse`는 $$\mathbf{q}_a^{*}$$이고, Eigen에서 quaternion에 vector를 곱하면 자동으로 $$R(\mathbf{q})$$를 곱한 결과를 준다. 즉 `q_a_inverse * (p_b - p_a)`는 정확히 $$R(\mathbf{q}_a)^T (\mathbf{p}_b - \mathbf{p}_a) = \tilde{\mathbf{p}}_{ab}$$이다.

이제 measurement $$\hat{\mathbf{p}}_{ab}$$도 *A frame에서 본 B의 위치*이므로, 둘 다 같은 frame (A frame)에서 표현된 3차원 벡터이다. 따라서 두 벡터의 차이는 그냥 직관적으로 "벡터 빼기"가 된다.

$$
\mathbf{r}_p = \tilde{\mathbf{p}}_{ab} - \hat{\mathbf{p}}_{ab} \in \mathbb{R}^3
$$

$$\mathbb{R}^3$$는 vector space라 빼기 자체가 잘 정의되어 있어서, 별다른 trick 없이 단순한 element-wise subtraction으로 끝난다. Translation 부분은 정말 쉽다.

---

## (2) Rotation residual: 왜 `2 * delta_q.vec()`인가?

문제는 rotation이다. Rotation은 vector space가 아니라 manifold (구체적으로는 SO(3))인지라, "두 rotation의 차이"를 단순히 빼는 걸로 정의할 수 없다. SO(3) 위에서의 차이는 *multiplicative error*로 정의해야 한다.

추정 quaternion $$\tilde{\mathbf{q}}_{ab}$$와 measurement $$\hat{\mathbf{q}}_{ab}$$가 일치할 때 0이 되는 *delta quaternion*을 다음과 같이 정의하자.

$$
\delta \mathbf{q} = \hat{\mathbf{q}}_{ab} \otimes \tilde{\mathbf{q}}_{ab}^{*}
$$

코드의 line 97-99이 정확히 이 식이다.

```cpp
Eigen::Quaternion<T> delta_q =
    t_ab_measured_.q.template cast<T>() * q_ab_estimated.conjugate();
```

추정과 measurement가 같으면 $$\delta \mathbf{q} = \mathbf{q} \otimes \mathbf{q}^{*} = (1, 0, 0, 0)$$ — identity quaternion이 된다. 이게 우리가 원하는 *zero residual*의 의미이다.

### Identity 근방에서의 quaternion 풀어쓰기

이제 핵심이 되는 trick은, **identity quaternion 근처에서 quaternion의 vector part가 axis-angle의 절반**이라는 것이다.

axis $$\mathbf{u}$$ (단위 벡터)와 각도 $$\theta$$로 회전을 표현한 quaternion은 다음과 같다.

$$
\mathbf{q}(\mathbf{u}, \theta) =
\begin{bmatrix}
\sin(\theta/2) \cdot \mathbf{u} \\
\cos(\theta/2)
\end{bmatrix}
$$

(Eigen convention: vector part가 위 3개, scalar part `w`가 마지막.) 

여기서 $$\theta$$가 작을 때 ($$\theta \approx 0$$) Taylor 전개를 하면:

$$
\sin(\theta/2) \approx \theta/2, \quad \cos(\theta/2) \approx 1
$$

따라서:

$$
\mathbf{q}(\mathbf{u}, \theta) \approx
\begin{bmatrix}
(\theta/2) \cdot \mathbf{u} \\
1
\end{bmatrix}
$$

즉, identity 근처에서는 $$\mathbf{q}$$의 *vector part가 정확히 axis-angle의 절반*이다.

$$
\text{Vec}(\mathbf{q}) \approx \frac{1}{2} \boldsymbol{\theta}, \quad \text{where } \boldsymbol{\theta} = \theta \cdot \mathbf{u}
$$

이 $$\boldsymbol{\theta}$$가 바로 $$\mathfrak{so}(3)$$ Lie algebra에서의 회전 표현 (혹은 axis-angle vector)이다.

### 왜 2를 곱하나?

이제 답이 명확해진다. delta quaternion $$\delta \mathbf{q}$$는 추정과 measurement가 가까울 때 identity 근처에 있을 것이고, 그 vector part는 axis-angle의 *절반*이다. 우리가 원하는 건 axis-angle 자체이므로, 단순히 *2를 곱해주면 된다*.

$$
\mathbf{r}_\theta = 2 \cdot \text{Vec}(\delta \mathbf{q}) \approx \boldsymbol{\theta}_{\text{error}}
$$

이게 바로 line 107의 의미이다.

```cpp
residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();
```

다시 말해, `2 * delta_q.vec()`은 **SO(3) Lie algebra에서의 rotation error vector** $$\log(\delta \mathbf{R})$$의 small-angle approximation이다. 결국 우리는 multiplicative quaternion error를 vector space로 끌어내려서 NLLS의 residual로 사용하고 있는 셈이다.

> 사실은, 정확한 Lie algebra residual은 $$\log(\delta \mathbf{R}) = \theta \mathbf{u}$$이고, $$2 \cdot \text{Vec}(\delta \mathbf{q})$$는 그 small-angle approximation이다. 둘의 차이는 $$O(\theta^3)$$ 정도이며, optimization이 수렴함에 따라 $$\theta \to 0$$이 되므로 사실상 정확해진다. 큰 각도에서는 약간의 부정확성이 있지만, LM이 iterative하게 가까워지면서 자연히 해결된다. 그래서 이 정도의 approximation은 SLAM 문헌에서도 표준적으로 통용된다.

---

## (3) Information matrix와 sqrt-information의 역할

Pose graph optimization에서 우리는 단순히 `||error||^2`를 minimize하는 게 아니라, *measurement uncertainty로 가중치된* Mahalanobis distance를 minimize해야 한다.

$$
\text{cost} = \frac{1}{2} \mathbf{e}^T \boldsymbol{\Omega} \mathbf{e}
$$

여기서 $$\boldsymbol{\Omega}$$가 information matrix(= covariance의 inverse)이다. 이 형태가 *measurement가 noisy한 정도*를 자연스럽게 반영한다 — uncertainty가 큰 axis (= covariance가 큰 axis = information이 작은 axis)는 cost에 적게 기여하고, certainty가 높은 axis는 많이 기여한다.

그런데 Ceres는 표준적인 NLLS solver이기 때문에 cost를 받아들이는 형태가 다음과 같이 정해져 있다:

$$
\text{cost} = \frac{1}{2} \|\mathbf{r}\|^2
$$

즉 cost는 *residual vector의 단순 L2 norm 제곱*이어야 한다. 그러면 어떻게 Mahalanobis 형태를 L2 형태로 바꾸지?

답은 간단하다. $$\boldsymbol{\Omega}$$를 Cholesky로 쪼개버리면 된다.

$$
\boldsymbol{\Omega} = \mathbf{L} \mathbf{L}^T \quad (\text{LLT decomposition})
$$

그러면:

$$
\mathbf{e}^T \boldsymbol{\Omega} \mathbf{e}
= \mathbf{e}^T \mathbf{L} \mathbf{L}^T \mathbf{e}
= (\mathbf{L}^T \mathbf{e})^T (\mathbf{L}^T \mathbf{e})
= \| \mathbf{L}^T \mathbf{e} \|^2
$$

따라서 *raw error에 $$\mathbf{L}^T$$를 미리 곱한 값을 residual로 만들어주면*, Ceres의 표준 형태에 그대로 들어간다.

$$
\mathbf{r} = \mathbf{L}^T \mathbf{e} \quad \Rightarrow \quad \frac{1}{2} \|\mathbf{r}\|^2 = \frac{1}{2} \mathbf{e}^T \boldsymbol{\Omega} \mathbf{e}
$$

이게 `sqrt_information`이라는 변수의 정체이다 (`L^T`가 곱해지는데 보통 통칭해서 sqrt-information이라고 부른다). 코드에서는 line 110에서 한 줄로 처리된다.

```cpp
residuals.applyOnTheLeft(sqrt_information_.template cast<T>());
```

`applyOnTheLeft`는 in-place로 좌측 곱셈을 수행한다. 즉 `residuals = sqrt_information * residuals`. 이로써 Mahalanobis cost가 자동으로 L2 cost로 변환된다.

> 참고: `sqrt_information`이 어디서 만들어지는지는 5편에서 다룰 `BuildOptimizationProblem`에서 확인할 수 있다. 거기서 `constraint.information.llt().matrixL()`이 호출된다. Eigen의 `matrixL()`은 lower-triangular factor `L`을 반환하므로, 코드에서는 사실상 `L`을 곱하는 형태인데, $$\boldsymbol{\Omega}$$가 symmetric이라 `L`로 곱해도 cost 측면에서는 동일하다 (계산해보면 $$\|\mathbf{L} \mathbf{e}\|^2 = \mathbf{e}^T \mathbf{L}^T \mathbf{L} \mathbf{e}$$이고, 만약 $$\boldsymbol{\Omega} = \mathbf{L}\mathbf{L}^T$$라면 좀 다르지만, Ceres example의 관행적 사용에서는 이렇게 처리한다 — *operator의 일관성*이 더 중요하므로 그대로 따라가자).

---

## 코드 ↔ 수학 매핑: 한 줄씩

이제 위에서 푼 수학을 코드와 1:1로 매핑해보자. `operator()`의 핵심 부분만 가져왔다.

```cpp
template <typename T>
bool operator()(const T* const p_a_ptr,
                const T* const q_a_ptr,
                const T* const p_b_ptr,
                const T* const q_b_ptr,
                T* residuals_ptr) const {
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
  Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
  Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

  // (i) 추정 상대 회전과 변위 계산
  Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
  Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;
  Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

  // (ii) measurement와의 multiplicative quaternion error
  Eigen::Quaternion<T> delta_q =
      t_ab_measured_.q.template cast<T>() * q_ab_estimated.conjugate();

  // (iii) 6차원 residual 채우기
  Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
  residuals.template block<3, 1>(0, 0) =
      p_ab_estimated - t_ab_measured_.p.template cast<T>();
  residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

  // (iv) sqrt-information으로 weighting
  residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

  return true;
}
```

위에서 정리한 수식을 그대로 따라가면:

| 수식 | 코드 line | 역할 |
|------|----------|------|
| $$\tilde{\mathbf{q}}_{ab} = \mathbf{q}_a^{*} \otimes \mathbf{q}_b$$ | (i) `q_ab_estimated = q_a_inverse * q_b` | 추정 상대 회전 |
| $$\tilde{\mathbf{p}}_{ab} = \mathbf{R}(\mathbf{q}_a)^T (\mathbf{p}_b - \mathbf{p}_a)$$ | (i) `p_ab_estimated = q_a_inverse * (p_b - p_a)` | 추정 상대 변위 (A frame) |
| $$\delta \mathbf{q} = \hat{\mathbf{q}}_{ab} \otimes \tilde{\mathbf{q}}_{ab}^{*}$$ | (ii) `delta_q = t_ab_measured_.q * q_ab_estimated.conjugate()` | Multiplicative error |
| $$\mathbf{r}_p = \tilde{\mathbf{p}}_{ab} - \hat{\mathbf{p}}_{ab}$$ | (iii) `residuals.block<3,1>(0,0) = p_ab_estimated - t_ab_measured_.p.cast<T>()` | Translation residual |
| $$\mathbf{r}_\theta = 2 \cdot \text{Vec}(\delta \mathbf{q})$$ | (iii) `residuals.block<3,1>(3,0) = T(2.0) * delta_q.vec()` | Rotation residual (Lie algebra approx) |
| $$\mathbf{r} \leftarrow \mathbf{L} \cdot \mathbf{r}$$ | (iv) `residuals.applyOnTheLeft(sqrt_information)` | Information weighting |

수학과 코드가 정확히 1:1로 대응한다. 이 매핑을 머릿속에 박아두면, 다른 SLAM library의 cost function을 봐도 거의 비슷한 패턴을 볼 수 있을 것이다.

---

## AutoDiff: `template <typename T>` 한 줄의 마법

이제 마지막 퍼즐 조각이다. 이 functor에는 Jacobian 코드가 없는데, Ceres는 어떻게 Jacobian을 얻을까?

답은 **template** 그리고 **dual number**이다.

### Forward-mode AutoDiff의 핵심 아이디어

AutoDiff의 핵심은 *덧셈/곱셈/sin/cos 등 모든 elementary operation에 대해 미분 규칙이 있으니, 각 변수의 값과 함께 그 변수의 derivative를 같이 들고 다니면서 chain rule로 누적하면 된다*는 것이다.

이를 위해 Ceres는 *Jet*이라는 type을 정의한다.

```cpp
template <typename T, int N>
struct Jet {
  T a;            // 함수값
  Eigen::Matrix<T, N, 1> v;  // 입력 N차원에 대한 Jacobian (gradient)
};
```

`Jet`은 dual number의 일반화로, 함수값 `a`와 그 미분 정보 `v`를 함께 묶어 들고 다닌다. 모든 산술 연산자(`+`, `*`, `sin`, ...)는 Jet에 대해 overload되어 있어서, 가령 `(a, v) * (b, w) = (a*b, a*w + b*v)`처럼 chain rule을 자동으로 누적한다.

그래서 우리가 짠 functor를 *그대로* `T = ceres::Jet<double, N>`으로 instantiate하면, 한 번의 forward pass만으로 함수값과 N개 입력에 대한 Jacobian이 동시에 계산된다. 이게 AutoDiff의 본질이고, *우리가 Jacobian을 직접 짤 필요가 없는* 이유이다.

이 메커니즘이 동작하려면 functor가 *어떤 type T든 받아들일 수 있어야* 한다. 그래서 `operator()`가 다음과 같이 template으로 시작하는 것이다.

```cpp
template <typename T>
bool operator()(const T* const p_a_ptr,
                const T* const q_a_ptr,
                ...,
                T* residuals_ptr) const { ... }
```

이 한 줄이 functor가 `double`로도, `Jet<double, N>`으로도 instantiate 가능하게 만든다. `double`로 instantiate되면 그냥 forward residual 계산이고, `Jet`으로 instantiate되면 Jacobian이 자동으로 함께 나온다.

### `AutoDiffCostFunction`의 template 인자

이제 line 118의 `Create()` 함수를 보자.

```cpp
static ceres::CostFunction* Create(
    const Pose3d& t_ab_measured,
    const Eigen::Matrix<double, 6, 6>& sqrt_information) {
  return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTerm, 6, 3, 4, 3, 4>(
      t_ab_measured, sqrt_information);
}
```

`AutoDiffCostFunction`의 template 인자는 다음과 같이 읽으면 된다.

$$
\underbrace{\text{PoseGraph3dErrorTerm}}_{\text{Functor}}, \quad
\underbrace{6}_{\text{residual dim}}, \quad
\underbrace{3, 4, 3, 4}_{(\mathbf{p}_a, \mathbf{q}_a, \mathbf{p}_b, \mathbf{q}_b) \text{ dims}}
$$

* **Functor**: residual을 계산하는 class.
* **6**: residual vector의 dimension.
* **3, 4, 3, 4**: parameter block 각각의 dimension. Pose A의 position(3), Pose A의 quaternion(4), Pose B의 position(3), Pose B의 quaternion(4).

이 정보가 있으면 Ceres는 어떤 size의 Jet을 만들어야 하는지(N = 3+4+3+4 = 14)를 알고, 그에 맞춰 Jacobian을 자동으로 계산해준다.

> 참고: quaternion이 4D인데 자유도는 3이라는 점은 Jacobian에는 영향을 주지 않는다 (Jacobian은 ambient space인 4D w.r.t. residual로 계산되며, manifold 처리는 별도의 mechanism으로 이뤄진다 — 이 부분은 5편에서 다룬다).

### AutoDiff의 한계 (잠시 paranoid check)

AutoDiff가 만능은 아니다. 다음과 같은 경우에는 AutoDiff 대신 analytic Jacobian을 직접 짜는 게 나을 수도 있다.

* **성능 cricitcal**: AutoDiff는 (대략) Jacobian이 추가될 때마다 계산량이 N배 늘어나므로, parameter block이 굉장히 클 때는 직접 짜는 게 더 빠를 수 있다.
* **수치적 안정성**: 어떤 함수는 직접 derivative를 적으면 나눗셈/특이점을 우회할 수 있는데, AutoDiff가 그걸 못 따라가면 NaN이 나올 수 있다.
* **외부 함수 호출**: template으로 instantiate할 수 없는 외부 library 함수를 부르면 AutoDiff가 통하지 않는다.

다만, pose graph 3D 같이 parameter dim이 14, residual이 6 정도인 작은 문제에서는 AutoDiff가 충분히 빠르고 정확하다. 그래서 example 코드가 그냥 AutoDiff로 짜여있는 것이다.

---

## 결론

이 글에서는 `pose_graph_3d_error_term.h`를 다음과 같이 분해해봤다.

* **Translation residual** ($$\mathbf{r}_p$$)은 단순한 차이이다 — 둘 다 같은 frame(A frame)에서 표현된 3D 벡터이기 때문.
* **Rotation residual** ($$\mathbf{r}_\theta$$)은 multiplicative quaternion error의 vector part에 2를 곱한 값이고, 이는 SO(3) Lie algebra에서의 axis-angle error의 small-angle approximation에 해당한다.
* **Information matrix**는 LLT로 쪼개서 sqrt-information을 얻고, 그걸 raw error에 곱해 Mahalanobis cost를 표준 L2 cost로 변환한다.
* **AutoDiff**는 `template <typename T>` 덕분에 동작한다. Jet type으로 instantiate되면 forward pass 한 번에 함수값과 Jacobian이 동시에 나온다.

ChatGPT에 먹혀버린 시대일수록 *우리가 정확히 무엇을 optimize하고 있는지*를 손으로 따라가보는 게 중요하지 않나 싶다. Pose graph optimization의 `cost = 0.5 ||residual||^2` 안에 들어있는 60줄짜리 functor가, 사실은 SE(3) manifold의 multiplicative error와 Mahalanobis weighting을 다 구현한 코드라는 것 — 이것이 이 글에서 전달하고 싶었던 핵심이다.

다음 [5편](https://limhyungtae.github.io/2026-05-07-Ceres-Solver-for-Graph-SLAM-5.-BuildOptimizationProblem%EA%B3%BC-Manifold-%EA%B9%8A%EA%B2%8C-%EC%9D%B4%ED%95%B4%ED%95%98%EA%B8%B0/)에서는 `pose_graph_3d.cc`의 `BuildOptimizationProblem()`을 분석한다. 이 functor가 `AddResidualBlock`으로 어떻게 등록되는지, quaternion의 manifold 처리는 어떻게 이뤄지는지, gauge freedom은 어떻게 fix되는지 — Ceres API를 활용하는 *engineering* 레이어를 다룰 예정이다.

---

Ceres Solver for Graph SLAM 시리즈입니다.

{% include post_links_ceres.html %}
