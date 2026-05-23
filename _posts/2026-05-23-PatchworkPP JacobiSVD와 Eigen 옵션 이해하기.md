---
layout: post
title: Eigen JacobiSVD와 Decomposition Option 이해하기
subtitle: ComputeFullU, ComputeThinU, SelfAdjointEigenSolver 중 무엇을 써야 할까
tags: [Patchwork++, Eigen, JacobiSVD, SelfAdjointEigenSolver, C++, SLAM, Code Reading]
comments: true
description: Patchwork++의 plane fitting 코드에서 Eigen::JacobiSVD와 ComputeFullU 옵션이 어떤 의미인지 정리한다. 3x3 covariance matrix에는 SelfAdjointEigenSolver가 왜 더 적합한지, singular value와 eigenvalue의 관계를 코드 기준으로 설명한다.
permalink: /2026/05/23/patchworkpp-jacobisvd-eigen-options/
---

## 들어가며

요즘 [Patchwork++](https://github.com/url-kaist/patchwork-plusplus) 코드를 리팩토링하고 있고,
최근에는 `pip install pypatchworkpp`가 되게끔 코드도 업데이트했다.
그러던 와중 Claude/Codex가 없던 시절에 짰던 plane fitting 코드의 `Eigen::JacobiSVD`가 눈에 들어왔다.

Patchwork++의 `estimate_plane()`에는 대략 이런 코드가 있다.

```cpp
Eigen::MatrixX3f centered = eigen_ground.rowwise() - eigen_ground.colwise().mean();
Eigen::MatrixX3f cov =
    (centered.adjoint() * centered) / static_cast<double>(eigen_ground.rows() - 1);

Eigen::JacobiSVD<Eigen::MatrixX3f> svd(
    cov,
    Eigen::DecompositionOptions::ComputeFullU
);
singular_values_ = svd.singularValues();
normal_ = svd.matrixU().col(2);
```


예전부터 궁금했던 것인데, 이 부분을 좀 더 공부해서 개선할 순 없을까 궁금증이 들었다.

이 글은 크게 4개의 흐름으로 읽으면 된다.

- **Preliminaries: Plane Fitting With Covariance**
- **Solver Choice: option보다 solver family**
  `ComputeFullU` vs. `ComputeThinU`, `JacobiSVD` vs. `BDCSVD`, vs. `SelfAdjointEigenSolver::computeDirect()`를 비교해서 어디에 성능 개선 여지가 있는지 본다.
- **Migration Convention: scale, order, normal sign**
  solver를 바꿀 때 singular value scale, eigenvalue 정렬 순서, normal 방향 convention을 어떻게 보존할지 정리한다.
- **Experimental Results: Patchwork++에서는 무엇을 바꿀까**
  실제로 어떤 변경이 합리적인지, 성능 기대치는 어느 정도인지, 변경 전에 무엇을 체크해야 하는지 정리한다.

---

## Preliminaries: Plane Fitting With Covariance

Patchwork++는 patch 안의 point들을 모아 plane을 fitting한다.
흐름은 다음과 같다.

```text
patch point cloud
  -> mean 계산
  -> centered point cloud 생성
  -> covariance matrix cov = X^T X / (N - 1)
  -> covariance의 principal directions 계산
  -> 가장 작은 방향을 plane normal로 사용
```

여기서 중요한 점은 `svd`에 들어가는 입력이 원본 point matrix가 아니라는 것이다.
입력은 이미 다음 형태의 covariance matrix이다.

```text
cov: 3 x 3 symmetric PSD matrix
```

covariance matrix는 patch point들이 중심점, 즉 mean point를 기준으로 각 방향으로 얼마나 퍼져 있는지를 나타낸다.
PSD는 positive semi-definite라는 뜻이다.
즉 모든 eigenvalue가 0 이상인 symmetric matrix이다.
covariance matrix는 수치 오차를 제외하면 이 조건을 만족한다.

Plane fitting에서는 covariance의 방향들이 중요하다.

- 가장 큰 방향: patch 안에서 point들이 가장 길게 퍼진 방향.
- 두 번째 방향: plane 위에서 그다음으로 퍼진 방향.
- 가장 작은 방향: point들이 거의 퍼지지 않는 방향.

평면 point cloud라면 가장 작은 방향이 plane normal이다.
그래서 기존 코드는 `svd.matrixU().col(2)`를 normal로 쓴다.
Eigen의 SVD singular values는 큰 값에서 작은 값 순서로 정렬되기 때문에,
`U.col(2)`가 가장 작은 singular value에 대응하는 방향이다.

---

## Solver Choice: Option보다 Solver Family

먼저 현재 코드에서 Eigen option이 실제로 무엇을 의미하는지 확인하고,
그다음 SVD solver family를 바꾸는 것이 의미 있는지 본다.
마지막으로 covariance의 symmetric PSD 구조를 직접 쓰는 eigensolver가 왜 더 자연스러운지 연결해 보자.

### Decomposition Option: ComputeFullU vs ComputeThinU

`JacobiSVD`는 기본적으로 singular value만 계산한다.
singular vector가 필요하면 option으로 U 또는 V를 계산하라고 명시해야 한다.

SVD는 matrix $$A$$를 다음처럼 세 부분으로 나누어 보는 방법이다.

$$
A = U \Sigma V^T
$$

여기서 `ComputeFullU`는 full-size U matrix를 계산하라는 뜻이다.
Patchwork++에서는 normal을 얻기 위해 `svd.matrixU().col(2)`를 읽으므로,
U는 필요하다.

반대로 V는 쓰지 않는다.
따라서 `ComputeFullV`나 `ComputeThinV`를 켜면 불필요한 계산이다.
현재 코드가 `ComputeFullU`만 켠 것은 이 점에서는 괜찮다.

Q. 그럼 `ComputeThinU`를 쓰면 더 빠를까?
이 코드에서는 거의 의미가 없다.

`ComputeThinU`는 rectangular matrix에서 중요하다.
예를 들어 입력이 `1000 x 3`이면 full U는 `1000 x 1000`이고 thin U는 `1000 x 3`이라 차이가 크다.
하지만 Patchwork++에서 SVD에 들어가는 것은 이미 `3 x 3` covariance matrix이다.
수학적으로 필요한 singular vector도 세 개뿐이므로, thin U로 바꿔 얻을 수 있는 구조적 이득이 없다.
즉 이 변경은 performance refactor의 핵심이 아니다.

오히려 fixed-size `Eigen::Matrix3f`에서는 Eigen version에 따라 `ComputeThinU`가 기대와 다르게 동작하거나 assert에 걸릴 수 있으므로,
Patchwork++처럼 3x3 covariance를 분해하는 코드에서는 `ComputeFullU`를 그대로 두는 편이 안전하다.
`ComputeFullU`가 문제라기보다, 일반 SVD solver를 쓰는 것 자체가 더 큰 비용이다.

즉 이 코드에서 `ComputeFullU` 자체가 큰 낭비라기보다는,
**3x3 symmetric covariance에 JacobiSVD를 쓰는 것 자체가 더 큰 이슈**이다.

### SVD Solver Family: JacobiSVD vs BDCSVD

Eigen에서 SVD라고 해도 선택지가 하나만 있는 것은 아니다.
Patchwork++의 plane fitting 관점에서 보면 다음처럼 나눠볼 수 있다.

#### JacobiSVD

`JacobiSVD`는 two-sided Jacobi rotation 기반 SVD이다.
일반 matrix에도 안정적으로 쓸 수 있고, Eigen 문서도 reliability와 accuracy를 장점으로 설명한다.
대신 큰 matrix에서는 bidiagonalizing SVD 계열보다 느릴 수 있다.

Patchwork++에서는 matrix가 작다.
그래서 "큰 matrix에서 느리다"는 문제가 그대로 적용되는 것은 아니다.
하지만 매 patch마다 3x3 decomposition을 반복하는 hot loop라면 이야기가 달라진다.
3x3 하나는 작아도 호출 횟수가 많기 때문이다.

`JacobiSVD`는 아주 일반적인 문제를 푸는 도구이다.
반면 Patchwork++의 `cov`는 이미 symmetric PSD라는 훨씬 강한 구조를 가진다.
이 구조를 알고도 일반 SVD를 쓰는 것은 문제에 비해 도구가 조금 무겁다.

#### BDCSVD

`BDCSVD`는 bidiagonal divide-and-conquer 방식의 SVD이다.
큰 dense matrix에서는 `JacobiSVD`보다 유리한 경우가 많다.
하지만 3x3 같은 tiny matrix에서는 setup overhead가 상대적으로 커서 좋은 선택이 아니다.
Eigen 문서도 `BDCSVD`가 내부적으로 small block을 `JacobiSVD`로 처리하며,
default switching size가 16이고 small matrix, 즉 16보다 작은 matrix에서는 `JacobiSVD`를 직접 쓰는 것이 좋다고 설명한다.

즉 Patchwork++ plane fitting에서는 `JacobiSVD`를 `BDCSVD`로 바꾸는 방향이 핵심 개선책은 아니다.
큰 matrix용 SVD로 바꾸는 것이 아니라,
**covariance의 symmetry를 쓰는 solver로 바꾸는 것**이 더 자연스럽다.

SVD 계열만 놓고 보면 결론은 단순하다.
`ComputeFullU`를 `ComputeThinU`로 바꾸는 것보다,
`JacobiSVD`를 `BDCSVD`로 바꾸는 것이 더 큰 변화처럼 보인다.
하지만 여기처럼 3x3 covariance로 plane fitting을 하는 경우에는 둘 다 핵심 개선책이 아니다.

진짜로 봐야 할 지점은 다음 단계이다.
입력 matrix가 아무 matrix가 아니라 symmetric PSD covariance라는 사실을 solver 선택에 반영할 수 있는가?

### Symmetric Eigensolver: SelfAdjointEigenSolver With computeDirect()

Eigen에는 symmetric/self-adjoint matrix 전용 solver가 있다.

```cpp
Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver;
solver.computeDirect(cov, Eigen::ComputeEigenvectors);
```

`SelfAdjointEigenSolver`는 matrix가 self-adjoint라는 사실을 이용한다.
real-valued covariance matrix에서는 self-adjoint가 symmetric이라고 보면 된다.

이 solver가 더 맞는 이유는 세 가지이다.

1. covariance matrix는 symmetric PSD이다.
2. 필요한 것은 principal direction과 eigenvalue이다.
3. 3x3 matrix에는 `computeDirect()`라는 closed-form direct method를 쓸 수 있다.

대칭 matrix에서는 eigendecomposition이 다음처럼 생긴다.

$$
A = V D V^T
$$

PSD이면 eigenvalue가 모두 0 이상이다.
따라서 covariance matrix에서는 left/right singular vector를 따로 구할 필요가 없다.
eigenvector 하나의 basis가 곧 필요한 방향이고, eigenvalue가 기존 코드에서 쓰던 singular value와 같은 scale이다.

`JacobiSVD`는 더 일반적인 도구이다.
일반 rectangular matrix나 non-symmetric matrix에도 쓸 수 있다.
하지만 지금 문제는 훨씬 특수하다.
특수한 문제에는 특수한 solver가 보통 더 빠르고 더 직접적이다.

여기서 한 단계 더 가면 `computeDirect()`가 있다.
Eigen 문서 기준으로 `computeDirect()`는 compile-time size를 아는 2x2, 3x3 matrix에 대해 지원된다.
일반 iterative QR algorithm보다 보통 훨씬 빠르지만, 경우에 따라 정확도는 조금 떨어질 수 있다고 설명되어 있다.
3x3 float에서는 eigenvalue worst-case relative error가 더 커질 수 있다는 caveat도 있다.


---

## Migration Convention: Scale, Order, Normal Sign

solver를 바꿀 때 가장 조심할 부분은 output convention이다.
Patchwork++ 현재 코드는 원본 centered point matrix `X`가 아니라 covariance matrix `C`에 SVD를 건다.
이 전제에서는 covariance의 singular value와 eigenvalue가 같은 scale이다.

$$
\sigma_i(C) = \lambda_i(C)
$$

따라서 `JacobiSVD(cov).singularValues()`를
`SelfAdjointEigenSolver(cov).eigenvalues()`로 바꿀 때,
일반적으로 sqrt를 취하면 안 된다.

다만 order는 다르다.

```text
JacobiSVD singularValues(): 큰 값 -> 작은 값
SelfAdjointEigenSolver eigenvalues(): 작은 값 -> 큰 값
```

이 order 차이가 실제 코드 변경에서 가장 중요하다.

### Caution About Convention

Implementation-wise로는 아래 기존 SVD 코드를:

```cpp
Eigen::JacobiSVD<Eigen::Matrix3f> svd(cov, Eigen::ComputeFullU);

Eigen::Vector3f singular_values = svd.singularValues();  // descending
Eigen::Vector3f principal       = svd.matrixU().col(0);  // largest direction
Eigen::Vector3f normal          = svd.matrixU().col(2);  // smallest direction
```

`SelfAdjointEigenSolver`로 바꾸면 order가 반대이므로 주의해야 한다.

```cpp
Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig;
eig.computeDirect(cov, Eigen::ComputeEigenvectors);

Eigen::Vector3f evals = eig.eigenvalues();      // ascending
Eigen::Matrix3f evecs = eig.eigenvectors();     // matching columns

Eigen::Vector3f normal    = evecs.col(0);       // smallest eigenvalue
Eigen::Vector3f principal = evecs.col(2);       // largest eigenvalue
```

기존 code가 `singular_values_(0)`을 가장 큰 값,
`singular_values_(2)`를 가장 작은 값으로 가정하고 있으므로,
저장할 때는 descending order로 맞추는 게 안전하다.

```cpp
Eigen::Vector3f evals = eig.eigenvalues().cwiseMax(0.0f);

out.singular_values_ << evals(2), evals(1), evals(0);
out.principal_ = eig.eigenvectors().col(2);
out.normal_    = eig.eigenvectors().col(0);
```

`cwiseMax(0.0f)`는 작은 수치 오차로 인해 `-1e-8` 같은 eigenvalue가 나오는 경우를 방어하는 용도이다.
covariance는 이론적으로 PSD이므로 음수 eigenvalue이면 안 된다.

normal sign 처리는 기존처럼 유지해야 한다.

```cpp
if (out.normal_(2) < 0.0f) {
  out.normal_ = -out.normal_;
}
```

SVD든 eigendecomposition이든 eigenvector/singular vector의 sign은 원래 ambiguous하다.
즉 `n`과 `-n`은 같은 plane normal이다.
Patchwork++처럼 z 방향이 양수가 되게 맞추는 post-processing은 계속 필요하다.


---

## Recommendation: Patchwork++에서는 무엇을 바꿀까

내 결론은 이렇다.

### 1. 지금 코드를 거의 유지한다면

`ComputeFullU`는 유지해도 된다.
normal을 얻으려면 U가 필요하기 때문이다.
대신 V는 계산하지 않는 것이 맞다.

```cpp
Eigen::JacobiSVD<Eigen::Matrix3f> svd(cov, Eigen::ComputeFullU);
```

이때 `cov` type은 가능하면 `Eigen::Matrix3f`가 낫다.
`patchworkpp.cpp`에서는 `Eigen::MatrixX3f cov`로 받고 있지만,
실제로는 `3 x 3` matrix이므로 fixed-size `Matrix3f`가 의도를 더 잘 드러낸다.

### 2. option만 바꾸고 싶다면

`ComputeThinU`로 얻는 이득은 기대하지 않는 게 좋다.
입력이 이미 `3 x 3`이기 때문이다.
또 fixed-size `Matrix3f`에서는 thin U가 원하는 방향의 최적화가 아닐 수 있으므로,
이 옵션만 바꾸는 수정은 추천하지 않는다.

`ComputeFullV`는 쓰지 않는 것이 맞다.
Patchwork++는 V를 사용하지 않는다.

### 3. 제대로 바꾸고 싶다면

`SelfAdjointEigenSolver<Matrix3f>::computeDirect()`가 더 맞다.

```cpp
Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig;
eig.computeDirect(cov, Eigen::ComputeEigenvectors);
```

그리고 기존 SVD output convention에 맞춰야 한다.

```cpp
const Eigen::Vector3f evals = eig.eigenvalues().cwiseMax(0.0f);

out.normal_ = eig.eigenvectors().col(0);
if (out.normal_(2) < 0.0f) out.normal_ = -out.normal_;

out.principal_ = eig.eigenvectors().col(2);
out.singular_values_ << evals(2), evals(1), evals(0);
```

이렇게 하면 기존의

```cpp
linearity_ = (s0 - s1) / s0;
planarity_ = (s1 - s2) / s0;
```

같은 계산도 그대로 유지할 수 있다.
여기서 `s0 >= s1 >= s2`라는 convention을 보존했기 때문이다.

### Expected Performance: 얼마나 빨라질까

`estimate_plane()`는 patch마다 반복 호출된다.
그래서 3x3 decomposition 하나하나는 작아도 전체 runtime에서는 의미가 있을 수 있다.

다만 이 최적화가 end-to-end runtime을 2배로 줄이는 종류의 변화는 아닐 가능성이 크다.
Patchwork++ 전체 runtime에는 patch 분할, sorting, vector allocation, point copy, iterative GLE 같은 다른 비용도 같이 들어간다.

따라서 기대치는 이렇게 잡는 게 현실적이다.

```text
per-plane decomposition: 꽤 빨라질 수 있음
전체 Patchwork++ runtime: modest improvement일 가능성이 큼
```

실제로 현재 버전에서 `JacobiSVD`를 eigen decomposition 쪽으로 바꾸는 `eigh only` 변경만 적용해 보면,
속도는 아래 정도로 나왔다.
여기서 `eigh only`는 per-call allocation 구조는 그대로 두고,
plane fitting의 decomposition만 바꾼 단계라고 보면 된다.

| Stage | patchwork++ | Δ vs baseline | patchwork | Δ vs baseline |
| --- | ---: | ---: | ---: | ---: |
| With `JacobiSVD`  | 10.26 ms / 97.5 Hz | - | 4.26 ms / 234.7 Hz | - |
| With `SelfAdjointEigenSolver` | 9.94 ms / 100.6 Hz | +3.2% Hz | 4.30 ms / 232.8 Hz | noise |

이 결과만 보면 patchwork++에서는 실제로 약간 빨라졌다.
다만 patchwork 쪽에서는 오히려 아주 조금 느려진 것처럼 보이는데,
이 정도 차이는 benchmark noise로 보는 게 맞다.
즉 eigensolver 변경은 "분명히 맞는 방향이지만, 전체 runtime을 크게 뒤집는 최적화는 아니다" 정도로 해석하는 것이 안전하다.

---

## 정리

`ComputeFullU` 자체가 Patchwork++의 큰 문제는 아니다.
3x3 covariance matrix에서는 full U와 thin U 차이가 사실상 없다.

핵심은 다음이다.

- Patchwork++는 원본 point matrix가 아니라 3x3 covariance matrix를 분해한다.
- covariance matrix는 symmetric PSD이다.
- 이 경우 singular value는 eigenvalue와 같다.
- 따라서 `SelfAdjointEigenSolver<Matrix3f>::computeDirect()`가 더 적합하다.
- 단, Eigen의 eigenvalue는 ascending order이므로 기존 SVD convention에 맞춰 reverse해서 저장해야 한다.
- normal은 `eig.eigenvectors().col(0)`이고, principal direction은 `eig.eigenvectors().col(2)`이다.
- sqrt를 넣으면 안 된다. sqrt는 원본 centered data matrix에 SVD를 걸었을 때 covariance eigenvalue와 연결되는 관계이다.

그래서 내가 내리는 결론은 이렇다.

> Patchwork++의 `ComputeFullU`를 `ComputeThinU`로 바꾸는 것은 큰 의미가 없고, 성능과 의미를 모두 생각하면 `JacobiSVD`에서 `SelfAdjointEigenSolver<Matrix3f>::computeDirect()`로 바꾸는 쪽이 더 맞다.

---

## Official Docs Check

Eigen의 dense linear solver reference는 Cholesky, LU, QR, SVD, Eigenvalues module로 나뉜다.
이 글과 직접 관련 있는 것은 SVD module의 `JacobiSVD`, `BDCSVD`와 Eigenvalues module의 `SelfAdjointEigenSolver`이다.

공식 문서 기준으로 다시 확인하면 다음과 같다.

- `JacobiSVD`는 singular value만 기본 계산하고, U/V가 필요하면 `ComputeFullU`, `ComputeThinU`, `ComputeFullV`, `ComputeThinV`를 명시해야 한다.
- `JacobiSVD`의 singular value는 decreasing order이다.
- `ComputeThinU`는 rectangular matrix에서 full U의 불필요한 column을 만들지 않기 위한 옵션이다.
- `BDCSVD`는 large matrix에서 유리하지만, 문서상 small matrix, 구체적으로 16보다 작은 matrix에서는 `JacobiSVD`를 직접 쓰는 것이 좋다고 되어 있다.
- `SelfAdjointEigenSolver`는 real matrix 기준 symmetric matrix용 eigensolver이고, general eigenvalue solver보다 self-adjoint structure를 이용해 더 빠르고 정확할 수 있다.
- `SelfAdjointEigenSolver::eigenvalues()`는 increasing order이다.
- `computeDirect()`는 compile-time size를 아는 2x2, 3x3 matrix에 대해 지원되고, 일반 iterative QR algorithm보다 보통 빠르지만 정확도 caveat가 있다.

따라서 이 글의 핵심 결론은 공식 문서와도 맞다.
다만 `ComputeThinU`에 대해서는 "3x3이면 사실상 같다" 정도로만 이해하기보다,
**Patchwork++의 fixed-size 3x3 covariance에서는 바꿀 이유가 없는 옵션**이라고 이해하는 편이 더 안전하다.

---

## References

- [Eigen documentation - JacobiSVD](https://libeigen.gitlab.io/eigen/docs-nightly/classEigen_1_1JacobiSVD.html)
- [Eigen documentation - SelfAdjointEigenSolver](https://libeigen.gitlab.io/eigen/docs-nightly/classEigen_1_1SelfAdjointEigenSolver.html)
- [Eigen documentation - BDCSVD](https://libeigen.gitlab.io/eigen/docs-nightly/classEigen_1_1BDCSVD.html)
- [Eigen documentation - Dense linear solver reference](https://libeigen.gitlab.io/eigen/docs-nightly/group__DenseLinearSolvers__Reference.html)
- [Eigen documentation - Matrix arithmetic and SVD tutorial](https://libeigen.gitlab.io/eigen/docs-nightly/group__TutorialLinearAlgebra.html)
- [Patchwork++ official repository](https://github.com/url-kaist/patchwork-plusplus)
