---
layout: post
title: Ceres Solver for Graph SLAM - 2. Pose Error를 정의하는 법
subtitle: From 2D pose to 3D quaternion, SLAM-friendly residual의 첫걸음
tags: [SLAM, Optimization, Ceres Solver, Pose Graph, SE(2), SE(3)]
comments: true
---

## Introduction

[1편](https://limhyungtae.github.io/2026-05-07-Ceres-Solver-for-Graph-SLAM-1.-%EA%B8%B0%EB%B3%B8-%EC%82%AC%EC%9A%A9%EB%B2%95-%EC%84%A4%EB%AA%85-%EB%B0%8F-%EC%98%88%EC%8B%9C/)에서는 *스칼라 변수 한 개*에 대한 Ceres의 가장 기본적인 사용법, Hello World와 multiple residual, 을 다뤘다. 그런데 SLAM에서 다루는 변수는 단순한 스칼라가 아니라 **pose**, 즉 robot의 위치와 자세이다. Pose는 SE(2)나 SE(3) 같은 *rigid body transformation manifold*에 살고 있어서, *그냥 빼기로 residual을 만드는* 1편의 패턴을 그대로 적용할 수가 없다.

이 글에서는 그 다리를 놓는다. 즉 SLAM optimization에서 흔히 말하는 *"pose error"*가 정확히 무엇이고, 그걸 어떻게 코드에 옮기는지를 *2D부터 3D까지* 차근차근 짚어본다. 이 글의 목표는 다음과 같다.

* **Pose error를 직관적으로 이해**: 두 pose의 차이를 어떻게 정의해야 하는지.
* **2D에서 손으로 따라가보기**: SE(2)에서는 angle wrapping만 신경쓰면 비교적 단순하다는 것을 확인.
* **3D로 가면 왜 어려워지는가**: rotation을 quaternion으로 표현할 때 *manifold가 등장하는 이유* 미리 보기.

여기서 다루는 내용은 [4편](https://limhyungtae.github.io/2026-05-07-Ceres-Solver-for-Graph-SLAM-4.-PoseGraph3dErrorTerm-%EA%B9%8A%EA%B2%8C-%EC%9D%B4%ED%95%B4%ED%95%98%EA%B8%B0/)의 더 깊은 분석을 위한 워밍업이다. 사실 진짜 어려운 건 4편에서 시작되는데, 이 글이 그 입구가 된다.

각설하고, 출발해보자.

---

## "Pose error"란 무엇인가

먼저 용어부터. SLAM에서 *pose*는 robot의 *위치 + 자세*를 함께 묶은 표현이다.

* 2D: pose $$\mathbf{x} = (x, y, \theta)$$, 평면 위의 위치 $$(x, y)$$와 yaw $$\theta$$.
* 3D: pose $$\mathbf{x} = (\mathbf{p}, \mathbf{q})$$, 3D position $$\mathbf{p}$$와 unit quaternion $$\mathbf{q}$$ (또는 SO(3) rotation).

Pose graph SLAM에서는 *두 pose 사이의 상대 변환* 측정값(measurement)이 주어지고, 우리는 *현재 추정 pose가 그 measurement와 얼마나 잘 맞는지*를 cost로 만들고 싶다. 이게 *pose error*이다.

수식으로 쓰면 다음 형태이다.

$$
\mathbf{e}(\mathbf{x}_a, \mathbf{x}_b) = \tilde{\mathbf{T}}_{ab}(\mathbf{x}_a, \mathbf{x}_b) \ominus \hat{\mathbf{T}}_{ab}
$$

* $$\tilde{\mathbf{T}}_{ab}$$: 추정 pose $$\mathbf{x}_a, \mathbf{x}_b$$로부터 *계산한* 상대 변환.
* $$\hat{\mathbf{T}}_{ab}$$: sensor가 *측정한* 상대 변환.
* $$\ominus$$: pose 사이의 *어떤 의미에서의 차이*. 이 정의가 manifold에 따라 다르다는 게 이 글의 핵심이다.

스칼라 또는 vector라면 $$\ominus$$는 단순한 빼기이다. 하지만 angle이나 quaternion이 들어오면 빼기로는 부족하다, 이 부분이 SLAM에서 *pose error를 정의하는 게* SLAM 입문자에게 까다로운 이유이기도 하다.

---

## 2D Pose (x, y, yaw)의 경우

2D pose $$\mathbf{x}_a = (x_a, y_a, \theta_a)$$, $$\mathbf{x}_b = (x_b, y_b, \theta_b)$$에 대해, 추정한 상대 변환 $$\tilde{\mathbf{T}}_{ab}$$는 다음과 같다.

$$
\tilde{\mathbf{T}}_{ab}
=
\begin{bmatrix}
\tilde{x}_{ab} \\
\tilde{y}_{ab} \\
\tilde{\theta}_{ab}
\end{bmatrix}
=
\begin{bmatrix}
\mathbf{R}(\theta_a)^T \cdot
\begin{bmatrix} x_b - x_a \\ y_b - y_a \end{bmatrix} \\
\theta_b - \theta_a
\end{bmatrix}
$$

* 위치 부분: world frame에서 본 변위를 A frame으로 회전.
* 각도 부분: 두 yaw의 차이.

Measurement $$\hat{\mathbf{T}}_{ab} = (\hat{x}_{ab}, \hat{y}_{ab}, \hat{\theta}_{ab})$$가 주어졌을 때, residual은:

$$
\mathbf{e} =
\begin{bmatrix}
\tilde{x}_{ab} - \hat{x}_{ab} \\
\tilde{y}_{ab} - \hat{y}_{ab} \\
\tilde{\theta}_{ab} - \hat{\theta}_{ab}
\end{bmatrix}
\in \mathbb{R}^3
$$

위치는 *그냥 빼기* 하면 된다. 둘 다 같은 frame(A frame)에서 표현된 2D 벡터이므로 vector subtraction이 의미가 있다.

문제는 마지막 줄, *각도 차이*이다. $$\theta$$는 $$[-\pi, \pi)$$ 안에서 정의되는데, 단순 빼기는 *wrapping* (예: $$179° - (-179°) = 358°$$이 되어야 하는데 실제로는 $$2°$$여야 함)을 반영하지 못한다. 이 함정을 피하기 위해서는 angle difference를 다음과 같이 wrapping해줘야 한다.

```cpp
template <typename T>
T NormalizeAngle(const T& angle_radians) {
  // wrap into [-π, π)
  T two_pi(2.0 * M_PI);
  return angle_radians - two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}
```

그래서 angle residual은 단순 빼기 대신:

```cpp
T angle_error = NormalizeAngle(theta_estimated - theta_measured);
```

이 정도가 2D pose error의 전부이다. 위치 2D + 각도 1D = 3D residual인 점, 그리고 angle wrapping이 필요한 점만 챙기면 된다.

> 사실 Ceres example 디렉토리에는 [`pose_graph_2d`](https://github.com/ceres-solver/ceres-solver/tree/master/examples/slam/pose_graph_2d)도 있는데, 위에서 설명한 정확히 그 형태로 되어있다. 코드량이 3D 버전보다 적고, manifold 처리도 angle 하나만 신경쓰면 되어서 SLAM 입문에 좋다. 한 번씩 열어보면 도움된다.

---

## 3D Pose (3D translation + 3D rotation)의 경우

이제 3D로 가보자. Pose가 $$\mathbf{x} = (\mathbf{p}, \mathbf{q})$$이고, $$\mathbf{p} \in \mathbb{R}^3$$, $$\mathbf{q}$$는 unit quaternion이다.

본격적으로 수식을 풀어내기 전에, 앞으로 자주 등장할 *표기법*을 한 번 정리하고 가자. 이후 글의 모든 수식은 이 약속을 따른다.

* **Subscript $$_a$$, $$_b$$**: 어느 frame/pose의 양인지 가리키는 index.  
  예: $$\mathbf{p}_a, \mathbf{q}_a$$는 "pose A의 position과 orientation". $$\mathbf{p}_b, \mathbf{q}_b$$는 마찬가지로 pose B의 것.
* **Subscript $$_{ab}$$**: "A에서 B로의 *상대* 변환".  
  예: $$\mathbf{p}_{ab}$$는 *A frame 좌표계에서 표현된 B의 위치 벡터*, $$\mathbf{q}_{ab}$$는 *A에서 B로 가는 회전*.
* **Tilde $$\tilde{\cdot}$$**: 현재 추정 pose로부터 *계산한 (estimated)* 값.  
  예: $$\tilde{\mathbf{q}}_{ab}$$는 "현재 추정값 $$\mathbf{q}_a, \mathbf{q}_b$$로부터 *계산한* 상대 회전".
* **Hat $$\hat{\cdot}$$**: sensor가 *측정한 (measured)* 값.  
  예: $$\hat{\mathbf{q}}_{ab}$$는 "A frame에서 B의 상대 회전을 *sensor가 측정해준* 값". 이 글에서 ~~hat~~을 단 변수는 모두 measurement이다.
* **Conjugate $$\mathbf{q}^{*}$$**: quaternion의 conjugate.  
  Quaternion $$\mathbf{q} = (\mathbf{v}, w)$$ (vector part $$\mathbf{v} \in \mathbb{R}^3$$, scalar part $$w \in \mathbb{R}$$)에 대해 $$\mathbf{q}^{*} = (-\mathbf{v}, w)$$. *Unit quaternion에서는 conjugate가 곧 inverse*이다, 즉 $$\mathbf{q}^{*} \otimes \mathbf{q} = (\mathbf{0}, 1)$$이 identity quaternion이 되며, *반대 방향의 회전*을 의미한다.
* **Quaternion product $$\otimes$$**: 두 quaternion의 곱.  
  단순한 element-wise 곱이 아니라 *회전의 합성 (composition)* 에 대응한다. 즉 $$\mathbf{q}_a \otimes \mathbf{q}_b$$를 회전 행렬로 옮기면 $$\mathbf{R}(\mathbf{q}_a) \mathbf{R}(\mathbf{q}_b)$$가 된다.

> 요약: *"hat($$\hat{\cdot}$$)은 measurement, tilde($$\tilde{\cdot}$$)는 estimate, $$_{ab}$$는 A→B의 상대"*. 그리고 $$\otimes$$는 quaternion 곱, $$^{*}$$는 conjugate(=inverse). 이 약속을 머릿속에 넣어두면 이 시리즈의 모든 수식을 같은 안경으로 읽을 수 있다.

이제 추정 상대 변환을 적어보면:

$$
\tilde{\mathbf{p}}_{ab} = \mathbf{R}(\mathbf{q}_a)^T (\mathbf{p}_b - \mathbf{p}_a), \quad
\tilde{\mathbf{q}}_{ab} = \mathbf{q}_a^{*} \otimes \mathbf{q}_b
$$

* $$\tilde{\mathbf{p}}_{ab}$$: world에서의 변위 $$(\mathbf{p}_b - \mathbf{p}_a)$$를 A frame으로 회전. $$\mathbf{R}(\mathbf{q}_a)^T$$가 *world → A frame* 회전.
* $$\tilde{\mathbf{q}}_{ab}$$: $$\mathbf{q}_a^{*}$$가 *world → A frame* 회전이고, 거기에 $$\mathbf{q}_b$$ (*world → B frame*)을 곱하면 *A → B*의 상대 회전이 나온다.

위치 부분 $$\tilde{\mathbf{p}}_{ab}$$는 2D 때와 동일하게 *그냥 빼기* 로 residual을 만들 수 있다.

$$
\mathbf{e}_{p} = \tilde{\mathbf{p}}_{ab} - \hat{\mathbf{p}}_{ab} \in \mathbb{R}^3
$$

문제는 회전이다. 2D에서는 angle이 $$\theta \in [-\pi, \pi)$$라서 *스칼라 빼기 + wrapping*으로 해결됐지만, 3D rotation은 quaternion (4D unit vector)이고 *그냥 빼기*가 의미가 없다.

다음과 같은 두 가지 문제가 생긴다.

1. **차이도 quaternion이어야 한다**: $$\tilde{\mathbf{q}}_{ab} - \hat{\mathbf{q}}_{ab}$$를 그냥 element-wise로 빼면 결과가 unit quaternion이 아닌 *4D vector*가 된다. 그러면 *3D 자유도*인 rotation을 4D vector로 표현해 residual에 넣게 되어서 자유도가 안 맞는다.
2. **Optimization update도 manifold-aware해야 한다**: LM solver가 update vector $$\Delta \mathbf{q}$$를 단순히 더해버리면 ($$\mathbf{q} \leftarrow \mathbf{q} + \Delta \mathbf{q}$$), 결과는 더 이상 unit quaternion이 아니다.

이 두 문제는 결국 *rotation이 vector space가 아니라 manifold(SO(3))*에 살고 있다는 한 가지 사실에서 비롯된다.

---

## SLAM 커뮤니티의 표준 해법: multiplicative quaternion error

위 (1)에 대한 SLAM 커뮤니티의 표준 해법은 *multiplicative quaternion error*이다. 즉 *빼기 대신 quaternion 곱*으로 차이를 정의한다.

$$
\delta \mathbf{q} = \hat{\mathbf{q}}_{ab} \otimes \tilde{\mathbf{q}}_{ab}^{*}
$$

추정과 measurement가 일치할 때 $$\delta \mathbf{q} = (0, 0, 0, 1)$$ ([x, y, z, w] order에서 *identity quaternion*)이 된다. 이게 우리가 원하는 *zero residual*의 의미이다.

그리고 identity 근처에서 quaternion의 vector part가 axis-angle의 *절반*이라는 사실 (4편에서 자세히 유도)을 이용해서, rotation residual을 다음과 같이 *3D vector*로 끌어내릴 수 있다.

$$
\mathbf{e}_{\theta} = 2 \cdot \text{Vec}(\delta \mathbf{q}) \in \mathbb{R}^3
$$

이로써 (translation 3 + rotation 3) = **6차원 residual**이 만들어진다. 이게 [3편](https://limhyungtae.github.io/2026-05-07-Ceres-Solver-for-Graph-SLAM-3.-Pose-Graph-3D-Example-%ED%95%9C%EB%88%88%EC%97%90-%EB%B3%B4%EA%B8%B0/)부터 다루는 `PoseGraph3dErrorTerm` 코드에 정확히 들어있는 형태이다.

문제 (2), *manifold-aware update*, 는 Ceres의 `Manifold` API로 해결되며, 이건 [5편](https://limhyungtae.github.io/2026-05-07-Ceres-Solver-for-Graph-SLAM-5.-BuildOptimizationProblem%EA%B3%BC-Manifold-%EA%B9%8A%EA%B2%8C-%EC%9D%B4%ED%95%B4%ED%95%98%EA%B8%B0/)에서 본격적으로 다룬다. 핵심 idea만 미리 말하자면, *update를 4D quaternion에 직접 더하지 않고, 3D tangent space (axis-angle)에서 받아서 manifold로 retract*시키는 방법이다.

---

## 정리: 1D부터 3D까지의 residual 비교

이 글에서 다룬 residual의 형태를 정리하면 다음과 같다.

| 변수 type | 변수 dim | residual 정의 | residual dim | manifold 처리 |
|----------|---------|---------------|--------------|--------------|
| 스칼라 ($$x$$) | 1 | $$x_{\text{est}} - x_{\text{meas}}$$ | 1 | 불필요 |
| 2D 위치 ($$x, y$$) | 2 | element-wise 빼기 | 2 | 불필요 |
| 각도 ($$\theta$$) | 1 | wrapped subtraction | 1 | (간단한) wrapping |
| 3D 위치 ($$\mathbf{p}$$) | 3 | element-wise 빼기 | 3 | 불필요 |
| Quaternion ($$\mathbf{q}$$) | 4 | $$2 \cdot \text{Vec}(\hat{\mathbf{q}} \otimes \tilde{\mathbf{q}}^{*})$$ | 3 | **필요** |
| 2D pose | 3 | (위치 빼기, 각도 wrapping) | 3 | (간단) |
| 3D pose | 7 (3+4) | (위치 빼기, multiplicative quaternion) | 6 | **필요** |

표가 보여주는 핵심은:

* **Position(translation)은 어디서나 vector space**라서 element-wise 빼기로 해결된다.
* **Rotation**은 manifold에 살고 있어서 *별도의 처리*가 필요하다.
  * 2D는 scalar angle이라 wrapping만 하면 되어 비교적 간단.
  * 3D는 multiplicative quaternion error + manifold-aware update가 모두 필요해서 갑자기 복잡해진다.

이 *2D는 쉽고 3D는 어려워지는* 지점이, 사실 SLAM 입문자가 가장 많이 막히는 부분이다.

---

## 결론

이 글에서는 SLAM의 *pose error*가 무엇인지, *추정 상대 변환과 측정 상대 변환의 차이*임, 그리고 그 정의가 *2D에서 3D로 갈 때 어떻게 점점 까다로워지는지*를 살펴봤다. 핵심은 한 줄로 요약된다.

> *Translation은 vector space라 그냥 빼면 되는데, rotation은 manifold라서 multiplicative error로 정의해야 한다.*

다음 [3편](https://limhyungtae.github.io/2026-05-07-Ceres-Solver-for-Graph-SLAM-3.-Pose-Graph-3D-Example-%ED%95%9C%EB%88%88%EC%97%90-%EB%B3%B4%EA%B8%B0/)부터는 이 idea가 실제로 구현된 *Ceres pose_graph_3d 공식 example*을 line-by-line으로 분석한다. 두 파일 (`pose_graph_3d.cc`, `pose_graph_3d_error_term.h`)이 이 글에서 짚은 모든 idea를 어떻게 코드로 풀어내는지, 그게 이 시리즈의 본론이다.

---

Ceres Solver for Graph SLAM 시리즈입니다.

{% include post_links_ceres.html %}
