---
layout: post
title: GTSAM Tutorial 1. SLAM을 위한 Between Factor 쉽게 이해하기
subtitle: How does GTSAM work? 
tags: [Jacobian, GTSAM, Optimization]
comments: true

---

## Introduction 

나는 지금 GTSAM의 주 관리자인 Luca Carlone 교수님네에 포닥으로 와있다보니, 
GTSAM을 기반으로 코드를 짜야할 일이 많아졌다. 그래서 GTSAM에 대해 깊게 공부할 일이 많아졌는데, 이번에는 이를 활용해서 GTSAM을 처음 써보는 이들을 위한 포스팅을 하고자 한다. 특히, GTSAM 내부적으로 어떻게 동작하는지에 대해 설명하는 글이 생각보다 없어서, GTSAM에서 제공하는 pose graph optimization(혹은 factor graph optimization)을 이해함과 동시에 좀더 깊게 Lie group 상에서 optimization이 어떻게 동작하는 지에 대해 면밀히 다뤄보도록 한다.

따라서 이 글에서는 [이미 GTSAM에서 제공하는 튜토리얼](https://gtsam.org/tutorials/intro.html)에 대한 이해를 돕기 위한 설명글로 출발해서, GTSAM 내부에서 어떤 일이 일어나고 있는지 분석해볼 것이다. 그러니, 이 글은 SLAM을 처음 접하는 이에게는 적절하지 않을 수도 있다. 오히려 SLAM을 돌려보긴 했으나 내부적으로 어떤 과정이 일어나서 optimization이 되는지 자세히 살펴보고자 하는 이들에게 큰 도움이 되리라 생각된다.
이 시리즈 글을 읽어보기 전 [Pose2SLAMExample.cpp](https://github.com/devbharat/gtsam/blob/master/examples/Pose2SLAMExample.cpp) 코드를 한번 보면서 읽어보기를 추천한다.
그리고 SLAM 자체에 대해 잘 모르는 이는 [김기섭 박사가 쓴 SLAM에 대한 intro 글](https://gisbi-kim.github.io/blog/2021/03/04/slambackend-1.html)을 과 Frank Dellaert 교수님의 [2D Pose SLAM in GTSAM](https://piazza.com/class_profile/get_resource/hbl3nsqea3z6uo/hf5dj0hcfey5fi#page=2.66)을 읽어보는 것을 추천한다. 이 시리즈는 **사용자 입장에서는 전혀 알 필요가 없는 부분들을 설명할 것**이기 때문에, 만약 나는 아묻따 GTSAM으로 pose graph SLAM하는 방법만 궁금한 이는 응창이 형이 쓴 [High-level GTSAM 튜토리얼](https://engcang.github.io/gtsam_tutorial.html)을 읽는 것을 권장한다.

---

## Pose Graph Optimization과 BetweenFactor

위의 `Pose2SLAMExample.cpp`에 보면 알겠지만, pose graph optimization은 (i) `NonlinearFactorGraph ` 클래스에다 우리가 원하고자 하는 constraint(혹은 factor. 이 SLAM 예제의 경우에는 `BetweenFactor`를 사용함)를 추가해서 graph 구조 생성, (ii) `Values` 클래스로 정의한 graph 구조에 필요한 initial value 제공, (iii) Optimizer로 optimize하면 끝난다.
     
그렇다면 아래의 `BetweenFactor`는 내부적으로 어떻게 동작해서 optimization을 할 수 있게 해줄까?

```cpp
graph.add(PriorFactor<Pose2>(1, Pose2(0, 0, 0), priorNoise));

// For simplicity, we will use the same noise model for odometry and loop closures
noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.1));

// 2b. Add odometry factors
// Create odometry (Between) factors between consecutive poses
graph.add(BetweenFactor<Pose2>(1, 2, Pose2(2, 0, 0     ), model));
graph.add(BetweenFactor<Pose2>(2, 3, Pose2(2, 0, M_PI_2), model));
graph.add(BetweenFactor<Pose2>(3, 4, Pose2(2, 0, M_PI_2), model));
graph.add(BetweenFactor<Pose2>(4, 5, Pose2(2, 0, M_PI_2), model));

// 2c. Add the loop closure constraint
// This factor encodes the fact that we have returned to the same pose. In real systems,
// these constraints may be identified in many ways, such as appearance-based techniques
// with camera images. We will use another Between Factor to enforce this constraint:
graph.add(BetweenFactor<Pose2>(5, 2, Pose2(2, 0, M_PI_2), model));
```

(위는 [Pose2SLAMExample.cpp](https://github.com/devbharat/gtsam/blob/master/examples/Pose2SLAMExample.cpp)의 일부분)

---

## `BetweenFactor`의 원리

`BetweenFactor`는 Lie group으로 정의된 두 객체 간의 거리 차이(여기서 '거리'는 position만 될 수도 있고, pose도 될수도 있다. 정의하기 나름)를 measurement로 쓰기 위한 factor이다. 만약 pose-to-point distance를 기반으로 한 error term을 활용하려면 `PoseToPointFactor`를 써야 한다. 이처럼 우리가 optimization하고자 하는 factor들만 설정해주면 GTSAM 내부적으로 어떻게 동작하는지 알지 못하더라도 optimization을 할 수 있다. 이러한 이유 때문에 Frank Dellaert 교수님은 이를 'factor graph'라고 부른다. 그리고 'pose graph'라는 용어는 이 factor graph에서 pose만 다루는 graph를 지칭하는, factor graph의 특수한 케이스라고 생각하면 된다.

각설하고, 이 글에서는 pose간 차이를 나타내는 `BetweenFactor`를 분석해보자.
`BetweenFactor`는 [여기](https://github.com/borglab/gtsam/blob/bb5cfb22840360a39e590cc19e5db84e58d6f897/gtsam/slam/BetweenFactor.h#L40) 코드에 보면 객체화되어 정의가 되어 있다.

```cpp
 /// evaluate error, returns vector of errors size of tangent space
Vector evaluateError(const T& p1, const T& p2,
  OptionalMatrixType H1, OptionalMatrixType H2) const override {
  T hx = traits<T>::Between(p1, p2, H1, H2); // h(x)
  // manifold equivalent of h(x)-z -> log(z,h(x))
#ifdef GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR
  typename traits<T>::ChartJacobian::Jacobian Hlocal;
  Vector rval = traits<T>::Local(measured_, hx, OptionalNone, (H1 || H2) ? &Hlocal : 0);
  if (H1) *H1 = Hlocal * (*H1);
  if (H2) *H2 = Hlocal * (*H2);
  return rval;
#else
  return traits<T>::Local(measured_, hx);
#endif
}
```

~~호우쮖 뭔가가 굉장히 복잡하게 생겼다.~~ 찬찬히 line-by-line으로 살펴보면, 크게 4 단계로 쪼개어 설명할 수 있다.

### Step 1

`traits<T>::Between`을 통해 `p1`과 `p2`에 해당하는 class의 `between` 함수 불러 옴. between 함수들은 모든 Lie Group의 엄마 객체인 [gtsam/base/Lie.h](https://github.com/borglab/gtsam/blob/7150f284a8b6356cbefe78f066b50ad31ea6cbcc/gtsam/base/Lie.h#L52)에 정의되어 있다: 

```cpp
Class between(const Class& g) const {
  return derived().inverse() * g;
}
```

```cpp
Class between(const Class& g, ChartJacobian H1,
    ChartJacobian H2 = {}) const {
  Class result = derived().inverse() * g;
  if (H1) *H1 = - result.inverse().AdjointMap();
  if (H2) *H2 = Eigen::Matrix<double, N, N>::Identity();
  return result;
}
```

그럼 여기서 '`p1`과 `p2`가 무엇일까?'하는 의문이 드는데, 이 변수들은 우리가 처음 아래처럼 할당해준 initial value 값에 대응된다:

```cpp
Values initialEstimate;
initialEstimate.insert(1, Pose2(0.5, 0.0,  0.2   ));
initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2   ));
initialEstimate.insert(3, Pose2(4.1, 0.1,  M_PI_2));
initialEstimate.insert(4, Pose2(4.0, 2.0,  M_PI  ));
initialEstimate.insert(5, Pose2(2.1, 2.1, -M_PI_2));
```

예를 들어서, `graph.add(BetweenFactor<Pose2>(2, 3, Pose2(2, 0, M_PI_2), model));` 의 `BetweenFactor`는 optimization 시 `2`에 해당되는 `Pose2`를 `p1`에, `3`에 해당되는 `Pose2`를 `p2`에 대입하여 relative pose를 계산한다. 즉, $$\left(\mathbf{T}^{w}_1\right)^{-1} \mathbf{T}^{w}_2$$를 통해 $$\mathbf{T}^{1}_2$$를 얻는 것이다 

[GTSAM convention](https://gtsam.org/gtsam.org/2020/06/28/gtsam-conventions.html)에 따르면 1이 `from`(i.e., pose의 기준이 되는 좌표계)이 되고 2가 `to`가 된다. 즉, `from`이 'with respect to'(어느 축을 기준으로 보았을 때)의 의미를 지닌다. 사실 registration의 경우에는 2를 `from`, 1을 `to`라고 쓰는데, 잘 이해가 안 가는 이는 [이 글]([www.naver.com](https://limhyungtae.github.io/2022-09-04-SLAM,-Robotics-%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D%EC%9D%84-%EC%9C%84%ED%95%9C-3%EA%B0%80%EC%A7%80-%EC%BD%94%EB%94%A9-%EA%BF%80%ED%8C%81-(1)/))를 다시 읽어보길.

### Step 2

Measurement `measured_` 값과 relative 추정 값인 `hx` 비교해서 residual `rval` 계산한다. `Vector rval = traits<T>::Local(measured_, hx, OptionalNone, (H1 || H2) ? &Hlocal : 0);` 줄에 `Local`이라는 함수가 있는데, 이는 Lie Group에서 SE(2)나 SE(3)를 Log mapping을 통해 vector화 하는 과정이다. 만약 `measured_`와 `hx`가 덧셈/뺄셈이 가능한 값들이었다면, `hx - measured_`를 하면 되었을 것이다. 
하지만 알고 있다시피, 2D/3D pose에서는 덧셈은 곱셈으로, 뺄셈은 inverse matrix와의 곱셈으로 표현되는 곱셈의 세계(?)이기 때문에, `hx.inverse() * measured_`를 계산해서 측정된 상대 pose(i.e., `measured_`)와 추정된 상대 pose(i.e., `hx`)간의 차이를 `Log()` 함수를 통해 최종적으로 vector인 `rval`로 표현한다.

여기서 Jacobians인 `H1`과 `H2`도 구해지지만, 이 구해지는 과정은 다른 글에서 심도 있게 다루도록 한다.

### Step 3 What's Next?: Solving objective function

그 다음엔 무슨 일이 일어날까? 이런 factor graph 상에서 모든 factor의 `evaluateError()` 함수를 통해 error와 Jacobian term들을 계산하고 난 후, 아래와 같이 optimization을 시행한다([2D Pose SLAM in GTSAM](https://piazza.com/class_profile/get_resource/hbl3nsqea3z6uo/hf5dj0hcfey5fi#page=2.66)에서 발췌)

---

![gtsam_solving](/img/gtsam_solving.png)

---

즉, 
`evaluateError` 함수가 실행될 때 `OptionalMatrixType H1`와 `OptionalMatrixType H2`에 저장된 값이 위의 첨부 수식 상의 $$H_{j1}$$, $$H_{j2}$$로 각각 대입되고, `hx`와 `measured_`는 각각 $$h(\xi_{j1}, \xi_{j2})$$와 $$\Delta \xi_i$$에 대입되어 최종적으로는 $$b_i$$로 표현된다 위의 식에서 최종적으로 우리가 구해야하는 값은 $$\delta_{j1}$$과 $$\delta_{j2}$$인데, 이 두 값은 각각 initialal values에 업데이트되는, pose의 변화량을 vector의 형태로 나타낸 값이다(예로 들자면, 위의 `graph.add(BetweenFactor<Pose2>(2, 3, Pose2(2, 0, M_PI_2), model));`에서, optimization을 하는 과정 중 graph 구조 내의 두 번째 pose와 세 번째 pose에 한 iteration 동안 어느 정도의 값을 update를 할지를 뜻한다). 

이를 통해 GTSAM이 어떻게 `BetweenFactor`를 활용해 optimization을 하는지 clear하게 이해할 수 있다!

### Step 4. Update Values in Vector Space → Retract

Optimization을 한 후에는, 그 후에는 변화량을 기존 pose의 값에 추가해줘야 한다.
이는 위에 나와 있는 제일 마지막 줄처럼 

$$\xi^{t+1}_i = \xi^{t}_i \oplus \delta_i$$

를 통해 다음 iteration $$t+1$$에 쓸 pose를 업데이트한다. 위의 경우에는 $$j1$$ 번째 pose와 $$j2$$ 번째 포즈를 $$\xi^{t}_i$$로 간단하게 나타내었지만, 단순화를 위해 임의로 $$\xi^{t}_i$$가 단일 node의 pose라고 가정하자. 여기서 $$\delta_i$$는 2D의 경우에는 $$\mathbb{R}^3$$(x, y, theta로 구성), 3차원의 경우 $$\mathbb{R}^6$$(rotation vector 3개와 x, y, z. 참고로 rotation vector $$\neq$$ (roll, pitch yaw)이다!)이다. 그 후, 다시 vector로 표현되어 있는 $$\xi^{t+1}_i$$을 2D의 경우 $$3\times3$$의 SE(2)로, 3D의 경우 $$4\times4$$의 SE(3)의 $$\mathbf{T}_i$$로 다시 변환해주는 것이 필요한데, 이 행위를 *retract*라고 부른다. 그 후 다시 Step 1로 돌아가서 이 values들이 수렴할 때까지(i.e., $$\delta_i$$의 크기가 0에 충분히 가까워질때 까지) 반복적으로 optimization을 시행한다. 이러한 동작 방식으로 인해 iterative opimization이라고 부르는 것이다.

---

## 실제 SLAM System에서는

그렇다면 실제 SLAM system에서는 무엇을 고려해주어야 할까? 
예제 코드와 다른 점은 실제 SLAM system은 실시간으로 코드를 받아온 후, 1) graph 구조를 생성하고 2) 해당 graph에 해당하는 초기 값들을 initial values로 점진적으로 업데이트시켜줘야 한다.
따라서 [FAST-LIO-SAM-QN](https://github.com/engcang/FAST-LIO-SAM-QN/blob/c586b8955a1b726243f4c44b56fe9ef9100b0e61/fast_lio_sam_qn/src/fast_lio_sam_qn.cpp#L132) 예제 코드에서 볼 수 있듯이 이전 node의 pose 대비 충분한 pose 차이가 났을 때, 아래와 같이 graph 부분에 대응되는 `gtsam_graph_`를 업데이트하고, 그 graph 구조에 대응되는 초기값(여기서는 `current_keyframe_idx_`번째의 pose)를 `init_esti_`에 업데이트해주는 것을 볼 수 있다.

```cpp
auto variance_vector = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
gtsam::Pose3 pose_from = poseEigToGtsamPose(keyframes_[current_keyframe_idx_ - 1].pose_corrected_eig_);
gtsam::Pose3 pose_to = poseEigToGtsamPose(current_frame_.pose_corrected_eig_);
{
    std::lock_guard<std::mutex> lock(graph_mutex_);
    gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(current_keyframe_idx_ - 1,
                                                        current_keyframe_idx_,
                                                        pose_from.between(pose_to),
                                                        odom_noise));
    init_esti_.insert(current_keyframe_idx_, pose_to);
}
```

참고로, 
* `std::lock_guard<std::mutex>` 부분은 여러 프로세스가 비동기적으로 동시에 돌아갈 때 segmentation fault를 방지하기 위한 부분이다 (잘 모르는 이는 mutex lock에 대해 공부해 보자) 이렇게 그래프/initial value를 생성해주는 부분과 뒷단에서 optimization한 후 initial value를 업데이트해주는 부분에서 동시에 `init_esti_`와 `gtsam_graph_`에 접근하면 segmentation fault가 일어날 수 있기 때문에, 이를 방지해주기 위함이다. 
* 참고로, 2차원에서 covariance의 순은 {trans, trans, rot}의 순인데, 3차원에서 covariance의 값은 {rot, rot, rot, trans, trans, trans} 순이다(즉, {x, y, z, rotation vector} 순이 아닌, rotation 후 translation 순임!)

위의 SLAM 예제가 이해하기 쉽게 되어 있으므로 online SLAM 전체 파이프라인 공부할 겸 살펴보는 것을 추천한다.

## 결론

'쉽게 이해하기'라고 했지만 '깊게 이해하기'가 더 적절한 제목이 아닌가 싶다. 사실, GTSAM에서 이미 제공하는 class를 통해 factor graph optimization을 할 때에는 전혀 몰라도 되는 매우 deep한 부분이다.
그러나 현재 내가 지금 무슨 행위를 하고 있는지 완벽히 이해하고 구현하는 것이 이 ChatGPT에 먹혀버린 시대의 연구자들이 지녀야 할 바람직한 자세이지 않나 싶다.
마지막은 올해 Frank 교수님과 함께 RSS 학회/MIT에서 얘기하고 찍은 사진과 함께, GTSAM library에 힘쓴 Frank 교수님에게 kudos to Frank를 하며 마무리한다. 

<table style="margin: 0 auto; border-collapse: collapse; border: none;">
  <tr style="border: none;">
    <td style="border: none; text-align: center;">
      <img src="/img/w_frank1.jpg" alt="In RSS" width="300"/>
    </td>
    <td style="border: none; text-align: center;">
      <img src="/img/w_frank2.jpg" alt="In MIT" width="300"/>
    </td>
  </tr>
</table>
---

GTSAM Tutorial 시리즈입니다.

{% include post_links_gtsam.html %}
