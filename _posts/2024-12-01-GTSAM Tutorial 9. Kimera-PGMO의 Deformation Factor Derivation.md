---
layout: post
title: GTSAM Tutorial 9. Kimera-PGMO의 Deformation Factor Derivation.md
subtitle: Understanding DeformationEdgeFactor
tags: [Jacobian, GTSAM]
comments: true
---

## Kimera-PGMO의 Deformation Factor Jacobian 이해하기

자, 이제 GTSAM을 잘 이해했다면 실전 연습으로 Yun이 짜둔 `DeformationEdgeFactor`를 분석해보자. 앞의 글들을 잘 이해했다면, 이 factor의 동작원리를 이해할 수 있어야 한다(주의: 내부적으로 리팩토링 중인 코드여서, 25년 2월 5일 기준 official repo의 코드와 100% 일치하지 않을 수 있다).


```cpp
class DeformationEdgeFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 private:
  gtsam::Point3 measurement_;

 public:
  DeformationEdgeFactor(gtsam::Key node1_key,
                        gtsam::Key node2_key,
                        const gtsam::Point3& measurement,
                        gtsam::SharedNoiseModel model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model,
                                                             node1_key,
                                                             node2_key),
        measurement_(measurement) {}

  DeformationEdgeFactor(gtsam::Key node1_key,
                        gtsam::Key node2_key,
                        const gtsam::Pose3& node1_pose,
                        const gtsam::Point3& node2_point,
                        gtsam::SharedNoiseModel model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model,
                                                             node1_key,
                                                             node2_key) {
    measurement_ =
        node1_pose.rotation().inverse().rotate(node2_point - node1_pose.translation());
  }

  virtual ~DeformationEdgeFactor() {}

  gtsam::Vector evaluateError(const gtsam::Pose3& p1,
                              const gtsam::Pose3& p2,
                              GtsamJacobianType H1 = JACOBIAN_DEFAULT,
                              GtsamJacobianType H2 = JACOBIAN_DEFAULT) const override {
    // position of node 2 in frame of node 1
    gtsam::Matrix H_R1, H_t1, H_t2;
    gtsam::Rot3 R1 = p1.rotation();
    gtsam::Point3 t1 = p1.translation(H_t1);
    // New position of node 2 according to deformation p1 of node 1
    gtsam::Point3 t2_1 = t1 + R1.rotate(measurement_, H_R1);
    gtsam::Point3 t2_2 = p2.translation(H_t2);

    // Calculate Jacobians
    if (H1) {
      Eigen::MatrixXd Jacobian_1 = Eigen::MatrixXd::Zero(3, 6);
      Jacobian_1.block<3, 3>(0, 0) = H_R1;
      Jacobian_1 = Jacobian_1 + H_t1;
      *H1 = Jacobian_1;
    }

    if (H2) {
      Eigen::MatrixXd Jacobian_2 = Eigen::MatrixXd::Zero(3, 6);
      Jacobian_2 = Jacobian_2 - H_t2;
      *H2 = Jacobian_2;
    }

    return t2_1 - t2_2;
  }

  inline gtsam::Point3 measurement() const { return measurement_; }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new DeformationEdgeFactor(*this));
  }
};
```

### Step 1. Update Function 정의

앞서 [Pose3 글](https://limhyungtae.github.io/2024-12-01-GTSAM-Tutorial-8.-Pose3%EC%9D%98-BetweenFactor-Jacobian-%EC%9C%A0%EB%8F%84/)에서 살펴보았듯이(잘 이해가 안되면 6번째 글부터 차근차근 다시 읽어보자), 3차원 상에서의 Pose를 vector form인 $$\boldsymbol{\xi} \in \mathbb{R}^6$$으로 표현하면,
update function $$\boldsymbol{\xi} \oplus \boldsymbol{\delta}$$는 다음과 같이 정의된다:

$$\boldsymbol{\xi} \oplus \boldsymbol{\delta} =  
\left[\begin{array}{c}
\mathrm{Log}\left( \mathbf{R} \left(\mathbf{I} + [\boldsymbol{w}]_\times\right) \right) \\ 
\mathbf{t} + \mathbf{R} \boldsymbol{v} 
\end{array}\right] \in \mathbb{R}^6 \; \; \; \; \text{(1)}$$

---

## 원래대로라면...
### Step 2. Measurement Function $$h(\cdot)$$ 정의

그런데 deformation factor에서는 재밌는 technique이 사용되었다(~~나만 재밌을지도~~).
먼저 원래의 measurement function을 살펴보자. 원래 error term은 아래와 같이 정의되어서: 

$$\boldsymbol{e} = ||\boldsymbol{z} - \mathbf{R}_1^{\intercal}(\mathbf{t}_2 - \mathbf{t}_1)||^2  \; \; \; \; \text{(2)}$$

measurement function은 아래와 같이 정의된다:

$$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) = \mathbf{R}_1^\intercal(\mathbf{t}_2 - \mathbf{t}_2) \in \mathbb{R}^3  \; \; \; \; \text{(3)}$$

### Step 3 & 4. 전개 및 유도

위의 measurement function에 대한 $$\mathbf{H}_1$$와 $$\mathbf{H}_2$$를 구하기 위래 (1)을 활용해서 전개하면 아래와 같이 되고:

$$h(\boldsymbol{\xi}_1 \oplus \boldsymbol{\delta}_1, \boldsymbol{\xi}_2 \oplus \boldsymbol{\delta}_2) = \left(\mathbf{I} - \left[\boldsymbol{w}\right]_\times\right)\mathbf{R}_1^\intercal(\mathbf{t}_2 + \mathbf{R}_2\boldsymbol{v}_2 - \mathbf{t}_1 - \mathbf{R}_1\boldsymbol{v}_1)  \; \; \; \; \text{(4)}$$

a) $$\left[\boldsymbol{w}_1\right]_\times$$ term과 $$\boldsymbol{v}$$이 서로 곱해지는 term은 다른 term에 비해 크기가 월등히 작아지므로 무시, b) $$[\boldsymbol{w}]_\times \boldsymbol{v} = - [\boldsymbol{v}]_\times \boldsymbol{w}$$의 성질을 이용해서 전개하면 (4)은 최종적으로 아래와 같이 전개된다:

$$h(\boldsymbol{\xi}_1 \oplus \boldsymbol{\delta}_1, \boldsymbol{\xi}_2 \oplus \boldsymbol{\delta}_2) = \mathbf{R}_1^\intercal(\mathbf{t}_2 - \mathbf{t}_1) + \mathbf{R}_1^\intercal \mathbf{R}_2 \boldsymbol{v}_2 - \boldsymbol{v}_1 - \left[\boldsymbol{w}_1\right]_\times \mathbf{R}_1^\intercal(\mathbf{t}_2 - \mathbf{t}_1)  \\
= h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2)  + \mathbf{R}_1^\intercal \mathbf{R}_2 \boldsymbol{v}_2 - \boldsymbol{v}_1 + \left[\mathbf{R}_1^\intercal(\mathbf{t}_2 - \mathbf{t}_1) \right]_\times \boldsymbol{w}_1 \; \; \; \; \text{(5)}$$

따라서 $$\mathbf{H}_1$$과 $$\mathbf{H}_2$$는 아래와 같이 정의된다(다시금 강조하지만 GTSAM에서의 Pose3는 (rotation vector, translation vector)의 순으로 state가 구성되어 있다): 

$$\mathbf{H}_1 = \left[\mathbf{R}_1^\intercal \left(\mathbf{t}_2 - \mathbf{t}_1\right) \;\;\; -\mathbf{I}_{3 \times 3} \right] \in \mathbb{R}^{3 \times 6}, \; \; \; \mathbf{H}_2 = \left[\mathbf{O}_{3 \times 3} \;\;\;  \mathbf{R}_1^\intercal \mathbf{R}_2 \right] \in \mathbb{R}^{3 \times 6}  \; \; \; \; \text{(6)}$$

---

## Proposed Using Rotation Invariance

### Step 2. Error term 정의

그러나, 위의 코드를 다시 살펴보자. 위의 factor에서 아래와 같이 error를 계산한다:

```cpp
    gtsam::Matrix H_R1, H_t1, H_t2;
    gtsam::Rot3 R1 = p1.rotation();
    gtsam::Point3 t1 = p1.translation(H_t1);
    // New position of node 2 according to deformation p1 of node 1
    gtsam::Point3 t2_1 = t1 + R1.rotate(measurement_, H_R1);
    gtsam::Point3 t2_2 = p2.translation(H_t2); 
    .
    .
    .
    return t2_1 - t2_2;
```

즉, 이를 수식으로 표현하면 error term이 아래와 같이 표현된다:

$$\boldsymbol{e} = || (\mathbf{R}_1\boldsymbol{z} + \mathbf{t}_1) - \mathbf{t}_2 ||^2  \; \; \; \; \text{(7)}$$

코드 상의 `t2_1`이 $$(\mathbf{R}_1\boldsymbol{z} + \mathbf{t}_1)$$이고, `t2_2`가 $$\mathbf{t}_2$$가 된다. 
의미를 설명하자면 `p1`의 좌표축 관점에서 본 `p2`의 position을 다시 world frame 관점으로 transformation을 했을 때의 warped point `t2_1`와 `p2`의 translation 값인 `t2_2`와의 차이를 error term으로 정의하였다. 

그런데 의아한 점이 있다. 우리가 예상한 error term은 (2)인데, 왜 (7)과 같이 계산할 수 있는 걸까? 여기서 주로 활용되는 technique이 'rotation invariance'이다. 
Rotation invariance은 위와 같이 거리 차이를 나타내는 error term에는 아무 rotation matrix을 곱해도 error의 크기가 불변한다는 것을 뜻한다.
비유를 통해 설명하자면, 어떤 막대기가 있고 그 막대기의 두 끝점 사이의 길이 차가 error 값이라고 하면, 이 막대기를 어느 방향으로 회전시키더라도 두 끝점 사이의 길이 차는 동일하다는 것이다.
이를 기반으로 (2)의 절댓값 안에 rotation matrix $$\mathbf{R}_1$$을 곱하면 (7)이 되는 것을 볼 수 있다.

### Step 3 & 4. 전개 및 유도

그럼 이 짓을 왜 하는 걸까? 마지막 부분에 답이 있으니, 궁금증을 뒤로 하고 우선 $$\mathbf{H}_1$$과 $$\mathbf{H}_2$$를 구해보자. $$\boldsymbol{e}$$의 절댓값 안에 있는 식에 $$\boldsymbol{\xi}_1 \oplus \boldsymbol{\delta}_1$$와 $$\boldsymbol{\xi}_2 \oplus \boldsymbol{\delta}_2$$를 넣고 전개해보자.
그럼 (7)은 아래와 같은 수식이 된다:

$$\mathbf{R}_1(\mathbf{I} + \left[\boldsymbol{w}_1\right]_\times)\boldsymbol{z} + \mathbf{t}_1 + \mathbf{R}_1\boldsymbol{v}_1 - \mathbf{t}_2 - \mathbf{R}_2\boldsymbol{v}_2 \\
= \mathbf{R}_1 \boldsymbol{z} + \mathbf{R}_1 \left[\boldsymbol{w}_1\right]_\times \boldsymbol{z} + \mathbf{t}_1 + \mathbf{R}_1\boldsymbol{v}_1 - \mathbf{t}_2 - \mathbf{R}_2\boldsymbol{v}_2 \\
= (\mathbf{R}_1 \boldsymbol{z} + \mathbf{t}_1 - \mathbf{t}_2) + \mathbf{R}_1 \left[\boldsymbol{w}_1\right]_\times\mathbf{R}_1^\intercal \mathbf{R}_1 \boldsymbol{z} + \mathbf{R}_1\boldsymbol{v}_1 - \mathbf{R}_2\boldsymbol{v}_2 \\
= (\mathbf{R}_1 \boldsymbol{z} + \mathbf{t}_1 - \mathbf{t}_2) - \left[ \mathbf{R}_1 \boldsymbol{z}\right]_\times \mathbf{R}_1 \boldsymbol{w}_1 + \mathbf{R}_1\boldsymbol{v}_1 - \mathbf{R}_2\boldsymbol{v}_2 \; \; \; \; \text{(8)} $$

따라서 (8)에 따라 $$\mathbf{H}_1$$과 $$\mathbf{H}_2$$는 아래와 같이 정의된다: 

$$\mathbf{H}_1 = \left[ - \left[ \mathbf{R}_1 \boldsymbol{z}\right]_\times \mathbf{R}_1 \;\;\; \mathbf{R}_1 \right] \in \mathbb{R}^{3 \times 6}, \; \; \; \mathbf{H}_2 = \left[\mathbf{O}_{3 \times 3} \;\;\; -\mathbf{R}_2 \right] \in \mathbb{R}^{3 \times 6}  \; \; \; \; \text{(9)}$$



## 디스커션 & 결론

최종적으로 (6)과 (9)를 살펴보면, 우리가 풀고자 하는 문제는 동일하지만, 최종적으로 계산되는 Jacobian이 달라지는 것을 볼 수 있다.
위의 행위의 장점은 (6)에서는 matrix multiplication을 $$\mathbf{R}_2^\intercal \left(\mathbf{t}_1 - \mathbf{t}_2\right)$$, $$\mathbf{R}_2^\intercal \mathbf{R}_1$$ (+ $$\mathbf{R}_2^\intercal$$ 계산해두기)를 두 번 수행하는 반면, 
(9)에서는 $$- \left[ \mathbf{R}_1 \boldsymbol{z}\right]_\times \mathbf{R}_1$$ 한 번만 수행하면 되는 장점이 있다(note, (9)에서 $$\mathbf{R}_1$$과 $$\mathbf{R}_2$$는 이미 코드 내의 Pose3인 `p1`, `p2`가 각각 member 변수로 지니고 있기 때문에, 추가적인 계산이 들지 않는다).

'고작 그게...전부?'라고 생각할 수 있지만, 우리의 SLAM 문제에서 위의 factor가 1,000개, 더 나아가 10,000개가 된다고 가정해보자. 그러면 optimization의 매 iteration 마다 matrix multiplication을 1,000번, 10,000번을 덜해도 된다는 뜻이다. 
즉, 동일한 optimization을 수행하는데 iteration 횟수 * factor의 갯수만큼 연산을 아낄 수 있다. 
참고로, 경험적으로 연구실 레벨의 스케일인 실내를 mapping했을 때 약 4,000개의 `DeformationEdgeFactor`가 형성되는 것을 확인했다.  

이처럼 본 연구에서는 rotation invariance를 활용하여 동일한 optimization을 수행하면서도 연산량을 효과적으로 절감할 수 있음을 확인해보았다.


---

GTSAM Tutorial 시리즈입니다.

{% include post_links_gtsam.html %}