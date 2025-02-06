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

$$||\boldsymbol{z} - \mathbf{R}_1^{\intercal}(\mathbf{t}_2 - \mathbf{t}_1)||^2$$

measurement function은 아래와 같이 정의된다:

$$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) = \mathbf{R}_1^\intercal(\mathbf{t}_2 - \mathbf{t}_2) \in \mathbb{R}^3  \; \; \; \; \text{(2)}$$

### Step 3 & 4. 전개 및 유도

위의 measurement function에 대한 $$\mathbf{H}_1$$와 $$\mathbf{H}_2$$를 구하기 위래 (1)을 활용해서 전개하면 아래와 같이 되고:

$$h(\boldsymbol{\xi}_1 \oplus \boldsymbol{\delta}_1, \boldsymbol{\xi}_2 \oplus \boldsymbol{\delta}_2) = \left(\mathbf{I} - \left[\boldsymbol{w}\right]_\times\right) \right)$$

$$\mathbf{R}_1^\intercal(\mathbf{t}_2$$
 
$$\mathbf{R}_2\boldsymbol{v}_2 - \mathbf{t}_1 - \mathbf{R}_1\boldsymbol{v}_1)  \; \; \; \; \text{(3)}$$


$$\mathbf{H}_1 = \left[\mathbf{R}_2^\intercal \left(\mathbf{t}_1 - \mathbf{t}_2\right) \;\;\;  \mathbf{R}_2^\intercal \mathbf{R}_1 \right]$$

$$\mathbf{H}_2 = \left[\mathbf{O}_{3 \times 3} \;\;\;  \mathbf{R}_2^\intercal \mathbf{R}_1 \right]$$

---

## Proposed Using Rotation Invariance

$$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) = (\mathbf{R}_1\boldsymbol{z} + \mathbf{t}_1) - \mathbf{t}_2 \in \mathbb{R}^3  \; \; \; \; \text{(3)}$$

위의 factor에서 update는 아래 부분에 대응된다:

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

즉, 이를 수식으로 표현하면  

$$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) = (\mathbf{R}_1\mathbf{z} + \mathbf{t}_1) - \mathbf{t}_2 \in \mathbb{R}^3  \; \; \; \; \text{(4)}$$

즉, 코드 상의 `t2_1`이 $$(\mathbf{R}_1\mathbf{z} + \mathbf{t}_1)$$이고, `t2_2`가 $$\mathbf{t}_2$$가 된다. 
의미를 설명하자면 `p1`의 좌표축 관점에서 본 `p2`의 position을 다시 world frame 관점으로 transformation을 했을 때의 warped point `t2_1`와 `p2`의 translation 값인 `t2_2`와의 차이를 error term으로 정의하였다. 

즉, 코드 상의 `t2_1`이 $$(\mathbf{R}_1\mathbf{z} + \mathbf{t}_1)$$이고, `t2_2`가 $$\mathbf{t}_2$$가 된다. 의미를 설명하자면 `p1`의 좌표축 관점에서 본 `p2`의 위치를 다시 world frame 관점으로 transformation을 했을 때 warped point `t2_1`와 `p2`의 translation 값인 `t2_2`와의 차이이다. 


---

GTSAM Tutorial 시리즈입니다.

{% include post_links_gtsam.html %}