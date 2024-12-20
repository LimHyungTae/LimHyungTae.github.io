---
layout: post
title: GTSAM Tutorial 8. Pose3의 BetweenFactor Jacobian 유도
subtitle: Understanding H matrices of Pose3 as an exercise
tags: [Jacobian, GTSAM]
comments: true
---

## Pose3의 Jacobian 구하기

이제 마지막으로 Pose3의 `H1`과 `H2`를 구해보자. Pose2때와 마찬가지로, 아래 세 단계를 따르면 쉽게 수식을 유도해 볼 수 있다. 다만 연습 문제 삼아 연습해볼 수 있도록 자세한 과정은 생략하고, 전개하는 과정에 필요한 테크닉만 hint만 적어두도록 하겠다~~절대 귀찮아서가 아님~~.

### Step 1. Update Function 정의

Translation-rotation 순으로 적었던 2D와는 다르게 GTSAM에서 3D pose를 vector화해서 표현할 때는 rotation-translation 순으로 적는 것에 유의하자(따로 수학적인 이유는 없고, 그냥 GTSAM의 convention이다). 따라서, vector꼴로 표현된 rotation/translation 변화량을 각각 $$\boldsymbol{w} \in \mathbb{R}^3$$, $$\boldsymbol{v} \in \mathbb{R}^3$$라 하면, $$\boldsymbol{\delta} = [\boldsymbol{w}; \boldsymbol{v}]^\intercal \in \mathbb{R}^6$$로 표현이 된다. 그리고 2D 상에서 $$\mathrm{Rot}(\delta\theta) \simeq \mathbf{I} + \hat{\Omega} \delta \theta$$라고 표현했던 것이 3차원에서는 $$\mathbf{I} + [\boldsymbol{w}]_\times$$와 대응되기 때문에(이해가 되지 않는다면 Skew Symmetric matrix 관련 글을 다시 읽어보자), 이를 풀어서 쓰면 아래와 같고:

$$\left[\begin{array}{cc}
\mathbf{R} & \mathbf{t} \\
\mathbf{0} & 1
\end{array}\right]\left[\begin{array}{cc}
\mathbf{I} + [\boldsymbol{w}]_\times & \boldsymbol{v} \\
\mathbf{0} & 1
\end{array}\right]  \; \; \; \; \text{(1)}$$

따라서, update function $$\boldsymbol{\xi} \oplus \boldsymbol{\delta}$$는 다음과 같이 정의된다:

$$\boldsymbol{\xi} \oplus \boldsymbol{\delta} =  
\left[\begin{array}{c}
\mathrm{Log}\left( \mathbf{R} \left(\mathbf{I} + [\boldsymbol{w}]_\times\right) \right) \\ 
\mathbf{t} + \mathbf{R} \boldsymbol{v} 
\end{array}\right] \in \mathbb{R}^6 \; \; \; \; \text{(2)}$$

여기서 $$\mathrm{Log}\left( \cdot \right)$$는 우리가 2차원에서 $$\mathrm{Rot}(\theta)$$을 $$\theta$$로 간단히 표현했던 것 처럼, 3차원 rotation matrix을 3차원 rotation vector로 변환해주는 함수라 받아들이면 된다.
우리가 사용하고자 하는 성질은 $$\mathrm{Log}\left(A\right) = \mathrm{Log}\left(B\right)$$이면 $$A=B$$라는 것이기 때문에, 저 $$\mathrm{Log}\left( \cdot \right)$$ 함수가 어떻게 동작하는지는 수식 전개 시 알 필요는 없다.

### Step 2. Measurement Function $$h(\cdot)$$ 정의

2D의 transformation matrix와 같이 relative pose에 대응하는 measurement function은 아래와 같고:

$$\left(\mathbf{T}^{w}_1\right)^{-1} \mathbf{T}^{w}_2 =
\left[\begin{array}{cc}
\mathbf{R}_1 & \mathbf{t}_1 \\
\mathbf{0} & 1
\end{array}\right]^{-1}\left[\begin{array}{cc}
\mathbf{R}_2 & \mathbf{t}_2 \\
\mathbf{0} & 1
\end{array}\right] \\ = 
\left[\begin{array}{cc}
\mathbf{R}^{\intercal}_1 & -\mathbf{R}^{\intercal}_1\mathbf{t}_1 \\
\mathbf{0} & 1
\end{array}\right]\left[\begin{array}{cc}
\mathbf{R}_2 & \mathbf{t}_2 \\
\mathbf{0} & 1
\end{array}\right] = 
\left[\begin{array}{cc}
\mathbf{R}^{\intercal}_1\mathbf{R}_2 & \mathbf{R}^{\intercal}_1(\mathbf{t}_2 - \mathbf{t}_1) \\
\mathbf{0} & 1
\end{array}\right] \; \; \; \; \text{(3)}$$

따라서 두 pose의 차이에 대한 함수를 vector화 해서 나타내면 

$$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) = 
\left[\begin{array}{c}
\mathrm{Log}\left(\mathbf{R}^{\intercal}_1 \mathbf{R}_2) \right) \\
\mathbf{R}^{\intercal}_1(\mathbf{t}_2 - \mathbf{t}_1) 
\end{array}\right] \in \mathbb{R}^6 \; \; \; \; \text{(4)}$$

로 표현할 수 있다. 2D에서 $$\theta_2 - \theta_1$$로 뺄셈으로 손쉽게 표현할 수 있었던 rotation이 차원이 증가해서 $$\mathrm{Log}\left(\mathbf{R}^{\intercal}_1 \mathbf{R}_2\right)$$라는 조금 복잡한 형태가 되었을 뿐, rotation의 차이를 표현하고자 하는 기저 원리는 같다.

### Step 3. $$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) \oplus \boldsymbol{\delta}$$와 $$h(\boldsymbol{\xi}_1 \oplus \boldsymbol{\delta}_1, \boldsymbol{\xi}_2 \oplus \boldsymbol{\delta}_2)$$ 전개하기 

자, 여기서부터는 스스로 한 번 해보는 것으로 하자!

**Step 3-1. $$h(\boldsymbol{\xi}_1 \oplus \boldsymbol{\delta}_1, \boldsymbol{\xi}_2 \oplus \boldsymbol{\delta}_2)$$ 전개하기**

**Step 3-2. $$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) \oplus \boldsymbol{\delta}$$ 전개하기**

전개를 하는 것 자체에는 어려움이 없을 것이다.


### Step 4. 수식 전개해서 `H1`, `H2`에 대응되는 값 유도

이제 $$h(\boldsymbol{\xi}_1 \oplus \boldsymbol{\delta}_1, \boldsymbol{\xi}_2 \oplus \boldsymbol{\delta}_2)$$와 $$h(\boldsymbol{\xi}_1, \boldsymbol{\xi}_2) \oplus \boldsymbol{\delta}$$의 translation/rotation 요소가 각각 같다고 하고 풀면 된다. 

**Hints**

* $$\mathrm{Log}\left(A\right) = \mathrm{Log}\left(B\right)$$이면 $$A=B$$.
* 미소 변화량 두개가 곱해지면 해당 term이 무시 가능해진다는 것을 활용하자. 즉, $$[\boldsymbol{w}_1]_\times$$과 $$\boldsymbol{v}_1$$, 
 혹은 $$[\boldsymbol{w}_1]_\times$$과 $$\boldsymbol{v}_2$$가 함께 곱해져 있는 term은 그 크기가 다른 term보다 월등히 작아지므로 무시 가능함.
* $$\mathbf{R}[\boldsymbol{w}]_\times \mathbf{R}^\intercal = [\mathbf{R} \boldsymbol{w}]_\times$$, $$\mathbf{R}^\intercal[\boldsymbol{w}]_\times \mathbf{R} = [\mathbf{R}^\intercal \boldsymbol{w}]_\times$$.
* $$[\boldsymbol{w}]_\times \boldsymbol{v} = - [\boldsymbol{v}]_\times \boldsymbol{w}$$. 여기서 $$\boldsymbol{v}$$와 $$\boldsymbol{w}$$는 임의의 3차원 vector를 의미함.
* $$\mathbf{R} \mathbf{R}^\intercal = \mathbf{I}$$. 힌트를 좀 더 주자면, translation 전개 시 $$\mathbf{R}_2^\intercal [\boldsymbol{w}]_\times \mathbf{R}_1 = \mathbf{R}_2^\intercal[\boldsymbol{w}]_\times \mathbf{R}_2 \mathbf{R}_2^\intercal \mathbf{R}_1 = [\mathbf{R}_2^\intercal \boldsymbol{w}]_\times \mathbf{R}_2^\intercal \mathbf{R}_1$$의 꼴로 표현하기 위해 필요함.
* $$[\boldsymbol{w}_a]_\times = [\boldsymbol{w}_b]_\times + [\boldsymbol{w}_c]_\times$$이면 $$\boldsymbol{w}_a = \boldsymbol{w}_b + \boldsymbol{w}_c$$.

최종적으로 풀면, 수식이 아래와 같이 나와야 한다:

$$
\mathbf{H}_1=-\left[\begin{array}{cc}
\mathbf{R}_2^\intercal \mathbf{R}_1 & \mathbf{0} \\
{\left[\mathbf{R}_2^\intercal \left(\mathbf{t}_1 - \mathbf{t}_2\right)\right]_{\times}}\mathbf{R}_2^\intercal \mathbf{R}_1 & \mathbf{R}_2^\intercal \mathbf{R}_1
\end{array}\right], \; \; \mathbf{H}_2 = \mathbf{I}_{6 \times 6}$$

---

GTSAM Tutorial 시리즈입니다.

{% include post_links_gtsam.html %}