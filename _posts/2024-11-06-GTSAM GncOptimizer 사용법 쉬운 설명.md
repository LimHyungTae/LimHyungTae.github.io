---
layout: post
title: GTSAM GncOptimizer 사용법 쉬운 설명
subtitle:
tags: [GTSAM, C++]
comments: true
---

## Introduction

Optimization 기법 중 graduated non-convexity (GNC)라는 기법이 있는데, 
현재 GTSAM에 이 GNC를 기반으로 해서 optimization을 하는 방법을 제공하고 있다 (엄밀히 말하자면 Luca가 GncOpimizer를 GTSAM library에 업데이트했다).
그래서 간략한 사용법 코드 snippet을 기록해두고자 한다.

## 코드 구조

```angular2html
gtsam::LevenbergMarquardtParams lmParams;
lmParams.setVerbosityLM("SUMMARY");
gtsam::GncParams<gtsam::LevenbergMarquardtParams> gncParams(lmParams);
gncParams.setKnownInliers(known_inlier_indices);
gncParams.setKnownOutliers(known_outlier_indices);
// Create GNC optimizer
gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>
    gnc_optimizer(input_nfg, input_values, gncParams);
gnc_optimizer.setInlierCostThresholdsAtProbability(0.7);

result = gnc_optimizer.optimize();
```

위에서 보이는 것처럼, Optimization class를 `GNCOptimizer`로 선언하면 된다.

여기서 가장 중요한 것은 `setInlierCostThresholdsAtProbability` 함수이다.
이 함수는 특정 확률에서 inlier와 outlier를 구분하는 cost threshold를 설정하는데, 
이 확률 값이 높을수록 더 strict한 기준을 적용하여 outlier를 filtering한다.
예를 들어, 높은 확률 (예: 0.7 이상)을 설정하면 threshold가 증가하여 더 많은 edge들이 outlier로 분류될 가능성이 커지고, 더 conservative하게 동작한다. 
반대로 낮은 확률을 설정하면 threshold가 낮아져 더 많은 edge들이 inlier로 간주될 수 있어 상대적으로 덜 conservative하게 동작하게 된다.


