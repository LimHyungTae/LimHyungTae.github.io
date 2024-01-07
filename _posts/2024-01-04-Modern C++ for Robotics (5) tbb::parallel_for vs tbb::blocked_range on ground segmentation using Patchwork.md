---
layout: post
title: 2024-01-04-Modern C++ for Robotics (5) tbb::parallel_for vs tbb::blocked_range on ground segmentation using Patchwork 
subtitle: Speed Comparison of tbb::Parallel_for vs tbb::blocked_range
tags: [C++, tbb, parallel programming, robotics]
comments: true
---

추가적으로, open source되어 있는 ground segmentation 알고리즘인 [Patchwork](https://github.com/LimHyungTae/patchwork)에서도 TBB를 적용해보았다.
그와 더불어 `tbb::parallel_for`과 `tbb::blocked_range`로 했을 때의 속도 비교를 해보았다.

그 결과, 약 3번 정도 KITTI Seq. 04에서의 속도 비교를 했을 때, 소요되는 시간은 다음과 같다 (04를 고른 이유는 가장 sequence가 짧기 때문에 04에서만 테스트해봤다).


```commandline
# (unit: Hz)
# tbb::parallel_for vs tbb::blocked_range
 117.34120452937404 vs 127.7070597970635
 116.97269208405731 vs 126.7121742920252
 116.44076084885228 vs 128.2542829423367
```

이 역시도 결과를 보면 `blocked_range`를 사용한 경우가 조금 더 빠른 것을 확인할 수 있다.
Patchwork code에서 `num_patches`가 그렇게 크지 않은 숫자임에도 불구하고, tbb::blocked_range가 더 높은 FPS를 보여주었다.

이유를 생각해보면 이전 글에서 **데이터 분할**과 **유연한 분할**이 관련이 있을 것 같다. 
자세히 설명하자면, 각 bin마다 할당된 points의 수가 다르고 iteration 횟수도 다르기 때문에, ground points를 뽑는 시간이 각 bin 마다 다르게 들 수 있다.
하지만 `tbb::parallel_for`은 각 bin을 처리하는데 걸리는 thread의 부하를 크게 고려하지 않은 채로 각 i번째 bin을 그때 그때 쉬고 있는 thread에 처리하라고 greedy하게 명령하는 반면,
`tbb::blocked_range`는 각 bin마다 소요되는 시간을 어느정도 고려해 유연하게 for loop를 적절히 여러 개의 block으로 나눠 thread에 할당하기 때문에, 좀 더 속도를 향상 시킬 수 있는 게 아닌가 싶다.

### 결론

N이 작은 경우에도 tbb::blocked_range가 더 빠른 것이 의외였다. `tbb::parallel_for(0, n, [&](int i) {...})`는 그럼 언제 써야 하는지 혹시 아는 이가 있다면 꼭 좀 나에게 알려 줬으면 좋겠다,,,

