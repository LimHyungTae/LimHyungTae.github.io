---
layout: post
title: 2024-01-04-Modern C++ for Robotics (4) Speed Comparison of TBB Parallel_for Using blocked_range 
subtitle: Speed Comparison of TBB Parallel_for Using blocked_range
tags: [C++, Eigen, Robotics]
comments: true
---

TBB 작성 글을 쓰다가, TBB를의 `parallel_for`을 활용해 병렬처리 기반 for문을 돌 때 `blocked_range`를 사용하면 더 빠르다고 주워들어, 이를 실제로 테스트 해보았다. 

혹시 TBB에 익숙치 않다면 [이 블로그 글](https://chryswoods.com/parallel_c++/parallel_for.html)을 읽어보는 것을 추천한다. 설명이 가장 간결하게 잘 되어있는 듯! 

비교 코드는 아래와 같다.

### 속도 비교 코드
#### Case A. Simple `parallel_for`

```cpp
tbb::parallel_for(0, nPtj, [&](int j) {
        searchKDTree(feature_tree_i, features_[fj][j], corres_K[j], dis_j[j], 1);
        //  corres_K[j][0] is j -> 'i'
        if (corres_K[j][0] >= 0 && corres_K[j][0] < nPti) {
            if (i_to_j_multi_flann[corres_K[j][0]] == -1) {
                searchKDTree(feature_tree_j,
                             features_[fi][corres_K[j][0]],
                             corres_K2[corres_K[j][0]],
                             dis_i[corres_K[j][0]],
                             1);
                i_to_j_multi_flann[corres_K[j][0]] = corres_K2[corres_K[j][0]][0];
            }
            j_to_i_multi_flann[j] = corres_K[j][0];
        }
    });
```

#### Case B. `parallel_for` With `blocked_range`


```cpp
tbb::parallel_for(tbb::blocked_range<int>(0, nPtj),
                       [&](tbb::blocked_range<int> r) {
    // HT comment: 즉 0~nPtj까지의 코드를 여러개의 block으로 나눈 후에, 적절히 thread에 분배하여 처리한다.
    // e.g.  0~30번까지 -> thread 1
    //      30~80번까지 -> thread 2
    //     81~100번까지 -> thread 3
    // ....                   
    for (int j = r.begin(); j < r.end(); ++j) {
        searchKDTree(feature_tree_i, features_[fj][j], corres_K[j], dis_j[j], 1);
        if (corres_K[j][0] >= 0 && corres_K[j][0] < nPti) {
            if (i_to_j_multi_flann[corres_K[j][0]] == -1) {
                searchKDTree(feature_tree_j,
                             features_[fi][corres_K[j][0]],
                             corres_K2[corres_K[j][0]],
                             dis_i[corres_K[j][0]],
                             1);
                i_to_j_multi_flann[corres_K[j][0]] = corres_K2[corres_K[j][0]][0];
            }
            j_to_i_multi_flann[j] = corres_K[j][0];
        }
    }
});
```

### 속도 비교 정량적 결과 
해당 코드가 포함된 matching 부분의 속도를 비교한 것이어서 위의 코드만 pure하게 비교한 것은 아니지만, 속도를 다른 세 데이터셋에서 각각 1,000번 실행한 후 평균 속도는 아래와 같았다.

```commandline
# (unit: sec)
#   Case A vs Case B
0.09677888 vs 0.09638458
0.06199665 vs 0.05407354
0.09957415 vs 0.08153457
```

속도의 차이가 엄청나지는 않지만, 결과를 보면 `blocked_range`를 사용한 경우가 조금 더 빠른 것을 확인할 수 있다.
주의할 것은, 내가 테스트한 경우는 nPtj가 적어도 8000개 이상의, 수가 충분히 큰 point cloud를 다루고 있다는 점이다 (즉 N이 충분히 큰 경우에 대해 테스트했음).

### 속도가 빨라지는 이유?

tbb::parallel_for를 사용할 때 tbb::blocked_range를 사용하는 것과 인덱스를 직접 사용하는 것 사이의 성능 차이는 주로 작업의 특성과 데이터의 크기에 따라 달라지는데, tbb::blocked_range는 데이터를 여러 스레드에 걸쳐 균등하게 분할하는 데 유용하다고 한다. 

그리고 세가지 장점이 있다고 한다.

* **데이터 분할:** 큰 데이터 세트를 처리할 때 blocked_range를 사용하면 데이터를 효율적으로 나누어 각 스레드가 균등한 작업 부하를 갖도록 할 수 있음
* **캐시 최적화:** 데이터를 블록으로 나누어 처리함으로써 캐시 활용도를 높일 수 있으며, 이는 성능 향상을 가져올 수 있음
* **유연한 분할:** blocked_range는 데이터의 분할을 TBB 라이브러리에 위임함으로써, 라이브러리가 실행 환경에 맞게 최적의 분할을 결정할 수 있게 함

반면, `tbb::parallel_for(0, n, [&](int i) {...})` 형태로 인덱스를 직접 사용하는 방법은 간단하고 오버헤드를 덜 야기하기 때문에 작업이 매우 간단하고 각 반복이 거의 동일한 시간이 걸린다면, 복잡한 데이터 분할 없이 인덱스를 직접 사용하는 것이 좋을 수 있다고 한다.

### 결론

결국, 어떤 방법이 더 빠른지는 주어진 작업의 특성, 데이터의 크기 및 구조, 그리고 실행 환경에 따라 달라질 수 있다는 것이다.
개인적인 경험으로는 성능 최적화를 위해서는 두 접근 방식을 **직접** 시험해 보고 빨라지는 것을 채택하는 것이 정답인거 같다.
그리고 데이터의 크기, N,이 그리 크지 않을 때 (한 100개~500개 이하)는 multi-threading을 하는 것이 오히려 느린 경우도 있다. 그러니 꼭 single thread로 짰을 때와 multi thread로 짰을 때 속도 차이가 유의미하게 나는지 확인해보는 습관이 중요한 거 같다.

TBB 잘 쓰고 싶다...오늘의 프로파일링 끗!