---
layout: post
title: Modern C++ for Robotics (2) Eigen::segment<>()와 std::transform 속도 비교 
subtitle: Comparison of element-wise operation and segment<>()
tags: [C++, Eigen, Robotics]
comments: true
---

오늘은 histogram update를 하는 코드를 작성하다가 Eigen::Matrix의 segment<>() 함수를 사용하면 어떻게 되는지 궁금해서 속도를 비교해보았다.
사실 ChatGPT가 segment<>() 함수를 쓰는 게 element-wise로 update하는 거보다 빠르다고 말했는데, 최근에는 직접 돌려보기 전까지는 믿지 않는 병이 생겨서, 이를 실제로 테스트 해보았다.

아래 두 코드 스니펫의 속도를 분석해보았다:

### Case 1. Element-wise operation

```cpp
auto t0 = std::chrono::high_resolution_clock::now();
Eigen::VectorXf histogram_total = Eigen::VectorXf::Zero(33);

for (int i = 0; i < 100000; i++) {
    Eigen::VectorXf hist_f1 = Generate11Vector();
    Eigen::VectorXf hist_f2 = Generate11Vector();
    Eigen::VectorXf hist_f3 = Generate11Vector();

    for (Eigen::MatrixXf::Index f1_i = 0; f1_i < nr_bins_f1; ++f1_i) {
        histogram_total[f1_i] += hist_f1(f1_i);
    }
    for (Eigen::MatrixXf::Index f2_i = 0; f2_i < nr_bins_f2; ++f2_i) {
        histogram_total[f2_i + nr_bins_f1] += hist_f2(f2_i);
    }
    for (Eigen::MatrixXf::Index f3_i = 0; f3_i < nr_bins_f3; ++f3_i) {
        histogram_total[f3_i + 22] += hist_f3(f3_i);
    }
}
auto t1 = std::chrono::high_resolution_clock::now();
std::cout << "Element-wise operation is taken: "
          << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() /
              1000000.0 << " sec" << std::endl;
```

### Case 2. Segment<>() operation

```cpp
auto t2 = std::chrono::high_resolution_clock::now();
Eigen::VectorXf histogram_total = Eigen::VectorXf::Zero(33);
for (int i = 0; i < 100000; i++) {
    Eigen::VectorXf hist_f1 = Generate11Vector();
    Eigen::VectorXf hist_f2 = Generate11Vector();
    Eigen::VectorXf hist_f3 = Generate11Vector();

    histogram_total.segment<11>(0) += hist_f1;
    histogram_total.segment<11>(11) += hist_f2;
    histogram_total.segment<11>(22) += hist_f3;
}
auto t3 = std::chrono::high_resolution_clock::now();
std::cout << "segment<>() taken: "
    << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() /
       1000000.0 << " sec" << std::endl;
```

### Case 3. std::transform()

```cpp
auto t4 = std::chrono::high_resolution_clock::now();
Eigen::VectorXf histogram_total = Eigen::VectorXf::Zero(33);

for (int i = 0; i < 100000; i++) {
    Eigen::VectorXf hist_f1 = Generate11Vector();
    Eigen::VectorXf hist_f2 = Generate11Vector();
    Eigen::VectorXf hist_f3 = Generate11Vector();

    std::transform(histogram_total.data(), histogram_total.data() + nr_bins_f1,
                   hist_f1.data(), histogram_total.data(), std::plus<float>());
    std::transform(histogram_total.data() + nr_bins_f1, histogram_total.data() + nr_bins_f1 + nr_bins_f2,
                   hist_f2.data(), histogram_total.data() + nr_bins_f1, std::plus<float>());
    std::transform(histogram_total.data() + 22, histogram_total.data() + 22 + nr_bins_f3,
                   hist_f3.data(), histogram_total.data() + 22, std::plus<float>());
}

auto t5 = std::chrono::high_resolution_clock::now();
std::cout << "std::transform is taken: "
          << std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count() /
              1000000.0 << " sec" << std::endl;
```

**놀랍게도** 돌릴 때 마다 속도 차이가 꽤 난다....그냥 무지성 N번 돌린 결과를 공유한다:
```commandline
# 1st trial  
Element-wise operation is taken: 1.58176 sec
segment<>() is taken: 1.55453 sec
std::transform is taken: 1.55213 sec
# 2nd trial  
Element-wise operation is taken: 1.62715 sec
segment<>() is taken: 1.61361 sec
std::transform is taken: 1.54134 sec
# 3rd trial
Element-wise operation is taken: 1.62649 sec
segment<>() is taken: 1.6119 sec
std::transform is taken: 1.60347 sec
# 4th trial
Element-wise operation is taken: 1.63739 sec
segment<>() is taken: 1.57691 sec
std::transform is taken: 1.61396 sec
# 5th trial
Element-wise operation is taken: 1.56396 sec
segment<>() is taken: 1.57683 sec
std::transform is taken: 1.5534 sec
# 6th trial 
Element-wise operation is taken: 1.55734 sec
segment<>() is taken: 1.56035 sec
std::transform is taken: 1.55791 sec
# 7th trial 
Element-wise operation is taken: 1.55245 sec
segment<>() is taken: 1.5441 sec
std::transform is taken: 1.54315 sec
# 8th trial
Element-wise operation is taken: 1.57594 sec
segment<>() is taken: 1.55151 sec
std::transform is taken: 1.56954 sec
```

보면 **경향성이 전혀 없다**. 아마 CPU 상에서 cache가 그때그때 다르게 저장된다던가, 동작하는 과정에서 그때그때마다 성능 차이가 많이 나는 듯하다.

세 가지에 대해서 Bard에게 물어봤더니, 아래와 같이 답변해 주었다 (무튼 std::transform이 제일 좋다고 함):

* **Element-wise Operations**: Accessing elements individually incurs overhead for memory access and loop iterations. Might not fully utilize vectorization capabilities for smaller vectors like your 11-element ones. Performance can be sensitive to cache utilization patterns.
 
* **segment<>()**: Designed for efficient subvector operations. Can leverage Eigen's optimizations for vector operations. Might still involve some overhead for segment creation and access. Potentially better cache utilization than element-wise operations.
 
* **std::transform**: Can be highly optimized by Eigen for vector operations. Might have advantages in terms of cache utilization for specific patterns. Overhead of function calls might be minimal compared to Eigen's internal optimizations.

TBB와의 호환성관련해서도 물어봤더니, 아래와 같이 대답해 주었다:

```commandline
TBB는 데이터의 동시성과 스레드 간의 작업 분할을 관리하는데 강력합니다. 세 가지 접근 방식 모두 histogram_total 배열에 대한 병렬 쓰기를 포함하기 때문에, TBB를 사용하여 이러한 작업을 병렬화하려면 동시성 제어 메커니즘이 필요합니다.

이를 고려할 때, 가장 TBB 친화적인 접근 방식은 **세 번째 방법 (std::transform 사용)**입니다. 이 방법은 함수형 프로그래밍 패러다임을 따르며, 각 변환 단계가 독립적이어서 TBB의 병렬 알고리즘으로 변환하기 적합합니다. std::transform을 TBB의 tbb::parallel_for_each나 유사한 기능으로 대체하고, histogram_total에 대한 동시 쓰기를 관리하기 위한 적절한 동시성 제어 메커니즘을 사용하면 효율적인 병렬 처리가 가능할 것입니다.
```

함수형 프로그래밍은 모던 C++의 큰 특징인데, [여기](https://mangkyu.tistory.com/111) 설명이 잘 되어 있으니 참고하면 좋을듯 하다.

---

{% include post_links_modern_cpp.html %}
