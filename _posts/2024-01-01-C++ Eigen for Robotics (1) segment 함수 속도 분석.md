---
layout: post
title: 2024-01-01-C++ Eigen for Robotics (1) segment 함수 속도 분석
subtitle: Comparison of element-wise operation and segment<>()
tags: [C++, Eigen, Robotics]
comments: true
---

오늘은 histogram update를 하는 코드를 작성하다가 Eigen::Matrix의 segment<>() 함수를 사용하면 어떻게 되는지 궁금해서 속도를 비교해보았다.
사실 ChatGPT가 segment<>() 함수를 쓰는 게 element-wise로 update하는 거보다 빠르다고 말했는데, 최근에는 직접 돌려보기 전까지는 믿지 않는 병이 생겨서, 이를 실제로 테스트 해보았다.

아래 두 코드 스니펫의 속도를 분석해보았다:

### Case1. Element-wise operation

```cpp
auto t0 = std::chrono::high_resolution_clock::now();
Eigen::VectorXf histogram_total = Eigen::VectorXf::Zero(33);

for (int i = 0; i < 10000; i++) {
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
std::cout << "Time taken: "
          << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() /
              1000000.0 << " sec" << std::endl;
```

### Case2. Segment<>() operation

```cpp
auto t3 = std::chrono::high_resolution_clock::now();
Eigen::VectorXf histogram_total = Eigen::VectorXf::Zero(33);
for (int i = 0; i < 10000; i++) {
    Eigen::VectorXf hist_f1 = Generate11Vector();
    Eigen::VectorXf hist_f2 = Generate11Vector();
    Eigen::VectorXf hist_f3 = Generate11Vector();

    histogram_total.segment<11>(0) += hist_f1;
    histogram_total.segment<11>(11) += hist_f2;
    histogram_total.segment<11>(22) += hist_f3;
}
auto t4 = std::chrono::high_resolution_clock::now();
std::cout << "Time taken: "
    << std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() /
       1000000.0 << " sec" << std::endl;
```


**놀랍게도** 속도는 약 0.151509 sec vs 0.153506 sec. 즉, segment를 쓰는 게 오히려 좀 더 느리다.
사실 레거시 코드가 case 1으로 되어있는데, 역시 미리 코드를 짠 사람들이 이렇게 짠 데에는 다 이유가 있는 법...
그리고 ChatGPT를 너무 맹신하지 말자. 그럴싸하게 말하지만, 생각보다 틀린게 많음.

오늘의 새해 끄적끄적 끝!