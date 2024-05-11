---
layout: post
title: Modern C++ for Robotics 10. std::copy_if() 쉬운 설명 & 예제
subtitle: Easy Explanation & Examples of Conditional Algorithms
tags: [C++, Eigen, Robotics]
comments: true
---

이 post에서는 STL의 조건부 알고리즘(conditional algorithms)에 대해 알아본다.
함수 이름들이 굉장히 직관적으로 잘 지어져있듯이, 이 조건부 알고리즘들은 컨테이너의 요소들을 lambda function의 조건이 참인지 거짓인지 판별한 후, 
그 결과에 따라 복사, 삭제, 탐색, 교체하는 역할을 한다.
주로 많이 쓰이는 것들은 아래와 같이 std::copy_if(), std::remove_if(), std::find_if(), std::replace_if()가 있다.
코드를 읽어보면 어떻게 동작하는 지 바로 알 수 있기 때문에, 자세한 설명은 생략하고 예제 위주로 공유하고자 한다. 

## std::copy_if()

```cpp
#include <algorithm>
#include <vector>
#include <iostream>

int main() {
    std::vector<int> src = {1, 2, 3, 4, 5};
    std::vector<int> dst;
    dst.reserve(src.size());
    std::copy_if(src.begin(), src.end(), std::back_inserter(dst), [](int x) { return x % 2 == 0; });

    // 출력: 2 4
    for (auto i : dst) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
}
```

## 로보틱스에서 활용 사례

먼저, Open3D의 코드 내 [메모리를 release하는 부분](https://github.com/isl-org/Open3D/blob/008cfb7fb9831f84b6026617e9aa60ab814c701a/cpp/open3d/core/MemoryManagerCached.cpp#L236)에서 다음과 같이 사용되고 있다:

```cpp
std::set<std::shared_ptr<RealBlock>, SizeOrder<RealBlock>>
                releasable_real_blocks;
                
std::copy_if(real_blocks_.begin(), real_blocks_.end(),
    std::inserter(releasable_real_blocks,
                  releasable_real_blocks.begin()),
    [this](const auto& r_block) { return IsReleasable(r_block); });
```
앞서 살펴본 예제에서는 `std::back_inserter`를 통해 vector의 뒷편에 요소를 삽입했다면, 
이 코드에서는 `std::inserter`를 통해 set에 삽입하게끔 작성되어 있다. 
set의 경우에는 내부적으로 정렬 순서에 따라서 요소를 배치하기 때문에, `std::back_inserter `를 지원하지 않는다.
아래는 이해를 돕기 위해 내가 작성한 예시 코드이다:

```cpp
#include <algorithm>
#include <vector>
#include <set>
#include <iostream>

int main() {
    std::vector<int> src = {1, 2, 3, 4, 5};
    std::set<float> dst = {1.5, 3.2};

    // Error!
    // std::copy_if(src.begin(), src.end(), std::back_inserter(dst), [](int x) { return x % 2 == 0; });

    std::copy_if(src.begin(), src.end(), std::inserter(dst, dst.begin()), [](int x) { return x % 2 == 0; });

    // 출력: 1.5 2 3.2 4
    for (auto i : dst) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
}
```

추가적으로 알 수 있는 것은, 다른 컨테이너(vector와 set)간의 데이터 교환이더라도 함수형 프로그래밍은 유동적으로 사용이 가능하다는 것을 확인할 수 있다.


또한 아래 [OpenVINO 코드](https://github.com/ethz-asl/openvino/blob/b4ad7a1755b4799f92ef042bacc719ec3c0c1cbd/inference-engine/src/vpu/common/src/ngraph/transformations/dynamic_to_static_shape_reduce.cpp#L61)에서처럼 std::find와 결합하여 아래와 같이 많이들 활용한다:

```cpp
std::vector<int64_t> range(data_rank_value);
std::iota(range.begin(), range.end(), 0);
std::vector<int64_t> indices;
std::copy_if(range.cbegin(), range.cend(), std::back_inserter(indices),
        [&axes](int64_t i) { return std::find(axes.cbegin(), axes.cend(), i) == axes.cend(); });

```

즉, 0번 째부터 N번쨰의 `range`의 요소인 index `i`에 대하여, `i`값이 `axes`에 존재하지 않으면 indices에 요소를 복사한다.


---

Robotics 연구자/개발자를 위한 Modern C++ 시리즈입니다.
사용된 코드들은 [여기](https://github.com/LimHyungTae/moderncpp_study)에서 확인할 수 있습니다.

{% include post_links_modern_cpp.html %}
