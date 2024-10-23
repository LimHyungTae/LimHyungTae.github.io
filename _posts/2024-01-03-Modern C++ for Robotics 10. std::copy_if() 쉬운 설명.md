---
layout: post
title: Modern C++ for Robotics 10. std::copy_if() 쉬운 설명
subtitle: Easy Explanation & Examples of Conditional Algorithms
tags: [C++, Eigen, Robotics]
comments: true
---

이번 글부터 [여기](https://limhyungtae.github.io/2024-01-03-Modern-C++-for-Robotics-13.-std-replace_if()-%EC%89%AC%EC%9A%B4-%EC%84%A4%EB%AA%85-&-%EC%98%88%EC%A0%9C/)까지는 함수 이름 끝에 `_if`가 달려 있는 STL의 조건부 알고리즘(conditional algorithms)에 대해 알아본다.
함수 이름들이 굉장히 직관적으로 잘 지어져있듯이, 이 조건부 알고리즘들은 컨테이너의 요소들을 lambda function의 조건이 참인지 거짓인지 판별한 후, 
그 결과에 따라 복사, [탐색](https://limhyungtae.github.io/2024-01-03-Modern-C++-for-Robotics-11.-std-find_if()-%EC%89%AC%EC%9A%B4-%EC%84%A4%EB%AA%85-&-%EC%98%88%EC%A0%9C/), [삭제](https://limhyungtae.github.io/2024-01-03-Modern-C++-for-Robotics-12.-std-remove_if()-%EC%89%AC%EC%9A%B4-%EC%84%A4%EB%AA%85-&-%EC%98%88%EC%A0%9C/), [교체](https://limhyungtae.github.io/2024-01-03-Modern-C++-for-Robotics-13.-std-replace_if()-%EC%89%AC%EC%9A%B4-%EC%84%A4%EB%AA%85-&-%EC%98%88%EC%A0%9C/)하는 역할을 한다.

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

    // Output: 2 4
    for (auto i : dst) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
}
```

`src` 내에서 조건을 만족하는 값을 `dst`의 뒷 쪽에 `std::back_inserter`를 통해 삽입하고 있는 것을 볼 수 있다.  

## 로보틱스에서 활용 사례

두 가지 활용 사례에 대해 살펴보자.
먼저, Open3D의 코드 내 [메모리를 release하는 부분](https://github.com/isl-org/Open3D/blob/008cfb7fb9831f84b6026617e9aa60ab814c701a/cpp/open3d/core/MemoryManagerCached.cpp#L236)에서 다음과 같이 사용되고 있다:

```cpp
std::set<std::shared_ptr<RealBlock>, SizeOrder<RealBlock>>
                releasable_real_blocks;
                
std::copy_if(real_blocks_.begin(), real_blocks_.end(),
    std::inserter(releasable_real_blocks,
                  releasable_real_blocks.begin()),
    [this](const auto& r_block) { return IsReleasable(r_block); });
```
앞서 살펴본 예제에서는 `std::back_inserter`를 통해 vector의 뒷부분에 요소를 삽입했다면, 
이 코드에서는 `std::inserter`를 통해 set에 삽입하는 방식으로 작성되어 있다. 
set의 경우에는 내부적으로 정렬 순서에 따라서 요소를 배치하기 때문에, `std::back_inserter `는 지원되지 않는다.
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

    // Output: 1.5 2 3.2 4
    for (auto i : dst) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
}
```

추가적으로, 다른 컨테이너(i.e., `src`는 `std::vector`이고 `dst`는 `std::set`임)간의 데이터 교환이라도 함수형 프로그래밍은 컨테이너의 iterator만 잘 주어진다면 유연하게 사용이 가능하다는 것을 확인할 수 있다.

또한 아래 [OpenVINO 코드](https://github.com/ethz-asl/openvino/blob/b4ad7a1755b4799f92ef042bacc719ec3c0c1cbd/inference-engine/src/vpu/common/src/ngraph/transformations/dynamic_to_static_shape_reduce.cpp#L61)에서처럼 std::find와 결합하여 아래와 같이 많이들 활용한다:

```cpp
std::vector<int64_t> range(data_rank_value);
std::iota(range.begin(), range.end(), 0);
std::vector<int64_t> indices;
std::copy_if(range.cbegin(), range.cend(), std::back_inserter(indices),
        [&axes](int64_t i) { return std::find(axes.cbegin(), axes.cend(), i) == axes.cend(); });

```

즉, 0번 째부터 N번쨰의 `range`의 요소인 index `i`에 대하여, `i`값이 `axes`에 존재하지 않으면 indices에 요소를 복사한다.

여기서 `cbegin()`과 `cend()`는 const iterator를 리턴하는 함수로, `begin()`과 `end()`와 유사하지만, iterator를 통해 컨테이너의 요소를 수정할 수 없게 한다.
따라서 요소들이 실수로 변경되는 것을 방지할 수 있다.

---

Robotics 연구자/개발자를 위한 Modern C++ 시리즈입니다.
사용된 코드들은 [여기](https://github.com/LimHyungTae/moderncpp_study)에서 확인할 수 있습니다.

{% include post_links_modern_cpp.html %}
