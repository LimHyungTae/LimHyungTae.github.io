---
layout: post
title: Modern C++ for Robotics 7. std::transform() 쉬운 설명 & 예제
subtitle: Easy Explanation & Examples of std::transform()
tags: [C++, Eigen, Robotics]
comments: true
---

## std::transform

여기부터는 그리 어렵지 않은 내용이다. std::transform() 함수는 컨테이너의 원소에 함수를 적용하고 결과를 다른 컨테이너에 저장한다. 즉, 매핑 함수의 역할을 한다.
이 함수는 컨테이너의 각 요소에 함수를 적용하고 결과를 다른 컨테이너에 저장하는 방식으로 작동한다.
개인적으로 std::transform의 매력은, 값들을 처리하는 데 있어서 굳이 다른 변수를 만들지 않아도 동일 변수 내에서 처리가 가능하다는 점이다.

아래 예제를 보면 쉽게 이해할 수 있다.

```cpp
#include <algorithm>
#include <vector>
#include <iostream>

int main() {
    std::vector<int> src = {1, 2, 3, 4, 5};
    std::vector<int> dst;
    dst.resize(src.size() + 2, 0);
    std::transform(src.begin(), src.end(), dst.begin(), [](int x) { return x * x; });

    // Output: 1 4 9 16 25 0 0
    for (auto i : dst) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
}
```

여기서 lambda expression으로 적혀있는 anonymous function의 인자인 `x`는 `src.begin()`과 `src.end()` 사이의 각 요소를 의미하며, 이를 제곱하여 `dst` 벡터에 저장한다. 
따라서 위 코드는 `src` 벡터의 각 요소를 제곱하여 `dst` 벡터에 저장한다.

또한, 만약 입력 값과 출력 값이 같은 경우에도 std::transform()을 사용할 수 있다.
아래에서 `dst` 벡터의 각 요소를 제곱하여 다시 `dst` 벡터에 저장하는 예제를 보면 된다.

```cpp
std::transform(dst.begin(), dst.end(), dst.begin(), [](int x) { return x * x; });
// Output: 1 16 81 256 625 0 0 
for (auto i : dst) {
    std::cout << i << " ";
}
std::cout << std::endl;

```

아주 깔끔하지 않은가?! (지극히 개인적 의견) 원래대로라면

```cpp
for (int i = 0; i < dst.size(); i++) {
    int sqr_value_tmp = dst[i] * dst[i];
    dst[i] = sqr_value_tmp;
}
```
라고 작성해줘야 하는 부분을 std::transform()을 사용하면 이렇게 간단하게 쓸 수 있다.

하지만 목적지 벡터(`dst`)의 크기가 충분하지 않다면, 정의되지 않은 동작(undefined behavior)이 발생할 수 있음을 주의해야 한다. 
예를 들어, `dst`의 크기가 `src.size()`보다 작다면, std::transform이 dst의 범위를 넘어서 값을 쓰려고 할 것이고, 
이는 프로그램 오류로 이어질 수 있으니 **반드시** `dst`의 크기가 `src`의 크기와 같거나 더 크게 잡혀있는지 확인해야 한다.
`dst`의 크기가 `src`보다 크기만 하다면 위에서 0 값이 계속 0으로 유지되는 것처럼, 큰 문제가 일어나진 않는다. 

## 로보틱스에서 활용 사례

로보틱스에서 pose transformation은 숙명이라고 볼 수 있는데, 만약 `Eigen::Vector4f`의 std::vector인 points가 있고,
`Eigen::Matrix4f`인 transformation `T`가 있으면, 아래와 같이 한 줄로 transformation을 작성할 수 있다.

```cpp
std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) { return T * point; });
```

(원 코드는 [여기](https://github.com/PRBonn/kiss-icp/blob/1129b6e451222a891a26ddfdb77d719ce481534b/cpp/kiss_icp/core/Registration.cpp#L51).
원 코드에서는 Sophus를 활용해서 std::vector\<Eigen::Vector3d\>를 transformation하는 것을 볼 수 있다. Sophus를 잘 쓰는 것도 중요할 듯!)

---

Robotics 연구자/개발자를 위한 Modern C++ 시리즈입니다.
사용된 코드들은 [여기](https://github.com/LimHyungTae/moderncpp_study)에서 확인할 수 있습니다.

{% include post_links_modern_cpp.html %}
