---
layout: post
title: Modern C++ for Robotics 9. std::all_of(), std::any_of(), std::none_of() 쉬운 설명 & 예제
subtitle: Easy Explanation & Examples of std::all_of(), std::any_of(), and std::none_of()
tags: [C++, Eigen, Robotics]
comments: true
---

## std::all_of(), std::any_of(), std::none_of()

해당 세 힘수는 이름에서도 알 수 있듯이, 컨테이너 내의 조건문을 탐색할 때 사용하는 함수이다.

* std::all_of: 벡터의 모든 요소가 조건을 만족하는지 확인
* std::any_of: 벡터의 어떤 요소라도 조건을 만족하는지 확인
* std::none_of: 벡터의 모든 요소가 조건을 만족하지 않는지 확인

이 함수들은 아래와 같이 사용할 수 있다. 이 세 함수는 사용법이 자명하니 사용법 자체에 대한 설명은 생략한다:

```cpp
#include <algorithm>
#include <vector>
#include <iostream>

int main() {
    std::vector<int> data = {2, 4, 6, 8, 10, 12};

    // all_of: 모든 요소가 2의 배수인지 확인
    bool allEven = std::all_of(data.begin(), data.end(), [](int x) {
        return x % 2 == 0;
    });

    // any_of: 하나라도 3의 배수가 있는지 확인
    bool anyMultipleOfThree = std::any_of(data.begin(), data.end(), [](int x) {
        return x % 3 == 0;
    });

    // none_of: 모든 요소가 음수가 아닌지 확인
    bool noneNegative = std::none_of(data.begin(), data.end(), [](int x) {
        return x < 0;
    });

    // 결과 출력
    // All elements are even: Yes
    // Any element is a multiple of three: Yes
    // No elements are negative: Yes
    std::cout << "All elements are even: " << (allEven ? "Yes" : "No") << std::endl;
    std::cout << "Any element is a multiple of three: " << (anyMultipleOfThree ? "Yes" : "No") << std::endl;
    std::cout << "No elements are negative: " << (noneNegative ? "Yes" : "No") << std::endl;

    return 0;
}
```

하지만 강조하고 싶은 것은, 이 세 함수는 **빠른 확인을 위해 조건을 만족하거나/만족하지 않는 즉시 break되게끔 설계가 되었다는 점이다.** 
좀 더 풀어서 설명해보자. 만약에 100,000개의 data라는 vector<int>가 있다고 가정하자.
만약 우리가 어떤 조건을 만족하는 요소가 있는지 확인하고 싶다면, 아래와 같이 코드를 짰을 것이다.

```cpp
bool has_odd = false;
for (auto i : data) {
    if (i % 2 == 1) {
        has_odd = true;
        break;
    }
}
```
여기서 break가 있는지 없는지는 속도에 큰 영향을 미친다. 만약 break가 없다면, 모든 요소를 확인해야 하기 때문에 속도가 느려질 것이다.

일반적으로는 모든 요소를 확인해야 한다. 그러나 std::any_of()의 경우 하나라도 있는지 확인하기 위해서 모든 요소를 확인하지 않고, 하나라도 만족하는 요소가 있으면 바로 true를 반환한다. 따라서 성능상 이점이 있다.


std::any_of()의 경우 하나라도 있는지 확인하기 위해서 모든 요소를 확인하지 않고, 하나라도 만족하는 요소가 있으면 바로 true를 반환한다. 따라서 성능상 이점이 있다.

---

Robotics 연구자/개발자를 위한 Modern C++ 시리즈입니다.
사용된 코드들은 [여기](https://github.com/LimHyungTae/moderncpp_study)에서 확인할 수 있습니다.

{% include post_links_modern_cpp.html %}
