---
layout: post
title: Modern C++ for Robotics 9. std::all_of(), std::any_of(), std::none_of() 쉬운 설명
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

하지만 강조하고 싶은 것은, 이 세 함수는 **빠른 확인을 위해 조건을 만족하거나/만족하지 않는 즉시 break되게끔 설계가 되었다는 점**이다. 

좀 더 풀어서 설명해보자. [이전 글에서 예시](https://limhyungtae.github.io/2024-01-01-Modern-C++-for-Robotics-8.-std-accumulate()-%EC%89%AC%EC%9A%B4-%EC%84%A4%EB%AA%85-&-%EC%98%88%EC%A0%9C/)처럼 만약에 100,000개의 `vector<int> data`가 있다고 가정하자.
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

위의 코드를 std::any_of를 사용하면 아래와 같이 작성이 가능하다:

```cpp
bool has_odd = std::any_of(data.begin(), data.end(), [](int i) {
    return i % 2 == 1;
});
```

std::any_of의 경우 하나라도 있는지 확인하기 위해서 모든 요소를 확인하지 않고, 하나라도 만족하는 요소가 있으면 바로 true를 반환해야 연산 효율적인데, 깊게 들어가보면 그렇게 구현되어 있는 것을 볼 수 있다:

```cpp
template<class InputIt, class UnaryPred>
constexpr bool any_of(InputIt first, InputIt last, UnaryPred p)
{
    return std::find_if(first, last, p) != last;
}
```

```cpp
template<class InputIt, class UnaryPred>
constexpr InputIt find_if(InputIt first, InputIt last, UnaryPred p)
{
    for (; first != last; ++first)
        if (p(*first))
            return first;
 
    return last;
}
```

뒤의 블로그 글에서 [std::find_if](https://limhyungtae.github.io/2024-01-03-Modern-C++-for-Robotics-11.-std-find_if()-%EC%89%AC%EC%9A%B4-%EC%84%A4%EB%AA%85-&-%EC%98%88%EC%A0%9C/)에 대해서 설명하겠지만,
대략적으로 살펴보면 컨테이너 내에서 어떤 조건을 만족하는 요소가 있으면(`if (p(*first))` 부분) for문을 그만 돌고 바로 리턴하는 것을 볼 수 있다(`return first` 부분).
즉, 함수형 프로그래밍의 코드를 짠 선조들이 우리보다 코드를 잘 짰을 확률이 높기 때문에 믿고 쓰면 된다...

## 로보틱스에서 활용 사례

사실 이 세 함수는 아주 직관적이어서 무궁무진하게 활용이 가능하다.
아래 코드는 ETH ASL에서 공개한 이동 가능 영역을 visualization을 많이 할 때 쓰이는 grid_map library인데, 그 내부에서도 이 조건 검색문들을 적극적으로 활용하는 것을 볼 수 있다

[사례 1](https://github.com/ethz-asl/grid_map/blob/43cb510d099daff8236a6bad8df4e7971f2df2bd/grid_map_core/src/GridMap.cpp#L77). `GridMap` 클래스인 `other`을 입력으로 받아 현재 `GridMap`과 동일한 `layers_`를 전부 가지고 있는지 확인하는 함수 

```cpp
bool GridMap::hasSameLayers(const GridMap& other) const {
  return std::all_of(layers_.begin(), layers_.end(),
                     [&](const std::string& layer){return other.exists(layer);});
```

[사례 2](https://github.com/ethz-asl/grid_map/blob/43cb510d099daff8236a6bad8df4e7971f2df2bd/grid_map_core/src/GridMap.cpp#L256). 어떤 `layer` 내의 `index` 번째 요소가 유효한지 판별하는 함수. 즉, 여러 layers들의 모든 `index` 번째 요소가 valid해야 true를 리턴함.

```cpp
bool GridMap::isValid(const Index& index, const std::vector<std::string>& layers) const {
  if (layers.empty()) {
    return false;
  }
  return std::all_of(layers.begin(), layers.end(),
              [&](const std::string& layer){return isValid(index, layer);});
}
```


---

Robotics 연구자/개발자를 위한 Modern C++ 시리즈입니다.
사용된 코드들은 [여기](https://github.com/LimHyungTae/moderncpp_study)에서 확인할 수 있습니다.

{% include post_links_modern_cpp.html %}
