---
layout: post
title: Modern C++ for Robotics 10. std::copy_if(), std::remove_if(), std::find_if(), std::replace_if() 쉬운 설명 & 예제
subtitle: Easy Explanation & Examples of Conditional Algorithms
tags: [C++, Eigen, Robotics]
comments: true
---


## std::replace_if()


```cpp
#include <algorithm>
#include <vector>
#include <iostream>

int main() {
    std::vector<int> v = {1, 2, 3, 4, 5};
    std::replace_if(v.begin(), v.end(), [](int x) { return x % 2 == 0; }, 0);

    // 출력: 1 0 3 0 5
    for (auto i : v) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
}

```

std::remove_if와 마찬가지로, std::replace_if도 뭔가 이상치/유효하지 않은 요소를 다룰 때 많이 사용된다.
하지만 std::remove_if는 유효하지 않은 값을 컨테이너에서 제거하는 게 목적이기 때문에 요소의 수가 유동적이어도 될 때 사용을 하는 반면,
std::replace_if는 컨테이너의 크기 자체는 건드리지 않는 채로 유효하지 않은 값들을 처리할 때 많이들 사용된다.


## 로보틱스에서 활용 사례

만약 LiDAR sensor에서 datasheet 상 매 0.1초마다 1,000개의 point가 들어온다고 되어있는데, 
내 멋대로 유효하지 않은 점들을 std::remove_if로 제거한 채로 950개만 제공해도 괜찮을까?
당연히 그렇지 않다. 

그렇기 때문에, 아래 코드에서 볼 수 있듯이 [gazebo ros pkgs](https://github.com/Robobench/gazebo_ros_pkgs/blob/1c260498be2861489a0f9df58fdd72821383501e/gazebo_plugins/src/gazebo_ros_laser.cpp#L2) 상에서 laser scan의 값을 
range에 따라 invalid한 값들에 각각 -INF와 INF를 대입하는 것을 볼 수 있다:

```cpp
std::replace_if(laser_msg.ranges.begin(), laser_msg.ranges.end(),
  std::bind2nd(std::less_equal<float>(), laser_msg.range_min),
  -std::numeric_limits<float>::infinity());
std::replace_if(laser_msg.ranges.begin(), laser_msg.ranges.end(),
  std::bind2nd(std::greater_equal<float>(), laser_msg.range_max),
  std::numeric_limits<float>::infinity());
```

이렇게 함으로써 `laser_msg`의 요소 전체의 크기는 건드리지 않는 채로 유효하지 않은 값들을 처리할 수 있다.
위에서 `std::bind2nd`는 두 개의 인자를 받는 함수 객체를 하나의 인자를 받는 함수 객체로 변환하는 어댑터로서,
쉽게 풀어써보자면 다음과 같은 역할을 하고 있다:

```cpp
// std::bind2nd(std::less_equal<float>(), laser_msg.range_min)
// second <- laser_msg.range_min
bool less_equal(float first, float second) {
    return first < second;
}

// std::bind2nd(std::greater_equal<float>(), laser_msg.range_max)
// second <- laser_msg.range_max
bool greater_equal(float first, float second) {
    return first > second;
}
```

--- 

P.S. 찾아보니 std::bind2nd는 C++11 이후로 더 이상 사용되지 않으며, C++17에서는 공식적으로 제거되었다고 한다. 
그래서 향후에는 std::bind를 써야된다고 한다. std::bind를 사용하면 아래와 같이 표현할 수 있다:

```cpp
std::replace_if(laser_msg.ranges.begin(), laser_msg.ranges.end(),
    std::bind(std::less_equal<float>(), std::placeholders::_1, laser_msg.range_min),
    -std::numeric_limits<float>::infinity());
std::replace_if(laser_msg.ranges.begin(), laser_msg.ranges.end(),
    std::bind(std::greater_equal<float>(), std::placeholders::_1, laser_msg.range_max),
    std::numeric_limits<float>::infinity());
```

즉, 위에서 first 인자가 함수 입력으로 들어가게 되는게 비직관적이다보니 `std::placeholders::_1`를 명시적으로 둬서 보여주게 만든 거 같다.

---

Robotics 연구자/개발자를 위한 Modern C++ 시리즈입니다.
사용된 코드들은 [여기](https://github.com/LimHyungTae/moderncpp_study)에서 확인할 수 있습니다.

{% include post_links_modern_cpp.html %}
