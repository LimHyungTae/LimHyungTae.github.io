---
layout: post
title: Modern C++ for Robotics 11. std::find_if() 쉬운 설명 & 예제
subtitle: Easy Explanation & Examples of Conditional Algorithms
tags: [C++, Eigen, Robotics]
comments: true
---


## std::find_if()

```cpp
#include <algorithm>
#include <vector>
#include <iostream>

int main() {
    std::vector<int> v = {1, 2, 3, 4, 5};
    auto it = std::find_if(v.begin(), v.end(), [](int x) { return x > 3; });

    // 출력: First element greater than 3: 4
    if (it != v.end()) {
        std::cout << "First element greater than 3: " << *it << std::endl;
    }
    std::cout << std::endl;
}

```

## stt::find vs std::find_if

만약 C++ 코딩을 조금 경험해 본 이라면 'std::find로도 요소를 찾을 수 있는데, 왜 굳이 std::find_if를 써야하지?'라는 의문이 생길 것이다.
그 이유는, std::find 함수는 컨테이너 내의 특정 값을 직접 비교하여 찾기 때문에, 단순히 원시 데이터 타입(int, char 등)이나 동등성 비교가 가능한 객체에 대해서만 직접 사용할 수 있는 반면, 
std::find_if는 객체의 특정 멤버 변수를 기준으로 검색하고자 할 때에도 활용이 가능하기 때문이다.
즉, std::find_if는 STL에서 검색 기능을 훨씬 더 유연하게 확장할 수 있게 해준다.

```cpp
#include <algorithm>
#include <vector>
#include <iostream>

struct Person {
    std::string name;
    int age;
};

int main() {
    std::vector<Person> people = {
        {"Alice", 30},
        {"Bob", 25},
        {"Charlie", 35}
    };

    int age_to_find = 25;
    auto it = std::find_if(people.begin(), people.end(), [age_to_find](const Person& p) {
        return p.age == age_to_find;
    });
 
    // 출력: Found: Bob is 25 years old.
    if (it != people.end()) {
        std::cout << "Found: " << it->name << " is " << it->age << " years old." << std::endl;
    } else {
        std::cout << "No person found with age " << age_to_find << std::endl;
    }

    return 0;
}
```


## 로보틱스에서 활용 사례

ETH ASL의 [GNSS 기반 pose estimator](https://github.com/ethz-asl/mav_gtsam_estimator/blob/b4bb6b042d6939c9be377e4dec3909d24b12e4c6/src/mav_state_estimator.cc#L870)에서 아래와 같이 원하는 IMU를 찾을 때 사용하는 것을 볼 수 있다:

```angular2html
auto start = idx_to_stamp->at(idx);
auto prev_imu = std::find_if(imus->begin(), imus->end(),
                             [&start](const sensor_msgs::Imu::ConstPtr& x) {
                               return x->header.stamp == start;
                             });
    if (prev_imu == imus->end()) {
      ROS_ERROR("Cannot not find start IMU message.");
    } else {
       ...
```

즉 `imus` vector 내에  `start`와 동일한 timestamp를 지니고 있는 IMU값의 위치로 `prev_imu`로 받아오는 것을 확인할 수 있다.
여기서 Lambda expression 상의 `&start`는 캡쳐 목록(capture clause)으로, 람다 표현식 내에서 사용할 외부 변수를 명시적으로 지정하는 방법이다.
캡처 목록에는 값 복사 또는 참조 복사를 사용하여 외부 변수를 지정할 수 있다. 뭔가 허전해 보이는 이유는, C++ 17 이상에서는 `auto &start`라고 원래 써야하는 것에서 `auto` 또한 생략한 채로 작성할 수 있게 되었기 때문이다.
즉, Lambda function 내에서 사용하고픈 변수가 있으면 &{변수명}으로 붙이면 된다. 
만약 외부의 모든 변수를 참조하여 사용하고 싶다면 `[&]`라고 쓰면 된다.
---

Robotics 연구자/개발자를 위한 Modern C++ 시리즈입니다.
사용된 코드들은 [여기](https://github.com/LimHyungTae/moderncpp_study)에서 확인할 수 있습니다.

{% include post_links_modern_cpp.html %}
