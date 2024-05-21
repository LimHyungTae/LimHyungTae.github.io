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

    // Output: First element greater than 3: 4
    if (it != v.end()) {
        std::cout << "First element greater than 3: " << *it << std::endl;
    }
    std::cout << std::endl;
}
```

[std::any_of](https://limhyungtae.github.io/2024-01-01-Modern-C++-for-Robotics-9.-std-all_of(),-std-any_of(),-std-none_of()-%EC%89%AC%EC%9A%B4-%EC%84%A4%EB%AA%85-&-%EC%98%88%EC%A0%9C/)를 설명할 때 이미 살펴보았는데, 컨테이너 내에서 순서대로 탐색하여 가장 먼저 발견되는 요소의 iterator를 반환하는 함수이다. 

## stt::find vs std::find_if

만약 C++ 코딩을 조금 경험해 본 이라면 'std::find로도 요소를 찾을 수 있는데, 왜 굳이 std::find_if를 써야하지?'라는 의문이 생길 것이다.
그 이유는 함수형 프로그래밍의 확장성에 있다.
std::find 함수는 컨테이너 내의 특정 값을 직접 비교하여 찾기 때문에, 이 함수는 단순한 원시 데이터 타입(int, char 등)이나 동등성 비교가 가능한 객체에 대해서만 사용할 수 있다. 
반면, std::find_if는 객체의 특정 멤버 변수를 기준으로 검색하고자 할 때에도 활용이 가능하다.
즉, std::find_if는 `=` operator가 정의되어 있지 않은 객체에 대해서도 탐색이 가능하게끔 해주기 때문에, 결론적으로 이 검색 기능을 훨씬 더 유연하게 확장할 수 있게 해준다.

아래는 `Person`이라는 struct가 주어졌을 때 간편히 서치를 하는 방법이다.

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
 
    // Output: Found: Bob is 25 years old.
    if (it != people.end()) {
        std::cout << "Found: " << it->name << " is " << it->age << " years old." << std::endl;
    } else {
        std::cout << "No person found with age " << age_to_find << std::endl;
    }

    return 0;
}
```

`[]` 내에 `age_to_find`가 입력으로 들어가는데, 이는 
lambda expression 상의 캡쳐 목록(capture clause)으로, 람다 표현식 내에서 사용할 외부 변수를 명시적으로 지정하는 방법이다
캡쳐 목록에는 값 복사 또는 참조 복사를 사용하여 외부 변수를 지정할 수 있다.
Modern C++에 친숙하지 않은 이라면 이 꼴이 어딘가 허전해보일 수가 있는데,
이는 C++ 17 이상에서는 `auto ${변수명}`이라고 원래 써야하는 것에서 `auto`를 생략할 수 있게 되었기 때문이다.
그래서 좀 허전해 보이지만, 코드가 동작하는 데에는 문제가 없다.

아래 예시를 통해 std::find_if와 캡쳐 목록의 예제를 한번 더 살펴보자.

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

즉, `imus` vector 내에  `start`와 동일한 timestamp를 지니고 있는 IMU값의 위치로 `prev_imu`로 받아오는 것을 확인할 수 있다.
여기서 lambda expression 내의 `&start`도 위에서 설명한 캡쳐 목록으로, 람다 표현식 내에서 사용할 timestamp를 입력으로 받아오는 것을 볼 수 있다.
즉, Lambda function 내에서 사용하고픈 변수가 있으면 &{변수명}으로 붙이면 된다. 
만약 외부의 모든 변수를 참조하여 사용하고 싶다면 `[&]`라고 쓰면 된다.

---

Robotics 연구자/개발자를 위한 Modern C++ 시리즈입니다.
사용된 코드들은 [여기](https://github.com/LimHyungTae/moderncpp_study)에서 확인할 수 있습니다.

{% include post_links_modern_cpp.html %}
