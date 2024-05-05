---
layout: post
title: Modern C++ for Robotics 5. std::move() 쉬운 설명 & 예제
tags: [C++, Eigen, Robotics]
comments: true
---

## std::move

std::insert는 C++ 표준 라이브러리의 컨테이너 클래스에서 사용할 수 있는 멤버 함수로, 특정 위치에 하나 이상의 요소를 삽입하는 데 사용된다. 
std::insert의 장점은, 다양한 컨테이너 타입(e.g., std::vector, std::unordered_set 등등)에 쉽게 사용가능하다는 것이다. 물론 각각의 컨테이너 타입에 따라 약간씩 다를 수 있다. 


### std::vector에서의 std::insert 사용법

std::vector에서 std::insert는 주어진 위치에 요소를 삽입한다. 삽입 위치는 반복자로 지정하고, 삽입할 값도 함께 아래와 같이 제공해야 한다. 또한 횟수를 지정해서 여러번 값을 insert를 할 수도 있다:

```cpp
#include <iostream>
#include <vector>

int main() {
    std::vector<int> vec = {1, 2, 4, 5};

    // 3을 vec[2] 위치에 삽입
    vec.insert(vec.begin() + 2, 3);

    // 출력: 1 2 3 4 5
    std::cout << "Vector after insertion: ";
    for (int elem : vec) {
        std::cout << elem << " ";
    }
    std::cout << "\n";

    // 100을 vec[0] 위치에 3번 삽입 
    vec.insert(vec.begin(), 3, 100);

    // 출력: 100 100 100 1 2 3 4 5
    std::cout << "Vector after multiple insertions: ";
    for (int elem : vec) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;

    return 0;
}
```

하지만 std::vector의 경우, 요소를 삽입할 때마다 컨테이너의 기존 요소들을 뒤로 밀어내야 하기 때문에 큰 벡터에서는 성능상 비용이 클 수 있습니다.
주로 std::vector에서는 여러 std::vector를 합칠 때 std::insert를 활용하면 편리하다. 아래 예제는 네 개의 std::vector를 합치는 예제이다:

```cpp
#include <iostream>
#include <vector>

int main() {
    std::vector<int> vec1 = {1, 2, 3};
    std::vector<int> vec2 = {4, 5, 6};
    std::vector<int> vec3 = {7, 8, 9};
    std::vector<int> vec4 = {10, 11, 12};

    // vec1의 끝에 vec2의 요소들을 삽입
    vec1.insert(vec1.end(), vec2.begin(), vec2.end());

    // vec1의 끝에 vec3의 요소들을 삽입
    vec1.insert(vec1.end(), vec3.begin(), vec3.end());

    // 출력: Combined vector: 1 2 3 4 5 6 7 8 9
    std::cout << "Combined vector: ";
    for (int elem : vec1) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;

    // std::make_move_iterator와 insert 함수의 콤-비네이숀으로 vec4의 요소들을 vec1의 끝에 '이동'시킬 수도 있음!
    vec1.insert(vec1.end(), std::make_move_iterator(vec4.begin()), std::make_move_iterator(vec4.end()));
    vec4.clear();

    // 출력: Combined vector: 1 2 3 4 5 6 7 8 9 10 11 12
    std::cout << "After move iterator: ";
    for (int elem : vec1) {
        std::cout << elem << " ";
    }
    std::cout << std::endl;

    return 0;
}
```

이를 통해 두 가지를 알 수 있는데, 첫 번째는 아래와 같이 for 문을 돌며 요소를 넣어주는 행위를 

```cpp
for (const auto& elem : vec4) {
    vec1.emplace_back(elem);
}
```

코드 한 줄로 간결히 표현할 수 있다는 것이다(코드의 간지 또한 덤으로 얻을 수 있다(?)).
그리고 두 번째로는 std::make_move_iterator를 사용하여 벡터의 요소를 복사 없이 '이동'시킬 수 있다는 것이다 (이동시킨다는 것이 중요하다!).
아무리 emplace_back이 push_back보다 효율적이라고 해도, std::make_move_iterator를 통해 요소를 이동시키는 것이 퍼포먼스 측면에서 보다 효율적임을 잊지 말자.
이는 다음 글 std::move에서 좀 더 자세히 다룰 예정이다.


### std::unordered_set에서의 std::insert 사용법

마찬가지로 std::insert를 set이나 map container에서도 사용이 가능하다.

```
#include <iostream>
#include <unordered_set>
#include <string>

int main() {
    // 두 개의 unordered_set 초기화
    std::unordered_set<std::string> set1 = {"apple", "banana", "mango"};
    std::unordered_set<std::string> set2 = {"banana", "orange", "grape"};

    // set2의 모든 요소를 set1에 병합
    set1.insert(set2.begin(), set2.end());

    // 병합된 unordered_set의 내용을 출력
    // 출력: Combined set contents: grape apple orange banana mango
    std::cout << "Combined set contents: ";
    for (const std::string& fruit : set1) {
        std::cout << fruit << " ";
    }
    std::cout << std::endl;

    return 0;
}
```

주목할 점은, set이나 map container에서는 중복된 요소를 허용하지 않기 때문에, insert를 통해 요소를 병합할 때 중복된 요소는 무시된다는 것이다.
그래서 원래 같으면 아래와 같이 요소가 있는지 확인 후 삽입해야 하는데, insert를 사용하면 이 과정을 생략할 수 있다:

```cpp
for (const auto& elem : set2) {
    if (set1.find(elem) == set1.end()) {
        set1.insert(elem);
    }
}
```


로보틱스의 경우에는 이를 통해 keyframes의 id를 다루거나, 혹은 multi-robot 상황에서 landmark들의 id를 unique하게 관리해야하는 상황에서 활용할 수 있다.


## 로보틱스에서 예제



이전에도 말했듯이, 이런 함수형 프로그래밍은 병렬 처리를 할 때 꽃으로 사용된다. 아래는 [kiss-icp](https://github.com/PRBonn/kiss-icp/blob/1129b6e451222a891a26ddfdb77d719ce481534b/cpp/kiss_icp/core/Registration.cpp#L91)에서 TBB를 활용할 때 insert를 통해 효율적으로 여러 correspondences를 하나로 취합하는 예제이다:


tbb::parallel_reduce가 무엇인지 자세히 알 필요는 없더라도 찬찬히 코드를 보면 이해할 수 있다.
1) 각 thread가 '1st lambda' function을 통해 각 thread들이 첫 번째로 `Correspondence` type의 `res`를 리턴하고, 
2) 이렇게 각 thread에서 리턴한 `res1`, `res2`, `res3`, ...들을 (`tbb::block_range`를 통해 각각의 thread가 독립적으로 1st lambda function을 수행해서 임의의 값을 리턴함) 
3) '2nd lambda' function을 통해 임이의 두 `res_a`와 `res_b`를 효율적으로 병합하는 예제이다.


## 결론 

Lambda expression과 STL에서 제공하는 algorithm을 사용하면 코드를 간결하게 작성할 수 있고, 병렬 처리를 할 때도 효율적으로 코드를 작성할 수 있다는 것을 확인할 수 있다.

---

Robotics 연구자/개발자를 위한 Modern C++ 시리즈입니다.
사용된 코드들은 [여기](https://github.com/LimHyungTae/moderncpp_study)에서 확인할 수 있습니다.

{% include post_links_modern_cpp.html %}
