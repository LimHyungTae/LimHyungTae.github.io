---
layout: post
title: Modern C++ for Robotics 4. std::transform() 쉬운 설명 & 예제
tags: [C++, Eigen, Robotics]
comments: true
---

## std::accumulate

std::accumulate() 함수는 범위 내의 모든 요소를 처음부터 끝까지 누적한 결과를 반환한다.
이 함수는  1) 시작 반복자, 2) 종료 반복자, 3) 초기 값, 4) (optional) 그리고 선택적으로 lambda expression을 인자로 받는다. 
기본적으로는 아래와 같이 사용할 수 있다:

```cpp
#include <numeric>
#include <vector>
#include <iostream>

int main() {
    std::vector<int> v = {1, 2, 3, 4, 5};
    int sum = std::accumulate(v.begin(), v.end(), 0);
    
    // 출력: Sum: 15
    std::cout << "Sum: " << sum << std::endl; 
}

```

여기서 중요한 것은 초기값과 return을 받는 변수의 타입이 같아야 한다는 것이다.
std::accumulate를 의도대로(?) 사용하면 평균을 구하는 함수도 한줄로 작성이 가능하다:

```cpp
// 거리 값들을 담고 있는 std::vector<double> dist가 있다고 가정
mean = std::accumulate(dist.begin(), dist.end(), 0.0) / dist.size();

```


## std::accumulate의 활용 

그런데 주목할 점은 std::accumulate을 **무궁무진하게 사용할 수 있다는 것**이다. std::accumulate을 아래와 같이 std::vector 내에 탐색을 할 때도 사용할 수 있다:

```cpp
#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>  // std::max를 위해 필요

int main() {
    std::vector<int> nums = {3, 1, 4, 1, 5, 9, 2, 6, 5, 3, 5};

    // std::accumulate를 사용하여 최대값 찾기
    int max_value = std::accumulate(nums.begin(), nums.end(), nums[0],
                                    [](int a, int b) { return std::max(a, b); });

    std::cout << "The maximum value is: " << max_value << std::endl;

    return 0;
}
```

즉,

```cpp
int max_value = 0;
for (const auto& elem: nums) {
    max_value = std::max(max_value, elem);
}
```
을 std::accumulate()를 사용하여 한 줄로 표현할 수 있다. 

이를 더 심화시키면 아래와 내가 만든 struct나 class가 지니는 멤버 변수의 값이 최대인 객체를 찾는 식으로도 활용이 가능하다:

```cpp
#include <numeric>
#include <vector>
#include <iostream>

struct MyStruct {
    int x = 0;
};
int main() {
    MyStruct s1, s2, s3, s4, s5;
    s1.x = 19;
    s2.x = 21;
    s3.x = 30;
    s4.x = 28;
    s5.x = 3;
    std::vector<MyStruct> v = {s1, s2, s3, s4, s5};
    const MyStruct& max_struct = std::accumulate(v.begin(), v.end(), v[0], [](MyStruct& a, MyStruct& b) {
        return a.x > b.x ? a : b;
    });

    // 출력: Sum: 30
    std::cout << "Max value of the max struct: " << max_struct.x << std::endl;
}
```

이러한 이유로 std::accumulate는 조건을 탐색할 때에도 많이 사용된다.

## 로보틱스에서 활용 

사실 합이나 평균을 구하는 과정에서는 모두 다 자명하게 사용 가능 하다.

```cpp
// From https://github.com/ethz-asl/maplab
double distance_sum = std::accumulate(minimum_distances.begin(), minimum_distances.end(), 0.0);
// From https://github.com/ethz-asl/panoptic_mapping
const float mean = std::accumulate(times.begin(), times.end(), 0.f) / times.size();
```

좀 더 색다르게 사용하면 [Open3D 내에 구현되어 있는](https://github.com/isl-org/Open3D/blob/5c982c7b5edc76f899860e2594a950c5c23ec88f/cpp/open3d/io/file_format/FileOBJ.cpp#L124) 아래 코드처럼 모든 법선이 설정되어 있는지 확인하는 코드도 가능하다: 

```cpp
// if not all normals have been set, then remove the vertex normals
    bool all_normals_set =
            std::accumulate(normals_indicator.begin(), normals_indicator.end(),
                            true, [](bool a, bool b) { return a && b; });
```

하지만 위의 코드는 파일을 불러오는 코드여서 상관없긴 하지만, `normals_indicator`의 모든 요소가 조건을 충족하는 지 확인해야 한다는 단점이 있다.
따라서 무진장 큰 vector라면, 모든 요소의 boolean 값을 계속해서 확인하는 것이 비효율 적일 수 있다.

이런 경우에는 [std::all_of](https://en.cppreference.com/w/cpp/algorithm/all_of)를 사용하는 것이 더 효율적이다.

```cpp
// 모든 element가 조건을 만족하는 지 확인
bool all_normals_set = std::all_of(normals_indicator.begin(), normals_indicator.end(), [](bool val) {
        return val; // 여기서 val이 false라면 바로 false를 반환하고 반복을 중단
    });
```

std::all_of가 std::accumulate보다 더 효율적인 이유는, 조건을 만족하지 않는 요소를 만나면 바로 false를 반환하고 반복을 중단하기 때문이다.

## 결론

Weighted sum, 평균, 최대값, 최소값, 조건을 만족하는지 확인하는 등 다양한 활용이 가능한 std::accumulate() 함수에 대해 알아보았다.
하지만 특정 조건을 만족하는지 확인하는 경우에는, 더 효율적인 함수형 프로그래밍을 지원할 수도 있으니 주의해서 사용해야 한다(키워드로는 std::all_of, std::any_of, std::none_of 등이 있다). 

---

Robotics 연구자/개발자를 위한 Modern C++ 시리즈입니다.
사용된 코드들은 [여기](https://github.com/LimHyungTae/moderncpp_study)에서 확인할 수 있습니다.

{% include post_links_modern_cpp.html %}
