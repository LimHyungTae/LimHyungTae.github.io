---
layout: post
title: Modern C++ for Robotics 12. std::remove_if() 쉬운 설명 & 예제
subtitle: Easy Explanation & Examples of Conditional Algorithms
tags: [C++, Eigen, Robotics]
comments: true
---


## std::remove_if()

```cpp
#include <algorithm>
#include <vector>
#include <iostream>

int main() {
    std::vector<int> v = {1, 2, 3, 4, 5, 6};
    auto new_end = std::remove_if(v.begin(), v.end(), [](int x) { return x % 2 == 0; });
    v.erase(new_end, v.end());

    // Output: 1 3 5
    for (auto i : v) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
}
```

std::remove_if의 결과로 `v` 상에 남아있는 값들을 위한 새로운 end iterator `new_end`가 출력되는 것을 볼 수있다.
따라서 std::remove_if는 주로 erase 계열 함수와 단짝을 이뤄 사용된다 
즉, 지우고자 하는 요소들을 `new_end` 뒤로 보내고, `new_end`부터 원래 `v`의 `v.end()`까지를 지우면 우리가 원하고자 하는 최종 값들만 컨테이너에 남게 된다.

또 알아두면 좋은 것은, std::remove_if를 통해 남게되는 요소들의 순서가 보장된다는 것이다 ({1, 3, 5}가 원래 vector 내에 존재하던 순서대로 남아있음). 

## 로보틱스에서 활용 사례

Open3D에서 point cloud 내에 [plane을 detection하는 코드](https://github.com/isl-org/Open3D/blob/bf48b4540583805b56a6286db13ceb30f2b0635b/cpp/open3d/geometry/PointCloudPlanarPatchDetection.cpp#L624)에서 두 가지 사례로 사용되는 걸 볼 수 있다:

```cpp
if (num_outliers > 0) {
    indices_.erase(std::remove_if(indices_.begin(), indices_.end(),
                                  [&outliers](const size_t& idx) {
                                      return outliers[idx];
                                  }),
                   indices_.end());
}
```

위에서 언급했듯이, std::remove_if는 제거하고자 하는 요소들을 컨테이너 뒤로 보내고 남기고자 하는 요소들을 앞쪽으로 이동시킨 후,
컨테이너의 새로운 end iterator부터 원래 container의 end iterator까지 제거한다.

만약, std::remove_if 없이 코드를 작성하면 아래와 같이 작성해야 할것이다:

```cpp
std::vector<size_t> new_indices;
for (size_t idx : indices_) {
    if (!outliers[idx]) {
        new_indices.push_back(idx); 
    }
}

// Replace indices_ with new_indices
indices_ = new_indices;
```

~~lambda expression을 쓰기 전의 나의 코드일지도...?~~ 
이를 통해 lambda expression의 간결함을 다시금 확인할 수 있다.
또한, 위의 코드의 경우 1) 불필요하게 `new_indices`라는 새로운 컨테이너를 선언, 2) `new_indices`에 값 자체를 복사해야 해서 비효율적임을 알 수 있다.

그에 비해 이러한 함수형 알고리즘들의 내부가 우리가 생각하는 것 이상으로 메모리와 성능 효율적으로 작성되어 있기 때문에, 되도록이면 STL에서 제공하는 알고리즘들을 최대한 활용하면 좋을 거 같다 
(그도 그럴 것이, 우리에 비해 C++ contributor들은 훨씬 더 코딩 경험이 많고 고수들일 것이기 때문에...).

두 번째로, 아래에서도 invalid한 plane을 컨테이너 내에서 제거할 때 std::remove_if가 사용된다.

```cpp
planes.erase(std::remove_if(planes.begin(), planes.end(),
                            [](const PlaneDetectorPtr& plane) {
                                return plane == nullptr;
                            }),
             planes.end());
```

---

Robotics 연구자/개발자를 위한 Modern C++ 시리즈입니다.
사용된 코드들은 [여기](https://github.com/LimHyungTae/moderncpp_study)에서 확인할 수 있습니다.

{% include post_links_modern_cpp.html %}
