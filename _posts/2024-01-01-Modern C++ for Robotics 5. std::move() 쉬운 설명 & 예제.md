---
layout: post
title: Modern C++ for Robotics 4. std::move() 쉬운 설명 & 예제
tags: [C++, Eigen, Robotics]
comments: true
---

## std::move

std::move는 C++11에서 도입된 기능으로, 객체의 상태를 다른 객체로 '이동'시킬 수 있게 해주는 유틸리티 함수이다.
그런데 실제로 std::move는 객체를 이동시키지 않는다 ~~이건 대체 뭔소리여~~...
std::move를 정확히 잘 이해하려면 C++17에서 도입된 return value optimization (RVO)과 C++11 rvalue 참조(reference)에 대한 이해가 필요하다 (lvalue와 rvalue 자체가 생소한 독자라면, [이 블로그](https://nanze.tistory.com/entry/Cpp-Lvalue-%EC%A2%8C%EC%B8%A1%EA%B0%92-Rvalue-%EC%9A%B0%EC%B8%A1%EA%B0%92-Rvalue-reference-%EC%9A%B0%EC%B8%A1%EA%B0%92-%EC%B0%B8%EC%A1%B0%EC%9E%90)를 한 번 읽어보는 것을 추천한다).

너무 깊게 들어가면 이해하기 어려울 수 있으니, 우리는 예제를 통해 이해해보자.

```cpp
#include <iostream>
#include <vector>

std::vector<int> createVector() {
    return std::vector<int>{1, 2, 3, 4, 5};
}

int main() {
    std::vector<int> vec = createVector();  // createVector의 반환값은 rvalue
    std::vector<int> vec2 = std::move(createVector());
    
    // 둘의 주소는 당연히 다름! std::move는 주소와는 상관없다
    std::cout << "vec: " << &vec << std::endl;
    std::cout << "vec2: " << &vec2 << std::endl;
    return 0;
}
```

`createVector()` 함수는 {1, 2, 3, 4, 5}로 채워진 벡터를 만들고 이 "임시 객체"를 반환하는 함수이다. 이 임시 객체는 함수가 끝난 후 곧바로 사라지는 객체입니다.
그런데 이 임시 객체를 `vec`과 `vec2`에 할당하는 과정에서 차이가 발생한다:

* `vec`: rvalue(대충 = 기호의 오른쪽에 있는 값들이라고 생각하면 된다)인 {1, 2, 3, 4, 5}가 vec로 할당되었고,  `createVector()` 함수로부터 '직접' 반환된 임시 벡터를 받아 저장한다.
* `vec2`: std::move를 통해 '`vec2`야, 임시 객체 내에 있는 1, 2, 3, 4, 5 값들 너한테 이동 가능하다?'하고 알려준다. 이를 통해 임시 객체가 통째로 `vec2`로 복사되는 것이 아니라, 저 1, 2, 3, 4, 5 값이 '전달'된다 ('전달'(reference)에 강조된 것을 주목하자. 값 자체가 '이동'하는 것은 절대 아니다!!! 그래서 std::move의 이름을 지은 사람이 이름을 잘 못 지었다고 후회를 한다고 한다...).  

vec와 vec2의 주소를 출력해보면, vec2의 주소가 vec의 주소가 어떻게 출력될까? 당연히 둘은 다른 주소를 띈다. std::move는 주소와는 상관없다!

위의 상황을 좀 더 비유로 통해 이해를 해보자. 자취방을 이사해야하는 상황에서 늘 짐들을 박스에 담아서 날라야 하는데, 이 과정이 여간 귀찮은 게 아니다.
짐을 박스에 담아야 하고, 날라야 하고, 최종적으로는 짐을 풀어야 하기 때문이다 (`vec`의 상황).
하지만 22세기에 물체 단위로 텔레포트 시킬 수 있는 순간 전달 장치가 개발되어서(일부로 헷갈림을 방지하기 위해 '이동'이라는 말을 자제함...'전달'이 더 정확한 표현이다!), 
날라야 하는 짐들에 파란 펜으로 체크만 해두면 이사 직원들이 뒤에 서있다가 순간 전달 장치로 짐들을 새 집으로 전달해주는 것이다 (`vec2`의 상황).  
즉, 박스에 짐을 담은 채 이사하는 것과, 짐 자체만 새 집으로 전달하는 것은 다르다는 것이다. 이것이 std::move의 개념이다.

### 사용 예제

```cpp
#include <iostream>
#include <vector>

int main() {
    // 원래의 벡터 초기화
    std::vector<int> original = {1, 2, 3, 4, 5};

    // 이동 생성자를 사용하여 'original'의 내용을 'moved_to'로 이동
    std::vector<int> moved_to = std::move(original);

    // 'original'과 'moved_to'의 내용 출력
    // 출력: Original vector:
    std::cout << "Original vector: ";
    for (auto& item : original) {
        std::cout << item << " ";
    }

    // 출력: Moved to vector: 1 2 3 4 5
    std::cout << "\nMoved to vector: ";
    for (auto& item : moved_to) {
        std::cout << item << " ";
    }
    std::cout << std::endl;

    /** 출력: Size: 0
     *       Capacity: 0
     **/
    std::cout << "Size: " << original.size() << std::endl;
    std::cout << "Capacity: " << original.capacity() << std::endl;

    return 0;
}
```

이 코드에서 `std::move(original)`은 `original`이 지니고 있던 요소 값들을 rvalue 참조로 변환한다. 
이 변환된 rvalue 참조는 `moved_to` 벡터의 이동 생성자에 사용되어 original의 모든 원소를 moved_to로 이동시키게 된다. 
이 과정에서 original 벡터는 자연히 빈 상태가 됩니다. 그래서 size가 0이 되는 것뿐만 아니라, capacity 또한 자연스레 0이 된다.
이는 즉, 불필요하게 메모리 상에 vector가 차지하는 공간을 효과적으로 줄일 수 있다는 것을 뜻한다.

결론적으로, std::move의 주요 이점은 객체 간에 데이터를 "복사"하는 대신 "이동"을 통해 리소스를 효율적으로 재사용할 수 있다는 점이다.
이는 특히 robotics에서 map 단위의 수 백만개의 point cloud를 다루거나 SfM의 수 십만개 이상의 feature를 다룰 때 계산 효율적으로 데이터를 다룰 수 있다.

## 로보틱스에서 예제

[Faster-LIO](https://github.com/gaoxiang12/faster-lio/blob/6f6f1d6ea97071902a82c138f3359d4711873e2b/include/ivox3d/ivox3d_node.hpp#L223C1-L237C3)에서도 보면 이러한 이점을 활용하기 위해 point를 리턴할 때 std::move를 통해 지니고 있던 point를 어딘가로 전달하는 것을 확인할 수 있다.

```cpp
template <typename PointT, int dim>
struct IVoxNodePhc<PointT, dim>::PhcCube {
    uint32_t idx = 0;
    pcl::CentroidPoint<PointT> mean;

    PhcCube(uint32_t index, const PointT& pt) { mean.add(pt); }

    void AddPoint(const PointT& pt) { mean.add(pt); }

    PointT GetPoint() const {
        PointT pt;
        mean.get(pt);
        return std::move(pt);
    }
};
```

만약 한 번만 쓰고 버릴 point를 전달할 때는 std::move를 사용하여 효율적으로 데이터를 전달할 수 있음을 시사한다.
또한 ROS나 ROS2에서도 `EigenToPointCloud2`라는 함수를 통해 return한 PointCloud2를 불필요한 복사 없이 std::move를 통해 return된 데이터를 '전달'하는 것을 확인할 수 있다.

```cpp
    kpoints_publisher_->publish(std::move(EigenToPointCloud2(keypoints, header)));
    map_publisher_->publish(std::move(EigenToPointCloud2(kiss_map, kiss_pose, header)));
```

## 결론 

std:move의 개념 자체는 어렵지만, 무튼 원래 지니고 있던 데이터가 복사되는 것이 아니라 전달된다는 것만 잘 기억하면 메모리 & 퍼포먼스 효율적으로 사용이 가능하다.
특히 real-time perception에서 먼저 온 데이터가 먼저 나가야하는 상황(first in first out, FIFO)에서 std::move를 잘 쓰면 메모리를 최소한으로 사용하면서 속도 향상을 기대할 수 있다.

---

Robotics 연구자/개발자를 위한 Modern C++ 시리즈입니다.
사용된 코드들은 [여기](https://github.com/LimHyungTae/moderncpp_study)에서 확인할 수 있습니다.

{% include post_links_modern_cpp.html %}
