---
layout: post
title: Modern C++ for Robotics 4. std::for_each() 쉬운 설명 & 예제
subtitle: Easy Explanation & Examples of std::for_each()
tags: [C++, Eigen, Robotics]
comments: true
---

## 흔히 아는 C++에서의 for문 

로보틱스 분야에서 기장 먼저 알아 둬야할 함수형 프로그래밍의 기본 중 하나인 `std::for_each`에 대해 알아보자.

우리가 흔히 아는 `for`문은 반복문을 사용할 때 가장 많이 사용하는 방법 중 하나로, 만약 C++ 처음 배웠다면 아래와 같이 주로 작성해왔을 것이다:

```cpp
std::vector<int> v = {1, 2, 3, 4, 5};

// 각 요소를 2배로 만드는 for문
for (int i = 0; i < v.size(); ++i) {
    v[i] *= 2;
}
```

그리고 코딩을 좀 해본 이라면 아래와 같이 C++11에서 도입된 범위 기반 for문을 사용해본 적이 있을 것이다:

```cpp

for (auto &i : v) {
    i *= 2;
}
```

위의 범위 기반 for문은, '어떤 container를 순회할 때 굳이 `int i = 0`과 같이 범위를 선언해서 요소를 탐색해야 하나?'하는 질문을 시작으로,
for문을 좀 더 간소하게 표현할 수 있도록 해주는 표현이다.
특히, 'v의 type을 아는데 우리가 그 요소의 타입을 굳이 또 명시해줘야 하나?'라는 개발자의 귀차니즘과 결합하여 주로 auto와 혼합하여 사용하여 표현을 간결히 할 수 있다.

## std::for_each

`std::for_each`은 C++17에서 도입된 개념으로, 아래와 같이 lambda expression을 통해 작성할 수 있다:

```cpp
std::for_each(v.begin(), v.end(), [](int &x) { x *= 2; });
```

앞에서 기억하듯이, 원래 함수의 이름이 들어가야할 부분을 `[]`로만 대체해주면 된다.
std::for_each는 진짜진짜 중요한데, 왜냐하면 std::for_each만 잘 쓰면 **병렬처리를 손쉽게 할 수 있기 때문**이다.
다양한 세팅이 있지만 우리가 기억할 것은 아래와 같이 가장 앞쪽에 `std::execution::par_unseq`를 추가해주면 된다.
의미는 "par"allel하게 "unseq"uenced하게 실행한다는 뜻이다. 즉, CPU가 편할대로 임의의 index로 접근하면서 해당 데이터를 병렬처리를 해준다는 뜻이다:

```cpp
std::for_each(std::execution::par_unseq, v.begin(), v.end(), [](int &x) { x *= 2; }); 
```

이렇게 손쉽게 병렬 처리가 가능해진 것은 C++17 때 Intel에서 병렬처리 알고리즘인 TBB를 C++의 STL에 기증했기 때문이다. 
따라서 컴퓨터에 inteltbb만 잘 설치되어 있다면 알고리즘의 병렬처리를 용이하게 해준다.
하지만 늘 그렇듯이, 병렬 처리를 할 때는 **vector나 데이터들을 독립적으로 잘 접근하는지**을 잘 세팅해야 한다. 이는 다른 포스트에서 좀 더 자세히 다루도록 하겠다 (작성 예정).
간략히만 말하자면, for_each 문 내에서 바깥에 있는 변수를 resize하거나, 바깥의 vector에 emplace_back(혹은 push_back)을 하게 되면 충돌이 일어나니, 조심해야 한다.


## 로보틱스에서 예제

std::for_each는 너무나도 많이 써서 아마 '내 알고리즘이 빠르다!'라고 주장하는 레포지토리를 보면 거진 대부분 std::for_each를 사용하고 있을 것이다. 

대표적으로, Faster-LIO 코드를 각 포인트마다 surface를 찾아서 residual을 계산하는 부분을 보면 for_each + std::par_unseq의 조합으로 수 만개 ~ 수 십만개의 데이터를 빠르게 처리하고 있는 것을 볼 수 있다:

```cpp
// 1. 각 포인트에 대한 결과 값들을 저장할 변수들을 미리 선언
can_down_world_->resize(cur_pts);
nearest_points_.resize(cur_pts);
residuals_.resize(cur_pts, 0);
point_selected_surf_.resize(cur_pts, true);
plane_coef_.resize(cur_pts, common::V4F::Zero());
.
.
.
// 2. 각 포인트에 대해 병렬로 처리하여 `point_selected_surf_`와 residual_`에 결과 값 할당
std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
    PointType &point_body = scan_down_body_->points[i];
    PointType &point_world = scan_down_world_->points[i];

    /* transform to world frame */
    common::V3F p_body = point_body.getVector3fMap();
    point_world.getVector3fMap() = R_wl * p_body + t_wl;
    point_world.intensity = point_body.intensity;

    auto &points_near = nearest_points_[i];
    if (ekfom_data.converge) {
        /** Find the closest surfaces in the map **/
        ivox_->GetClosestPoint(point_world, points_near, options::NUM_MATCH_POINTS);
        point_selected_surf_[i] = points_near.size() >= options::MIN_NUM_MATCH_POINTS;
        if (point_selected_surf_[i]) {
            point_selected_surf_[i] =
                common::esti_plane(plane_coef_[i], points_near, options::ESTI_PLANE_THRESHOLD);
        }
    }

    if (point_selected_surf_[i]) {
        auto temp = point_world.getVector4fMap();
        temp[3] = 1.0;
        float pd2 = plane_coef_[i].dot(temp);

        bool valid_corr = p_body.norm() > 81 * pd2 * pd2;
        if (valid_corr) {
            point_selected_surf_[i] = true;
            residuals_[i] = pd2;
        }
    }
});
```

유심히 봐야할 점은, 해당 for문이 병렬로 돌기 전에 값을 저장해야하는 부분은 다들 index의 size만큼 resize를 해뒀다는 것이다.
즉, 병렬 처리 시 i번 째 point를 각 쓰레드에서 독립적으로 처리하기 때문에 각 쓰레드에서 i번 째 point에 대한 값을 저장할 공간을 미리 할당해두어야 한다.
만약 `residual_.clear()`후 `residuals_.emplace_back(pd2)`로 값을 넣게 되면 여러 쓰레드가 `residual_`의 끝 부분에 동시에 접근하여 충돌이 일어나, 최종적으로는 Segmentation fault 에러가 날 수 있다.

## 결론 

특별한 노력 없이 병렬 처리를 할 수 있는데 std:for_each를 안 쓸텐가? 
하지만 주의할 점은, **병렬 처리는 마법처럼 늘 속도를 빠르게 하는 게 아니라는 점**이다.
그러니 반드시 single thread로 구현을 해보고 요소 별로 병렬 처리화하면서 속도가 빨라졌는지 체크해보면서 개발하는 것이 중요하다.
무튼 이 글을 통해 독자의 논문 제목에도 'Fast'를 넣을 수 있게 되길 응원한다! ~~(혹시 논문쓰게 되면 내 논문들도 인용 좀...굽신굽신)~~

---

Robotics 연구자/개발자를 위한 Modern C++ 시리즈입니다.
사용된 코드들은 [여기](https://github.com/LimHyungTae/moderncpp_study)에서 확인할 수 있습니다.

{% include post_links_modern_cpp.html %}
