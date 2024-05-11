---
layout: post
title: Modern C++ for Robotics 3. Lambda Expression의 Anonymous Function과 Named Function
subtitle: Anoynmous and Named Function in Lambda Expression
tags: [C++, Eigen, Robotics]
comments: true
---

## Anonymous Function과 Named Function

Lambda expression을 두 가지 유형으로 나눌 수 있는데, 이는 Anonymous function과 Named function이다.

### Anonymous Function (익명 함수)

이전에 본 예제들처럼, 함수에 이름을 붙이지 않는 경우를 anonymous function이라고 한다. 아래의 sorting 예제를 다시 보자:

```cpp
std::sort(numbers2.begin(), numbers2.end(), [](int a, int b) { return a > b; });
```

위와 같이 함수에 이름을 굳이 지정하지 않아도 되는 상황에서 유용하게 사용된다 (원래 함수의 이름 부분을 `[]`로 대신해서 표현한다고 외우면 기억하기 쉽다!).
예제들은 뒤의 STL의 알고리즘들을 소개할 때 다시 다루겠지만, 
이를 통해 코드의 가독성을 높이고, 코드의 재사용성을 높이며, 코드의 유지보수를 쉽게 만들어준다. 

기괴하게(?) 사용하자면 아래와 같이 사용하는 것도 가능하다:

```cpp
#include <iostream>

using namespace std;

int main() {

  int c = 0;
//  []() { ++c;}(); // (x) It's wrong because 'c' is not defined nor captured
  [&]() { ++c;}();
  [&](int &d) { ++d;}(c);

  std::cout << c << std::endl;

  return 0;
}
```

Lambda expression을 잘 이해했다면 위의 결과에서 `c`가 2로 출력되는 것을 짐작할 수 있다. 

### Named Function 

재밌는 것은, lambda expression을 통해 sorting에서 쓰인 anonymous function을 named function으로 바꿔서 쓸 수도 있다:

```cpp
auto decending_order = [](int a, int b) { return a > b; };
std::sort(numbers3.begin(), numbers3.end(), decending_order);
```

아주 신기하게도, 함수를 변수처럼 사용할 수 있다! (이를 어려운 말로 '일급 객체'라고 부른다. 뭔가가 특별해서 '일급'이 아니라, 우리가 주로 선언하는 변수들처럼 사용이 가능하다는 것을 의미한다.)
`decending_order`가 어떤 큰 코드 내에서 '딱 한 번'만 쓰인다면 (즉 함수로서 반복되서 사용될 가능성이 적다면) 굳이 이 함수를 클래스의 멤버 함수나 저어어 멀리 있는 헤더파일에 정의할 필요가 없다는 것을 시사한다.

또한, named function을 사용해서 굳이 코드에 주석을 달지 않더라도 코드의 의미를 알 수 있게 해준다. 아래는 내가 Cyrill Stachniss 교수님네에 있을 때의 동료인 Nacho의ㅣ [KISS-ICP의 일부분](https://github.com/PRBonn/kiss-icp/blob/1129b6e451222a891a26ddfdb77d719ce481534b/cpp/kiss_icp/core/Threshold.cpp#L37C1-L48C2)인데:

```cpp
void AdaptiveThreshold::UpdateModelDeviation(const Sophus::SE3d &current_deviation) {
    const double model_error = [&]() {
        const double theta = Eigen::AngleAxisd(current_deviation.rotationMatrix()).angle();
        const double delta_rot = 2.0 * max_range_ * std::sin(theta / 2.0);
        const double delta_trans = current_deviation.translation().norm();
        return delta_trans + delta_rot;
    }();
    if (model_error > min_motion_threshold_) {
        model_sse_ += model_error * model_error;
        num_samples_++;
    }
}
```

입력값을 받아서 주저리주저리 코드를 적는 것보다, 위의 네 줄이 model의 error를 구하는 부분이라고 알려주어 코드를 읽는 사람이 더 쉽게 이해할 수 있게 해준다 (물론 lambda expression과 친숙하지 않은 이에게는 고통일지도...).
즉, 위의 코드에서는 함수의 black box 성질을 이용해 '마! error가 어떻게 계산되는지는 내가 잘 짜뒀으니 대충 이 과정을 통해 `model_error`가 리턴된다'를 알려 줄 수 있다는 것이다. 
이를 코드를 추상화(abstraction)한다고 표현하는데, 
핵심적인 기능과 개념에 집중하면서 불필요한 세부사항을 숨기는 것을 의미한다. 
마치 건축가가 건물의 청사진을 그릴 때, 벽돌이나 철근 콘크리트와 같은 구체적인 세부사항보다는 건물의 전체적인 구조와 디자인에 집중하는 것과 유사하다고 생각할 수 있다..
즉, 복잡한 내부 로직을 감추고 사용자나 다른 프로그램 개발자가 간편하게 사용할 수 있는 인터페이스를 제공함으로써, 프로그램의 복잡도를 관리하고 사용의 용이성을 높인다. 


만약에 위의 코드가 아래와 같이 적혀 있다고 생각해보자:

```cpp
void AdaptiveThreshold::UpdateModelDeviation(const Sophus::SE3d &current_deviation) {
    const double theta = Eigen::AngleAxisd(current_deviation.rotationMatrix()).angle();
    const double delta_rot = 2.0 * max_range_ * std::sin(theta / 2.0);
    const double delta_trans = current_deviation.translation().norm();
    const doulbe model_error = delta_trans + delta_rot;
 
    if (model_error > min_motion_threshold_) {
        model_sse_ += model_error * model_error;
        num_samples_++;
    }
}
```

그럼 읽는 사람 입장에서 `theta`, `delta_rot`, `delta_tans`가 각각 무엇을 의미하는지 한 번 더 생각하게 만든다 (그리고 이것은 사용자 입장에서는 전혀 할 필요가 없는 행위임에도 불구하고 코드를 해석하는 데에 에너지를 더 쏟게 만든다). 

### Lambda Expression을 통한 병렬 처리 

모오-던한 open source 코드에서는 lambda expression을 통해 병렬 처리를 하는 경우가 많다. 이제는 아래와 같은 코드를 보더라도 두려워 할 필요는 없다 (코드는 내가 박사 과정 시절 연구한 ground segmentation 알고리즘인 [Patchwork](https://github.com/LimHyungTae/patchwork/blob/531b8ecb55421d5c6843af412a906cb785e027b3/include/patchwork/patchwork.hpp#L643C5-L683C8)에서 발췌):

```cpp
tbb::parallel_for(tbb::blocked_range<int>(0, num_patches),
             [&](tbb::blocked_range<int> r) {
  for (int i = r.begin(); i < r.end(); ++i) {
      const auto &patch_idx     = patch_indices_[i];
      const int  zone_idx       = patch_idx.zone_idx_;
      const int  ring_idx       = patch_idx.ring_idx_;
      const int  sector_idx     = patch_idx.sector_idx_;
      const int  concentric_idx = patch_idx.concentric_idx_;

      auto &patch = ConcentricZoneModel_[zone_idx][ring_idx][sector_idx];

      auto &feat                 = features_[i];
      auto &regionwise_ground    = regionwise_grounds_[i];
      auto &regionwise_nonground = regionwise_nongrounds_[i];
      auto &status               = statuses_[i];

      if ((int)patch.points.size() > num_min_pts_) {
          // 22.05.02 update
          // Region-wise sorting is adopted, which is much faster than global sorting!
          sort(patch.points.begin(), patch.points.end(), point_z_cmp<PointT>);
          extract_piecewiseground(zone_idx, patch, feat, regionwise_ground, regionwise_nonground);

          const double ground_z_vec       = abs(feat.normal_(2));
          const double ground_z_elevation = feat.mean_(2);
          const double surface_variable   =
                           feat.singular_values_.minCoeff() /
                               (feat.singular_values_(0) + feat.singular_values_(1) + feat.singular_values_(2));

          status = determine_ground_likelihood_estimation_status(concentric_idx, ground_z_vec,
                                                                 ground_z_elevation, surface_variable);
      } else {
          // Why? Because it is better to reject noise points
          // That is, these noise points sometimes lead to mis-recognition or wrong clustering
          // Thus, in practice, just rejecting them is better than using them
          // But note that this may degrade quantitative ground segmentation performance
          regionwise_ground = patch;
          regionwise_nonground.clear();
          status = FEW_POINTS;
      }
  }
});
```

위의 코드를 자세히 알 필요는 없지만, `tbb::parallel_for`을 통해 병렬 처리를 할 때 각 스레드가 처리해야 할 함수의 요소를 lambda function으로 표현하는 것을 확인할 수 있다. 
이를 통한 장점은 lambda function도 '함수'이기 때문에 함수의 내부에서 새로운 변수를 선언하면 이 변수들은 로컬 변수가 되고, segmentation fault에서 자유로워진다. 
예를 들어, 위의 코드에서 각 스레드들이 `zone_idx`, `ring_idx`, `sector_idx`, `concentric_idx`, `ground_z_vec`, `ground_z_elevation`, `surface_variable` 등을 로컬 변수로 편하게 선언해서 사용하는 것을 볼 수 있다. 
따라서 코드가 매우 thread-safe하게 된다!

로보틱스 분야에서는 '그래서 네 알고리즘이 real-time으로 쓸 수 있냐?'에 대해 생각보다 엄격하게 평가하지만, multi-threading 기법을 잘 활용하면 알고리즘을 real-time으로 사용할 수 있는 척 할 수 있다
(물론 얼마만큼의 CPU를 쓰는지는 논문들 내에서 안 알려주지 않는다...)

## 결론

이번 글에서는 lambda expression에 대해 알아보았다. Lambda expression은 C++11부터 지원되는 기능으로, 함수를 정의하지 않고도 사용할 수 있게 해준다. 이를 통해 코드의 가독성을 높이고, 코드의 재사용성을 높이며, 유지보수를 쉽게 만들어준다.
그리고 향후 소위 '좋은 오픈 소스'에서는 lambda expression이 심심치 않게 사용되는 것을 볼 수 있는데, 
모쪼록 이 글을 통해서 lambda expression이 무엇인지, 어떻게 사용하는지 알게 되었으면 좋겠다. 끝!

---

Robotics 연구자/개발자를 위한 Modern C++ 시리즈입니다.
사용된 코드들은 [여기](https://github.com/LimHyungTae/moderncpp_study)에서 확인할 수 있습니다.

{% include post_links_modern_cpp.html %}
