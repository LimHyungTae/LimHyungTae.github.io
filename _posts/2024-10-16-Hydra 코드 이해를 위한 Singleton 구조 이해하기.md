---
layout: post
title: Hydra 코드 이해를 위한 Singleton 구조 이해하기
subtitle:
tags: [Ubuntu, GitHub]
comments: true
---

## Introduction

최근에는 연구실에서 [Hydra](https://www.youtube.com/watch?v=qZg2lSeTuvM)의 코드 유지 밎 배포를 연구실 동료와 함께하고 있다.
그런데, 코드를 읽는 중 도통 이해가 되지 않는 구간이 있어서 Nathan에게 직접 물어보았는데,
Singleton(싱글톤)이라는 신기한 구조에 대해 설명을 해주어서, 기록을 남기고자 한다.

## Why Do We Need to Know Singleton?

Hydra 코드를 살펴보다가 현재 해당 class가 어디에 사용되고 있는지 살펴보았는데,
**그 어느 곳에서 member 변수로 선언된 곳이 없다**는 것을 알게 되었다(이 것이 가장 큰 당황 포인트인듯)...
예로 들어 `hydra_ros`에서 queue에 `push_back()`을 하고는, 
`hydra`라는 다른 package에서, member 변수로 queue가 선언되지 않았는데, 갑자기 `queue.front()`를 한다는 식이다.

그런데 자세히보면 이러한 요상한 코드는 다들 `instance()`라는 함수를 사용하는 것을 확인할 수 있다:

```cpp
      const auto& info = hydra::GlobalInfo::instance();
```

```cpp
    auto& queue = PipelineQueues::instance().backend_queue;
```

~~해당 GlobalInfo가 어느 곳에서 다른 Class 내부에 선언이 안돼 있는데 뭐가 어떻게 동작하고 있는 거지!~~

## What Is Singleton?

싱글톤 패턴 자체는 [이 블로그](https://inpa.tistory.com/entry/GOF-%F0%9F%92%A0-%EC%8B%B1%EA%B8%80%ED%86%A4Singleton-%ED%8C%A8%ED%84%B4-%EA%BC%BC%EA%BC%BC%ED%95%98%EA%B2%8C-%EC%95%8C%EC%95%84%EB%B3%B4%EC%9E%90)에 아주 상세하게 설명되어 있다.
간단히 말하자면, 여러 클래스에서 같은 전역 변수를 공유할 때 유용한 디자인 패턴이라고 할 수 있다.

위의 코드를 보면 다들 `instance()` 함수로 객체를 가져오는 부분이 있다. 이는 싱글톤 패턴을 사용하는 대표적인 방식인데,
Singleton 패턴은 아래 Hydra의 [global_info.h](https://github.com/MIT-SPARK/Hydra/blob/main/include/hydra/common/global_info.h) 와 [global_info.cpp](https://github.com/MIT-SPARK/Hydra/blob/main/src/common/global_info.cpp)를 보면 확인할 수 있다.
특히 주목해야할 부분은 `global_info.h`에서 `instance_` 변수를 `unique_ptr`로 선언해둔 부분과: 

```cpp
// NOTE: You MUST put `static` in the member variable declaration
static std::unique_ptr<GlobalInfo> instance_;
```

`global_info.cpp`의 아래 부분을 부분을 보면 확인할 수 있다:

```cpp
GlobalInfo& GlobalInfo::instance() {
  if (!instance_) {
    instance_.reset(new GlobalInfo());
  }
  return *instance_;
}
```

이 코드 덕분에, 다른 cpp 파일에서도 `GlobalInfo::instance()`를 호출해 동일한 정보를 어디서든지 접근할 수 있게 된다.
부연 설명을 하자면, `global_info.h` 파일 내에 `GlobalInfo`라는 클래스에서 static으로 `instance_`를 지니고 있기 때문에,
굳이 해당 클래스를 member 변수로 선언하지 않더라도 project 내의 어디에서든 `GlobalInfo::instance()`라는 명령어를 통해 `GlobalInfo`에 접근할 수 있고,
`instance_`가 한 번이라도 선언되어 있지 않으면, i.e., `if (!instance_)`가 `true`로 동작하면, `instance_`라는 포인터가 새로운 `GlobalInfo()`를 가리킬 수 있게 되는 것이다. 

## 결론

`static`을 member 변수에 적용하여서 프로그램이 종료되기 전까지 영구적으로 꺼내 먹을 수 있는(?) 이 코드 패턴이 굉장히 인상 깊었다.
그리고 Hydra 내에서는 위의 global 변수들 뿐만 아니라 센서 데이터, SLAM 데이터 등도 위와 같은 Singleton 패턴으로 되어 있다.
그리고 이러한 이유로, 작년에 석사생 친구들이랑 나랑 Hydra 코드가 어떻게 동작하는지 잘 모르고 우왕좌왕했었는데, 이제는 파이프라인을 잘 알게 된 거 같다.

Robotics에서 수많은 코드를 보았지만, 이렇게 정성 들여서 코드를 짠 경우는 거의 없는 것 같다.
SLAM 코드를 작성할 때는 여러 프로세스가 동시에 정보를 접근해야 하는 경우가 많은데, 앞으로 SLAM 코드에 이런 Singleton 패턴을 적용해보는 것도 좋을 것 같다.



