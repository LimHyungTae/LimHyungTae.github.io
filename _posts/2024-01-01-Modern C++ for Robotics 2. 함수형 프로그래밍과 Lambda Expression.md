---
layout: post
title: Modern C++ for Robotics 2. 함수형 프로그래밍과 Lambda Expression
subtitle: Comparison of element-wise operation and segment<>()
tags: [C++, Eigen, Robotics]
comments: true
---

## Introduction 

사실 가장 다루고 싶었던 것은 함수형 프로그래밍이었다. 만약 이 글을 읽고 있는 독자에게 '{1, 2, 3, 4, 5}의 vector가 있을 때 각 요소에 2를 곱해 2배로 만드는 코드를 짜라'고 했을 때, 머릿 속에서 아래와 같이 그려진다면: 

```cpp
std::vector<int> src = {1, 2, 3, 4, 5};
std::vector<int> dst;
for (int i = 0; i < src.size(); i++) {
    dst.emplace_back(src[i] * 2);
}
```

이 함수형 프로그래밍을 다루는 글을 정독해보면 큰 도움이 될 것이다. 함수형 프로그래밍을 잘 활용하면 아래와 같이 코드를 작성할 수 있다:

```cpp
std::vector<int> src = {1, 2, 3, 4, 5};
std::vector<int> dst;
dst.resize(src.size());
std::transform(src.begin(), src.end(), dst.begin(), [](int x) { return x * x; });
```

'뭐여 똑같이 네 줄이구만 ㅋ ㅋ'이라고 생각할 수 있겠지만, 함수형 프로그래밍의 가장 큰 장점은 모듈화와 확장성이다. 즉, 함수형 프로그래밍을 통해 **다른 함수를 인자로 다시 재사용하여서 코드를 작성**할 수 있다. 이는 코드의 재사용성을 높이고, 코드의 가독성을 높이며, 코드의 유지보수를 쉽게 만들어준다
(뒤에서 다시 설명하겠지만, 이는 또한 **병렬 처리**를 할 때 큰 장점이 된다. Robotics 분야같이 real-time으로 동작하는 게 생명인 분야에서는 병렬 처리만 좀 잘해줘도 알고리즘이 효율적인 척(?) 할 수 있다).


함수형 프로그래밍을 잘 활용하려면 크게 두 가지를 알아야 한다: a) Lambda expression과 b) std::algorithm이다.
따라서 앞으로는 이 두가지에 대해서 알아보도록 한다.  
함수형 프로그래밍은 자체에 대해 좀 더 자세히 알고 싶은 사람은 [여기](https://mangkyu.tistory.com/111) 설명이 잘 되어 있으니 참고하면 좋을듯 하다.

## Lambda Expression이란 

먼저 Lambda expression에 대해 알아보자. Lambda expression은 C++11부터 지원되는 기능으로, 함수를 정의하지 않고도 함수를 사용할 수 있게 해준다. 위에 예시를 다시 보면

```cpp
[](int x) { return x * x; }
```

요런 요상한 형태의 인자가 사용되는 것이 볼 수 있다. 이 것을 바로 Lambda expression이다 (사실 Modern C++에만 있는 것은 아니고, Python이나 다른 언어에서도 이 Lambda expression이 존재한다). 
아마 독자들 중 한번이라도 C++에서 sorting을 해보았다면, 아래와 같은 코드를 본적이 있을 것이다:

```cpp
#include <iostream>
#include <vector>
#include <algorithm>

// 비교 함수 정의
bool compare(int a, int b) {
    return a > b;  // 내림차순 정렬을 위해 'a'가 'b'보다 클 때 true 반환
}

int main() {
    // 정렬할 벡터
    std::vector<int> numbers = {4, 1, 3, 9, 7, 5, 8};

    // std::sort 함수 사용, 비교 함수로 'compare' 전달
    std::sort(numbers.begin(), numbers.end(), compare);

    // 정렬된 벡터 출력
    std::cout << "Sorted in descending order: ";
    for (int num : numbers) {
        std::cout << num << " ";
    }
    std::cout << std::endl;

    return 0;
}
```

그런데 '굳이 내림차순을 하기 위해서 다른 곳에 boolean 함수를 정의해야 하는가?'하는 의문이 든다. 위의 예제는 짧은 코드여서 크게 문제가 없지만, 코드가 길어지면 이런 함수를 외부에다 정의하는 것이 귀찮을 수 있고, 
또 다른 사람이 코드를 이해하기 위해서는 어쩔 수 없이 여러 파일을 여러보게끔 만들기도 한다. 

따라서 이러한 문제를 해결하기 위해 Lambda expression을 쓰는데, 이를 통해 표현하면 한 줄로 간결하게 표현이 가능하다:

```cpp
std::sort(numbers2.begin(), numbers2.end(), [](int a, int b) { return a > b; });
```

즉, 외부에 함수를 따로 선언하지 않아도 되고, 코드의 가독성도 높아진다. 그래서 Lambda expression의 프로그래밍 언어에서 코드를 더 쉽게 읽고 쓸 수 있도록 만드는 언어 구성 요소나 문법을 가리켜, 'Syntatic Sugar'라고도 많이 부른다.
추가적인 함수 선언 없이 쉽게 코드를 이해할 수 있게 만들어주기 때문이다 

(갑분 영어 상식: 영미권에서는 이 '달다'는 표현이 대체로 긍정적인 느낌을 주거나 상황을 매력적으로 만드는 것을 의미한다. 그러므로, 마치 음식에 설탕을 추가해 맛을 향상시키는 것처럼, 프로그래밍에서 이 "sugar"를 통해 코드를 더욱 매력적이고 사용하기 쉽게 만든다는 것을 뜻한다).

## Lambda Expression의 대표적 구조

Lambda expression이 처음에 쓰려고 하면 머리에 잘 기억이 남지 않는데, 나는 그래서 '내가 아는 괄호를 다 때려 넣으면 된다'고 외웠다. 실제로 lambda expression은 \[\](){}의, 세 개의 괄호로 구성되어 있다. 뒤의 두개는 ${함수 이름}(<변수가 들어갈 곳>) {<함수 내용이 들어갈 곳>;}으로 동일하고 함수의 이름 부분만 \[\]로 대체되었다고 생각하면 외우기 쉽다.

각각의 역할을 다음과 같다.

![lambda](..//img/lambda_expression.png)

1. \[\] 여기에는 주로 a) `&`, b) `=`, c) `(텅 빔), i.e., []`가 들어갈 수 있다. 
    - a) `&`는 참조로 변수를 받아올 때 사용한다. 
    - b) `=`는 값으로 변수를 받아올 때 사용한다. 
    - c) `[]`: 외부의 값은 추가적으로 사용하지 않겠다는 의미를 뜻한다. 즉, {} 내부가 외부의 변수를 사용하지 않는다는 의미이다. 
2. ()는 함수의 인자를 받아올 때 사용한다. 우리가 아는 함수의 입력 인자를 넣는 것과 동일하다. 
3. {}는 함수의 내용을 담는다. 자명하니 생략

즉 다시 `[](int a, int b) { return a > b; }`를 보면, 1) 외부의 변수는 사용하지 않으면서, 2) a, b를 받아서 3) a가 b보다 크면 true를 반환하는 함수를 만들어라는 의미이다.

따라서 STL의 algorithm을 사용할 때, Lambda expression에 사용할 때는 lambda expression에서 따로 함수의 이름을 지어주지 않으며 사용할 수 있다. 이를 익명 함수(anonymous function)이라고 부른다.
다음 글에서는 lambda expression을 어떻게 robotics 분야에서 잘 활용하는지 설명을 하고, 그 예시를 살펴본다.

---

{% include post_links_modern_cpp.html %}
