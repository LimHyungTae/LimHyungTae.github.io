---
layout: post
title: Python Numpy npz 쉽게 저장하고 불러오는 법
subtitle: Easy Explanation & Examples of Conditional Algorithms
tags: [C++, Eigen, Robotics]
comments: true
---

ChatGPT가 다 알려주는 마당에 무슨 의미가 있겠냐만은, 쉽게 npz로 저장하고 불러오는 방법을 알아보자.

개인적으로 python의 dictionary와 `**` 문법을 통해 아래와 같이 사용하는 것이 가장 효율적이라고 생각한다: 

```python
import numpy as np

results_dict = {}
results_dict["test"] = np.array([0.1, 0.2, 0.3])
results_dict["train"] = np.array([0.4, 0.5, 0.6])

np.savez('dictionary.npz', **results_dict)

loaded_dictionary = np.load('dictionary.npz')

# 딕셔너리로 변환
for key in loaded_dictionary:
  print(f"{key}: {loaded_dictionary[key]}")
```

여기서 `**`은 키워드 인수 언패킹(Keyword Argument Unpacking)인데, 함수 호출 시 사전(dictionary)에 있는 키-값 쌍을 키워드 인수로 전달한다.
(C++에는 없는 문법...역시 갓-이썬).

함수를 호출할 때와 정의할 때 두 가지로 나뉘어서 사용할 수 있다: 

### 함수 호출 시:

```python

def my_function(a, b, c):
    print(a, b, c)

my_dict = {'a': 1, 'b': 2, 'c': 3}
my_function(**my_dict)  # 출력: 1 2 3
```

### 함수 정의 시:
```python
def my_function(**kwargs):
    for key, value in kwargs.items():
        print(f"{key}: {value}")

my_function(a=1, b=2, c=3)
```
