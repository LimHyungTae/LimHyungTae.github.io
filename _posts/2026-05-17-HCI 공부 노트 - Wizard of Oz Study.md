---
layout: post
title: HCI 공부 노트 - Wizard of Oz Study
subtitle: 구현보다 먼저 상호작용을 검증하는 법
tags: [HCI, Wizard of Oz, User Study, Prototyping]
comments: true
description: HCI에서 오래전부터 사용된 Wizard of Oz Study가 무엇인지, 왜 구현 전에 사람이나 실험 장치로 미래 시스템의 조건을 시뮬레이션하는지, 그리고 요즘 AI와 로봇 연구에서 왜 여전히 중요한지 정리한다.
permalink: /2026/05/17/hci-note-wizard-of-oz-study/
redirect_from:
  - '/2026-05-17-HCI 공부 노트 - Wizard of Oz Study/'
  - '/2026-05-17-HCI-공부-노트-Wizard-of-Oz-Study/'
---

## 들어가며

요즘 HCI 쪽을 공부하면서 연구분야를 확장하고 있다.
HCI에서 재미있게 보는 개념 중 하나가 **Wizard of Oz Study**이다.
이름만 들으면 뭔가 동화 같은데, 실제로는 꽤 진지한 연구 방법론이다.
HCI, intelligent user interface, natural language interface, HRI 쪽에서는 거의 초창기부터 이어져 온 사고방식이라고 봐도 된다.

핵심은 단순하다.
아직 완성되지 않은 시스템, 기능, 혹은 하드웨어 조건을 사용자에게 "이미 존재하는 것처럼" 경험하게 만든다.
전통적인 예시에서는 그 뒤에서 사람이 시스템의 일부 기능을 대신 수행한다.
사용자는 컴퓨터나 로봇, 음성 인터페이스, AI agent와 상호작용한다고 느끼지만, 실제로는 커튼 뒤의 사람이 일부 지능을 흉내내는 식이다.
그래서 이름이 Wizard of Oz이다.
오즈의 마법사가 거대한 존재처럼 보였지만, 사실 커튼 뒤에는 사람이 있었던 것처럼 말이다.

하지만 여기서 중요한 점이 있다.
Wizard of Oz Study에서 반드시 사람이 무언가를 직접 조작해야 하는 것은 아니다.
넓게 보면 핵심은 **아직 실제로 존재하지 않거나 구현되지 않은 조건을, 현재 가능한 방식으로 시뮬레이션해서 사용자의 경험을 먼저 관찰하는 것**이다.
wizard는 사람일 수도 있고, 실험 장치일 수도 있고, 현재 기기를 살짝 비틀어 만든 proxy일 수도 있다.

이 방법이 재미있는 이유는 "가짜로 그럴듯하게 보여주자"가 아니라,
**기술 구현보다 먼저 사용자의 상호작용을 검증하자**는 데 있다.

---

## Wizard of Oz Study의 뜻 

좁은 의미에서 Wizard of Oz Study는 사용자가 실제 시스템과 상호작용한다고 믿는 상황을 만들고,
실제로는 human operator, 즉 wizard가 시스템의 일부 혹은 전부를 조작하는 실험이다.

다만 HCI에서 이 개념은 조금 더 넓게 쓰이기도 한다.
완성되지 않은 기능, sensing condition, hardware configuration, autonomy level을 실험자가 통제 가능한 방식으로 대신 만들어두고,
사용자가 마치 그 미래 시스템이 실제로 있는 상황처럼 행동하게 만드는 실험도 같은 정신으로 이해할 수 있다.

예를 들어보자.

- 음성 인식기가 아직 없는데, 사용자가 말하면 뒤에서 사람이 받아 적고 시스템 출력처럼 보여준다.
- 자연어 대화 시스템이 아직 없는데, 사용자의 질문을 사람이 읽고 챗봇처럼 답한다.
- 로봇의 자율 주행 기능이 아직 불안정한데, 사용자는 자율 로봇이라고 생각하고, 뒤에서는 사람이 경로를 조정한다.
- AI assistant가 아직 구현되지 않았는데, 사용자는 agent에게 요청하고, 뒤에서는 연구자가 적절한 응답을 만들어준다.
- 아직 화면 아래에 카메라가 있는 스마트폰이 없는데, 폰을 거꾸로 들고 화면도 뒤집어서 "하단 카메라가 있는 스마트폰"처럼 실험한다.

겉으로 보기에는 "속이는 실험"처럼 보일 수 있다.
하지만 연구 관점에서는 목적이 다르다.
사용자를 속이는 것이 목적이 아니라, **사용자가 그런 시스템이 실제로 존재할 때 어떻게 행동하고, 무엇을 기대하고, 어디에서 막히는지**를 먼저 관찰하려는 것이다.

즉, 이 방법은 "기술이 가능한가?"보다 먼저 "이 상호작용이 의미가 있는가?"를 묻는다.

---

## 이렇게까지 하는 이유

공학을 하던 사람 입장에서는 처음에 이 방식이 조금 이상하게 느껴질 수 있다.
"그냥 구현하고 테스트하면 되는 거 아닌가?"라는 생각이 먼저 든다.
나도 robotics 쪽에서 오래 있다 보니, 일단 돌아가는 system을 만드는 것에 익숙하다.
센서 붙이고, perception 돌리고, planner 짜고, controller 붙이고, demo를 찍는 식이다.

그런데 HCI에서는 질문의 순서가 조금 다르다.
우리가 만들 수 있는가도 중요하지만, 그 전에 다음 질문이 중요하다.

- 사용자가 정말 이 기능을 원하는가?
- 사용자가 이 시스템을 어떤 mental model로 이해하는가?
- 시스템이 실수했을 때 사용자는 어떻게 반응하는가?
- 어느 정도의 지연 시간, 오류율, 설명 방식까지 사용자가 받아들일 수 있는가?
- 사용자는 우리가 의도한 방식으로 시스템을 쓰는가, 아니면 전혀 다른 방식으로 쓰는가?

이 질문들은 full implementation을 다 끝낸 뒤에야 물어보면 너무 늦다.
특히 AI나 로봇처럼 구현 비용이 큰 분야에서는 더 그렇다.
몇 달 동안 모델을 학습시키고, 로봇 시스템을 통합하고, UI를 다 만들었는데,
막상 user study를 해보니 사용자가 그 기능을 원하지 않거나 이해하지 못한다면 매우 슬픈 일이다.

Wizard of Oz Study는 이 순서를 뒤집는다.
**먼저 사용자의 행동과 기대를 관찰하고, 그 다음에 무엇을 구현할지 결정한다.**
이게 이 방법의 핵심적인 힘이다.

---

## 구현 전에 알 수 있는 것들

Wizard of Oz Study가 좋은 이유는 "아직 없는 기술"을 연구할 수 있게 해준다는 점이다.
특히 HCI에서는 시스템의 내부 성능보다 사용자 경험의 구조가 더 중요한 경우가 많다.

예를 들어 음성 비서 연구를 한다고 해보자.
이때 실제 speech recognition model을 완벽히 만드는 것은 아주 어려운 문제이다.
하지만 우리가 당장 알고 싶은 것이 "이 task에서 사용자가 음성 명령을 편하게 느끼는가?"라면,
완벽한 음성 인식기를 먼저 만들 필요는 없을 수 있다.
뒤에서 사람이 음성을 듣고 텍스트로 바꿔주면, 사용자는 어느 정도 실제 음성 시스템처럼 경험할 수 있다.

이렇게 하면 다음과 같은 것을 미리 볼 수 있다.

- 사용자가 어떤 표현으로 명령을 내리는지
- 시스템이 어느 정도로 proactive하게 행동해도 괜찮은지
- 사용자가 시스템을 도구로 보는지, 동료로 보는지, 자동화 장치로 보는지
- 실패 상황에서 사용자가 blame을 누구에게 돌리는지
- 설명이 필요한 순간과 필요하지 않은 순간이 언제인지

이런 데이터는 나중에 실제 모델이나 interface를 설계할 때 굉장히 중요하다.
결국 좋은 시스템은 "기술적으로 가능한 것"과 "사용자가 자연스럽게 받아들이는 것"의 교집합 위에 있어야 하기 때문이다.

---

## HiFiGaze의 하단 카메라 실험

이 개념을 이해하는 데 좋은 예시가 김태준 박사의 **HiFiGaze** 논문에 나온다.
이 논문은 스마트폰의 user-facing camera로 눈을 찍고,
사용자의 눈에 비친 화면 반사(screen reflection)와 실제 화면 내용(screen content)을 함께 사용해서 gaze estimation accuracy를 높이는 방법을 제안한다.

그런데 main study를 분석해보니, 사용자가 화면 아래쪽 target을 볼 때 error가 커지는 경향이 있었다.
원인은 꽤 직관적이다.
아래쪽을 볼수록 윗눈꺼풀과 속눈썹이 각막에 맺힌 화면 반사(corneal screen reflection)를 가리기 쉽다.
그렇다면 한 가지 가능한 해결책은 camera를 phone 위쪽이 아니라 아래쪽에 두는 것이다.
아래쪽에서 눈을 보면 이런 윗눈꺼풀/속눈썹 가림(occlusion)이 줄어들 수 있기 때문이다.

문제는 현재 일반적인 스마트폰에는 screen 아래쪽에 user-facing camera가 없다는 점이다.
그렇다고 하단 카메라가 있는 새로운 스마트폰 prototype을 만들어서 실험하기에는 비용이 너무 크다.
그래서 HiFiGaze의 supplemental study에서는 iPhone을 거꾸로 뒤집었다.
그러면 원래 위쪽에 있던 user-facing camera가 물리적으로 아래쪽에 오게 된다.
동시에 screen display도 invert해서, 실험상으로는 "하단 카메라가 있는 스마트폰"처럼 동작하게 만들었다.

이게 바로 Wizard of Oz Study의 좋은 예다.
여기서는 커튼 뒤에서 사람이 뭔가를 조작하지 않는다.
대신 아직 존재하지 않는 하드웨어 configuration을 현재 device를 뒤집는 방식으로 시뮬레이션한다.
즉, wizard가 사람이 아니라 **실험 apparatus**인 셈이다.

논문에서는 이 supplemental study에서 기존 main study 참가자 중 10명을 다시 모집해 within-participants comparison을 했다.
결과적으로 화면 반사를 활용하는 모델들은 bottom-camera condition에서 error가 줄어드는 경향을 보였다.
예를 들어 Eye Crops + Reflection Vector는 약 8%, Eye Crops + Screen Thumbnail은 약 18% 개선되었다고 보고한다.
반면 conventional Eye Crops baseline은 bottom-camera condition에서 뚜렷한 개선이 없었다.

이 실험이 흥미로운 이유는, 실제로 하단 카메라 스마트폰을 만든 것이 아니라도,
"만약 미래에 하단 카메라가 들어간다면 이 방법이 더 좋아질까?"라는 질문에 어느 정도 답할 수 있었다는 점이다.
이게 HCI식 Wizard of Oz 사고방식의 핵심이다.
미래 기술을 완전히 구현하기 전에, 그 기술이 만들어낼 조건을 먼저 만들어보고 사람과 시스템의 관계를 관찰한다.

---

## 학계가 이어받은 정신

내가 이 개념에서 제일 흥미롭게 느낀 부분은, Wizard of Oz Study가 단순한 실험 테크닉이 아니라는 점이다.
이건 HCI 학계가 꽤 오랫동안 이어온 하나의 연구 태도에 가깝다.

그 태도는 이렇게 정리할 수 있을 것 같다.

> 아직 완성된 기술이 없어도, 미래의 상호작용은 지금 연구할 수 있다.

이게 꽤 중요한 말이다.
공학 연구에서는 종종 "일단 성능이 나와야 한다"는 압박이 크다.
정확도, latency, robustness, benchmark score 같은 것들이 연구의 중심이 된다.
물론 이것들은 중요하다.
하지만 HCI는 거기에 한 가지 질문을 더 얹는다.

**그래서 사람이 그 시스템과 어떻게 같이 살고, 일하고, 실수하고, 적응할 것인가?**

Wizard of Oz Study는 이 질문을 기술 완성 이후로 미루지 않는다.
오히려 기술이 완성되기 전에, 사람이 그 시스템을 어떻게 해석할지 먼저 본다.
그래서 AI agent, social robot, autonomous vehicle, mixed reality interface 같은 연구에서 여전히 자주 등장한다.
아직 시스템이 완벽하지 않아도, 인간과 시스템 사이의 상호작용은 실험할 수 있기 때문이다.

---

## 좋은 Wizard of Oz Study를 하려면

물론 대충 그럴듯하게 꾸민다고 좋은 실험이 되는 것은 아니다.
오히려 이 방법은 잘못 설계하면 굉장히 위험하다.
사람이 wizard 역할을 할 때 너무 똑똑하게 행동하면, 실제 시스템으로 구현 불가능한 경험을 테스트하게 된다.
반대로 wizard가 너무 일관성 없이 행동하면, 사용자의 반응이 시스템 디자인 때문인지 operator의 실수 때문인지 알 수 없어진다.
HiFiGaze처럼 apparatus로 미래 하드웨어 조건을 시뮬레이션하는 경우에도 마찬가지다.
현재 장치로 만든 proxy가 미래 장치의 어떤 성질을 잘 보존하고, 어떤 성질은 왜곡하는지 분명히 해야 한다.

그래서 좋은 Wizard of Oz Study를 하려면 적어도 다음을 명확히 해야 한다.

1. **무엇을 시뮬레이션하는가.**
   시스템의 어떤 부분이나 조건을 대신 만드는지 정확히 정해야 한다.
   자연어 이해인지, 응답 생성인지, 로봇의 이동 판단인지, 추천 알고리즘인지, 아니면 카메라 위치 같은 하드웨어 configuration인지가 불명확하면 실험 결과도 흐려진다.

2. **simulation protocol은 무엇인가.**
   사람이 조작하는 경우라면 wizard가 매번 즉흥적으로 판단하면 안 된다.
   가능한 입력, 응답 템플릿, 실패 처리, 지연 시간, 개입 범위 등을 미리 정해둬야 한다.
   장치로 조건을 대체하는 경우라면 device orientation, coordinate transform, display mapping, calibration procedure 같은 것들을 명확히 해야 한다.

3. **실제 구현 가능성과 얼마나 떨어져 있는가.**
   WOz는 미래 기술을 가정할 수 있지만, 완전히 마법 같은 시스템을 가정하면 연구적으로 위험하다.
   "지금은 사람이 대신하지만, 나중에 어떤 기술로 대체할 수 있는가?" 또는 "지금은 proxy apparatus로 만들었지만, 미래 hardware와 어떤 점에서 동등한가?"에 대한 설명이 필요하다.

4. **사용자에게 언제, 어떻게 설명할 것인가.**
   deception이 들어가는 실험이라면 윤리적으로 매우 조심해야 한다.
   실험 전 동의 절차, 실험 후 debriefing, 수집 데이터의 범위, 참가자가 받을 수 있는 불편함을 명확히 다뤄야 한다.

여기서 중요한 건, Wizard of Oz Study가 "몰래 사람을 쓰면 된다"가 아니라는 점이다.
논문에서는 오히려 wizard가 어떤 역할을 했는지, 어떤 protocol로 움직였는지, apparatus가 어떤 미래 조건을 대신했는지, 사용자가 무엇을 알고 무엇을 몰랐는지 투명하게 설명해야 한다.

---

## 요즘 AI 연구와도 잘 맞는다

요즘은 LLM 때문에 이 개념이 더 흥미로워졌다.
예전에는 사람이 시스템 뒤에서 AI인 척했다면,
이제는 AI가 사람처럼 꽤 그럴듯하게 답하는 상황이 되었다.
그래도 Wizard of Oz의 정신은 여전히 유효하다.

예를 들어 새로운 AI research assistant를 만든다고 해보자.
처음부터 RAG pipeline, tool calling, memory, evaluation harness, UI를 전부 만들 필요는 없을 수 있다.
먼저 사용자가 어떤 순간에 도움을 원하는지,
어떤 종류의 suggestion을 신뢰하는지,
AI가 어느 정도로 개입하면 귀찮아지는지,
틀린 답을 했을 때 사용자가 어떻게 회복하는지를 보는 것이 더 중요할 수 있다.

이때 일부 기능을 사람이 대신하거나, LLM 출력 중 일부를 사람이 고르거나 수정하는 방식으로 WOz 실험을 만들 수 있다.
그러면 full system을 만들기 전에 interaction design의 큰 방향을 검증할 수 있다.

로봇 연구에서도 마찬가지다.
자율성이 100% 완성되지 않아도, 사용자가 로봇의 어떤 움직임을 안전하다고 느끼는지,
언제 로봇이 설명해야 하는지,
어느 정도의 주도권을 로봇에게 넘길 수 있는지를 먼저 볼 수 있다.
이런 질문은 단순히 navigation success rate만으로는 답하기 어렵다.

---

## 개인적으로 느낀 점

Wizard of Oz Study를 공부하면서, HCI가 왜 "사람을 보는 학문"인지 조금 더 이해하게 되었다.
공학자는 종종 system 내부를 먼저 본다.
모델 구조, 최적화, 데이터셋, metric, architecture를 본다.
반면 HCI는 system 바깥에서 사람이 실제로 무엇을 하는지를 집요하게 본다.

이 둘은 서로 반대가 아니다.
오히려 좋은 연구는 둘을 같이 봐야 한다.
기술이 없으면 상호작용은 공허해지고,
상호작용에 대한 이해가 없으면 기술은 종종 쓸모없는 방향으로 정교해진다.

Wizard of Oz Study는 그 사이에 있는 매우 실용적인 다리 같다.
아직 기술이 완성되지 않았을 때도,
우리는 사용자의 기대와 행동을 먼저 관찰할 수 있다.
그리고 그 관찰을 바탕으로 더 나은 기술을 만들 수 있다.

무튼 HCI를 공부하다 보면 이런 개념들이 꽤 자주 나오는데,
Wizard of Oz Study는 그중에서도 "왜 HCI가 단순한 UI 디자인이 아닌가"를 잘 보여주는 사례라고 생각한다.
겉으로는 단순히 그럴듯한 실험 조건을 꾸미는 방법처럼 보이지만,
그 안에는 미래의 기술을 사람의 경험 속에서 먼저 시험해보려는 학문적 태도가 들어 있다.

---

## 정리

- 좁은 의미의 Wizard of Oz Study는 사용자가 실제 시스템과 상호작용한다고 느끼게 하되, 뒤에서 사람이 일부 기능을 대신 수행하는 연구 방법이다.
- 더 넓게는 아직 구현되지 않은 기능, sensing condition, hardware configuration을 현재 가능한 방식으로 시뮬레이션하는 연구 방법이라고 볼 수 있다.
- 목적은 사용자를 속이는 것이 아니라, 구현 전에 사용자의 행동, 기대, mental model을 관찰하는 것이다.
- HCI에서는 "기술을 만들 수 있는가"뿐 아니라 "사람이 그 기술과 어떻게 상호작용하는가"가 핵심 질문이다.
- AI, 로봇, 음성 인터페이스, intelligent agent처럼 구현 비용이 큰 분야에서 특히 유용하다.
- HiFiGaze의 하단 카메라 실험처럼, 사람이 직접 조작하지 않아도 미래 hardware condition을 proxy로 만들면 WOz의 정신에 가깝다.
- 다만 wizard의 역할, 행동 규칙, apparatus setup, 지연 시간, 실패 처리, debriefing 같은 실험 설계를 투명하게 다뤄야 한다.

요약하면 Wizard of Oz Study는 이런 말로 정리할 수 있을 것 같다.

> 완성된 시스템을 만들기 전에, 완성된 시스템이 있을 때의 인간 경험을 먼저 연구하자.

이게 아마 HCI 학계가 오래전부터 이어받아 온 꽤 중요한 정신 중 하나가 아닐까 싶다.

---

## References

- J. F. Kelley, [An empirical methodology for writing user-friendly natural language computer applications](https://dl.acm.org/doi/10.1145/800045.801609), CHI 1983.
- J. D. Gould, J. Conti, and T. Hovanyecz, [Composing Letters with a Simulated Listening Typewriter](https://research.ibm.com/publications/composing-letters-with-a-simulated-listening-typewriter--1), Communications of the ACM, 1983.
- N. Dahlback, A. Jonsson, and L. Ahrenberg, [Wizard of Oz studies: why and how](https://www.sciencedirect.com/science/article/abs/pii/095070519390017N), Knowledge-Based Systems, 1993.
- T. Kim, V. Mollyn, R. Arakawa, and C. Harrison, [HiFiGaze: Improving Eye Tracking Accuracy Using Screen Content Knowledge](https://doi.org/10.1145/3772318.3791339), CHI 2026.
