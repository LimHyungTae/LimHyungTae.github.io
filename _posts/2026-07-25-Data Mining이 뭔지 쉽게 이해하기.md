---
layout: post
title: Data Mining이 뭔지 쉽게 이해하기
subtitle: 데이터 속에서 유용한 패턴을 찾아 지식으로 바꾸는 과정
tags: [Data Mining, Machine Learning, Statistics, Data Science, KDD]
comments: true
use_math: true
description: Data mining이 무엇인지 데이터 조회, 통계, 머신러닝과의 차이부터 설명한다. 장바구니 분석과 로봇 운행 로그 예시로 association rule, classification, clustering, anomaly detection의 의미와 실제 분석 절차를 정리한다.
permalink: /2026/07/25/what-is-data-mining/
redirect_from:
  - '/2026-07-25-Data Mining이 뭔지 쉽게 이해하기/'
  - '/2026-07-25-Data-Mining이-뭔지-쉽게-이해하기/'
---

## 들어가며

Data mining이라는 말을 처음 들으면 조금 거창하게 느껴진다.
엄청나게 큰 database에서 AI가 사람은 보지 못한 진실을 자동으로 캐내는 장면이 떠오를 수도 있다.

하지만 핵심은 데이터의 크기도, AI의 복잡함도 아니다.

데이터가 10 TB 있어도 이미 알고 있는 평균 하나만 계산했다면 data mining이라고 부르기 애매하다.
반대로 1만 개의 로봇 운행 log에서 사람이 미처 몰랐던 localization failure 조건을 찾아내고, 다른 기간의 log에서도 그 조건이 반복되는지 확인했다면 훌륭한 data mining이다.

내가 이해한 결론부터 말하면 다음과 같다.

> **Data mining은 많은 관측값 속에서 이전에 알지 못했던 유용한 구조, 관계, 규칙, 예외를 발견하고, 그것이 우연이나 데이터 수집 방식의 산물이 아닌지 검증하여 실제 판단이나 행동에 쓸 수 있는 지식으로 바꾸는 과정이다.**

여기서 중요한 단어는 **발견**, **검증**, **활용**이다.
그럴듯한 pattern 하나를 찾았다고 끝나는 것이 아니다.
새로운 데이터에서도 살아남고, 현실의 설명과 행동으로 연결되어야 쓸 만한 지식이 된다.

---

## 데이터와 지식은 다르다

먼저 data mining을 이해하려면 raw data와 knowledge를 구분해야 한다.

가게의 결제 기록에 다음과 같은 한 줄이 있다고 하자.

```text
2026-07-25 10:32, order_id=1842, pasta=1, tomato_sauce=1
```

이 한 줄은 관측된 사실, 즉 **record**이다.
이런 record를 수천 개 모으면 dataset이 된다.
하지만 record가 많다고 그 자체로 지식이 되지는 않는다.

대략 다음 단계를 거쳐야 한다.

| 단계 | 예시 |
|---|---|
| Data | 주문마다 어떤 품목이 팔렸는가 |
| Information | 지난달 pasta는 600번 팔렸다 |
| Pattern | pasta를 산 주문에는 tomato sauce도 자주 포함된다 |
| Knowledge | 이 관계는 우연보다 강하고, 다음 달 주문에서도 반복된다 |
| Action | 두 품목의 재고를 함께 관리하거나 추천에 반영한다 |

이 구분이 학문적으로 완전히 딱 잘리는 것은 아니다.
핵심은 **data mining의 목적이 record를 쌓는 데 있지 않고, 유용한 규칙성이나 의미 있는 예외를 찾아 의사결정 가능한 형태로 바꾸는 데 있다**는 점이다.

몇 가지 기본 용어도 여기서 잡고 가자.

- **Sample 또는 record**: 관측 한 건이다. 고객 한 명, 주문 한 건, 로봇 임무 한 번, LiDAR scan 한 장이 될 수 있다.
- **Feature**: sample을 설명하는 값이다. 속도, 날씨, 구매 품목, point density 등이 해당한다.
- **Target**: 맞히거나 설명하고 싶은 결과 전체이다. 고장 여부, 남은 수명, trajectory error 등이 해당한다.
- **Label**: 보통 정상/고장처럼 sample에 붙은 범주형 정답을 가리킨다. 문맥에 따라 target과 거의 같은 뜻으로 쓰이기도 한다.
- **Pattern**: 데이터에서 관찰되는 규칙성, 관계 또는 의미 있는 예외이다.
- **Model**: 발견한 pattern을 새로운 데이터에도 적용할 수 있도록 표현한 수학적 또는 계산적 규칙이다.

---

## 왜 이름이 Mining인가?

Mining은 광산에서 광물을 캐는 행위이다.
광산의 대부분은 흙과 돌이고, 우리가 원하는 광물은 그 안에 조금 섞여 있다.
마찬가지로 dataset에도 값은 아주 많지만 실제로 의사결정에 도움이 되는 pattern은 일부뿐이다.

```text
많은 raw record
  -> 후보 pattern 탐색
  -> 우연한 pattern 제거
  -> 새로운 데이터에서 검증
  -> 실제로 쓸 수 있는 knowledge
```

다만 이 비유에는 주의할 점이 있다.
광물은 원래 땅속에 있는 물체지만, 데이터의 pattern은 분석 방법과 관측 방식에 영향을 받는다.
어떤 sample을 모았는지, feature를 어떻게 만들었는지, 결측값을 어떻게 처리했는지에 따라 발견되는 pattern도 달라진다.

즉 data mining은 숨겨진 진실을 기계적으로 꺼내는 과정이 아니다.
**분석자가 만든 관측과 가정 위에서 후보 pattern을 찾고 검증하는 과정**에 가깝다.

---

## 좁은 의미와 넓은 의미

학술적으로는 **KDD**(Knowledge Discovery in Databases)와 data mining을 구분하기도 한다.

좁은 의미에서 data mining은 전체 KDD pipeline 중 algorithm으로 pattern을 뽑는 단계이다.

```text
데이터 선택
  -> 정제
  -> 변환
  -> Data mining
  -> Pattern 평가와 해석
```

반면 실무에서는 문제 정의, 데이터 정리, pattern 발견, 검증, 배포까지 전부 묶어서 data mining이라고 부르는 경우가 많다.

이 글에서는 이해를 돕기 위해 **넓은 의미**를 사용한다.
좋은 algorithm을 돌리는 것만으로는 실제 문제가 해결되지 않기 때문이다.
무엇을 한 행으로 볼지 정하고, 잘못된 log를 걸러내고, 미래 정보가 새어 들어오지 않게 나누고, 결과를 실제 행동으로 연결하는 과정까지 모두 중요하다.

---

## 단순 조회와 무엇이 다른가?

가게 주문 database를 예로 들면 차이가 명확하다.

- "지난달 pasta가 몇 개 팔렸나?"는 이미 알고 있는 질문에 값을 가져오는 **database query**이다.
- "여러 쇼핑몰에서 상품 가격을 자동으로 모은다"는 **data collection 또는 web scraping**이다.
- "pasta와 함께 반복해서 팔리는 품목은 무엇인가?"는 관계를 설명하는 **descriptive data mining**이다.
- "다음 고객이 pasta를 살 확률은 얼마인가?"는 data mining 안에서 미래 값을 맞히는 **predictive modeling**이다.
- "pasta 할인 때문에 sauce 판매가 늘었는가?"는 원인을 확인해야 하는 **causal inference** 문제이다.

물론 경계가 절대적인 것은 아니다.
SQL만으로 새로운 pattern을 탐색할 수도 있고, machine learning model이 data mining의 도구가 될 수도 있다.
중요한 것은 사용하는 library보다 **무엇을 알아내려는가**이다.

인접한 분야들을 비교하면 다음과 같다.

| 분야 | 중심 질문 | 대표적인 결과 |
|---|---|---|
| Database | 데이터를 어떻게 정확하고 빠르게 저장하고 조회할까? | Query 결과, 집계표 |
| Statistics | 관측된 관계의 불확실성은 얼마이며 모집단에도 성립할까? | 추정치, confidence interval, hypothesis test |
| Machine Learning | 새로운 입력에서도 잘 동작하는 함수나 표현을 어떻게 학습할까? | 예측 모델, embedding |
| Data Mining | 많은 데이터 속에 어떤 유용하고 검증 가능한 pattern이 있는가? | 규칙, cluster, anomaly, 예측 pattern |
| Data Science | 수집부터 분석, 제품화, 운영까지 데이터 문제 전체를 어떻게 해결할까? | 분석 결과와 운영 system |

이들은 서로 경쟁하는 별개의 분야가 아니다.
실제 project에서는 database로 데이터를 모으고, machine learning으로 pattern을 찾고, statistics로 신뢰성을 확인하며, 그 전체를 data science workflow 안에서 운영한다.

---

## Data Mining에서 주로 찾는 것

Data mining의 task는 "정답 label이 있는가?"와 "어떤 형태의 결과를 원하는가?"에 따라 나뉜다.

| 묻는 질문 | 대표 task | 예시 |
|---|---|---|
| 이미 알려진 종류 중 어디에 속하는가? | Classification | 정상 주행인지 localization failure인지 분류 |
| 연속적인 값이 얼마인가? | Regression | 부품 고장까지 남은 시간 또는 trajectory error 예측 |
| 정답 없이 어떤 집단이 존재하는가? | Clustering | 비슷한 SLAM failure mode끼리 묶기 |
| 무엇이 함께 나타나는가? | Association rule mining | Pasta와 tomato sauce의 동시 구매 |
| 평소와 유난히 다른 것은 무엇인가? | Anomaly detection | 비정상 sensor log 또는 calibration drift 탐지 |
| 어떤 사건 순서가 반복되는가? | Sequential pattern mining | Vibration 증가 → residual 증가 → tracking loss |
| 고차원 데이터를 어떻게 요약할까? | Dimensionality reduction | 수백 개 feature를 낮은 차원에서 시각화 |

Sequential pattern의 화살표는 관측된 시간 순서를 뜻할 뿐, 앞의 사건이 뒤의 사건을 일으켰다는 인과관계를 자동으로 뜻하지는 않는다.

Classification과 regression처럼 label을 사용하는 방법을 보통 **supervised learning**이라고 한다.
Clustering과 anomaly detection처럼 정답 없이 구조를 찾는 방법은 흔히 **unsupervised learning**에 속한다.

하지만 data mining은 supervised냐 unsupervised냐로만 결정되지 않는다.
어떤 경우에는 decision tree로 failure를 예측하면서 사람이 읽을 수 있는 규칙도 발견하고, clustering으로 나눈 failure group에 나중에 사람이 의미를 붙이기도 한다.

---

## 예제 1: 장바구니에서 Association Rule 찾기

가장 고전적인 예제인 장바구니 분석을 직접 계산해보자.
가상의 주문이 다섯 건 있다고 하자.

| 거래 | 구매 품목 |
|---|---|
| T1 | pasta, tomato sauce |
| T2 | pasta, tomato sauce, cheese |
| T3 | pasta, tomato sauce |
| T4 | tomato sauce, bread |
| T5 | bread, cheese |

여기서 다음 규칙을 발견했다고 하자.

```text
pasta를 산 주문 -> tomato sauce도 산다
```

이 화살표는 시간 순서나 원인 관계를 뜻하지 않는다.
"Pasta가 있는 거래를 조건으로 보았을 때 tomato sauce도 나타나는가?"라는 조건부 규칙을 간단히 적은 것이다.

이 규칙이 얼마나 의미 있는지 평가할 때 **support**, **confidence**, **lift**를 많이 사용한다.

### Support

Support(지지도)는 전체 거래 중 $$A$$와 $$B$$가 함께 등장한 비율이다.

$$
\text{support}(A \rightarrow B)
= P(A \cap B)
= \frac{\#(A와 B가 함께 등장한 거래)}{\#(전체 거래)}
$$

여기서 $$\cap$$는 $$A$$와 $$B$$가 동시에 나타난다는 뜻이고, $$\#$$은 해당 거래의 개수를 뜻한다.

Pasta와 tomato sauce는 T1, T2, T3에서 함께 등장했다.

$$
\text{support}
= \frac{3}{5}
= 0.60
$$

즉 전체 주문의 60%에 두 품목이 함께 들어 있다.

### Confidence

Confidence(신뢰도)는 $$A$$가 등장한 거래만 보았을 때 $$B$$도 함께 등장한 비율이다.
여기서 confidence는 association rule의 이름이며, 통계의 confidence interval과는 다른 개념이다.

$$
\text{confidence}(A \rightarrow B)
= P(B \mid A)
= \frac{P(A \cap B)}{P(A)}
$$

$$P(B \mid A)$$의 세로선은 "$$A$$가 주어졌을 때"라는 조건을 뜻한다.

Pasta가 들어간 주문은 T1, T2, T3이고, 세 거래 모두 tomato sauce를 포함한다.

$$
\text{confidence}
= \frac{3}{3}
= 1.00
$$

즉 이 작은 dataset에서는 pasta 구매자의 100%가 tomato sauce도 샀다.

하지만 confidence만 보면 안 된다.
Tomato sauce가 원래 거의 모든 주문에 들어가는 흔한 품목이라면, pasta와 특별한 관계가 없어도 confidence는 높게 나올 수 있다.

### Lift

Lift(향상도)는 $$A$$가 있을 때의 $$B$$ 확률을, 아무 조건 없이 본 $$B$$의 기본 확률과 비교한다.

$$
\text{lift}(A \rightarrow B)
= \frac{P(B \mid A)}{P(B)}
$$

Tomato sauce는 전체 다섯 주문 중 네 주문에 있으므로 $$P(B)=4/5=0.8$$이다.

$$
\text{lift}
= \frac{1.00}{0.80}
= 1.25
$$

이를 다음처럼 읽으면 된다.

- $$\text{lift}=1$$: 관측된 주변 확률을 기준으로, 두 품목이 독립일 때 기대되는 동시 등장 비율과 같다.
- $$\text{lift}>1$$: 두 품목이 baseline보다 더 자주 함께 나타난다.
- $$\text{lift}<1$$: 두 품목이 오히려 덜 함께 나타난다.

여기서는 pasta 주문에서 tomato sauce가 나타나는 비율이 전체 평균의 1.25배, 즉 25% 높은 비율이다.

역방향 규칙인 `tomato sauce -> pasta`의 confidence는 $$3/4=0.75$$로 달라진다.
그러나 lift는 $$0.75/(3/5)=1.25$$로 같다.
Confidence는 조건의 방향에 따라 달라지지만, 두 itemset 사이의 lift는 같은 결합 확률을 사용하므로 대칭이다.

### Python으로 그대로 계산하기

외부 package 없이도 계산할 수 있다.

```python
transactions = [
    {"pasta", "tomato_sauce"},
    {"pasta", "tomato_sauce", "cheese"},
    {"pasta", "tomato_sauce"},
    {"tomato_sauce", "bread"},
    {"bread", "cheese"},
]


def support(items):
    count = sum(items <= transaction for transaction in transactions)
    return count / len(transactions)


a = {"pasta"}
b = {"tomato_sauce"}

support_ab = support(a | b)
confidence = support_ab / support(a)
lift = confidence / support(b)

print(f"support    = {support_ab:.2f}")
print(f"confidence = {confidence:.2f}")
print(f"lift       = {lift:.2f}")
```

출력은 다음과 같다.

```text
support    = 0.60
confidence = 1.00
lift       = 1.25
```

이 결과만 보고 "pasta가 tomato sauce 구매를 일으켰다"고 말할 수는 없다.
두 품목을 함께 할인했거나, 원래 함께 만드는 요리가 많아서 동시에 샀을 수도 있다.
또한 거래가 다섯 건뿐이라 새 거래 한두 건만 들어와도 값이 크게 바뀐다.

즉 association rule은 **함께 나타나는 pattern**을 보여주지, **원인**을 증명하지 않는다.

---

## 예제 2: 로봇 운행 Log에서 Failure Pattern 찾기

이번에는 robotics 관점에서 보자.

배송 로봇 50대가 수행한 임무 10,000건을 모았다고 하자.
분석 단위는 LiDAR point 한 개나 scan 한 장이 아니라 **임무 한 번**이다.
분석 대상은 모두 출발 후 60초까지 관측 가능한 임무로 한정하고, 60초 이전에 발생한 recovery는 target에서 제외하거나 별도 cohort로 다룬다고 가정한다.

각 행에는 다음과 같은 값이 들어 있다.

| Column | 의미 |
|---|---|
| `robot_id` | 어떤 로봇이 수행했는가 |
| `route_id` | 어떤 경로를 주행했는가 |
| `weather` | 날씨 조건 |
| `firmware_version` | 탑재된 software version |
| `lidar_valid_return_ratio_60s` | 출발 후 첫 60초의 유효 LiDAR return 비율 |
| `recovery_after_60s` | 첫 60초 이후 localization을 잃어 recovery routine이 실행되었는가 |

Mining 결과 다음 숫자를 얻었다고 하자.

- 전체 임무: 10,000건
- 출발 60초 이후 localization loss로 recovery routine이 실행된 임무: 500건
- 첫 60초의 LiDAR 유효 return 비율이 55% 미만인 임무: 1,000건
- 그 1,000건 중 이후 recovery가 발생한 임무: 200건

찾은 규칙은 다음과 같다.

```text
초기 LiDAR 유효 return 비율 저하
  -> 이후 localization loss로 recovery routine 실행
```

여기서 recovery는 성공적인 복구 자체보다, localization loss가 발생해 복구 절차가 필요해졌다는 incident marker로 사용한다.
앞서 본 세 지표를 그대로 계산하면 다음과 같다.

| 지표 | 계산 | 의미 |
|---|---:|---|
| Support | $$200/10{,}000=2\%$$ | 전체 임무 중 조건과 recovery가 함께 나타난 비율 |
| Confidence | $$200/1{,}000=20\%$$ | 초기 LiDAR 품질이 낮을 때 이후 recovery가 발생한 비율 |
| Lift | $$20\%/(500/10{,}000)=4$$ | 전체의 60초 이후 recovery 기본 비율 5%와 비교한 값 |

즉 초기 LiDAR 품질이 낮은 임무에서는 recovery가 전체 평균의 네 배 빈도로 발생했다.
이 pattern은 다음 행동을 검토할 근거가 된다.

- 출발 전 sensor 오염 또는 가림 상태 확인
- LiDAR 품질 저하 시 camera와 IMU 기반 fallback 강화
- 위험도가 높은 임무에 조기 경고 표시
- 해당 조건의 log를 regression test set에 추가

하지만 lift 4를 "고장 확정"으로 읽으면 안 된다.
조건에 걸린 1,000건 중 800건에는 recovery가 없었다.
반대로 전체 recovery 500건 중 이 규칙이 찾아낸 것은 200건, 즉 40%뿐이다.

이 규칙을 경고기로 사용한다면 confidence 20%는 precision과 같고, recovery의 40%를 찾았다는 값은 recall에 해당한다.

강한 연관성이 있다는 것과 완벽한 predictor라는 것은 전혀 다른 말이다.

실제로 적용하기 전에는 다음을 더 확인해야 한다.

- 비, 먼지, tunnel, 특정 route가 두 현상의 공통 원인은 아닌가?
- 특정 robot model이나 firmware에만 나타나는 pattern은 아닌가?
- 55%라는 threshold를 조금 바꿔도 결과가 유지되는가?
- Pattern을 찾는 데 쓰지 않은 이후 기간의 데이터에서도 재현되는가?
- 첫 60초 feature만 사용하고 그 이후 recovery를 예측하여 미래 정보 누출을 막았는가?

같은 10,000개 임무 log라도 질문에 따라 서로 다른 mining task가 된다.

- **Classification**: 이 임무에서 recovery가 발생할지 예측한다.
- **Regression**: recovery까지 남은 시간이나 최종 trajectory error를 예측한다.
- **Clustering**: sensor 오염, dynamic scene, 오래된 map처럼 failure mode를 자동으로 묶는다.
- **Anomaly detection**: 특정 로봇에서만 갑자기 나타난 calibration drift를 찾는다.
- **Sequential pattern mining**: vibration 증가 후 inlier ratio가 떨어지고 tracking loss가 발생하는 순서를 찾는다.

여기서 SLAM과 data mining의 차이도 보인다.

> 하나의 주행에서 현재 pose와 map을 추정하는 것은 SLAM 문제이다. 수천 번의 주행 log를 모아 반복되는 failure condition을 발견하는 것은 data mining 문제이다.

실제 system에서는 두 영역이 연결된다.
Data mining으로 찾은 failure pattern을 다음 SLAM algorithm의 gating, fallback, test design에 다시 넣을 수 있다.

---

## 실제 Data Mining Workflow

현실의 data mining은 algorithm 이름을 고르는 일보다 앞뒤 단계가 더 길다.
전체 흐름을 한 줄로 쓰면 다음과 같다.

```text
질문과 행동 정의
  -> 분석 단위 결정
  -> 수집과 정제
  -> Feature 구성과 탐색
  -> Pattern 또는 model 발견
  -> 보지 않은 데이터에서 검증
  -> 해석과 행동
  -> Monitoring
```

### 1. 먼저 바꾸고 싶은 결정을 정한다

"데이터에서 재미있는 걸 찾아보자"는 출발점은 너무 넓다.

```text
어떤 map을 먼저 갱신할 것인가?
어떤 임무를 출발 전에 점검할 것인가?
어떤 고객에게 어떤 상품을 추천할 것인가?
```

처럼 결과가 바꿀 행동을 먼저 정하면 필요한 data와 metric도 명확해진다.

### 2. 분석 단위를 정한다

한 행이 고객 한 명인지, 주문 한 건인지, LiDAR scan 한 장인지, 임무 한 번인지 정해야 한다.

예를 들어 frame 단위로 분석하면 순간적인 sensor 상태를 자세히 볼 수 있다.
반면 mission 단위로 분석하면 실제 운용 결과와 연결하기 쉽다.
어느 쪽이 맞는지는 질문에 달려 있다.

### 3. 데이터를 모으고 정제한다

현실의 log에는 다음 문제가 흔하다.

- 같은 record가 중복 저장됨
- Timestamp가 맞지 않음
- Sensor packet이 누락됨
- Software version 표기가 섞임
- 사람이 붙인 failure label이 일관되지 않음
- 실패가 너무 심해 log 자체가 저장되지 않음

마지막 경우는 특히 위험하다.
업로드에 성공한 log만 분석하면 가장 심각한 failure가 dataset에서 사라질 수 있다.

### 4. Feature를 만든다

Raw packet을 그대로 algorithm에 넣는 것보다 질문을 잘 설명하는 feature가 유용할 때가 많다.

예를 들어 LiDAR packet 자체 대신 다음 값을 만들 수 있다.

```text
valid return ratio
registration inlier ratio
mean residual
IMU vibration RMS
dynamic object ratio
map age
```

Feature engineering은 단순한 전처리가 아니다.
분석자가 문제에 대해 가진 domain knowledge를 data representation에 넣는 과정이다.

### 5. 후보 Pattern을 찾는다

이 단계에서 association rule, decision tree, clustering, anomaly detection, neural network 같은 도구를 사용한다.

중요한 것은 복잡한 model이 항상 더 좋은 mining 결과를 주는 것은 아니라는 점이다.
운영자가 이해하고 행동해야 하는 문제라면 간단한 rule이나 작은 decision tree가 거대한 black-box model보다 유용할 수 있다.

### 6. 보지 않은 데이터에서 검증한다

같은 데이터에서 pattern을 찾고 그 pattern을 다시 평가하면 우연한 관계를 진짜처럼 착각하기 쉽다.

가능하면 다음처럼 분리해야 한다.

```text
과거 기간: 후보 pattern 발견
이후 기간: pattern 검증
새로운 route 또는 robot: generalization 확인
```

연속된 robot frame을 무작위로 섞어 train/test split하면 바로 옆 frame이 양쪽에 들어갈 수 있다.
두 frame은 사실상 같은 장면이므로 성능이 과장된다.
날짜, route, robot 단위의 split이 더 정직한 경우가 많다.

### 7. Domain knowledge로 해석하고 행동으로 연결한다

숫자가 높다는 이유만으로 바로 정책을 바꾸면 안 된다.
Sensor 구조, 환경, software pipeline을 아는 사람이 pattern이 현실적으로 말이 되는지 살펴봐야 한다.

그 다음에야 sensor cleaning warning, map refresh, 추천 policy, test case 같은 행동으로 연결할 수 있다.

### 8. 배포 후에도 Monitoring한다

Firmware, sensor, 사용자, 날씨, 운행 지역이 바뀌면 과거 pattern이 더는 성립하지 않을 수 있다.
입력 데이터의 분포가 달라지는 현상을 **data drift**라고 한다.
예를 들어 우천 운행 비중이 늘거나 다른 종류의 sensor를 쓰기 시작하면 feature의 분포가 바뀐다.

같은 입력과 결과 사이의 관계가 달라지는 현상은 **concept drift**라고 한다.
예를 들어 firmware 수정 후에는 똑같이 낮은 LiDAR return 비율도 예전만큼 localization loss로 이어지지 않을 수 있다.
배포 뒤에는 두 변화를 구분해 계속 확인해야 한다.

발견은 끝이 아니라 다음 monitoring cycle의 시작이다.

---

## 좋은 Pattern의 조건

Algorithm이 출력한 모든 pattern이 지식은 아니다.
적어도 다음 질문을 통과해야 한다.

### 충분히 반복되는가?

두 번 관측해서 두 번 모두 실패한 rule은 confidence가 100%이다.
하지만 sample이 두 개뿐이라면 믿기 어렵다.
Confidence와 함께 support와 실제 sample count를 봐야 한다.

### 새로운 데이터에서도 유지되는가?

Pattern을 찾은 dataset이 아니라 이후 기간, 다른 route, 다른 robot에서도 재현되어야 한다.
한 번만 나타난 우연한 관계라면 지식으로 쓰기 어렵다.

### 이미 알고 있던 사실을 넘어서는가?

"배터리가 0%이면 로봇이 멈춘다"는 rule은 맞지만 새롭지는 않다.
Data mining은 단순히 정확한 pattern이 아니라, 이전 판단을 개선할 정보를 찾는 데 의미가 있다.

### 실제 행동을 바꿀 수 있는가?

통계적으로 유의하더라도 아무 행동으로 연결할 수 없다면 운영 가치가 낮을 수 있다.
반대로 완벽하지 않은 rule이라도 값싼 사전 점검을 유도해 큰 failure를 막는다면 유용하다.

### 근거와 적용 범위를 설명할 수 있는가?

누가 어떤 조건에서 무엇이 달라지는지, 어떤 데이터에서 검증했는지 설명할 수 있어야 한다.
복잡한 model의 내부를 모두 말로 풀기 어렵더라도 판단의 근거, 불확실성, 적용할 수 없는 범위는 남겨야 한다.
특히 안전과 비용이 걸린 system에서는 이 기록이 중요하다.

한 문장으로 줄이면 다음과 같다.

> 좋은 pattern은 자주 보이는 pattern이 아니라, **안정적으로 재현되고 실제 판단을 개선하는 pattern**이다.

---

## 가장 흔한 함정

### 1. Correlation을 Causation으로 읽기

비가 오는 날 LiDAR return이 줄고 localization failure가 늘었다고 하자.
그렇다고 비가 직접 원인이라고 바로 결론 내릴 수는 없다.

우천 시 특정 route만 운행했거나, lens 오염, 조도, 교통량 같은 제3의 변수가 함께 바뀌었을 수 있다.
원인을 주장하려면 controlled experiment, intervention, causal analysis가 추가로 필요하다.

### 2. Data Leakage

예측 시점에는 알 수 없는 미래 정보가 feature에 들어가는 문제이다.

Localization failure를 미리 예측하면서 failure가 발생한 뒤 계산되는 `relocalization_result`를 feature로 쓰면 거의 완벽한 성능이 나올 것이다.
하지만 실제 운용에서는 사용할 수 없다.

다음 두 시점을 명확히 구분해야 한다.

```text
Feature를 관측할 수 있는 시점
예측하려는 event가 발생하는 시점
```

### 3. 수많은 Pattern 중 우연히 좋은 것 고르기

Feature와 threshold를 수천 가지 조합하면 우연히 lift가 높은 rule 하나쯤은 나온다.
이것을 같은 데이터에서 가장 좋아 보인다는 이유로 채택하면 과적합된다.

Minimum support를 두고, pattern 발견용 data와 검증용 data를 나누고, 가능하면 새로운 환경에서 재현해야 한다.

### 4. Accuracy만 보기

10,000개 임무 중 recovery가 500건이라면, 아무것도 하지 않고 항상 "정상"이라고 답해도 accuracy는 95%이다.

이런 imbalance 문제에서는 다음 값을 함께 봐야 한다.

- **Precision**: 위험하다고 경고한 것 중 실제 위험은 얼마나 되는가?
- **Recall**: 실제 위험 중 얼마나 찾아냈는가?
- **False positive cost**: 불필요한 경고가 운영을 얼마나 방해하는가?
- **False negative cost**: 놓친 failure의 비용은 얼마나 큰가?

좋은 metric은 model이 아니라 실제 decision cost에 맞춰 정해야 한다.

### 5. Sampling Bias

수집된 data가 현실 전체를 대표하지 않을 수 있다.

- 좋은 날씨에만 주행한 log
- upload에 성공한 임무만 남은 log
- 특정 robot model에 편중된 log
- 적극적인 사용자만 남긴 feedback

Dataset 밖에 어떤 사례가 빠져 있는지 묻지 않으면 매우 정확하지만 현실에는 쓸 수 없는 pattern을 얻는다.

### 6. Distribution Shift

과거에 맞았던 pattern이 미래에도 맞는다는 보장은 없다.
새 firmware, 다른 sensor, 새로운 도시, 계절 변화가 들어오면 data distribution이 달라진다.

그래서 production data mining은 한 번의 report가 아니라 지속적인 monitoring과 재검증을 포함한다.

### 7. Privacy와 Bias

사람의 위치, 행동, 구매 기록을 다루면 개인정보와 공정성 문제가 생긴다.
수집할 수 있다는 것과 수집해도 된다는 것은 다르다.

목적에 필요한 정보만 최소한으로 모으고, 직접 식별자는 가명 처리하며, 접근 권한과 보관 기간을 제한해야 한다.
위치나 행동 기록은 이름을 지워도 다른 정보와 결합하면 개인을 다시 알아볼 수 있으므로 단순한 식별자 제거만으로는 충분하지 않다.
또한 발견한 pattern이 특정 집단에 불리한 기존 편향을 반복하거나 강화하지 않는지 살펴야 한다.

---

## 언제 Data Mining이 필요하지 않은가?

모든 데이터 문제를 mining으로 풀 필요는 없다.

- 이미 질문과 계산식이 정확히 정해져 있다면 SQL query와 aggregation이면 충분할 수 있다.
- 원인을 확인해야 한다면 관측 pattern보다 experiment와 causal inference가 더 중요하다.
- 물리 model과 sensor equation이 명확한 작은 문제라면 estimation이나 optimization이 더 직접적일 수 있다.
- Sample이 너무 적으면 복잡한 algorithm보다 domain expert의 case analysis가 더 정직할 수 있다.

Data mining은 답을 자동으로 만들어주는 만능 도구가 아니다.
**무엇을 결정하거나 설명하고 싶은지는 분명하지만, 어떤 관계나 예외가 그 판단에 유용한지는 아직 모를 때** 특히 유용하다.

---

## 실전 Checklist

새로운 data mining project를 시작한다면 다음을 확인해보자.

- 결과로 어떤 판단이나 행동을 바꾸고 싶은가?
- 한 sample의 단위와 시간 범위가 명확하며, feature는 실제 판단 시점에 관측 가능한가?
- 단순한 baseline과 비교하고, 목적에 맞는 metric을 골랐는가?
- 발견용 data와 검증용 data를 분리했으며, 다른 시기·route·robot에서도 재현되는가?
- Correlation을 원인으로 과장하거나 누락된 sample과 sampling bias를 놓치지 않았는가?
- 배포 후 data drift와 concept drift를 계속 감시할 수 있는가?

---

## 정리

Data mining은 "데이터가 많으니 model을 돌려보는 것"이 아니다.

핵심은 다음과 같다.

- 이미 알고 있는 값을 조회하는 것이 아니라, 미처 몰랐던 관계, 구조, 규칙 또는 예외를 발견한다.
- Association rule, classification, regression, clustering, anomaly detection 등 여러 도구를 사용할 수 있다.
- 좋은 pattern은 support나 accuracy 하나가 높다고 끝나지 않는다.
- 보지 않은 데이터에서 검증하고, domain knowledge로 해석하고, 실제 행동으로 연결해야 한다.
- Correlation, leakage, sampling bias, distribution shift를 항상 경계해야 한다.

결국 data mining의 좋은 결과는 화려한 graph나 복잡한 model이 아니다.

> **이전에는 몰랐지만, 이제는 반복해서 검증되었고, 다음 행동을 더 낫게 바꿀 수 있는 지식.**

그 지식에 도달하기까지의 탐색과 검증 과정이 data mining이다.
