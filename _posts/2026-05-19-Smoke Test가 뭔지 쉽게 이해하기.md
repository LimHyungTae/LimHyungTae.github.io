---
layout: post
title: Smoke Test가 뭔지 쉽게 이해하기
subtitle: 제대로 맞는지는 몰라도, 일단 켜지는지는 확인하자
tags: [Software Engineering, Testing, Debugging, Research Code]
comments: true
description: Smoke test의 뜻과 목적을 초보자도 이해할 수 있게 설명한다. Unit test, integration test, benchmark와의 차이를 짚고, 연구 코드와 SLAM 코드에서 어떤 smoke test를 만들면 좋은지 예시로 정리한다.
permalink: /2026/05/19/what-is-smoke-test/
redirect_from:
  - '/2026-05-19-Smoke Test가 뭔지 쉽게 이해하기/'
  - '/2026-05-19-Smoke-Test가-뭔지-쉽게-이해하기/'
---

## 들어가며

코드를 짜다 보면 "test를 돌려봤냐"는 말을 자주 듣는다.
그런데 test에도 종류가 많다.
unit test, integration test, regression test, end-to-end test, benchmark, ablation 등등 이름만 들어도 피곤하다.

그중에서 내가 연구 코드나 오픈소스 코드를 만질 때 가장 먼저 챙기려고 하는 것이 **smoke test**이다.
이름이 조금 이상한데, 뜻은 단순하다.

> 일단 켰을 때 연기가 나는지 안 나는지 보는 테스트.

이 글을 쓰는 이유도 사실 꽤 단순하다.
요즘 Claude나 Codex 같은 coding agent에게 코드를 고쳐달라고 하면,
"smoke test를 돌려보겠다", "간단한 smoke test로 확인했다" 같은 말을 꽤 자주 한다.
그런데 막상 smoke test가 정확히 무엇을 의미하는지 모르면,
agent가 지금 깊은 검증을 했다는 건지, 그냥 최소 실행만 확인했다는 건지 헷갈릴 수 있다.
그래서 이 글에서는 coding agent들이 말하는 smoke test가 대체 어느 정도의 확인을 뜻하는지 정리해보려고 한다.

즉 smoke test는 "정확히 잘 동작하는가"를 깊게 검증하는 테스트가 아니다.
그 전에, **프로그램이 최소한 실행 가능한 상태인지**를 빠르게 확인하는 테스트이다.

---

## Smoke Test의 뜻

Smoke test라는 말은 하드웨어 쪽에서 온 표현으로 이해하면 쉽다.
전자기기나 회로를 처음 켰을 때 연기가 나면 망한 것이다.
그 단계에서는 성능이 좋은지, 정확도가 높은지, 효율이 좋은지 따질 필요도 없다.
일단 전원을 넣자마자 타버리면 그 다음 검증은 의미가 없다.

소프트웨어에서도 똑같다.
어떤 코드를 수정했는데,

- import가 안 된다.
- 빌드가 안 된다.
- 실행하자마자 crash가 난다.
- config file을 못 읽는다.
- 첫 번째 input에서 바로 NaN이 난다.
- output directory를 만들지 못한다.

이런 상태라면 더 깊은 test나 benchmark는 의미가 없다.
smoke test는 이런 "초기 폭발"을 빠르게 잡는 테스트이다.

한 문장으로 정의하면 다음과 같다.

> Smoke test는 시스템의 핵심 실행 경로가 최소한 죽지 않고 끝까지 한 번 통과하는지 확인하는 빠른 테스트이다.

여기서 중요한 단어는 **빠른**과 **최소한**이다.

---

## Smoke Test는 정답을 보장하지 않는다

smoke test를 처음 접하면 흔히 오해하는 부분이 있다.
"테스트를 통과했으면 코드가 맞는 것 아닌가?"라고 생각할 수 있는데, smoke test는 그런 테스트가 아니다.

smoke test가 말해주는 것은 딱 이 정도이다.

- 이 코드가 실행은 된다.
- 주요 dependency가 깨지지는 않았다.
- config와 input/output path가 기본적으로 맞는다.
- 가장 중요한 pipeline 하나는 crash 없이 지나간다.

반대로 smoke test가 말해주지 못하는 것도 많다.

- 알고리즘이 정확한가?
- edge case에서 robust한가?
- 성능이 논문 수치와 같은가?
- long sequence에서 drift가 작은가?
- memory leak이 없는가?

즉 smoke test는 정밀 검사가 아니라 체온 측정에 가깝다.
열이 없다고 건강하다는 보장은 없지만, 열이 40도면 일단 다른 검사를 하기 전에 그 문제부터 봐야 한다.

---

## Unit Test와 뭐가 다른가

헷갈리기 쉬운 테스트들을 비교해보자.

| 테스트 | 묻는 질문 | 예시 |
|---|---|---|
| Smoke test | 일단 켜지는가? 핵심 path가 죽지 않는가? | demo data 5 frame을 돌렸을 때 crash 없이 output 생성 |
| Unit test | 작은 함수 하나가 맞는가? | `pose_to_matrix()`가 expected matrix를 만드는지 확인 |
| Integration test | 여러 module이 같이 맞물리는가? | dataset loader + tracker + mapper가 함께 동작하는지 확인 |
| Regression test | 예전에 고친 bug가 다시 생기지 않았는가? | 특정 입력에서 NaN이 나던 bug를 재현해 확인 |
| End-to-end test | 실제 workflow 전체가 돌아가는가? | full sequence를 돌려 trajectory와 mesh를 생성 |
| Benchmark | 얼마나 잘/빠르게 동작하는가? | KITTI sequence ATE, runtime, memory 측정 |

smoke test는 보통 가장 앞단에 있다.
unit test보다도 더 coarse할 수 있다.
정확한 값 비교를 하지 않고, 단순히 "이 command가 exit code 0으로 끝나는가"만 볼 때도 많다.

---

## 연구 코드에서 Smoke Test가 중요한 이유

연구 코드는 일반적인 product code보다 smoke test가 더 중요할 때가 많다.
이유는 간단하다.
연구 코드는 환경과 dependency가 훨씬 잘 깨진다.

- CUDA version이 다르다.
- PyTorch version이 다르다.
- dataset path가 다르다.
- config file이 많다.
- optional dependency가 많다.
- GPU가 없으면 다른 code path를 타야 한다.
- 논문용 script와 실제 release code가 다를 때가 많다.

이런 상황에서 full experiment를 바로 돌리면 시간이 너무 많이 든다.
예를 들어 SLAM 코드에서 KITTI 00 전체를 돌리는 데 30분이 걸린다고 하자.
그런데 28분 뒤에 output 저장 path 문제로 죽으면 굉장히 허무하다.

그래서 smoke test는 가능하면 작아야 한다.

> 30분짜리 실험을 돌리기 전에, 30초짜리 smoke test로 "일단 돌아가는 코드인지" 확인하자.

이 습관 하나만 있어도 debugging 시간이 꽤 줄어든다.

---

## 좋은 Smoke Test의 조건

좋은 smoke test는 다음 조건을 만족한다.

1. **빠르다.**
   보통 몇 초에서 몇 분 안에 끝나야 한다.
   오래 걸리면 사람들이 안 돌린다.

2. **대표 path를 지난다.**
   너무 trivial하면 의미가 없다.
   예를 들어 `print("hello")`만 하는 test는 거의 도움이 안 된다.
   최소한 config loading, model initialization, data loading, main function call 정도는 지나야 한다.

3. **작은 input을 쓴다.**
   full dataset이 아니라 tiny sample, first 5 frames, synthetic input 같은 것을 쓴다.

4. **자동화 가능하다.**
   사람이 GUI를 보고 "대충 되는 것 같네"라고 판단하는 test는 CI에 넣기 어렵다.
   command 하나로 pass/fail이 나와야 한다.

5. **실패가 명확하다.**
   실패했는데도 exit code 0이면 안 된다.
   assert를 걸거나, output file 존재 여부를 확인해야 한다.

6. **외부 요인에 덜 흔들린다.**
   network download, random seed, 외부 server 상태에 의존하면 smoke test가 flaky해진다.

---

## Python Package 예시

가장 간단한 Python package smoke test는 import test이다.

```bash
python -c "import my_package; print('import ok')"
```

이건 정말 최소한이다.
조금 더 나아가면 package의 핵심 객체를 생성해본다.

```python
def test_smoke_import_and_init():
    import my_package

    model = my_package.Model()
    assert model is not None
```

여기까지는 아직 correctness test가 아니다.
단지 package import, class initialization, dependency loading이 깨지지 않았는지 보는 것이다.

ML package라면 작은 tensor 하나를 forward시켜볼 수 있다.

```python
def test_smoke_forward():
    import torch
    from my_package import Model

    model = Model()
    x = torch.zeros(1, 3, 64, 64)
    y = model(x)

    assert y is not None
    assert torch.isfinite(y).all()
```

이 test가 통과했다고 model이 좋은 것은 아니다.
하지만 최소한 forward path가 죽지는 않는다는 뜻이다.

---

## C++ / CMake 예시

C++ project에서는 보통 build 자체가 smoke test가 된다.

```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

여기서 한 단계 더 나아가면 example binary를 하나 실행한다.

```bash
./build/examples/minimal_example --help
./build/examples/minimal_example --input ./data/tiny
```

이 test의 목표는 "모든 알고리즘이 맞다"가 아니다.
다음이 깨지지 않았는지 확인하는 것이다.

- CMake config
- compiler flags
- include path
- linked library
- runtime shared library
- command-line argument parsing
- example data path

오픈소스 C++ 라이브러리에서는 이 정도 smoke test만 있어도 사용자 입장에서 훨씬 안심된다.

---

## SLAM 코드에서의 Smoke Test

SLAM 코드는 smoke test를 만들기 좋은 동시에, 꼭 만들어야 하는 분야라고 생각한다.
pipeline이 길기 때문이다.

예를 들어 LiDAR SLAM 코드는 대략 다음을 모두 통과해야 한다.

```text
dataset loader
  -> point cloud preprocessing
  -> initial pose guess
  -> registration
  -> map update
  -> trajectory logging
  -> optional mesh / map saving
```

full sequence를 돌리기 전에, first 5~10 frames만 돌리는 smoke test를 만들 수 있다.

```bash
python3 pin_slam.py ./config/lidar_slam/run_demo.yaml --range 0 10 1 -s
```

그 다음 output이 생겼는지 확인한다.

```bash
test -d ./experiments
```

조금 더 제대로 하려면 다음을 check할 수 있다.

- process exit code가 0인가?
- trajectory file이 생성되었는가?
- pose 개수가 입력 frame 수와 맞는가?
- pose matrix에 NaN/Inf가 없는가?
- map file이 생성되었는가?
- GPU memory error가 나지 않았는가?

이런 smoke test는 논문 성능을 보장하지 않는다.
하지만 "이 repo를 clone한 사람이 example command를 쳤을 때 최소한 죽지 않는다"는 강한 신호를 준다.

---

## 블로그나 문서에도 Smoke Test가 있다

smoke test는 코드에만 있는 개념이 아니다.
블로그나 문서에도 적용할 수 있다.

예를 들어 Jekyll blog라면 다음이 smoke test가 될 수 있다.

```bash
bundle exec jekyll build
```

이 command가 통과하면 최소한 다음은 확인된다.

- front matter YAML이 깨지지 않았다.
- Markdown이 Jekyll에서 parse된다.
- permalink 충돌이 없다.
- Liquid template error가 없다.
- 전체 site build가 된다.

글 내용이 좋은지는 별개의 문제이다.
하지만 build가 깨지면 배포 자체가 안 되므로, smoke test로 먼저 잡아야 한다.

---

## Smoke Test와 Sanity Test의 차이

smoke test와 sanity test는 현업에서도 꽤 섞여 쓰인다.
엄밀히 구분하지 않아도 큰 문제는 없지만, 나는 이렇게 이해한다.

- **Smoke test**: 새 build나 새 환경이 최소한 켜지는지 보는 넓고 얕은 테스트.
- **Sanity test**: 특정 수정 후, 그 수정이 말이 되는 방향으로 동작하는지 빠르게 보는 테스트.

예를 들어,

- "새 Docker image가 import부터 demo 실행까지 되는가?" → smoke test
- "방금 고친 quaternion convention 때문에 yaw sign이 뒤집히지 않았는가?" → sanity test

물론 둘 다 "빠르게 큰 문제를 잡는다"는 점에서는 비슷하다.
용어에 너무 집착할 필요는 없다.
중요한 것은 긴 실험 전에 짧은 검증을 먼저 한다는 습관이다.

---

## 안 좋은 Smoke Test

다음은 smoke test로 적합하지 않다.

### 너무 오래 걸리는 테스트

```bash
python train.py --dataset full_kitti --epochs 100
```

이건 smoke test가 아니라 training run이다.
사람들이 매번 돌리지 않을 것이고, CI에 넣기도 부담스럽다.

### 너무 사소한 테스트

```bash
python -c "print('hello')"
```

이건 Python이 설치되었는지만 확인한다.
project code path를 전혀 지나지 않으므로 대부분 의미가 없다.

### 성공/실패가 애매한 테스트

프로그램이 warning을 잔뜩 뱉고 output도 안 만들었는데 exit code 0으로 끝난다면 smoke test로 부족하다.
최소한 중요한 output이 생겼는지 assert해야 한다.

### 외부 다운로드에 의존하는 테스트

매번 test 중에 큰 dataset을 다운로드하면 network 상태 때문에 자주 실패한다.
smoke test input은 repo 안의 tiny sample이거나, CI cache에 안정적으로 들어가는 작은 file이 좋다.

---

## 내가 생각하는 연구 코드 Smoke Test Template

연구 코드라면 최소한 아래 정도의 smoke test가 있으면 좋다.

```text
1. package import
2. config load
3. tiny dataset load
4. model/map/tracker 초기화
5. 1~10 frame 실행
6. output file 생성 확인
7. NaN/Inf check
```

이를 command 하나로 만들면 더 좋다.

```bash
python scripts/smoke_test.py --config configs/demo.yaml --input data/tiny
```

또는 Makefile에 넣어도 된다.

```makefile
smoke:
	python scripts/smoke_test.py --config configs/demo.yaml --input data/tiny
```

그러면 새 코드를 받았을 때 가장 먼저 이렇게 칠 수 있다.

```bash
make smoke
```

이 한 줄이 통과하면, 그 다음에 unit test, full sequence, benchmark를 돌리면 된다.

---

## 언제 Smoke Test를 돌려야 하나

내 기준으로는 다음 상황에서 smoke test를 먼저 돌린다.

- 새 repo를 clone했을 때
- dependency를 새로 설치했을 때
- Docker image를 새로 만들었을 때
- 큰 refactor를 한 뒤
- config file을 바꾼 뒤
- dataset loader를 수정한 뒤
- CI에 올리기 전
- 밤새 돌릴 실험을 시작하기 전

특히 마지막이 중요하다.
긴 실험을 돌리기 전에는 반드시 작은 입력으로 smoke test를 먼저 해야 한다.
안 그러면 다음 날 아침에 "첫 frame에서 path error로 죽어 있음" 같은 슬픈 장면을 보게 된다.

---

## 정리

- smoke test는 시스템이 최소한 켜지고 핵심 실행 경로를 통과하는지 보는 빠른 테스트이다.
- 정확도, 성능, robustness를 보장하는 테스트가 아니다.
- unit test보다 coarse하고, benchmark보다 훨씬 빠르다.
- 연구 코드에서는 dependency, dataset path, config, GPU 환경이 자주 깨지기 때문에 특히 중요하다.
- 좋은 smoke test는 빠르고, 작은 input을 쓰고, 자동화 가능하며, pass/fail이 명확해야 한다.
- 긴 실험을 돌리기 전에 `make smoke` 같은 한 줄짜리 검증을 먼저 만들자.

결국 smoke test의 철학은 단순하다.

> 깊게 검증하기 전에, 일단 불이 안 나는지부터 확인하자.

이 작은 습관이 debugging 시간을 꽤 많이 줄여준다.
