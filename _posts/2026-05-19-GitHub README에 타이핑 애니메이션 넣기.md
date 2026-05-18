---
layout: post
title: GitHub README에 타이핑 애니메이션 넣기
subtitle: readme-typing-svg와 vhs로 hello world를 타닥타닥 치게 만들기
tags: [GitHub, README, SVG, Productivity, Open Source]
comments: true
description: GitHub README는 JavaScript를 못 돌리는데도 어떻게 "hello world"가 타닥타닥 쳐지는 타이핑 애니메이션을 넣을 수 있는지, readme-typing-svg와 vhs를 중심으로 정리하고 실제 사용 예시까지 보인다.
permalink: /2026/05/19/github-readme-typing-animation/
redirect_from:
  - '/2026-05-19-GitHub README에 타이핑 애니메이션 넣기/'
  - '/2026-05-19-GitHub-README에-타이핑-애니메이션-넣기/'
  - '/github-readme-typing-animation/'
---

## 왜 README에 타이핑 애니메이션이 필요할까

오픈소스 레포를 만들다 보면, README 맨 위에서 "이 라이브러리는 이렇게 설치해서 이렇게 쓰면 됩니다"를 한 번에 보여주고 싶은 순간이 온다.
보통은 그냥 코드 블록으로 `pip install something` 한 줄을 박는다.
그런데 가끔은 그 한 줄이 정적인 텍스트가 아니라, 터미널에서 누가 직접 타이핑하는 것처럼 보이게 하고 싶을 때가 있다.

이게 의외로 강력하다.
방문자가 README 상단을 열었을 때 시선이 먼저 닿는 곳은 움직이는 영역이기 때문이다.
"이 레포는 pip로 깔린다", "이 레포는 한 줄 명령으로 실행된다" 같은 메시지를 가장 빠르게 박아 넣는 트릭인 셈이다.

문제는 GitHub README에서는 JavaScript를 못 돌린다는 점이다.
`<script>` 태그는 sanitize 되어서 사라진다.
그래서 흔히 알고 있는 typed.js 같은 JS 라이브러리는 쓸 수가 없다.

방법은 크게 세 가지로 좁혀진다.

1. SVG SMIL 애니메이션 (외부 서비스가 동적으로 생성해 주는 SVG)
2. 실제 터미널을 녹화해 만든 GIF
3. 직접 손으로 짜는 SVG 애니메이션

이 글에서는 1번을 메인으로 다루고, 2번을 보조로 다룬다.
3번은 가성비가 너무 떨어져서 보통 안 한다.

---

## 방법 1. readme-typing-svg (가장 간단)

가장 널리 쓰이는 도구는 [DenverCoder1/readme-typing-svg](https://github.com/DenverCoder1/readme-typing-svg)이다.
이건 URL 파라미터만 바꾸면 SMIL 기반 애니메이션 SVG를 동적으로 만들어서 돌려주는 서비스이다.
그래서 `<img src="...">` 한 줄만 README에 박아 넣으면 끝이다.

### 가장 짧은 예시

```markdown
![Typing SVG](https://readme-typing-svg.demolab.com?lines=hello+pioneer+statement)
```

이거 한 줄만 넣어도 `hello pioneer statement`가 한 글자씩 타이핑되는 SVG가 README에 박힌다.
GitHub은 이걸 그냥 일반 이미지처럼 취급하기 때문에 별도 설정이 필요 없다.

### 가운데 정렬해서 좀 더 예쁘게

`<p align="center">`로 감싸면 GitHub README의 가운데 정렬도 그대로 먹는다.

```markdown
<p align="center">
  <a href="https://github.com/DenverCoder1/readme-typing-svg">
    <img src="https://readme-typing-svg.demolab.com?font=Fira+Code&size=28&duration=2500&pause=800&color=22C55E&center=true&vCenter=true&width=600&lines=hello+pioneer+statement" alt="Typing SVG"/>
  </a>
</p>
```

이게 내가 실제로 쓰는 형태이다.
앵커로 한 번 감싸는 이유는 방문자가 SVG를 클릭했을 때 어떤 서비스를 쓴 건지 보여주기 위함이다.

### URL 파라미터 정리

자주 만지게 되는 파라미터만 따로 정리한다.

| 파라미터 | 의미 | 예시 |
|---|---|---|
| `lines` | 타이핑될 문자열들. `;`로 구분하면 순환된다. | `hello+world;pip+install+foo` |
| `font` | Google Fonts 이름. 공백은 `+`로. | `Fira+Code`, `JetBrains+Mono` |
| `size` | 글자 크기 (px) | `24`, `28` |
| `duration` | 한 줄을 다 타이핑하는 데 걸리는 시간 (ms) | `2500` |
| `pause` | 다 친 뒤 다음 줄로 넘어가기 전 멈추는 시간 (ms) | `800` |
| `color` | Hex 컬러 (`#` 없이) | `22C55E`, `F75C7E` |
| `width` | SVG 폭 (px) | `600` |
| `center` | 가로 가운데 정렬 여부 | `true` |
| `vCenter` | 세로 가운데 정렬 여부 | `true` |
| `repeat` | 반복 여부. `false`로 두면 한 번만 친다. | `true` |

전체 파라미터는 [공식 문서의 Options 표](https://github.com/DenverCoder1/readme-typing-svg#options)에 더 있다.

### 인코딩 주의점

URL이라 띄어쓰기와 특수문자를 그냥 못 쓴다.
한 번씩 헷갈리는 인코딩만 정리해 둔다.

| 원래 문자 | URL 인코딩 |
|---|---|
| 공백 | `+` 또는 `%20` |
| `$` | `%24` |
| `;` | `lines` 안에서는 줄 구분자라 진짜 세미콜론을 넣고 싶으면 `%3B` |
| `&` | `%26` (안 그러면 다음 파라미터로 잘려 버린다) |
| 한글 | percent-encoded UTF-8. 보통 그냥 브라우저에 한글로 쳐서 나오는 인코딩을 복붙한다. |

예를 들어 `$ pip install foo  ✔`처럼 터미널 프롬프트와 체크 마크를 같이 넣고 싶으면 이렇게 된다.

```
lines=%24+pip+install+foo++%E2%9C%94
```

이게 좀 귀찮은데, 한 번 만들어두면 그 다음부터는 거의 안 만진다.

### 옵션 갤러리: "Hyungtae's blog"로 변주해 보기

같은 텍스트(`Hyungtae's blog`)를 두고 URL 파라미터만 바꾸면 인상이 어떻게 달라지는지 7가지 변주를 모아본다.
복붙해서 자기 레포에 바로 써먹을 수 있는 형태로 정리했다.

참고로 URL에서 `Hyungtae's blog`는 `Hyungtae%27s+blog`로 인코딩된다 (`'`이 `%27`).

#### (1) 기본형: 투명 배경 + 검정 글자

가장 단순한 형태이다.
배경은 README의 색을 그대로 따라간다.

```markdown
<p align="center">
  <img src="https://readme-typing-svg.demolab.com?font=Fira+Code&size=28&duration=2500&pause=800&color=111827&center=true&vCenter=true&width=600&lines=Hyungtae%27s+blog" alt="Typing SVG"/>
</p>
```

가벼운 정적 느낌이라 깔끔한 라이브러리 README에 잘 어울린다.

#### (2) 어두운 박스 + 네온 그린: 진짜 터미널 느낌

`background=` 파라미터로 SVG 안에 자체 배경 박스를 깐다.
GitHub light mode에서 보면 흰 페이지 위에 어두운 박스가 떠 있어서 시선이 강하게 잡힌다.
"이 레포는 터미널에서 쓰는 도구다"를 강조하고 싶을 때 가장 자주 쓰는 변주이다.

```markdown
<p align="center">
  <img src="https://readme-typing-svg.demolab.com?background=0D1117&color=22C55E&font=Fira+Code&size=28&duration=2500&pause=800&center=true&vCenter=true&multiline=false&width=600&height=80&lines=%24+Hyungtae%27s+blog" alt="Typing SVG"/>
</p>
```

- `background=0D1117`은 GitHub dark mode 배경색이라 어색하지 않다.
- `color=22C55E`는 Tailwind의 `green-500`. 옛 터미널 phosphor green이 그립다면 `00FF41`도 좋다.
- `%24`(`$`)를 앞에 붙여서 프롬프트 흉내를 냈다.

#### (3) Light / Dark mode 자동 전환 (`<picture>` 활용)

이게 사실 가장 멋있는 방법이다.
GitHub markdown은 `<picture>` 태그를 지원하기 때문에, **방문자의 GitHub 테마에 따라 SVG를 다르게 보여줄 수 있다**.
즉, light mode 사용자에게는 어두운 박스, dark mode 사용자에게는 밝은 박스가 뜬다.

```markdown
<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)"
      srcset="https://readme-typing-svg.demolab.com?background=FFFFFF&color=111827&font=Fira+Code&size=28&duration=2500&pause=800&center=true&vCenter=true&width=600&height=80&lines=Hyungtae%27s+blog"/>
    <source media="(prefers-color-scheme: light)"
      srcset="https://readme-typing-svg.demolab.com?background=0D1117&color=22C55E&font=Fira+Code&size=28&duration=2500&pause=800&center=true&vCenter=true&width=600&height=80&lines=Hyungtae%27s+blog"/>
    <img alt="Typing SVG"
      src="https://readme-typing-svg.demolab.com?background=0D1117&color=22C55E&font=Fira+Code&size=28&duration=2500&pause=800&center=true&vCenter=true&width=600&height=80&lines=Hyungtae%27s+blog"/>
  </picture>
</p>
```

구조는 `<source>` 두 개와 fallback `<img>` 하나이다.
브라우저는 위에서부터 매칭되는 `<source>`를 고르고, 둘 다 안 맞으면 `<img>`로 떨어진다.
이 패턴은 타이핑 SVG뿐 아니라 로고, 다이어그램, 그래프 이미지 등 README의 모든 자산에 그대로 쓸 수 있어서 한 번 익혀두면 두루 쓰인다.

#### (4) 여러 줄 순환

`lines=A;B;C` 형태로 적으면 `A → 지움 → B → 지움 → C → 지움 → A ...` 가 무한 반복된다.
블로그 README에서 본인의 정체성을 여러 슬로건으로 돌리고 싶을 때 잘 어울린다.

```markdown
<p align="center">
  <img src="https://readme-typing-svg.demolab.com?font=Fira+Code&size=28&duration=2500&pause=800&color=F75C7E&center=true&vCenter=true&width=600&lines=Hyungtae%27s+blog;SLAM+%26+Robotics+notes;Welcome+%21" alt="Typing SVG"/>
</p>
```

- `%26`은 `&`, `%21`은 `!`이다.
- 줄 사이 구분자는 `;`이라 진짜 세미콜론을 본문에 넣고 싶으면 `%3B`로 인코딩한다.

#### (5) 굵은 JetBrains Mono로 묵직하게

폰트와 weight만 바꿔도 분위기가 확 달라진다.

```markdown
<p align="center">
  <img src="https://readme-typing-svg.demolab.com?font=JetBrains+Mono&weight=700&size=36&color=2563EB&center=true&vCenter=true&width=700&height=80&lines=Hyungtae%27s+blog" alt="Typing SVG"/>
</p>
```

- `weight=700`이 bold이다. `300`(light), `500`(medium), `900`(black)도 가능한 폰트가 많다.
- `font=` 값은 [Google Fonts](https://fonts.google.com/)의 폰트 이름을 그대로 넣으면 된다. 공백은 `+`.

#### (6) 파스텔 톤 (개인 블로그/포트폴리오용)

리서치 레포가 아니라 개인 블로그 README라면 좀 더 부드러운 톤도 잘 어울린다.

```markdown
<p align="center">
  <img src="https://readme-typing-svg.demolab.com?background=FCE7F3&color=BE185D&font=Quicksand&weight=600&size=30&center=true&vCenter=true&width=600&height=80&lines=Hyungtae%27s+blog" alt="Typing SVG"/>
</p>
```

`Quicksand`처럼 둥근 sans-serif를 쓰면 코드 느낌이 사라지고 글 사이트 같은 인상이 된다.

#### (7) 한 번만 치고 멈추기

기본은 무한 반복이다.
한 번만 타이핑하고 그대로 멈추게 하려면 `repeat=false`를 넣는다.

```markdown
<p align="center">
  <img src="https://readme-typing-svg.demolab.com?font=Fira+Code&size=28&duration=2500&pause=800&color=111827&center=true&vCenter=true&width=600&repeat=false&lines=Hyungtae%27s+blog" alt="Typing SVG"/>
</p>
```

페이지가 무한히 움직이는 게 부담스러운 문서형 README에 잘 어울린다.
방문자가 페이지를 새로 고칠 때마다 한 번씩만 타이핑된다.

#### 한 장으로 비교

| # | 인상 | 핵심 파라미터 |
|---|---|---|
| (1) 기본형 | 깔끔, 무난 | `color=111827` |
| (2) 터미널 박스 | 시선 강탈, "도구"스러움 | `background=0D1117`, `color=22C55E` |
| (3) Light/Dark 자동 | 두 테마 모두 자연스러움 | `<picture>` + `prefers-color-scheme` |
| (4) 여러 줄 순환 | 동적, 메시지 다양 | `lines=A;B;C` |
| (5) Bold + 큰 폰트 | 묵직, 브랜드용 | `font=JetBrains+Mono&weight=700&size=36` |
| (6) 파스텔 톤 | 개인 블로그/포트폴리오 | `background=FCE7F3&color=BE185D` |
| (7) 한 번만 | 문서형 README | `repeat=false` |

내 추천 순서를 굳이 매기자면 (3) → (2) → (4)이다.
(3)은 사용자 경험이 가장 좋고, (2)는 가장 임팩트가 강하며, (4)는 자기 소개에 가장 잘 어울린다.

### 한계

readme-typing-svg는 외부 서비스를 호출한다.
즉, 그 서버가 죽으면 README 상단에 X 이미지가 뜬다.
실제로 과거에 `*.herokuapp.com` 도메인이 가끔 다운된 적이 있었기 때문에, 지금은 메인테이너가 `*.demolab.com` 미러를 권장한다.
보수적인 프로젝트라면 (예: 회사 공식 레포) 외부 서비스 의존을 피하고 싶을 수 있다.
그런 경우는 다음에 나올 GIF 방법이 더 안전하다.

---

## 방법 2. vhs로 진짜 터미널 GIF 만들기

readme-typing-svg는 글자가 흐르는 시각적 효과만 흉내낸다.
"진짜 터미널에서 이 명령을 치면 이런 출력이 나옵니다"까지 보여주고 싶다면 결국 터미널을 녹화해야 한다.
이때 가장 결과물이 깔끔한 도구가 Charm 사의 [vhs](https://github.com/charmbracelet/vhs)이다.

asciinema나 terminalizer도 비슷한 일을 하지만, vhs의 장점은 따로 있다.
녹화가 아니라 **tape 파일이라는 스크립트로 결과를 재현 가능하게 만들 수 있다는 점**이다.
한 번 만들어두면, 명령이 바뀌어도 tape 파일만 고쳐서 다시 GIF를 뽑으면 된다.
git에 tape 파일을 같이 커밋해두면 협업자도 동일한 GIF를 만들 수 있다.

### tape 파일 예시

`demo.tape`:

```
Output demo.gif

Set FontSize 22
Set Width 900
Set Height 300
Set Theme "Dracula"

Type "pip install pioneer-statements"
Sleep 500ms
Enter
Sleep 2s

Type "python -c 'import pioneer_statements; pioneer_statements.hello()'"
Sleep 500ms
Enter
Sleep 2s
```

이 파일을 만들고 다음 명령을 돌리면 끝이다.

```bash
vhs demo.tape
```

`demo.gif`가 생성된다.
이걸 그냥 레포에 같이 커밋하고 README에서 참조한다.

```markdown
<p align="center">
  <img src="docs/demo.gif" alt="demo" width="700"/>
</p>
```

### vhs를 추천하는 이유

| 도구 | 결과물 | 재현성 | 학습 비용 |
|---|---|---|---|
| asciinema + agg | SVG 또는 GIF | 녹화 기반이라 재현 어려움 | 낮음 |
| terminalizer | GIF | YAML config로 어느 정도 가능 | 중간 |
| **vhs** | **GIF / WebM / MP4** | **tape script로 완전 재현 가능** | **낮음** |

리서치 코드처럼 라이브러리 버전이 자주 바뀌는 레포라면 재현성이 큰 이점이다.
명령 출력이 달라져도 tape 파일을 그대로 재실행해서 GIF를 갱신하면 된다.

### 단점

GIF는 SVG보다 무겁다.
폭 700px / 4초 분량이면 200KB ~ 1MB쯤 나온다.
README 상단이 무거워지면 모바일에서 첫 화면 로딩이 느려질 수 있으니, 너무 길게 만들지는 말자.
보통 3 ~ 5초 안에 끝내는 것을 권장한다.

---

## 둘 중 어떤 걸 쓸까

내 기준은 이렇다.

- **"이 레포는 한 줄 명령으로 깔린다"를 강조하고 싶다**: readme-typing-svg. 가볍고 빠르다.
- **"이 명령을 치면 진짜로 이런 출력이 나옵니다"까지 보여주고 싶다**: vhs GIF.
- **둘 다 하고 싶다**: 상단에 readme-typing-svg로 한 줄 강조, 본문 demo 섹션에 vhs GIF.

요약하면, 시선을 잡는 용도는 SVG, 진짜 동작을 증명하는 용도는 GIF이다.

---

## 실제로 적용해 본 예시

내가 다른 dummy 레포 README 상단에 박아 둔 한 줄은 위 (1) 기본형의 변주(`color=22C55E`로 네온 그린만 입힌 정도)이다.
딱 한 줄 추가했을 뿐인데 GitHub에서 페이지를 열 때마다 `hello pioneer statement`가 초록색으로 한 글자씩 타이핑돼서 페이지 인상이 꽤 달라진다.

처음 시도하는 거라면 README 상단에 그냥 자기 레포 이름이나 자기 소개 문구 한 줄을 넣어보는 걸 추천한다.
그래야 나중에 `pip install your-lib` 같은 실제 메시지로 바꿔 끼울 때도 부담이 적다.

---

## 정리

- GitHub README는 JS를 못 돌리기 때문에, 타이핑 애니메이션은 SVG SMIL 또는 GIF 중 하나로 해결한다.
- 가장 쉬운 방법은 [readme-typing-svg](https://github.com/DenverCoder1/readme-typing-svg) 서비스의 URL을 `<img src=...>`로 박는 것이다.
- URL 파라미터로 폰트, 크기, 색, 속도, 여러 줄 순환을 모두 조절할 수 있다.
- 외부 서비스 의존이 부담되거나 진짜 터미널 출력까지 보여주고 싶다면 [vhs](https://github.com/charmbracelet/vhs)로 tape 파일을 만들고 GIF를 굽는 것이 좋다.
- vhs는 tape script로 재현이 가능해서, 리서치 레포처럼 명령이 자주 바뀌는 경우에 유리하다.

오픈소스 레포의 첫인상은 README 상단 3줄에서 결정된다는 말이 있다.
그 3줄을 살리는 가장 싼 트릭 중 하나가 이 타이핑 애니메이션이다.
한 번 세팅해두면 다른 레포에도 그대로 복붙해서 쓸 수 있으니, 한 번쯤 해 둘 만하다.

---

## References

- [DenverCoder1/readme-typing-svg (GitHub)](https://github.com/DenverCoder1/readme-typing-svg)
- [readme-typing-svg demo page](https://readme-typing-svg.demolab.com/demo/)
- [charmbracelet/vhs (GitHub)](https://github.com/charmbracelet/vhs)
- [asciinema](https://asciinema.org/)
- [terminalizer (GitHub)](https://github.com/faressoft/terminalizer)
