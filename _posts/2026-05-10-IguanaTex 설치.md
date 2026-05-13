---
layout: post
title: IguanaTex으로 PowerPoint에 LaTeX 수식 넣기
subtitle: 슬라이드에도 진짜 LaTeX 수식을 박는 법
tags: [LaTex, PowerPoint, Productivity]
comments: true
description: PowerPoint에서 수식을 캡처 이미지로 붙이는 대신, IguanaTex으로 진짜 LaTeX 코드를 컴파일해 vector 품질로 슬라이드에 박는 방법을 정리한다. Windows 설치 절차와 macOS 사용자를 위한 LaTeXiT 대안까지.
permalink: /2026/05/10/iguanatex-installation-and-usage/
redirect_from:
  - '/2026-05-10-IguanaTex 설치/'
  - '/2026-05-10-IguanaTex-설치/'
  - '/IguanaTex 설치/'
  - '/IguanaTex-설치/'
---

## 왜 PowerPoint에서 LaTeX이 필요한가

학회 발표 슬라이드든 그룹 미팅 자료든, 결국 **수식이 들어가야 하는 순간**이 온다.

PowerPoint 내장 수식 편집기(Insert → Equation)도 있지만, 한 번이라도 써본 사람은 안다.
- LaTeX과 문법이 미묘하게 다르다.
- 복잡한 `align` 환경, `\newcommand`, `\usepackage`를 못 받는다.
- 논문에 짜둔 수식을 복붙해도 그대로 들어가지 않는다.

논문에서는 LaTeX으로 깔끔하게 짜놓고, 발표에서는 또 GUI로 같은 수식을 다시 그려야 한다는 게 비효율 그 자체이다.
이 글에서 소개할 **IguanaTex**는 그 비효율을 정확히 끊는다.
PowerPoint 안에서 **실제 LaTeX 코드를 그대로 컴파일해 슬라이드에 박아 넣는** add-in이다.

---

## IguanaTex이 다른 방법보다 나은 점

| 방법 | 한계 |
|---|---|
| PowerPoint 내장 수식 편집기 | 문법 다름, 복잡한 환경/preamble 불가 |
| PDF에서 캡처해서 붙이기 | 픽셀 이미지라 zoom 시 흐릿함 |
| Word equation 복붙 | 호환성 깨짐, 폰트 불일치 |
| **IguanaTex** | **LaTeX 그대로, vector 품질, 재편집 가능** |

가장 큰 강점은 두 가지이다.

1. **Vector 그래픽(EMF)으로 들어간다**: 슬라이드를 키우거나 인쇄해도 픽셀이 깨지지 않는다.
2. **재편집이 가능하다**: 박아둔 수식을 다시 선택해서 `Edit LaTeX display`를 누르면 원본 LaTeX 소스창이 다시 뜬다. PDF 캡처 방식과는 차원이 다르다.

---

## 설치 (Windows 기준)

IguanaTex은 **Windows용 PowerPoint add-in**이다.
macOS는 별도 도구를 써야 하는데, 이 부분은 글 끝에 따로 다룬다.

### 1. LaTeX distribution 설치

가장 흔히 쓰는 [MiKTeX](https://miktex.org/download)을 권장한다. TeX Live도 가능하다.
설치 후 명령창에서 `latex --version`이 잘 떨어지는지만 확인하면 충분하다.

### 2. Ghostscript 설치

IguanaTex이 DVI 결과물을 EMF/PNG로 변환할 때 Ghostscript가 쓰인다.
[Ghostscript 공식 다운로드 페이지](https://www.ghostscript.com/download/gsdnld.html)에서 Windows용 64bit 버전을 받아 설치한다.

### 3. IguanaTex 본체 다운로드

구글에 "IguanaTex"으로 검색해서 공식 페이지에 들어가면 최신 `.ppam` 파일을 받을 수 있다.
적당한 경로에 풀어둔다 (예: `C:\Tools\IguanaTex\`).

### 4. PowerPoint에 추가 기능으로 등록

PowerPoint를 열고 다음 순서대로 진행한다.

1. `파일` → `옵션` → `추가 기능`
2. 하단의 `관리` 드롭다운에서 **PowerPoint 추가 기능**을 선택한 뒤 `이동` 클릭.
3. `새로 만들기`를 누르고 방금 받아둔 `.ppam` 파일을 지정.
4. 체크박스가 활성화된 상태로 두면 끝이다.

세부 절차가 헷갈리면 [Microsoft 공식 가이드](https://support.microsoft.com/ko-kr/office/추가-기능-실행하기-3de8bbc2-2481-457a-8841-7334cd5b455f)를 참고하면 된다.

이제 PowerPoint 리본에 **IguanaTex 탭**이 새로 보일 것이다.

---

## 기본 사용법

### Display 수식 넣기

1. 슬라이드에서 수식을 넣고 싶은 위치에 커서를 둔다.
2. IguanaTex 탭 → **New LaTeX display** 클릭.
3. 뜨는 입력창에 LaTeX 본문만 적는다 (math mode는 자동으로 감싸진다).

   ```latex
   \mathbf{R} = \exp(\boldsymbol{\phi}^\wedge), \quad \boldsymbol{\phi} \in \mathfrak{so}(3)
   ```

4. `Generate`를 누르면 슬라이드에 vector 이미지로 박힌다.

### 박힌 수식 재편집

이미 박혀 있는 IguanaTex 객체를 클릭해서 선택한 뒤 IguanaTex 탭 → **Edit LaTeX display**를 누르면 원본 LaTeX 소스창이 다시 뜬다.
이 재편집 기능 덕분에 발표 직전에 수식 하나 고치는 일이 진짜로 빨라진다.

### Preamble 등록

논문에서 쓰던 `\newcommand`, `\usepackage`를 슬라이드 수식에서도 그대로 쓰고 싶다면 IguanaTex의 `Settings` → `LaTeX Preamble`에 본인 preamble을 등록해두면 된다.
한 번 등록해두면 그 PowerPoint 파일 안의 모든 IguanaTex 수식이 같은 preamble을 공유한다.
(`\newcommand`를 잘 활용하는 방법은 [LaTex을 통한 심화 논문 작성법 - 4. newcommand 활용하기](https://limhyungtae.github.io/2024/11/23/latex-paper-writing-04-newcommand/)에 따로 정리해뒀다.)

---

## macOS 사용자에게 한 마디

안타깝게도 IguanaTex은 **Windows PowerPoint 전용**이다.
PowerPoint for Mac은 `.ppam` add-in 호환이 불완전하다.

Mac에서 비슷한 워크플로를 원한다면 다음과 같은 대안이 있다.

- **LaTeXiT (macOS 전용)**: LaTeX 코드를 입력하면 EMF/PDF로 렌더해주고, drag-and-drop으로 PowerPoint나 Keynote에 그대로 떨굴 수 있다. 사실상 IguanaTex의 Mac 카운터파트이다. 재편집은 IguanaTex만큼 매끄럽지는 않지만, vector 품질로 들어간다는 본질은 동일하다.
- **Keynote 전환 고려**: 발표용이라면 Keynote의 자체 수식 입력(`Insert → Equation`)이 PowerPoint 내장 편집기보다 LaTeX 친화적이다. 슬라이드 자체를 Keynote에서 짠다면 굳이 IguanaTex이 없어도 견딜 만하다.
- **Parallels / Windows VM**: 굳이 IguanaTex을 써야겠다면 VM에서 작업하고 결과 PPTX를 macOS로 옮기는 방법이 있긴 하다. 다만 번거로워서 추천은 안 한다.

---

## 정리

- PowerPoint 발표에서 수식이 캡처 이미지 → 픽셀 깨짐의 악순환을 끝내고 싶다면 IguanaTex을 써라.
- 필요한 건 세 가지이다: **LaTeX distribution (MiKTeX)** + **Ghostscript** + **IguanaTex `.ppam`**.
- 설치 후엔 리본의 IguanaTex 탭에서 `New LaTeX display`로 수식을 넣고 `Edit LaTeX display`로 재편집한다.
- macOS는 IguanaTex을 못 쓰니 **LaTeXiT** 또는 Keynote 자체 수식 입력으로 대체하자.

한 번 세팅해두면 박사 과정 내내 쓰는 도구이다.
논문 LaTeX 소스를 그대로 슬라이드로 가져갈 수 있게 되면, 그룹 미팅과 학회 발표 준비 시간이 체감으로 줄어든다.

---

## References

- [Microsoft - 추가 기능 실행하기](https://support.microsoft.com/ko-kr/office/추가-기능-실행하기-3de8bbc2-2481-457a-8841-7334cd5b455f)
- [How to use IguanaTex for PPT (PDF)](https://www.quantsummaries.com/How%20to%20use%20IguanaTex%20for%20PPT_YZ%20ver%201A.pdf)
