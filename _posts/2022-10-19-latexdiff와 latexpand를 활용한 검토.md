---
layout: post
title: latexdiff와 latexpand를 활용한 검토
subtitle: 교수님께 사랑받는 대학원생이 되자
tags: [latex, latexdiff, latexpand]
comments: true
---

# Latexdiff과 latexpand를 활용한 효과적인 논문 검토

오늘은 Robotics와는 크게 관련은 없으나, latex을 활용해 논문 작성을 할 때 버전 관리 및 수정 부분을 자동으로 highlight해주는 latexdiff와 latexpand의 사용법에 대해 알아보도록 한다.

결과부터 보여주자면,

command 기반으로 latex의 history를 관리할 수 있는 명령어

```
# For installation of `latexdiff`
$ sudo apt install latexdiff
# For installation of `latexpand`
$ apt-get install texlive-extra-utils
```

둘 다 설치가 완료돼었다면, 
```
latexdiff --help
latexpand --help
```
라고 하면 뭐가 떠야 됨!

덧: 간혹 논문 workspace는 window로, 실험 workspace는 Ubuntu로 이분화해서 사용하는 이들이 있는데, 이는 비효율적이다.

아래의 명령어를 통하면 바로 latex을 Ubuntu에서 수정/작성할 수 있게끔 latex 컴파일러를 설치하고 Tex Studio를 설치할 수 있다:

```
$ sudo apt-get install texlive-latex-base texlive-fonts-recommended texlive-fonts-extra texlive-latex-extra texlive-font-utils texlive-science
$ sudo apt install texstudio
```


## How to install

### Latexdiff

Overleaf 공식 홈페이지에서 [latexdiff 설치 방법](https://www.overleaf.com/learn/latex/Articles/Using_Latexdiff_For_Marking_Changes_To_Tex_Documents)을 알려줌

https://github.com/ftilmann/latexdiff/에서 다운받아서 Documents\latexdiff에 넣음


```bash
latexpand main.tex > v2.tex
```

```bash
latexdiff v1_1.tex v2.tex > diff.tex
```
---
아래 줄을 
```
\providecommand{\DIFaddtex}[1]{{\protect\color{blue}\uwave{#1}}} %DIF PREAMBLE
\providecommand{\DIFdeltex}[1]{{\protect\color{red}\sout{#1}}}                  
```

이렇게 고치면

```
\providecommand{\DIFaddtex}[1]{{\protect\color{red}{#1}}} %DIF PREAMBLE
\providecommand{\DIFdeltex}[1]{{\protect\color{red}\sout{}}}                 
```

물결이 없고 빨강이 없게 출력이 됨!

