---
layout: post
title: latexdiff와 latexpand를 활용한 검토
subtitle: 교수님께 사랑받는 대학원생이 되자
tags: [latex, latexdiff, latexpand]
comments: true
---

# Latexdiff과 latexpand를 활용한 효과적인 논문 검토

(작성 중)

command 기반으로 latex의 history를 관리할 수 있는 명령어

latexdiff:

latexpand:

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


\providecommand{\DIFaddtex}[1]{{\protect\color{red}{#1}}} %DIF PREAMBLE
\providecommand{\DIFdeltex}[1]{{\protect\color{red}\sout{}}}                 

