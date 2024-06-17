---
layout: post
title: Overleaf, TeXstudio 쓰지 마세요! Pycharm의 TeXiFy를 활용한 효율적인 LaTex 작성
subtitle: Leverage IntelliJ for integration IDE
tags: [Pycharm, IDE, Latex, Overleaf, TeXiFy]
comments: true
---

갑자기 SLAM을 다루는 블로그에서 웬 `TeXiFy`라는 듣보잡(?) 툴을 다루는 게 이상해 보이나, 
사실 내가 늘 `TeXiFy`에서 BibTex 세팅을 하는 법을 자꾸 까먹어서, 그걸 메모해 둘 겸 `TeXiFy`를 소개할 겸 작성해 본다.
필자는 논문을 작성할 때 요즘에는 로컬에서 작업하는 것을 선호하다 보니, 이것저것 써보다가 TeXiFy로 정착했다.
근데 ㄹㅇ 우리나라에서 나밖에 안 쓰는 듯...ㅠ 추천할 때마다 다른 연구자들이 아주 흥미가 없다는 눈빛을 주지만, 언젠가 빛을 발할 것에 대비해서(?) 미리 포스트 해본다.


## Why Not Overleaf? 너무 느린 컴파일 속도!

사실, 필자도 멋 모를 때는 Overleaf을 통해서 작업을 했었는데, 그때는 느리다는 것을 몰랐다.
그런데 22년도 겨울에 Cyrill Stachniss 교수님네에서 논문을 쓰게 되면서 강제로 로컬에서 작업을 하게 되었는데 (사실 Cyrill 교수님은 보안 상의 이유로 Overleaf을 쓰지 말라고 하셨음),
그 때 로컬로 작업을 하면서 큰 깨달음을 깨닫고, 필자는 엄청난 Overleaf 혐오자가 되었다. 그 이유는 다음과 같다.

- **컴파일 속도가 너무 느리다**. Cyrill 교수님네에서 [13 페이지 RSS'23 논문](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/lim2023rss.pdf)을 쓸 때, 논문 작성을 딱 12월 말에 시작해서 거의 1달만에 논문 작성을 마쳤었음. 원래 논문 작성 속도 대비 진척률이 굉장히 빨라졌다는 것을 깨닫고 이유를 생각해보니, 로컬에서 작업해서 Latex 파일 compile하는 속도를 아꼈기에 가능해졌다는 것을 깨달음.
- 이 경험을 기반으로 필자가 [작년과 올해 두 컬럼 26페이지 짜리 저널](https://journals.sagepub.com/doi/full/10.1177/02783649231207654)을 리비전할 때도 로컬에서 작업을 했는데,  한 번 build 할 때 수십 초가 드는 Overleaf에서 벗어나니, 너어어어어ㅓㅓㅓ무 편했음...ㄹㅇ 신세계. 
  - Long journal을 필연적으로 써야하는 한국 연구 문화에서 Overleaf은 한국을 품기에는(?) 속도가 너무 느림을 다시금 느낌...  
- 가슴에 손을 얹고, **공저자들과 반드시 동시에 논문을 작성해야 하는가?**에 질문을 했을 때 No임. Overleaf의 가장 큰 장점은 real-time으로 공동으로 논문을 작성할 수 있다는 것인데, 과연 컴파일의 느린 속도 <<< real-time 수정가능인가?라고 스스로 질문했을 때, 차라리 v1을 빠르게 다 완성한 후 offline으로 업무 분담을 잘 하고 Git pull/push로 주거니 받거니 하는 게 더 효율적임
- 교수님이 코멘트 주신 부분을 반영해서 어디가 수정되었는지 효과적으로 보여드리기 위해 [Latexdiff](https://limhyungtae.github.io/2022-10-19-%EB%8C%80%ED%95%99%EC%9B%90%EC%83%9D%EC%9D%84-%EC%9C%84%ED%95%9C-latexdiff%EC%99%80-latexpand%EB%A5%BC-%ED%99%9C%EC%9A%A9%ED%95%9C-%EB%85%BC%EB%AC%B8-%EA%B2%80%ED%86%A0/_)를 쓰는데, 로컬 커맨드 창에서 즉각적으로 파일을 입력으로 받고 싶었는데, 그 부분을 해소할 수 있었음.  

## Why Not TeXstudio? 한글 입력의 에러로 인한 의도치 않은 Typo 

그래서 필자는 Overleaf을 쓰지 않고 로컬에서 작업을 하게 되었다. 처음에는 가장 유명한 TeXstudio을 많이 썼었는데, TeXstudio는 한글을 치면 이상한 글자로 입력이 되는 불편함이 있었다 (특히, 한글 입력 상태에서 `Ctrl+ S`(저장) 누르면 자꾸 공백에 요상한 문자를 남겨둬서 짜증났음).
그리고 Latex을 잘 쓰게 되면서, 코드들과 마찬가지로 파일을 적절히 분할하거나 다른 파일에 변수 선언을 할 일이 많아졌는데, 그럼에 따라 C++이나 Python의 IDE처럼 파일 간 빠르게 점프할 수 있는 기능이 있었으면 좋겠다고 생각했다. 하지만 TeXstudio는 그 부분이 없었다 ㅜ.

또한, 필자는 heavy VIM 유저인데, Latex을 작성할 때도 Vim 단축키들을 쓰고 싶은 갈증이 있었다. 
파일 쉭쉭 빠르게 마킹하고 변수들 tmp clip보드에 저장해두고 붙여넣기 하면 편할 것 같다고 늘 생각했었는데, TeXstudio에는 어떻게 해도 Vim을 못 쓰더라...


### Solution: IntelliJ의 TeXiFy

그러다가 IntelliJ에서 TeXiFy라는 플러그인을 발견했는데, 이게 정말 신세계였다! 친숙한 IntelliJ의 IDE에서 Latex 작업을 할 수 있다는 점이 굉장히 편했기 때문이다. 
필자는 IntelliJ IDE를(예: Pycharm이나 CLion)을 사용하는데, 이 IDE pro 버젼을 쓰면 `Setting Syncs` 옵션을 통해 어느 컴퓨터에서 IDE를 설치하기만 하면 내가 기존에 쓰던 옵션을 쉽게 동기화할 수 있다는 장점이 있다 (VSCode에도 이러한 동기화 기능이 있는걸로 알고 있다. IntelliJ에만 있는 기능은 아님).

설치 방법도 아래와 같이 Plugin을 설치만 하면 됐다.

![](/img/texify_how_to_install.png)

(혹시 이 글을 보시는 보시고 따라하는 사람이 혹여나 있다면, Grazie Pro와 PDF Viewer도 설치하는 걸 추천드립니다.)

### BibTex 세팅하기

그리고 필자는 BibTex를 이용해 reference에 관리하는데, 가끔 *BibTex이 build가 안 돼서 `.bbl` 파일이 생성되지 않는 불편한 에러*를 겪었다 (이렇게 BibTex가 잘 build되지 않으면 pdf에 무수한 ?가 떠있는 것을 볼 수 있음) 
하지만 이 문제는 아래 세 가지 step을 통하면 쉽게 해결할 수 있다. 주 원인은 BibTex가 자동으로 동작하지 않아 발생한 것이라 볼 수 있다. 따라서 BibTex의 세팅만 아래와 같이 설정해주면 된다.

1. 아래와 같이 main 파일과 bib 파일에 대한 Configuration을 세팅해줘야 한다.
아래의 External LaTeX programs: 에 `Enabled`가 되어 있고, `bibliography BibTex`가 들어있어야 함.

![](/img/texify_working.png)

![](/img/texify_working_bib.png)

아래의, BibTex가 동작이 됐을 때를 비교해서 살펴보면 해당 부분이 `Disabled` 되어있고, 아래의 화살표를 클릭해도 `bibliography BibTex`가 없다.

![](/img/texify_not_working.png)


2. 그리고 원래 Overleaf이나 TeXstudio에서는 `main.bib` 파일을 `./main`과 같이만 써줘도 동작이 했는데, TeXiFy는 아래와 같이 명시적으로 적어줘야 하는 것 같다.


```latex
\bibliographystyle{IEEEtran}
% argument is your BibTeX string definitions and bibliography database(s)
%\bibliography{./main,./IEEEabrv} (x). TeXiFy에서는 이렇게 쓰면 안 됨.
\bibliography{main.bib, IEEEabrv.bib} % (o) 이렇게 써줘야 함.
```

그렇지 않으면 아래와 같은 에러 `I couldn't open database file` 에러가 발생한다.

```commandline
bibtex main
This is BibTeX, Version 0.99d (TeX Live 2019/Debian)
The top-level auxiliary file: main.aux
The style file: IEEEtran.bst
I couldn't open database file ./main.bib
---line 223 of file main.aux
 : \bibdata{./main
 :                ,./IEEEabrv}
I'm skipping whatever remains of this command
I found no database files---while reading file main.aux
```

정상적으로 동작하면 아래와 같이 `Done.`하고 뜸

```commandline
bibtex main
This is BibTeX, Version 0.99d (TeX Live 2019/Debian)
The top-level auxiliary file: main.aux
The style file: IEEEtran.bst
Database file #1: main.bib
-- IEEEtran.bst version 1.11 (2003/04/02) by Michael Shell.
-- http://www.ctan.org/tex-archive/macros/latex/contrib/supported/IEEEtran/
-- See the "IEEEtran_bst_HOWTO.pdf" manual for usage information.
Warning--empty journal in welch1995kalman

Done.
(There was 1 warning)
```

3. (Optional) 2번을 따랐는데도 에러가 발생한다면, `.bib` 파일과 `.bst` 파일에 `sudo chmod -R 777 ${FILE NAME}`을 해본다. 접근 권한이 없어서 파일을 못 여는 걸수도 있음

---
