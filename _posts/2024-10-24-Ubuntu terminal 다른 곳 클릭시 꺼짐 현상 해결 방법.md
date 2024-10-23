---
layout: post
title: Ubuntu terminal 다른 곳 클릭시 꺼짐 현상 해결 방법
subtitle:
tags: [Ubuntu, GitHub]
comments: true
---

## Introduction

LunarVim 세팅 와중에 갑자기 터미널이 켜졌다가 몇 초 뒤면 꺼지는 현상이 생겼다.
처음 겪어보는 현상이라 '혹시 포맷해야하나'라는 걱정이 잠시 뇌리를 스쳤으나, 다행히 그러지 않아도 됐다.
이를 해결한 과정을 메모해보고자 한다.

## 문제 설명 및 해결책들

nautilus에서 마우스 오른쪽으로 눌러서 'Open in Terminal'로 연 터미널은 안 꺼지는데, `Ctrl+Alt+t`로 실행시킨 터미널만 바로 꺼지는 게 아닌가!

### Trial 1 (찜찜한 해결!)

최종적으로는 `Ctrl+Alt+t`을 통해 연 터미널에서 `Preferences`에서 `Always on top`을 눌렀다가 해제했더니 해결되었다...
왜인지는 모르겠는데 `Hide on lose focus`가 내재적으로 on으로 설정돼있어서 다른 곳을 클릭하면 꺼진 것이 아니었을까 싶다.

### Trial 2

ChatGPT가 아래의 명령어로 백업해보고 한 번 실행해라고 해보아서 해보았다:

```
mv ~/.bashrc ~/.bashrc_backup
mv ~/.zshrc ~/.zshrc_backup
```

rc 파일을 백업하고 돌려보니, 터미널이 종료되진 않았다. 그렇다고 문제가 해결된 것 아니었다.  

### Trial 3

`sudo update-alternatives --config x-terminal-emulator` 명령어를 알게 되었는데, 이 명령어는 `Ctrl+Alt+t`를 했을 때 켜지는 terminal의 emulator를 고를 수 있게 해준다.
종류는 아래와 같이 있더라:

```angular2html
Selection    Path                             Priority   Status
------------------------------------------------------------
  0            /usr/bin/terminator               50        auto mode
* 1            /usr/bin/gnome-terminal.wrapper   40        manual mode
  2            /usr/bin/koi8rxterm               20        manual mode
  3            /usr/bin/lxterm                   30        manual mode
  4            /usr/bin/terminator               50        manual mode
  5            /usr/bin/uxterm                   20        manual mode
  6            /usr/bin/xterm                    20        manual mode
```

그런데 `terminator`로 실행시켰을 때만 종료되는 현상이 있는 것 아닌가! 그래서 Trial 1에서 설명한 것처럼 Terminator의 `Preferences`를 고쳐야 하는 것이 아닌가 하는 생각을 하게 되었다. 그리고 다행히 그게 맞았다.

## 결론

무언가를 수정할 때는 **꼭** 자기가 지금 뭘 고치고 있는건지 판단하고 고치길...

