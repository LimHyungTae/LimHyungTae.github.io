---
layout: post
title: Terminator 단축키 변경하는 법
subtitle: Terminator 단축키 변경하는 법
tags: [Terminator]
comments: true
---

최근 키보드 세팅을 효율적으로 하는데에 빠져서, Vim, Clion, Terminator의 단축키들을 좀 더 일관성 있게 두게끔 해보았다. Terminator 상의 shortcut을 만들기 위해 bindkey나 이것저것들을 시도해보았는데, 아주 쉽게 Terminator로 띄운 terminal 창 마우스 오른쪽 클릭 → `Preferences` → `Keybindings`에 보면 아주 쉽게 고칠 수 있는 것을 확인했다.

현재 나의 세팅은 아래와 같다

* Jetbrain의 InterlliJ 상에서 각 탭 별 움직이는 키를 `Shift + Ctrl` + `h/j/k/l`로 세팅해두었다. 따라서 나눠진 창에서도 똑같이 움직일 수 있도록 세팅했다.
    * `go_down`: `Shift + Ctrl + J`
    * `go_left`: `Shift + Ctrl + H`
    * `go_right`: `Shift + Ctrl + L`
    * `go_up`: `Shift + Ctrl + K`

* Vim 상에서 `PgUp/PgDn`하는 키에 맞춰서 terminal 상에서 `PgUp/PgDn`을 쉽게 할 수 있게 수정하였다 (원키는 `Shift + PgUp/PgDn`인데, 손을 `PgUp/PgDn`로 보내기 싫었다 ~~진성 Vim 변태가 되어가는 중...~~)
    * `page_up`: `Ctrl + U`
    * `page_down`: `Ctrl + D`

* 그 외 (빈도수가 위에 비해는 높지 않지만 자주 쓰는 기능들. 주로 default로 켜둬야하는 것들, i.e. IDE나 roscore 등등,을 terminal의 다른 탭에 두어서 공간을 절약한다)
    * `next_tab`: `Ctrl + Alt + L`
    * `prev_tab`: `Ctrl + Alt + H`
