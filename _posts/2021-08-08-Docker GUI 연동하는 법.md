---
layout: post
title: Docker GUI 연동하는 법
subtitle: xhost +
tags: [docker]
comments: true
---

# Docker에서 gui 연동하게 하는 법

python으로 짜져있는 딥러닝 알고리즘들을 돌릴 때, 요즘은 dependency가 점점 더 복잡해져서 도커는 거의 필수인 것 같다.

그런데 간혹 demo.py 같이 저자들이 만들어둔 파일을 돌릴 때 gui가 지원이 되지 안 되어 에러가 날 때가 있다.

docker에서도 local computer의 gui를 연동할 수 있게 세팅하는 법에 대해 알아보자.

## 1. OpenGL이 설치돼있는 docker 를 pull한다.

## 2. docker를 실행시키기 전 terminal에서 `xhost +`를 입력한다.

```
$ xhost +
```

## 3. Docker 실행

<script src="https://gist.github.com/LimHyungTae/477e1a5db5900c67e697d176e184439e.js"></script>

## 4. Docker 내에서 아래의 명령어 실행

```
$ apt-get update
$ apt install mesa-utils
$ glxgears
```

## 5. glxgears가 뜨는 지 확인!
