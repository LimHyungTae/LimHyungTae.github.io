---
layout: post
title: Ubuntu에서 library 깔끔히 제거하는 방법
subtitle: 라이브러리는 꼬이지 않는다. 잘 제거하지 못 했을 뿐
tags: [Ubuntu, filesystem, library]
comments: true
---

석사 때는 Ubuntu를 사용하면서 라이브러리를 깔끔히 제거하는 방법을 잘 몰랐다보니, 뭔가 막히면 컴퓨터를 울면서 포맷했었던 기억이 난다.
그런데, 사실 그럴 필요가 없었다는 생각이 드는 요즘이다.

기본적으로 library를 '설치'하게 되면 파일들이 `/usr/local/` 아래로 가게 되는데, 그냥 이 하위 폴더 아래 해당하는 파일들을 `sudo rm`로 제거해주기만 하면 된다.
참고로 설치 시 `sudo make install`을 사용했다면 `sudo make uninstall`을 사용할 수 있다.

**해당 library의 파일들 위치 찾기**

```bash
sudo find /usr/local -name "*${FILE_NAME}*"
```

e.g. 내가 제거하려는 대상이 `gtsam`이라면:

```bash
sudo find /usr/local -name "*gtsam*"
```
