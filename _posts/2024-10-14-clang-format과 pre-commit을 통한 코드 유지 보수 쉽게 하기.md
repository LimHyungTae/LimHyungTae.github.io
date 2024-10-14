---
layout: post
title: clang-format과 pre-commit을 통한 코드 유지 보수 쉽게 하기
subtitle:
tags: [Ubuntu, GitHub]
comments: true
---

## Why `pre-commit` needs?

MIT에 온 이후로, 코드를 다 함께 짜는 시간이 훨씬 많아졌다.
그러다 보니 이것 저것 재미있는 협업 tool들을 접할 기회가 있었는데, 요새 인상 깊게 본 것은 `pre-commit`이다. 
pre-commit은 코드를 저장소에 저장하기 전에 자동으로 일련의 검사를 수행할 수 있는 도구다. 

이것이 왜 필요한지 쉬운 예를 들어 보자.

예를 들어, 학생이 C++ 코드를 작성하고 있다고 해 보자. 코드 스타일 중 하나로 괄호의 위치가 있을 수 있다. 괄호는 함수를 선언하거나 클래스를 정의할 때 사용되며, 괄호를 다음 줄에 시작할지 이전 줄에 붙일지는 팀이나 프로젝트의 스타일 가이드에 따라 다를 수 있다. 예를 들면:

```cpp
코드 복사
// 괄호를 이전 줄에 붙이는 스타일
void function() {
    // do something
}

// 괄호를 다음 줄에 시작하는 스타일
void function()
{
    // do something
}
```

이제, 팀에서는 괄호를 다음 줄에 놓는 것을 선호한다고 가정해보자. 여러 사람이 코드를 작성할 때 스타일을 잘못 적용하는 경우가 있을 수 있다. 이런 실수가 발생하면 코드의 일관성이 떨어지고, 읽기 어려워져서 나중에 유지보수를 어렵게 할 수 있다.

**pre-commit은 이러한 문제를 미리 방지할 수 있다.** 코드를 커밋하기 전에 자동으로 스타일을 검사해서 괄호 위치가 맞지 않으면 알려주고, 때로는 자동으로 고쳐주기도 한다. 이렇게 하면 코드의 품질이 유지되고, 일관성을 유지할 수 있으며, 모든 팀원이 동일한 규칙을 따르도록 할 수 있다.

결국, pre-commit을 사용하면 코드의 품질을 높이고, 일관성을 유지하며, 모든 팀원이 동일한 규칙을 따르도록 만드는 데 큰 도움을 줄 수 있다. 이런 도구를 사용하는 습관을 들이면 더욱 전문적인 개발자로 성장하는 데 좋은 밑거름이 될 수 있다.

--- 

## How to Use

Lukas가 미리 세팅해둔 `.clang-format`파일과 `.pre-commit-config.yaml` 파일을 활용해서 [Patchwork](https://github.com/LimHyungTae/patchwork)에 적용시켜 보았다.

먼저 local 컴퓨터에 pre-commit을 설치해야 한다:

```angular2html
pip3 install pre-commit
```

그 이후, 해당 repository에 위의 `.clang-format`파일과 `.pre-commit-config.yaml`을 디렉토리의 root에 두고, 아래 명령어를 실행하면 pre-commit이 설정된다:

```angular2html
pre-commit install
```

뭔가 꼬였다고 느껴지면 uninstall을 했다가 재설치를 하면 된다:
```angular2html
pre-commit uninstall
```

그 후, 

```angular2html
pre-commit run --all-files
```
를 입력하면 세팅돼 있는  `.clang-format`파일과 `.pre-commit-config.yaml`의 option을 기반으로 pre-commit 설정된다.
나의 경우 명령어를 실행하니, 아래와 같은 에러가 발생했는데:

```angular2html
pre-commit run --all-files
[INFO] Initializing environment for https://github.com/pre-commit/pre-commit-hooks.
[INFO] Initializing environment for https://github.com/psf/black.
[INFO] Initializing environment for https://github.com/pre-commit/mirrors-clang-format.
[INFO] Initializing environment for https://github.com/cpplint/cpplint.
[INFO] Installing environment for https://github.com/pre-commit/pre-commit-hooks.
[INFO] Once installed this environment will be reused.
[INFO] This may take a few minutes...
An unexpected error has occurred: CalledProcessError: command: ('/usr/bin/python3', '-mvirtualenv', '/home/shapelim/.cache/pre-commit/repotpctp_ta/py_env-python3')
return code: 1
stdout:
    AttributeError: module 'virtualenv.create.via_global_ref.builtin.cpython.mac_os' has no attribute 'CPython2macOsFramework'
stderr: (none)
Check the log at /home/shapelim/.cache/pre-commit/pre-commit.log
```

`virtualenv`가 꼬여서 위의 에러가 발생한 듯 하다. 그래서 

```angular2html
pip uninstall virtualenv
pip install virtualenv
```

를 해주니 해결됐다.

## Details of Using `pre-commit`

위의 `pre-commit run --all-files`를 실행시키면 아마 에러들이 아래와 같이 주욱 뜰 것이다:


![](/img/pre-commit-example.png)


![](/img/pre-commit-example2.png)
