---
layout: post
title: clang-format Configuration file(s) do(es) not support Json 해결방법
subtitle:
tags: [Ubuntu, GitHub, Maintenance]
comments: true
---

## 문제점

어느 날 KISS-Matcher 오픈 소스를 준비하던 중, clang-format을 설정했는데 아래와 같은 에러가 발생했다:

```angular2html
Configuration file(s) do(es) not support Json: /home/shapelim/git/kiss-matcher/.clang-format Configuration file(s) do(es) not support Json:
```

내 workspace에 `compile_commands.json` 파일이 존재한다. 
이 파일을 검사해야 하는데(사실 검사 안해도 됨...자동 완성을 위한 파일이기 때문에) clang-format에는 json 파일에 대해서 어떻게 뭘 검토해야하는지 명시되어 있지 않아서 에러가 난 듯 하다.

## 해결 방법

해결 방법은 two stage로 구성되어 있다.

먼저, `.clang-format`에 아래와 같이 라인을 추가해주면 된다:

```angular2html
---
# For addressing 'Configuration file(s) do(es) not support Json' error
Language: Json
BasedOnStyle: llvm
```

그런데 그 줄을 추가하고 나면, 추가적인 에러가 발생한다:

```angular2html
check yaml...............................................................Failed
- hook id: check-yaml
- exit code: 1

expected a single document in the stream
  in ".clang-format", line 2, column 1
but found another document
  in ".clang-format", line 109, column 1
```

즉, `.clang-format`에는 `---`로 하나의 언어만 선언하게끔 원래 되어있나 보다(확실하지 않음).
따라서 `.clang-format`을 아래와 같이 pre-commit의 대상에서 제외해주면 된다. `.pre-commit-config.yaml`에 아래와 같이 `exclude \.clang-format`을 추가해주면 된다:

```angular2html
repos:
  - repo: https://github.com/pre-commit/pre-commit-hooks
    rev: v4.4.0
    hooks:
      - id: check-yaml
        exclude: \.clang-format$
      - id: check-json
      - id: check-xml
      - id: end-of-file-fixer
      - id: trailing-whitespace
      - id: check-added-large-files
      - id: check-merge-conflict

  - repo: https://github.com/psf/black
    rev: "23.7.0"
    hooks:
      - id: black

  # - repo: https://github.com/pylint-dev/pylint
  #   rev: "v3.0.0a6"
  #   hooks:
  #     - id: pylint

  - repo: https://github.com/pre-commit/mirrors-clang-format
    rev: "v16.0.6"
    hooks:
      - id: clang-format

  - repo: https://github.com/cpplint/cpplint
    rev: "1.6.1"
    hooks:
      - id: cpplint
        args:
          [
            "--filter=-whitespace/line_length,-legal/copyright,-build/include_order,-runtime/references,-build/c++11,-build/namespaces",
          ]
        exclude: 'cpp/kiss_matcher/3rdparty|cpp/kiss_matcher/core/kiss_matcher/kdtree|cpp/kiss_matcher/core/kiss_matcher/tsl|cpp/kiss_matcher/core/kiss_matcher/points'
```


