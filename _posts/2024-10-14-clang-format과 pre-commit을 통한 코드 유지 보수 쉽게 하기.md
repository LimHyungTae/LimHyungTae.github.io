---
layout: post
title:  clang-format과 pre-commit을 통한 코드 유지 보수 쉽게 하기
subtitle:
tags: [Ubuntu, GitHub, Maintenance]
comments: true
---

## Why `pre-commit` needs?

MIT에 온 이후로, 코드를 다 함께 짜는 시간이 훨씬 많아졌다.
그러다 보니 이것저것 재미있는 협업 tool들을 접할 기회가 있었는데, 요즘 인상 깊게 본 것은 `pre-commit`이다. 
`pre-commit`은 코드를 저장소에 저장하기 전에 자동으로 일련의 검사를 수행할 수 있는 도구다. 

이것이 왜 필요한지 쉬운 예를 들어 보자.

예를 들어, 학생이 C++ 코드를 작성하고 있다고 해 보자. 
코드 스타일 중 대표적인 예시로 하나로 괄호의 위치가 있을 수 있는데, 괄호를 다음 줄에 시작할지 이전 줄에 붙일지는 팀이나 프로젝트의 스타일 가이드에 따라 다를 수 있다. 
예를 들면:

```cpp
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

이제, 팀에서는 괄호를 다음 줄에 놓는 것을 선호한다고 가정해보자. 
여러 사람이 코드를 작성할 때 스타일을 잘못 적용하는 경우가 있을 수 있다. 
이런 실수가 발생하면 코드의 일관성이 떨어져서, 최종적으로는 코드가 지저분한 느낌이 난다(필자의 개발자 친구들은 늘 '레포지토리는 한 명의 개발자가 작성한 것과 같이 되어야 한다'고 강조했던 적이 있다). 

`pre-commit`**은 이러한 문제를 미리 방지할 수 있다.** 
코드를 커밋하기 전에 자동으로 스타일을 검사해서 괄호 위치가 맞지 않으면 알려주고, 때로는 자동으로 고쳐주기도 한다. 
이렇게 하면 코드의 품질이 유지되고, 일관성을 유지할 수 있으며, 모든 팀원이 동일한 규칙을 따르도록 할 수 있다.

결국, `pre-commit`을 사용하면 코드의 품질을 높이고, 일관성을 유지하며, 모든 팀원이 동일한 규칙을 따르도록 만드는 데 큰 도움을 줄 수 있다. 

--- 

## How to Use

굳이 모든 파일이 무슨 의미인지 깊게 이해할 필요는 없다 (만약 궁금하다면 [이 글](https://pypy.dev/git/pre-commit-%EC%9C%BC%EB%A1%9C-git-hook-%EC%82%AC%EC%9A%A9%ED%95%98%EA%B8%B0-with-python/)과 [저 글](https://daco2020.tistory.com/291)을 읽어보길 추천한다)!
Lukas가 [Khronos](https://github.com/MIT-SPARK/Khronos) 코드 공개할 때 미리 세팅해둔 `.clang-format`파일과 `.pre-commit-config.yaml` 파일을 활용해서 [Patchwork](https://github.com/LimHyungTae/patchwork)에 적용시켜 보았다.

먼저 local 컴퓨터에 pre-commit을 설치해야 한다:

```angular2html
pip3 install pre-commit
```

그 이후, 해당 repository에 위의 `.clang-format`파일과 `.pre-commit-config.yaml`을 디렉토리의 root에 두고, 아래 명령어를 실행하면 해당 Git 레포지토리의 pre-commit이 설정된다:

```angular2html
pre-commit install
```

설치 중 뭔가가 꼬였다고 느껴지면 uninstall을 했다가 재설치를 하면 된다:
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

## Modification of `pre-commit`

ChatGPT의 시대가 되었으니, line-by-line이 무슨 역할을 하는지 궁금한 사람은 ChatGPT에게 물어보면 아주 친절히 답을 들을 수 있을 것이다. 
나는 pre-commit을 설정하는 과정에서 내가 겪었던 변경 사항을 공유하고자 한다(그리고 아래 변경 사항이 레포지토리를 세팅하는 입장에서 수정할 사항이라고 생각한다).

### 1. `exclude`: 특정 파일이나 폴더를 제외하고 싶을 때 
위의 `pre-commit run --all-files`를 실행시키면 아마 에러들이 아래와 같이 주욱 뜰 것이다:


![](/img/pre-commit-example.png)

이는 `.pre-commit-config.yaml` 내의 cpplint가 현재 코드가 코드 포맷의 rule을 따르지 않았으니 user에게 수정하라고 명령하는 것이다. 
그런데, 나의 경우에는 `nanoflann.hpp`와 `nanoflann_utils.hpp`는 내가 짠 코드가 아니고 원 코드에서 복붙해온 코드였어서, `cpplint`의 대상에서 제외시키고 싶었다.  
그럴 때는 아래와 같이 `exclude` 줄에 제외하고 싶은 파일이나 폴더를 `|` 뒤에 추가하면 된다 (`|` 는 정규 표현식에서 or를 뜻한다):

**Original code** from Khronos:

```angular2html
- repo: https://github.com/cpplint/cpplint
    rev: "1.6.1"
    hooks:
      - id: cpplint
        args:
          [
            "--filter=-whitespace/line_length,-legal/copyright,-build/include_order,-runtime/references,-build/c++11,-build/namespaces",
          ]
        exclude: 3rd_party/
```

**Revised code** in Patchwork:

```angular2html
- repo: https://github.com/cpplint/cpplint
    rev: "1.6.1"
    hooks:
      - id: cpplint
        args:
          [
            "--filter=-whitespace/line_length,-legal/copyright,-build/include_order,-runtime/references,-build/c++11,-build/namespaces",
          ]
        exclude: '3rd_party/|include/label_generator/nanoflann\.hpp|include/label_generator/nanoflann_utils\.hpp'
```

### 2. `--filter`: 특정 파일이나 폴더를 제외하고 싶을 때

그리고 고쳐야 하는 리스트 중 굳이 안 고쳐도 될 것 같은 부분을 제외하고 싶을 때가 있다(하지만 Lukas가 세팅한 default 옵션은 굉장히 reasonable하긴 하다. 되도록이면 그냥 따르는 것을 추천한다).
특히 나의 경우는, 해당 repository를 더이상 관리하지 않기 때문에 노력을 많이 쏟고 싶지 않았다.
그래서 고치지 않아도 무방한 부분을 해당 `--filter`에 추가해주었다. 
추가하는 방법은, 아래 스크린샷의 `[${CPPLINT TYPE}]`의 `${CPPLINT TYPE}` 부분을 filter 뒤에 추가해주면 된다: 

![](/img/pre-commit-example2.png)

```angular2html
- repo: https://github.com/cpplint/cpplint
    rev: "1.6.1"
    hooks:
      - id: cpplint
        args:
          [
            "--filter=-whitespace/line_length,-legal/copyright,-build/include_order,-runtime/references,-build/c++11,-build/namespaces,-build/header_guard,-runtime/string",
          ]
        exclude: '3rd_party/|include/label_generator/nanoflann\.hpp|include/label_generator/nanoflann_utils\.hpp'
```

---

##추가: 내가 사용하는 `.clang-format`

내가 사용하는 clang format을 공유한다:

```angular2html
---
Language: Cpp
BasedOnStyle: Google
AccessModifierOffset: -1
AlignAfterOpenBracket: Align
AlignConsecutiveAssignments: true
AlignConsecutiveDeclarations: false
AlignOperands: true
AlignTrailingComments: true
AllowAllParametersOfDeclarationOnNextLine: false
AllowShortBlocksOnASingleLine: false
AllowShortCaseLabelsOnASingleLine: false
AllowShortFunctionsOnASingleLine: All
AllowShortIfStatementsOnASingleLine: true
AllowShortLoopsOnASingleLine: true
AlwaysBreakAfterDefinitionReturnType: None
AlwaysBreakAfterReturnType: None
AlwaysBreakBeforeMultilineStrings: true
AlwaysBreakTemplateDeclarations: true
BinPackArguments: false
BinPackParameters: false
BraceWrapping:
  AfterClass: false
  AfterControlStatement: false
  AfterEnum: false
  AfterFunction: false
  AfterNamespace: false
  AfterObjCDeclaration: false
  AfterStruct: false
  AfterUnion: false
  BeforeCatch: false
  BeforeElse: false
  IndentBraces: false
BreakBeforeBinaryOperators: None
BreakBeforeBraces: Attach
BreakBeforeTernaryOperators: true
BreakConstructorInitializersBeforeComma: false
CommentPragmas: "^ IWYU pragma:"
ConstructorInitializerAllOnOneLineOrOnePerLine: true
ConstructorInitializerIndentWidth: 4
ContinuationIndentWidth: 4
Cpp11BracedListStyle: true
DerivePointerAlignment: true
DisableFormat: false
ExperimentalAutoDetectBinPacking: false
ForEachMacros:
  - foreach
  - Q_FOREACH
  - BOOST_FOREACH
IncludeCategories:
  # Spacers
  - Regex: "^<clang-format-priority-15>$"
    Priority: 15
  - Regex: "^<clang-format-priority-25>$"
    Priority: 25
  - Regex: "^<clang-format-priority-35>$"
    Priority: 35
  - Regex: "^<clang-format-priority-45>$"
    Priority: 45
  # C system headers
  - Regex: '^[<"](aio|arpa/inet|assert|complex|cpio|ctype|curses|dirent|dlfcn|errno|fcntl|fenv|float|fmtmsg|fnmatch|ftw|glob|grp|iconv|inttypes|iso646|langinfo|libgen|limits|locale|math|monetary|mqueue|ndbm|netdb|net/if|netinet/in|netinet/tcp|nl_types|poll|pthread|pwd|regex|sched|search|semaphore|setjmp|signal|spawn|stdalign|stdarg|stdatomic|stdbool|stddef|stdint|stdio|stdlib|stdnoreturn|string|strings|stropts|sys/ipc|syslog|sys/mman|sys/msg|sys/resource|sys/select|sys/sem|sys/shm|sys/socket|sys/stat|sys/statvfs|sys/time|sys/times|sys/types|sys/uio|sys/un|sys/utsname|sys/wait|tar|term|termios|tgmath|threads|time|trace|uchar|ulimit|uncntrl|unistd|utime|utmpx|wchar|wctype|wordexp)\.h[">]$'
    Priority: 10
  # C++ system headers
  - Regex: '^[<"](algorithm|any|array|atomic|bitset|cassert|ccomplex|cctype|cerrno|cfenv|cfloat|charconv|chrono|cinttypes|ciso646|climits|clocale|cmath|codecvt|complex|condition_variable|csetjmp|csignal|cstdalign|cstdarg|cstdbool|cstddef|cstdint|cstdio|cstdlib|cstring|ctgmath|ctime|cuchar|cwchar|cwctype|deque|exception|execution|filesystem|forward_list|fstream|functional|future|initializer_list|iomanip|ios|iosfwd|iostream|istream|iterator|limits|list|locale|map|memory|memory_resource|mutex|new|numeric|optional|ostream|queue|random|ratio|regex|scoped_allocator|set|shared_mutex|sstream|stack|stdexcept|streambuf|string|string_view|strstream|system_error|thread|tuple|type_traits|typeindex|typeinfo|unordered_map|unordered_set|utility|valarray|variant|vector)[">]$'
    Priority: 20
  # Other library h files (with angles)
  - Regex: "^<"
    Priority: 30
  # Your project's h files (with angles)
  - Regex: "^<kiss_matcher"
    Priority: 40
  # Your project's h files
  - Regex: '^"kiss_matcher'
    Priority: 50
IndentCaseLabels: true
IndentWidth: 2
IndentWrappedFunctionNames: false
KeepEmptyLinesAtTheStartOfBlocks: false
MacroBlockBegin: ""
MacroBlockEnd: ""
ColumnLimit: 100
MaxEmptyLinesToKeep: 1
NamespaceIndentation: None
ObjCBlockIndentWidth: 2
ObjCSpaceAfterProperty: false
ObjCSpaceBeforeProtocolList: false
PenaltyBreakBeforeFirstCallParameter: 1
PenaltyBreakComment: 300
PenaltyBreakFirstLessLess: 120
PenaltyBreakString: 1000
PenaltyExcessCharacter: 1000000
PenaltyReturnTypeOnItsOwnLine: 200
PointerAlignment: Left
ReflowComments: true
SortIncludes: true
SpaceAfterCStyleCast: false
SpaceBeforeAssignmentOperators: true
SpaceBeforeParens: ControlStatements
SpaceInEmptyParentheses: false
SpacesBeforeTrailingComments: 2
SpacesInAngles: false
SpacesInContainerLiterals: true
SpacesInCStyleCastParentheses: false
SpacesInParentheses: false
SpacesInSquareBrackets: false
Standard: Auto
TabWidth: 4
UseTab: Never
```

사용할 때는 `kiss_matcher`라고 되어 있는 부분만 프로젝트에 따라 변경해 주면 된다. 그러면 `"kiss_matcher/BLABLA"`로 되어있는 헤더 선언 파일이 자동적으로 가장 뒤로 가서 배치된다!
그 이외로 선호도를 탈 거 같은 것은

* `AlignConsecutiveAssignments: true`: `=`을 기준으로 맞춰 줌. VIM 유저로서 block highlight를 많이 쓰기 때문에 =를 기준으로 줄 맞춰주면 편해서 사용함.
* `AlignConsecutiveDeclarations: false`: 함수 변수들도 한 줄로 맞춰는데, 이걸 `true`로 하면 내 기준 살짝 과하게 정렬하는 것 같아서, false로 사용함

## 결론 

이러면 이제 여러 명이서 같이 작업할 때 코드 포맷에 대해서 스트레스를 받지 않아도 된다. pre-commit 짱짱!
여담으로, 내가 처음 C++ 배울 때만 해도 indent를 4 space로 했는데, 요즘은 2 space가 대세인듯 하다.  

