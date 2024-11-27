---
layout: post
title: Pybind11 Line by Line - 3. pyproject.toml을 통한 pip3 install 지원하기
subtitle: Understanding How pip3 works
tags: [Pybind11, Pybinding, Python, C++]
comments: true
---

이제 Pybinding이 잘 된다는 가정 하에, 더 많은 사람들이 손 쉽게 쓸 수 있게 Pypi에 python 코드를 업로드해서 `pip3 install`로 설치하는 방법을 알려주고자 한다.
참고로 나도 TEASER++의 pip3 install화를 위해 line by line 스터디를 한 것이기 때문에, 실제로 연구자가 자기의 코드를 open할 때 겪는 시행착오들을 공유하고자 한다.

### 1. `pyproject.toml` 파일 구성

먼저, `pyproject.toml` 파일을 구성해야한다. 다른 블로그들([블로그1](https://gbdai.tistory.com/59), [블로그2](https://teddylee777.github.io/python/pypi/))에서는 `setup.py`를 설정해야 한다고 되어 있다.
하지만 이 [블로그 글](https://miintto.github.io/docs/python-pypi-packages)을 보면 이제는 `pyproject.toml` 파일로 표현하는 것으로 표준화가 되었다고 한다.
실제로 나의 동료인 Nacho의 [KISS-ICP](https://github.com/PRBonn/kiss-icp/blob/main/python/pyproject.toml)에서도 `pyproject.toml`를 사용하는 것을 볼 수 있다.

현재 KISS-ICP 상의 `pyproject.toml`을 따라서 아래와 같이 TEASER++의 `pyproject.toml` 파일을 구성해 보았다:

```angular2html
[build-system]
requires = ["scikit_build_core", "pybind11"]
build-backend = "scikit_build_core.build"

[project]
name = "teaserpp_python"
version = "1.1.0"
description = "Python binding for TEASER++"
readme = "README.md"
authors = [
    { name = "Jingnan Shi", email = "jnshi@mit.edu" },
]
requires-python = ">=3.6"
keywords = [
    "Point cloud",
    "Registration",
    "Non-minimal solver",
    "Solver",
]
classifiers = [
    "Intended Audience :: Developers",
    "Intended Audience :: Education",
    "Intended Audience :: Other Audience",
    "Intended Audience :: Science/Research",
    "License :: OSI Approved :: MIT License",
    "Operating System :: MacOS",
    "Operating System :: Microsoft :: Windows",
    "Operating System :: Unix",
    "Programming Language :: C++",
    "Programming Language :: Python :: 3",
    "Programming Language :: Python :: 3.7",
    "Programming Language :: Python :: 3.8",
    "Programming Language :: Python :: 3.9",
    "Programming Language :: Python :: 3.10",
    "Programming Language :: Python :: 3.11",
]
dependencies = [
    "numpy",
]

[project.urls]
Homepage = "https://github.com/MIT-SPARK/TEASER-plusplus"

[tool.scikit-build]
build-dir = "build/{wheel_tag}"
cmake.verbose = false
cmake.minimum-version = "3.16"
editable.mode = "redirect"
sdist.exclude = ["pybind/"]
wheel.install-dir = "teaser/pybind/"

[tool.black]
line-length = 100

[tool.isort]
profile = "black"

[tool.pylint.format]
max-line-length = "100"

[tool.cibuildwheel]
archs = ["auto64"]
skip = ["*-musllinux*",  "pp*", "cp36-*"]

[tool.cibuildwheel.macos]
environment = "MACOSX_DEPLOYMENT_TARGET=10.14"
archs = ["auto64", "arm64"]
```

2. `pyproject.toml` 파일을 통한 빌드 

먼저 향후 빌드에 필요한 library들을 미리 설치해보자:

```angular2html
pip3 install setuptools build wheel twine
```

그 후, 아래의 명령어를 `pyproject.toml` 파일에서 입력하면 아마 프로젝트가 빌드 될것이다:

```
python3 -m build
```

여기서 `-m build` 명령어는 프로젝트의 `pyproject.toml` 파일을 읽고, `.whl` 및 `.tar.gz` 파일을 생성합니다.
아마 성공적으로 build가 되면 커맨드라인에 아래와 같이 성공적으로 build되었다는 알림이 뜬다:

![](/img/1127_buid_done.png)

![](/img/1127_linux_x86_64_error.png)