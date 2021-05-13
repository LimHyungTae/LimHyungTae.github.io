---
layout: post
title: C++ KITTI dataset 파일 load하기 (3) label 파일
subtitle: KITTI data Load하기
tags: [C++, Text, Eigen]
comments: true
---

# KITTI Dataset label 불러오기

최근에는 SemanticKITTI dataset에서 point-wise로 semantic label이 되어있는 label을 오픈했다.

label은 또 .label파일로 되어있기 때문에, 파싱이 필요하다.

rangenet++에서 좋은 passing 방법이 있어서, 정리한 코드를 공유한다.

해당 코드는 아래와 같다.

<script src="https://gist.github.com/LimHyungTae/4964102b3f46765845467dcf2b4bd608.js"></script>

코드를 설명하자면, 핵심을 `seekg()`와 `tellg()` 함수인데,

1. `seekg()` 함수를 통해 binary 파일 내의 입력 위치 지정자의 위치를 맨 뒤로 이동시켜서(` scan_input.seekg(0, std::ios::end);) 
2. 전체 point 수의 갯수를 리턴 받은 후(27번 째 줄 `uint32_t num_points = scan_input.tellg() / (4 * sizeof(float));`),
3. 다시 입력 위치 지정자를 제일 앞으로 옮긴 후 (`scan_input.seekg(0, std::ios::beg);`)
4. values vector에 binary 파일 내의 binary 값들을 옮긴다 (`scan_input.read((char*)&values[0], 4 * num_points * sizeof(float));`)

마찬가지로, .label 파일을 불러올때도 똑같은데,

SemanticKITTI dataset에서 point의 수와 label의 수는 무조건 동일하므로 여기서는 num_points를 체크하지 않고 바로 vector에 데이터를 담는 것을 볼 수 있다.
