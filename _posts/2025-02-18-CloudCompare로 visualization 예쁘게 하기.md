---
layout: post
title: CloudCompare로 visualization 예쁘게 하기
subtitle: How to visualize beautiful point cloud figure
tags: [CloudCompare]
comments: true
description: CloudCompare의 EDL 셰이더와 z 좌표 Scalar Field export를 활용해 point cloud 시각화를 깔끔하게 다듬는 단계별 팁을 정리한다.
image: /img/cloud_compare_highlighted.png
permalink: /2025/02/18/cloudcompare-pretty-visualization/
redirect_from:
  - '/2025-02-18-CloudCompare로 visualization 예쁘게 하기/'
  - '/2025-02-18-CloudCompare로-visualization-예쁘게-하기/'
---

## TL;DR 

1. Point cloud를 load한 후 좌측의 EDL (Eye Dome Lighting OpenGL Shader)를 클릭

![img](/img/cloud_compare_highlighted.png)

2. `DB Tree` 구간에서 point cloud를 클릭한 후, Tools > Projection > Export coordinate(s) to SF(s) 클릭 후 `z`를 check하고 OK 클릭

3. 좌하단의 `Properties`의 `Colors`에서 `Scalar field`를 선택 후(SF를 export함으로써 `Colors` 란이 생김), `SF display params`에서 color의 threshold만 적절히 변경해주면 끝! 

최종적으로 아래와 같은 결과를 얻을 수 있다:

![img](/img/cloud_compare_after.png)
