---
layout: post
title: C++ KITTI dataset 파일 load하기 (2) pointcloud bin 파일
subtitle: KITTI data Load하기
tags: [C++, Text, Eigen]
comments: true
---

# KITTI Dataset pointcloud 불러오기

KITTI dataset의 pointcloud는 pcd 형식으로 되어있으면 좋을텐데, 아쉽게도 bin 파일로 되어 있다.

따라서 bin 파일을 연 후에 파싱을 해주어야 한다.

주로 사용하는 `kittiloader.hpp`를 공유한다.

해당 코드는 아래와 같다.

<script src="https://gist.github.com/LimHyungTae/cb52f6540210bcad91d942eb0c88bbaf.js"></script>

```cpp
# velodyne path
string target_path = "/your/kitti/path/sequences/00/velodyne";
KittiLoader loader(target_path);
# what you want to call
int seq_frame = 10;
loader.cloud(seq_frame);
```
