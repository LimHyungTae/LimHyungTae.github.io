---
layout: post
title: C++ zero padding하는 법
subtitle: Use boost format
tags: [C++]
comments: true
---

# C++ Zero padding하는 법

파일 시스템을 관리하다보면 zero padding을 꼭해야할 때가 있다.

만약 Sort를 해야하는 상황이나 sequence 순서대로 데이터를 저장/불러오기 할때에는 zero padding을 꼭 해야 한다.

그렇지 않으면, 파일 순서가 섞일 수도 있기 때문이다, e.g. 10.pcd와 100.pcd같이 같은 1로 시작하는 데이터들이 오름차순으로 위치하게 되면 순서가 뒤죽박죽된다.

따라서, 그러한 문제를 해결하기 위해 zero padding이 필요하다.


기존에는 c++의 std를 이용해서 약간 문제를 우회해서 풀었는데,

최근에 `boost/format`을 알게되어 공유한다.

해당 코드는 아래와 같다.

<script src="https://gist.github.com/LimHyungTae/812985af7b8ab9ae7e85fda3ec3ec675.js"></script>

## 20221024 업데이트

이미 pcd 파일이 zero padding 없이 만들어졌다면? 아래의 명령어를 command 상에서 입력하면 파일의 숫자의 칸이 6칸인 파일 명으로 변경해준다 (e.g. 12.pcd를 000012.pcd로)

```bash
for file in [0-9]*.pcd;
do
name=${file%.*}
extension=${file##*.}
new_name=`printf %06d.%s ${name} ${extension}`
mv -n $file $new_name
done
```
