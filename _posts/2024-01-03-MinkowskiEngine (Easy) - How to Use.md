---
layout: post
title: MinkowskiEngine (Easy) - How to Use
subtitle: Easy Explanation & Examples of MinkowskiEngine
tags: [Pytorch, Python, MinkowskiEngine]
comments: true
---

## MinkowskiEngine 설치

경험 상 MinkowskiEngine을 설치하는 것이 꽤 까다로운 편이다.
그래서 Docker를 사용하는 것을 추천하고, 원작자가 제공한 Docker는 CUDA와 Torch가 너무 옛날버전이어서,
여기 [Install MinkowskiEngine](https://brunch.co.kr/@leadbreak/18) 브런치 글을 참고하면 쉽게 docker로 설치할 수 있다.

## MinkowskiEngine Operation에 대한 배경지식

기억해야할 것은, 이 SparseTensor는 Pytorch의 Tensor와는 다르게, `features`와 `coordinates`로 구성되어 있는 점이다.
왜냐하면 2D image와는 다르게 point cloud는 sparse(군데군데 비어있음)하다보니, 이를 잘 표현하기 위해 데이터를 coordinates를 통해 저장하고,
그에 대응하는 features(2D image 상에서 channel이라고 이해하면 쉬울듯)로 나누어서 저장하는 것이다.

이는 사실 그래프 이론에서 graph를 표현할 때 compressed sparse row (CSR) 표현방식 비슷하다 (하지만 원작자의 의도가 아닌 저의 뇌피셜).
CSR 형식은 희소 행렬이나 그래프를 저장하는 효율적인 방법인데, 
이 형식은 행의 시작 위치와 non-zero 값들, 그리고 해당 값들의 열 인덱스를 별도로 저장한다. 
CSR은 메모리를 절약하고, 행 기반의 연산을 빠르게 수행할 수 있게 해줘서 논문에 'fast graph-theoretic approach'라는 표현이 있으면 거진 이 CSR을 내부적으로 활용하고 있다고 생각하면 편하다.



## MinkowskiEngine 더 쉬운 예제 코드

공식 홈페이지 예제코드를 수정해서 어떻게 MinkowskiEngine을 사용하는는지 작성해보았다.
(설명은 향후 추가 예정)

```python
import torch
import MinkowskiEngine as ME

data0 = [
    [0, 0, 2.1, 0, 0],
    [0, 1, 1.4, 3, 0],
    [0, 0, 4.0, 0, 0]
]

data1 = [
    [0, 3.0, 1.9, 0, 0],
    [0, 0.0, 0.0, 0, 0],
    [0, 1.0, 0.0, 0, 7.0]
]

def to_sparse_coo(data):
    # An intuitive way to extract coordinates and features
    coords, feats = [], []
    for i, row in enumerate(data):
        for j, val in enumerate(row):
            if val != 0:
                coords.append([i, j])
                feats.append([val])
    return torch.IntTensor(coords), torch.FloatTensor(feats)

coords0, feats0 = to_sparse_coo(data0)
coords1, feats1 = to_sparse_coo(data1)

voxels0 = torch.FloatTensor([0.5])
voxels1 = torch.FloatTensor([1.5])

coords, feats = ME.utils.sparse_collate([coords0, coords1], [feats0, feats1])
tensor = ME.SparseTensor(feats, coordinates=coords)

print("Original: ")
print(tensor.F.transpose(0, 1))
# => tensor([[2.1000, 1.0000, 1.4000, 3.0000, 4.0000, 3.0000, 1.9000, 1.0000, 7.0000]])

voxels0_expanded = voxels0.expand(coords0.size(0), -1)
voxels1_expanded = voxels1.expand(coords1.size(0), -1)

# We don't need to use `sparse_collate` in this case
# coords, voxels = ME.utils.sparse_collate([coords0, coords1], [voxels0_expanded, voxels1_expanded])
voxels_expanded = torch.cat([voxels0_expanded, voxels1_expanded], dim=0)
tensor1 = ME.SparseTensor(features=voxels_expanded,
                          coordinate_manager=tensor.coordinate_manager,
                          coordinate_map_key=tensor.coordinate_map_key)

print("In place: multiplication")
tensor *= tensor1
print(tensor.F.transpose(0, 1))
# => tensor([[ 1.0500,  0.5000,  0.7000,  1.5000,  2.0000,  4.5000,  2.8500,  1.5000, 10.5000]])

tensor += tensor1
print("In place: addition")
print(tensor.F.transpose(0, 1))
# => tensor([[ 1.5500,  1.0000,  1.2000,  2.0000,  2.5000,  6.0000,  4.3500,  3.0000, 12.0000]])

# Only features whose coordinates are matched with `coords0` are updated
coords_batch0 = tensor.coordinates_at(0)
voxels2_expanded = torch.FloatTensor([100.0]).expand(coords_batch0.size(0), -1)
coords2, voxels2 = ME.utils.sparse_collate([coords_batch0], [voxels2_expanded])
tensor2 = ME.SparseTensor(coordinates=coords2, features=voxels2, coordinate_manager=tensor.coordinate_manager)

tensor = tensor * tensor2

print("After multiplication")
print(tensor.F.transpose(0, 1))
# => tensor([[120.0000, 155.0000, 4.3500, 100.0000, 200.0000, 3.0000, 250.0000, 12.0000, 6.0000]])

# it does not work!
# tensor *= tensor1
# print(tensor.F.transpose(0, 1))

print("======== Test per-batch operation ========")

coords_batch0 = tensor.coordinates_at(0)
voxels_batch0 = torch.FloatTensor([-500.0]).expand(coords_batch0.size(0), -1)
coords_batch1 = tensor.coordinates_at(1)
voxels_batch1 = torch.FloatTensor([500.0]).expand(coords_batch1.size(0), -1)

# coords2, voxels2 = ME.utils.sparse_collate([coords_batch0, coords_batch1], [voxels_batch0, voxels_batch1])

voxels_expanded = torch.cat([voxels_batch0, voxels_batch1], dim=0)
tensor_per_batch_operation = ME.SparseTensor(features=voxels_expanded,
                          coordinate_manager=tensor.coordinate_manager,
                          coordinate_map_key=tensor.coordinate_map_key)

tensor += tensor_per_batch_operation
print(tensor.F.transpose(0, 1))
# => tensor([[-380.0000, -345.0000, -495.6500, -400.0000, -300.0000,  503.0000, 750.0000,  512.0000,  506.0000]])
print("Done")
```

예제와 유사한데, 

## 결론

MinkowskiEngine을 보다 잘 이용하려면 C++의 unordered_map과 같은 자료구조를 이해하고 있으면 좋다.

