---
layout: post
title: Python Ray를 딥러닝 병렬 처리
subtitle: How to use Ray for deep learning parallel processing
tags: [Python, Ray, 딥러닝]
comments: true
---

How to give them CUDA

Set up `num_gpus=1`

```python
import ray
import torch
import os
import torch
print("CUDA Available:", torch.cuda.is_available())
print("CUDA Device Count:", torch.cuda.device_count())
print("Current CUDA Device:", torch.cuda.current_device())
print("CUDA Device Name:", torch.cuda.get_device_name(torch.cuda.current_device()))

print(os.environ.get("CUDA_VISIBLE_DEVICES"))

ray.init(num_gpus=1)

@ray.remote(num_gpus=1)
def check_cuda():
    print(os.environ.get("CUDA_VISIBLE_DEVICES"))
    return torch.cuda.is_available(), torch.cuda.device_count()

print(ray.get(check_cuda.remote()))
```

```
    @staticmethod
    @ray.remote(num_gpus=1)
    def match_in_parallel(descriptor, mutual_matching, inlier_search, num_keypts, 
                          des_r, src_pts, tgt_pts, src_pcd_raw, tgt_pcd_raw, dataset_name, cfg):
        # Keypoint Sampling
        s_pts_flipped, t_pts_flipped = src_pts[None].transpose(1, 2).contiguous(), tgt_pts[None].transpose(1,2).contiguous()
        s_fps_idx = pnt2.furthest_point_sample(src_pts[None], num_keypts)
        t_fps_idx = pnt2.furthest_point_sample(tgt_pts[None], num_keypts)
        kpts1 = pnt2.gather_operation(s_pts_flipped, s_fps_idx).transpose(1, 2).contiguous()
        kpts2 = pnt2.gather_operation(t_pts_flipped, t_fps_idx).transpose(1, 2).contiguous()

        # Compute descriptors
        src = descriptor(src_pcd_raw[None], kpts1, des_r, dataset_name)
        tgt = descriptor(tgt_pcd_raw[None], kpts2, des_r, dataset_name)
        src_des, src_equi, s_R = src['desc'], src['equi'], src['R']
        tgt_des, tgt_equi, t_R = tgt['desc'], tgt['equi'], tgt['R']

        # Mutual Matching
        s_mids, t_mids = mutual_matching(src_des, tgt_des)

        ss_kpts = kpts1[0, s_mids]
        ss_equi = src_equi[s_mids]
        ss_R = s_R[s_mids]
        tt_kpts = kpts2[0, t_mids]
        tt_equi = tgt_equi[t_mids]
        tt_R = t_R[t_mids]


        ind = inlier_search(ss_equi[:, :, 1:cfg.patch.ele_n - 1],
                            tt_equi[:, :, 1:cfg.patch.ele_n - 1])

        # Recovery pose
        angle = ind * 2 * np.pi / cfg.patch.azi_n + 1e-6
        angle_axis = torch.zeros_like(ss_kpts)
        angle_axis[:, -1] = 1
        angle_axis = angle_axis * angle[:, None]
        azi_R = Convert.axis_angle_to_rotation_matrix(angle_axis)

        R = tt_R @ azi_R @ ss_R.transpose(-1, -2)
        t = tt_kpts - (R @ ss_kpts.unsqueeze(-1)).squeeze()

        return R, t, ss_kpts, tt_kpts


```

하지만 직렬화 오버헤드가 문제가 될 수 있다. 

## 직렬화 오버헤드란?
직렬화(Serialization)는 객체를 저장하거나 네트워크로 전송하기 위해 바이트(byte) 형태로 변환하는 과정이다.
반대로 역직렬화(Deserialization)는 바이트 형태 데이터를 다시 원래 객체로 변환하는 과정이다.

## 직렬화 오버헤드(Serialization Overhead)
데이터를 직렬화하는 데 걸리는 추가적인 연산 비용을 의미한다.
Ray에서 remote()를 호출할 때, 객체를 pickle로 변환해서 전달해야 한다.
하지만 객체가 크거나, 너무 자주 직렬화하면 속도가 느려질 수 있다.
## 직렬화 오버헤드가 발생하는 이유

```python
futures = [MyNetwork.process_data.remote(x, self.calculator) for x in data_list]
```

`self.calculator`는 일반적인 Python 객체이므로, Ray는 이를 다른 프로세스로 넘기기 위해 pickle을 이용해 직렬화한다.
100개의 Worker가 생성될 때마다 `self.calculator`를 계속 직렬화/역직렬화하기 때문에 불필요한 오버헤드가 발생한다.

실제로 네트워크를 pickle로 전달하는 과정에서 오히려 시간이 더 걸리는 경우가 많다.
