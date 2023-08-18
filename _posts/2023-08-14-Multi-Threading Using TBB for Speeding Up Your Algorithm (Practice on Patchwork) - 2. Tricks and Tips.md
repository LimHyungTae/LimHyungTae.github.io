
---

layout: post
title: 2023-08-14-Multi-Threading Using TBB for Speeding Up Your Algorithm (Practice on Patchwork)
subtitle: Towards TBB
tags: [SLAM, LiDAR, Pointcloud, ROS, PCL, Patchwork]
comments: true

# Under Writing!!! Will be completed soon!

---

## Motivation

Nowadays, many open-source codes employ TBB to speed up their algorithms. 
However, there are few tutorials or materials to help we understand how to use TBB for speeding up our own algorithms.
It is the same for me also; I really wanted to get to know about multi-threading better.
While there are many reference codes, they are already completed, so they do not provide any tips regarding how to change an algorithm that is single-thread to multi-thread.
For that reason, I imp

I would like to share my experience of using TBB to speed up my own algorithm.
Benchmarking [KISS-ICP](https://github.com/PRBonn/kiss-icp), I aim for speeding up a point cloud ground segmentation approach, *Patchwork*, as much as possible.

### About Patchwork 


Since the original code was implemented by me two years ago, and it was a single thread-based version.
Nevertheless, it operates at almost 50 Hz. But I felt that it is not enough for real-time applications.
And as I argued, ground segmentation is a preprocessing step for many SLAM algorithms, so it is the faster, the better.

#### pc2czm

Mean time: 
[1.35000000e+02 1.75901487e-02 9.74399181e+01 9.13463236e+01
 9.68828310e+01 9.13463236e+01]
Hz:  56.85000261069478
Mean time: 
0.004318566531365313


```commandline
tbb::mutex mtx;
int N = src.points.size();
tbb::parallel_for(0, N, [&](int i) {
        auto pt = src.points[i];
        int    ring_idx, sector_idx;
        double r = xy2radius(pt.x, pt.y);
        if ((r <= max_range_) && (r > min_range_)) {
            double theta = xy2theta(pt.x, pt.y);

            if (r < min_range_z2_) { // In First rings
                ring_idx =
                    min(static_cast<int>(((r - min_range_) / ring_sizes_[0])), num_rings_each_zone_[0] - 1);
                sector_idx = min(static_cast<int>((theta / sector_sizes_[0])), num_sectors_each_zone_[0] - 1);
                tbb::mutex::scoped_lock lock(mtx);
                czm[0][ring_idx][sector_idx].points.emplace_back(pt);
            } else if (r < min_range_z3_) {
                ring_idx =
                    min(static_cast<int>(((r - min_range_z2_) / ring_sizes_[1])), num_rings_each_zone_[1] - 1);
                sector_idx = min(static_cast<int>((theta / sector_sizes_[1])), num_sectors_each_zone_[1] - 1);
                tbb::mutex::scoped_lock lock(mtx);
                czm[1][ring_idx][sector_idx].points.emplace_back(pt);
            } else if (r < min_range_z4_) {
                ring_idx =
                    min(static_cast<int>(((r - min_range_z3_) / ring_sizes_[2])), num_rings_each_zone_[2] - 1);
                sector_idx = min(static_cast<int>((theta / sector_sizes_[2])), num_sectors_each_zone_[2] - 1);
                tbb::mutex::scoped_lock lock(mtx);
                czm[2][ring_idx][sector_idx].points.emplace_back(pt);
            } else { // Far!
                ring_idx =
                    min(static_cast<int>(((r - min_range_z4_) / ring_sizes_[3])), num_rings_each_zone_[3] - 1);
                sector_idx = min(static_cast<int>((theta / sector_sizes_[3])), num_sectors_each_zone_[3] - 1);
                tbb::mutex::scoped_lock lock(mtx);
                czm[3][ring_idx][sector_idx].points.emplace_back(pt);
            }
        }
    });
```


0.018506663837638375, 총 Hz는 약 32.0 Hz로 오히려 느려짐!

---
for_each test?

python3 calc_time.py
Mean time: 
[1.35000000e+02 1.78156232e-02 9.74399181e+01 9.13463236e+01
 9.68828310e+01 9.13463236e+01]
Hz:  56.13050894278104
Mean time: 
0.004432473284132842
> 
> python3 calc_time.py
Mean time: 
[1.35000000e+02 1.81374760e-02 9.74399181e+01 9.13463236e+01
 9.68828310e+01 9.13463236e+01]
Hz:  55.13446298626156
Mean time: 
0.004611111771217712
> python3 calc_time.py
> 
> 
> python3 calc_time.py
Mean time: 
[1.35000000e+02 1.50958550e-02 9.74399181e+01 9.13463236e+01
 9.68828310e+01 9.13463236e+01]
Hz:  66.2433496626857
Mean time: 
0.003960348228782287
> 
속도가 빨라질 때도 있고 느려질 때도 있음?

---

python3 calc_time.py
Mean time: 
[1.35000000e+02 1.78851716e-02 9.74399181e+01 9.13463236e+01
 9.68828310e+01 9.13463236e+01]
Hz:  55.912239653476156
Mean time: 
0.0043353101107011065
> python3 calc_time.py
Mean time: 
[1.35000000e+02 2.27533376e-02 9.74399181e+01 9.13463236e+01
 9.68828310e+01 9.13463236e+01]
Hz:  43.94959613807925
Mean time: 
0.005798643394833949
> 
> python3 calc_time.py
Mean time: 
[1.35000000e+02 1.47000391e-02 9.74399181e+01 9.13463236e+01
 9.68828310e+01 9.13463236e+01]
Hz:  68.02702987511185
Mean time: 
0.0036656171586715866
> python3 calc_time.py
Mean time: 
[1.35000000e+02 1.52904048e-02 9.74399181e+01 9.13463236e+01
 9.68828310e+01 9.13463236e+01]
Hz:  65.40049222196293
Mean time: 
0.00394320667896679
 
std::for_each 함수는 단일 스레드 또는 병렬 처리를 선택할 때 라이브러리 구현에 따라 다르게 동작할 수 있습니다. 예를 들어, 라이브러리가 현재 시스템의 CPU 코어 수 등을 고려하여 자동으로 단일 스레드 또는 병렬 처리를 선택할 수 있습니다. 때문에 실행할 때마다 시스템의 상황에 따라 다르게 동작하는 것을 볼 수 있습니다.

또한 병렬 처리를 위해서는 스레드 간의 작업 분배 및 스케줄링이 필요하며, 이 과정은 실행할 때마다 다를 수 있습니다. 때문에 속도의 차이가 발생할 수 있습니다.

마지막으로, 시스템의 다른 프로세스나 스레드들의 영향도 속도에 영향을 미칠 수 있습니다. 시스템 리소스가 현재 실행 중인 프로세스나 스레드들에 의해 사용되고 있을 경우, 병렬 처리 속도가 예측하지 못한 방식으로 변할 수 있습니다.

---

auto zone = ConcentricZoneModel_[k];
auto &zone = ConcentricZoneModel_[k];

> python3 calc_time.py
Mean time: 
[1.35000000e+02 1.40829520e-02 9.74399181e+01 9.13463236e+01
 9.68828310e+01 9.13463236e+01]
Hz:  71.00783968473566
Mean time: 
0.00393244889298893
> python3 calc_time.py
Mean time: 
[1.35000000e+02 1.38032059e-02 9.74399181e+01 9.13463236e+01
 9.68828310e+01 9.13463236e+01]
Hz:  72.44693783101029
Mean time: 
0.003856644981549816

python3 calc_time.py
Mean time: 
[1.35000000e+02 1.36652900e-02 9.74399181e+01 9.13463236e+01
 9.68828310e+01 9.13463236e+01]
Hz:  73.17810286497401
Mean time: 
0.0038003691143911433

유의미하게 빨라짐!

---

insert make_move_iterator했을 때
- 생성 후 emplace_back보다 빠르지 않을까 예상함!
> python3 calc_time.py
Mean time: 
[1.35000000e+02 1.38283004e-02 9.74399181e+01 9.13463236e+01
 9.68828310e+01 9.13463236e+01]
Hz:  72.31546707225947
Mean time: 
0.003821524317343174
> 
> python3 calc_time.py
Mean time: 
[1.35000000e+02 1.34980934e-02 9.74399181e+01 9.13463236e+01
 9.68828310e+01 9.13463236e+01]
Hz:  74.08453723667901
Mean time: 
0.003692722915129152
> 
> python3 calc_time.py
Mean time: 
[1.35000000e+02 1.40419535e-02 9.74399181e+01 9.13463236e+01
 9.68828310e+01 9.13463236e+01]
Hz:  71.21516244914358
Mean time: 
0.0039314563099631
> 
python3 calc_time.py
Mean time: 
[1.35000000e+02 1.42424474e-02 9.74399181e+01 9.13463236e+01
 9.68828310e+01 9.13463236e+01]
Hz:  70.21265171098325
Mean time: 
0.003955806273062731

뭐 그닥 좋아지진 않는듯?

---

parallel_for 과 block_range

Case 1

int num_patches = patch_indices_.size();

tbb::parallel_for(0, num_patches, [&](int i) {

Case 2
