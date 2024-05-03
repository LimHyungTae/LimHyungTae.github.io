---
layout: post
title: Modern C++ for Robotics (3) TBB의 Segmentation fault 해결하는 방법 
subtitle: 1주일 동안 디버깅을 하며 알아낸 것들
tags: [C++, Eigen, Robotics]
comments: true
---

새 해에 나는 TBB를 좀더 자유롭게 사용할 수 있도록 여기저기에 연습삼아 적용해보고 있다.
대표적인 예시로 [Patchwork](https://github.com/LimHyungTae/patchwork)에 TBB를 적용시켜보니, 속도가 50 Hz에서 약 100 Hz까지 점프하는 것을 볼 수 있었다.
Robotics 분야에서는 센서의 frame FPS에 맞게 동작하는 것이 괴애애애앵장히 중요한데, 이러한 니즈를 TBB를 통해 잘 충족시킬 수 있지 않을까 싶다.

그래서 최근에 feature matching하는 부분에 TBB를 적용시키려 노력했는데, 
계속 Segmentation fault error가 나서, gdb를 써서 여기저기 찍어보며 한 1주일 정도 날리며 깨달은 것들을 공유하고자 한다.

독자들도 어디가 틀렸는지 한번 예측해보길!

### 문제의 코드

```cpp
// Original code snippet (with segmentation fault)

std::vector<std::vector<int>> corres_K(nPtj);
std::<<vector<std::vector<int>> corres_K2(nPti);
std::vector<std::vector<float>> dis_j(nPtj);
std::vector<std::vector<float>> dis_i(nPti);

std::vector<std::pair<int, int>> corres;
std::vector<std::pair<int, int>> corres_ij;
std::vector<std::pair<int, int>> corres_ji;

std::vector<int> i_to_j_multi_flann(nPti, -1);
std::vector<int> j_to_i_multi_flann(nPtj, -1);
std::vector<std::pair<int, int>> corres_multi_flann;
corres_multi_flann.reserve(nPti / 3); // heuristic
std::vector<int> j_idx(nPtj);
std::iota(j_idx.begin(), j_idx.end(), 0);

tbb::parallel_for(0, nPtj, [&](int j) {
    searchKDTree(feature_tree_i, features_[fj][j], corres_K[j], dis_j[j], 1);
    //  corres_K[j][0] is j -> 'i'
    if (corres_K[j][0] >= 0 && corres_K[j][0] < nPti) {
        if (i_to_j_multi_flann[corres_K[j][0]] == -1) {
            searchKDTree(feature_tree_j,
                         features_[fi][corres_K[j][0]],
                         corres_K2[corres_K[j][0]],
                         dis_i[corres_K[j][0]],
                         1);
            i_to_j_multi_flann[corres_K[j][0]] = corres_K2[corres_K[j][0]][0];
        }
        j_to_i_multi_flann[j] = corres_K[j][0];
//            std::cout << "(" << j << " / " << nPtj << ", " << corres_K[j][0] << " / " << nPti << ") " << std::endl;
    }
});

for(int j = 0; j < nPtj; j++){
    int ji = j_to_i_multi_flann[j];
    if(j == i_to_j_multi_flann[ji]){
        corres_multi_flann.emplace_back(std::pair<int, int>(ji, j));
    }
}
corres = corres_multi_flann;
.
.
.
// searchKDTree function
void searchKDTree(const KDTree& tree, const T& input, std::vector<int>& indices,
                           std::vector<float>& dists, int nn) {
  int rows_t = 1;
  int dim = input.size();

  std::vector<float> query;
  query.resize(rows_t * dim);
  for (int i = 0; i < dim; i++)
    query[i] = input(i);
  flann::Matrix<float> query_mat(&query[0], rows_t, dim);

  indices.resize(rows_t * nn);
  dists.resize(rows_t * nn);
  flann::Matrix<int> indices_mat(&indices[0], rows_t, nn);
  flann::Matrix<float> dists_mat(&dists[0], rows_t, nn);


  tree.knnSearch(query_mat, indices_mat, dists_mat, nn, flann::SearchParams(128));
}
```

## 원인 

틀린 부분은 `searchKDTree` 함수에서 `query`와 `indices`를 `resize`하는 부분이다.

```cpp
indices.resize(rows_t * nn);
dists.resize(rows_t * nn);
```

저 부분이 문제가 될 것이라고는 전혀 예기치 못했다. 
왜냐하면 처음 변수를 선언할 때 충분히 thread-safe하게 correspondences와 distances에 대한 vector의 크기를 데이터의 크기에 맞게 선언했다고 생각했기 때문이다.

```cpp
std::vector<std::vector<int>> corres_K(nPtj);
std::<<vector<std::vector<int>> corres_K2(nPti);
std::vector<std::vector<float>> dis_j(nPtj);
std::vector<std::vector<float>> dis_i(nPti); 
```

하지만 **각 요소가 비어있는 것을 간과했다.** 이 현상을 해결하면서 알게된 사실인데, `resize`를 하게 되면,
 
* Case A. 원래 벡터의 크기가 resize된 값보다 크다면, 원래 벡터의 크기는 메모리 주소를 유지하면서 resize된 값으로 줄어들게 된다.
* Case B. (**문제가 되는 경우**) 원래 벡터의 크기가 resize된 값보다 작다면, 벡터는 새로운, 더 큰 메모리 블록을 할당받아야 하기 때문에 이 과정에서 **벡터의 메모리 주소가 바뀔 수 있다.** 그 후, 새로운 메모리 블록으로 요소들이 이동되고, 기존의 메모리 블록은 해제된다.

Case B를 토대로 backtrace해보면, `rows_t * nn`이 1로, i.e. resize(1), 그리 크지 않은 수이다보니 resize를 해도 별 문제가 생기지 않겠거니 한 것이 잘못이었던 것이다.
즉, 처음에 변수를 선언할 때 empty vector<int> 나 vector<float>으로 initialization을 했다보니, 해당 vector의 크기가 커지게 되면서(0 → 1) 예기치 못하게 memory 주소가 옮겨질 수 있다.
그 결과, 해당 부분에서 Segmentation fault가 발생하게 되는 것이다!

(디버깅을 할 때는 cmake 빌드할 때 `cmake -DCMAKE_BUILD_TYPE=Debug ..`를 한 후에 gdb로 binary file을 실행하면, segfault가 났을 때 어디에서 발생하는지 자세히 알려준다. GBD 짱짱!)

 

## 해결책

따라서 위의 코드를 아래와 같이 두가지 수정해주면 된다.

1. Corresdponecnes와 distances의 각 요소에 해당하는 vector의 크기가 1이라는 것(가장 가까운 1개에 대해서 return 받기 때문)을 알기 때문에, 그냥 초기값을 0이고 크기가 1인 vector로 manual하게 선언해준다.

```cpp
// Before 
std::vector<std::vector<int>> corres_K(nPtj);
std::<<vector<std::vector<int>> corres_K2(nPti);
std::vector<std::vector<float>> dis_j(nPtj);
std::vector<std::vector<float>> dis_i(nPti);
// After
std::vector<std::vector<int>> corres_K(nPtj, std::vector<int>(1, 0));
std::vector<std::vector<int>> corres_K2(nPti, std::vector<int>(1, 0));
std::vector<std::vector<float>> dis_j(nPtj, std::vector<float>(1, 0.0));
std::vector<std::vector<float>> dis_i(nPti, std::vector<float>(1, 0.0));
```

2. 그 후, `resize`하는 부분을 제거해준다 (크기가 1로 고정이기 때문에 resize를 해줄 필요가 없) 

```cpp
void searchKDTree(const KDTree& tree, const T& input, std::vector<int>& indices,
                           std::vector<float>& dists, int nn) {
  int rows_t = 1;
  int dim = input.size();

  std::vector<float> query;
  query.resize(rows_t * dim);
  for (int i = 0; i < dim; i++)
    query[i] = input(i);
  flann::Matrix<float> query_mat(&query[0], rows_t, dim);

  // HT: `resize` is commented
  // indices.resize(rows_t * nn);
  // dists.resize(rows_t * nn);
  flann::Matrix<int> indices_mat(&indices[0], rows_t, nn);
  flann::Matrix<float> dists_mat(&dists[0], rows_t, nn);


  tree.knnSearch(query_mat, indices_mat, dists_mat, nn, flann::SearchParams(128));
}
```

## 결론

* `resize`를 할 때는, **기존의 메모리 주소를 유지하면서 크기를 줄이는 경우**에만 thread-safe하다. 그냥 TBB 쓸때는 메모리를 건드릴 생각을 하지말자!
* 이 Segmentation fault 때문에 근 3개월 헤맸는데, 역시 코드는 거짓말을 하지 않는다...

---

Robotics 연구자/개발자를 위한 Modern C++ 시리즈입니다.

{% include post_links_tips_modern_cpp.html %}
