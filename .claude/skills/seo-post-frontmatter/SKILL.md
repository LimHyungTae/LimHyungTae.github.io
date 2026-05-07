---
name: seo-post-frontmatter
description: Apply SEO frontmatter (description, image, permalink, redirect_from, alt text) to a Jekyll _posts/*.md file so individual posts rank well on Google. Use this whenever the user creates a new post in `_posts/` and asks for SEO, "검색 잘되게", "frontmatter 채워줘", or otherwise wants the same treatment that was applied site-wide on 2026-05-07.
---

# Apply SEO frontmatter to a Jekyll post

This skill is the canonical "make this post Google-friendly" workflow for the repo. It's the same procedure that was applied site-wide on 2026-05-07; following it keeps every post's metadata shape consistent.

## When to use

- User just wrote a new post under `_posts/` and asks for SEO / search optimization / frontmatter polishing.
- User says things like "이 글도 SEO 적용해줘", "permalink 만들어줘", "검색 잘 되게".
- A post is missing one or more of: `description`, `image`, `permalink`, `redirect_from`.

If the user wants you to apply this to MANY files at once (e.g., a series), still apply the rules below per-file; consider dispatching subagents for parallelism.

## Inputs

- Path(s) to one or more `_posts/*.md` files. If none provided, ask which post.

## Steps for EACH file

1. **Read the file.**
2. **Locate existing YAML frontmatter** (between `---` delimiters at the top).
3. **Add these new keys** without removing or modifying existing keys:
    ```yaml
    description: <Korean 1-2 sentences, 80-150 chars>
    image: <first /img/... path in body, OMIT this key entirely if no image>
    permalink: /YYYY/MM/DD/<english-slug>/
    redirect_from:
      - '/YYYY-MM-DD-<filename-stem>/'
    ```
4. **Improve image alt text in the body**: replace any `![](path)` (empty alt) with `![<short Korean alt>](path)`. Don't change images that already have alt text. Don't modify other markdown.
5. **Write the file back.**

Don't remove or change any existing frontmatter keys. Don't reflow code blocks, math, or tables.

## Rules per key

### `description`
1-2 Korean sentences, ~80-150 characters. Specific, with keywords from post topic. Match the existing voice of the post (`-한다/-이다` plain or `-합니다` formal).

GOOD:
- `"PCL VoxelGrid filter로 pointcloud를 voxel 단위로 다운샘플링해 메모리와 연산량을 줄이는 방법과 setLeafSize 사용법을 정리한다."`
- `"GTSAM의 BetweenFactor를 SLAM 입문자가 이해할 수 있도록, factor graph 위에서 두 pose 사이의 상대 변환을 어떻게 표현하는지 차근차근 풀어본다."`

BAD (too generic):
- `"PCL 사용법을 설명한다."`
- `"이 글에서는 ~을 다룬다."`

### `image`
Scan the body for the FIRST `/img/<filename>.<ext>` reference (whether in `![alt](path)` or `<img src="path">` form). Use that path. If post has NO `/img/` reference, OMIT the `image:` key entirely (don't write empty value).

### `permalink`
Format: `/YYYY/MM/DD/<english-slug>/`. The date comes from the filename's `YYYY-MM-DD-` prefix or, if the filename lacks one, from the `date:` field in frontmatter.

Slug rules:
- lowercase, hyphens between tokens, no underscores/spaces/dots/parens
- English words verbatim, lowercased: "Ceres Solver" → `ceres-solver`
- Translate Korean to natural English keywords:
  - 쉬운 설명 → drop or `easy-explanation`
  - 한눈에 보기 → `overview`
  - 사용법 → `usage` / `tutorial`
  - 유도 → `derivation`
  - 이해하기 → drop or `understanding`
  - 기본 사용법 → `basic-usage`
  - 정리 → `summary`
  - 활용한 → `with` / `using`
  - 설치하는 법 → `installation`
  - 에러 해결 / 해결 방법 → `error-fix` / `fix`
  - 분석 → `analysis`
  - 세팅 → `setup`
  - 깊게 이해하기 → `deep-dive` or drop "깊게"
- Chapter/part numbers → zero-padded 2-digit: "Lec 1" → `lec-01`, "Tutorial 8" → `tutorial-08`
- Keep slug compact (3-7 hyphenated tokens is the sweet spot)

Examples:
| Filename | Permalink |
|----------|-----------|
| `2021-09-12-ROS Point Cloud Library (PCL) - 5. Voxelization.md` | `/2021/09/12/pcl-tutorial-05-voxelization/` |
| `2024-12-01-GTSAM Tutorial 1. SLAM을 위한 Between Factor 쉽게 이해하기.md` | `/2024/12/01/gtsam-tutorial-01-between-factor/` |
| `2024-01-01-Modern C++ for Robotics 6. std::move() 쉬운 설명.md` | `/2024/01/01/modern-cpp-robotics-06-std-move/` |
| `2026-05-07-Ceres Solver for Graph SLAM - 4. PoseGraph3dErrorTerm 깊게 이해하기.md` | `/2026/05/07/ceres-graph-slam-04-posegraph3d-error-term/` |
| `2022-04-01-IMU Preintegration (Easy) - 2. Preliminaries (1) Keyframe.md` | `/2022/04/01/imu-preintegration-02a-keyframe/` |

### `redirect_from`
The OLD default URL the post served at. Default permalink in `_config.yml` is `/:year-:month-:day-:title/`, so for a file with date prefix the URL is `/<filename without .md>/`:

| Filename | redirect_from |
|----------|---------------|
| `2021-09-12-ROS Point Cloud Library (PCL) - 5. Voxelization.md` | `'/2021-09-12-ROS Point Cloud Library (PCL) - 5. Voxelization/'` |
| `2024-12-01-GTSAM Tutorial 1. SLAM을 위한 Between Factor 쉽게 이해하기.md` | `'/2024-12-01-GTSAM Tutorial 1. SLAM을 위한 Between Factor 쉽게 이해하기/'` |

For files WITHOUT a date prefix (e.g., `LIO-SAM Line by Line - 1. Introduction.md`), use the `date:` from frontmatter and the filename stem:

```yaml
redirect_from:
  - '/2022-04-07-LIO-SAM Line by Line - 1. Introduction/'
```

ALWAYS single-quote the URL string — parens and special chars need quoting.

### Image alt text
Find `![](path)` patterns and add a short Korean alt:
- `![](/img/voxel_zoom.png)` → `![voxel 결과 확대](/img/voxel_zoom.png)`
- `![](/img/icp_result.png)` → `![ICP registration 결과](/img/icp_result.png)`

Use 2-4 Korean words. If you genuinely can't tell from context, use a post-topic-related generic phrase. Skip images that already have non-empty alt text.

## Sub-part / multi-file series

For posts with `(1)`, `(2)`, `(3)` sub-parts under the same major chapter (e.g., `Lec 02 (1)`, `Lec 02 (2)`), use `02a`, `02b`, `02c` in the permalink slug to keep them sortable but distinct:

```
imu-preintegration-02a-keyframe
imu-preintegration-02b-rotation-uncertainty
```

## Tone and style notes

- Don't introduce em-dashes (`—`) — Korean prose uses commas/colons/periods instead. (This was a separate cleanup pass on 2026-05-07.)
- Keep description concise; if you find yourself writing more than 2 sentences, trim.
- Match the existing voice (plain `-한다` vs formal `-합니다`) so the description doesn't clash with the post body.

## After applying

- Suggest the user commit with a message like `Add SEO frontmatter to <post-slug>`.
- If the post is part of a series and you've added a new post, remind the user to verify the relevant `_includes/post_links_*.html` already picks it up (the includes filter on `title contains 'X'`, so usually no edit is needed).

## Related

- `slam-add-lecture` — full lecture publish workflow on `/slam/`.
- `slam-lecture-entry` — just the slam.md edit.
