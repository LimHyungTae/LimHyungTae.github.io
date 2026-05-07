---
layout: page
title: 모두를 위한 SLAM
subtitle: SLAM for Everyone
permalink: /slam/
---

<style>
.slam-intro {
    margin-bottom: 30px;
    line-height: 1.8;
    color: #555;
}

.slam-section {
    margin-bottom: 40px;
}

.slam-section h3 {
    border-bottom: 2px solid #1B2C8F;
    padding-bottom: 8px;
    margin-bottom: 16px;
    color: #2c3e50;
}

.lecture-list {
    list-style: none;
    padding-left: 0;
}

.lecture-list li {
    padding: 8px 0;
    border-bottom: 1px solid #f0f0f0;
    display: flex;
    align-items: center;
    flex-wrap: wrap;
    gap: 8px;
}

.lecture-list li:last-child {
    border-bottom: none;
}

.lecture-title {
    flex: 1;
    min-width: 200px;
    font-size: 15px;
}

.lecture-num {
    font-weight: bold;
    color: #1B2C8F;
    margin-right: 4px;
}

.lecture-links {
    display: flex;
    gap: 6px;
    flex-shrink: 0;
}

.lecture-links a {
    display: inline-flex;
    align-items: center;
    gap: 4px;
    padding: 3px 10px;
    border-radius: 4px;
    font-size: 13px;
    font-weight: 500;
    text-decoration: none;
    transition: all 0.2s ease;
}

.btn-video {
    background-color: #1B2C8F;
    color: white !important;
    border: 1px solid #1B2C8F;
}

.btn-video:hover {
    background-color: #142373;
    border-color: #142373;
    text-decoration: none !important;
    color: white !important;
}

/* Placeholder pill for lectures whose video isn't recorded yet.
   Kept on .btn-video so the language toggle (which hides .btn-video in English mode)
   continues to work without further wiring. The size/padding rules in
   .lecture-links a above don't apply to <span>, so re-declare them here. */
.btn-video.tbu,
.btn-video.tbu:hover {
    display: inline-flex;
    align-items: center;
    padding: 3px 10px;
    font-size: 13px;
    font-weight: 500;
    line-height: 1.4;
    border-radius: 4px;
    background-color: #f0f0f0;
    color: #999 !important;
    border: 1px solid #e0e0e0;
    cursor: default;
    pointer-events: none;
    text-decoration: none !important;
}

.btn-slide {
    background-color: #f8f9fa;
    color: #495057 !important;
    border: 1px solid #dee2e6;
}

.btn-slide:hover {
    background-color: #e9ecef;
    border-color: #adb5bd;
    text-decoration: none !important;
    color: #212529 !important;
}

.btn-code {
    background-color: #f8f9fa;
    color: #495057 !important;
    border: 1px solid #dee2e6;
}

.btn-code:hover {
    background-color: #e9ecef;
    border-color: #adb5bd;
    text-decoration: none !important;
    color: #212529 !important;
}

.tba-badge {
    display: inline-block;
    padding: 3px 10px;
    background-color: #f0f0f0;
    color: #999;
    border-radius: 4px;
    font-size: 12px;
    font-weight: 500;
}

/* --- Language toggle --- */
.lang-toggle {
    display: inline-flex;
    border: 1px solid #dee2e6;
    border-radius: 6px;
    overflow: hidden;
    margin-bottom: 20px;
}

.lang-toggle button {
    padding: 6px 14px;
    background: #f8f9fa;
    border: none;
    cursor: pointer;
    font-size: 13px;
    font-weight: 600;
    color: #495057;
    transition: all 0.15s ease;
}

.lang-toggle button + button {
    border-left: 1px solid #dee2e6;
}

.lang-toggle button:hover {
    background: #e9ecef;
}

.lang-toggle button.active {
    background: #1B2C8F;
    color: #fff;
}

/* Show only the active language.
   Each translatable element is written twice — once with .lang-ko, once with .lang-en —
   sitting next to each other in the markup. CSS hides whichever isn't currently selected.
   To add new content: paste both language variants side-by-side; no JS edits needed. */
body[data-lang="ko"] .lang-en { display: none !important; }
body[data-lang="en"] .lang-ko { display: none !important; }
body[data-lang="en"] .btn-video { display: none !important; }

@media (max-width: 576px) {
    .lecture-list li {
        flex-direction: column;
        align-items: flex-start;
    }
    .lecture-links {
        margin-top: 4px;
    }
}

/* --- Inline PDF viewer (PDF.js) ---
   The "View" toggle button is injected next to each .btn-slide via JS, and
   the viewer panel slots in directly under the host <li>. PDF.js itself is
   lazy-loaded — nothing fetches until the user clicks View. */
.btn-view-toggle {
    display: inline-flex;
    align-items: center;
    gap: 4px;
    padding: 3px 10px;
    border-radius: 4px;
    font-size: 13px;
    font-weight: 500;
    line-height: 1.4;
    cursor: pointer;
    background-color: #f8f9fa;
    color: #495057;
    border: 1px solid #dee2e6;
    font-family: inherit;
    transition: all 0.2s ease;
}

.btn-view-toggle:hover {
    background-color: #e9ecef;
    border-color: #adb5bd;
    color: #212529;
}

.btn-view-toggle.active {
    background-color: #1B2C8F;
    color: #fff;
    border-color: #1B2C8F;
}

.pdf-inline-viewer {
    width: 100%;
    background: #f8f9fa;
    border: 1px solid #dee2e6;
    border-radius: 6px;
    margin: 8px 0 16px;
    overflow: hidden;
    box-shadow: 0 1px 3px rgba(0,0,0,0.05);
}

.pdf-inline-viewer-toolbar {
    display: flex;
    align-items: center;
    gap: 12px;
    padding: 8px 12px;
    border-bottom: 1px solid #dee2e6;
    background: #fff;
    font-size: 13px;
    color: #495057;
    flex-wrap: wrap;
}

.pdf-inline-viewer-toolbar .pdf-status-label {
    font-weight: 500;
    color: #495057;
}

.pdf-inline-viewer-toolbar .pdf-spacer {
    flex: 1;
}

.pdf-inline-viewer-toolbar .pdf-zoom-group {
    display: inline-flex;
    align-items: center;
    border: 1px solid #dee2e6;
    border-radius: 4px;
    overflow: hidden;
    background: #fff;
}

.pdf-inline-viewer-toolbar button {
    font: inherit;
    font-size: 13px;
    padding: 3px 10px;
    border: none;
    background: #fff;
    color: #495057;
    cursor: pointer;
    line-height: 1.4;
}

.pdf-inline-viewer-toolbar button:hover {
    background: #e9ecef;
    color: #212529;
}

.pdf-inline-viewer-toolbar .pdf-zoom-label {
    min-width: 3em;
    text-align: center;
    font-size: 12px;
    color: #495057;
    user-select: none;
    padding: 3px 4px;
    line-height: 1.4;
    background: #fff;
}

.pdf-inline-viewer-toolbar .pdf-open-new {
    border: 1px solid #dee2e6;
    border-radius: 4px;
    text-decoration: none !important;
    color: #495057 !important;
    padding: 3px 10px;
    font-size: 13px;
    line-height: 1.4;
    background: #fff;
}

.pdf-inline-viewer-toolbar .pdf-open-new:hover {
    background: #e9ecef;
    color: #212529 !important;
    text-decoration: none !important;
}

.pdf-inline-viewer-pages {
    padding: 12px;
    max-height: min(80vh, 720px);
    overflow-y: auto;
    background: #e8eaee;
}

.pdf-inline-viewer .page-wrap {
    position: relative;
    margin: 0 auto 8px;
    background: #fff;
    border-radius: 4px;
    box-shadow: 0 2px 14px rgba(0,0,0,0.13);
    max-width: 800px;
    overflow: hidden;
}

.pdf-inline-viewer .page-wrap canvas {
    width: 100%;
    display: block;
    background: #fff;
}

.pdf-inline-viewer .page-num {
    position: absolute;
    bottom: 4px;
    right: 6px;
    font-size: 11px;
    background: rgba(0,0,0,0.4);
    color: #fff;
    padding: 2px 6px;
    border-radius: 3px;
    pointer-events: none;
}

.pdf-inline-viewer .pdf-status {
    text-align: center;
    padding: 30px 20px;
    color: #8a9ab0;
    font-size: 14px;
}
</style>

<div class="lang-toggle" role="group" aria-label="Language toggle">
  <button type="button" data-lang="en" class="active">English</button>
  <button type="button" data-lang="ko">Korean (한국어)</button>
</div>

<div class="slam-intro">
<span class="lang-ko"> SLAM(Simultaneous Localization and Mapping)을 처음 접하는 분들을 위한 강의 시리즈입니다.
제가 처음 SLAM을 공부할 때, 개념을 직관적으로 설명해주는 자료를 찾기 어려워 꽤 오랜 시간 헤맸던 기억이 있습니다.
그때 “이걸 누군가 쉽게 풀어서 설명해줬다면 얼마나 좋았을까”라는 생각이 들었고,
그 경험을 바탕으로 이 강의를 만들게 되었습니다.
학부 3-4학년 수준에서도 이해할 수 있도록 직관과 유추를 중심으로,
SLAM의 핵심 아이디어부터 전체 구조까지 차근차근 설명합니다.
강의 자료는 순차적으로 업데이트될 예정입니다. </span>
<span class="lang-en"> This lecture series is designed for those who are new to SLAM (Simultaneous Localization and Mapping).
When I first started learning SLAM, I struggled to find materials that explained the concepts in an intuitive way,
and I often felt lost trying to connect the math with the bigger picture.
I remember wishing there were a resource that could guide me more clearly, and this series grew out of that experience.
With that in mind, these lectures aim to explain SLAM in a simple and intuitive way,
covering the key ideas and the overall pipeline step by step.
The materials will be updated sequentially. </span>
</div>

---

### <span class="lang-ko">강의 일정</span><span class="lang-en">Schedule</span>

<div class="slam-section">
<h3><span class="lang-ko">Lecture 01: Introduction</span><span class="lang-en">Lecture 01: Introduction</span></h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">1-1.</span>
      <span class="lang-ko">SLAM이란 무엇인가?</span>
      <span class="lang-en">What is SLAM?</span>
    </span>
    <span class="lecture-links">
      <a href="https://youtu.be/gQtpTGrgtqE" class="btn-video">Video</a>
      <a href="/slam_slides/Lec01_SLAM_for_everyone.pdf" class="btn-slide">Slide</a>
    </span>
  </li>
</ul>
</div>

<div class="slam-section">
<h3><span class="lang-ko">Lecture 02: Categories &amp; Terminology of Robot Navigation</span><span class="lang-en">Lecture 02: Categories &amp; Terminology of Robot Navigation</span></h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">2-1.</span>
      <span class="lang-ko">Robot navigation 구성 요소</span>
      <span class="lang-en">Components of robot navigation</span>
    </span>
    <span class="lecture-links">
      <a href="https://youtu.be/DpKo0N-UjXs" class="btn-video">Video</a>
      <a href="/slam_slides/Lec02_SLAM_for_everyone.pdf" class="btn-slide">Slide</a>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">2-2.</span>
      <span class="lang-ko">Perception vs. Cognition</span>
      <span class="lang-en">Perception vs. cognition</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">2-3.</span>
      <span class="lang-ko">Localization, Mapping, SLAM 용어</span>
      <span class="lang-en">Terminology: localization, mapping, SLAM</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">2-4.</span>
      <span class="lang-ko">결론: SLAM ⊂ Robot navigation</span>
      <span class="lang-en">Conclusion: SLAM ⊂ robot navigation</span>
    </span>
  </li>
</ul>
</div>

<div class="slam-section">
<h3><span class="lang-ko">Lecture 03: State, Measurement, and Estimation</span><span class="lang-en">Lecture 03: State, Measurement, and Estimation</span></h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">3-1.</span>
      <span class="lang-ko">Introduction: 키 재기 예시로 시작</span>
      <span class="lang-en">Introduction: starting with a height measurement example</span>
    </span>
    <span class="lecture-links">
      <a href="https://youtu.be/JIUWrz2161A" class="btn-video">Video</a>
      <a href="/slam_slides/Lec03_SLAM_for_everyone.pdf" class="btn-slide">Slide</a>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">3-2.</span>
      <span class="lang-ko">State와 measurement 정의</span>
      <span class="lang-en">Definition of state and measurement</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">3-3.</span>
      <span class="lang-ko">Calculation vs. Estimation</span>
      <span class="lang-en">Calculation vs. estimation</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">3-4.</span>
      <span class="lang-ko">Example A – MLE: 평균 기반 키 추정</span>
      <span class="lang-en">Example A – MLE: height estimation via average</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">3-5.</span>
      <span class="lang-ko">Example B – MAP: 사전 지식 활용 키 추정</span>
      <span class="lang-en">Example B – MAP: height estimation using prior</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">3-6.</span>
      <span class="lang-ko">결론: Estimation = Weighted Average + Uncertainty</span>
      <span class="lang-en">Conclusion: estimation = weighted average + uncertainty</span>
    </span>
  </li>
</ul>
</div>

<div class="slam-section">
<h3><span class="lang-ko">Lecture 04: Maximum Likelihood Estimation (MLE), Maximum A Posteriori (MAP)</span><span class="lang-en">Lecture 04: Maximum Likelihood Estimation (MLE), Maximum A Posteriori (MAP)</span></h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">4-1.</span>
      <span class="lang-ko">Introduction: MLE와 MAP 개요</span>
      <span class="lang-en">Introduction: overview of MLE and MAP</span>
    </span>
    <span class="lecture-links">
      <a href="https://youtu.be/y2PqPEPTK1A?si=pOsJ2XuOFQ1d3qgg" class="btn-video">Video</a>
      <a href="/slam_slides/Lec04_SLAM_for_everyone.pdf" class="btn-slide">Slide</a>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">4-2.</span>
      <span class="lang-ko">Probability vs. Likelihood</span>
      <span class="lang-en">Probability vs. likelihood</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">4-3.</span>
      <span class="lang-ko">확률 모델과 MLE: Gaussian noise → least squares</span>
      <span class="lang-en">Probabilistic model and MLE: Gaussian noise → least squares</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">4-4.</span>
      <span class="lang-ko">Maximum a Posteriori (MAP): prior 활용</span>
      <span class="lang-en">Maximum a posteriori (MAP): incorporating prior</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">4-5.</span>
      <span class="lang-ko">Weighted Least Squares와 Graph SLAM 연결</span>
      <span class="lang-en">Weighted least squares and connection to Graph SLAM</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">4-6.</span>
      <span class="lang-ko">결론: State estimation = weighted averages</span>
      <span class="lang-en">Conclusion: state estimation is a game of weighted averages</span>
    </span>
  </li>
</ul>
</div>

<div class="slam-section">
<h3><span class="lang-ko">Lecture 05: Filtering vs Smoothing (1) – Kalman Filter from a MAP Perspective</span><span class="lang-en">Lecture 05: Filtering vs Smoothing (1) – Kalman Filter from a MAP Perspective</span></h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">5-1.</span>
      <span class="lang-ko">Filtering vs. Smoothing: sequential vs. batch 비교</span>
      <span class="lang-en">Filtering vs. smoothing: sequential vs. batch</span>
    </span>
    <span class="lecture-links">
      <a href="https://youtu.be/LDeWEYFJacw" class="btn-video">Video</a>
      <a href="/slam_slides/Lec05_SLAM_for_everyone.pdf" class="btn-slide">Slide</a>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">5-2.</span>
      <span class="lang-ko">Kalman Filter in 1D: prediction &amp; update step</span>
      <span class="lang-en">Kalman filter in 1D: prediction &amp; update steps</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">5-3.</span>
      <span class="lang-ko">Kalman Filter for Multivariable Systems: 행렬 형태로의 확장</span>
      <span class="lang-en">Kalman filter for multivariable systems: matrix form</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">5-4.</span>
      <span class="lang-ko">결론: KF update = MAP</span>
      <span class="lang-en">Conclusion: KF update = MAP</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">Supp.</span>
      <span class="lang-ko">수학 보충 자료</span>
      <span class="lang-en">Math supplementary</span>
    </span>
    <span class="lecture-links">
      <a href="/slam_slides/Lec05_math_supplementary.pdf" class="btn-slide">Slide</a>
    </span>
  </li>
</ul>
</div>

<div class="slam-section">
<h3><span class="lang-ko">Lecture 06: Filtering vs Smoothing (2) – Graph Optimization from a MAP Perspective</span><span class="lang-en">Lecture 06: Filtering vs Smoothing (2) – Graph Optimization from a MAP Perspective</span></h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">6-1.</span>
      <span class="lang-ko">Filtering vs. Smoothing 비교 복습</span>
      <span class="lang-en">Filtering vs. smoothing: recap</span>
    </span>
    <span class="lecture-links">
      <a href="https://youtu.be/_fSSED4vtdI" class="btn-video">Video</a>
      <a href="/slam_slides/Lec06_SLAM_for_everyone.pdf" class="btn-slide">Slide</a>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">6-2.</span>
      <span class="lang-ko">Graph SLAM: Factor Graph 수식 꼴</span>
      <span class="lang-en">Graph SLAM: factor graph-based formulation</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">6-3.</span>
      <span class="lang-ko">Graph-Based SLAM Framework: Front-end &amp; Back-end</span>
      <span class="lang-en">Graph-based SLAM framework: front-end &amp; back-end</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">6-4.</span>
      <span class="lang-ko">Mass-spring 비유를 통한 직관적 이해 </span>
      <span class="lang-en">Intuitive understanding: mass-spring analogy</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">6-5.</span>
      <span class="lang-ko">Example: 2D LiDAR Graph SLAM + Landmarks</span>
      <span class="lang-en">Example: 2D LiDAR Graph SLAM with landmarks</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">6-6.</span>
      <span class="lang-ko">결론: Filtering vs. Smoothing, 어느 쪽이 나은가?</span>
      <span class="lang-en">Conclusion: filtering vs. smoothing — which is better?</span>
    </span>
  </li>
</ul>
</div>

<div class="slam-section">
<h3><span class="lang-ko">Lecture 07: Rotation, and Transformation Matrix</span><span class="lang-en">Lecture 07: Rotation, and Transformation Matrix</span></h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">7-1.</span>
      <span class="lang-ko">What We've Learned: 결국 SLAM = MAP이다!</span>
      <span class="lang-en">What we've learned: SLAM as a MAP problem</span>
    </span>
    <span class="lecture-links">
      <span class="btn-video tbu" aria-label="Video coming soon">Video (TBU)</span>
      <a href="/slam_slides/Lec07_SLAM_for_everyone.pdf" class="btn-slide">Slide</a>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">7-2.</span>
      <span class="lang-ko">Remaining Topics: Pose 표현·최적화·Data association</span>
      <span class="lang-en">Remaining topics: pose representation, optimization, data association</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">7-3.</span>
      <span class="lang-ko">1D vs. 2D/3D: Translation &amp; Rotation의 결합</span>
      <span class="lang-en">1D vs. 2D/3D: coupling of translation and rotation</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">7-4.</span>
      <span class="lang-ko">Rotation Representation: Euler, Angle-Axis, Quaternion, SO(n)</span>
      <span class="lang-en">Rotation representation: Euler, angle-axis, quaternion, SO(n)</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">7-5.</span>
      <span class="lang-ko">Transformation Matrices: SE(2) &amp; SE(3)</span>
      <span class="lang-en">Transformation matrices: SE(2) &amp; SE(3)</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">7-6.</span>
      <span class="lang-ko">결론: Rotation + Translation = SE(n)</span>
      <span class="lang-en">Conclusion: rotation + translation = SE(n)</span>
    </span>
  </li>
</ul>
</div>

<div class="slam-section">
<h3><span class="lang-ko">Lecture 08: Three Practical Tips for Transformations in Code</span><span class="lang-en">Lecture 08: Three Practical Tips for Transformations in Code</span></h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">8-1.</span>
      <span class="lang-ko">SE(3) 복습</span>
      <span class="lang-en">Recap: SE(3)</span>
    </span>
    <span class="lecture-links">
      <span class="btn-video tbu" aria-label="Video coming soon">Video (TBU)</span>
      <a href="/slam_slides/Lec08_SLAM_for_everyone.pdf" class="btn-slide">Slide</a>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">8-2.</span>
      <span class="lang-ko">Tip 1: Transform 방향 명시할 것 (T_cam_lidar 규칙)</span>
      <span class="lang-en">Tip 1: make transform direction explicit (T_cam_lidar convention)</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">8-3.</span>
      <span class="lang-ko">Tip 2: Quaternion 계수 순서 확인 (WXYZ vs. XYZW)</span>
      <span class="lang-en">Tip 2: verify quaternion coefficient order (WXYZ vs. XYZW)</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">8-4.</span>
      <span class="lang-ko">Tip 3: 좌표계 명시적 문서화 (LIO-SAM vs. FAST-LIO 예시)</span>
      <span class="lang-en">Tip 3: document all pose frames explicitly (LIO-SAM vs. FAST-LIO)</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">8-5.</span>
      <span class="lang-ko">결론: 주석에 의도를 explicit하게 표현해두자</span>
      <span class="lang-en">Conclusion: never rely on implicit conventions</span>
    </span>
  </li>
</ul>
</div>

<div class="slam-section">
<h3><span class="lang-ko">Lecture 09: Lie Group and Lie Algebra</span><span class="lang-en">Lecture 09: Lie Group and Lie Algebra</span></h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">9-1.</span>
      <span class="lang-ko">Learning Objectives: SO(3) 위에서의 update 문제</span>
      <span class="lang-en">Learning objectives: the update problem on SO(3)</span>
    </span>
    <span class="lecture-links">
      <span class="btn-video tbu" aria-label="Video coming soon">Video (TBU)</span>
      <a href="/slam_slides/Lec09_SLAM_for_everyone.pdf" class="btn-slide">Slide</a>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">9-2.</span>
      <span class="lang-ko">Intuition: Lie Algebra = SO(3)의 접평면 공간</span>
      <span class="lang-en">Intuition: Lie algebra as the tangent space of SO(3)</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">9-3.</span>
      <span class="lang-ko">Lie Algebra for SO(3): skew-symmetric matrix와 exp map</span>
      <span class="lang-en">Lie algebra for SO(3): skew-symmetric matrix and exponential map</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">9-4.</span>
      <span class="lang-ko">Lie Algebra for SE(3): twist와 SE(3) exp map</span>
      <span class="lang-en">Lie algebra for SE(3): twist and exponential map on SE(3)</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">9-5.</span>
      <span class="lang-ko">Examples/Applications: SLAM 최적화에서의 활용</span>
      <span class="lang-en">Examples/applications: use in SLAM optimization</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">9-6.</span>
      <span class="lang-ko">결론: Lie Algebra로 곡면 위 최적화 가능</span>
      <span class="lang-en">Conclusion: Lie algebra enables optimization on curved spaces</span>
    </span>
  </li>
</ul>
</div>

<div class="slam-section">
<h3><span class="lang-ko">Lecture 10: Non-linearity and Non-linear Least Squares</span><span class="lang-en">Lecture 10: Non-linearity and Non-linear Least Squares</span></h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title">
      <span class="lang-ko">강의 자료 준비 중</span>
      <span class="lang-en">Coming soon</span>
    </span>
    <span class="tba-badge">TBU</span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">Supp.</span>
      <span class="lang-ko">Matrix decomposition (Dense/sparse QR, Cholesky, multifrontal Cholesky 등)</span>
      <span class="lang-en">Matrix decomposition (Dense/sparse QR, Cholesky, multifrontal Cholesky, etc.)</span>
    </span>
    <span class="lecture-links">
      <a href="/slam_slides/Lec10_math_supplementary.pdf" class="btn-slide">Slide</a>
    </span>
  </li>
</ul>
</div>

<div class="slam-section">
<h3><span class="lang-ko">Lecture 11: Sensor Front-end</span><span class="lang-en">Lecture 11: Sensor Front-end</span></h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title">
      <span class="lang-ko">강의 자료 준비 중</span>
      <span class="lang-en">Coming soon</span>
    </span>
    <span class="tba-badge">TBU</span>
  </li>
</ul>
</div>

<div class="slam-section">
<h3><span class="lang-ko">강의는 계속 추가될 예정입니다...</span><span class="lang-en">More lectures coming soon...</span></h3>
</div>

<script>
(function () {
    var STORAGE_KEY = 'slam-lang';
    var supported = ['ko', 'en'];
    var initial = localStorage.getItem(STORAGE_KEY);
    if (supported.indexOf(initial) === -1) initial = 'en';

    // Header title / subtitle come from the Jekyll layout, not this page's markup,
    // so we swap them in JS. To localize more header strings later, add keys here.
    var titleMap    = { ko: '모두를 위한 SLAM', en: 'SLAM for Everyone' };
    var subtitleMap = { ko: 'SLAM for Everyone', en: '' };

    var headingH1       = document.querySelector('.page-heading h1, .post-heading h1, .intro-header h1');
    var headingSubtitle = document.querySelector('.page-subheading, .post-subheading');
    var headingHr       = headingSubtitle ? headingSubtitle.previousElementSibling : null;
    if (headingHr && headingHr.tagName !== 'HR') headingHr = null;

    var buttons = document.querySelectorAll('.lang-toggle button[data-lang]');

    function setActive(lang) {
        document.body.setAttribute('data-lang', lang);
        localStorage.setItem(STORAGE_KEY, lang);
        buttons.forEach(function (btn) {
            btn.classList.toggle('active', btn.dataset.lang === lang);
        });
        if (headingH1) headingH1.textContent = titleMap[lang];
        if (headingSubtitle) {
            var sub = subtitleMap[lang];
            headingSubtitle.textContent = sub;
            headingSubtitle.style.display = sub ? '' : 'none';
            if (headingHr) headingHr.style.display = sub ? '' : 'none';
        }
    }
    setActive(initial);
    buttons.forEach(function (btn) {
        btn.addEventListener('click', function () {
            setActive(btn.dataset.lang);
        });
    });
})();
</script>

<script>
(function () {
    // PDF.js is fetched lazily on the first View click — no cost on page load
    // for users who only want to scan the lecture list.
    var PDFJS_VERSION = '4.4.168';
    var PDFJS_CDN     = 'https://cdnjs.cloudflare.com/ajax/libs/pdf.js/' + PDFJS_VERSION;
    var pdfjsPromise  = null;

    function loadPdfJs() {
        if (!pdfjsPromise) {
            pdfjsPromise = import(PDFJS_CDN + '/pdf.min.mjs').then(function (lib) {
                lib.GlobalWorkerOptions.workerSrc = PDFJS_CDN + '/pdf.worker.min.mjs';
                return lib;
            });
        }
        return pdfjsPromise;
    }

    var ZOOM_STEPS = [0.5, 0.75, 1, 1.25, 1.5, 1.75, 2];

    function attachViewToggle(slideLink) {
        var url = slideLink.getAttribute('href');
        if (!url || !/\.pdf$/i.test(url)) return;

        var btn = document.createElement('button');
        btn.type = 'button';
        btn.className = 'btn-view-toggle';
        btn.textContent = '▽ View';
        btn.setAttribute('aria-expanded', 'false');
        slideLink.insertAdjacentElement('afterend', btn);

        var viewer = null;
        var loaded = false;

        btn.addEventListener('click', function () {
            var li = btn.closest('li');
            if (!li) return;

            // Toggle: if already expanded, collapse and bail
            if (viewer && viewer.style.display !== 'none') {
                viewer.style.display = 'none';
                btn.classList.remove('active');
                btn.setAttribute('aria-expanded', 'false');
                btn.textContent = '▽ View';
                return;
            }

            if (!viewer) {
                viewer = document.createElement('div');
                viewer.className = 'pdf-inline-viewer';
                viewer.innerHTML =
                    '<div class="pdf-inline-viewer-toolbar">' +
                        '<span class="pdf-status-label">Loading…</span>' +
                        '<span class="pdf-spacer"></span>' +
                        '<div class="pdf-zoom-group">' +
                            '<button type="button" class="pdf-zoom-out" aria-label="Zoom out">−</button>' +
                            '<span class="pdf-zoom-label">100%</span>' +
                            '<button type="button" class="pdf-zoom-in" aria-label="Zoom in">+</button>' +
                        '</div>' +
                        '<a class="pdf-open-new" href="' + url + '" target="_blank" rel="noopener">Open ↗</a>' +
                    '</div>' +
                    '<div class="pdf-inline-viewer-pages">' +
                        '<div class="pdf-status">⏳ Loading PDF…</div>' +
                    '</div>';
                li.insertAdjacentElement('afterend', viewer);
            } else {
                viewer.style.display = '';
            }

            btn.classList.add('active');
            btn.setAttribute('aria-expanded', 'true');
            btn.textContent = '△ Hide';

            if (!loaded) {
                loaded = true;
                renderPdfInto(viewer, url).catch(function (err) {
                    var pages = viewer.querySelector('.pdf-inline-viewer-pages');
                    if (pages) {
                        pages.innerHTML =
                            '<div class="pdf-status">⚠️ Failed to load: ' +
                            String(err && err.message || err) + '</div>';
                    }
                });
            }
        });
    }

    function renderPdfInto(container, url) {
        var pages       = container.querySelector('.pdf-inline-viewer-pages');
        var statusLabel = container.querySelector('.pdf-status-label');
        var zoomLabelEl = container.querySelector('.pdf-zoom-label');
        var zoomInBtn   = container.querySelector('.pdf-zoom-in');
        var zoomOutBtn  = container.querySelector('.pdf-zoom-out');

        var zoom = 1;
        var slots = [];
        var io = null;
        // renderGen invalidates in-flight renders when the layout changes
        // (zoom step). Each renderSlot captures the current value and bails
        // on every async boundary if it has changed.
        var renderGen = 0;

        return loadPdfJs().then(function (pdfjs) {
            return pdfjs.getDocument(url).promise;
        }).then(function (pdf) {
            var n = pdf.numPages;
            statusLabel.textContent = n + ' pages';
            pages.innerHTML = '';

            var pagePromises = [];
            for (var i = 1; i <= n; i++) pagePromises.push(pdf.getPage(i));

            return Promise.all(pagePromises).then(function (pageObjs) {
                pageObjs.forEach(function (page, idx) {
                    var vp0  = page.getViewport({ scale: 1 });
                    var wrap = document.createElement('div');
                    wrap.className     = 'page-wrap';
                    wrap.style.aspectRatio = vp0.width + ' / ' + vp0.height;
                    wrap.dataset.idx   = String(idx);

                    var num = document.createElement('div');
                    num.className   = 'page-num';
                    num.textContent = (idx + 1) + ' / ' + n;
                    wrap.appendChild(num);

                    pages.appendChild(wrap);
                    slots.push({ page: page, wrap: wrap, vp0: vp0, rendered: false });
                });

                applyZoom();
                setupLazyRender();
            });
        });

        function setupLazyRender() {
            if (io) io.disconnect();
            io = new IntersectionObserver(function (entries) {
                entries.forEach(function (entry) {
                    if (!entry.isIntersecting) return;
                    var slot = slots[Number(entry.target.dataset.idx)];
                    if (slot) renderSlot(slot);
                });
            }, { root: pages, rootMargin: '400px 0px' });
            slots.forEach(function (s) { io.observe(s.wrap); });
        }

        function renderSlot(slot) {
            if (slot.rendered) return;
            slot.rendered = true;
            var myGen = renderGen;

            var cssW = slot.wrap.getBoundingClientRect().width;
            if (!cssW) { slot.rendered = false; return; }

            var DPR   = Math.min(window.devicePixelRatio || 1, 2);
            var scale = (cssW * DPR) / slot.vp0.width;
            var vp    = slot.page.getViewport({ scale: scale });

            var oldCanvas = slot.wrap.querySelector('canvas');
            if (oldCanvas) oldCanvas.remove();

            var canvas = document.createElement('canvas');
            canvas.width  = vp.width;
            canvas.height = vp.height;
            slot.wrap.insertBefore(canvas, slot.wrap.firstChild);

            slot.page.render({
                canvasContext: canvas.getContext('2d'),
                viewport: vp
            }).promise.catch(function () {
                if (myGen === renderGen) slot.rendered = false;
            });
        }

        function applyZoom() {
            zoomLabelEl.textContent = Math.round(zoom * 100) + '%';
            slots.forEach(function (s) {
                s.wrap.style.maxWidth = (800 * zoom) + 'px';
            });
        }

        function rebuildAtNewZoom() {
            renderGen++;
            slots.forEach(function (s) {
                s.rendered = false;
                var c = s.wrap.querySelector('canvas');
                if (c) c.remove();
            });
            applyZoom();
            // wait one frame so the new max-width has propagated before the
            // observer measures wrap widths for the canvas backing-store size
            requestAnimationFrame(setupLazyRender);
        }

        function stepZoom(dir) {
            var i = ZOOM_STEPS.findIndex(function (z) { return z >= zoom - 1e-3; });
            if (i < 0) i = ZOOM_STEPS.length - 1;
            if (dir > 0 && ZOOM_STEPS[i] <= zoom + 1e-3) i++;
            if (dir < 0) i--;
            i = Math.max(0, Math.min(ZOOM_STEPS.length - 1, i));
            if (ZOOM_STEPS[i] === zoom) return;
            zoom = ZOOM_STEPS[i];
            rebuildAtNewZoom();
        }

        zoomInBtn.addEventListener('click',  function () { stepZoom(+1); });
        zoomOutBtn.addEventListener('click', function () { stepZoom(-1); });
        zoomLabelEl.addEventListener('click', function () {
            if (zoom !== 1) { zoom = 1; rebuildAtNewZoom(); }
        });
    }

    var slideLinks = document.querySelectorAll('.lecture-list .lecture-links .btn-slide');
    slideLinks.forEach(attachViewToggle);
})();
</script>
