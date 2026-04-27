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

@media (max-width: 576px) {
    .lecture-list li {
        flex-direction: column;
        align-items: flex-start;
    }
    .lecture-links {
        margin-top: 4px;
    }
}
</style>

<div class="lang-toggle" role="group" aria-label="Language toggle">
  <button type="button" data-lang="ko" class="active">한국어</button>
  <button type="button" data-lang="en">English</button>
</div>

<div class="slam-intro">
<span class="lang-ko">SLAM(Simultaneous Localization and Mapping)을 처음 접하는 분들을 위한 강의 시리즈입니다.
수학적 기초부터 실제 시스템 구현까지, SLAM의 전반적인 내용을 다룹니다.
강의 자료는 순차적으로 업데이트될 예정입니다.</span>
<span class="lang-en">A lecture series for those who are new to SLAM (Simultaneous Localization and Mapping).
From the mathematical foundations to real-world system implementations, we cover SLAM end-to-end.
Materials will be updated sequentially.</span>
</div>

---

### <span class="lang-ko">강의 일정</span><span class="lang-en">Schedule</span>

<div class="slam-section">
<h3><span class="lang-ko">Lecture 1: Introduction</span><span class="lang-en">Lecture 1: Introduction</span></h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">1-1.</span>
      <span class="lang-ko">SLAM이란 무엇인가?</span>
      <span class="lang-en">What is SLAM?</span>
    </span>
    <span class="lecture-links">
      <a href="https://limhyungtae.github.io/" class="btn-video">Video</a>
      <a href="https://www.dropbox.com/scl/fi/7owksuii3djeqacchlc15/Lec01_SLAM_for_everyone.pdf?rlkey=wg5a9c9ocr8j4w0vfs1q1i9op&dl=0" class="btn-slide">Slide</a>
    </span>
  </li>
</ul>
</div>

<div class="slam-section">
<h3><span class="lang-ko">Lecture 2: Categories &amp; Terminology of Robot Navigation</span><span class="lang-en">Lecture 2: Categories &amp; Terminology of Robot Navigation</span></h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">2-1.</span>
      <span class="lang-ko">Robot navigation 구성 요소</span>
      <span class="lang-en">Components of robot navigation</span>
    </span>
    <span class="lecture-links">
      <a href="https://limhyungtae.github.io/" class="btn-video">Video</a>
      <a href="https://www.dropbox.com/scl/fi/9zju3acvd22oy8cab3jpt/Lec02_SLAM_for_everyone.pdf?rlkey=4mf4wngii91b5bxo9ngio29tg&dl=0" class="btn-slide">Slide</a>
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
<h3><span class="lang-ko">Lecture 3: State, Measurement, and Estimation</span><span class="lang-en">Lecture 3: State, Measurement, and Estimation</span></h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">3-1.</span>
      <span class="lang-ko">Introduction: 키 재기 예시로 시작</span>
      <span class="lang-en">Introduction: starting with a height measurement example</span>
    </span>
    <span class="lecture-links">
      <a href="https://youtu.be/JIUWrz2161A" class="btn-video">Video</a>
      <a href="https://www.dropbox.com/scl/fi/dnyorx6ky888so4u5ezhj/Lec03_SLAM_for_everyone.pdf?rlkey=fpnkrld2q5ta3a54s23h2k3i4&dl=0" class="btn-slide">Slide</a>
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
<h3><span class="lang-ko">Lecture 4: Maximum Likelihood Estimation (MLE), Maximum A Posteriori (MAP)</span><span class="lang-en">Lecture 4: Maximum Likelihood Estimation (MLE), Maximum A Posteriori (MAP)</span></h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">4-1.</span>
      <span class="lang-ko">Introduction: MLE와 MAP 개요</span>
      <span class="lang-en">Introduction: overview of MLE and MAP</span>
    </span>
    <span class="lecture-links">
      <a href="https://youtu.be/y2PqPEPTK1A?si=pOsJ2XuOFQ1d3qgg" class="btn-video">Video</a>
      <a href="https://www.dropbox.com/scl/fi/hgns1a3qoua3swb8mbfao/Lec04_SLAM_for_everyone.pdf?rlkey=tfqh2mkw9f48nmlgm7r5sxzse&dl=0" class="btn-slide">Slide</a>
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
<h3><span class="lang-ko">Lecture 5: Filtering vs Smoothing (1) – Kalman Filter from a MAP Perspective</span><span class="lang-en">Lecture 5: Filtering vs Smoothing (1) – Kalman Filter from a MAP Perspective</span></h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">5-1.</span>
      <span class="lang-ko">Filtering vs. Smoothing: sequential vs. batch 비교</span>
      <span class="lang-en">Filtering vs. smoothing: sequential vs. batch</span>
    </span>
    <span class="lecture-links">
      <a href="https://limhyungtae.github.io/" class="btn-video">Video</a>
      <a href="https://www.dropbox.com/scl/fi/u1cmdp19t39dyotrscpfk/Lec05_SLAM_for_everyone.pdf?rlkey=01fwyofj7xs7q75wd7rjrdnr0&dl=0" class="btn-slide">Slide</a>
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
      <span class="lang-ko">Kalman Filter for Multivariable Systems: 행렬 형태 확장</span>
      <span class="lang-en">Kalman filter for multivariable systems: matrix form</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">5-4.</span>
      <span class="lang-ko">결론: KF update = MAP, marginalization은 비가역적</span>
      <span class="lang-en">Conclusion: KF update = MAP, marginalization is irreversible</span>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">Supp.</span>
      <span class="lang-ko">수학 보충 자료</span>
      <span class="lang-en">Math supplementary</span>
    </span>
    <span class="lecture-links">
      <a href="https://www.dropbox.com/scl/fi/5lw9dlpxgfoas6d3pf3qg/Lec05_math_supplementary.pdf?rlkey=qsb2gwijankhhwp108k3l5usk&dl=0" class="btn-slide">Slide</a>
    </span>
  </li>
</ul>
</div>

<div class="slam-section">
<h3><span class="lang-ko">Lecture 6: Filtering vs Smoothing (2) – Graph Optimization from a MAP Perspective</span><span class="lang-en">Lecture 6: Filtering vs Smoothing (2) – Graph Optimization from a MAP Perspective</span></h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">6-1.</span>
      <span class="lang-ko">Filtering vs. Smoothing 비교 복습</span>
      <span class="lang-en">Filtering vs. smoothing: recap</span>
    </span>
    <span class="lecture-links">
      <a href="https://limhyungtae.github.io/" class="btn-video">Video</a>
      <a href="https://www.dropbox.com/scl/fi/dkoydewchta3brymtqkg5/Lec06_SLAM_for_everyone.pdf?rlkey=wac8b6dvfvh207gxegryb0ysr&dl=0" class="btn-slide">Slide</a>
    </span>
  </li>
  <li>
    <span class="lecture-title"><span class="lecture-num">6-2.</span>
      <span class="lang-ko">Graph SLAM: Factor Graph 기반 정식화</span>
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
      <span class="lang-ko">직관적 이해: mass-spring 비유</span>
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
<h3><span class="lang-ko">강의는 계속 추가될 예정입니다...</span><span class="lang-en">More lectures coming soon...</span></h3>
</div>

<script>
(function () {
    var STORAGE_KEY = 'slam-lang';
    var supported = ['ko', 'en'];
    var initial = localStorage.getItem(STORAGE_KEY);
    if (supported.indexOf(initial) === -1) initial = 'ko';

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
