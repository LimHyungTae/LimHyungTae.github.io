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
    background-color: #ff0000;
    color: white !important;
    border: 1px solid #ff0000;
}

.btn-video:hover {
    background-color: #cc0000;
    border-color: #cc0000;
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

<div class="slam-intro">
SLAM(Simultaneous Localization and Mapping)을 처음 접하는 분들을 위한 강의 시리즈입니다.
수학적 기초부터 실제 시스템 구현까지, SLAM의 전반적인 내용을 다룹니다.
강의 자료는 순차적으로 업데이트될 예정입니다.
</div>

---

### Schedule

<div class="slam-section">
<h3>Lecture 1: Introduction</h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">1-1.</span> SLAM이란 무엇인가?</span>
    <span class="lecture-links">
      <a href="https://limhyungtae.github.io/" class="btn-video">Video</a>
      <a href="https://www.dropbox.com/scl/fi/7owksuii3djeqacchlc15/Lec01_SLAM_for_everyone.pdf?rlkey=wg5a9c9ocr8j4w0vfs1q1i9op&dl=0" class="btn-slide">Slide</a>
    </span>
  </li>
</ul>
</div>

<div class="slam-section">
<h3>Lecture 2: Categories & Terminology of Robot Navigation</h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">2-1.</span> Robot Navigation의 분류와 용어 정리</span>
    <span class="lecture-links">
      <a href="https://limhyungtae.github.io/" class="btn-video">Video</a>
      <a href="https://www.dropbox.com/scl/fi/9zju3acvd22oy8cab3jpt/Lec02_SLAM_for_everyone.pdf?rlkey=4mf4wngii91b5bxo9ngio29tg&dl=0" class="btn-slide">Slide</a>
    </span>
  </li>
</ul>
</div>

<div class="slam-section">
<h3>Lecture 3: State, Measurement, and Estimation</h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">3-1.</span> State, measurement, estimation의 개념</span>
    <span class="lecture-links">
      <a href="https://limhyungtae.github.io/" class="btn-video">Video</a>
      <a href="https://limhyungtae.github.io/" class="btn-slide">Slide</a>
    </span>
  </li>
</ul>
</div>

<div class="slam-section">
<h3>Lecture 4: Maximum Likelihood Estimation (MLE), Maximum A Posteriori (MAP)</h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">4-1.</span> MLE와 MAP의 개념과 SLAM에서의 역할</span>
    <span class="lecture-links">
      <a href="https://limhyungtae.github.io/" class="btn-video">Video</a>
      <a href="https://limhyungtae.github.io/" class="btn-slide">Slide</a>
    </span>
  </li>
</ul>
</div>

<div class="slam-section">
<h3>Lecture 5: Kalman Filter</h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">5-1.</span> Kalman filter의 원리와 SLAM 적용</span>
    <span class="lecture-links">
      <a href="https://limhyungtae.github.io/" class="btn-video">Video</a>
      <a href="https://limhyungtae.github.io/" class="btn-slide">Slide</a>
    </span>
  </li>
</ul>
</div>

<div class="slam-section">
<h3>To be updated</h3>
<ul class="lecture-list">
  <li>
    <span class="lecture-title"><span class="lecture-num">6-1.</span> To be updated</span>
    <span class="lecture-links">
      <a href="https://limhyungtae.github.io/" class="btn-video">Video</a>
      <a href="https://limhyungtae.github.io/" class="btn-slide">Slide</a>
    </span>
  </li>
</ul>
</div>

<div class="slam-section">
<h3>More lectures coming soon...</h3>
</div>
