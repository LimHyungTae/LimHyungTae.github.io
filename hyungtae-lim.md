---
layout: page
title: About Me
---

<p class="about-text">
<span class="fa fa-briefcase about-icon"></span>
Currently a <strong>postdoc</strong> in <a href="http://web.mit.edu/sparklab/"><strong>SPARK Lab</strong></a> at Massachusetts Institute of Technology (MIT) since Apr. 2024.
Previously worked as a <strong>postdoc</strong> in <a href="https://urobot.kaist.ac.kr/"><strong>Urban Robotics Labaratory</strong></a> at Korea Advanced Institute of Science and Technology (KAIST), a <strong>visiting student</strong> in <a href="https://www.ipb.uni-bonn.de" target="_blank">StachnissLab</a> at University of Bonn, and <strong>research intern</strong> in <a href="https://www.naverlabs.com/" target="_blank">Naver Labs</a>.
</p>

<p class="about-text">
<span class="fa fa-graduation-cap about-icon"></span>
Earned my Ph.D and M.S in <strong>Electrical Engineering</strong> from KAIST (supervised by academic father, <a href="https://urobot.kaist.ac.kr/url_teams/prof-hyunmyung/" target="_blank">Prof. Hyun Myung</a>) and my Bachelor of <strong>Mechanical Engineering</strong> from KAIST.
</p>

<p class="about-text">
<span class="fa fa-code about-icon"></span>
I'm a robotics researcher and an enthusiast of open source as well. So I enjoy open-sourcing <strong>my researches regarding robotics</strong> to help others &mdash; check out <a href="https://github.com/LimHyungTae">my Github repository</a>. 
</p>

[![Linkedin Badge](https://img.shields.io/badge/-LinkedIn-blue?style=flat-square&logo=Linkedin&logoColor=white&link=https://www.linkedin.com/in/hyungtae-lim-34b8a015a/)](https://www.linkedin.com/in/hyungtae-lim-34b8a015a/)
[![Scholar Badge](https://img.shields.io/badge/Scholar-4285F4?style=flat-square&logo=GoogleScholar&logoColor=white&link=https://scholar.google.com/citations?user=S1A3nbIAAAAJ&hl=en)](https://scholar.google.com/citations?user=S1A3nbIAAAAJ&hl=en)
[![CV Badge](https://img.shields.io/badge/CV-navy?style=flat-square&logo=AdobeAcrobatReader&logoColor=white&link=https://github.com/LimHyungTae/LimHyungTae.github.io/blob/master/cv_and_research_statement/cv.pdf)](https://github.com/LimHyungTae/LimHyungTae.github.io/blob/master/cv_and_research_statement/cv.pdf)
[![Research Statement1](https://img.shields.io/badge/Research%20Statement1-darkorange?style=flat-square&logo=ResearchGate&logoColor=white&link=https://github.com/LimHyungTae/LimHyungTae.github.io/blob/master/cv_and_research_statement/research_statement.pdf)](https://github.com/LimHyungTae/LimHyungTae.github.io/blob/master/cv_and_research_statement/research_statement.pdf)
[![Research Statement2](https://img.shields.io/badge/Research%20Statement2-forestgreen?style=flat-square&logo=arXiv&logoColor=white&link=https://arxiv.org/abs/2405.11176)](https://arxiv.org/abs/2405.11176)

<details>
  <summary>**Detailed Bio (click)**</summary>
  Hyungtae Lim is a postdoctoral researcher in the SPARK Lab at the Massachusetts Institute of Technology (MIT) since April 2024.
  Previously, he worked as a postdoctoral researcher in the Urban Robotics Laboratory at the Korea Advanced Institute of Science and Technology (KAIST), a visiting student in StachnissLab at the University of Bonn, and a research intern at Naver Labs.

  He earned his Ph.D. and M.S. degrees in Electrical Engineering from KAIST under the supervision of Prof. Hyun Myung, and his Bachelor's degree in Mechanical Engineering from KAIST.
</details>

---

## News

<div class="container">
  <div class="list-group">
    {% assign latest_news = site.data.news | slice: 0, 3 %}
    {% assign older_news = site.data.news | slice: 3, site.data.news.size %}
    
    <!-- 최신 3개 뉴스 표시 -->
    {% for news in latest_news %}
    <div class="list-group-item">
      <strong>{{ news.date }}</strong> {{ news.content }}
    </div>
    {% endfor %}

    <!-- 나머지 뉴스 (초기에 숨김) -->
    <div id="older-news" style="display: none;">
      {% for news in older_news %}
      <div class="list-group-item">
        <strong>{{ news.date }}</strong> {{ news.content }}
      </div>
      {% endfor %}
    </div>
  </div>

  <!-- 더보기 버튼 -->
  {% if older_news.size > 0 %}
  <button id="toggle-news" class="btn btn-outline-primary mt-3">Show More ▽</button>
  {% endif %}
</div>

<!-- JavaScript로 토글 기능 추가 -->
<script>
  document.getElementById("toggle-news").addEventListener("click", function() {
    var olderNews = document.getElementById("older-news");
    if (olderNews.style.display === "none") {
      olderNews.style.display = "block";
      this.textContent = "Show Less △";
    } else {
      olderNews.style.display = "none";
      this.textContent = "Show More ▽";
    }
  });
</script>

---

## Highlights

* Postdoc associate at [SPARK Lab, MIT](https://mit.edu/sparklab/people.html) (advisor: [Prof. Luca Carlone](https://lucacarlone.mit.edu/))
* 1st prize in [HILTI SLAM Challenge'24 in IEEE ICRA](https://construction-robots.github.io/#challenge)
* [RSS Pioneers 2024](https://sites.google.com/view/rsspioneers2024/participants) (only 30 young researchers are selected each year)
* 20+ IROS, ICRA, RA-L, RSS, IJRR, CVPR, and AAAI papers during grad school (12 first-author papers)
* 2022 IEEE RA-L Best Paper Award (among 1,100 papers, only 5 papers are selected)
* 1st prize in [HILTI SLAM Challenge'23 in IEEE ICRA](https://hilti-challenge.com/leader-board-2023.html) among 63 international teams
* [CES'23 innovation award](https://www.ces.tech/innovation-awards/honorees/2023/honorees/h/hi-bot-hologram-image-guide-robot.aspx) via tech. transfer regarding SLAM of mobile robots (collaborated with HILLS Robotics)
* Visiting scholar of Univ. Bonn, Germany (advisor: [Prof. Cyrill Stachniss](https://www.ipb.uni-bonn.de/people/cyrill-stachniss/index.html))
* From 2022, serve as a SLAM part outside expert, CTO division of LG Electronics, Republic of Korea
* 2nd cash prize in [HILTI SLAM Challenge'22 in IEEE ICRA](https://hilti-challenge.com/leader-board-2022.html) (in total, 4th place)
* Research intern of vision/deep learning team of [NAVER LABS](https://www.naverlabs.com/), Republic of Korea
* 1st prize in [Hitachi-LG Data Storage](https://hitachi-lg.com/) LiDAR application competition, Republic of Korea

## Field of Research Interest

My research interest includes, but not confined to:

* SLAM using LiDAR, camera, or radar sensors
* Robust 3D point cloud registration
* Mobile robotics including autonomous vehicles
* All kinds of perception that help perceiving neighbor situations
    * In particular, ground segmentation as pre-processing by using a 3D point cloud
    * Instance segmentation and clustering
* Localization including visual localization (i.e. visual place recognition)
* Static map building by updating a prior 3D point cloud map
* Robust visual(-inertial) odometry
* Analytical redundancy

## Detailed Education

* **Ph.D candidate Electrical Engineering(also complete Robotics Program) / KAIST** - 2020.03~ 2023.02 
  * Thesis title: Robust LiDAR SLAM Framework for Autonomous Vehicles Leveraging Ground Segmentation
* **M.S Electrical Engineering(also complete Robotics Program) / KAIST** - 2018.03~2020.02
  * Thesis title: Two-stage Depth Prediction using a 2D LiDAR and a Monocular Camera via Deep Learning  
* **B.S Mechanical Engineering / KAIST** - 2013.03~2018.02

## Experience

* **Invited Visiting Scholar at StachnissLab, Univ. Bonn, Germany** - 2022.11 ~ 2023.02
* **Ouside Expert of CTO Division of LG Electronics** - 2022.03 ~
* **CV / Deep Learning Research Intern, NAVER LABS Corp.** 2021.04 ~ 2021.09 (6 months)

## Honors & Awards

* **CES 2023 Innovation Awards With Hills Robotics** - 2022.10
* **2nd Prize among academia in HILTI SLAM Challenge 2022 (workshop in IEEE ICRA)** - 2022.05
* **Student Best Paper Award in ICCAS 2020** - 2020.10
* **Received Kim Sung-bue Creative Activity Award, from KAIST** - 2020.07
* **Hitachi-LG LiDAR application Competition, Grand Prize ($ 5,000)** - 2019.08
* **Received Han Cheolhui Augustine Scholarship, from EE, KAIST** - 2019.04
* **2018 Smart City Service and Start-up Competition, Excellence Prize** - 2018.09
* **Listed on _Dean's list_ of ME, KAIST (GPA 4.23/4.3 at the semester)** - Fall semester, 2015

## Fun Facts

I won the grand prize at Taeul Singing Competition, which is the largest singing competition of Chungcheong four years in a row!

![taewool](/img/taewool.png)

--- 

## Contact

shapelim `at` mit `dot` edu

