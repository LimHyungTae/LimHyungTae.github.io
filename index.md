---
layout: page
title: Hyungtae Lim
---

I am a Postdoctoral Associate in the [Laboratory for Information and Decision Systems](https://lids.mit.edu/) (LIDS) at [MIT](https://mit.edu/), where I work with [Luca Carlone](https://lucacarlone.mit.edu/) in the [SPARK Lab](http://web.mit.edu/sparklab/). My research focuses on developing robust algorithms for robot perception, state estimation, and lifelong mapping that enable long-term autonomy in real-world environments.

I obtained my Ph.D. and M.S. in Electrical Engineering from [KAIST](https://www.kaist.ac.kr/), where I was fortunate to work with [Hyun Myung](https://urobot.kaist.ac.kr/url_teams/prof-hyunmyung/) in the [Urban Robotics Laboratory](https://urobot.kaist.ac.kr/). During my graduate studies, I was a visiting scholar at [StachnissLab](https://www.ipb.uni-bonn.de) at the University of Bonn, working with [Cyrill Stachniss](https://www.ipb.uni-bonn.de/people/cyrill-stachniss/), and a research intern at [NAVER LABS](https://www.naverlabs.com/).

I am passionate about open-source research and actively contribute to the robotics community through well-documented, reproducible code. Check out my [GitHub](https://github.com/LimHyungTae) for various projects on SLAM, point cloud processing, and robust perception.

[![Linkedin Badge](https://img.shields.io/badge/-LinkedIn-blue?style=flat-square&logo=Linkedin&logoColor=white&link=https://www.linkedin.com/in/hyungtae-lim-34b8a015a/)](https://www.linkedin.com/in/hyungtae-lim-34b8a015a/)
[![Scholar Badge](https://img.shields.io/badge/Scholar-4285F4?style=flat-square&logo=GoogleScholar&logoColor=white&link=https://scholar.google.com/citations?user=S1A3nbIAAAAJ&hl=en)](https://scholar.google.com/citations?user=S1A3nbIAAAAJ&hl=en)
[![CV Badge](https://img.shields.io/badge/CV-navy?style=flat-square&logo=AdobeAcrobatReader&logoColor=white&link=https://github.com/LimHyungTae/LimHyungTae.github.io/blob/master/cv_and_research_statement/cv.pdf)](https://github.com/LimHyungTae/LimHyungTae.github.io/blob/master/cv_and_research_statement/cv.pdf)
[![Research Statement](https://img.shields.io/badge/Research%20Statement-darkorange?style=flat-square&logo=ResearchGate&logoColor=white&link=https://github.com/LimHyungTae/LimHyungTae.github.io/blob/master/cv_and_research_statement/research_statement.pdf)](https://github.com/LimHyungTae/LimHyungTae.github.io/blob/master/cv_and_research_statement/research_statement.pdf)

---

## News

<div class="container" style="max-width: 750px;">
  <div class="list-group">
    {% assign latest_news = site.data.news | slice: 0, 5 %}
    {% assign older_news = site.data.news | slice: 5, site.data.news.size %}

    {% for news in latest_news %}
    <div class="list-group-item small">
      <strong>{{ news.date }}</strong> {{ news.content }}
    </div>
    {% endfor %}

    <div id="older-news" style="display: none;">
      {% for news in older_news %}
      <div class="list-group-item small">
        <strong>{{ news.date }}</strong> {{ news.content }}
      </div>
      {% endfor %}
    </div>
  </div>

  {% if older_news.size > 0 %}
  <button id="toggle-news" class="btn btn-outline-primary mt-3">Show More ▽</button>
  {% endif %}
</div>

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

<!-- ## Research Interests -->

<!-- My research aims to enable terrestrial robots—including wheeled mobile robots, autonomous vehicles, and quadruped robots—to operate reliably in complex, real-world environments. I focus on three interconnected areas: (1) **robust egocentric perception**, developing algorithms for ground segmentation and traversability estimation that work across diverse platforms and sensor configurations; (2) **outlier-robust SLAM**, designing localization and mapping systems that handle spurious correspondences, sensor noise, and dynamic objects; and (3) **lifelong map management**, building techniques for static map construction and multi-session SLAM that allow robots to maintain and update their spatial understanding over extended operations. -->

<!-- I am particularly interested in algorithms that generalize well beyond their training conditions and that can be easily reproduced and deployed by other researchers and practitioners. -->

<!-- --- -->

## Selected Honors

* [RSS Pioneers 2024](https://sites.google.com/view/rsspioneers2024/participants) (30 young researchers selected worldwide)
* [2022 IEEE RA-L Best Paper Award](https://www.ieee-ras.org/publications/ra-l/ra-l-paper-awards) (5 papers selected among 1,100)
* [IEEE ICRA 2025 Outstanding Reviewer Award](https://www.ieee-ras.org/conferences-workshops/fully-sponsored/icra/icra-ceb-awards) (5 selected among 7,641 reviewers)
* 1st prize in [HILTI SLAM Challenge](https://hilti-challenge.com/) at IEEE ICRA 2023 and 2024
* [CES 2023 Innovation Award](https://www.ces.tech/innovation-awards/honorees/2023/honorees/h/hi-bot-hologram-image-guide-robot.aspx) via tech transfer on SLAM (with HILLS Robotics)

---

## Contact

shapelim `at` mit `dot` edu
