---
layout: page
title: Hyungtae Lim
description: "Building spatial AI for robots"
---

<div class="home-intro">
  <p class="home-intro-label">Research mission</p>
  <p class="home-mission">{{ page.description }}</p>

  <nav class="home-cta-list" aria-label="Research profiles and documents">
    <a class="home-cta home-cta--primary" href="{{ '/publications/' | relative_url }}">
      <i class="fa-solid fa-book-open" aria-hidden="true"></i>
      <span>Publications</span>
    </a>
    <a class="home-cta" href="https://scholar.google.com/citations?user=S1A3nbIAAAAJ&amp;hl=en">
      <i class="fa-solid fa-graduation-cap" aria-hidden="true"></i>
      <span>Scholar</span>
    </a>
    <a class="home-cta" href="{{ '/cv_and_research_statement/cv.pdf' | relative_url }}">
      <i class="fa-solid fa-file-pdf" aria-hidden="true"></i>
      <span>CV</span>
    </a>
    <a class="home-cta" href="https://github.com/LimHyungTae">
      <i class="fa-brands fa-github" aria-hidden="true"></i>
      <span>GitHub</span>
    </a>
  </nav>
</div>

I am a Postdoctoral Associate at the [Laboratory for Information and Decision Systems](https://lids.mit.edu/) (LIDS) at [MIT](https://mit.edu/), working with [Luca Carlone](https://lucacarlone.mit.edu/) in the [SPARK Lab](http://web.mit.edu/sparklab/).

I received my Ph.D. and M.S. in Electrical Engineering from [KAIST](https://www.kaist.ac.kr/), advised by [Hyun Myung](https://urobot.kaist.ac.kr/url_teams/prof-hyunmyung/) in the [Urban Robotics Laboratory](https://urobot.kaist.ac.kr/), where I also continued as a postdoctoral fellow before joining MIT. During my doctoral studies, I was a visiting scholar at [StachnissLab](https://www.ipb.uni-bonn.de), University of Bonn, with [Cyrill Stachniss](https://www.ipb.uni-bonn.de/people/cyrill-stachniss/), and a research intern at [NAVER LABS](https://www.naverlabs.com/). I currently serve as an [Associate Editor for *IEEE Robotics and Automation Letters* (RA-L)](https://www.ieee-ras.org/publications/ra-l).

I am passionate about open-source research and contribute to the community through well-documented, reproducible code — including [Patchwork++](https://github.com/url-kaist/patchwork-plusplus), [KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher), [ERASOR](https://github.com/LimHyungTae/ERASOR), and [Quatro](https://github.com/url-kaist/Quatro), and as a co-maintainer of [TEASER++](https://github.com/MIT-SPARK/TEASER-plusplus) and [Hydra](https://github.com/MIT-SPARK/Hydra). See my [GitHub](https://github.com/LimHyungTae) for more.

<details markdown="1" style="margin-top: 1.2em;">
<summary><strong>Speaker introduction</strong> — for event organizers (click to expand)</summary>

<br>

**Short version Bio.**

Hyungtae Lim is a Postdoctoral Associate at MIT's SPARK Lab, working with Prof. Luca Carlone. He received his Ph.D. in Electrical Engineering from KAIST in 2023, advised by Prof. Hyun Myung. His research develops deployable spatial intelligence that keeps mobile robots and autonomous vehicles reliable across sensors, environments, and time. He is a recipient of the RSS Pioneers 2024 award and the 2022 IEEE RA-L Best Paper Award, an Associate Editor for IEEE RA-L, and the author of widely used open-source libraries such as [Patchwork++](https://github.com/url-kaist/patchwork-plusplus) and [KISS-Matcher](https://github.com/mit-spark/kiss-matcher).

**Long version Bio.**

Hyungtae Lim is a Postdoctoral Associate in the Laboratory for Information and Decision Systems (LIDS) and the SPARK Lab at the Massachusetts Institute of Technology, working with Prof. Luca Carlone. Before joining MIT in 2024, he was a postdoctoral fellow at the Urban Robotics Lab at the Korea Advanced Institute of Science and Technology (KAIST), where he also obtained his Ph.D. in Electrical Engineering in 2023, advised by Prof. Hyun Myung. During his doctoral studies, he was a visiting scholar at StachnissLab at the University of Bonn, working with Prof. Cyrill Stachniss, and a research intern at NAVER LABS. His research develops deployable spatial intelligence for mobile robots and autonomous vehicles, spanning robust state estimation, outlier-robust registration, and lifelong map adaptation. He emphasizes algorithms that generalize beyond their design conditions and that the broader community can readily reproduce. He is a recipient of the RSS Pioneers 2024 award, the 2022 IEEE RA-L Best Paper Award, and the IEEE ICRA 2025 Outstanding Reviewer Award (5 selected among 7,641 reviewers), and currently serves as an Associate Editor for IEEE Robotics and Automation Letters. His team won first place at the HILTI SLAM Challenge at IEEE ICRA 2023 and 2024.

</details>

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

* Associate Editor, [*IEEE Robotics and Automation Letters* (RA-L)](https://www.ieee-ras.org/publications/ra-l), 2024–
* [IEEE ICRA 2025 Outstanding Reviewer Award](https://www.ieee-ras.org/conferences-workshops/fully-sponsored/icra/icra-ceb-awards) (5 selected among 7,641 reviewers)
* [RSS Pioneers 2024](https://sites.google.com/view/rsspioneers2024/participants) (30 young researchers selected worldwide)
* 1st prize in the [Nothing Stands Still (NSS) Challenge](https://www.nothing-stands-still.com/challenge_2024) at IEEE ICRA 2024
* 1st prize in the [HILTI SLAM Challenge](https://hilti-challenge.com/leaderboard-2023) at IEEE ICRA 2023
* 2nd cash prize, 4th overall in the [HILTI SLAM Challenge](https://hilti-challenge.com/) at IEEE ICRA 2022
* [2022 IEEE RA-L Best Paper Award](https://www.ieee-ras.org/publications/ra-l/ra-l-paper-awards/) (5 papers selected among 1,100)
* [CES 2023 Innovation Award](https://www.ces.tech/innovation-awards/honorees/2023/honorees/h/hi-bot-hologram-image-guide-robot.aspx) via tech transfer on SLAM (with HILLS Robotics)
* 1st Prize, Hitachi-LG Data Storage LiDAR Application Competition (Korea, 2019)

---

## Contact

shapelim `at` mit `dot` edu
