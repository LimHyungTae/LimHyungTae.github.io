---
layout: page
title: Publications

---

Please refer to my [Google scholar](https://scholar.google.com/citations?user=S1A3nbIAAAAJ&hl=en&oi=ao), [CV](https://github.com/LimHyungTae/LimHyungTae.github.io/blob/master/cv_and_research_statement/cv.pdf), and [Research statement](https://github.com/LimHyungTae/LimHyungTae.github.io/blob/master/cv_and_research_statement/research_statement.pdf) (updated on 2025-03-08).

## Publications

(under construction...)


{% assign pubs = site.data.publications | reverse %}
<ul style="list-style-type: none; padding-left: 0;">
  {% for pub in pubs %}
  <li style="margin-bottom: 2em; display: flex; align-items: flex-start;">
    {% if pub.image %}
    <div style="margin-right: 1em;">
      <img src="{{ pub.image }}" alt="thumbnail" style="width: 150px; border-radius: 8px; box-shadow: 0 0 5px rgba(0,0,0,0.1);">
    </div>
    {% endif %}
    <div>
      <p style="margin-bottom: 0.0em;"><strong>{{ pub.title }}</strong></p>
      <p style="margin-bottom: 0.0em;"><em>{{ pub.authors }}</em></p>
      <p style="margin-bottom: 0.0em;">({{ pub.venue }})</p>
      <p style="margin: 0;">
        {% if pub.paper_link %}<a href="{{ pub.paper_link }}">[PDF]</a>{% endif %}
        {% if pub.arxiv_link %}<a href="{{ pub.arxiv_link }}">[arXiv]</a>{% endif %}
        {% if pub.poster_link %}<a href="{{ pub.poster_link }}">[Poster]</a>{% endif %}
        {% if pub.project_page %}<a href="{{ pub.project_page }}">[Project Page]</a>{% endif %}
      </p>
    </div>
  </li>
  {% endfor %}
</ul>
