---
layout: page
title: Publications

---

Please refer to my [Google scholar](https://scholar.google.com/citations?user=S1A3nbIAAAAJ&hl=en&oi=ao), [CV](https://github.com/LimHyungTae/LimHyungTae.github.io/blob/master/cv_and_research_statement/cv.pdf), and [Research statement](https://github.com/LimHyungTae/LimHyungTae.github.io/blob/master/cv_and_research_statement/research_statement.pdf) (updated on 2025-03-08).

## Publications

<div class="container mt-4">
  <div class="row">
    {% for pub in site.data.publications %}
    <div class="col-12 mb-4">
      <div class="d-flex align-items-start">
        {% if pub.image %}
        <div class="me-3" style="flex: 0 0 180px;">
          <img src="{{ pub.image }}" class="img-fluid rounded shadow-sm" alt="{{ pub.title }}" style="width: 180px;">
        </div>
        {% endif %}
        <div>
          <h5 class="mb-1">{{ pub.title }}</h5>
          <p class="mb-1"><strong>{{ pub.authors }}</strong></p>
          <p class="mb-1"><em>{{ pub.venue }}</em></p>
          <div>
            {% if pub.paper_link %}
            <a href="{{ pub.paper_link }}" class="btn btn-outline-primary btn-sm me-2">📄 PDF</a>
            {% endif %}
            {% if pub.poster_link %}
            <a href="{{ pub.poster_link }}" class="btn btn-outline-secondary btn-sm me-2">📜 Poster</a>
            {% endif %}
            <button class="btn btn-outline-secondary btn-sm" type="button" data-bs-toggle="collapse" data-bs-target="#bibtex-{{ pub.id }}" aria-expanded="false">
              📑 BibTeX
            </button>
          </div>
          <div class="collapse mt-2" id="bibtex-{{ pub.id }}">
            <pre class="bg-light p-2 border">{{ pub.bibtex }}</pre>
          </div>
        </div>
      </div>
    </div>
    {% endfor %}
  </div>
</div>
