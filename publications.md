---
layout: page
title: Publications

---

Please refer to my [Google scholar](https://scholar.google.com/citations?user=S1A3nbIAAAAJ&hl=en&oi=ao), [CV](https://github.com/LimHyungTae/LimHyungTae.github.io/blob/master/cv_and_research_statement/cv.pdf), and [Research statement](https://github.com/LimHyungTae/LimHyungTae.github.io/blob/master/cv_and_research_statement/research_statement.pdf) (updated on 2025-03-08).

## Publications

<div class="container mt-4">
  <div class="row row-cols-1 row-cols-md-2 row-cols-lg-3 g-4">
    {% for pub in site.data.publications %}
    <div class="col">
      <div class="card h-100 shadow-sm">
        <img src="{{ pub.image }}" class="card-img-top img-fluid" alt="{{ pub.title }}">
        <div class="card-body">
          <h5 class="card-title">{{ pub.title }}</h5>
          <p class="card-text"><strong>{{ pub.authors }}</strong></p>
          <p class="card-text"><em>{{ pub.venue }}</em></p>
          <div class="d-flex flex-wrap">
            {% if pub.paper_link %}
            <a href="{{ pub.paper_link }}" class="btn btn-outline-primary btn-sm me-2">📄 Paper</a>
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
