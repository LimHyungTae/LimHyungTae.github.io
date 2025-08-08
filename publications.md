---
layout: page
title: Publications
---

<style>
.publications-container {
    margin-top: 20px;
}

.publication-item {
    display: flex;
    margin-bottom: 40px;
    padding: 20px 0;
    border-bottom: 1px solid #eee;
}

.publication-item:last-child {
    border-bottom: none;
}

.publication-image {
    flex: 0 0 200px;
    margin-right: 30px;
    text-align: center;
}

.publication-image img {
    max-width: 100%;
    max-height: 150px;
    width: auto;
    height: auto;
    border-radius: 8px;
    box-shadow: 0 2px 8px rgba(0,0,0,0.1);
}

.publication-details {
    flex: 1;
    line-height: 1.6;
}

.publication-title {
    font-size: 18px;
    font-weight: bold;
    margin: 0 0 8px 0;
    color: #2c3e50;
}

.publication-authors {
    margin: 0 0 8px 0;
    color: #555;
}

.publication-authors .my-name {
    font-weight: bold;
    color: #2c3e50;
}

.publication-venue {
    margin: 0 0 12px 0;
    font-style: italic;
    color: #666;
}

.publication-venue .highlight {
    font-weight: bold;
    color: #e74c3c;
}

.publication-links {
    margin: 0;
}

.publication-links a {
    color: #3498db;
    text-decoration: none;
    margin-right: 15px;
}

.publication-links a:hover {
    color: #2980b9;
    text-decoration: underline;
}

.year-header {
    font-size: 24px;
    font-weight: bold;
    margin: 40px 0 20px 0;
    padding-bottom: 10px;
    border-bottom: 2px solid #3498db;
    color: #2c3e50;
}

.year-header:first-child {
    margin-top: 20px;
}

@media (max-width: 768px) {
    .publication-item {
        flex-direction: column;
    }
    
    .publication-image {
        flex: none;
        margin-right: 0;
        margin-bottom: 15px;
    }
}
</style>

Please refer to my [Google Scholar](https://scholar.google.com/citations?user=S1A3nbIAAAAJ&hl=en&oi=ao), [CV](https://github.com/LimHyungTae/LimHyungTae.github.io/blob/master/cv_and_research_statement/cv.pdf), and [Research Statement](https://github.com/LimHyungTae/LimHyungTae.github.io/blob/master/cv_and_research_statement/research_statement.pdf) (updated on 2025-03-08).

## Publications

<div class="publications-container">

<!-- Generated from _data/publications.yml -->
{% assign publications_by_year = site.data.publications | group_by: 'year' | sort: 'name' | reverse %}

{% for year_group in publications_by_year %}
<div class="year-header">{{ year_group.name }}</div>

{% assign year_publications = year_group.items | sort: 'news_date' | reverse %}
{% for pub in year_publications %}
<div class="publication-item">
    <div class="publication-image">
        {% if pub.image %}
            <img src="{{ pub.image }}" alt="{{ pub.title }}">
        {% else %}
            <img src="/img/publications/BUFFER-X.gif" alt="{{ pub.title }}">
        {% endif %}
    </div>
    <div class="publication-details">
        <div class="publication-title">{{ pub.title }}</div>
        <div class="publication-authors">
            {% assign authors = pub.authors | replace: pub.my_name, '<span class="my-name">' | append: pub.my_name | append: '</span>' %}
            {{ authors }}
        </div>
        <div class="publication-venue">
            {% if pub.status == 'accepted_highlight' %}
                <span class="highlight">{{ pub.venue }} (Highlight)</span>
            {% elsif pub.status == 'accepted' %}
                {{ pub.venue }}
            {% elsif pub.status == 'under_review' %}
                <em>{{ pub.venue }} (Under review)</em>
            {% else %}
                {{ pub.venue }}
            {% endif %}
        </div>
        <div class="publication-links">
            {% if pub.arxiv %}
                <a href="{{ pub.arxiv }}">[arXiv]</a>
            {% endif %}
            {% if pub.paper_link %}
                <a href="{{ pub.paper_link }}">[Paper]</a>
            {% endif %}
            {% if pub.github %}
                <a href="{{ pub.github }}">[Code]</a>
            {% endif %}
            {% if pub.project_page %}
                <a href="{{ pub.project_page }}">[Project]</a>
            {% endif %}
            {% if pub.poster_link %}
                <a href="{{ pub.poster_link }}">[Poster]</a>
            {% endif %}
            {% if pub.ieee_link %}
                <a href="{{ pub.ieee_link }}">[IEEE]</a>
            {% endif %}
        </div>
    </div>
</div>
{% endfor %}
{% endfor %}

</div>
