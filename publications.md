---
layout: page
title: Publications
---


I rarely update this page. Please refer to my [Google Scholar](https://scholar.google.com/citations?user=S1A3nbIAAAAJ&hl=en&oi=ao) and [CV](https://github.com/LimHyungTae/LimHyungTae.github.io/blob/master/cv_and_research_statement/cv.pdf).

## Selected Publications

<div class="publications-container">

<!-- Generated from _data/publications.yml -->
{% assign publications_by_year = site.data.publications | group_by: 'year' | sort: 'name' | reverse %}

{% for year_group in publications_by_year %}
<div class="year-header">{{ year_group.name }}</div>

{% assign year_publications = year_group.items %}
{% for pub in year_publications %}
<div class="publication-item">
    <div class="publication-image">
        {% assign media_src = pub.image | default: "/img/publications/BUFFER-X.gif" %}
        {% if media_src contains ".mp4" or media_src contains ".webm" %}
            <video autoplay loop muted playsinline preload="metadata">
                <source src="{{ media_src }}" type="video/{% if media_src contains '.webm' %}webm{% else %}mp4{% endif %}">
            </video>
        {% else %}
            <img src="{{ media_src }}" alt="{{ pub.title }}" loading="lazy">
        {% endif %}
    </div>
    <div class="publication-details">
        <div class="publication-title">{{ pub.title }}</div>
        <div class="publication-authors">
            {% assign authors = pub.authors | replace: pub.my_name, '<span class="my-name">Hyungtae Lim</span>' %}
            {{ authors }}
        </div>
        <div class="publication-venue">
            {% assign venue_name = pub.venue %}
            {% if pub.venue_key %}
                {% if pub.venue_type == 'journal' %}
                    {% assign venue_name = site.data.venues.journals[pub.venue_key] %}
                {% elsif pub.venue_type == 'conference' %}
                    {% assign venue_name = site.data.venues.conferences[pub.venue_key] %}
                {% endif %}
                {% if pub.venue_modifier %}
                    {% assign venue_name = venue_name | append: site.data.venues.modifiers[pub.venue_modifier] %}
                {% endif %}
            {% endif %}
            
            {% if pub.status == 'accepted_highlight' %}
                <span class="highlight">{{ venue_name }} (Highlight)</span>
            {% elsif pub.status == 'best_paper_award' %}
                <span class="highlight">{{ venue_name }} (Best Paper Award)</span>
            {% elsif pub.status == 'accepted' %}
                {{ venue_name }}
            {% elsif pub.status == 'under_review' %}
                <em>{{ venue_name }} (Under review)</em>
            {% else %}
                {{ venue_name }}
            {% endif %}
        </div>
        <div class="publication-links">
            {% if pub.arxiv %}
                <a href="{{ pub.arxiv }}" class="link-button primary">Paper</a>
            {% elsif pub.paper_link %}
                <a href="{{ pub.paper_link }}" class="link-button primary">Paper</a>
            {% elsif pub.ieee_link %}
                <a href="{{ pub.ieee_link }}" class="link-button primary">Paper</a>
            {% endif %}
            
            {% if pub.github %}
                <a href="{{ pub.github }}" class="link-button">
                    Code
                    <span class="github-stars" data-repo="{{ pub.github | replace: 'https://github.com/', '' }}">
                        <svg class="star-icon" viewBox="0 0 24 24">
                            <path d="M12 2l3.09 6.26L22 9.27l-5 4.87 1.18 6.88L12 17.77l-6.18 3.25L7 14.14 2 9.27l6.91-1.01L12 2z"/>
                        </svg>
                        <span class="star-count">-</span>
                    </span>
                </a>
            {% endif %}
            
            {% if pub.project_page %}
                <a href="{{ pub.project_page }}" class="link-button">Project Page</a>
            {% endif %}
            
            {% if pub.poster_link %}
                <a href="{{ pub.poster_link }}" class="link-button">Poster</a>
            {% endif %}
            
            {% if pub.video_link %}
                <a href="{{ pub.video_link }}" class="link-button">Video</a>
            {% endif %}
            
            {% if pub.slides_link %}
                <a href="{{ pub.slides_link }}" class="link-button">Slides</a>
            {% endif %}
            
            {% if pub.bibtex %}
                <a href="#" class="link-button" onclick="toggleBibtex('{{ pub.id }}')">BibTeX</a>
            {% endif %}
        </div>
        
        {% if pub.bibtex %}
        <div id="bibtex-{{ pub.id }}" class="bibtex-container" style="display: none;">
            <pre><code>{{ pub.bibtex }}</code></pre>
        </div>
        {% endif %}
    </div>
</div>
{% endfor %}
{% endfor %}

</div>

<script>
// Toggle BibTeX display
function toggleBibtex(pubId) {
    const bibtexDiv = document.getElementById('bibtex-' + pubId);
    if (bibtexDiv.style.display === 'none') {
        bibtexDiv.style.display = 'block';
    } else {
        bibtexDiv.style.display = 'none';
    }
}

// Fetch GitHub stars
async function fetchGitHubStars(repo) {
    try {
        const response = await fetch(`https://api.github.com/repos/${repo}`);
        if (response.ok) {
            const data = await response.json();
            return data.stargazers_count;
        }
    } catch (error) {
        console.log('Error fetching GitHub stars:', error);
    }
    return null;
}

// Update GitHub stars for all repositories
document.addEventListener('DOMContentLoaded', function() {
    const starElements = document.querySelectorAll('.github-stars');
    
    starElements.forEach(async function(element) {
        const repo = element.dataset.repo;
        if (repo) {
            const stars = await fetchGitHubStars(repo);
            const starCountElement = element.querySelector('.star-count');
            if (stars !== null && starCountElement) {
                starCountElement.textContent = stars.toLocaleString();
            }
            element.style.opacity = '1';
        }
    });
});
</script>
