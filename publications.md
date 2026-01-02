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
    flex: 0 0 240px;
    margin-right: 30px;
    text-align: center;
    display: flex;
    align-items: center;
    justify-content: center;
}

.publication-image img {
    max-width: 240px;
    max-height: 180px;
    width: auto;
    height: auto;
    object-fit: contain;
    border-radius: 8px;
    box-shadow: 0 2px 8px rgba(0,0,0,0.1);
    background-color: #f8f9fa;
    padding: 10px;
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
    display: flex;
    flex-wrap: wrap;
    gap: 8px;
}

.publication-links .link-button {
    display: inline-block;
    padding: 4px 10px;
    background-color: #f8f9fa;
    border: 1px solid #dee2e6;
    border-radius: 4px;
    color: #495057;
    text-decoration: none;
    font-size: 13px;
    font-weight: 500;
    transition: all 0.2s ease;
}

.publication-links .link-button:hover {
    background-color: #e9ecef;
    border-color: #adb5bd;
    color: #212529;
    text-decoration: none;
}

.publication-links .link-button.primary {
    background-color: #1B2C8F;
    border-color: #1B2C8F;
    color: white;
}

.publication-links .link-button.primary:hover {
    background-color: #142373;
    border-color: #142373;
    color: white;
}

.publication-links .github-stars {
    display: inline-flex;
    align-items: center;
    gap: 4px;
    font-size: 12px;
    color: #6c757d;
}

.publication-links .github-stars .star-icon {
    width: 12px;
    height: 12px;
    fill: #ffc107;
}

.year-header {
    font-size: 24px;
    font-weight: bold;
    margin: 40px 0 20px 0;
    padding-bottom: 10px;
    border-bottom: 2px solid #6c757d;
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
        text-align: center;
    }
    
    .publication-image img {
        max-width: 350px;
        max-height: 220px;
        width: auto;
        height: auto;
    }
    
    .publication-links {
        flex-direction: column;
        align-items: flex-start;
    }
}

@media (max-width: 480px) {
    .publication-image img {
        max-width: 300px;
        max-height: 200px;
    }
}

.bibtex-container {
    margin-top: 10px;
    padding: 10px;
    background-color: #f8f9fa;
    border: 1px solid #dee2e6;
    border-radius: 4px;
    font-size: 12px;
}

.bibtex-container pre {
    margin: 0;
    white-space: pre-wrap;
    word-wrap: break-word;
}
</style>

Please refer to my [Google Scholar](https://scholar.google.com/citations?user=S1A3nbIAAAAJ&hl=en&oi=ao), [CV](https://github.com/LimHyungTae/LimHyungTae.github.io/blob/master/cv_and_research_statement/cv.pdf), and [Research Statement](https://github.com/LimHyungTae/LimHyungTae.github.io/blob/master/cv_and_research_statement/research_statement.pdf) (updated on 2025-Sep-22).

## Selected Publications

(On updating figures and meta information...)

<div class="publications-container">

<!-- Generated from _data/publications.yml -->
{% assign publications_by_year = site.data.publications | group_by: 'year' | sort: 'name' | reverse %}

{% for year_group in publications_by_year %}
<div class="year-header">{{ year_group.name }}</div>

{% assign year_publications = year_group.items %}
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
                element.style.opacity = '1';
            } else {
                element.style.display = 'none';
            }
        }
    });
});
</script>
