# Publication-to-News Automation System

This system automatically generates news entries from publication updates in `_data/publications.yml`.

## How it works

### 1. Publication Configuration

In `_data/publications.yml`, each publication can have these automation fields:

```yaml
- id: "unique_publication_id"
  title: "Your Paper Title"
  authors: "Author List"
  venue: "Conference/Journal Name"
  status: "accepted"  # or "under_review", "published", "accepted_highlight"
  year: 2025
  generate_news: true  # Enable automatic news generation
  news_date: "July 25"  # When the news should appear
  news_template: "📝🏆 <a href='{arxiv}'>Paper Title</a> accepted at {venue}!"
  arxiv: "https://arxiv.org/abs/xxxx"
  github: "https://github.com/user/repo"
```

### 2. Template Variables

Available variables in `news_template`:
- `{title}` - Publication title
- `{venue}` - Publication venue
- `{arxiv}` - arXiv link
- `{github}` - GitHub repository link
- `{paper_link}` - Direct paper link
- `{ieee_link}` - IEEE Xplore link
- `{project_page}` - Project page link

### 3. Automatic News Generation

#### Method 1: Jekyll Plugin (Recommended)
The `_plugins/news_generator.rb` automatically generates news during Jekyll build.

#### Method 2: Manual Script
Run the update script manually:

```bash
cd /path/to/your/blog
ruby scripts/update_news.rb
```

### 4. News Display

The system automatically:
1. Checks existing news in `_data/news.yml`
2. Generates new entries for publications with `generate_news: true`
3. Combines manual and auto-generated news
4. Sorts by date (most recent first)

### 5. Adding New Publications

To add a new publication that should generate news:

1. Add the publication to `_data/publications.yml` with:
   - Unique `id`
   - `generate_news: true`
   - `news_date` when you want the announcement
   - `news_template` with your desired message format
   
2. The news will be automatically generated on next build

### 6. Manual News Entries

You can still add manual news entries to `_data/news.yml`:

```yaml
- date: "Aug. 25"
  content: "🎤 Gave a seminar at XYZ University"
  # No pub_id means this is manual
```

### 7. Preventing Duplicates

The system uses `pub_id` to track which publications already have news entries, preventing duplicates.

## Example Workflow

1. **New paper accepted**: Update `_data/publications.yml` with `status: "accepted"` and set `news_date`
2. **News auto-generated**: System creates appropriate news entry
3. **Manual updates**: Add any additional context to `_data/news.yml` if needed

## Benefits

- ✅ Consistent news formatting
- ✅ No manual duplication of publication info
- ✅ Automatic chronological ordering
- ✅ Support for both manual and auto-generated news
- ✅ Easy to maintain and update
- ✅ Prevents forgotten news announcements