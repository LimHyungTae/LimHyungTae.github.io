module Jekyll
  class NewsGenerator < Generator
    safe true
    priority :high

    def generate(site)
      # Get publication data
      publications = site.data['publications'] || []
      
      # Get existing news
      existing_news = site.data['news'] || []
      
      # Generate news entries from publications
      generated_news = []
      
      publications.each do |pub|
        next unless pub['generate_news'] && pub['news_template'] && pub['news_date']
        
        # Check if news already exists for this publication
        existing_entry = existing_news.find { |news| news['pub_id'] == pub['id'] }
        next if existing_entry
        
        # Generate news entry
        news_content = generate_news_content(pub)
        
        generated_news << {
          'date' => pub['news_date'],
          'content' => news_content,
          'pub_id' => pub['id'],
          'auto_generated' => true
        }
      end
      
      # Sort all news by date (most recent first)
      all_news = (existing_news + generated_news).sort_by do |news|
        # Convert date strings to comparable format
        parse_date(news['date'])
      end.reverse
      
      # Update site data
      site.data['all_news'] = all_news
      site.data['generated_news'] = generated_news
    end

    private

    def generate_news_content(pub)
      template = pub['news_template']
      
      # Replace template variables
      content = template.dup
      content = content.gsub('{title}', pub['title'] || '')
      content = content.gsub('{venue}', pub['venue'] || '')
      content = content.gsub('{arxiv}', pub['arxiv'] || '')
      content = content.gsub('{github}', pub['github'] || '')
      content = content.gsub('{paper_link}', pub['paper_link'] || '')
      content = content.gsub('{ieee_link}', pub['ieee_link'] || '')
      content = content.gsub('{project_page}', pub['project_page'] || '')
      
      content
    end

    def parse_date(date_str)
      # Handle various date formats
      case date_str
      when /^(\w+\.?)\s+(\d{2})$/  # "Jul. 24", "July 25"
        month = $1
        year = "20#{$2}"
        month_num = parse_month(month)
        Date.new(year.to_i, month_num, 1)
      when /^(\d{4})-(\d{2})-(\d{2})$/  # "2024-07-15"
        Date.new($1.to_i, $2.to_i, $3.to_i)
      else
        Date.new(2024, 1, 1)  # Default fallback
      end
    rescue
      Date.new(2024, 1, 1)  # Fallback for any parsing errors
    end

    def parse_month(month_str)
      month_map = {
        'Jan' => 1, 'Jan.' => 1, 'January' => 1,
        'Feb' => 2, 'Feb.' => 2, 'February' => 2,
        'Mar' => 3, 'Mar.' => 3, 'March' => 3,
        'Apr' => 4, 'Apr.' => 4, 'April' => 4,
        'May' => 5, 'May.' => 5,
        'Jun' => 6, 'Jun.' => 6, 'June' => 6,
        'Jul' => 7, 'Jul.' => 7, 'July' => 7,
        'Aug' => 8, 'Aug.' => 8, 'August' => 8,
        'Sep' => 9, 'Sep.' => 9, 'September' => 9,
        'Oct' => 10, 'Oct.' => 10, 'October' => 10,
        'Nov' => 11, 'Nov.' => 11, 'November' => 11,
        'Dec' => 12, 'Dec.' => 12, 'December' => 12
      }
      
      month_map[month_str] || 1
    end
  end
end