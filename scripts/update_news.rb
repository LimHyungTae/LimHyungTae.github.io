#!/usr/bin/env ruby

require 'yaml'

# Read publications data
publications_file = '_data/publications.yml'
news_file = '_data/news.yml'

unless File.exist?(publications_file)
  puts "Error: #{publications_file} not found"
  exit 1
end

unless File.exist?(news_file)
  puts "Creating new #{news_file}"
  File.write(news_file, [].to_yaml)
end

publications = YAML.load_file(publications_file) || []
existing_news = YAML.load_file(news_file) || []

# Generate news from publications
new_news_entries = []

publications.each do |pub|
  next unless pub['generate_news'] && pub['news_template'] && pub['news_date']
  
  # Check if news already exists
  existing_entry = existing_news.find { |news| news['pub_id'] == pub['id'] }
  next if existing_entry
  
  # Generate news content
  content = pub['news_template'].dup
  content.gsub!('{title}', pub['title'] || '')
  content.gsub!('{venue}', pub['venue'] || '')
  content.gsub!('{arxiv}', pub['arxiv'] || '')
  content.gsub!('{github}', pub['github'] || '')
  content.gsub!('{paper_link}', pub['paper_link'] || '')
  content.gsub!('{ieee_link}', pub['ieee_link'] || '')
  content.gsub!('{project_page}', pub['project_page'] || '')
  
  new_entry = {
    'date' => pub['news_date'],
    'content' => content,
    'pub_id' => pub['id'],
    'auto_generated' => true
  }
  
  new_news_entries << new_entry
  puts "Generated news for: #{pub['title']}"
end

if new_news_entries.empty?
  puts "No new news entries to generate"
else
  # Combine with existing news
  all_news = existing_news + new_news_entries
  
  # Sort by date (most recent first)
  all_news.sort! do |a, b|
    parse_date(b['date']) <=> parse_date(a['date'])
  end
  
  # Write back to file
  File.write(news_file, all_news.to_yaml)
  puts "Updated #{news_file} with #{new_news_entries.length} new entries"
end

def parse_date(date_str)
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