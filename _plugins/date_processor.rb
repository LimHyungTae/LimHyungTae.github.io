module Jekyll
  class DateProcessor < Generator
    safe true
    priority :high

    def generate(site)
      publications = site.data['publications'] || []
      
      publications.each do |pub|
        # Create a sortable date for each publication
        pub['sort_date'] = create_sort_date(pub)
      end
      
      site.data['publications'] = publications
    end

    private

    def create_sort_date(pub)
      # Priority order for date sources:
      # 1. news_date (for sorting within year)
      # 2. date (publication date)
      # 3. Default to year only
      
      date_str = pub['news_date'] || pub['date']
      year = pub['year'] || 2024
      
      if date_str
        parsed_date = parse_date(date_str, year)
        return parsed_date.strftime('%Y-%m-%d')
      else
        # Default to December for publications without specific dates
        return "#{year}-12-31"
      end
    end

    def parse_date(date_str, year)
      case date_str.to_s.strip
      when /^(\w+\.?)\s+(\d{1,2})$/  # "Jul. 24", "July 25", "Feb. 25"
        month = $1
        day = $2.to_i
        month_num = parse_month(month)
        Date.new(year.to_i, month_num, day)
      when /^(\w+\.?)\s+(\d{1,2}),?\s+(\d{4})$/  # "Feb. 1, 2024"
        month = $1
        day = $2.to_i
        year = $3.to_i
        month_num = parse_month(month)
        Date.new(year, month_num, day)
      when /^(\d{4})-(\d{2})-(\d{2})$/  # "2024-07-15"
        Date.new($1.to_i, $2.to_i, $3.to_i)
      else
        # Default to December if we can't parse
        Date.new(year.to_i, 12, 31)
      end
    rescue
      Date.new(year.to_i, 12, 31)  # Fallback
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
      
      month_map[month_str] || 12
    end
  end
end