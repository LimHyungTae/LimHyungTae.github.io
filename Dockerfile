FROM jekyll/jekyll

FROM ruby:2.6

RUN apt-get update && apt-get install -y build-essential
# Bundler 버전 명시적 설치
RUN gem install bundler -v 1.17.1

COPY --chown=jekyll:jekyll Gemfile .
COPY --chown=jekyll:jekyll Gemfile.lock .

# RUN bundle install --quiet --clean
# RUN bundle _1.17.1_ install --quiet --clean
RUN bundle config set --local clean 'true'

CMD ["jekyll", "serve", "--host=0.0.0.0"]
