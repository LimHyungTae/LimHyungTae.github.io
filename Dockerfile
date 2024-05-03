FROM jekyll/jekyll

COPY --chown=jekyll:jekyll Gemfile .
COPY --chown=jekyll:jekyll Gemfile.lock .

# RUN bundle install --quiet --clean
RUN bundle config set --local clean 'true'

CMD ["jekyll", "serve"]
