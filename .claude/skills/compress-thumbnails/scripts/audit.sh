#!/usr/bin/env bash
# Lists thumbnail assets in DIR (default: cwd) with size + dimensions,
# sorted by size descending. Anything past the targets in SKILL.md is a candidate.
#
# Usage: audit.sh [DIR]

set -euo pipefail
shopt -s nullglob nocaseglob

dir="${1:-.}"
cd "$dir"

for f in *.gif *.png *.jpg *.jpeg *.mp4 *.webm; do
  [ -f "$f" ] || continue
  size=$(stat -f%z "$f" 2>/dev/null || stat -c%s "$f")
  dims=$(identify -format "%wx%h" "${f}[0]" 2>/dev/null \
      || ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of csv=p=0:s=x "$f" 2>/dev/null \
      || echo "?x?")
  printf "%10d  %-12s  %s\n" "$size" "$dims" "$f"
done | sort -rn
