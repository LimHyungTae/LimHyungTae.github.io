#!/usr/bin/env bash
# Compress a GIF by resizing to fit 640x480 and dropping every Nth frame.
# Uses palettegen/paletteuse — naive ffmpeg re-encoding produces banded output.
# Writes in-place by default via tmp+mv (atomic).
#
# FPS_DIVIDER:
#   2 (default) — keep half the frames
#   3           — keep a third (use for >15MB originals)
#
# Usage: compress_gif.sh INPUT [OUTPUT] [FPS_DIVIDER]

set -euo pipefail

if [ $# -lt 1 ]; then
  echo "usage: $0 INPUT [OUTPUT] [FPS_DIVIDER]" >&2
  exit 1
fi

in="$1"
out="${2:-$in}"
div="${3:-2}"

tmp="$(mktemp -t compress_gif_XXXXXX).gif"
trap 'rm -f "$tmp"' EXIT

ffmpeg -y -i "$in" \
  -vf "fps=source_fps/${div},scale='min(640,iw)':'min(480,ih)':force_original_aspect_ratio=decrease:flags=lanczos,split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle" \
  -loop 0 "$tmp"

mv "$tmp" "$out"
