#!/usr/bin/env bash
# Compress an MP4 with H.264: bitrate-capped, audio stripped, faststart enabled.
# Bitrate drives size predictably — for ~5MB at 10s, use ~4000 kbps.
#
# Flags that matter:
#   -an             strips audio (site thumbnails autoplay muted)
#   -pix_fmt yuv420p required for Safari/most browsers to play the clip
#   +faststart      moves the moov atom up front so playback starts before full DL
#
# Writes in-place by default via tmp+mv.
#
# Usage: compress_mp4.sh INPUT [OUTPUT] [BITRATE_K]

set -euo pipefail

if [ $# -lt 1 ]; then
  echo "usage: $0 INPUT [OUTPUT] [BITRATE_K]" >&2
  exit 1
fi

in="$1"
out="${2:-$in}"
br="${3:-4000}"
maxrate=$(( br * 9 / 8 ))
bufsize=$(( br * 2 ))

tmp="$(mktemp -t compress_mp4_XXXXXX).mp4"
trap 'rm -f "$tmp"' EXIT

ffmpeg -y -i "$in" \
  -vf "scale='min(640,iw)':-2" \
  -c:v libx264 -b:v "${br}k" -maxrate "${maxrate}k" -bufsize "${bufsize}k" \
  -preset medium -pix_fmt yuv420p -movflags +faststart \
  -an "$tmp"

mv "$tmp" "$out"
