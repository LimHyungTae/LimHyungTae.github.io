#!/usr/bin/env bash
# Resize a PNG/JPG to fit within 640x480, preserving aspect ratio, never enlarging.
# Strips metadata. Writes in-place by default; pass OUTPUT to write elsewhere.
#
# The '>' on -resize is what prevents upscaling small images — keep it.
#
# Usage: compress_image.sh INPUT [OUTPUT]

set -euo pipefail

if [ $# -lt 1 ]; then
  echo "usage: $0 INPUT [OUTPUT]" >&2
  exit 1
fi

in="$1"
out="${2:-$in}"

tmp="$(mktemp -t compress_image_XXXXXX)"
ext="${in##*.}"
mv "$tmp" "${tmp}.${ext}"
tmp="${tmp}.${ext}"
trap 'rm -f "$tmp"' EXIT

convert "$in" -resize '640x480>' -strip -quality 85 "$tmp"
mv "$tmp" "$out"
