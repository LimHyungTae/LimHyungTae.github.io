---
name: compress-thumbnails
description: Resize and compress publication thumbnail images (PNG, JPG, GIF, MP4) in img/publications/ so page-load payloads stay small. Use whenever new thumbnails are added or when auditing existing ones.
---

# Compress publication thumbnails

The `publications.md` template renders thumbnails at **max 240×180** (desktop) and **max 350×220** (mobile). Anything delivered larger than ~640×480 is wasted bytes. Heavy GIFs can easily hit tens of MB, which is painful for users on mobile data.

## Targets

| Asset type | Max width | Max height | Typical size goal |
|------------|-----------|------------|-------------------|
| PNG / JPG  | 640 px    | 480 px     | < 500 KB          |
| GIF        | 640 px    | 480 px     | < 3 MB (after frame drop) |
| MP4 / WebM | 640 px    | 480 px     | < 5 MB            |

Preserve aspect ratio — never stretch.

## Tooling

`ffmpeg`, ImageMagick (`convert`, `identify`) — both available via Homebrew. `gifsicle` is *not* installed on this machine; use `ffmpeg` for GIFs instead.

## Workflow

1. **Audit** — list files over budget. Run from `img/publications/`:
   ```bash
   for f in *.gif *.png *.jpg *.jpeg *.mp4; do
     [ -f "$f" ] || continue
     size=$(stat -f%z "$f")
     # first frame only (for GIFs/videos identify prints every frame)
     dims=$(identify -format "%wx%h" "$f[0]" 2>/dev/null || ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of csv=p=0:s=x "$f")
     printf "%10d  %-12s  %s\n" "$size" "$dims" "$f"
   done | sort -rn
   ```

2. **Resize PNG/JPG** — only if width > 640 OR height > 480. Preserve aspect ratio with `>` operator (ImageMagick only shrinks, never enlarges):
   ```bash
   convert input.png -resize '640x480>' -strip -quality 85 output.png
   ```
   For PNGs with big flat regions, `pngquant --quality=65-85 input.png -o output.png` can cut size further.

3. **Compress GIF (resize + frame drop)** — drop every other frame (keeps half the frames, halves filesize floor) and clamp dims:
   ```bash
   ffmpeg -y -i input.gif \
     -vf "fps=source_fps/2,scale='min(640,iw)':'min(480,ih)':force_original_aspect_ratio=decrease:flags=lanczos,split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle" \
     -loop 0 output.gif
   ```
   Notes:
   - `source_fps/2` drops every other frame. Use `source_fps/3` for aggressive (>15 MB originals).
   - The palette split preserves colors — a naive `ffmpeg -i in.gif out.gif` looks terrible.
   - If the GIF is still too large, halve width (`min(480,iw)` or lower) before dropping more frames.

4. **Compress MP4** — size-target via bitrate, not CRF, when you need a specific cap:
   ```bash
   # ~5 MB target: bitrate = size_bits / duration_seconds. For 5MB / 10s ≈ 4000k.
   ffmpeg -y -i input.mp4 \
     -vf "scale='min(640,iw)':-2" \
     -c:v libx264 -b:v 4000k -maxrate 4500k -bufsize 8000k \
     -preset medium -pix_fmt yuv420p -movflags +faststart \
     -an output.mp4
   ```
   - `-an` strips audio (thumbnails autoplay muted anyway).
   - `-pix_fmt yuv420p` is required for browser/Safari playback.
   - `+faststart` puts the moov atom up front so the browser starts rendering before full download.
   - If the clip can be sped up (non-critical footage), use `setpts=PTS/N` inside `filter_complex`; preserve specific time ranges by concatenating `trim` segments.

5. **Crop when needed** — e.g. stripping RViz toolbar from the top:
   ```bash
   ffmpeg -y -i input.mp4 -vf "crop=W:H:X:Y" -c:v libx264 -b:v 4000k ... output.mp4
   ```
   where `W×H` is the output size and `X,Y` is the top-left of the crop rectangle in the source.

6. **Verify** — re-run the audit; open the site locally (or push to a branch) and spot-check that motion still reads clearly after frame drops.

7. **Commit** — overwrite the original in place (don't add `_compressed` suffix to the checked-in file); a single commit per batch keeps history clean.

## Pitfalls

- `identify` on a multi-frame GIF lists every frame. Use `file[0]` for just the first frame, or `ffprobe` instead.
- `convert -resize 640x480` without `>` will upscale small images — always include `>`.
- Naive GIF re-encoding without a palette filter produces dithered, banded output. Always use the `palettegen`/`paletteuse` pair.
- Dropping frames below ~8 fps makes motion look janky. If a GIF is already low fps, resize instead.
- Speeding up a video segment with `setpts` requires dropping the matching audio with `atempo` — or just use `-an`.
