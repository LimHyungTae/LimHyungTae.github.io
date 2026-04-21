# Pitfalls and diagnosis

Read this when a compression result looks wrong (banding, ghosting, janky motion, unexpected file size).

## Symptom: GIF colors look banded or dithered

You ran `ffmpeg` directly without the `palettegen`/`paletteuse` pair. GIF is a 256-color format, so without a custom palette you get the global default and it looks awful. Use `compress_gif.sh` — it has the palette filter built in. If you must invoke `ffmpeg` by hand, the filter chain is:

```
split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle
```

## Symptom: GIF motion looks janky after compression

You dropped too many frames. The default `compress_gif.sh` divider is 2 (keep half). Going past 3 (keep a third) on a clip that's already low-fps drops you below ~8fps where motion stops reading as continuous. Resize smaller instead, or accept a larger file.

## Symptom: PNG/JPG got bigger or didn't change

`convert -resize '640x480>'` only shrinks; the `>` is what prevents upscaling. If the source is already within budget, the output is just stripped of metadata — that's the only saving. If a PNG with big flat regions stays large, `pngquant --quality=65-85 in.png -o out.png` helps — but pngquant isn't installed on this machine, so install it first or accept the limit.

## Symptom: MP4 won't play in Safari / some browsers

You skipped `-pix_fmt yuv420p`. Most encoders default to `yuv444p` or similar, which Safari refuses. `compress_mp4.sh` sets it explicitly.

## Symptom: MP4 takes a long time to start playing on the site

The `moov` atom (metadata) is at the end of the file by default, so the browser has to download the whole thing before rendering. `-movflags +faststart` moves it to the front. `compress_mp4.sh` always sets it.

## Symptom: `identify` prints one line per frame for a GIF

`identify` is per-frame by default. Use `identify "${f}[0]"` to get just the first frame, or `ffprobe` instead. `audit.sh` already does the right thing.

## Symptom: Speeding up an MP4 segment broke audio sync

`setpts=PTS/N` speeds video but not audio — you need `atempo=N` on the audio side, or just `-an` to strip audio entirely (which is what `compress_mp4.sh` does by default since site thumbnails play muted).

## Tooling that's NOT installed

Don't reach for these — they're not on this machine:
- `gifsicle` — use `compress_gif.sh` (ffmpeg-based) instead
- `pngquant` — for now, accept the result of `compress_image.sh` alone
