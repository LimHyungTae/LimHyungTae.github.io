---
name: compress-thumbnails
description: Resize and compress publication thumbnail images and videos (PNG, JPG, GIF, MP4) under `img/publications/` so the publications page stays fast on mobile. Use whenever the user adds new thumbnails, mentions oversized media on the site, asks to audit `img/publications/`, or wants to shrink a GIF/MP4 anywhere in the LimHyungTae.github.io repo — even if they don't say the word "compress" (e.g. "this gif is huge", "make this png smaller for the site", "thumbnail is 5MB").
---

# Compress publication thumbnails

`publications.md` renders thumbnails at **max 240×180** (desktop) and **max 350×220** (mobile), so anything served larger than ~640×480 wastes bytes. GIFs are the worst offender — without frame drops they balloon to tens of MB and hurt mobile users.

All work is delegated to scripts in `scripts/`. They write **in-place** by default (atomic via tmp-file + `mv`), or to `OUTPUT` if you pass a second arg. Read this file end-to-end before reaching into the scripts.

## Targets

| Asset type | Max dims | Size goal                    |
|------------|----------|------------------------------|
| PNG / JPG  | 640×480  | < 500 KB                     |
| GIF        | 640×480  | < 3 MB (after frame drop)    |
| MP4 / WebM | 640×480  | < 5 MB                       |

Aspect ratio is always preserved — never stretch.

## Tooling

`ffmpeg` and ImageMagick (`convert`, `identify`) are required and present. `gifsicle` and `pngquant` are NOT installed on this machine — don't reach for them.

## Workflow

The skill directory layout:

```
compress-thumbnails/
├── SKILL.md
├── scripts/
│   ├── audit.sh
│   ├── compress_image.sh
│   ├── compress_gif.sh
│   └── compress_mp4.sh
└── references/
    └── pitfalls.md   ← read only when a result looks wrong
```

Paths below are relative to the repo root.

### 1. Audit

List everything over budget:

```bash
bash .claude/skills/compress-thumbnails/scripts/audit.sh img/publications/
```

Output is `size dims filename`, sorted by size descending. Cross-reference with the targets table to pick candidates.

### 2. PNG / JPG — only when width > 640 OR height > 480

```bash
bash .claude/skills/compress-thumbnails/scripts/compress_image.sh img/publications/foo.png
```

Resizes to fit 640×480 (preserves aspect, never enlarges) and strips metadata. If a PNG with big flat regions is still over budget after this, accept it — `pngquant` isn't available here.

### 3. GIF — palette + frame drop

```bash
bash .claude/skills/compress-thumbnails/scripts/compress_gif.sh img/publications/foo.gif
# aggressive, for >15 MB originals: drop 2 of every 3 frames
bash .claude/skills/compress-thumbnails/scripts/compress_gif.sh img/publications/foo.gif foo.gif 3
```

The custom palette is what separates clean output from dithered banding. The script already includes it — don't shortcut it with a plain `ffmpeg -i in.gif out.gif`.

### 4. MP4 — H.264, audio stripped

```bash
bash .claude/skills/compress-thumbnails/scripts/compress_mp4.sh img/publications/foo.mp4
# custom bitrate in kbps (default 4000, ~5MB / 10s):
bash .claude/skills/compress-thumbnails/scripts/compress_mp4.sh img/publications/foo.mp4 foo.mp4 3000
```

Sets `yuv420p` (Safari requirement) and `+faststart` (browser starts rendering before full download).

### 5. Crop — case-by-case, no script

When you need to strip something specific (RViz toolbar, a watermark band), call ffmpeg directly — geometry is per-clip:

```bash
ffmpeg -y -i in.mp4 -vf "crop=W:H:X:Y" \
  -c:v libx264 -b:v 4000k -preset medium -pix_fmt yuv420p \
  -movflags +faststart -an out.mp4
```

`W×H` is the output size; `X,Y` is the top-left corner of the crop rectangle in source coords.

### 6. Verify and commit

Re-run `audit.sh` to confirm everything is under budget. Spot-check the rendered site (or branch preview) to make sure motion still reads after frame drops — past ~8 fps it stops looking continuous.

Overwrite originals in place when committing — don't add a `_compressed` suffix to checked-in files. One commit per batch keeps history clean.

## When something looks wrong

If a result has banding, ghosting, janky motion, or a surprising file size, read `references/pitfalls.md` for diagnosis. Don't load it preemptively — it's only useful when there's a symptom to match.
