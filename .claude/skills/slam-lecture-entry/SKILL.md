---
name: slam-lecture-entry
description: Fill in (or refresh) a lecture entry under /slam/ in slam.md. Each lecture's sub-items are derived from the top-bar section markers of its Beamer-style slide deck. Use when a new Lec NN deck is finalized or when existing placeholders need to be replaced with real sub-topics.
---

# Fill a SLAM lecture entry in slam.md

The `/slam/` page (`slam.md`) lists each lecture as a `<div class="slam-section">`:
- One `<h3>` with the lecture title (bilingual `.lang-ko` + `.lang-en` pair)
- A `<ul class="lecture-list">` with one `<li>` per **sub-item**
- The first `<li>` of a lecture carries the Video + Slide buttons; remaining sub-items are text-only (the slide deck is shared, so repeating the buttons on every row is noisy)

The **sub-items come from the top-bar section markers** of the lecture deck — the thin horizontal category bar that Beamer-style decks show above each content slide (e.g., `Robot navigation | Perception | SLAM | Conclusion`). Each unique segment of that bar becomes one sub-item.

## Inputs

- Lecture number `N`
- The compressed slide PDF inside the repo at `slam_slides/LecNN_SLAM_for_everyone.pdf`. The canonical slide href is the local path `/slam_slides/LecNN_SLAM_for_everyone.pdf` (Dropbox links are no longer used). If the PDF is not yet compressed/placed, use the `slam-add-lecture` skill instead — it bundles the gs compression step with this entry edit.
- The source PDF for reading top-bar markers. Canonical Dropbox path:
  ```
  /Users/fudxo/Library/CloudStorage/Dropbox/!lectures/모두를 위한 SLAM/slides/LecNN_SLAM_for_everyone.pdf
  ```

## Steps

1. **Read the PDF.** Use the Read tool with `pages: "1-N"` to pull all slides. Scan the top edge of each content slide for the category bar.

2. **Extract sub-items** — one per unique top-bar segment, in the order they appear.
   - If a deck has no top bar (pure overview / intro decks), keep the lecture as a single sub-item that summarizes the deck. Don't force a split.
   - Use the section's label verbatim (or a tight paraphrase) — do not invent finer structure.

3. **Write both languages — briefly, outline-style (개조식).** Korean does not need to be a full translation; a clipped phrase mirroring the English is fine (and often clearer). Examples of the right register:
   - `Robot navigation 구성 요소` / `Components of robot navigation`
   - `Example A – MLE: 평균 기반 키 추정` / `Example A – MLE: height estimation via average`
   - `결론: SLAM ⊂ Robot navigation` / `Conclusion: SLAM ⊂ robot navigation`

   No hype adjectives. No full sentences. Keep each line short enough to fit on one row at desktop width.

4. **Edit `slam.md`.** Replace the placeholder `<div class="slam-section">` block with:

   ```html
   <div class="slam-section">
   <h3><span class="lang-ko">Lecture N: <Korean title></span><span class="lang-en">Lecture N: <English title></span></h3>
   <ul class="lecture-list">
     <li>
       <span class="lecture-title"><span class="lecture-num">N-1.</span>
         <span class="lang-ko"><ko sub-topic 1></span>
         <span class="lang-en"><en sub-topic 1></span>
       </span>
       <span class="lecture-links">
         <a href="<video_url_or_placeholder>" class="btn-video">Video</a>
         <a href="/slam_slides/LecNN_SLAM_for_everyone.pdf" class="btn-slide">Slide</a>
       </span>
     </li>
     <li>
       <span class="lecture-title"><span class="lecture-num">N-2.</span>
         <span class="lang-ko"><ko sub-topic 2></span>
         <span class="lang-en"><en sub-topic 2></span>
       </span>
     </li>
     <!-- …N-M for each remaining top-bar category; NO .lecture-links on these -->
   </ul>
   </div>
   ```

   - Video placeholder (not yet recorded): `https://limhyungtae.github.io/`. Do not fabricate a YouTube URL — if the user hasn't supplied one, keep the placeholder.
   - If the lecture H3 still has a `(TBU)` suffix, remove it once real sub-items are in place.

5. **Preserve ordering.** Lectures are listed in numeric order in `slam.md`. If you're replacing a placeholder block (e.g., Lec 4 currently shows as TBU with a single row), edit in place — don't append a duplicate.

6. **Verify.**
   - Grep `slam.md` for `"Lecture N"` to confirm exactly one block exists.
   - Check that each `.lang-ko` has a matching `.lang-en` sibling (the toggle relies on pairs).
   - Eyeball the rendered page: the language toggle at the top of `/slam/` should swap every sub-item line.

## Do not

- Do not restructure the h3/li/links layout (the page's CSS expects buttons inside the first `<li>`).
- Do not add Video/Slide buttons to every sub-item — only the first of each lecture.
- Do not translate the top-bar English verbatim into Korean if the English is already a recognized technical term. Mixed-script ko lines like `Perception vs. Cognition` read better than `지각 vs. 인지`.
- Do not edit other lectures' blocks. Scope each run to the one lecture you were asked about.
