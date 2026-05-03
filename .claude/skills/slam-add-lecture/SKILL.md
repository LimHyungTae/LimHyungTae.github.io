---
name: slam-add-lecture
description: Compress a finished lecture PDF with ghostscript, drop it into slam_slides/, and add a new Lecture NN entry to slam.md. Use this whenever the user says they "finished" or "completed" a new SLAM lecture, asks to "add Lec NN" / "Lecture NN을 추가", supplies a path to LecNN_SLAM_for_everyone.pdf, or otherwise wants to publish the next lecture on the /slam/ page. This is the canonical end-to-end "publish a new lecture" workflow for this repo — prefer it over running the gs command and editing slam.md by hand.
---

# Add a new SLAM lecture (compress PDF + edit slam.md)

This skill captures the full publish-a-new-lecture workflow: the source PDF lives in Dropbox, gets compressed by ghostscript, lands in the repo at `slam_slides/LecNN_SLAM_for_everyone.pdf`, and gets a `<div class="slam-section">` block inserted into `slam.md` so the toggle, language pairs, and link buttons all line up with the rest of the page.

## When to use

- User finished a new lecture and wants it published.
- User points at `LecNN_SLAM_for_everyone.pdf` (or a math supplementary PDF) and asks to add it.
- User says something like "Lec 10도 추가하자" / "compress and add Lec NN".

If the user only wants the slam.md edit (e.g., refreshing sub-items for a deck that's already in `slam_slides/`), use `slam-lecture-entry` directly instead.

## Inputs

- Lecture number `N` (e.g., 9, 10).
- Source PDF, normally at:
  ```
  /Users/fudxo/Library/CloudStorage/Dropbox/!lectures/모두를 위한 SLAM/slides/LecNN_SLAM_for_everyone.pdf
  ```
  For math supplementary decks: `LecNN_math_supplementary.pdf` in the same directory.
- (Optional) YouTube URL. If not provided, use the placeholder `https://limhyungtae.github.io/` — never fabricate a real YouTube ID.

## Steps

1. **Read the PDF.** Use the Read tool with `pages: "1-N"` to scan all slides. The sub-items for slam.md come from the **top-bar section markers** in the deck (the thin Beamer-style category bar above each content slide). See `slam-lecture-entry` for the detailed formatting rules — they apply here.

2. **Compress with ghostscript.** Use this exact command — the `/prepress` quality preserves figure clarity, which matters for the math-heavy decks:

   ```bash
   gs -sDEVICE=pdfwrite \
      -dCompatibilityLevel=1.4 \
      -dPDFSETTINGS=/prepress \
      -dPrinted=false \
      -dNOPAUSE -dQUIET -dBATCH \
      -sOutputFile="/Users/fudxo/git/LimHyungTae.github.io/slam_slides/LecNN_SLAM_for_everyone.pdf" \
      "/Users/fudxo/Library/CloudStorage/Dropbox/!lectures/모두를 위한 SLAM/slides/LecNN_SLAM_for_everyone.pdf"
   ```

   The output goes directly into `slam_slides/` (no underscore prefix — Jekyll excludes `_`-prefixed folders from the build). Verify the size after — typical compressed Lec PDFs are 1.3–5 MB.

3. **Edit slam.md.** Insert a new `<div class="slam-section">` block **immediately before** the "More lectures coming soon..." section (search for `강의는 계속 추가될 예정입니다` to locate it). Follow the layout rules from `slam-lecture-entry`:
   - One `<h3>` with bilingual `.lang-ko` + `.lang-en` title.
   - One `<li>` per top-bar segment.
   - Video + Slide buttons on the **first `<li>` only**.
   - Slide href: `/slam_slides/LecNN_SLAM_for_everyone.pdf` (local path, not Dropbox).
   - Video href: real YouTube URL if supplied, otherwise placeholder.

4. **For supplementary decks** (e.g., `LecNN_math_supplementary.pdf`): add a single extra `<li>` to the existing Lecture NN block with `<span class="lecture-num">Supp.</span>` and only a Slide button (no Video).

5. **Commit and push.** Stage both the new PDF and `slam.md`. Use a commit message like `Add Lecture NN: <Title>`. Push so GitHub Pages rebuilds.

## Do not

- Don't put the PDF in `_slam_slides/` (leading underscore → Jekyll excludes it from the published site).
- Don't link to Dropbox in slam.md anymore — the canonical slide URL is now the local `/slam_slides/...` path.
- Don't fabricate a YouTube URL when the lecture isn't recorded yet — the page hides Video buttons in English mode anyway, and Korean viewers should see the placeholder rather than a dead link.
- Don't restructure the existing toggle/CSS or reorder existing lectures.

## Verify

After committing:
- Run `grep -n "Lecture NN" slam.md` — exactly one block should match.
- Confirm the slide button href points at `/slam_slides/LecNN_...pdf` (no underscore prefix on the folder).
- After push, give GitHub Pages 1–3 min, then hard-refresh the live `/slam/` page.
