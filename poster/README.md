# ft-tilt poster (posuta)

The ft-tilt poster, built with [posuta](https://github.com/alvgaona/posuta).

    pnpm install
    pnpm dev                 # http://localhost:4321
    pnpm build               # static site in dist/
    pnpm pdf                 # dist/poster.pdf, 423 × 853 mm (the original's page)

- Text: `src/pages/index.mdx`; references: `src/pages/refs.bib`.
- Charts: `charts/*.js` (Observable Plot) reading `data/*.csv`. Regenerate the CSVs from the
  bags with `pixi run python poster/data/export_data.py > poster/data/data.json` (from the repo root), then `python3 data/from-json.py`.
- Diagrams: edit `figures/src/*.svg` (maths as `$…$`), then `node figures/diagrams.mjs`.
- Look: `src/styles/theme.css`; header and footer: `src/layouts/Poster.astro`;
  the play button for section 1: `src/components/Player.astro`.
