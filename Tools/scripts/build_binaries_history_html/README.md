# build_binaries_history_html

Generate a **static, fully-offline HTML site** from a `build_binaries` history
SQLite database (the one produced by `Tools/scripts/build_binaries_history.py`).
The tool is meant to run periodically against the database; each run regenerates
the whole site.

The browser viewing the output has **no access to the database** — every page is
self-contained (its chart data is inlined into the page), so the tree can be
opened directly from local disk (`file://`) or served over plain HTTP.

```sh
Tools/scripts/build_binaries_history_html/build_binaries_history_html.py \
    --db build_binaries_history-YYYYMMDD.sqlite --out /path/to/site
# then open /path/to/site/index.html
```

A full run over the 2020–2026 DB (~3.1 M rows) takes ~15 s and ~25 MB RAM.

---

## 1. Repository layout

```text
build_binaries_history_html/
  build_binaries_history_html.py   # the generator (Python 3, stdlib only)
  assets/
    uplot.min.js / uplot.min.css   # vendored uPlot (MIT, ~50 KB) — third-party
    app.js                         # OUR front-end: builds charts from inlined data
    style.css                      # OUR shared styles
  README.md                        # this file
```

`assets/` is copied verbatim into `<out>/assets/` on every run (`copy_assets`).
Edit `app.js`/`style.css` here, not in the generated output — the output is
disposable and overwritten.

This script reads the **hwdef tree of the repo it lives in** (for flash sizes),
so keep it inside an ArduPilot checkout. `REPO_ROOT`/`HWDEF_DIR` are derived from
`__file__`.

---

## 2. How it works (architecture & data flow)

Everything lives in one class, `SiteGenerator`, driven by `generate()`. The
pipeline, in order:

1. `prepare_out()` — delete only the trees/files we manage in `--out` (so a
   re-run leaves no orphans), then recreate the dir.
2. `board_flash_totals()` — compute each board's program-flash region once, via
   the real HWDef system (see §6). Skipped with `--no-flash-free`.
   `latest_run_hash()` records the newest *completed* latest-tag run (a hash in
   the `run` table; see §11) — the reference `flush_board()` uses for each
   board's coverage status and flash size.
3. **Streaming scan.** `build_query()` returns a `SELECT … ORDER BY (vehicle,
   board, tag, frame, start_time)`. The loop accumulates rows for the *current*
   `(vehicle, board)` into a `series` dict keyed `"<tag>|<frame>"`, and when the
   key changes it calls `flush_board()`. Because the query is ordered, all rows
   for a board are contiguous, so only one board is ever held in memory — this is
   why memory stays flat regardless of DB size. Global aggregates (per-vehicle
   board lists, coverage, monthly build counts, totals) are accumulated here too.
4. `flush_board()` — for one board: compute `compute_changes()` (notable size
   jumps), assemble the per-board data dict (including `flash_region`), record
   the board's latest-tag status for the index/coverage pages, and write the
   board detail page via `write_board_page()` (data **inlined**, see §5).
5. Top-level pages, each a `write_*` method. **Note the filenames don't match
   the method names:** `write_coverage` writes the Coverage **landing page** to
   `index.html`, and `write_index` writes the Overview dashboard to
   `overview.html`. In call order: `write_vehicle_indexes`, `write_coverage`
   (→ `index.html`), `write_trends` (the `failures.html` build-trend grid, fed by
   `build_trends()`), `write_flash`, `write_runs` (run charts, fed by
   `load_runs()`, `runs_commits_since_last()`, `runs_build_counts()`),
   `write_index` (→ `overview.html`).
6. `copy_assets()` — copy `assets/` into the output.

HTML is built by string concatenation with `html.escape`; there is no template
engine. `page_head(title, root, active)` / `page_foot()` wrap every page;
`root` is the relative path back to the site root (`""` for top-level pages,
`"../../"` for `vehicles/<v>/<b>.html`).

---

## 3. Input: database schema (read-only — never written)

Two tables matter (writer: `build_binaries_history.py`):

- `build(hash, tag, vehicle, board, frame, text, data, bss, start_time, duration)`
  — one row per (commit, tag, vehicle, board, frame) build. `text/data/bss` are
  ELF section sizes in bytes; **NULL sizes mean the build failed / produced no
  binary**. `start_time` is a unix epoch float; `duration` is seconds.
- `run(hash, tag, start_time, duration)` — one row per whole build_binaries run.

The DB is opened read-only (`file:…?mode=ro`); do not add writes. There is **no**
flash-capacity or `flash_free` column — flash headroom is derived from the
hwdefs instead (§6).

Rough scale: ~3.1 M build rows, ~10 k runs, 8 raw vehicle names, ~446 boards,
4 tags (`latest` dominates; also `beta`, `beta-4.3`, `stable`), frame mostly
NULL or `heli`.

---

## 4. Output: page tree

```text
index.html        Coverage (the landing page): board × vehicle matrix of status
                  at the most recent COMPLETED latest-tag run (see §11), split
                  into "Boards with failures" (top) and "Boards with no failures".
                  Boards built by no vehicle at that run (renamed/retired/removed
                  from autobuild, e.g. 2RAWH743 -> IFLIGHT_2RAW_H7) are omitted;
                  the Build trends page covers per-run history for boards built
                  within its --trend-runs window.
overview.html     Overview dashboard: count cards, vehicle list, builds-per-month
                  chart (linked from the right-most "Overview" tab)
failures.html     "Build trends": one row per board, a strip of per-run boxes
                  (newest left) coloured built/failed/not-built, each linking to
                  the commit; rows sorted most-recently-failed first. A top
                  "completed" row marks which runs finished (✓) vs partial. An
                  optional checkbox also interleaves the git commits we did NOT
                  build (see §8), rendered on demand.
flash.html        "Flash free": per (vehicle, board) program-flash headroom for
                  the latest build, tightest first, low-headroom rows flagged
runs.html         build_binaries run duration over time (by tag), boards per run
                  and builds per run (completed latest-tag runs only, so partial
                  runs don't plot as near-zero spikes), and "commits since
                  previous run" (git commits between consecutive latest-tag runs)
vehicles/<vehicle>/index.html    board list + latest status for a vehicle
vehicles/<vehicle>/<board>.html  per-board detail: Flash free (or Flash usage
                                 when the region is unknown) + build duration
                                 charts + a "notable size changes" table
assets/...        copied from assets/
```

There are **no separate `.json` data files** — each page inlines its chart data
in a `<script>` (`inline_json`, which escapes `<`/`>`/`&`). This is what lets
pages work from `file://`, because browsers block `fetch()` of `file://` URLs.

---

## 5. The inlined-data contract (page ↔ app.js)

The generator emits small globals that `app.js` reads. If you change a shape,
change both sides.

**Board detail page** (`write_board_page` → `BBH_boardDetail`):

```js
window.BBH = { commitBase };          // GitHub commit URL prefix
window.BBH_DATA = {
  vehicle, board,
  flash_region: <int bytes | null>,   // program-flash region, drives Flash-free
  series: {                           // key = "<tag>|<frame>"
    "latest|default": {
      t:    [unix_s, ...],            // x axis
      text: [int|null, ...],          // null => failed build
      data: [int|null, ...],
      bss:  [int|null, ...],
      dur:  [sec|null, ...],
      hash: ["abcdef123456", ...]     // 12-char SHA, used for tooltip + click
    }, ...
  },
  changes: [ {t, hash, metric, delta, series}, ... ]   // notable size jumps
};
```

**Single-series pages** (overview build-volume, runs → `BBH_simpleChart`):

```js
window.BBH_SIMPLE = cfg | [cfg, ...];   // one chart, or several (runs has four)
// cfg = {
//   host: "<element id>", title, fmt,  // "bytes"|"duration"|"secs"|"pct"|"count"
//   data: { series: { "<label>": { t:[...], v:[...], hash:[...] } } },
//   commitBase?   // if set, points hover/click through to the commit
// }
```

Both fall back to `fetch(dataUrl)` if `data`/`BBH_DATA` is absent, so an
HTTP-served variant could split data into files later without front-end changes.

---

## 6. Flash-free via the HWDef system

The DB has no flash capacity, so `board_flash_totals()` runs the real
`libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py` (same code the firmware
build uses) per board and reads `env_vars['FLASH_TOTAL']` — the internal
program-flash region. For external-flash-primary boards it adds the external
region (`HAS_EXTERNAL_FLASH_SECTIONS` + `EXT_FLASH_SIZE_MB` − reserves), because
`.text/.data` span both. Free = region − (text+data).

We deliberately did **not** use a hand-rolled hwdef parser: a cross-check across
all boards showed a lightweight parser silently over-estimates the region by the
storage area (~256 KB) on the 65 boards that set `STORAGE_FLASH_PAGE` without an
explicit `FLASH_RESERVE_END_KB` — i.e. it *under-warns*, the unsafe direction.
The HWDef path costs ~2.5 s for all ~424 boards, once per run (not per build
row), with 0 errors. Boards with no hwdef (Linux/SITL) and any build whose size
exceeds the modelled region are omitted from `flash.html`.

If you bump uPlot or refactor and the numbers look wrong, sanity-check against
the known-tight boards: `Pixhawk1-1M`, the `f103-*` periph nodes, F405 wings —
these should sit at the top of `flash.html` with single-digit-KB headroom.

---

## 7. Extending the tool

**Add a new top-level page** — four edits, and forgetting any one is the usual
bug:

1. Write a `write_<name>()` method that builds the page and calls
   `self.write_page("<name>.html", "".join(body))`.
2. Call it from `generate()`.
3. Add `("<name>.html", "<Nav label>")` to the `nav` list in `page_head()`.
4. Add `"<name>.html"` to the cleanup list in `prepare_out()` (else stale copies
   linger across runs).

**Add a chart to the board page** — compute an aligned series in
`BBH_boardDetail` (`app.js`) with `align(d.series, keys, fn)` and call
`makeChart(host, title, keys, aligned, fmtFn, cfg.commitBase)`. `align()` builds
the union x-axis and the parallel hash grid used by tooltips/click.

**Add a value format** — add a `fmtX` function in `app.js` and a case in
`pickFmt`; reference it via `BBH_SIMPLE.fmt`.

**Add a per-build field to charts** — it must be carried in the inlined `series`
(see §5), which is populated in the streaming loop in `generate()`.

Keep the generator stdlib-only and the output dependency-free.

---

## 8. Front-end (`assets/app.js`)

- Entry points (all exposed on `window`): `BBH_boardDetail()` (board detail
  pages), `BBH_simpleChart()` (overview build-volume + the four runs charts),
  `BBH_trend()` (build-trends checkbox; see below).
- `align(seriesMap, keys, metricFn)` → `{data, hashGrid}`: merges all series
  onto one sorted union x-axis (each series is NULL where it has no sample). Plot
  series use `spanGaps: true` so the union NULLs don't blank the line.
- `makeChart(...)` builds a uPlot chart, wires `tooltipPlugin`, and handles
  resize.
- `tooltipPlugin` shows a floating tooltip (date, each visible series value, a
  short-hash commit link) **and** makes a click on the hovered point open that
  commit on GitHub. It tracks the hovered hash in `setCursor`, sets a pointer
  cursor, and a small mousedown→click distance guard suppresses the click after
  a zoom-drag.
- Formatters: `fmtBytes`, `fmtDuration` (s/min/h), `fmtSecs`, `fmtPct`,
  `fmtCount`, selected by `pickFmt`.
- `trendView()` (driven by `window.BBH_TREND`) powers the build-trends checkbox.
  The default page is the server-rendered built-only grid; ticking the box
  builds — once, on demand — a much larger grid that interleaves every git
  commit in the window (built + unbuilt). Because that grid can be hundreds of
  thousands of cells, cells carry no per-cell `href`/`title`: a single delegated
  `click`/`mouseover` on the container opens the commit / sets the tooltip,
  using the cell's column index into the inlined `hashes`/`dates`. The dataset
  (`hashes`, `dates`, `bi` = built-column index or -1, `boards`, per-board
  `status` strings over the built columns) is built by `trend_git_commits()`
  walking `git log --first-parent` from the newest built commit back to the
  oldest in the window.

---

## 9. CLI options

| flag | default | meaning |
|------|---------|---------|
| `--db` | (required) | path to the history SQLite database |
| `--out` | (required) | output directory for the site |
| `--assets` | `./assets` next to the script | vendored uplot + app assets dir |
| `--tags` | all | comma-separated tags to include (e.g. `latest,beta`) |
| `--since` | all time | only builds on/after `YYYY-MM-DD` (use for fast dev runs) |
| `--trend-runs` | `75` | number of recent runs (columns) on the build-trends page |
| `--flash-warn-kb` | `20` | flag boards with less than this much free flash (KiB) |
| `--no-flash-free` | off | skip the HWDef flash-region computation (~2.5 s) |

---

## 10. Dev workflow & verification

```sh
# fast iteration: a recent window only (seconds instead of ~15 s)
./build_binaries_history_html.py --db DB.sqlite --out /tmp/bbh --since 2026-01-01

# render check straight from disk (no server) with headless chromium:
chromium --headless --no-sandbox --disable-gpu --virtual-time-budget=8000 \
    --window-size=1300,1200 --screenshot="$HOME/shot.png" \
    "file:///tmp/bbh/index.html"
```

Gotchas when verifying:

- **snap Chromium can't see `/tmp`.** Write screenshots under `$HOME` and, if you
  need a server, run it from a `$HOME` path. The site itself is fine anywhere.
- **No `fetch` from `file://`.** Data is inlined, so this is fine; just don't
  reintroduce a `fetch`-only path for local viewing.
- **Testing JS interactions headlessly:** load a tiny harness HTML that pulls in
  the real `assets/uplot.min.js` + `assets/app.js`, set `window.BBH*`, call the
  entry point, then dispatch synthetic `mousemove`/`click` on `.u-over` and read
  the result (e.g. override `window.open`, dump via `--dump-dom`). This is how
  the click-to-commit behaviour was verified.

Worth eyeballing after a change: a busy board (`vehicles/ArduCopter/CubeOrange.html`)
draws lines; `coverage.html` shows red boxes in the failures section;
`failures.html` shows the trend grid with recently-failed boards on top;
`flash.html` lists the known-tight boards first; the footer reports the
generation time.

---

## 11. Gotchas / invariants

- The DB is read-only — do not add writes or schema changes here.
- `APMrover2` is rewritten to `Rover` in SQL (`VEHICLE_EXPR`), in **both** the
  SELECT and ORDER BY, so merged rows stay contiguous for the streaming flush.
- NULL `text` ⇒ failed build everywhere (coverage, trends, flash skip).
- "Latest" for coverage and the flash table means **the most recent *completed*
  latest-tag run** (`latest_run_hash()` / `self.latest_hash`): the newest commit
  that has a row in the `run` table. build_binaries writes a `run` row only after
  every `build_*` finishes, so a hash in `run` ⇒ that run completed. The newest
  *build* commit is often a partial/in-progress run (e.g. 1561 of ~2231 boards),
  which would wrongly show ~700 boards as "not built". We use the run table (and
  fall back to the newest build commit only if it's empty). This also avoids the
  earlier stale-failure bug where a vehicle that stopped building a board years
  ago showed a frozen red instead of "not built".
- The build-trends grid, by contrast, keeps *all* recent runs as columns
  (including partial ones) and marks completion in its top "completed" row
  (`columns[i]["done"]`, from the `run` table). So coverage = the newest column
  with a ✓.
- `runs_commits_since_last()` and `trend_git_commits()` depend on git history
  being present (degrade gracefully if not).
- The RAM chart is intentionally omitted: `bss` on most ChibiOS boards is the
  whole reserved RAM region, not real usage. `data`/`bss` are still inlined and
  feed the "notable size changes" table.
- Board/vehicle names are run through `slugify()` for file paths; keep links and
  files using the same slug.
- Build-trend columns come from the **build** table, not `run` (the two don't
  perfectly overlap).
- The "show unbuilt commits" checkbox needs the git history present (the built
  commits must be on the first-parent chain from the newest one). In a shallow
  clone, or if the DB's commits aren't in this checkout, `trend_git_commits()`
  returns None and the checkbox is simply omitted — the built-only grid is
  unaffected.

---

## 12. Updating the vendored uPlot

```sh
cd assets
curl -sSL -o uplot.min.js  https://cdn.jsdelivr.net/npm/uplot@<ver>/dist/uPlot.iife.min.js
curl -sSL -o uplot.min.css https://cdn.jsdelivr.net/npm/uplot@<ver>/dist/uPlot.min.css
```

This is the only step needing network access; generation and viewing never do.

---

## 13. Ideas for future rework

- Lazy-render the trend grid so `--trend-runs` can be much larger without a
  multi-MB page.
- Per-vehicle trend grids / a vehicle filter (the grid currently aggregates
  across vehicles).
- Distinguish "stopped building" (went all-grey) from "failing" (red) in the
  trend sort.
- Surface RAM only for boards where `bss` is a genuine figure (small, e.g. the
  Linux boards) rather than omitting it everywhere.
- Optionally record exact `flash_free` from the `.apj` at capture time in
  `build_binaries_history.py` (a new column) for authoritative future data, with
  the hwdef figure as the historical fallback.
