#!/usr/bin/env python3

'''
Generate a static HTML site visualising an ArduPilot build_binaries history
database (as produced by build_binaries_history.py).

The database is read once, in a single ordered streaming scan, and rendered into
a tree of static HTML pages (each with its chart data inlined) plus a vendored
copy of uPlot.  The resulting site needs no database access in the browser, has
no inter-file data dependencies, and so works fully offline -- including by
opening the pages directly from local disk (file://).

Example:
    Tools/scripts/build_binaries_history_html.py \
        --db build_binaries_history-20260604.sqlite --out /tmp/bbh

AINFO: this file was written with AI assistance.
'''

import argparse
import datetime
import html
import json
import os
import shutil
import sqlite3
import subprocess
import sys
import time

# APMrover2 is the historical name for Rover; merge the two so the rename is
# hidden and a board's Rover history is continuous.  Applied in both the SELECT
# and ORDER BY so rows for the merged vehicle stay contiguous and time-ordered.
VEHICLE_EXPR = "case when vehicle='APMrover2' then 'Rover' else vehicle end"

COMMIT_BASE = "https://github.com/ArduPilot/ardupilot/commit/"
HASH_LEN = 12              # abbreviated commit SHA stored in JSON (GitHub-resolvable)
CHANGE_THRESHOLD = 1024    # bytes; size jumps at/above this are "notable"
MAX_CHANGES = 200          # cap notable changes recorded per board

# Path to the ChibiOS hwdef tree, relative to this script
# (Tools/scripts/build_binaries_history_html/ -> repo root).
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
HWDEF_DIR = os.path.join(REPO_ROOT, "libraries", "AP_HAL_ChibiOS", "hwdef")


def board_flash_totals(hwdef_dir, progress):
    '''per-board program-flash region size in bytes, via the HWDef system.

    Uses libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py (the same code
    the firmware build uses) so the number is authoritative.  Only ChibiOS
    boards have a hwdef; Linux/SITL boards are absent from the result.'''
    import contextlib
    import glob
    import io
    import tempfile

    scripts = os.path.join(hwdef_dir, "scripts")
    if not os.path.isdir(scripts):
        progress("hwdef scripts not found at %s; flash-free disabled" % scripts)
        return {}
    sys.path.insert(0, scripts)
    try:
        import chibios_hwdef
    except Exception as e:  # noqa: BLE001
        progress("could not import chibios_hwdef (%s); flash-free disabled" % e)
        return {}

    empty = tempfile.NamedTemporaryFile(suffix=".parm", delete=False)
    empty.close()
    outdir = tempfile.mkdtemp()
    totals = {}
    errors = 0
    for dat in sorted(glob.glob(os.path.join(hwdef_dir, "*", "hwdef.dat"))):
        board = os.path.basename(os.path.dirname(dat))
        try:
            c = chibios_hwdef.ChibiOSHWDef(
                outdir=outdir, hwdef=[dat], quiet=True,
                default_params_filepath=empty.name)
            with contextlib.redirect_stdout(io.StringIO()), \
                    contextlib.redirect_stderr(io.StringIO()):
                c.run()
            ft = c.env_vars.get('FLASH_TOTAL')
            if not ft:
                continue
            capacity = int(ft)   # internal program-flash region (bytes)
            # boards that run primarily from external flash place .text/.data
            # across internal + external regions, so add the external region.
            ext_mb = int(c.env_vars.get('EXT_FLASH_SIZE_MB', 0) or 0)
            if (c.env_vars.get('HAS_EXTERNAL_FLASH_SECTIONS') and ext_mb
                    and not c.env_vars.get('INT_FLASH_PRIMARY')):
                ext_rs = c.get_config('EXT_FLASH_RESERVE_START_KB', default=0, type=int)
                ext_re = c.get_config('EXT_FLASH_RESERVE_END_KB', default=0, type=int)
                capacity += (ext_mb * 1024 - ext_rs - ext_re) * 1024
            totals[board] = capacity
        except Exception:  # noqa: BLE001
            errors += 1
    shutil.rmtree(outdir, ignore_errors=True)
    os.remove(empty.name)
    progress("flash region computed for %u boards (%u hwdef errors)" % (
        len(totals), errors))
    return totals


def slugify(name):
    '''filesystem/URL-safe form of a vehicle or board name'''
    out = []
    for ch in str(name):
        if ch.isalnum() or ch in "._-":
            out.append(ch)
        else:
            out.append("_")
    return "".join(out)


UTC = datetime.timezone.utc


def utc_dt(ts):
    return datetime.datetime.fromtimestamp(ts, UTC)


def month_key(ts):
    d = utc_dt(ts)
    return "%04u-%02u" % (d.year, d.month)


def month_start_ts(mkey):
    y, m = mkey.split("-")
    return int(datetime.datetime(int(y), int(m), 1, tzinfo=UTC).timestamp())


class SiteGenerator(object):
    def __init__(self, db_path, out_dir, assets_dir, tags, since_ts, trend_runs,
                 flash_warn, no_flash_free):
        self.db_path = db_path
        self.out_dir = out_dir
        self.assets_dir = assets_dir
        self.tags = tags            # None => all tags
        self.since_ts = since_ts    # None => all time
        self.trend_runs = trend_runs  # columns on the build-trend page
        self.flash_warn = flash_warn  # bytes; free flash below this is flagged
        self.no_flash_free = no_flash_free
        self.flash_total = {}       # board -> program-flash region bytes
        self.latest_hash = None     # newest 'latest'-tag run commit (full)
        self.latest_hash12 = None   # ...abbreviated to match stored series hashes
        self.gen_start = None

        # accumulated while streaming
        self.vehicles = {}          # vehicle -> list of board status dicts
        self.coverage = {}          # vehicle -> {board: status}
        self.all_boards = set()
        self.month_total = {}       # month -> count (build-volume chart)
        self.total_builds = 0
        self.total_failed = 0
        self.last_build_ts = 0
        self.distinct_hashes = set()

    def progress(self, msg):
        print("BBHTML: %s" % msg)

    # ---- DB ----------------------------------------------------------------

    def connect(self):
        uri = "file:%s?mode=ro" % os.path.abspath(self.db_path)
        return sqlite3.connect(uri, uri=True)

    def build_query(self):
        sql = ("select %s as vehicle, board, tag, frame, hash, text, data, bss, "
               "start_time, duration from build" % VEHICLE_EXPR)
        params = []
        where = []
        if self.tags:
            where.append("tag in (%s)" % ",".join("?" * len(self.tags)))
            params.extend(self.tags)
        if self.since_ts is not None:
            where.append("start_time >= ?")
            params.append(self.since_ts)
        if where:
            sql += " where " + " and ".join(where)
        sql += " order by %s, board, tag, frame, start_time" % VEHICLE_EXPR
        return sql, params

    def latest_run_hash(self):
        '''commit of the most recent *completed* 'latest'-tag run.

        A run is "completed" iff it has a row in the `run` table (build_binaries
        records that only after every build_* finishes). The newest build-table
        commit is often a partial/in-progress run that built only some boards, so
        we prefer the run table. Falls back to the newest build commit if the run
        table has no latest entries.'''
        conn = self.connect()
        params = []
        sql = "select hash from run where tag='latest'"
        if self.since_ts is not None:
            sql += " and start_time >= ?"
            params.append(self.since_ts)
        sql += " order by start_time desc limit 1"
        row = conn.execute(sql, params).fetchone()
        if row is None:
            sql = "select hash from build where tag='latest'"
            params = []
            if self.since_ts is not None:
                sql += " and start_time >= ?"
                params.append(self.since_ts)
            sql += " group by hash order by max(start_time) desc limit 1"
            row = conn.execute(sql, params).fetchone()
        conn.close()
        return row[0] if row else None

    # ---- streaming ---------------------------------------------------------

    def generate(self):
        self.gen_start = time.monotonic()
        self.prepare_out()
        if not self.no_flash_free:
            self.progress("computing board flash sizes via HWDef system...")
            self.flash_total = board_flash_totals(HWDEF_DIR, self.progress)
        self.latest_hash = self.latest_run_hash()
        self.latest_hash12 = self.latest_hash[:HASH_LEN] if self.latest_hash else None
        conn = self.connect()
        sql, params = self.build_query()
        self.progress("scanning build table...")

        cur_key = None
        series = {}        # "tag|frame" -> dict of parallel lists
        nrows = 0

        for row in conn.execute(sql, params):
            (vehicle, board, tag, frame, h, text, data, bss,
             start_time, duration) = row
            key = (vehicle, board)
            if key != cur_key:
                if cur_key is not None:
                    self.flush_board(cur_key[0], cur_key[1], series)
                cur_key = key
                series = {}
            skey = "%s|%s" % (tag, frame if frame else "default")
            s = series.get(skey)
            if s is None:
                s = {"t": [], "text": [], "data": [], "bss": [],
                     "dur": [], "hash": []}
                series[skey] = s
            ts = int(start_time)
            s["t"].append(ts)
            s["text"].append(text)
            s["data"].append(data)
            s["bss"].append(bss)
            s["dur"].append(round(duration, 1) if duration is not None else None)
            s["hash"].append((h or "")[:HASH_LEN])

            # global aggregates
            self.total_builds += 1
            mk = month_key(start_time)
            self.month_total[mk] = self.month_total.get(mk, 0) + 1
            if text is None:
                self.total_failed += 1
            if ts > self.last_build_ts:
                self.last_build_ts = ts
            if h:
                self.distinct_hashes.add(h)

            nrows += 1
            if nrows % 250000 == 0:
                self.progress("  %u rows..." % nrows)

        if cur_key is not None:
            self.flush_board(cur_key[0], cur_key[1], series)
        conn.close()
        self.progress("scanned %u rows" % nrows)

        # the run table (secondary view)
        runs = self.load_runs()
        trend_columns, trend_status = self.build_trends()
        trend_commits = self.trend_git_commits(trend_columns)

        # top-level pages + data
        self.write_vehicle_indexes()
        self.write_coverage()
        self.write_trends(trend_columns, trend_status, trend_commits)
        self.write_flash()
        boards_series, builds_series = self.runs_build_counts()
        self.write_runs(runs, self.runs_commits_since_last(),
                        boards_series, builds_series)
        self.write_index()
        self.copy_assets()
        self.progress("done -> %s (%s)" % (self.out_dir, self.elapsed_str()))

    def load_runs(self):
        conn = self.connect()
        sql = "select tag, start_time, duration from run"
        params = []
        if self.tags:
            sql += " where tag in (%s)" % ",".join("?" * len(self.tags))
            params.extend(self.tags)
        sql += " order by tag, start_time"
        series = {}
        for tag, start_time, duration in conn.execute(sql, params):
            s = series.setdefault(tag, {"t": [], "v": [], "hash": []})
            s["t"].append(int(start_time))
            s["v"].append(round(duration, 1) if duration is not None else None)
            s["hash"].append(None)
        conn.close()
        return series

    def runs_commits_since_last(self):
        '''for each 'latest'-tag run, how many first-parent git commits landed
        since the previous run. Returns a simpleChart series dict, or None if
        git history is unavailable / runs aren't on the first-parent chain.'''
        conn = self.connect()
        sql = "select hash, start_time from run where tag='latest'"
        params = []
        if self.since_ts is not None:
            sql += " and start_time >= ?"
            params.append(self.since_ts)
        sql += " order by start_time asc"
        runs = conn.execute(sql, params).fetchall()
        conn.close()
        if len(runs) < 2:
            return None
        newest = runs[-1][0]
        try:
            out = subprocess.check_output(
                ["git", "-C", REPO_ROOT, "log", "--first-parent", "--format=%H",
                 "-n", "200000", newest],
                stderr=subprocess.DEVNULL).decode("utf-8", "replace")
        except Exception as e:  # noqa: BLE001
            self.progress("git history unavailable (%s); commits-per-run chart "
                          "disabled" % e)
            return None
        idx = {h: i for i, h in enumerate(out.split())}   # 0 == newest commit
        t, v, hsh = [], [], []
        prev = None
        for h, st in runs:
            if prev is not None and h in idx and prev in idx:
                delta = idx[prev] - idx[h]   # prev is older (larger index)
                if delta >= 0:
                    t.append(int(st))
                    v.append(delta)
                    hsh.append(h[:HASH_LEN])
            prev = h
        if not t:
            return None
        return {"commits since previous run": {"t": t, "v": v, "hash": hsh}}

    def runs_build_counts(self):
        '''per *completed* 'latest'-tag run: how many boards and how many builds
        it produced. Partial/in-progress runs are excluded (they would plot as
        near-zero spikes that swamp real steps); a run counts as completed iff it
        has a row in the `run` table. Returns (boards_series, builds_series) in
        simpleChart shape, or (None, None) if there is nothing to plot.'''
        conn = self.connect()
        sql = ("select hash, max(start_time) mt, count(distinct board) nboards, "
               "count(*) nbuilds from build where tag='latest' "
               "and hash in (select hash from run where tag='latest')")
        params = []
        if self.since_ts is not None:
            sql += " and start_time >= ?"
            params.append(self.since_ts)
        sql += " group by hash order by mt asc"
        boards = {"t": [], "v": [], "hash": []}
        builds = {"t": [], "v": [], "hash": []}
        for h, mt, nboards, nbuilds in conn.execute(sql, params):
            for s, val in ((boards, nboards), (builds, nbuilds)):
                s["t"].append(int(mt))
                s["v"].append(val)
                s["hash"].append(h[:HASH_LEN])
        conn.close()
        if not boards["t"]:
            return None, None
        return ({"boards per run": boards}, {"builds per run": builds})

    # ---- per-board output --------------------------------------------------

    def compute_changes(self, series):
        '''notable size jumps per series, biggest first'''
        changes = []
        for skey, s in series.items():
            for metric in ("text", "data", "bss"):
                prev = None
                vals = s[metric]
                for i in range(len(vals)):
                    v = vals[i]
                    if v is None:
                        continue
                    if prev is not None:
                        delta = v - prev
                        if abs(delta) >= CHANGE_THRESHOLD:
                            changes.append({
                                "t": s["t"][i],
                                "hash": s["hash"][i],
                                "metric": metric,
                                "delta": delta,
                                "series": skey,
                            })
                    prev = v
        changes.sort(key=lambda c: abs(c["delta"]), reverse=True)
        return changes[:MAX_CHANGES]

    def flush_board(self, vehicle, board, series):
        vslug = slugify(vehicle)
        bslug = slugify(board)
        self.all_boards.add(board)

        changes = self.compute_changes(series)
        region = self.flash_total.get(board)
        data = {"vehicle": vehicle, "board": board,
                "series": series, "changes": changes, "flash_region": region}

        # Status/size as of the most recent 'latest'-tag run (the same reference
        # commit the build-trends view uses), aggregated over frames. A vehicle
        # that was not built at that commit is "missing" (not a stale failure).
        latest_status = "missing"
        latest_size = None
        any_fail = False
        used_sizes = []
        for skey, s in series.items():
            if skey.split("|", 1)[0] != "latest":
                continue
            for i, h in enumerate(s["hash"]):
                if h != self.latest_hash12:
                    continue
                if s["text"][i] is None:
                    any_fail = True
                else:
                    used_sizes.append(s["text"][i] + (s["data"][i] or 0))
        if any_fail:
            latest_status = "failed"
        elif used_sizes:
            latest_status = "ok"
            latest_size = max(used_sizes)   # worst-case headroom across frames

        status = {"board": board, "slug": bslug, "status": latest_status,
                  "flash": latest_size, "n": sum(len(s["t"]) for s in series.values())}
        self.vehicles.setdefault(vehicle, []).append(status)
        self.coverage.setdefault(vehicle, {})[board] = latest_status

        self.write_board_page(vehicle, board, vslug, bslug, data)

    # ---- HTML helpers ------------------------------------------------------

    @staticmethod
    def inline_json(obj):
        '''compact JSON safe to embed inside a <script> element'''
        s = json.dumps(obj, separators=(",", ":"))
        # avoid prematurely terminating the <script> and other HTML pitfalls
        return s.replace("<", "\\u003c").replace(">", "\\u003e").replace("&", "\\u0026")

    def page_head(self, title, root, active):
        nav = [("index.html", "Coverage"),
               ("failures.html", "Build trends"),
               ("flash.html", "Flash free"),
               ("runs.html", "Runs"),
               ("overview.html", "Overview")]
        links = "".join(
            '<a href="%s%s"%s>%s</a>' % (
                root, href,
                ' style="font-weight:600"' if href == active else "",
                html.escape(label))
            for href, label in nav)
        return (
            "<!DOCTYPE html>\n<html lang='en'><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width, initial-scale=1'>"
            "<title>%s</title>"
            "<link rel='stylesheet' href='%sassets/uplot.min.css'>"
            "<link rel='stylesheet' href='%sassets/style.css'>"
            "</head><body>"
            "<header class='site'><h1>ArduPilot build_binaries history</h1>"
            "<nav>%s</nav></header><main>" % (
                html.escape(title), root, root, links))

    def elapsed_str(self):
        if self.gen_start is None:
            return "?"
        secs = time.monotonic() - self.gen_start
        if secs >= 60:
            return "%um %02us" % (int(secs // 60), int(secs % 60))
        return "%.1fs" % secs

    def page_foot(self):
        now = datetime.datetime.now(UTC).strftime("%Y-%m-%d %H:%M UTC")
        return ("</main><footer class='site'>Generated %s by "
                "build_binaries_history_html.py in %s</footer></body></html>" % (
                    now, self.elapsed_str()))

    def write_page(self, rel_path, html_str):
        path = os.path.join(self.out_dir, rel_path)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write(html_str)

    def write_board_page(self, vehicle, board, vslug, bslug, data):
        root = "../../"
        title = "%s / %s" % (vehicle, board)
        body = [self.page_head(title, root, "index.html")]
        body.append("<div class='crumbs'><a href='%sindex.html'>Coverage</a> / "
                    "<a href='%svehicles/%s/index.html'>%s</a> / %s</div>" % (
                        root, root, vslug, html.escape(vehicle), html.escape(board)))
        body.append("<h2>%s &mdash; %s</h2>" % (
            html.escape(vehicle), html.escape(board)))
        body.append("<p class='note'>Each line is a build series "
                    "(<code>tag|frame</code>). Click legend entries to toggle; "
                    "drag on a chart to zoom, double-click to reset.</p>")
        body.append("<div id='charts'></div>")
        body.append("<script src='%sassets/uplot.min.js'></script>" % root)
        body.append("<script src='%sassets/app.js'></script>" % root)
        body.append("<script>window.BBH=%s;window.BBH_DATA=%s;"
                    "window.BBH_boardDetail();</script>" % (
                        json.dumps({"commitBase": COMMIT_BASE}),
                        self.inline_json(data)))
        body.append(self.page_foot())
        self.write_page("vehicles/%s/%s.html" % (vslug, bslug), "".join(body))

    def write_vehicle_indexes(self):
        for vehicle, boards in sorted(self.vehicles.items()):
            vslug = slugify(vehicle)
            boards = sorted(boards, key=lambda b: b["board"].lower())
            root = "../../"
            body = [self.page_head(vehicle, root, "index.html")]
            body.append("<div class='crumbs'><a href='%sindex.html'>Coverage</a> "
                        "/ %s</div>" % (root, html.escape(vehicle)))
            body.append("<h2>%s &mdash; %u boards</h2>" % (
                html.escape(vehicle), len(boards)))
            body.append("<table><thead><tr><th>Board</th><th>Latest status</th>"
                        "<th class='num'>Flash (text+data)</th>"
                        "<th class='num'>Builds</th></tr></thead><tbody>")
            for b in boards:
                flash = self.fmt_bytes(b["flash"])
                body.append(
                    "<tr><td><a href='%s.html'>%s</a></td>"
                    "<td><span class='chip %s'>%s</span></td>"
                    "<td class='num'>%s</td><td class='num'>%u</td></tr>" % (
                        b["slug"], html.escape(b["board"]),
                        b["status"], b["status"], flash, b["n"]))
            body.append("</tbody></table>")
            body.append(self.page_foot())
            self.write_page("vehicles/%s/index.html" % vslug, "".join(body))

    def coverage_matrix(self, vehicles, boards):
        '''render a coverage matrix table for the given board rows'''
        out = ["<div class='matrix'><table><thead><tr><th>Board</th>"]
        for v in vehicles:
            out.append("<th>%s</th>" % html.escape(v))
        out.append("</tr></thead><tbody>")
        for board in boards:
            bslug = slugify(board)
            out.append("<tr><td>%s</td>" % html.escape(board))
            for v in vehicles:
                st = self.coverage.get(v, {}).get(board)
                if st is None:
                    out.append("<td class='cell'><span class='dot missing'></span></td>")
                else:
                    out.append(
                        "<td class='cell'><a href='vehicles/%s/%s.html' "
                        "title='%s %s: %s'><span class='dot %s'></span></a></td>" % (
                            slugify(v), bslug, html.escape(v),
                            html.escape(board), st, st))
            out.append("</tr>")
        out.append("</tbody></table></div>")
        return "".join(out)

    def write_coverage(self):
        vehicles = sorted(self.vehicles.keys())

        # Only list boards that are actually part of the reference run: a board
        # built by no vehicle at the most recent completed run (all-grey row) is
        # retired/renamed/removed from autobuild (e.g. 2RAWH743 was renamed to
        # IFLIGHT_2RAW_H7), so it is noise on this "current state" page. Such
        # boards still appear on the Build-trends page, which tracks history.
        boards = sorted(
            (b for b in self.all_boards
             if any(self.coverage.get(v, {}).get(b) in ("ok", "failed")
                    for v in vehicles)),
            key=str.lower)

        # a board is "failing" if any vehicle's latest build for it failed
        failing = [b for b in boards
                   if any(self.coverage.get(v, {}).get(b) == "failed"
                          for v in vehicles)]
        clean = [b for b in boards if b not in set(failing)]

        root = ""
        body = [self.page_head("Coverage", root, "index.html")]
        body.append("<h2>Board coverage (latest tag)</h2>")
        body.append("<p class='note'>Status of each board at the most recent "
                    "<b>completed</b> <code>latest</code>-tag run (the newest run "
                    "recorded as finished, not a partial/in-progress one), so a "
                    "vehicle that no longer builds a board shows as not-built "
                    "rather than a stale failure. Boards built by no vehicle at "
                    "that run (renamed/retired/removed from autobuild) are "
                    "omitted here; the Build trends page shows per-run history "
                    "for boards built within its window. "
                    "<span class='dot ok'></span> ok "
                    "<span class='dot failed'></span> failed "
                    "<span class='dot missing'></span> not built.</p>")

        body.append("<h2>Boards with failures (%u)</h2>" % len(failing))
        if failing:
            body.append(self.coverage_matrix(vehicles, failing))
        else:
            body.append("<p class='note'>No boards have a failing latest build.</p>")

        body.append("<h2>Boards with no failures (%u)</h2>" % len(clean))
        body.append(self.coverage_matrix(vehicles, clean))

        body.append(self.page_foot())
        self.write_page("index.html", "".join(body))

    def build_trends(self):
        '''per-board build outcome over the most recent latest-tag runs.

        Returns (columns, status):
          columns: list of {"hash", "t", "done"} newest-first (one per run);
                   "done" is True if the run completed (is in the `run` table)
          status:  {board: {hash: "ok" | "failed"}}  (absent => not built)
        A board's run aggregates across vehicles: failed if any vehicle's build
        for that commit failed, otherwise ok.'''
        conn = self.connect()
        base = "from build where tag='latest'"
        params = []
        if self.since_ts is not None:
            base += " and start_time >= ?"
            params.append(self.since_ts)
        cols = conn.execute(
            "select hash, max(start_time) mt %s group by hash "
            "order by mt desc limit ?" % base,
            params + [self.trend_runs]).fetchall()
        completed = set(h for (h,) in
                        conn.execute("select hash from run where tag='latest'"))
        columns = [{"hash": h, "t": int(mt), "done": h in completed}
                   for h, mt in cols]
        hs = [c["hash"] for c in columns]
        status = {}
        if hs:
            ph = ",".join("?" * len(hs))
            q = ("select board, hash, text from build where tag='latest' "
                 "and hash in (%s)" % ph)
            for board, h, text in conn.execute(q, hs):
                bs = status.setdefault(board, {})
                if text is None:
                    bs[h] = "failed"
                elif bs.get(h) != "failed":
                    bs[h] = "ok"
        conn.close()
        return columns, status

    def trend_git_commits(self, columns):
        '''ordered (newest-first) git first-parent commits spanning the built
        window, for the optional "show commits we didn't build" view.

        Returns a list of {"hash", "ts", "bi"} from the newest built commit back
        through the oldest (inclusive), where "bi" is the index of that commit in
        `columns` (the built runs) or -1 if it was never built.  Assumes the
        built commits lie on the first-parent chain (linear history); returns
        None if git history is unavailable or doesn't cover the window.'''
        if not columns:
            return None
        built = [c["hash"] for c in columns]   # newest-first, full hashes
        pos = {h: i for i, h in enumerate(built)}
        newest, oldest = built[0], built[-1]
        try:
            out = subprocess.check_output(
                ["git", "-C", REPO_ROOT, "log", "--first-parent",
                 "--format=%H %ct", "-n", "50000", newest],
                stderr=subprocess.DEVNULL).decode("utf-8", "replace")
        except Exception as e:  # noqa: BLE001
            self.progress("git history unavailable (%s); skip-columns disabled" % e)
            return None
        commits = []
        reached = False
        for line in out.splitlines():
            parts = line.split()
            if len(parts) != 2:
                continue
            h, ts = parts[0], int(parts[1])
            commits.append({"hash": h, "ts": ts, "bi": pos.get(h, -1)})
            if h == oldest:
                reached = True
                break
        if not reached or not all(b in {c["hash"] for c in commits} for b in built):
            self.progress("git history does not cleanly span the built window; "
                          "skip-columns disabled")
            return None
        return commits

    def write_trends(self, columns, status, full_commits=None):
        root = ""
        body = [self.page_head("Build trends", root, "failures.html")]
        body.append("<h2>Build trends</h2>")
        body.append("<p class='note'>One row per board; one box per "
                    "<code>latest</code>-tag run, newest on the left "
                    "(last %u runs). <span class='box ok'></span> built "
                    "<span class='box failed'></span> failed "
                    "<span class='box missing'></span> not built. The top "
                    "<b>completed</b> row marks runs that finished "
                    "(<span class='runmark done'>✓</span>); unmarked runs "
                    "are partial/in-progress. Click a box for the commit on "
                    "GitHub. Boards that failed most recently are listed "
                    "first.</p>" % len(columns))

        if not columns:
            body.append("<p class='note'>No build data.</p>")
            body.append(self.page_foot())
            self.write_page("failures.html", "".join(body))
            return

        # sort: most-recently-failed first (smallest failing column index),
        # then boards with no failure in the window, both alphabetical.
        def sort_key(board):
            bs = status[board]
            first_fail = next((i for i, c in enumerate(columns)
                               if bs.get(c["hash"]) == "failed"), None)
            if first_fail is None:
                return (1, 0, board.lower())
            return (0, first_fail, board.lower())

        boards = sorted(status.keys(), key=sort_key)
        dates = [utc_dt(c["t"]).strftime("%Y-%m-%d") for c in columns]

        # optional checkbox to also show the commits we did not build
        if full_commits:
            n_skip = sum(1 for c in full_commits if c["bi"] < 0)
            body.append(
                "<p><label><input type='checkbox' id='show-skipped'> Show the "
                "%u commits we did not build (interleaved, greyed; may take a "
                "moment to render)</label></p>" % n_skip)

        body.append("<div id='trend-built'><table class='trend'><tbody>")
        # completed-run indicator row (checkmark = run finished)
        marks = []
        for c, date in zip(columns, dates):
            if c["done"]:
                marks.append("<span class='runmark done' title='%s — completed'>"
                             "&#10003;</span>" % date)
            else:
                marks.append("<span class='runmark' title='%s — partial run'>"
                             "&middot;</span>" % date)
        body.append("<tr><td class='name'>completed</td>"
                    "<td class='boxes'>%s</td></tr>" % "".join(marks))
        for board in boards:
            bs = status[board]
            cells = []
            for c, date in zip(columns, dates):
                st = bs.get(c["hash"])
                cls = st if st else "missing"
                label = st if st else "not built"
                cells.append(
                    "<a class='box %s' href='%s%s' title='%s &mdash; %s'></a>" % (
                        cls, COMMIT_BASE, c["hash"], date, label))
            body.append("<tr><td class='name'>%s</td>"
                        "<td class='boxes'>%s</td></tr>" % (
                            html.escape(board), "".join(cells)))
        body.append("</tbody></table></div>")

        # full (built + unbuilt) view is rendered on demand by app.js from the
        # compact dataset below, to keep the default page light.
        if full_commits:
            body.append("<div id='trend-full' style='display:none;overflow-x:auto'></div>")
            st_map = {"ok": "o", "failed": "x"}
            trend_data = {
                "commitBase": COMMIT_BASE,
                "hashes": [c["hash"][:HASH_LEN] for c in full_commits],
                "dates": [utc_dt(c["ts"]).strftime("%Y-%m-%d") for c in full_commits],
                "bi": [c["bi"] for c in full_commits],
                "done": [1 if c["done"] else 0 for c in columns],  # per built column
                "boards": boards,
                "status": ["".join(st_map.get(status[b].get(c["hash"]), ".")
                                   for c in columns) for b in boards],
            }
            body.append("<script src='assets/app.js'></script>")
            body.append("<script>window.BBH_TREND=%s;window.BBH_trend();</script>"
                        % self.inline_json(trend_data))

        body.append(self.page_foot())
        self.write_page("failures.html", "".join(body))

    def write_flash(self):
        # one row per (vehicle, board) whose latest build and flash region are
        # both known; free = region - (text+data), tightest first.
        rows = []
        skipped = 0
        for vehicle, boards in self.vehicles.items():
            for b in boards:
                region = self.flash_total.get(b["board"])
                used = b["flash"]   # latest text+data, None if latest failed
                if region is None or used is None:
                    continue
                if used > region:
                    # capacity model doesn't fit this build (e.g. an unusual
                    # external-flash layout); omit rather than show negative.
                    skipped += 1
                    continue
                rows.append((vehicle, b, region, used, region - used))
        rows.sort(key=lambda r: r[4])  # least free first

        root = ""
        body = [self.page_head("Flash free", root, "flash.html")]
        body.append("<h2>Flash free (latest tag)</h2>")
        if not self.flash_total:
            body.append("<p class='note'>Flash region sizes are unavailable "
                        "(HWDef system not found or disabled).</p>")
            body.append(self.page_foot())
            self.write_page("flash.html", "".join(body))
            return
        body.append("<p class='note'>Program-flash headroom for the most recent "
                    "<code>latest</code>-tag build, tightest first. Region is the "
                    "program-flash area from the board hwdef (current tree, via the "
                    "HWDef system; internal + external for external-flash boards); "
                    "used = text+data. Rows under %s free are flagged. Boards with "
                    "no hwdef (Linux/SITL)%s are omitted.</p>"
                    % (self.fmt_bytes(self.flash_warn),
                       " and %u builds whose size exceeds the modelled "
                       "region" % skipped if skipped else ""))
        body.append("<table><thead><tr><th>Vehicle</th><th>Board</th>"
                    "<th class='num'>Region (bytes)</th><th class='num'>Used (bytes)</th>"
                    "<th class='num'>Free (bytes)</th><th class='num'>Used %</th>"
                    "</tr></thead><tbody>")
        for vehicle, b, region, used, free in rows:
            warn = " style='color:var(--fail);font-weight:600'" if free < self.flash_warn else ""
            pct = 100.0 * used / region if region else 0
            body.append(
                "<tr%s><td>%s</td>"
                "<td><a href='vehicles/%s/%s.html'>%s</a></td>"
                "<td class='num'>%d</td><td class='num'>%d</td>"
                "<td class='num'>%d</td><td class='num'>%.1f%%</td></tr>" % (
                    warn, html.escape(vehicle),
                    slugify(vehicle), b["slug"], html.escape(b["board"]),
                    region, used, free, pct))
        body.append("</tbody></table>")
        body.append(self.page_foot())
        self.write_page("flash.html", "".join(body))

    def write_runs(self, runs, commits_since=None, boards_series=None,
                   builds_series=None):
        root = ""
        body = [self.page_head("Runs", root, "runs.html")]
        body.append("<h2>build_binaries run duration</h2>")
        body.append("<p class='note'>Wall-clock duration of each whole "
                    "build_binaries run, by tag.</p>")
        body.append("<div id='runchart'></div>")

        charts = [{"host": "runchart", "data": {"series": runs},
                   "title": "Run duration", "fmt": "duration"}]
        if boards_series:
            body.append("<h2>Boards per run</h2>")
            body.append("<p class='note'>Distinct boards built in each "
                        "<b>completed</b> <code>latest</code>-tag run.</p>")
            body.append("<div id='boardschart'></div>")
            charts.append({"host": "boardschart",
                           "data": {"series": boards_series},
                           "title": "Boards per run", "fmt": "count",
                           "commitBase": COMMIT_BASE})
        if builds_series:
            body.append("<h2>Builds per run</h2>")
            body.append("<p class='note'>Total builds (vehicle × board × frame) "
                        "in each <b>completed</b> <code>latest</code>-tag run.</p>")
            body.append("<div id='buildschart'></div>")
            charts.append({"host": "buildschart",
                           "data": {"series": builds_series},
                           "title": "Builds per run", "fmt": "count",
                           "commitBase": COMMIT_BASE})
        if commits_since:
            body.append("<h2>Commits since previous run</h2>")
            body.append("<p class='note'>First-parent git commits that landed "
                        "between each <code>latest</code>-tag run and the one "
                        "before it.</p>")
            body.append("<div id='commitschart'></div>")
            charts.append({"host": "commitschart",
                           "data": {"series": commits_since},
                           "title": "Commits since previous run", "fmt": "count",
                           "commitBase": COMMIT_BASE})

        body.append("<script src='assets/uplot.min.js'></script>")
        body.append("<script src='assets/app.js'></script>")
        body.append("<script>window.BBH_SIMPLE=%s;window.BBH_simpleChart();</script>"
                    % self.inline_json(charts))
        body.append(self.page_foot())
        self.write_page("runs.html", "".join(body))

    def write_index(self):
        months = sorted(self.month_total.keys())
        series = {"builds": {
            "t": [month_start_ts(m) for m in months],
            "v": [self.month_total[m] for m in months],
            "hash": [None] * len(months),
        }}

        nvehicles = len(self.vehicles)
        nboards = len(self.all_boards)
        last = (utc_dt(self.last_build_ts).strftime("%Y-%m-%d %H:%M")
                if self.last_build_ts else "—")
        fail_pct = (100.0 * self.total_failed / self.total_builds
                    if self.total_builds else 0)

        body = [self.page_head("Overview", "", "overview.html")]
        body.append("<h2>Overview</h2>")
        body.append("<div class='cards'>")
        for num, lbl in [
            ("{:,}".format(self.total_builds), "build records"),
            (str(nvehicles), "vehicles"),
            (str(nboards), "boards"),
            (str(len(self.distinct_hashes)), "commits"),
            ("%.1f%%" % fail_pct, "failed builds"),
            (last, "last build"),
        ]:
            body.append("<div class='card'><div class='num'>%s</div>"
                        "<div class='lbl'>%s</div></div>" % (num, html.escape(lbl)))
        body.append("</div>")

        body.append("<h2>Vehicles</h2><table><thead><tr><th>Vehicle</th>"
                    "<th class='num'>Boards</th></tr></thead><tbody>")
        for vehicle in sorted(self.vehicles.keys()):
            body.append("<tr><td><a href='vehicles/%s/index.html'>%s</a></td>"
                        "<td class='num'>%u</td></tr>" % (
                            slugify(vehicle), html.escape(vehicle),
                            len(self.vehicles[vehicle])))
        body.append("</tbody></table>")

        body.append("<h2>Build volume</h2>")
        body.append("<div id='volchart'></div>")
        body.append("<script src='assets/uplot.min.js'></script>")
        body.append("<script src='assets/app.js'></script>")
        body.append("<script>window.BBH_SIMPLE=%s;window.BBH_simpleChart();</script>"
                    % self.inline_json({"host": "volchart", "data": {"series": series},
                                        "title": "Builds per month", "fmt": "count"}))
        body.append(self.page_foot())
        self.write_page("overview.html", "".join(body))

    # ---- misc --------------------------------------------------------------

    def fmt_bytes(self, v):
        if v is None:
            return "—"
        if v >= 1024 * 1024:
            return "%.2f MiB" % (v / (1024.0 * 1024.0))
        if v >= 1024:
            return "%.1f KiB" % (v / 1024.0)
        return "%u B" % v

    def prepare_out(self):
        # Remove the trees/files we manage so a periodic re-run does not leave
        # orphans for boards or vehicles that have been renamed or dropped.
        # Only our own outputs are touched; anything else in --out is left alone.
        for sub in ("vehicles", "assets", "data"):
            shutil.rmtree(os.path.join(self.out_dir, sub), ignore_errors=True)
        for name in ("index.html", "overview.html", "coverage.html",
                     "failures.html", "flash.html", "runs.html"):
            try:
                os.remove(os.path.join(self.out_dir, name))
            except FileNotFoundError:
                pass
        os.makedirs(self.out_dir, exist_ok=True)

    def copy_assets(self):
        dst = os.path.join(self.out_dir, "assets")
        os.makedirs(dst, exist_ok=True)
        for name in ("uplot.min.js", "uplot.min.css", "app.js", "style.css"):
            src = os.path.join(self.assets_dir, name)
            if not os.path.exists(src):
                raise IOError("missing asset %s (see --assets); vendor uPlot "
                              "into the assets dir first" % src)
            shutil.copyfile(src, os.path.join(dst, name))


def parse_since(s):
    if s is None:
        return None
    d = datetime.datetime.strptime(s, "%Y-%m-%d")
    return d.timestamp()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--db", required=True, help="path to history sqlite database")
    ap.add_argument("--out", required=True, help="output directory for the site")
    default_assets = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "assets")
    ap.add_argument("--assets", default=default_assets,
                    help="directory holding vendored uplot/app assets")
    ap.add_argument("--tags", default=None,
                    help="comma-separated tags to include (default: all)")
    ap.add_argument("--since", default=None,
                    help="only builds on/after this date (YYYY-MM-DD)")
    ap.add_argument("--trend-runs", type=int, default=75,
                    help="number of recent runs shown on the build-trend page")
    ap.add_argument("--flash-warn-kb", type=int, default=20,
                    help="flag boards with less than this much free flash (KiB)")
    ap.add_argument("--no-flash-free", action="store_true",
                    help="skip computing per-board flash sizes via the HWDef system")
    args = ap.parse_args()

    tags = [t.strip() for t in args.tags.split(",")] if args.tags else None
    since_ts = parse_since(args.since)

    gen = SiteGenerator(args.db, args.out, args.assets, tags, since_ts,
                        args.trend_runs, args.flash_warn_kb * 1024,
                        args.no_flash_free)
    gen.generate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
