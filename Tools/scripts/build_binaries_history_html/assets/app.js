/* build_binaries history static site - front-end chart logic.
 * No framework, no external requests. Expects a global window.BBH set by the
 * board detail page:
 *   window.BBH = { dataUrl, commitBase, vehicle, board };
 * and uPlot loaded from uplot.min.js.
 */
(function () {
  "use strict";

  var COLORS = [
    "#0969da", "#cf222e", "#1a7f37", "#9a6700",
    "#8250df", "#bf3989", "#1b7c83", "#6e7781"
  ];

  function fmtBytes(v) {
    if (v == null) { return "—"; }
    if (v >= 1024 * 1024) { return (v / (1024 * 1024)).toFixed(2) + " MiB"; }
    if (v >= 1024) { return (v / 1024).toFixed(1) + " KiB"; }
    return v + " B";
  }
  function fmtSecs(v) {
    if (v == null) { return "—"; }
    return v.toFixed(1) + " s";
  }
  function fmtDuration(v) {
    if (v == null) { return "—"; }
    if (v >= 3600) { return (v / 3600).toFixed(2) + " h"; }
    if (v >= 60) { return (v / 60).toFixed(1) + " min"; }
    return v.toFixed(0) + " s";
  }
  function fmtPct(v) {
    if (v == null) { return "—"; }
    return v.toFixed(1) + " %";
  }
  function fmtCount(v) {
    if (v == null) { return "—"; }
    return String(v);
  }
  function pickFmt(name) {
    if (name === "secs") { return fmtSecs; }
    if (name === "duration") { return fmtDuration; }
    if (name === "pct") { return fmtPct; }
    if (name === "count") { return fmtCount; }
    return fmtBytes;
  }
  function fmtDate(ts) {
    var d = new Date(ts * 1000);
    return d.toISOString().slice(0, 16).replace("T", " ");
  }

  /* Build a union x-axis across every series and align each series' metric
   * values (and hashes) onto it. metricFn(s, i) returns a number or null. */
  function align(seriesMap, keys, metricFn) {
    var tset = {};
    keys.forEach(function (k) {
      seriesMap[k].t.forEach(function (t) { tset[t] = true; });
    });
    var xs = Object.keys(tset).map(Number).sort(function (a, b) { return a - b; });
    var idxOf = {};
    xs.forEach(function (t, i) { idxOf[t] = i; });

    var cols = [xs];
    var hashGrid = [];
    keys.forEach(function (k) {
      var col = new Array(xs.length).fill(null);
      var hcol = new Array(xs.length).fill(null);
      var s = seriesMap[k];
      for (var i = 0; i < s.t.length; i++) {
        var j = idxOf[s.t[i]];
        col[j] = metricFn(s, i);
        hcol[j] = s.hash[i];
      }
      cols.push(col);
      hashGrid.push(hcol);
    });
    return { data: cols, hashGrid: hashGrid };
  }

  /* Floating tooltip plugin. Also makes a click on the hovered point open its
   * commit on GitHub (the same hash shown in the tooltip). */
  function tooltipPlugin(keys, hashGrid, valueFmt, commitBase) {
    var el;
    var curHash = null;          // hash under the cursor, or null
    var downX = null, downY = null, moved = false;  // drag-vs-click guard
    return {
      hooks: {
        init: function (u) {
          el = document.createElement("div");
          el.className = "bbh-tip";
          el.style.cssText =
            "position:absolute;z-index:10;pointer-events:none;display:none;" +
            "background:#fff;border:1px solid #d0d7de;border-radius:6px;" +
            "padding:6px 8px;font-size:12px;box-shadow:0 1px 4px rgba(0,0,0,.15)";
          u.over.appendChild(el);
          u.over.addEventListener("mousedown", function (e) {
            downX = e.clientX; downY = e.clientY; moved = false;
          });
          u.over.addEventListener("mousemove", function (e) {
            if (downX != null &&
                (Math.abs(e.clientX - downX) > 4 || Math.abs(e.clientY - downY) > 4)) {
              moved = true;       // a drag (e.g. zoom select), not a click
            }
          });
          u.over.addEventListener("click", function () {
            if (!moved && curHash) {
              window.open(commitBase + curHash, "_blank");
            }
            downX = null;
          });
        },
        setCursor: function (u) {
          var idx = u.cursor.idx;
          curHash = null;
          u.over.style.cursor = "default";
          if (idx == null || u.cursor.left < 0) { el.style.display = "none"; return; }
          var ts = u.data[0][idx];
          var rows = "<div style='color:#6a737d'>" + fmtDate(ts) + "</div>";
          var hash = null;
          for (var s = 1; s < u.series.length; s++) {
            if (u.series[s].show === false) { continue; }
            var v = u.data[s][idx];
            if (v == null) { continue; }
            if (hash == null) { hash = hashGrid[s - 1][idx]; }
            rows += "<div><span style='color:" + COLORS[(s - 1) % COLORS.length] +
              "'>●</span> " + keys[s - 1] + ": <b>" + valueFmt(v) + "</b></div>";
          }
          if (hash) {
            curHash = hash;
            u.over.style.cursor = "pointer";
            rows += "<div style='margin-top:3px'><a href='" + commitBase +
              hash + "' target='_blank'>" + hash.slice(0, 10) + " &#8599;</a></div>";
          }
          el.innerHTML = rows;
          el.style.display = "block";
          var left = u.cursor.left, top = u.cursor.top;
          el.style.left = (left + 14) + "px";
          el.style.top = (top + 14) + "px";
        }
      }
    };
  }

  function makeChart(host, title, keys, aligned, valueFmt, commitBase) {
    var block = document.createElement("div");
    block.className = "chart-block";
    var h = document.createElement("h3");
    h.textContent = title;
    block.appendChild(h);
    var chartDiv = document.createElement("div");
    chartDiv.className = "uplot-host";
    block.appendChild(chartDiv);
    host.appendChild(block);

    var series = [{}];
    keys.forEach(function (k, i) {
      series.push({
        label: k,
        stroke: COLORS[i % COLORS.length],
        width: 1.5,
        // series are merged onto a shared union x-axis, so each is null at the
        // sample times of the others; bridge those gaps so lines are visible.
        spanGaps: true,
        points: { show: false },
        value: function (u, v) { return valueFmt(v); }
      });
    });

    var width = chartDiv.clientWidth || 1100;
    var opts = {
      width: width,
      height: 320,
      series: series,
      scales: { x: { time: true } },
      axes: [
        {},
        { size: 72, values: function (u, vals) { return vals.map(valueFmt); } }
      ],
      cursor: { focus: { prox: 30 } },
      plugins: [tooltipPlugin(keys, aligned.hashGrid, valueFmt, commitBase)]
    };
    var u = new uPlot(opts, aligned.data, chartDiv);
    window.addEventListener("resize", function () {
      u.setSize({ width: chartDiv.clientWidth || width, height: 320 });
    });
    return u;
  }

  function renderChanges(host, changes, commitBase) {
    if (!changes || !changes.length) { return; }
    var h = document.createElement("h2");
    h.textContent = "Notable size changes";
    host.appendChild(h);
    var p = document.createElement("p");
    p.className = "note";
    p.textContent = "Builds where a section size jumped relative to the previous build of the same series.";
    host.appendChild(p);
    var tbl = document.createElement("table");
    tbl.innerHTML =
      "<thead><tr><th>Date</th><th>Series</th><th>Metric</th>" +
      "<th class='num'>Change</th><th>Commit</th></tr></thead>";
    var tb = document.createElement("tbody");
    changes.forEach(function (c) {
      var tr = document.createElement("tr");
      var cls = c.delta >= 0 ? "delta-pos" : "delta-neg";
      var sign = c.delta >= 0 ? "+" : "−";
      tr.innerHTML =
        "<td>" + fmtDate(c.t) + "</td>" +
        "<td>" + c.series + "</td>" +
        "<td>" + c.metric + "</td>" +
        "<td class='num " + cls + "'>" + sign + fmtBytes(Math.abs(c.delta)) + "</td>" +
        "<td><a href='" + commitBase + c.hash + "' target='_blank'><code class='hash'>" +
        c.hash.slice(0, 10) + "</code></a></td>";
      tb.appendChild(tr);
    });
    tbl.appendChild(tb);
    host.appendChild(tbl);
  }

  function boardDetail() {
    var cfg = window.BBH;
    var host = document.getElementById("charts");
    var render = function (d) {
      var keys = Object.keys(d.series).sort();
      if (!keys.length) {
        host.innerHTML = "<p class='note'>No build records for this board.</p>";
        return;
      }
      var dur = align(d.series, keys, function (s, i) { return s.dur[i]; });

      if (d.flash_region) {
        var region = d.flash_region;
        var free = align(d.series, keys, function (s, i) {
          return (s.text[i] == null || s.data[i] == null) ? null
            : region - (s.text[i] + s.data[i]);
        });
        makeChart(host, "Flash free (region " + fmtBytes(region) + " − text+data)",
          keys, free, fmtBytes, cfg.commitBase);
      } else {
        // no known flash region (e.g. Linux/SITL): fall back to usage
        var flash = align(d.series, keys, function (s, i) {
          return (s.text[i] == null || s.data[i] == null) ? null : s.text[i] + s.data[i];
        });
        makeChart(host, "Flash usage (text + data)", keys, flash, fmtBytes, cfg.commitBase);
      }
      makeChart(host, "Build duration", keys, dur, fmtSecs, cfg.commitBase);
      renderChanges(host, d.changes, cfg.commitBase);
    };
    // Data is inlined into the page (window.BBH_DATA) so pages open from
    // file://; fall back to fetch() when only a dataUrl is provided.
    if (window.BBH_DATA) {
      render(window.BBH_DATA);
    } else if (cfg && cfg.dataUrl) {
      fetch(cfg.dataUrl).then(function (r) { return r.json(); }).then(render)
        .catch(function (e) {
          host.innerHTML = "<p class='note'>Failed to load data: " + e + "</p>";
        });
    }
  }

  /* One or more simple single-series time charts (runs.html / dashboard),
   * driven by window.BBH_SIMPLE = cfg or [cfg, ...] where
   * cfg = { host, title, fmt, data | dataUrl, commitBase? }. */
  function simpleChart() {
    var cfgs = window.BBH_SIMPLE;
    if (!cfgs) { return; }
    if (!Array.isArray(cfgs)) { cfgs = [cfgs]; }
    cfgs.forEach(function (c) {
      var host = document.getElementById(c.host);
      var render = function (d) {
        var keys = Object.keys(d.series).sort();
        var fmt = pickFmt(c.fmt);
        var aligned = align(d.series, keys, function (s, i) { return s.v[i]; });
        makeChart(host, c.title, keys, aligned, fmt, c.commitBase || "");
      };
      if (c.data) {
        render(c.data);
      } else if (c.dataUrl) {
        fetch(c.dataUrl).then(function (r) { return r.json(); }).then(render);
      }
    });
  }

  /* Build-trends page: a checkbox that swaps the built-only grid for a full
   * grid including the commits we did not build. Driven by window.BBH_TREND and
   * rendered on demand (the full grid can be hundreds of thousands of cells).
   *   { commitBase, hashes[], dates[], bi[], boards[], status[] }
   * hashes/dates/bi are per full-commit (newest first); bi = index into the
   * built columns or -1; status[r] is a per-board string over the built columns
   * ('o' ok, 'x' failed, '.' missing). */
  function trendView() {
    var d = window.BBH_TREND;
    if (!d) { return; }
    var cb = document.getElementById("show-skipped");
    var built = document.getElementById("trend-built");
    var full = document.getElementById("trend-full");
    if (!cb || !built || !full) { return; }
    var rendered = false;

    function esc(s) {
      return s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
    }
    function colIndex(box) {
      return Array.prototype.indexOf.call(box.parentNode.children, box);
    }
    function render() {
      var H = d.hashes, bi = d.bi, names = d.boards, st = d.status, n = H.length;
      var out = ["<table class='trend full'><tbody>"];
      // completed-run indicator row (✓ = finished run, · = partial, blank = a
      // commit we never ran)
      var marks = new Array(n);
      for (var mc = 0; mc < n; mc++) {
        var mb = bi[mc];
        if (mb >= 0 && d.done && d.done[mb]) {
          marks[mc] = "<span class='runmark done' title='" + d.dates[mc] +
            " completed'>✓</span>";
        } else if (mb >= 0) {
          marks[mc] = "<span class='runmark' title='" + d.dates[mc] +
            " partial run'>·</span>";
        } else {
          marks[mc] = "<span class='runmark'></span>";
        }
      }
      out.push("<tr><td class='name'>completed</td><td class='boxes'>" +
        marks.join("") + "</td></tr>");
      for (var r = 0; r < names.length; r++) {
        var row = st[r];
        var cells = new Array(n);
        for (var c = 0; c < n; c++) {
          var b = bi[c], k;
          if (b < 0) {
            k = "skipped";
          } else {
            var ch = row.charAt(b);
            k = ch === "o" ? "ok" : ch === "x" ? "failed" : "missing";
          }
          cells[c] = "<i class='box " + k + "'></i>";
        }
        out.push("<tr><td class='name'>" + esc(names[r]) +
          "</td><td class='boxes'>" + cells.join("") + "</td></tr>");
      }
      out.push("</tbody></table>");
      full.innerHTML = out.join("");
      // event delegation: per-cell href/title would bloat the DOM hugely
      full.addEventListener("click", function (e) {
        var b = e.target;
        if (b.classList && b.classList.contains("box")) {
          var i = colIndex(b);
          if (i >= 0) { window.open(d.commitBase + d.hashes[i], "_blank"); }
        }
      });
      full.addEventListener("mouseover", function (e) {
        var b = e.target;
        if (b.classList && b.classList.contains("box") && !b.title) {
          var i = colIndex(b);
          if (i >= 0) {
            b.title = d.dates[i] + " " + d.hashes[i].slice(0, 10) +
              (d.bi[i] < 0 ? " (not built)" : "");
          }
        }
      });
    }

    cb.addEventListener("change", function () {
      if (cb.checked) {
        if (!rendered) {
          full.innerHTML = "<p class='note'>Rendering…</p>";
          full.style.display = "";
          built.style.display = "none";
          setTimeout(function () { render(); rendered = true; }, 0);
        } else {
          full.style.display = "";
          built.style.display = "none";
        }
      } else {
        full.style.display = "none";
        built.style.display = "";
      }
    });
  }

  window.BBH_boardDetail = boardDetail;
  window.BBH_simpleChart = simpleChart;
  window.BBH_trend = trendView;
})();
