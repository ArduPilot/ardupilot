/* ArduSub adapter for the preserved ROS2 dashboard markup (read-only and interactive). */
(() => {
  'use strict';

  const controlEnabled = document.body?.dataset?.atalControl === '1';
  const $ = (id) => document.getElementById(id);
  const set = (id, value) => { const el = $(id); if (el) el.textContent = value; };
  const number = (value, digits = 1, unit = '') =>
    Number.isFinite(value) ? `${value.toFixed(digits)}${unit}` : '—';
  const active = (state, key) => state && state[key] === true;
  const tabs = [...document.querySelectorAll('.nav button[id^="tn-"]')];
  const panels = [...document.querySelectorAll('.panel[id^="p-"]')];
  let currentTab = 'sum';
  let latest = null;
  const samples = [];
  let lastSampleAt = 0;

  async function apiJson(url, method = 'GET', body = null) {
    const opts = { method, headers: {} };
    if (body !== null) {
      opts.headers['Content-Type'] = 'application/json';
      opts.body = JSON.stringify(body);
    }
    const resp = await fetch(url, opts);
    let data = {};
    try { data = await resp.json(); } catch (_) { data = { ok: false, error: `HTTP ${resp.status}` }; }
    if (!resp.ok && data.ok === undefined) data.ok = false;
    return data;
  }

  function toast(msg, ok = true) {
    const container = $('toast-container');
    if (!container) return;
    const item = document.createElement('div');
    item.className = `toast-item ${ok ? 'ok' : 'err'}`;
    item.style.cssText = `padding:8px 12px;margin-top:6px;border-radius:6px;font-size:11.5px;font-weight:700;background:${ok ? '#064e3b' : '#7f1d1d'};color:#fff;border:1px solid ${ok ? '#10b981' : '#ef4444'}`;
    item.textContent = msg;
    container.appendChild(item);
    setTimeout(() => item.remove(), 3500);
  }

  function relabelRate(id, label) {
    const value = $(id);
    if (!value) return;
    const wrapper = value.parentElement;
    [...wrapper.childNodes].forEach((node) => {
      if (node.nodeType === Node.TEXT_NODE) node.textContent = '';
    });
    wrapper.insertBefore(document.createTextNode(`${label} `), value);
  }
  relabelRate('hb-thrhz', 'OUT');
  relabelRate('dvl-hz', 'DVL');

  // Strip original inline ROS handlers (blocked by CSP anyway) and wire clean listeners.
  document.querySelectorAll('[onclick], [onchange], [oninput], [onkeydown], [onkeyup]').forEach((el) => {
    for (const attr of [...el.attributes]) {
      if (attr.name.startsWith('on')) el.removeAttribute(attr.name);
    }
  });

  if (!controlEnabled) {
    document.querySelectorAll('button, input, select, textarea').forEach((el) => {
      if (el.closest('.nav') && el.id.startsWith('tn-')) return;
      if (el.id === 'theme-toggle') return;
      el.disabled = true;
      el.classList.add('atal-disabled');
      el.title = 'Start dashboard_api.py with --enable-control for interactive ArduSub mode';
    });
    document.querySelectorAll('a[href^="/api/"], a[href^="/control-lab"], a[href^="/debug"]').forEach((el) => {
      el.removeAttribute('href');
      el.classList.add('atal-disabled');
    });
  } else {
    // Hardware-locked features stay explicitly disabled
    ['fp-arm-untethered-btn', 'btn-start-flash'].forEach((id) => {
      const el = $(id);
      if (el) {
        el.disabled = true;
        el.classList.add('atal-disabled');
        el.title = 'Locked during software development; requires hardware qualification';
      }
    });
  }

  const banner = document.createElement('div');
  banner.id = 'atal-port-banner';
  banner.setAttribute('role', 'status');
  banner.innerHTML = `<span>ARDUSUB · ${controlEnabled ? 'INTERACTIVE CONTROL' : 'READ-ONLY PORT'}</span>` +
    '<span><a href="/">Operator view</a><a href="/diagnostics">Diagnostics</a></span>';
  document.body.prepend(banner);

  try {
    document.documentElement.classList.toggle('light', localStorage.getItem('atal-classic-theme') === 'light');
  } catch (_) { /* ignore */ }
  $('theme-toggle')?.addEventListener('click', () => {
    const light = document.documentElement.classList.toggle('light');
    try { localStorage.setItem('atal-classic-theme', light ? 'light' : 'dark'); } catch (_) { /* ignore */ }
    if (latest && currentTab === 'sum') render(latest);
  });

  // Insert ArduSub Vehicle State & Flight Mode summary card
  const summary = document.createElement('div');
  summary.className = 'card atal-readonly-card';
  summary.innerHTML = '<div class="ct">ArduSub vehicle state</div>' +
    '<div class="atal-kv">Mode <b id="atal-mode">—</b></div>' +
    '<div class="atal-kv">EKF <b id="atal-ekf">—</b></div>' +
    '<div class="atal-kv">Pressure <b id="atal-pressure">—</b></div>' +
    '<div class="atal-kv">Water temp <b id="atal-water-temp">—</b></div>' +
    '<div class="atal-kv">Horizontal pos <b id="atal-position">—</b></div>' +
    '<div class="atal-kv">Mission <b id="atal-mission">—</b></div>' +
    (controlEnabled
      ? '<div class="atal-mode-grid">' +
        ['STABILIZE', 'ALT_HOLD', 'POSHOLD', 'GUIDED', 'AUTO', 'MANUAL'].map((m) =>
          `<button type="button" class="atal-mode-chip" data-atal-mode="${m}">${m}</button>`
        ).join('') + '</div>'
      : '');

  const sum = $('p-sum');
  const columns = sum ? sum.querySelectorAll('.sum-col') : [];
  const outputHeading = $('octo-canvas')?.parentElement?.querySelector('.ct');
  if (outputHeading) outputHeading.textContent = 'SERVO 1–8 · REPORTED PWM';
  if (columns.length) {
    if (!controlEnabled) {
      columns.forEach((column) => {
        [...column.children].forEach((card) => {
          if (!card.querySelector('#ai-canvas') && !card.querySelector('#v-r')) card.hidden = true;
        });
      });
      if (columns[2]) columns[2].hidden = true;
    }
    columns[0].prepend(summary);
  }

  // Raw tab setup
  const raw = $('p-raw');
  if (raw) raw.innerHTML = '<div class="atal-unported">Live ArduSub MAVLink snapshot. Stale fields are null.</div><pre id="atal-raw">Waiting for telemetry…</pre>';

  // Plots tab setup
  const plots = $('p-plots');
  if (plots) {
    const card = document.createElement('div');
    card.className = 'card atal-readonly-card atal-supported';
    card.innerHTML = '<div class="ct">Live ArduSub telemetry · last 60 seconds</div>' +
      '<p class="atal-chart-note">Roll <span style="color:#f87171">■</span> · Pitch <span style="color:#38bdf8">■</span> · Yaw <span style="color:#c084fc">■</span></p>' +
      '<canvas id="atal-attitude-chart" class="atal-chart"></canvas>' +
      '<p class="atal-chart-note">Depth <span style="color:#2dd4bf">■</span> (LOCAL_POSITION_NED)</p>' +
      '<canvas id="atal-depth-chart" class="atal-chart"></canvas>';
    plots.prepend(card);
    if (!controlEnabled) {
      [...plots.children].forEach((c) => { if (c !== card) c.hidden = true; });
    }
  }

  // Mission tab setup (keep timed_mission.html visible when controlEnabled!)
  const mission = $('p-mission');
  if (mission) {
    const status = document.createElement('div');
    status.className = 'card atal-readonly-card atal-supported';
    status.innerHTML = '<div class="ct">ArduSub MAVLink MISSION_CURRENT</div>' +
      '<div class="atal-kv">State <b id="atal-mission-state">—</b></div>' +
      '<div class="atal-kv">Mode <b id="atal-mission-mode">—</b></div>' +
      '<div class="atal-kv">Current item <b id="atal-mission-progress">—</b></div>';
    mission.prepend(status);
    if (!controlEnabled) {
      [...mission.children].forEach((child) => { if (child !== status) child.hidden = true; });
    }
  }

  if (!controlEnabled) {
    $('gate-aim-view')?.closest('.card')?.setAttribute('hidden', '');
    $('gate-guidance-history')?.closest('.card')?.setAttribute('hidden', '');
    for (const panel of panels) {
      if (panel.id === 'p-sum' || panel.id === 'p-raw') continue;
      const notice = document.createElement('div');
      notice.className = 'atal-unported';
      notice.textContent = panel.id === 'p-gate'
        ? 'Proposal telemetry from the optional ROS companion. Read-only.'
        : 'Read-only mode active. Start dashboard_api.py with --enable-control to enable interactive ArduSub controls.';
      panel.prepend(notice);
    }
  }

  function selectTab(name) {
    if (!tabs.some((tab) => tab.id === `tn-${name}`)) name = 'sum';
    currentTab = name;
    panels.forEach((panel) => panel.classList.toggle('show', panel.id === `p-${name}`));
    tabs.forEach((tab) => tab.classList.toggle('on', tab.id === `tn-${name}`));
    if (latest) render(latest);
  }
  tabs.forEach((tab) => tab.addEventListener('click', () => { location.hash = tab.id.slice(3); }));
  window.addEventListener('hashchange', () => selectTab(location.hash.slice(1)));
  selectTab(location.hash.slice(1) || 'sum');

  // ── Interactive ArduSub wiring (active only when controlEnabled) ──
  const pidGroups = {
    roll: ['ATC_ANG_RLL_P', 'ATC_RAT_RLL_P', 'ATC_RAT_RLL_I', 'ATC_RAT_RLL_D', 'ATC_RAT_RLL_IMAX'],
    pitch: ['ATC_ANG_PIT_P', 'ATC_RAT_PIT_P', 'ATC_RAT_PIT_I', 'ATC_RAT_PIT_D', 'ATC_RAT_PIT_IMAX'],
    yaw: ['ATC_ANG_YAW_P', 'ATC_RAT_YAW_P', 'ATC_RAT_YAW_I', 'ATC_RAT_YAW_D', 'ATC_RAT_YAW_IMAX'],
    depth: ['PSC_D_POS_P', 'PSC_D_VEL_P', 'PSC_D_ACC_P', 'PSC_D_ACC_I', 'PSC_D_ACC_D'],
    pos: ['PSC_NE_POS_P', 'PSC_NE_VEL_P', 'PSC_NE_VEL_I', 'PSC_NE_VEL_D'],
    vel: ['WP_SPD', 'WP_SPD_DN', 'WP_SPD_UP', 'WP_RADIUS_M'],
    gen: ['MOT_PWM_MIN', 'MOT_PWM_MAX', 'FRAME_CONFIG', 'BARO_SPEC_GRAV'],
  };
  let activePidGroup = 'roll';
  let cachedParams = {};
  let navQueue = [];
  const navTrail = [];
  let pilotEnabled = false;
  let speedScale = 0.5;
  const pressedKeys = new Set();

  function renderPidGroup() {
    const box = $('p-fields');
    if (!box) return;
    const keys = pidGroups[activePidGroup] || [];
    box.innerHTML = keys.map((k) => {
      const val = cachedParams[k];
      const disp = Number.isFinite(val) ? val : '';
      return `<div class="comp-row" style="display:flex;align-items:center;justify-content:space-between;padding:4px 6px;">` +
        `<label style="font-size:10.5px;font-family:'JetBrains Mono',monospace;">${k}</label>` +
        `<div style="display:flex;gap:4px;align-items:center;">` +
        `<input type="number" step="any" data-atal-param="${k}" value="${disp}" style="width:74px;background:var(--bg0);color:var(--tx);border:1px solid var(--bd);border-radius:4px;padding:2px 5px;font-size:11px;font-family:'JetBrains Mono',monospace;">` +
        `<button type="button" class="set-btn" data-atal-set-param="${k}">SET</button></div></div>`;
    }).join('');
    box.querySelectorAll('[data-atal-set-param]').forEach((btn) => {
      btn.addEventListener('click', async () => {
        const key = btn.getAttribute('data-atal-set-param');
        const input = box.querySelector(`[data-atal-param="${key}"]`);
        const val = parseFloat(input?.value);
        if (!Number.isFinite(val)) return toast(`Invalid value for ${key}`, false);
        const res = await apiJson('/api/set_param', 'POST', { param: key, value: val });
        if (res.ok) { cachedParams[key] = res.value; toast(`Set ${key} = ${res.value}`); }
        else toast(res.error || `Failed ${key}`, false);
      });
    });
  }

  async function refreshParams() {
    const res = await apiJson('/api/get_params');
    if (res.ok && res.params) {
      cachedParams = res.params;
      renderPidGroup();
    }
  }

  async function refreshProfiles() {
    const sel = $('profile-select');
    if (!sel) return;
    const res = await apiJson('/api/profiles');
    if (res.ok) {
      sel.innerHTML = (res.profiles || []).map((p) => `<option value="${p.name}">${p.name} (${p.count})</option>`).join('');
    }
  }

  async function refreshMotorConfig() {
    const list = $('cfg-mt-list');
    if (!list) return;
    const res = await apiJson('/api/motor_pins');
    if (!res.ok) return;
    const pins = res.pins || {};
    const locked = !res.actuators_enabled;
    list.innerHTML = Array.from({ length: 8 }, (_, idx) => {
      const m = idx + 1;
      const dir = cachedParams[`MOT_${m}_DIRECTION`] === -1 ? 'REV' : 'NRM';
      return `<div class="thr-row" style="align-items:center;">` +
        `<div>●</div><div>M${m}</div>` +
        `<div id="cfg-pwm-${m}" style="text-align:right;font-family:'JetBrains Mono',monospace;">1500</div>` +
        `<div id="cfg-thr-${m}" style="text-align:right;">0%</div>` +
        `<input type="range" min="1300" max="1700" step="10" value="1500" data-atal-motor-slider="${m}" ${locked ? 'disabled' : ''}>` +
        `<button type="button" class="btn btn-bl btn-sm" data-atal-motor-test="${m}" ${locked ? 'disabled' : ''}>Test</button>` +
        `<button type="button" class="btn btn-k btn-sm" data-atal-motor-stop="${m}">1500</button>` +
        `<button type="button" class="btn btn-k btn-sm" data-atal-motor-dir="${m}">${dir}</button>` +
        `<div>1.0</div><div style="text-align:center;">GPIO ${pins[String(m)] ?? '?'}</div></div>`;
    }).join('');
    list.querySelectorAll('[data-atal-motor-dir]').forEach((btn) => {
      btn.addEventListener('click', async () => {
        const m = parseInt(btn.getAttribute('data-atal-motor-dir'), 10);
        const nextDir = btn.textContent === 'REV' ? 1 : -1;
        const resDir = await apiJson('/api/set_motor_direction', 'POST', { motor: m, direction: nextDir });
        if (resDir.ok) {
          cachedParams[`MOT_${m}_DIRECTION`] = nextDir;
          btn.textContent = nextDir === -1 ? 'REV' : 'NRM';
          toast(`MOT_${m}_DIRECTION = ${nextDir}`);
        } else toast(resDir.error || 'Failed', false);
      });
    });
    list.querySelectorAll('[data-atal-motor-test]').forEach((btn) => {
      btn.addEventListener('click', async () => {
        const m = parseInt(btn.getAttribute('data-atal-motor-test'), 10);
        const slider = list.querySelector(`[data-atal-motor-slider="${m}"]`);
        const pwm = parseInt(slider?.value || '1500', 10);
        const r = await apiJson('/api/motor_test_pwm', 'POST', { motor: m, pwm, duration_s: 1.0 });
        toast(r.ok ? `M${m} test ${pwm} µs` : (r.error || 'Motor test blocked'), r.ok);
      });
    });
    list.querySelectorAll('[data-atal-motor-stop]').forEach((btn) => {
      btn.addEventListener('click', async () => {
        await apiJson('/api/motor_test_stop', 'POST', {});
        toast('Motor neutral sent');
      });
    });
  }

  function renderNavTable() {
    const tbody = $('nav-mission-tbody');
    if (!tbody) return;
    tbody.innerHTML = navQueue.map((wp, idx) =>
      `<tr><td style="padding:4px 6px;">${idx + 1}</td>` +
      `<td style="padding:4px 6px;">${wp.north.toFixed(2)}</td>` +
      `<td style="padding:4px 6px;">${wp.east.toFixed(2)}</td>` +
      `<td style="padding:4px 6px;">${wp.depth.toFixed(2)}</td>` +
      `<td style="padding:4px 6px;">${wp.yaw.toFixed(0)}°</td>` +
      `<td style="padding:4px 6px;">${wp.hold_sec.toFixed(1)}s</td>` +
      `<td style="padding:4px 6px;">${wp.status || 'pending'}</td>` +
      `<td style="padding:4px 6px;text-align:center;"><button type="button" data-atal-del-wp="${idx}">✕</button></td></tr>`
    ).join('');
    tbody.querySelectorAll('[data-atal-del-wp]').forEach((btn) => {
      btn.addEventListener('click', () => {
        navQueue.splice(parseInt(btn.getAttribute('data-atal-del-wp'), 10), 1);
        renderNavTable();
      });
    });
  }

  function drawNavMap(state) {
    const canvas = $('nav-map-canvas');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width;
    const h = canvas.height;
    ctx.fillStyle = '#070d18';
    ctx.fillRect(0, 0, w, h);
    const scale = 32;
    const cx = w / 2;
    const cy = h / 2;
    ctx.strokeStyle = 'rgba(100,140,200,0.14)';
    ctx.lineWidth = 1;
    for (let m = -10; m <= 10; m++) {
      ctx.beginPath(); ctx.moveTo(cx + m * scale, 0); ctx.lineTo(cx + m * scale, h); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0, cy - m * scale); ctx.lineTo(w, cy - m * scale); ctx.stroke();
    }
    navQueue.forEach((wp, idx) => {
      const px = cx + wp.east * scale;
      const py = cy - wp.north * scale;
      ctx.fillStyle = '#22c55e';
      ctx.beginPath(); ctx.arc(px, py, 5, 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = '#e8edf5';
      ctx.font = '10px JetBrains Mono, monospace';
      ctx.fillText(`W${idx + 1}`, px + 7, py + 3);
    });
    if (navTrail.length > 1) {
      ctx.strokeStyle = '#38bdf8';
      ctx.lineWidth = 2;
      ctx.beginPath();
      navTrail.forEach(([n, e], i) => {
        const px = cx + e * scale;
        const py = cy - n * scale;
        if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
      });
      ctx.stroke();
    }
    if (active(state, 'position_alive') && Number.isFinite(state.pos_n) && Number.isFinite(state.pos_e)) {
      const bx = cx + state.pos_e * scale;
      const by = cy - state.pos_n * scale;
      ctx.fillStyle = '#fbbf24';
      ctx.beginPath(); ctx.arc(bx, by, 6, 0, Math.PI * 2); ctx.fill();
    }
  }

  if (controlEnabled) {
    // Mode chips
    summary.querySelectorAll('[data-atal-mode]').forEach((btn) => {
      btn.addEventListener('click', async () => {
        const m = btn.getAttribute('data-atal-mode');
        const res = await apiJson('/api/set_mode', 'POST', { mode: m });
        toast(res.ok ? `Mode → ${m}` : (res.recent_status || res.error || `Mode ${m} rejected`), res.ok);
      });
    });

    // Arm & Kill
    [$('btn-arm-toggle'), $('th-arm-btn')].forEach((btn) => {
      btn?.addEventListener('click', async () => {
        const armTarget = !(latest && latest.armed === true);
        const res = await apiJson('/api/arm', 'POST', { arm: armTarget });
        toast(res.ok ? (armTarget ? 'ARMED' : 'DISARMED') : (res.recent_status || res.error || 'Arm failed'), res.ok);
      });
    });
    $('btn-nuke-all')?.addEventListener('click', async () => {
      const res = await apiJson('/api/kill_all', 'POST', {});
      toast(res.ok ? 'EMERGENCY KILL SENT' : (res.error || 'Kill failed'), res.ok);
    });
    $('dash-guided-btn')?.addEventListener('click', () => selectTab('throttle'));

    // PID Tabs & Save
    Object.keys(pidGroups).forEach((g) => {
      $(`pt-${g}`)?.addEventListener('click', () => {
        activePidGroup = g;
        document.querySelectorAll('.pid-tab[id^="pt-"]').forEach((t) => t.classList.toggle('on', t.id === `pt-${g}`));
        renderPidGroup();
      });
    });
    document.querySelector('#p-sum .comp-pid + div button:first-child')?.addEventListener('click', async () => {
      const updates = {};
      document.querySelectorAll('#p-fields [data-atal-param]').forEach((inp) => {
        const val = parseFloat(inp.value);
        if (Number.isFinite(val)) updates[inp.getAttribute('data-atal-param')] = val;
      });
      const res = await apiJson('/api/save_params', 'POST', { params: updates });
      toast(res.ok ? `Applied ${Object.keys(res.applied || {}).length} ArduSub parameters` : 'Some parameters failed', res.ok);
    });

    // Gain Profiles
    document.querySelector('button[title="delete selected"]')?.previousElementSibling?.addEventListener('click', async () => {
      const name = $('profile-select')?.value;
      if (!name) return;
      const res = await apiJson('/api/load_profile', 'POST', { name });
      toast(res.ok ? `Loaded profile ${name}` : (res.error || 'Load failed'), res.ok);
      if (res.ok) refreshParams();
    });
    document.querySelector('button[title="delete selected"]')?.addEventListener('click', async () => {
      const name = $('profile-select')?.value;
      if (!name) return;
      await apiJson('/api/delete_profile', 'POST', { name });
      refreshProfiles();
    });
    document.querySelector('#profile-select')?.closest('.card')?.querySelector('button.btn-bl')?.addEventListener('click', async () => {
      const name = window.prompt('Profile name:');
      if (!name) return;
      const res = await apiJson('/api/save_profile', 'POST', { name });
      toast(res.ok ? `Saved profile ${res.name}` : (res.error || 'Save failed'), res.ok);
      refreshProfiles();
    });

    // Calibration buttons
    const calButtons = [...document.querySelectorAll('#cal-fb')].map((fb) => fb.previousElementSibling?.querySelectorAll('button')).flat().filter(Boolean);
    if (calButtons.length >= 4) {
      calButtons[0].addEventListener('click', async () => {
        const r = await apiJson('/api/reset_dvl_origin', 'POST', {});
        toast(r.ok ? 'EKF local origin updated' : (r.error || 'Failed'), r.ok);
      });
      calButtons[2].addEventListener('click', async () => {
        const r = await apiJson('/api/tare_depth', 'POST', {});
        toast(r.ok ? 'Bar30 pressure calibration sent' : (r.recent_status || r.error || 'Failed'), r.ok);
      });
      calButtons[3].addEventListener('click', async () => {
        const r = await apiJson('/api/reset_gyro', 'POST', {});
        toast(r.ok ? 'Gyro calibration sent' : (r.recent_status || r.error || 'Failed'), r.ok);
      });
    }

    // Navigation mode buttons & sp-inputs
    $('m-hold')?.addEventListener('click', async () => {
      const r = await apiJson('/api/hold_position', 'POST', {});
      toast(r.ok ? 'Hold mode requested' : (r.recent_status || r.error || 'Hold rejected'), r.ok);
    });
    $('m-set')?.addEventListener('click', () => {
      const sp = $('sp-inputs');
      if (sp) { sp.style.opacity = '1'; sp.style.pointerEvents = 'auto'; }
    });
    ['sp-r', 'sp-p', 'sp-y', 'sp-d', 'sp-fwd', 'sp-lat', 'sp-vz'].forEach((id) => {
      $(id)?.addEventListener('change', async () => {
        const r = await apiJson('/api/set_target', 'POST', {
          roll: parseFloat($('sp-r')?.value || '0') || 0,
          pitch: parseFloat($('sp-p')?.value || '0') || 0,
          yaw: parseFloat($('sp-y')?.value || '0') || 0,
          depth: parseFloat($('sp-d')?.value || '0') || 0,
          fwd: parseFloat($('sp-fwd')?.value || '0') || 0,
          lat: parseFloat($('sp-lat')?.value || '0') || 0,
          vz: parseFloat($('sp-vz')?.value || '0') || 0,
          mode: 'setpoint',
        });
        toast(r.ok ? 'Target updated' : (r.error || 'Target error'), r.ok);
      });
    });

    // Data logging
    $('btn-log-start')?.addEventListener('click', async () => {
      const st = await apiJson('/api/log/status');
      const res = await apiJson(st.active ? '/api/log/stop' : '/api/log/start', 'POST', {});
      const btn = $('btn-log-start');
      if (btn) btn.textContent = res.active ? '⏹ Stop Log' : '▶ Start Log';
      const dl = $('btn-log-download');
      if (dl && res.file) dl.style.display = 'block';
    });
    $('btn-log-download')?.addEventListener('click', () => { window.location.href = '/api/recording/download'; });

    // Guided Pilot (Throttle tab)
    $('pilot-master-toggle')?.addEventListener('change', (ev) => {
      pilotEnabled = ev.target.checked;
      set('pilot-status-text', pilotEnabled ? 'ARMED KEYBOARD' : 'STANDBY');
      if (!pilotEnabled) apiJson('/api/set_target', 'POST', { x: 0, y: 0, z: 500, r: 0, mode: 'manual' });
    });
    $('btn-stop-pilot')?.addEventListener('click', () => {
      pressedKeys.clear();
      apiJson('/api/set_target', 'POST', { x: 0, y: 0, z: 500, r: 0, mode: 'manual' });
      toast('ALL STOP sent');
    });
    [['speed-btn-25', 0.25], ['speed-btn-50', 0.5], ['speed-btn-100', 1.0]].forEach(([id, sc]) => {
      $(id)?.addEventListener('click', () => { speedScale = sc; });
    });
    window.addEventListener('keydown', (e) => { if (pilotEnabled) pressedKeys.add(e.code); });
    window.addEventListener('keyup', (e) => { pressedKeys.delete(e.code); });
    setInterval(() => {
      if (!pilotEnabled || currentTab !== 'throttle') return;
      let x = 0, y = 0, z = 500, r = 0;
      if (pressedKeys.has('KeyW')) x += 1000 * speedScale;
      if (pressedKeys.has('KeyS')) x -= 1000 * speedScale;
      if (pressedKeys.has('KeyD')) y += 1000 * speedScale;
      if (pressedKeys.has('KeyA')) y -= 1000 * speedScale;
      if (pressedKeys.has('KeyE')) r += 1000 * speedScale;
      if (pressedKeys.has('KeyQ')) r -= 1000 * speedScale;
      if (pressedKeys.has('ShiftLeft') || pressedKeys.has('KeyR')) z += 400 * speedScale;
      if (pressedKeys.has('ControlLeft') || pressedKeys.has('KeyF')) z -= 400 * speedScale;
      apiJson('/api/set_target', 'POST', { x, y, z, r, mode: 'manual', lease_s: 0.4 });
    }, 100);

    // Waypoint tab
    $('nav-mode-single-btn')?.addEventListener('click', () => {
      $('nav-deck-single').style.display = 'block';
      $('nav-deck-mission').style.display = 'none';
      $('nav-mode-single-btn').classList.add('on');
      $('nav-mode-mission-btn').classList.remove('on');
    });
    $('nav-mode-mission-btn')?.addEventListener('click', () => {
      $('nav-deck-single').style.display = 'none';
      $('nav-deck-mission').style.display = 'block';
      $('nav-mode-single-btn').classList.remove('on');
      $('nav-mode-mission-btn').classList.add('on');
    });
    const wpPanelButtons = [...document.querySelectorAll('#nav-deck-single button')];
    wpPanelButtons[0]?.addEventListener('click', async () => {
      const res = await apiJson('/api/set_target_waypoint', 'POST', {
        north: parseFloat($('wp-target-north')?.value || '0') || 0,
        east: parseFloat($('wp-target-east')?.value || '0') || 0,
        depth: parseFloat($('wp-target-depth')?.value || '0') || 0,
        yaw: parseFloat($('wp-target-yaw')?.value || '0') || 0,
      });
      toast(res.ok ? 'Waypoint commanded (GUIDED)' : (res.recent_status || res.error || 'Rejected'), res.ok);
    });
    wpPanelButtons[1]?.addEventListener('click', async () => {
      const res = await apiJson('/api/hold_position', 'POST', {});
      toast(res.ok ? 'Hold position requested' : (res.error || 'Rejected'), res.ok);
    });
    wpPanelButtons[2]?.addEventListener('click', async () => {
      const res = await apiJson('/api/set_target_waypoint', 'POST', { north: 0, east: 0, depth: 0.5, yaw: 0 });
      toast(res.ok ? 'Return to (0,0) commanded' : (res.error || 'Rejected'), res.ok);
    });

    // Multi-WP Queue buttons
    document.querySelector('#nav-deck-mission button[title="Add current inputs as new WP"]')?.addEventListener('click', () => {
      navQueue.push({
        north: parseFloat($('wp-target-north')?.value || '0') || 0,
        east: parseFloat($('wp-target-east')?.value || '0') || 0,
        depth: parseFloat($('wp-target-depth')?.value || '0.5') || 0.5,
        yaw: parseFloat($('wp-target-yaw')?.value || '0') || 0,
        max_vel: parseFloat($('wp-target-speed')?.value || '0.4') || 0.4,
        tolerance: parseFloat($('wp-target-radius')?.value || '0.25') || 0.25,
        hold_sec: 2.0,
        status: 'pending',
      });
      renderNavTable();
    });
    document.querySelector('#nav-deck-mission button[title="Clear queue"]')?.addEventListener('click', () => {
      navQueue = [];
      renderNavTable();
    });
    $('nav-btn-start-mission')?.addEventListener('click', async () => {
      const res = await apiJson('/api/mission/start', 'POST', { wps: navQueue });
      toast(res.ok ? 'Mission started' : (res.error || 'Mission rejected'), res.ok);
    });
    $('nav-btn-pause-mission')?.addEventListener('click', async () => {
      await apiJson('/api/mission/pause', 'POST', {});
      toast('Mission paused');
    });
    $('nav-btn-skip-wp')?.addEventListener('click', async () => {
      await apiJson('/api/mission/skip', 'POST', {});
    });
    $('nav-btn-abort-mission')?.addEventListener('click', async () => {
      await apiJson('/api/mission/abort', 'POST', {});
      toast('Mission aborted');
    });

    // Flight Plan buttons
    $('fp-start-btn')?.addEventListener('click', async () => {
      const res = await apiJson('/api/flight_plan/start', 'POST', {
        settings: {
          submerge_depth_m: parseFloat($('fp-p2-depth')?.value || '0.4') || 0.4,
          yaw_angle_deg: parseFloat($('fp-p2-yaw')?.value || '0') || 0,
          surge_effort: parseFloat($('fp-p2-vel')?.value || '0.2') || 0.2,
          surge_time_sec: parseFloat($('fp-p2-t-fwd')?.value || '4.0') || 4.0,
          dive_time_sec: parseFloat($('fp-p2-t-depth')?.value || '4.0') || 4.0,
          yaw_timeout_sec: parseFloat($('fp-p2-t-turn')?.value || '10.0') || 10.0,
        },
      });
      toast(res.ok ? 'Flight plan started' : (res.error || 'Flight plan failed'), res.ok);
    });
    $('fp-stop-btn')?.addEventListener('click', async () => {
      await apiJson('/api/flight_plan/stop', 'POST', {});
      toast('Flight plan stopped');
    });

    // Periodic status refresh for debug logs & flight plan
    setInterval(async () => {
      if (currentTab === 'sum') {
        const dbg = await apiJson('/api/debug_state');
        const list = $('logs-list');
        if (list && Array.isArray(dbg.logs) && dbg.logs.length) {
          list.innerHTML = dbg.logs.slice(-20).map((t) => `<div>${t}</div>`).join('');
        }
      } else if (currentTab === 'flightplan') {
        const fp = await apiJson('/api/flight_plan/status');
        set('fp-status-badge', (fp.status || 'idle').toUpperCase());
        set('fp-current-step-title', `Step ${fp.current_step ?? 0}: ${fp.step_name || 'Ready'}`);
        set('fp-timing-display', `Step: ${fp.step_elapsed ?? 0}s / ${fp.step_duration ?? 0}s | Total: ${fp.total_elapsed ?? 0}s`);
      }
    }, 1000);

    refreshParams().then(refreshMotorConfig);
    refreshProfiles();
  }

  // ── Canvas Renderers & Telemetry View Updates ──
  function drawAttitude(state) {
    [$('ai-canvas'), $('th-ai-canvas')].forEach((canvas) => {
      if (!canvas) return;
      const ctx = canvas.getContext('2d');
      const width = canvas.width;
      const height = canvas.height;
      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = '#091522';
      ctx.fillRect(0, 0, width, height);
      if (!active(state, 'imu_alive') || !Number.isFinite(state.roll) || !Number.isFinite(state.pitch)) {
        ctx.fillStyle = '#fbbf24';
        ctx.font = '12px Inter, sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('IMU unavailable', width / 2, height / 2);
        return;
      }
      ctx.save();
      ctx.translate(width / 2, height / 2);
      ctx.rotate(-state.roll * Math.PI / 180);
      const horizon = state.pitch * 2.2;
      ctx.fillStyle = '#286782';
      ctx.fillRect(-width, -height * 2 + horizon, width * 2, height * 2);
      ctx.fillStyle = '#746044';
      ctx.fillRect(-width, horizon, width * 2, height * 2);
      ctx.strokeStyle = '#e9f2fa';
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(-width, horizon); ctx.lineTo(width, horizon); ctx.stroke();
      ctx.restore();
      ctx.strokeStyle = '#fbbf24';
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(width / 2 - 30, height / 2); ctx.lineTo(width / 2 - 10, height / 2);
      ctx.moveTo(width / 2 + 10, height / 2); ctx.lineTo(width / 2 + 30, height / 2);
      ctx.stroke();
    });
  }

  function drawOutputs(state) {
    [$('octo-canvas'), $('th-octo-canvas')].forEach((canvas) => {
      if (!canvas) return;
      const ctx = canvas.getContext('2d');
      const w = canvas.width;
      const h = canvas.height;
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = '#091522';
      ctx.fillRect(0, 0, w, h);
      ctx.font = '12px JetBrains Mono, monospace';
      if (!active(state, 'output_alive')) {
        ctx.fillStyle = '#fbbf24';
        ctx.textAlign = 'center';
        ctx.fillText('Motor output unavailable', w / 2, h / 2);
        return;
      }
      state.pwm.forEach((pwm, index) => {
        const x = 14 + (index % 2) * (w / 2);
        const y = 20 + Math.floor(index / 2) * (h / 4);
        const delta = Number.isFinite(pwm) ? pwm - 1500 : 0;
        ctx.fillStyle = '#a8c3d5';
        ctx.textAlign = 'left';
        ctx.fillText(`S${index + 1}  ${pwm ?? '—'} μs`, x, y);
        ctx.fillStyle = delta > 0 ? '#22c55e' : delta < 0 ? '#fb7185' : '#64748b';
        ctx.fillRect(x, y + 8, Math.min(110, Math.abs(delta) / 4), 7);
      });
    });
  }

  function drawChart(id, series, unit) {
    const canvas = $(id);
    if (!canvas) return;
    const ratio = window.devicePixelRatio || 1;
    const width = Math.max(300, Math.floor(canvas.clientWidth * ratio));
    const height = Math.max(140, Math.floor(canvas.clientHeight * ratio));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width; canvas.height = height;
    }
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = '#091522';
    ctx.fillRect(0, 0, width, height);
    const values = samples.flatMap((sample) => series.map(([key]) => sample[key]).filter(Number.isFinite));
    if (!values.length) {
      ctx.fillStyle = '#fbbf24';
      ctx.font = `${12 * ratio}px Inter, sans-serif`;
      ctx.fillText('No fresh data', 16 * ratio, height / 2);
      return;
    }
    let min = Math.min(...values);
    let max = Math.max(...values);
    const padding = Math.max(unit === 'm' ? 0.25 : 5, (max - min) * .15);
    min -= padding; max += padding;
    const left = 44 * ratio, right = width - 12 * ratio, top = 15 * ratio, bottom = height - 24 * ratio;
    ctx.strokeStyle = '#35516b'; ctx.lineWidth = ratio;
    ctx.strokeRect(left, top, right - left, bottom - top);
    ctx.fillStyle = '#9baec8'; ctx.font = `${10 * ratio}px JetBrains Mono, monospace`;
    ctx.fillText(`${max.toFixed(1)}${unit}`, 4 * ratio, top + 9 * ratio);
    ctx.fillText(`${min.toFixed(1)}${unit}`, 4 * ratio, bottom);
    for (const [key, color] of series) {
      ctx.strokeStyle = color; ctx.lineWidth = 2 * ratio; ctx.beginPath();
      let drawing = false;
      samples.forEach((sample, index) => {
        const value = sample[key];
        if (!Number.isFinite(value)) { drawing = false; return; }
        const x = left + index / Math.max(1, samples.length - 1) * (right - left);
        const y = bottom - (value - min) / (max - min) * (bottom - top);
        if (drawing) ctx.lineTo(x, y); else ctx.moveTo(x, y);
        drawing = true;
      });
      ctx.stroke();
    }
  }

  function render(state) {
    latest = state;
    const linked = active(state, 'link_alive');
    banner.className = linked ? (state.demo_mode ? 'demo' : '') : 'disconnected';
    banner.firstElementChild.textContent = !linked
      ? `ARDUSUB · NO LIVE VEHICLE LINK · ${controlEnabled ? 'CONTROL ARMED' : 'READ-ONLY'}`
      : state.demo_mode
        ? `SOFTWARE DEMO · SYNTHETIC SENSORS · ${controlEnabled ? 'INTERACTIVE' : 'READ-ONLY'}`
        : `ARDUSUB · LIVE TELEMETRY · ${controlEnabled ? 'INTERACTIVE' : 'READ-ONLY'}`;

    set('hb-imuhz', active(state, 'imu_alive') ? number(state.imu_hz, 0) : '—');
    set('hb-thrhz', active(state, 'output_alive') ? 'live' : '—');
    set('dvl-hz', active(state, 'dvl_alive') ? number(state.dvl_hz, 1) : (active(state, 'position_alive') ? 'ekf' : '—'));
    $('hb-dot')?.classList.toggle('on', active(state, 'imu_alive'));
    $('thr-dot')?.classList.toggle('on', active(state, 'output_alive'));
    $('dvl-dot')?.classList.toggle('on', active(state, 'dvl_alive') || active(state, 'position_alive'));

    const armedText = !linked ? 'LINK LOST' : state.armed_known ? (state.armed ? 'ARMED' : 'DISARMED') : 'ARM UNKNOWN';
    set('st-txt', armedText);
    $('st-dot')?.classList.toggle('on', state.armed_known && state.armed === true);
    if (controlEnabled) {
      const btnText = (state.armed_known && state.armed) ? '▼ DISARM SYSTEM' : '▲ ARM SYSTEM';
      set('btn-arm-toggle', btnText);
      set('th-arm-btn', (state.armed_known && state.armed) ? '▼ DISARM' : '▲ ARM');
    }

    set('atal-mode', state.vehicle_mode || '—');
    summary.querySelectorAll('[data-atal-mode]').forEach((b) => {
      b.classList.toggle('active', b.getAttribute('data-atal-mode') === state.vehicle_mode);
    });
    const ekfLabel = !state.ekf_status_alive ? 'UNAVAILABLE'
      : state.ekf_uninitialized ? 'UNINITIALIZED'
      : state.ekf_absolute_position_ok ? 'POS ABS READY'
      : state.ekf_relative_position_ok ? 'POS REL READY (DVL)'
      : state.ekf_attitude_ok ? 'ATTITUDE ONLY' : 'NOT READY';
    set('atal-ekf', ekfLabel);
    set('atal-pressure', active(state, 'pressure_alive') ? number(state.pressure_mbar, 1, ' mbar') : '—');
    set('atal-water-temp', active(state, 'pressure_alive') ? number(state.water_temp, 1, ' °C') : '—');
    set('atal-position', active(state, 'position_alive')
      ? `${number(state.pos_n, 2, ' m N')} / ${number(state.pos_e, 2, ' m E')}` : '—');
    set('atal-mission', active(state, 'mission_alive') ? `${state.mission_state || '—'} · ${state.mission_seq ?? '—'} / ${state.mission_total ?? '—'}` : '—');
    set('atal-mission-state', active(state, 'mission_alive') ? state.mission_state || '—' : '—');
    set('atal-mission-mode', active(state, 'mission_alive') ? state.mission_mode || '—' : '—');
    set('atal-mission-progress', active(state, 'mission_alive')
      ? `${state.mission_seq ?? '—'} / ${state.mission_total ?? '—'}` : '—');

    if (active(state, 'output_alive') && Array.isArray(state.pwm)) {
      state.pwm.forEach((p, idx) => {
        set(`cfg-pwm-${idx + 1}`, Number.isFinite(p) ? `${p}` : '—');
        set(`cfg-thr-${idx + 1}`, Number.isFinite(p) ? `${Math.round((p - 1500) / 2)}%` : '—');
      });
    }

    set('gate-link-pill', state.gate_link_alive ? (state.gate_link_ok ? 'LINK OK' : 'LINK DOWN') : 'LINK UNKNOWN');
    set('gate-guidance-freshness', state.gate_guidance_alive ? 'GUIDANCE LIVE' : 'NO GUIDANCE');
    set('gate-state-name', state.gate_guidance_alive ? state.gate_state || '—' : 'WAITING');
    set('gate-state-reason', state.gate_guidance_alive ? 'ROS companion proposal' : 'No fresh guidance telemetry');
    set('gate-validity-banner', state.gate_proposal_valid ? 'VALID PROPOSAL' : 'NO VALID PROPOSAL');
    for (const axis of ['forward', 'lateral', 'vertical', 'yaw']) {
      set(`gate-${axis}-value`, state.gate_proposal_valid ? number(state[`gate_${axis}`], 2) : '—');
    }

    set('v-r', active(state, 'imu_alive') ? number(state.roll, 1, '°') : '—');
    set('v-p', active(state, 'imu_alive') ? number(state.pitch, 1, '°') : '—');
    set('v-y', active(state, 'imu_alive') ? number(state.yaw, 1, '°') : '—');
    set('v-d', active(state, 'position_alive') ? number(state.depth_m, 2, 'm') : '—');
    set('depth-text', active(state, 'position_alive') ? number(state.depth_m, 2, 'm') : '—');
    set('th-mode', state.vehicle_mode || '—');
    set('th-head-txt', active(state, 'imu_alive') ? number(state.yaw, 1, '°') : '—');
    set('th-depth-txt', active(state, 'position_alive') ? number(state.depth_m, 2, 'm') : '—');
    set('dvl-x', active(state, 'position_alive') ? number(state.pos_n, 2, ' m') : '—');
    set('dvl-y', active(state, 'position_alive') ? number(state.pos_e, 2, ' m') : '—');
    set('dvl-alt', active(state, 'dvl_alive') ? number(state.dvl_altitude, 2, ' m') : '—');
    set('dvl-fom', active(state, 'dvl_alive') ? number(state.dvl_fom, 3, ' m/s') : '—');
    set('nav-osd-n', active(state, 'position_alive') ? number(state.pos_n, 2) : '—');
    set('nav-osd-e', active(state, 'position_alive') ? number(state.pos_e, 2) : '—');
    set('nav-osd-hdg', active(state, 'imu_alive') ? number(state.yaw, 1, '°') : '—');
    set('nav-att-roll', active(state, 'imu_alive') ? number(state.roll, 1, '°') : '—');
    set('nav-att-pitch', active(state, 'imu_alive') ? number(state.pitch, 1, '°') : '—');
    set('nav-att-yaw', active(state, 'imu_alive') ? number(state.yaw, 1, '°') : '—');
    set('nav-att-depth', active(state, 'position_alive') ? number(state.depth_m, 2, ' m') : '—');
    set('nav-dvl-lock-badge', active(state, 'dvl_alive') ? (state.dvl_locked ? '● LOCKED' : '○ NO LOCK') : '○ UNAVAILABLE');
    set('nav-dvl-alt', active(state, 'dvl_alive') ? number(state.dvl_altitude, 2) : '—');
    set('nav-dvl-fom', active(state, 'dvl_alive') ? number(state.dvl_fom, 3) : '—');

    const depth = active(state, 'position_alive') ? state.depth_m : null;
    const cursor = $('depth-cursor');
    const depthMax = Number.isFinite(depth) ? Math.max(0.5, Math.ceil(depth * 2) / 2) : 0.5;
    for (let i = 0; i < 5; i++) set(`d-scale-${i}`, number(depthMax * i / 4, 2, 'm'));
    if (cursor) {
      cursor.style.display = Number.isFinite(depth) ? 'block' : 'none';
      cursor.style.top = Number.isFinite(depth) ? `${Math.max(0, Math.min(100, depth / depthMax * 100))}%` : '0%';
    }
    const fill = $('depth-fill');
    if (fill) fill.style.height = Number.isFinite(depth) ? `${Math.max(0, Math.min(100, depth / depthMax * 100))}%` : '0%';

    if (active(state, 'position_alive') && Number.isFinite(state.pos_n) && Number.isFinite(state.pos_e)) {
      navTrail.push([state.pos_n, state.pos_e]);
      if (navTrail.length > 300) navTrail.shift();
    }

    const rawPre = $('atal-raw');
    if (rawPre && currentTab === 'raw') rawPre.textContent = JSON.stringify(state, null, 2);
    if (currentTab === 'sum' || currentTab === 'throttle') {
      drawAttitude(state);
      drawOutputs(state);
    } else if (currentTab === 'plots') {
      drawChart('atal-attitude-chart', [['roll', '#f87171'], ['pitch', '#38bdf8'], ['yaw', '#c084fc']], '°');
      drawChart('atal-depth-chart', [['depth', '#2dd4bf']], 'm');
    } else if (currentTab === 'waypoint') {
      drawNavMap(state);
    }
  }

  const stream = new EventSource('/api/stream');
  stream.onmessage = (event) => {
    try {
      const state = JSON.parse(event.data);
      const now = Date.now();
      if (now - lastSampleAt >= 200) {
        samples.push({
          roll: state.imu_alive ? state.roll : null,
          pitch: state.imu_alive ? state.pitch : null,
          yaw: state.imu_alive ? state.yaw : null,
          depth: state.position_alive ? state.depth_m : null,
        });
        if (samples.length > 300) samples.shift();
        lastSampleAt = now;
      }
      render(state);
    } catch (error) { console.error('Invalid ArduSub state', error); }
  };
  stream.onerror = () => {
    render({
      ...latest, link_alive: false, armed_known: false, imu_alive: false,
      pressure_alive: false, position_alive: false, output_alive: false,
      mission_alive: false, ekf_status_alive: false, dvl_alive: false,
      gate_link_alive: false, gate_guidance_alive: false, gate_proposal_valid: false,
    });
    banner.firstElementChild.textContent = 'ARDUSUB · TELEMETRY STREAM DISCONNECTED';
  };
})();
