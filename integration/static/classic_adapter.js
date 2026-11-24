/* Read-only ArduSub wiring for the original ROS2 dashboard markup. */
(() => {
  'use strict';

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
  relabelRate('dvl-hz', 'POS');

  // The original page has inline handlers for ROS writes. The source scripts are
  // absent; remove all handlers and disable every non-navigation control.
  document.querySelectorAll('[onclick], [onchange], [oninput], [onkeydown], [onkeyup]').forEach((el) => {
    for (const attr of [...el.attributes]) {
      if (attr.name.startsWith('on')) el.removeAttribute(attr.name);
    }
  });
  document.querySelectorAll('button, input, select, textarea').forEach((el) => {
    if (el.closest('.nav') && el.id.startsWith('tn-')) return;
    if (el.id === 'theme-toggle') return;
    el.disabled = true;
    el.classList.add('atal-disabled');
    el.title = 'ROS2 control has not been ported to ArduSub';
  });
  document.querySelectorAll('a[href^="/api/"], a[href^="/control-lab"], a[href^="/debug"]').forEach((el) => {
    el.removeAttribute('href');
    el.classList.add('atal-disabled');
  });

  const banner = document.createElement('div');
  banner.id = 'atal-port-banner';
  banner.setAttribute('role', 'status');
  banner.innerHTML = '<span>ARDUSUB · READ-ONLY DASHBOARD PORT</span><a href="/">Current operator dashboard</a>';
  document.body.prepend(banner);

  try {
    document.documentElement.classList.toggle('light', localStorage.getItem('atal-classic-theme') === 'light');
  } catch (_) { /* Browser storage may be unavailable. */ }
  $('theme-toggle')?.addEventListener('click', () => {
    const light = document.documentElement.classList.toggle('light');
    try { localStorage.setItem('atal-classic-theme', light ? 'light' : 'dark'); } catch (_) { /* Local-only preference. */ }
    if (latest && currentTab === 'sum') render(latest);
  });

  const summary = document.createElement('div');
  summary.className = 'card atal-readonly-card';
  summary.innerHTML = '<div class="ct">ArduSub vehicle state</div>' +
    '<div class="atal-kv">Mode <b id="atal-mode">—</b></div>' +
    '<div class="atal-kv">EKF <b id="atal-ekf">—</b></div>' +
    '<div class="atal-kv">Pressure <b id="atal-pressure">—</b></div>' +
    '<div class="atal-kv">Water temperature <b id="atal-water-temp">—</b></div>' +
    '<div class="atal-kv">Horizontal position <b id="atal-position">—</b></div>' +
    '<div class="atal-kv">Mission <b id="atal-mission">—</b></div>';
  const sum = $('p-sum');
  const columns = sum ? sum.querySelectorAll('.sum-col') : [];
  const outputHeading = $('octo-canvas')?.parentElement?.querySelector('.ct');
  if (outputHeading) outputHeading.textContent = 'SERVO 1–8 · REPORTED PWM';
  if (columns.length) {
    // Keep the original orientation/depth/thruster card. Other dashboard cards
    // display ROS controller state or controls that have no ArduSub equivalent.
    columns.forEach((column) => {
      [...column.children].forEach((card) => {
        if (!card.querySelector('#ai-canvas') && !card.querySelector('#v-r')) card.hidden = true;
      });
    });
    if (columns[2]) columns[2].hidden = true;
    columns[0].prepend(summary);
  }

  const raw = $('p-raw');
  if (raw) raw.innerHTML = '<div class="atal-unported">Live, read-only MAVLink snapshot. Unknown or stale fields are null.</div><pre id="atal-raw">Waiting for telemetry…</pre>';
  const plots = $('p-plots');
  if (plots) {
    const card = document.createElement('div');
    card.className = 'card atal-readonly-card atal-supported';
    card.innerHTML = '<div class="ct">Live ArduSub telemetry · last 60 seconds</div>' +
      '<p class="atal-chart-note">Roll <span style="color:#f87171">■</span> · Pitch <span style="color:#38bdf8">■</span> · Yaw <span style="color:#c084fc">■</span>. Gaps mean stale IMU data.</p>' +
      '<canvas id="atal-attitude-chart" class="atal-chart" aria-label="Attitude history"></canvas>' +
      '<p class="atal-chart-note">Depth <span style="color:#2dd4bf">■</span> from LOCAL_POSITION_NED. No line means position telemetry is unavailable.</p>' +
      '<canvas id="atal-depth-chart" class="atal-chart" aria-label="Depth history"></canvas>';
    plots.prepend(card);
  }
  const mission = $('p-mission');
  if (mission) {
    const status = document.createElement('div');
    status.className = 'card atal-readonly-card atal-supported';
    status.innerHTML = '<div class="ct">ArduSub mission status · read only</div>' +
      '<div class="atal-kv">State <b id="atal-mission-state">—</b></div>' +
      '<div class="atal-kv">Mode <b id="atal-mission-mode">—</b></div>' +
      '<div class="atal-kv">Current item <b id="atal-mission-progress">—</b></div>';
    mission.prepend(status);
    [...mission.children].forEach((child) => { if (child !== status) child.hidden = true; });
  }
  $('gate-aim-view')?.closest('.card')?.setAttribute('hidden', '');
  $('gate-guidance-history')?.closest('.card')?.setAttribute('hidden', '');
  const gate = $('p-gate');
  if (gate) {
    const stateHeading = gate.querySelector('.gate-top-grid .ct');
    if (stateHeading) stateHeading.textContent = 'Guidance state';
    const correctionNote = gate.querySelector('.gate-command-grid')?.previousElementSibling?.lastElementChild;
    if (correctionNote) correctionNote.textContent = 'Normalized proposal axes · no motor command is sent';
  }
  for (const panel of panels) {
    if (panel.id === 'p-sum' || panel.id === 'p-raw') continue;
    const notice = document.createElement('div');
    notice.className = 'atal-unported';
    notice.textContent = panel.id === 'p-gate'
      ? 'Proposal telemetry from the optional ROS companion. No camera detection or vehicle command is inferred from it.'
      : panel.id === 'p-mission'
        ? 'ArduSub mission status is live below. The original ROS mission builder and execution controls are not ported.'
        : panel.id === 'p-plots'
          ? 'Live ArduSub plots below. The original ROS controller plots are hidden until their fields are mapped.'
        : 'Original ROS2 layout preserved. This page is not yet connected to ArduSub; its controls are disabled.';
    panel.prepend(notice);
    if (panel.id !== 'p-gate' && panel.id !== 'p-mission') {
      [...panel.children].forEach((child) => {
        if (child !== notice && !child.classList.contains('atal-supported')) child.hidden = true;
      });
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

  function drawAttitude(state) {
    const canvas = $('ai-canvas');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = '#091522';
    ctx.fillRect(0, 0, width, height);
    if (!active(state, 'imu_alive') || !Number.isFinite(state.roll) || !Number.isFinite(state.pitch)) {
      ctx.fillStyle = '#fbbf24';
      ctx.font = '14px Inter, sans-serif';
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
    ctx.beginPath();
    ctx.moveTo(-width, horizon);
    ctx.lineTo(width, horizon);
    ctx.stroke();
    ctx.restore();
    ctx.strokeStyle = '#fbbf24';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(width / 2 - 44, height / 2);
    ctx.lineTo(width / 2 - 12, height / 2);
    ctx.moveTo(width / 2 + 12, height / 2);
    ctx.lineTo(width / 2 + 44, height / 2);
    ctx.stroke();
  }

  function drawOutputs(state) {
    const canvas = $('octo-canvas');
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
      const y = 18 + Math.floor(index / 2) * (h / 4);
      const delta = Number.isFinite(pwm) ? pwm - 1500 : 0;
      ctx.fillStyle = '#a8c3d5';
      ctx.textAlign = 'left';
      ctx.fillText(`S${index + 1}  ${pwm ?? '—'} μs`, x, y);
      ctx.fillStyle = delta > 0 ? '#22c55e' : delta < 0 ? '#fb7185' : '#64748b';
      ctx.fillRect(x, y + 8, Math.min(110, Math.abs(delta) / 4), 7);
    });
  }

  function drawChart(id, series, unit) {
    const canvas = $(id);
    if (!canvas) return;
    const ratio = window.devicePixelRatio || 1;
    const width = Math.max(300, Math.floor(canvas.clientWidth * ratio));
    const height = Math.max(140, Math.floor(canvas.clientHeight * ratio));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
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
    min -= padding;
    max += padding;
    const left = 44 * ratio;
    const right = width - 12 * ratio;
    const top = 15 * ratio;
    const bottom = height - 24 * ratio;
    ctx.strokeStyle = '#35516b';
    ctx.lineWidth = ratio;
    ctx.strokeRect(left, top, right - left, bottom - top);
    ctx.fillStyle = '#9baec8';
    ctx.font = `${10 * ratio}px JetBrains Mono, monospace`;
    ctx.fillText(`${max.toFixed(1)}${unit}`, 4 * ratio, top + 9 * ratio);
    ctx.fillText(`${min.toFixed(1)}${unit}`, 4 * ratio, bottom);
    for (const [key, color] of series) {
      ctx.strokeStyle = color;
      ctx.lineWidth = 2 * ratio;
      ctx.beginPath();
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
      ? 'ARDUSUB · NO LIVE VEHICLE LINK · READ-ONLY PORT'
      : state.demo_mode
        ? 'SOFTWARE DEMO · SYNTHETIC SENSORS · NO VEHICLE CONNECTED'
        : 'ARDUSUB · LIVE TELEMETRY · READ-ONLY PORT';
    set('hb-imuhz', active(state, 'imu_alive') ? number(state.imu_hz, 0) : '—');
    set('hb-thrhz', active(state, 'output_alive') ? 'live' : '—');
    set('dvl-hz', active(state, 'position_alive') ? 'live' : '—');
    $('hb-dot')?.classList.toggle('on', active(state, 'imu_alive'));
    $('thr-dot')?.classList.toggle('on', active(state, 'output_alive'));
    $('dvl-dot')?.classList.toggle('on', active(state, 'position_alive'));
    set('st-txt', !linked ? 'LINK LOST' : state.armed_known ? (state.armed ? 'ARMED' : 'DISARMED') : 'ARM UNKNOWN');
    $('st-dot')?.classList.toggle('on', state.armed_known && state.armed === true);
    set('atal-mode', state.vehicle_mode || '—');
    set('atal-ekf', !state.ekf_status_alive ? 'UNAVAILABLE' : state.ekf_uninitialized ? 'UNINITIALIZED'
      : state.ekf_absolute_position_ok ? 'POSITION READY' : state.ekf_attitude_ok ? 'ATTITUDE ONLY' : 'NOT READY');
    set('atal-pressure', active(state, 'pressure_alive') ? number(state.pressure_mbar, 1, ' mbar') : '—');
    set('atal-water-temp', active(state, 'pressure_alive') ? number(state.water_temp, 1, ' °C') : '—');
    set('atal-position', active(state, 'position_alive')
      ? `${number(state.pos_n, 2, ' m N')} / ${number(state.pos_e, 2, ' m E')}` : '—');
    set('atal-mission', active(state, 'mission_alive') ? `${state.mission_state || '—'} · ${state.mission_seq ?? '—'} / ${state.mission_total ?? '—'}` : '—');
    set('atal-mission-state', active(state, 'mission_alive') ? state.mission_state || '—' : '—');
    set('atal-mission-mode', active(state, 'mission_alive') ? state.mission_mode || '—' : '—');
    set('atal-mission-progress', active(state, 'mission_alive')
      ? `${state.mission_seq ?? '—'} / ${state.mission_total ?? '—'}` : '—');
    set('gate-link-pill', state.gate_link_alive ? (state.gate_link_ok ? 'LINK OK' : 'LINK DOWN') : 'LINK UNKNOWN');
    set('gate-guidance-freshness', state.gate_guidance_alive ? 'GUIDANCE LIVE' : 'NO GUIDANCE');
    set('gate-state-name', state.gate_guidance_alive ? state.gate_state || '—' : 'WAITING');
    set('gate-state-reason', state.gate_guidance_alive ? 'Companion proposal only' : 'No fresh guidance telemetry');
    set('gate-validity-banner', state.gate_proposal_valid ? 'VALID PROPOSAL · NOT COMMANDED' : 'NO VALID PROPOSAL');
    set('gate-detection-status', 'UNAVAILABLE');
    set('gate-confidence', '—');
    set('gate-center-error', '—');
    set('gate-guidance-age', '—');
    for (const axis of ['forward', 'lateral', 'vertical', 'yaw']) {
      set(`gate-${axis}-value`, state.gate_proposal_valid ? number(state[`gate_${axis}`], 2) : '—');
      const bar = $(`gate-${axis}-bar`);
      if (bar) bar.style.width = '0%'; // Limits are not carried in the dashboard snapshot.
    }
    set('v-r', active(state, 'imu_alive') ? number(state.roll, 1, '°') : '—');
    set('v-p', active(state, 'imu_alive') ? number(state.pitch, 1, '°') : '—');
    set('v-y', active(state, 'imu_alive') ? number(state.yaw, 1, '°') : '—');
    set('v-d', active(state, 'position_alive') ? number(state.depth_m, 2, 'm') : '—');
    set('depth-text', active(state, 'position_alive') ? number(state.depth_m, 2, 'm') : '—');
    set('depth-tgt-txt', '—');
    ['ref-r', 'ref-p', 'ref-y', 'ref-d', 'd-r', 'd-p', 'd-y', 'd-d'].forEach((id) => set(id, '—'));
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
    const targetLine = $('depth-target-line');
    if (targetLine) targetLine.style.display = 'none';
    const raw = $('atal-raw');
    if (raw && currentTab === 'raw') raw.textContent = JSON.stringify(state, null, 2);
    if (currentTab === 'sum') {
      drawAttitude(state);
      drawOutputs(state);
    } else if (currentTab === 'plots') {
      drawChart('atal-attitude-chart', [['roll', '#f87171'], ['pitch', '#38bdf8'], ['yaw', '#c084fc']], '°');
      drawChart('atal-depth-chart', [['depth', '#2dd4bf']], 'm');
    }
  }

  const stream = new EventSource('/api/stream');
  stream.onmessage = (event) => {
    try {
      const state = JSON.parse(event.data);
      const now = Date.now();
      if (now - lastSampleAt >= 200) {
        samples.push({ roll: state.imu_alive ? state.roll : null,
          pitch: state.imu_alive ? state.pitch : null,
          yaw: state.imu_alive ? state.yaw : null,
          depth: state.position_alive ? state.depth_m : null });
        if (samples.length > 300) samples.shift();
        lastSampleAt = now;
      }
      render(state);
    } catch (error) { console.error('Invalid ArduSub state', error); }
  };
  stream.onerror = () => {
    render({ ...latest, link_alive: false, armed_known: false, imu_alive: false,
      pressure_alive: false, position_alive: false, output_alive: false,
      mission_alive: false, ekf_status_alive: false, gate_link_alive: false,
      gate_guidance_alive: false, gate_proposal_valid: false });
    banner.firstElementChild.textContent = 'ARDUSUB · TELEMETRY STREAM DISCONNECTED';
  };
})();
