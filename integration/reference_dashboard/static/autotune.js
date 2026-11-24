/* Dashboard controls for Release A; ROS supervisor owns experiment decisions. */
(() => {
  const $ = id => document.getElementById('at-' + id);
  const fields = ['amplitude_rad_s', 'frequencies_hz', 'cycles', 'ramp_sec', 'settle_sec',
    'recover_sec', 'max_duration_sec', 'min_depth_m', 'max_depth_m', 'max_depth_excursion_m'];
  const activeStages = new Set(['SNAPSHOT', 'SETTLE', 'EXCITE', 'RECOVER']);
  let state = {}, busy = false, loaded = false, dirty = false, polling = false, lastRuns = 0;
  const points = [];
  function feedback(message, error = false) {
    $('feedback').textContent = message;
    $('feedback').style.color = error ? 'var(--rd)' : 'var(--tx)';
  }
  async function api(path, body) {
    const r = await fetch('/api/autotune/' + path, body === undefined ? {} : {
      method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)
    });
    const data = await r.json();
    if (!r.ok || !data.ok) throw new Error(data.error || data.message || 'Request failed');
    return data;
  }
  function settings() {
    const result = {};
    fields.forEach(key => {
      const text = $(key).value.trim();
      if (!text) throw new Error('Every experiment setting is required');
      result[key] = key === 'frequencies_hz' ? text.split(',').map(v => {
        if (!v.trim()) throw new Error('Frequency list contains an empty value');
        return Number(v.trim());
      }) : Number(text);
      if (Array.isArray(result[key]) ? result[key].some(v => !Number.isFinite(v)) : !Number.isFinite(result[key]))
        throw new Error('Settings must contain finite numbers');
    });
    return result;
  }
  function loadSettings() {
    const saved = state.supervisor?.settings;
    if (!saved) return;
    fields.forEach(key => { if (saved[key] !== undefined) $(key).value = Array.isArray(saved[key]) ? saved[key].join(', ') : saved[key]; });
    loaded = true; dirty = false;
  }
  function render() {
    const s = state.supervisor || {}, c = state.controller || {};
    const online = state.supervisor_online, active = online && activeStages.has(s.stage);
    $('stage').textContent = online ? s.stage || 'WAITING' : 'OFFLINE';
    $('run-id').textContent = s.run_id || 'No run selected';
    $('reason').textContent = online ? (s.precheck_reason || s.reason || 'Waiting for checks') : (state.launch_error || 'Prepare the supervisor and wait for live status.');
    $('progress').value = s.stage === 'COMPLETE' ? 100 : Math.min(100, 100 * (s.elapsed_sec || 0) / (s.duration_sec || 1));
    const show = (id, value, unit) => { $(id).textContent = state.controller_online && Number.isFinite(value) ? value.toFixed(3) + ' ' + unit : '—'; };
    show('requested', c.requested_rad_s, 'rad/s'); show('applied', c.applied_rad_s, 'rad/s'); show('depth', c.depth_m, 'm');
    const checks = [
      [state.interfaces_available, 'Autotune interfaces available'], [online, 'Supervisor status live'],
      [state.controller_online, 'Controller telemetry live'], [c.armed, 'Vehicle armed through normal controls'],
      [c.ready && state.controller_online, 'Controller ready / opt-in enabled'],
      [c.depth_fresh, 'Depth readings fresh'], [!c.manual_test_active && state.controller_online, 'Manual PWM test inactive'],
      [!!s.settings?.operator_ready, 'Operator opt-in enabled']
    ];
    $('checks').replaceChildren(...checks.map(([ok, label]) => { const li = document.createElement('li'); li.textContent = (ok ? '✓ ' : '○ ') + label; return li; }));
    $('prepare').disabled = busy || !!online || !state.interfaces_available;
    $('enable').textContent = s.settings?.operator_ready ? 'Disable experiments' : 'Enable experiments';
    $('enable').disabled = busy || !online || active || (!s.settings?.operator_ready && !$('ack').checked);
    $('save').disabled = busy || !online || active;
    $('settings').disabled = busy || !!active;
    $('start').disabled = busy || !online || active || dirty || !state.controller_online || !c.ready || !s.settings?.operator_ready || !!s.precheck_reason;
    // Cancellation stays independent of configuration/start requests and stale status.
    $('cancel').disabled = false;
    if (!loaded && !dirty) loadSettings();
  }
  function chart() {
    const cv = $('chart'), ctx = cv.getContext('2d');
    if (!ctx) return;
    cv.width = Math.max(300, cv.clientWidth); cv.height = 110;
    ctx.clearRect(0, 0, cv.width, cv.height);
    ctx.strokeStyle = '#64748b'; ctx.beginPath(); ctx.moveTo(0,55); ctx.lineTo(cv.width,55); ctx.stroke();
    ['#38bdf8', '#34d399'].forEach((color, j) => {
      ctx.strokeStyle = color; ctx.beginPath();
      points.forEach((p, i) => { const x = i / 119 * cv.width, y = 55 - Math.max(-0.08,Math.min(.08,p[j])) / .08 * 45; i ? ctx.lineTo(x,y) : ctx.moveTo(x,y); }); ctx.stroke();
    });
  }
  async function runs() {
    const data = await api('runs');
    $('runs').replaceChildren();
    if (!data.runs.length) { const row = $('runs').insertRow(); row.insertCell().textContent = 'No recordings yet.'; return; }
    data.runs.forEach(run => {
      const row = $('runs').insertRow();
      const date = new Date(run.started_utc);
      row.insertCell().textContent = Number.isNaN(date.getTime()) ? 'Unknown' : date.toLocaleString();
      row.insertCell().textContent = run.stage || 'INCOMPLETE';
      row.insertCell().textContent = run.reason || 'Recording incomplete';
      const cell = row.insertCell();
      if (['COMPLETE','CANCELLED','ABORTED'].includes(run.stage) && /^[a-f0-9]{32}$/.test(run.run_id))
        ['metadata.json','samples.jsonl'].forEach(file => { const a = document.createElement('a'); a.href = '/api/autotune/runs/' + run.run_id + '/' + file; a.textContent = file; a.style.marginRight = '10px'; cell.append(a); });
    });
  }
  async function poll() {
    if (polling) return;
    polling = true;
    try {
      state = await api('state'); render();
      const c = state.controller;
      if (state.controller_online && Number.isFinite(c?.requested_rad_s) && Number.isFinite(c?.applied_rad_s)) {
        points.push([c.requested_rad_s,c.applied_rad_s]); if (points.length > 120) points.shift();
      }
      chart();
      if (Date.now() - lastRuns > 5000) { await runs(); lastRuns = Date.now(); }
    } catch (e) { state = {}; render(); feedback(e.message, true); }
    finally { polling = false; }
  }
  async function action(path, body = {}) {
    busy = true; render();
    try { const data = await api(path, body); feedback(data.message || 'Done'); if (path === 'settings') { dirty = false; loaded = true; } }
    catch (e) { feedback(e.message, true); }
    finally { busy = false; await poll(); render(); }
  }
  fields.forEach(key => $(key).addEventListener('input', () => { dirty = true; render(); }));
  $('ack').addEventListener('change', render);
  $('prepare').onclick = () => action('prepare');
  $('enable').onclick = () => action('enable', {enabled: !state.supervisor?.settings?.operator_ready, acknowledged: $('ack').checked});
  $('save').onclick = () => { try { action('settings', settings()); } catch(e) { feedback(e.message, true); } };
  $('reload').onclick = () => { loadSettings(); render(); };
  $('start').onclick = () => action('start');
  $('cancel').onclick = async () => { try { const data = await api('cancel', {}); feedback(data.message); } catch(e) { feedback(e.message,true); } await poll(); };
  setInterval(poll, 500); poll();
})();
