// Read-only ArduSub telemetry on the original ATAL dashboard shell.
const $ = id => document.getElementById(id);
const number = (value, digits = 1) => Number.isFinite(value) ? value.toFixed(digits) : '—';
const history = [];
let hadLink = false;
let lastSnapshot = null;
let selectedTab = 'dashboard';
let recordingActive = false;
let recordingBusy = false;

function showRecording(state) {
  recordingActive = !!state.active;
  $('recording-toggle').textContent = recordingActive ? 'Stop recording' : 'Start recording';
  $('recording-toggle').disabled = recordingBusy;
  $('recording-download').hidden = !state.file;
  $('recording-status').textContent = recordingActive ? `Recording ${state.file} · ${state.rows} rows` :
    state.file ? `Saved ${state.file} · ${state.rows} rows` : 'No recording active';
}

async function refreshRecording() {
  if (recordingBusy) return;
  try {
    const response = await fetch('/api/recording', { cache: 'no-store' });
    if (!response.ok) throw new Error('Recorder unavailable');
    showRecording(await response.json());
  } catch (_) {
    $('recording-status').textContent = 'Recorder unavailable';
  }
}

async function toggleRecording() {
  if (recordingBusy) return;
  recordingBusy = true;
  $('recording-toggle').disabled = true;
  try {
    const path = recordingActive ? '/api/recording/stop' : '/api/recording/start';
    const response = await fetch(path, { method: 'POST', headers: { 'X-ATAL-Local-Action': 'recording' } });
    if (!response.ok) throw new Error('Recording request failed');
    showRecording(await response.json());
  } catch (_) {
    $('recording-status').textContent = 'Recording request failed';
  } finally {
    recordingBusy = false;
    $('recording-toggle').disabled = false;
  }
}

refreshRecording();
setInterval(refreshRecording, 1000);
$('recording-toggle').addEventListener('click', toggleRecording);

for (let i = 1; i <= 8; i++) {
  const motor = document.createElement('div');
  motor.className = 'motor';
  motor.innerHTML = `<span>CH ${i}</span><strong>—</strong><i></i>`;
  $('motor-grid').append(motor);
  const marker = document.createElement('span');
  marker.textContent = `${i}  —`;
  $('thruster-schematic').append(marker);
}

function status(id, text, tone) {
  const element = $(id);
  element.textContent = text;
  element.className = tone;
}

function render(s, requestFailed = false) {
  lastSnapshot = s;
  if (s.link_alive) hadLink = true;
  const linkText = requestFailed ? 'Connection error' : s.link_alive ? 'Live' : hadLink ? 'Lost' : 'Waiting';
  status('link-status', linkText, s.link_alive ? 'good' : hadLink || requestFailed ? 'danger' : 'warn');
  status('arming-status', !s.armed_known ? 'Unknown' : s.armed ? 'Armed' : 'Disarmed', !s.armed_known ? 'warn' : s.armed ? 'danger' : 'good');
  status('mode-status', s.link_alive && s.vehicle_mode ? s.vehicle_mode : 'Unavailable', s.link_alive && s.vehicle_mode ? 'good' : 'warn');
  for (const [name, alive] of [['imu', s.imu_alive], ['pressure', s.pressure_alive], ['position', s.position_alive], ['motor', s.output_alive]]) {
    status(`${name}-status`, alive ? 'Fresh' : 'Unavailable', alive ? 'good' : 'warn');
  }
  $('state-text').textContent = !s.armed_known ? 'UNKNOWN' : s.armed ? 'ARMED' : 'DISARMED';
  $('state-dot').className = `dot ${!s.armed_known ? 'warn' : s.armed ? 'err' : 'on'}`;
  $('imu-dot').className = `dot ${s.imu_alive ? 'on' : 'warn'}`;
  $('output-dot').className = `dot ${s.output_alive ? 'on' : 'warn'}`;
  $('imu-rate').textContent = s.imu_alive ? number(s.imu_hz, 0) : '—';
  const banner = $('conn-banner');
  banner.className = s.link_alive ? 'live' : hadLink || requestFailed ? 'lost' : '';
  banner.textContent = requestFailed ? 'Dashboard cannot reach telemetry API' : s.link_alive ? 'ArduSub telemetry connected · read-only display' : hadLink ? 'ArduSub telemetry lost · values shown only when fresh' : 'Waiting for ArduSub telemetry';

  for (const key of ['roll', 'pitch', 'yaw']) $(key).textContent = s.imu_alive ? number(s[key]) : '—';
  const horizon = $('horizon');
  horizon.classList.toggle('live', !!s.imu_alive);
  horizon.style.setProperty('--roll', s.imu_alive ? `${-s.roll}deg` : '0deg');
  horizon.style.setProperty('--pitch', s.imu_alive ? `${Math.max(-45, Math.min(45, s.pitch)) * 2}px` : '0px');
  horizon.setAttribute('aria-label', s.imu_alive ? `Roll ${number(s.roll)} degrees, pitch ${number(s.pitch)} degrees` : 'Attitude unavailable');
  $('sample-age').textContent = s.imu_alive ? `Attitude sample ${number(s.imu_age, 1)} s old` : 'Attitude data unavailable';
  const missionFresh = !!s.mission_alive;
  const missionState = missionFresh ? s.mission_state : null;
  status('mission-state', missionState || 'Unavailable', missionState === 'Active' ? 'good' : 'warn');
  $('mission-item').textContent = missionFresh && Number.isInteger(s.mission_seq) ? String(s.mission_seq) : '—';
  $('mission-total').textContent = missionFresh && Number.isInteger(s.mission_total) ? String(s.mission_total) : '—';
  $('mission-mode').textContent = missionFresh ? s.mission_mode || '—' : '—';
  const depth = s.position_alive ? s.depth_m : null;
  $('depth-reading').textContent = $('depth-small').textContent = number(depth, 2);
  const displayRange = Number(document.querySelector('.depth-track').dataset.displayRangeM);
  const fill = Number.isFinite(depth) && displayRange > 0 ? `${Math.max(0, Math.min(100, depth / displayRange * 100))}%` : '0%';
  $('depth-fill').style.setProperty('--fill', fill);
  $('depth-cursor').style.setProperty('--fill', fill);
  $('pressure').textContent = s.pressure_alive ? number(s.pressure_mbar, 1) : '—';
  $('temperature').textContent = s.pressure_alive ? number(s.water_temp, 1) : '—';
  for (const [id, key] of [['vel-fwd', 'vel_fwd'], ['vel-lat', 'vel_lat']]) $(id).textContent = s.position_alive && s.imu_alive ? number(s[key], 2) : '—';
  for (const [id, key] of [['pos-n', 'pos_n'], ['pos-e', 'pos_e'], ['vel-z', 'vel_z']]) $(id).textContent = s.position_alive ? number(s[key], 2) : '—';

  [...$('motor-grid').children].forEach((motor, index) => {
    const pwm = s.output_alive && Array.isArray(s.pwm) ? s.pwm[index] : null;
    motor.querySelector('strong').textContent = Number.isFinite(pwm) ? String(pwm) : '—';
    motor.style.setProperty('--level', Number.isFinite(pwm) ? `${Math.max(0, Math.min(100, (pwm - 1100) / 8))}%` : '0%');
    const marker = $('thruster-schematic').children[index];
    marker.textContent = `${index + 1}  ${Number.isFinite(pwm) ? pwm : '—'}`;
    marker.classList.toggle('live', Number.isFinite(pwm));
  });
  $('raw-data').textContent = JSON.stringify(s, null, 2);
  history.push({ time: Date.now(), roll: s.imu_alive ? s.roll : null, pitch: s.imu_alive ? s.pitch : null, yaw: s.imu_alive ? s.yaw : null, depth });
  const cutoff = Date.now() - 60_000;
  while (history.length && history[0].time < cutoff) history.shift();
  if (selectedTab === 'plots') drawPlots();
}

function drawPlot(id, series) {
  const canvas = $(id);
  const width = Math.max(1, Math.floor(canvas.clientWidth));
  const height = 220;
  const ratio = window.devicePixelRatio || 1;
  canvas.width = width * ratio;
  canvas.height = height * ratio;
  const ctx = canvas.getContext('2d');
  ctx.scale(ratio, ratio);
  const colors = getComputedStyle(document.documentElement);
  const grid = colors.getPropertyValue('--bd2').trim();
  const text = colors.getPropertyValue('--tx3').trim();
  const plot = { left: 42, right: width - 12, top: 18, bottom: height - 30 };
  ctx.strokeStyle = grid;
  ctx.fillStyle = text;
  ctx.font = '10px ui-monospace, monospace';
  const values = history.flatMap(point => series.map(item => point[item.key]).filter(Number.isFinite));
  if (!values.length) {
    ctx.textAlign = 'center';
    ctx.fillText('No fresh samples yet', width / 2, height / 2);
    return;
  }
  const lo = Math.min(...values);
  const hi = Math.max(...values);
  const pad = Math.max(0.5, (hi - lo) * .15);
  for (let row = 0; row < 3; row++) {
    const y = plot.top + row * (plot.bottom - plot.top) / 2;
    ctx.beginPath(); ctx.moveTo(plot.left, y); ctx.lineTo(plot.right, y); ctx.stroke();
    ctx.fillText(number(hi + pad - row * (hi - lo + 2 * pad) / 2, 1), 2, y + 3);
  }
  const start = Date.now() - 60_000;
  for (const item of series) {
    ctx.strokeStyle = item.color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    let drawing = false;
    for (const point of history) {
      const value = point[item.key];
      if (!Number.isFinite(value)) { drawing = false; continue; }
      const x = plot.left + (point.time - start) / 60_000 * (plot.right - plot.left);
      const y = plot.bottom - (value - lo + pad) / (hi - lo + 2 * pad) * (plot.bottom - plot.top);
      if (drawing) ctx.lineTo(x, y); else ctx.moveTo(x, y);
      drawing = true;
    }
    ctx.stroke();
  }
  ctx.fillStyle = text;
  ctx.fillText('60 s ago', plot.left, height - 8);
  ctx.fillText('now', plot.right - 22, height - 8);
}

function drawPlots() {
  drawPlot('attitude-plot', [{ key: 'roll', color: '#f87171' }, { key: 'pitch', color: '#38bdf8' }, { key: 'yaw', color: '#a78bfa' }]);
  drawPlot('depth-plot', [{ key: 'depth', color: '#2dd4bf' }]);
}

function selectTab(tab) {
  selectedTab = tab;
  document.querySelectorAll('.nav button[data-tab]').forEach(item => {
    const active = item.dataset.tab === tab;
    item.classList.toggle('on', active);
    if (active) item.setAttribute('aria-current', 'page'); else item.removeAttribute('aria-current');
  });
  document.querySelectorAll('.view').forEach(view => {
    const show = view.id === `p-${selectedTab}`;
    view.hidden = !show;
    view.classList.toggle('show', show);
  });
  if (selectedTab === 'plots') drawPlots();
}

const themeButton = $('theme-toggle');
function applyTheme(light) {
  document.documentElement.classList.toggle('light', light);
  themeButton.textContent = light ? '🌙' : '☀️';
  themeButton.setAttribute('aria-label', light ? 'Switch to dark theme' : 'Switch to light theme');
  if (selectedTab === 'plots') drawPlots();
}
applyTheme(localStorage.getItem('atal-theme') === 'light');
$('theme-toggle').addEventListener('click', toggleTheme);
document.querySelectorAll('.nav button[data-tab]').forEach(button => {
  button.addEventListener('click', () => selectTab(button.dataset.tab));
});
function toggleTheme() {
  const light = !document.documentElement.classList.contains('light');
  localStorage.setItem('atal-theme', light ? 'light' : 'dark');
  applyTheme(light);
}
window.addEventListener('resize', () => { if (selectedTab === 'plots') drawPlots(); });

async function refresh() {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 2000);
  try {
    const response = await fetch('/api/state', { signal: controller.signal, cache: 'no-store' });
    if (!response.ok) throw new Error('Telemetry request failed');
    render(await response.json());
  } catch (_) {
    render({ link_alive: false, imu_alive: false, pressure_alive: false, position_alive: false, output_alive: false, armed_known: false, pwm: [] }, true);
  } finally {
    clearTimeout(timeout);
    setTimeout(refresh, 500);
  }
}
refresh();
