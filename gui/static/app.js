// uPlot (global, vendored <script>) for 2D time series; three.js (vendored ESM) loaded lazily for 3D.
const $ = id => document.getElementById(id);
function fill(select, values) { select.replaceChildren(...values.map(value => new Option(value, value))); }
async function json(url, options) { const response = await fetch(url, options); if (!response.ok) throw new Error(await response.text()); return response.json(); }
function showTab(name) { document.querySelectorAll('.tab').forEach(tab => tab.classList.toggle('hidden', tab.id !== name)); document.querySelectorAll('header nav button').forEach(button => button.classList.toggle('active', button.dataset.tab === name)); if (name === 'simulator' && !$('live').classList.contains('hidden')) ensureLive(); }
// Scoped to the sub-tab's own tab: a document-wide query hid the other tab's sub-tabs (blank Simulator after visiting Propeller).
function showSubtab(name) { const scope = $(name).closest('.tab'); scope.querySelectorAll('.subtab').forEach(tab => tab.classList.toggle('hidden', tab.id !== name)); scope.querySelectorAll('.subnav button').forEach(button => button.classList.toggle('active', button.dataset.subtab === name)); if (name === 'live') ensureLive(); }
// Canvas (uPlot, three.js) can't read CSS var(); resolve design tokens from styles.css :root.
const tok = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const span = (className, textContent) => Object.assign(document.createElement('span'), { className, textContent });
function readout(label, value) { const o = document.createElement('output'); o.append(span('k', label), span('v', value)); return o; }
const setReadout = (id, value) => { $(id).querySelector('.v').textContent = value; };
async function loadConfig() { $('startup-config').value = JSON.stringify(await json('/api/config/startup'), null, 2); }

// ------------------------------------------------------------ propulsion model
let mmCharts = null, pmCharts = null;

function outCards(container, fields, r) {
  container.replaceChildren(...fields.map(([label, key, unit, dec]) => readout(label, `${Number(r[key]).toFixed(dec)}${unit ? ' ' + unit : ''}`)));
}

async function runMotorModel() {
  const body = {
    kv: +$('mm-kv').value, rm: +$('mm-rm').value, i0: +$('mm-i0').value, k_iron: +$('mm-kiron').value,
    v_batt: +$('mm-vbatt').value, eta_esc: +$('mm-etaesc').value, throttle: +$('mm-throttle').value,
    t_winding_c: +$('mm-twind').value, load_torque_nm: +$('mm-load').value,
  };
  $('mm-error').textContent = ''; $('mm-result').replaceChildren();
  try {
    const r = await json('/api/propulsion/motor', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    outCards($('mm-result'), [
      ['RPM', 'rpm', '', 0], ['Current', 'current_A', 'A', 2], ['Torque', 'torque_Nm', 'Nm', 4],
      ['Shaft power', 'P_shaft_W', 'W', 1], ['Electrical power', 'P_elec_W', 'W', 1], ['Efficiency', 'eta_motor', '', 3],
    ], r);
    if (!mmCharts) mmCharts = {
      rpm: makeChart($('mm-plot-rpm'), [{ label: 'RPM', color: tok('--series-1') }]),
      current: makeChart($('mm-plot-current'), [{ label: 'A', color: tok('--series-1') }]),
      torque: makeChart($('mm-plot-torque'), [{ label: 'Nm', color: tok('--series-1') }]),
      eta: makeChart($('mm-plot-eta'), [{ label: 'eta', color: tok('--series-1') }]),
    };
    const s = r.sweep;
    mmCharts.rpm?.setData([s.throttle, s.rpm]);
    mmCharts.current?.setData([s.throttle, s.current_A]);
    mmCharts.torque?.setData([s.throttle, s.torque_Nm]);
    mmCharts.eta?.setData([s.throttle, s.eta_motor]);
  } catch (error) { $('mm-error').textContent = error.message; }
}

async function runPropellerModel() {
  const body = {
    diameter_in: +$('pm-d').value, pitch_in: +$('pm-p').value, blades: +$('pm-blades').value,
    airfoil: $('pm-airfoil').value.trim() || 'clark_y', rpm: +$('pm-rpm').value, v_climb: +$('pm-vclimb').value,
  };
  $('pm-error').textContent = ''; $('pm-result').replaceChildren();
  try {
    const r = await json('/api/propulsion/propeller', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    outCards($('pm-result'), [
      ['Thrust', 'T_N', 'N', 3], ['Torque', 'Q_Nm', 'Nm', 4], ['Aero power', 'P_aero_W', 'W', 1],
      ['CT', 'CT', '', 5], ['CQ', 'CQ', '', 5], ['Efficiency', 'eta', '', 3],
    ], r);
    $('pm-result').append(readout('Geometry', `${r.source}${r.apc_file ? ' (' + r.apc_file + ')' : ''}`));
    if (!pmCharts) pmCharts = {
      dt: makeChart($('pm-plot-dt'), [{ label: 'dT/dr', color: tok('--series-1') }]),
      dq: makeChart($('pm-plot-dq'), [{ label: 'dQ/dr', color: tok('--series-1') }]),
      aoa: makeChart($('pm-plot-aoa'), [{ label: 'AoA', color: tok('--series-1') }]),
      f: makeChart($('pm-plot-f'), [{ label: 'F', color: tok('--series-1') }]),
    };
    pmCharts.dt?.setData([r.r_norm, r.dT_dr]);
    pmCharts.dq?.setData([r.r_norm, r.dQ_dr]);
    pmCharts.aoa?.setData([r.r_norm, r.alpha_arr_deg]);
    pmCharts.f?.setData([r.r_norm, r.F_arr]);
  } catch (error) { $('pm-error').textContent = error.message; }
}

// ---------------------------------------------------------------- live data
let live = { started: false, charts: null, three: null, buf: null };
const MAXPTS = 600; // ~12s of history at 50Hz telemetry rate

// ---- typed ring buffer (O(1) push, no Array.shift) ----
function mkRing() { return { data: new Float64Array(MAXPTS), head: 0, len: 0, dirty: false, cache: null }; }
function pushRing(ring, v) {
  ring.data[ring.head] = (v == null ? NaN : v);
  ring.head = (ring.head + 1) % MAXPTS;
  if (ring.len < MAXPTS) ring.len++;
  ring.dirty = true;
}
function ringToArray(ring) {
  if (!ring.dirty && ring.cache) return ring.cache;
  const n = ring.len, h = ring.head, buf = ring.data;
  let out;
  if (n < MAXPTS) {
    out = Array.from(buf.subarray(0, n));
  } else {
    out = new Array(MAXPTS);
    for (let i = 0; i < MAXPTS; i++) out[i] = buf[(h + i) % MAXPTS];
  }
  ring.cache = out; ring.dirty = false;
  return out;
}

function makeChart(el, series) {
  if (typeof uPlot === "undefined") {
    el.innerHTML = "<p class='muted small'>uPlot failed to load (vendor/uplot missing).</p>";
    return null;
  }
  const leg = document.createElement("div");
  leg.className = "chart-legend";
  leg.innerHTML = series.map(s => `<span class="leg-item"><i class="leg-swatch" style="background:${s.color}"></i>${s.label}</span>`).join("");
  el.parentNode.insertBefore(leg, el.nextSibling);
  const opts = {
    width: el.clientWidth || 400, height: 180,
    series: [{}, ...series.map(s => ({ label: s.label, stroke: s.color, width: 1.5 }))],
    axes: [0, 1].map(i => ({ stroke: tok("--fg-muted"), grid: { stroke: tok("--line") }, ticks: { stroke: tok("--line") }, font: `12px ${tok("--mono")}`, ...(i ? { size: 64 } : {}) })),
    legend: { show: false },
    scales: { x: { time: false } },
  };
  return new uPlot(opts, [[], ...series.map(() => [])], el);
}

function initCharts() {
  const C = ["--series-1", "--series-2", "--series-3"].map(tok);
  const charts = {
    accel: makeChart($("plot-accel"), [{ label: "x", color: C[0] }, { label: "y", color: C[1] }, { label: "z", color: C[2] }]),
    gyro: makeChart($("plot-gyro"), [{ label: "p", color: C[0] }, { label: "q", color: C[1] }, { label: "r", color: C[2] }]),
    alt: makeChart($("plot-alt"), [{ label: "state", color: C[0] }]),
    wind: makeChart($("plot-wind"), [{ label: "N", color: C[0] }, { label: "E", color: C[1] }, { label: "D", color: C[2] }]),
  };
  if (typeof ResizeObserver !== "undefined") {
    const debounced = (() => { let t; return fn => (...a) => { clearTimeout(t); t = setTimeout(() => fn(...a), 150); }; })();
    const ro = new ResizeObserver(debounced(() => {
      for (const [k, u] of Object.entries(charts)) {
        if (!u) continue;
        const el = $(`plot-${k}`);
        if (el && el.clientWidth > 0) u.setSize({ width: el.clientWidth, height: 180 });
      }
    }));
    ro.observe($("live"));
  }
  return charts;
}

function newBuf() {
  const r3 = () => [mkRing(), mkRing(), mkRing()];
  return { t: mkRing(), accel: r3(), gyro: r3(), alt: [mkRing()], wind: r3() };
}

async function initThree() {
  const el = $("plot-3d");
  let THREE, OrbitControls;
  try {
    THREE = await import("three");
    ({ OrbitControls } = await import("/vendor/three/OrbitControls.js"));
  } catch (e) {
    el.innerHTML = "<p class='muted small'>3D view unavailable (vendor/three failed to load): " + e.message + "</p>";
    return { addPoint() {} };
  }
  const scene = new THREE.Scene(); scene.background = new THREE.Color(tok("--inset"));
  const cam = new THREE.PerspectiveCamera(55, el.clientWidth / el.clientHeight, 0.1, 5000);
  cam.position.set(10, 8, 10);
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(el.clientWidth, el.clientHeight); el.innerHTML = ""; el.appendChild(renderer.domElement);
  const controls = new OrbitControls(cam, renderer.domElement);
  scene.add(new THREE.GridHelper(40, 40, tok("--line-strong"), tok("--line")));
  scene.add(new THREE.AxesHelper(3));
  const geom = new THREE.BufferGeometry();
  const positions = new Float32Array(MAXPTS * 3);
  geom.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geom.setDrawRange(0, 0);
  const line = new THREE.Line(geom, new THREE.LineBasicMaterial({ color: tok("--series-1") }));
  scene.add(line);
  const marker = new THREE.Mesh(new THREE.SphereGeometry(0.25), new THREE.MeshBasicMaterial({ color: tok("--series-2") }));
  scene.add(marker);
  let n = 0, pending = false;
  function addPoint(E, N, Up) {
    if (n >= MAXPTS) { positions.copyWithin(0, 3); n = MAXPTS - 1; }
    positions[n * 3] = E; positions[n * 3 + 1] = Up; positions[n * 3 + 2] = N;
    n++;
    marker.position.set(E, Up, N);
    pending = true;
  }
  (function loop() {
    controls.update();
    if (pending) { geom.setDrawRange(0, n); geom.attributes.position.needsUpdate = true; pending = false; }
    renderer.render(scene, cam);
    requestAnimationFrame(loop);
  })();
  window.addEventListener("resize", () => { renderer.setSize(el.clientWidth, el.clientHeight); cam.aspect = el.clientWidth / el.clientHeight; cam.updateProjectionMatrix(); });
  return { addPoint };
}

async function ensureLive() {
  if (live.started) return;
  live.started = true;
  live.buf = newBuf();
  live.charts = initCharts();
  live.three = await initThree();
}

let rafPending = false;
function scheduleRedraw() {
  if (rafPending || document.hidden) return;
  rafPending = true;
  requestAnimationFrame(() => {
    rafPending = false;
    if (document.hidden) return;
    const b = live.buf, c = live.charts || {};
    const t = ringToArray(b.t);
    c.accel?.setData([t, ...b.accel.map(ringToArray)]);
    c.gyro?.setData([t, ...b.gyro.map(ringToArray)]);
    c.alt?.setData([t, ...b.alt.map(ringToArray)]);
    c.wind?.setData([t, ...b.wind.map(ringToArray)]);
  });
}

function onTelemetry(d) {
  const b = live.buf;
  pushRing(b.t, d.t);
  const v3 = a => Array.isArray(a) ? a : [null, null, null];
  const ac = v3(d.accel), gy = v3(d.gyro), wd = v3(d.wind_ned);
  for (let i = 0; i < 3; i++) { pushRing(b.accel[i], ac[i]); pushRing(b.gyro[i], gy[i]); pushRing(b.wind[i], wd[i]); }
  pushRing(b.alt[0], d.state_alt);
  if (live.three && Array.isArray(d.pos_ned)) live.three.addPoint(d.pos_ned[1], d.pos_ned[0], -d.pos_ned[2]);
  setReadout('altitude', `${Number(d.state_alt).toFixed(2)} m`);
  setReadout('wind', wd.map(Number).map(v => v.toFixed(2)).join(', '));
  setReadout('accel', ac.map(Number).map(v => v.toFixed(2)).join(', '));
  setReadout('gyro', gy.map(Number).map(v => v.toFixed(3)).join(', '));
  scheduleRedraw();
}

function withWipeOverride(cfg) { cfg.sitl = { ...(cfg.sitl || {}), preserve_eeprom: !$('wipe-eeprom').checked }; return cfg; }

async function boot() {
  document.querySelectorAll('header nav button').forEach(button => button.onclick = () => showTab(button.dataset.tab));
  document.querySelectorAll('.subnav button').forEach(button => button.onclick = () => showSubtab(button.dataset.subtab));
  const options = await json('/api/options'); fill($('param-file'), options.param_files); fill($('motor'), options.motors); fill($('battery'), options.batteries); await loadConfig();
  $('wipe-eeprom').checked = !(JSON.parse($('startup-config').value).sitl?.preserve_eeprom ?? true);
  $('wipe-eeprom').onchange = () => { $('startup-config').value = JSON.stringify(withWipeOverride(JSON.parse($('startup-config').value)), null, 2); };
  const saved = await json('/api/status'); $('ardupilot-dir').value = saved.ardupilot_dir || ''; $('venv-activate').value = saved.venv_activate || '';
  $('detect-ip').onclick = async () => { $('host-ip').value = (await json('/api/detect-ip')).host_ip; };
  $('launch').onclick = async () => {
    const config = withWipeOverride(JSON.parse($('startup-config').value));
    try { $('status').textContent = JSON.stringify(await json('/api/launch', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({param_file:$('param-file').value,ip:$('host-ip').value,ardupilot_dir:$('ardupilot-dir').value,venv_activate:$('venv-activate').value,config})})); }
    catch (err) { $('status').textContent = `Error: ${err.message}`; }
  };
  $('stop').onclick = async () => { $('status').textContent = JSON.stringify(await json('/api/stop', {method:'POST'})); updateStatus({}); };
  $('save-config').onclick = async () => { await json('/api/config/startup', {method:'PUT',headers:{'Content-Type':'application/json'},body:JSON.stringify({data:withWipeOverride(JSON.parse($('startup-config').value))})}); };
  $('calculate').onclick = async () => {
    const body = {motor_name:$('motor').value,battery_name:$('battery').value,num_motors:+$('motor-count').value,num_batteries:+$('battery-count').value,structural_mass:+$('structural-mass').value,payload_mass:+$('payload-mass').value,max_empty_weight:+$('max-empty-weight').value,required_twr:+$('required-twr').value,flight_twr:+$('flight-twr').value,bank_angle:+$('bank-angle').value,avionics_power:+$('avionics-power').value};
    $('flight-error').textContent = ''; $('flight-result').replaceChildren();
    try {
      const r = await json('/api/flight-time', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
      const fields = [['Flight time','flight_time_min','min',2],['MTOW','MTOW_kg','kg',3],['Empty weight','empty_weight_kg','kg',3],['Payload ratio','payload_ratio','',3],['Total power','total_power_w','W',1],['Battery energy','total_battery_energy_wh','Wh',1],['Depth of discharge','depth_of_discharge','',2],['Motor throttle','motor_throttle','',2]];
      outCards($('flight-result'), fields, r);
    } catch (error) { $('flight-error').textContent = error.message; }
  };
  $('btn-live-clear').onclick = () => { live.buf = newBuf(); };
  $('mm-run').onclick = runMotorModel;
  $('pm-run').onclick = runPropellerModel;
  connectWS();
}

// ---- persistent top-panel dashboard (status messages, ~every 10s from sim) ----
function healthClass(el, level) { el.classList.remove('ok', 'warn', 'bad'); if (level) el.classList.add(level); }
function band(v, okMin, warnMin) { return v == null ? null : v >= okMin ? 'ok' : v >= warnMin ? 'warn' : 'bad'; }
function setStat(id, text, level) { const el = $(id); el.textContent = text; healthClass(el, level); }
function updateStatus(d) {
  setStat('st-rt', d.realtime_pct != null ? d.realtime_pct.toFixed(0) + '%' : '—', band(d.realtime_pct, 90, 70));
  setStat('st-cpu', d.cpu_pct != null ? d.cpu_pct.toFixed(0) + '%' : 'n/a', d.cpu_pct != null ? band(100 - d.cpu_pct, 50, 25) : null);
  setStat('st-fps', d.fps != null ? d.fps.toFixed(0) : '—', null);
  setStat('st-t', d.t != null ? d.t.toFixed(2) + 's' : '—', null);
  setStat('st-conn', d.connected ? 'yes' : 'no', d.connected ? 'ok' : 'bad');
}

function connectWS() {
  const socket = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws/telemetry`);
  socket.onopen = () => { $('live-status').textContent = 'connected'; };
  socket.onclose = () => { $('live-status').textContent = 'disconnected — retrying'; setStat('st-conn', 'no', 'bad'); setTimeout(connectWS, 1500); };
  socket.onmessage = event => {
    const data = JSON.parse(event.data);
    if (data.type === 'status') { updateStatus(data); return; }
    if (data.type !== 'telemetry') return;
    if (live.started) onTelemetry(data);
  };
}

boot().catch(error => { $('status').textContent = error.message; });
