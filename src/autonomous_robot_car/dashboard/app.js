const $ = (id) => document.getElementById(id);
let connected = false;
let payloadLoaded = true;
let toastTimer;

function toast(text, isError=false) {
  const node = $('toast');
  node.textContent = text;
  node.style.color = isError ? 'var(--red)' : 'var(--cyan)';
  node.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => node.classList.remove('show'), 2200);
}

async function command(value) {
  try {
    const response = await fetch('/api/command', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({command: value}),
    });
    if (!response.ok) throw new Error(await response.text());
    toast(`Command sent: ${value.toUpperCase()}`);
  } catch (error) {
    toast('Command failed — dashboard disconnected', true);
  }
}

function formatTime(totalSeconds) {
  const hours = Math.floor(totalSeconds / 3600);
  const minutes = Math.floor((totalSeconds % 3600) / 60);
  const seconds = Math.floor(totalSeconds % 60);
  return hours ? `${hours}:${String(minutes).padStart(2,'0')}:${String(seconds).padStart(2,'0')}` : `${String(minutes).padStart(2,'0')}:${String(seconds).padStart(2,'0')}`;
}

function update(data) {
  connected = true;
  $('connection').className = 'connection online';
  $('connection').innerHTML = '<span></span> ROS CONNECTED';
  const state = data.state || 'UNKNOWN';
  $('stateBadge').textContent = state;
  $('stateBadge').className = `badge ${state === 'CAMERA_LOST' ? 'danger' : state === 'IDLE' || state === 'PAUSED' ? 'idle' : ''}`;
  $('missionDetail').textContent = data.detail || 'No status detail.';

  const cameraOkay = Boolean(data.camera_ok) && (data.camera_stream_age === null || data.camera_stream_age < 1.5);
  $('cameraBadge').textContent = cameraOkay ? 'LIVE' : 'NO SIGNAL';
  $('cameraBadge').className = `badge ${cameraOkay ? '' : 'warning'}`;
  $('cameraSafety').textContent = cameraOkay ? 'NOMINAL' : 'STALE';
  $('cameraSafety').style.color = cameraOkay ? 'var(--green)' : 'var(--red)';
  $('cameraAge').textContent = data.camera_stream_age === null ? 'NO SIGNAL' : `${Number(data.camera_stream_age).toFixed(1)}s AGO`;

  $('distance').textContent = data.front_distance == null ? '—' : Number(data.front_distance).toFixed(2);
  const odom = data.odometry || {};
  $('speed').textContent = Number(odom.speed || 0).toFixed(2);
  $('pose').textContent = `${Number(odom.x || 0).toFixed(1)}, ${Number(odom.y || 0).toFixed(1)}`;
  $('heading').textContent = `heading ${Number(odom.yaw || 0).toFixed(0)}°`;
  $('target').textContent = data.target_visible ? 'LOCKED' : 'SEARCH';
  $('target').style.color = data.target_visible ? 'var(--green)' : 'var(--ink)';
  $('targetDistance').textContent = data.target_distance == null ? `destination ${data.destination || 'NONE'}` : `${Number(data.target_distance).toFixed(2)} metres`;
  $('targetCue').textContent = data.target_visible ? `TARGET ${data.destination} LOCK` : `TARGET ${data.destination || '—'}`;
  $('driveCommand').textContent = `${Number(data.commanded_linear || 0).toFixed(2)} / ${Number(data.commanded_angular || 0).toFixed(2)}`;
  const plannerActive = state === 'PATH_FOLLOWING' || state === 'PATH_BLOCKED';
  $('depthSafety').textContent = plannerActive ? 'INTERVENING' : 'ACTIVE';
  $('depthSafety').style.color = plannerActive ? 'var(--amber)' : 'var(--green)';
  $('plannerStatus').textContent = data.planned_heading_degrees == null
    ? 'STANDBY'
    : `${Number(data.planned_heading_degrees).toFixed(0)}° / ${Number(data.planned_gap_width || 0).toFixed(2)} m`;
  $('plannerStatus').style.color = data.planned_heading_degrees == null ? 'var(--muted)' : 'var(--cyan)';

  payloadLoaded = Boolean(data.payload_loaded);
  $('payloadValue').innerHTML = payloadLoaded ? '100 g<br><small>LOADED</small>' : '0 g<br><small>EMPTY</small>';
  $('payloadText').textContent = payloadLoaded ? 'Reference payload is loaded and secured.' : 'Delivery tray is empty.';
  $('payloadButton').style.color = payloadLoaded ? 'var(--amber)' : 'var(--muted)';
  $('deliveredButton').disabled = !data.arrived;
  $('uptime').textContent = `UP ${formatTime(data.dashboard_uptime || 0)}`;
}

async function pollStatus() {
  try {
    const response = await fetch(`/api/status?t=${Date.now()}`, {cache: 'no-store'});
    if (!response.ok) throw new Error('status unavailable');
    update(await response.json());
  } catch (error) {
    if (connected) toast('Dashboard connection lost', true);
    connected = false;
    $('connection').className = 'connection offline';
    $('connection').innerHTML = '<span></span> OFFLINE';
  }
}

function refreshCamera() {
  const preload = new Image();
  preload.onload = () => { $('camera').src = preload.src; };
  preload.src = `/camera.jpg?t=${Date.now()}`;
}

$('startButton').onclick = () => command(`start:${$('destination').value}`);
$('pauseButton').onclick = () => command('pause');
$('resumeButton').onclick = () => command('resume');
$('stopButton').onclick = () => command('stop');
$('deliveredButton').onclick = () => command('delivered');
$('payloadButton').onclick = () => command(payloadLoaded ? 'payload:empty' : 'payload:loaded');

pollStatus();
refreshCamera();
setInterval(pollStatus, 400);
setInterval(refreshCamera, 250);
