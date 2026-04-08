/* ═══════════════════════════════════════════════════════════════════
   Polling & Real-time Display
   ═══════════════════════════════════════════════════════════════════ */

let pollTimer = null;
let pollIntervalMs = 200;
let polling = false;

function startPolling() {
  stopPolling();
  pollTimer = setInterval(pollState, pollIntervalMs);
  const d = document.getElementById('pollDot');
  if (d) d.className = 'status-dot green';
  const t = document.getElementById('pollText');
if (t) t.textContent = `轮询中: ${pollIntervalMs}ms`;
}

function stopPolling() {
  if (pollTimer) { clearInterval(pollTimer); pollTimer = null; }
  const d = document.getElementById('pollDot');
  if (d) { d.className = 'status-dot red'; d.style.width='6px'; d.style.height='6px'; d.style.animation='none'; }
  const t = document.getElementById('pollText');
if (t) t.textContent = '轮询: 已停止';
}

function updatePollInterval() {
  pollIntervalMs = parseInt(document.getElementById('pollInterval').value) || 200;
  if (pollTimer) startPolling();
}

async function pollState() {
  if (polling) return;
  polling = true;
  try {
    const res = await apiFetch("/api/get_state");
    if (res.success) updateRealtimeDisplay(res.state, res.normalized, res.target);
  } catch(e) {}
  finally { polling = false; }
}

function updateRealtimeDisplay(state, normalized, target) {
  if (!state || !normalized) return;

  // Update real-time gauge readouts
  JOINTS.forEach(j => {
    const norm = normalized[j];
    const key = j + ".pos";
    const deg = state[key];
    const tgt = target ? target[j] : undefined;
    const realEl = document.getElementById(`greal_${j}`);
    if (realEl && norm !== undefined) {
      let text = `实际: ${norm.toFixed(3)}`;
      if (deg !== undefined) text += ` (${deg.toFixed(1)}°)`;
      if (tgt !== undefined) text += ` → 目标: ${tgt.toFixed(3)}`;
      realEl.textContent = text;
    }
  });

  // Update left joystick real marker (X=shoulder_pan, Y=elbow_flex)
  if (joyLeftCtrl && normalized.shoulder_pan !== undefined && normalized.elbow_flex !== undefined) {
    joyLeftCtrl.positionRealMarker(normalized.shoulder_pan, normalized.elbow_flex);
  }
  // Update right joystick real marker (X=wrist_flex, Y=shoulder_lift)
  if (joyRightCtrl && normalized.wrist_flex !== undefined && normalized.shoulder_lift !== undefined) {
    joyRightCtrl.positionRealMarker(normalized.wrist_flex, normalized.shoulder_lift);
  }
  // Update gripper real fill
  if (normalized.gripper !== undefined) {
    const grf = document.getElementById('gripperRealFill');
    if (grf) grf.style.height = (normalized.gripper * 100) + '%';
  }
}
