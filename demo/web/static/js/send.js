/* ═══════════════════════════════════════════════════════════════════
   Gauge Display & Velocity Send Logic
   
   The server runs its own control loop at 50Hz. The frontend only
   sends the current joystick deflection ("velocity") when it changes.
   The server continuously applies the velocity to the target position.
   ═══════════════════════════════════════════════════════════════════ */

function updateGaugeDisplay(joint) {
  const el = document.getElementById(`gval_${joint}`);
  if (!el) return;
  if (joint === 'gripper') {
    el.textContent = gripperTarget.toFixed(2);
  } else {
    // Show delta value with sign
    const d = deltaValues[joint];
    el.textContent = (d >= 0 ? '+' : '') + d.toFixed(3);
  }
}

/* ── Send velocity to server (only on change) ── */
let _lastSentVelocity = {};
let _velocitySendPending = false;

function onDeltaChanged() {
  if (!isController) return;
  // Debounce: batch rapid changes into a single request
  if (!_velocitySendPending) {
    _velocitySendPending = true;
    requestAnimationFrame(() => {
      _velocitySendPending = false;
      _sendVelocityIfChanged();
    });
  }
}

function _sendVelocityIfChanged() {
  // Build velocity object from current deltaValues (exclude gripper)
  const vel = {};
  let changed = false;
  for (const j of JOINTS) {
    if (j === 'gripper') continue;
    const v = deltaValues[j];
    vel[j] = v;
    if (Math.abs((v || 0) - (_lastSentVelocity[j] || 0)) > 0.001) {
      changed = true;
    }
  }

  if (!changed) return;

  // Update last-sent record
  _lastSentVelocity = { ...vel };

  // Fire-and-forget: send velocity to server
  apiFetch("/api/set_velocity", {
    method: "POST",
    body: JSON.stringify({ velocity: vel }),
  }).then(res => {
    if (!res.success) {
addLog(`设置速度失败: ${res.message}`, "log-err");
    }
  });
}

// No-op stubs for compatibility (controls.js calls these)
function startContinuousSend() {}
function stopContinuousSend() {
  // When stopping, send zero velocity to halt all movement
  const vel = {};
  for (const j of JOINTS) {
    if (j === 'gripper') continue;
    vel[j] = 0;
  }
  _lastSentVelocity = { ...vel };
  apiFetch("/api/set_velocity", {
    method: "POST",
    body: JSON.stringify({ velocity: vel }),
  });
}

/* ── Gripper absolute send ── */
let gripperSendTimer = null;

function scheduleGripperSend() {
  if (!isController) return;
  clearTimeout(gripperSendTimer);
  gripperSendTimer = setTimeout(() => { sendGripperValue(); }, 80);
}

async function sendGripperValue() {
  if (!isController) return;
  const res = await apiFetch("/api/set_gripper", {
    method: "POST",
    body: JSON.stringify({ value: gripperTarget }),
  });
  if (res.success) {
addLog("夹爪设置成功 ✅", "log-ok");
  } else {
addLog(`夹爪设置失败: ${res.message}`, "log-err");
  }
}
