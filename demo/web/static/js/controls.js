/* ═══════════════════════════════════════════════════════════════════
   Control Acquire / Release / UI / Lock Check
   ═══════════════════════════════════════════════════════════════════ */

async function acquireControl() {
  const res = await apiFetch("/api/acquire", { method: "POST" });
  if (res.success) {
    isController = true;
    isLockedByOther = false;
addLog("已获取控制权 ✅", "log-ok");
    updateUI();
    initControls();
    // Re-center joystick knobs after layout is fully computed (panel just became visible)
    requestAnimationFrame(() => requestAnimationFrame(() => {
      if (joyLeftCtrl && joyLeftCtrl.centerKnob) joyLeftCtrl.centerKnob();
      if (joyRightCtrl && joyRightCtrl.centerKnob) joyRightCtrl.centerKnob();
    }));
    startPolling();
    enterFullscreenLandscape();
  } else {
    isLockedByOther = true;
addLog(`获取控制失败: ${res.message}`, "log-err");
    updateUI();
    startLockCheck();
  }
}

async function releaseControl() {
addLog("正在释放控制 — 恢复机械臂至保存位置...", "log-info");
  stopContinuousSend();
  stopPolling();

  // Tell server to release (which will also home the arm and disconnect)
  const res = await apiFetch("/api/release", { method: "POST" });
  if (res.success) {
    isController = false;
addLog("已释放控制，机械臂已恢复 ✅", "log-ok");
    exitFullscreen();
  } else {
addLog(`释放失败: ${res.message}`, "log-err");
  }
  updateUI();
}

/* ── Stand Up ── */
async function doStandUp() {
  if (sending) return;
  sending = true;
  updateUI();
  const durationMs = parseInt(document.getElementById("durationMs").value) || 2000;
addLog(`站立复位中 (${durationMs}ms)...`);
  const res = await apiFetch("/api/stand_up", {
    method: "POST",
    body: JSON.stringify({ duration_ms: durationMs }),
  });
  if (res.success) {
addLog("站立复位完成 ✅", "log-ok");
  } else {
addLog(`站立复位失败: ${res.message}`, "log-err");
  }
  sending = false;
  updateUI();
}

/* ── Emergency Stop ── */
async function doEmergencyStop() {
  if (sending) return;
  sending = true;
  updateUI();

  // Immediately zero all local joystick velocities
  stopContinuousSend();

addLog("⚠️ 紧急停止 — 正在回归初始位置...", "log-err");
  const durationMs = parseInt(document.getElementById("durationMs").value) || 2000;
  const res = await apiFetch("/api/emergency_stop", {
    method: "POST",
    body: JSON.stringify({ duration_ms: durationMs }),
  });
  if (res.success) {
addLog("紧急停止完成 — 机械臂已回归初始位置 ✅", "log-ok");
  } else {
addLog(`紧急停止失败: ${res.message}`, "log-err");
  }
  sending = false;
  updateUI();
}

/* ── UI State Update ── */
function updateUI() {
  const dot = document.getElementById("statusDot");
  const text = document.getElementById("statusText");
  const btnAcq = document.getElementById("btnAcquire");
  const btnRel = document.getElementById("btnRelease");
  const btnStand = document.getElementById("btnStandUp");
  const btnEstop = document.getElementById("btnEmergencyStop");
  const liveBadge = document.getElementById("liveBadge");
  const controlPanel = document.getElementById("controlPanel");
  const controlHint = document.getElementById("controlHint");

  if (isController) {
    dot.className = "status-dot green";
text.textContent = sending ? "发送中..." : "控制中";
    btnAcq.style.display = "none";
    btnRel.style.display = "";
    if (btnStand) btnStand.disabled = sending;
    if (btnEstop) btnEstop.disabled = sending;
    if (liveBadge) {
      liveBadge.style.display = "";
      liveBadge.className = sending ? "live-tag busy" : "live-tag";
document.getElementById('liveText').textContent = sending ? "忙碌" : "在线";
    }
    if (controlPanel) controlPanel.style.display = "";
    if (controlHint) controlHint.style.display = "none";
  } else {
    btnAcq.style.display = "";
    btnRel.style.display = "none";
    if (btnStand) btnStand.disabled = true;
    if (btnEstop) btnEstop.disabled = true;
    if (liveBadge) liveBadge.style.display = "none";
    if (controlPanel) controlPanel.style.display = "none";
    if (controlHint) controlHint.style.display = "";

    const hintIcon = document.getElementById("hintIcon");
    const hintTitle = document.getElementById("hintTitle");
    const hintDesc = document.getElementById("hintDesc");
    if (isLockedByOther) {
      dot.className = "status-dot yellow";
text.textContent = "已被占用";
      btnAcq.disabled = true;
      controlHint.className = "hint-overlay occupied";
      if (hintIcon) hintIcon.textContent = "⛔";
if (hintTitle) hintTitle.textContent = "机械臂已被占用";
if (hintDesc) hintDesc.innerHTML = '机械臂当前正被<strong>其他操作员控制</strong>。<br>请等待对方释放控制权。';
    } else {
      dot.className = "status-dot red";
text.textContent = "空闲";
      btnAcq.disabled = false;
      controlHint.className = "hint-overlay";
      if (hintIcon) hintIcon.textContent = "🔓";
if (hintTitle) hintTitle.textContent = "准备就绪";
if (hintDesc) hintDesc.innerHTML = '点击 <strong>「获取控制」</strong> 按钮来接管机械臂并进入操控界面。';
    }
  }
}

/* ── Lock Check (when occupied by another user) ── */
let lockCheckTimer = null;

function startLockCheck() {
  stopLockCheck();
  lockCheckTimer = setInterval(checkLockStatus, 3000);
}
function stopLockCheck() {
  if (lockCheckTimer) { clearInterval(lockCheckTimer); lockCheckTimer = null; }
}
async function checkLockStatus() {
  const res = await apiFetch("/api/status");
  if (res.locked === false) {
    isLockedByOther = false;
    stopLockCheck();
addLog("机械臂已可用 ✅", "log-ok");
    updateUI();
  }
}

/* ── Fullscreen + Landscape helpers (iOS compatible) ── */

/**
 * Detect iOS (Safari doesn't support standard Fullscreen API on <html>).
 */
function isIOS() {
  return /iPad|iPhone|iPod/.test(navigator.userAgent) ||
    (navigator.platform === 'MacIntel' && navigator.maxTouchPoints > 1);
}

/**
 * Enter fullscreen and lock orientation to landscape.
 * On iOS: use CSS-based "fake fullscreen" since Safari doesn't support the API.
 * On Android/Desktop: use standard Fullscreen API.
 */
function enterFullscreenLandscape() {
  if (isIOS()) {
    // iOS: apply CSS class for simulated fullscreen
    document.documentElement.classList.add('ios-fullscreen');
    // Scroll to top to hide Safari address bar
    window.scrollTo(0, 1);
addLog("已进入全屏模式 (iOS)", "log-info");
    return;
  }

  const el = document.documentElement;
  const requestFS = el.requestFullscreen
    || el.webkitRequestFullscreen
    || el.mozRequestFullScreen
    || el.msRequestFullscreen;
  if (!requestFS) return;

  requestFS.call(el).then(() => {
    const orientation = screen.orientation || screen.mozOrientation || screen.msOrientation;
    if (orientation && orientation.lock) {
      orientation.lock("landscape").catch(() => {});
    }
addLog("已进入全屏横屏模式", "log-info");
  }).catch(() => {});
}

/**
 * Exit fullscreen and unlock orientation.
 */
function exitFullscreen() {
  if (isIOS()) {
    document.documentElement.classList.remove('ios-fullscreen');
addLog("已退出全屏模式 (iOS)", "log-info");
    return;
  }

  const orientation = screen.orientation || screen.mozOrientation || screen.msOrientation;
  if (orientation && orientation.unlock) {
    try { orientation.unlock(); } catch (_) {}
  }

  const fsEl = document.fullscreenElement || document.webkitFullscreenElement;
  if (fsEl) {
    const exitFS = document.exitFullscreen || document.webkitExitFullscreen
      || document.mozCancelFullScreen || document.msExitFullscreen;
    if (exitFS) exitFS.call(document).catch(() => {});
addLog("已退出全屏模式", "log-info");
  }
}
