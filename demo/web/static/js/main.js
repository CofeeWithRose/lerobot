/* ═══════════════════════════════════════════════════════════════════
   Init Controls & App Entry Point
   ═══════════════════════════════════════════════════════════════════ */

let joyLeftCtrl = null;
let joyRightCtrl = null;
let rotaryCtrl = null;
let gripperCtrl = null;

function initControls() {
  // ISO excavator standard mapping (direction inversion handled server-side):
  // Left stick:  X = shoulder_pan (swing L/R),  Y = elbow_flex (arm in/out)
  // Right stick: X = wrist_flex (bucket curl/dump), Y = shoulder_lift (boom up/down)
  joyLeftCtrl = initJoystick('joyLeft', 'knobLeft', 'realLeft', 'shoulder_pan', 'elbow_flex');
  joyRightCtrl = initJoystick('joyRight', 'knobRight', 'realRight', 'wrist_flex', 'shoulder_lift');
  rotaryCtrl = initRotary();
  gripperCtrl = initGripper();
}

/* ── App Init ── */
async function init() {
  const res = await apiFetch("/api/status");
  if (res.move_ranges) moveRanges = res.move_ranges;
  isController = res.is_controller;
  isLockedByOther = !res.is_controller && res.locked === true;
  updateUI();

  if (isLockedByOther) {
addLog("机械臂已被其他用户占用", "log-err");
    startLockCheck();
  }

  if (isController) {
    initControls();
    // Re-center joystick knobs after layout is fully computed
    requestAnimationFrame(() => requestAnimationFrame(() => {
      if (joyLeftCtrl && joyLeftCtrl.centerKnob) joyLeftCtrl.centerKnob();
      if (joyRightCtrl && joyRightCtrl.centerKnob) joyRightCtrl.centerKnob();
    }));
    startPolling();
addLog("已通过现有会话重新连接", "log-ok");
  }
}

init();
