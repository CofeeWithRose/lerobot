/* ═══════════════════════════════════════════════════════════════════
   Joystick Logic — axis-locked (H or V only), spring-back on release
   ═══════════════════════════════════════════════════════════════════ */

// Dead-zone threshold (px) before axis lock is decided
const AXIS_LOCK_THRESHOLD = 8;

function initJoystick(containerId, knobId, realMarkerId, xJoint, yJoint) {
  const container = document.getElementById(containerId);
  const knob = document.getElementById(knobId);
  const realMarker = document.getElementById(realMarkerId);
  let dragging = false;
  let springBackAnim = null;

  // Axis lock state
  let axisLocked = null;   // null | 'x' | 'y'
  let startPx = 0;         // pointer start X (client px)
  let startPy = 0;         // pointer start Y (client px)

  const radius = () => container.offsetWidth / 2;
  const knobR = () => knob.offsetWidth / 2;

  /**
   * Position the knob using percentage-based left/top + transform.
   * normX/normY are in [0, 1], where 0.5 = center.
   * We scale the movement range so the knob edge stays within the container.
   */
  function positionKnob(normX, normY) {
    const r = radius();
    const kr = knobR();
    if (r <= 0) {
      // Container not laid out yet — rely on CSS default centering
      return;
    }
    // maxTravel: how far (in %) the knob center can move from 50%
    // The knob center can go from kr to (2r - kr), i.e. range = 2*(r-kr)
    // In percentage of container: range / (2r) * 100 = (r-kr)/r * 100
    const maxPct = (r - kr) / r * 100; // max offset from center in %

    const pctX = 50 + (normX - 0.5) * 2 * maxPct;
    const pctY = 50 - (normY - 0.5) * 2 * maxPct;

    knob.style.left = pctX + '%';
    knob.style.top  = pctY + '%';
    knob.style.transform = 'translate(-50%, -50%)';
  }

  function positionRealMarker(normX, normY) {
    const r = radius();
    const mr = 6;
    if (r <= 0) return;
    const maxPct = (r - mr) / r * 100;

    const pctX = 50 + (normX - 0.5) * 2 * maxPct;
    const pctY = 50 - (normY - 0.5) * 2 * maxPct;

    realMarker.style.left = pctX + '%';
    realMarker.style.top  = pctY + '%';
    realMarker.style.transform = 'translate(-50%, -50%)';
  }

  function setDeltaFromNorm(normX, normY) {
    deltaValues[xJoint] = normX - 0.5;
    deltaValues[yJoint] = normY - 0.5;
    updateGaugeDisplay(xJoint);
    updateGaugeDisplay(yJoint);
    onDeltaChanged();
  }

  function computeNorm(e) {
    const rect = container.getBoundingClientRect();
    const r = radius();
    const kr = knobR();
    let px, py;
    if (e.touches) { px = e.touches[0].clientX; py = e.touches[0].clientY; }
    else { px = e.clientX; py = e.clientY; }

    let dx = px - rect.left - r;
    let dy = py - rect.top - r;
    const maxD = r - kr;

    // Apply axis lock: zero out the locked-out axis
    if (axisLocked === 'x') { dy = 0; }
    else if (axisLocked === 'y') { dx = 0; }

    // Clamp to circle
    const dist = Math.sqrt(dx * dx + dy * dy);
    if (dist > maxD) { dx = dx / dist * maxD; dy = dy / dist * maxD; }

    const normX = Math.max(0, Math.min(1, 0.5 + dx / (2 * maxD)));
    const normY = Math.max(0, Math.min(1, 0.5 - dy / (2 * maxD)));
    return { normX, normY, clientX: px, clientY: py };
  }

  function updateFromEvent(e) {
    const { normX, normY } = computeNorm(e);
    positionKnob(normX, normY);
    setDeltaFromNorm(normX, normY);
  }

  function springBack() {
    if (springBackAnim) cancelAnimationFrame(springBackAnim);

    const startX = deltaValues[xJoint] + 0.5;
    const startY = deltaValues[yJoint] + 0.5;
    const startTime = performance.now();

    function animate(now) {
      const elapsed = now - startTime;
      const t = Math.min(1, elapsed / SPRING_BACK_MS);
      const ease = 1 - Math.pow(1 - t, 3);

      const curX = startX + (0.5 - startX) * ease;
      const curY = startY + (0.5 - startY) * ease;

      positionKnob(curX, curY);
      setDeltaFromNorm(curX, curY);

      if (t < 1) {
        springBackAnim = requestAnimationFrame(animate);
      } else {
        positionKnob(0.5, 0.5);
        deltaValues[xJoint] = 0;
        deltaValues[yJoint] = 0;
        updateGaugeDisplay(xJoint);
        updateGaugeDisplay(yJoint);
        onDeltaChanged();
        springBackAnim = null;
      }
    }
    springBackAnim = requestAnimationFrame(animate);
  }

  function getPointerPos(e) {
    if (e.touches) return { x: e.touches[0].clientX, y: e.touches[0].clientY };
    return { x: e.clientX, y: e.clientY };
  }

  function onStart(e) {
    e.preventDefault();
    if (springBackAnim) { cancelAnimationFrame(springBackAnim); springBackAnim = null; }
    dragging = true;
    axisLocked = null; // reset axis lock for new gesture
    const pos = getPointerPos(e);
    startPx = pos.x;
    startPy = pos.y;
    knob.classList.add('active');
    // Don't move knob yet — wait for axis lock decision
  }

  function onMove(e) {
    if (!dragging) return;
    e.preventDefault();

    // If axis not yet locked, check if we've moved enough to decide
    if (axisLocked === null) {
      const pos = getPointerPos(e);
      const adx = Math.abs(pos.x - startPx);
      const ady = Math.abs(pos.y - startPy);
      if (adx < AXIS_LOCK_THRESHOLD && ady < AXIS_LOCK_THRESHOLD) {
        return; // still in dead zone, don't move
      }
      // Lock to the dominant axis
      axisLocked = (adx >= ady) ? 'x' : 'y';
    }

    updateFromEvent(e);
  }

  function onEnd() {
    if (!dragging) return;
    dragging = false;
    axisLocked = null;
    knob.classList.remove('active');
    springBack();
  }

  knob.addEventListener('mousedown', onStart);
  knob.addEventListener('touchstart', onStart, { passive: false });
  container.addEventListener('mousedown', onStart);
  container.addEventListener('touchstart', onStart, { passive: false });
  window.addEventListener('mousemove', onMove);
  window.addEventListener('touchmove', onMove, { passive: false });
  window.addEventListener('mouseup', onEnd);
  window.addEventListener('touchend', onEnd);

  // Initial position: exact center (no movement)
  // Use double rAF to ensure the container has been laid out after display:none → visible
  function centerKnob() {
    positionKnob(0.5, 0.5);
    positionRealMarker(0.5, 0.5);
  }
  requestAnimationFrame(() => requestAnimationFrame(centerKnob));

  return { positionKnob, positionRealMarker, centerKnob };
}
