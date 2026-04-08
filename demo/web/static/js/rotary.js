/* ═══════════════════════════════════════════════════════════════════
   Rotary Knob (Wrist Roll) — with spring-back to center
   ═══════════════════════════════════════════════════════════════════ */

function initRotary() {
  const container = document.getElementById('rotaryContainer');
  const indicator = document.getElementById('rotaryIndicator');
  let dragging = false;
  let springBackAnim = null;

  function setIndicatorAngle(norm) {
    const angle = (norm - 0.5) * 270;
    indicator.style.transform = `translateX(-50%) rotate(${angle}deg)`;
  }

  function setRotaryDelta(norm) {
    deltaValues['wrist_roll'] = norm - 0.5;
    setIndicatorAngle(norm);
    updateGaugeDisplay('wrist_roll');
    onDeltaChanged();
  }

  function updateFromEvent(e) {
    const rect = container.getBoundingClientRect();
    const cx = rect.left + rect.width/2;
    const cy = rect.top + rect.height/2;
    let px, py;
    if (e.touches) { px = e.touches[0].clientX; py = e.touches[0].clientY; }
    else { px = e.clientX; py = e.clientY; }
    const angle = Math.atan2(px - cx, -(py - cy)) * 180 / Math.PI;
    const norm = Math.max(0, Math.min(1, 0.5 + angle / 270));
    setRotaryDelta(norm);
  }

  function springBack() {
    if (springBackAnim) cancelAnimationFrame(springBackAnim);
    const startNorm = deltaValues['wrist_roll'] + 0.5;
    const startTime = performance.now();

    function animate(now) {
      const elapsed = now - startTime;
      const t = Math.min(1, elapsed / SPRING_BACK_MS);
      const ease = 1 - Math.pow(1 - t, 3);
      const cur = startNorm + (0.5 - startNorm) * ease;
      setRotaryDelta(cur);
      if (t < 1) {
        springBackAnim = requestAnimationFrame(animate);
      } else {
        deltaValues['wrist_roll'] = 0;
        setIndicatorAngle(0.5);
        updateGaugeDisplay('wrist_roll');
        onDeltaChanged();
        springBackAnim = null;
      }
    }
    springBackAnim = requestAnimationFrame(animate);
  }

  container.addEventListener('mousedown', (e) => {
    e.preventDefault();
    if (springBackAnim) { cancelAnimationFrame(springBackAnim); springBackAnim = null; }
    dragging = true;
    updateFromEvent(e);
  });
  container.addEventListener('touchstart', (e) => {
    e.preventDefault();
    if (springBackAnim) { cancelAnimationFrame(springBackAnim); springBackAnim = null; }
    dragging = true;
    updateFromEvent(e);
  }, {passive:false});
  window.addEventListener('mousemove', (e) => { if (dragging) updateFromEvent(e); });
  window.addEventListener('touchmove', (e) => { if (dragging) { e.preventDefault(); updateFromEvent(e); } }, {passive:false});
  window.addEventListener('mouseup', () => { if (dragging) { dragging = false; springBack(); } });
  window.addEventListener('touchend', () => { if (dragging) { dragging = false; springBack(); } });

  // Initial: center (no rotation delta)
  setIndicatorAngle(0.5);

  return { setIndicatorAngle };
}
