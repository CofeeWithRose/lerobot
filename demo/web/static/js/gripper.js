/* ═══════════════════════════════════════════════════════════════════
   Gripper Slider — absolute control (no spring-back)
   ═══════════════════════════════════════════════════════════════════ */

function initGripper() {
  const track = document.getElementById('gripperTrack');
  const thumb = document.getElementById('gripperThumb');
  const fill = document.getElementById('gripperFill');
  let dragging = false;

  function setGripper(norm) {
    gripperTarget = norm;
    const pct = norm * 100;
    fill.style.height = pct + '%';
    thumb.style.bottom = `calc(${pct}% - 12px)`;
    updateGaugeDisplay('gripper');
    scheduleGripperSend();
  }

  function updateFromEvent(e) {
    const rect = track.getBoundingClientRect();
    let py;
    if (e.touches) py = e.touches[0].clientY;
    else py = e.clientY;
    const norm = Math.max(0, Math.min(1, 1 - (py - rect.top) / rect.height));
    setGripper(norm);
  }

  thumb.addEventListener('mousedown', (e) => { e.preventDefault(); dragging = true; });
  thumb.addEventListener('touchstart', (e) => { e.preventDefault(); dragging = true; }, {passive:false});
  track.addEventListener('mousedown', (e) => { e.preventDefault(); dragging = true; updateFromEvent(e); });
  track.addEventListener('touchstart', (e) => { e.preventDefault(); dragging = true; updateFromEvent(e); }, {passive:false});
  window.addEventListener('mousemove', (e) => { if (dragging) updateFromEvent(e); });
  window.addEventListener('touchmove', (e) => { if (dragging) { e.preventDefault(); updateFromEvent(e); } }, {passive:false});
  window.addEventListener('mouseup', () => { dragging = false; });
  window.addEventListener('touchend', () => { dragging = false; });

  setGripper(0.5);

  return { setGripper };
}
