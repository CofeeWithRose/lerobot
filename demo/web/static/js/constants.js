/* ═══════════════════════════════════════════════════════════════════
   Constants & State
   ═══════════════════════════════════════════════════════════════════ */
const JOINTS = [
  "shoulder_pan", "shoulder_lift", "elbow_flex",
  "wrist_flex", "wrist_roll", "gripper"
];

// Delta values: joystick deflection for each joint.
// Range: -0.5 to +0.5 (0 = no movement, center position of joystick)
// These are sent to the server as "velocity" — the server applies them
// continuously in its own control loop.
const deltaValues = {};
JOINTS.forEach(j => deltaValues[j] = 0.0);

// Gripper is absolute (not incremental) — user drags to a position
let gripperTarget = 0.5;

let isController = false;
let isLockedByOther = false;
let moveRanges = {};
let sending = false;

// Spring-back animation speed (how fast knob returns to center)
const SPRING_BACK_MS = 150;
