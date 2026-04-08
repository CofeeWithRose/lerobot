/* ═══════════════════════════════════════════════════════════════════
   API Helpers & Logging
   ═══════════════════════════════════════════════════════════════════ */

function addLog(msg, cls = "log-info") {
  const panel = document.getElementById("logPanel");
  if (!panel) return;
  const ts = new Date().toLocaleTimeString();
  const div = document.createElement("div");
  div.className = `log-entry ${cls}`;
  div.textContent = `[${ts}] ${msg}`;
  panel.prepend(div);
  while (panel.children.length > 50) panel.removeChild(panel.lastChild);
}

async function apiFetch(url, opts = {}) {
  try {
    const resp = await fetch(url, {
      credentials: "same-origin",
      headers: { "Content-Type": "application/json" },
      ...opts,
    });
    return await resp.json();
  } catch (e) {
    addLog(`网络错误: ${e.message}`, "log-err");
    return { success: false, message: e.message };
  }
}
