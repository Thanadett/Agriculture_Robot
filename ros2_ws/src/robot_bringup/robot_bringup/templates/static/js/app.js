// --- Utility ---
const $ = (sel) => document.querySelector(sel);

// --- Set video size from body data attributes ---
const body = document.body;
const video = $("#video-stream");
const width = parseInt(body.dataset.videoWidth) || 640;
const height = parseInt(body.dataset.videoHeight) || 480;

video.style.width = width + "px";
video.style.height = height + "px";

// --- Timestamp ---
function updateTimestamp() {
  const now = new Date();
  const options = {
    year: 'numeric', month: '2-digit', day: '2-digit',
    hour: '2-digit', minute: '2-digit', second: '2-digit',
    hour12: false
  };
  $("#timestamp").innerText = now.toLocaleString('en-GB', options);
}
setInterval(updateTimestamp, 1000);
updateTimestamp();

// --- Status Indicator ---
let streamConnected = false;
let telemetryErrors = 0;

function updateStatusIndicator(connected) {
  const dot = $("#status-dot");
  const status = $("#stream-status");
  if (connected) {
    dot.className = "status-indicator status-connected";
    status.innerText = "Connected";
  } else {
    dot.className = "status-indicator status-disconnected";
    status.innerText = "Disconnected";
  }
}

// --- Telemetry Fetch ---
async function fetchTelemetry() {
  try {
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), 5000);
    const r = await fetch("/api/telemetry", { signal: controller.signal });
    clearTimeout(timeoutId);
    if (!r.ok) throw new Error(`HTTP ${r.status}`);
    const data = await r.json();
    $("#fps").textContent = data.fps?.toFixed(1) || "0";
    telemetryErrors = 0;
    if (!streamConnected) {
      streamConnected = true;
      updateStatusIndicator(true);
    }
  } catch (e) {
    telemetryErrors++;
    if (streamConnected) {
      streamConnected = false;
      updateStatusIndicator(false);
    }
    if (telemetryErrors > 5) await new Promise(resolve => setTimeout(resolve, 2000));
  }
}

const telemetryInterval = setInterval(fetchTelemetry, 500);
fetchTelemetry();

// --- Video stream events ---
video.addEventListener("load", () => { streamConnected = true; updateStatusIndicator(true); });
video.addEventListener("error", () => {
  streamConnected = false; updateStatusIndicator(false);
  setTimeout(() => { video.src = "/video?" + new Date().getTime(); }, 2000);
});
video.addEventListener("abort", () => { streamConnected = false; updateStatusIndicator(false); });
