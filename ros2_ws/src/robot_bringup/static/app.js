const $ = (sel) => document.querySelector(sel);

$("#btn-grid").addEventListener("click", async () => {
  const r = await fetch("/toggle/grid");
  $("#status").innerText = await r.text();
});

$("#btn-center").addEventListener("click", async () => {
  const r = await fetch("/toggle/center");
  $("#status").innerText = await r.text();
});

setInterval(() => {
  $("#timestamp").innerText = new Date().toLocaleTimeString();
}, 1000);

// Telemetry polling (live overlay)
const fetchOnce = false; // เปิด polling
async function fetchTelemetry() {
  try {
    const r = await fetch("/api/telemetry");
    const data = await r.json();

    // แสดง FPS และ center
    $("#fps").textContent = data.fps?.toFixed(1) || 0;
    $("#cx").textContent = data.center?.x || 0;
    $("#cy").textContent = data.center?.y || 0;

    // แสดง raw telemetry JSON
    $("#telemetry").textContent = JSON.stringify(data, null, 2);
  } catch (e) {
    $("#telemetry").textContent = "fetch error: " + e;
  }
}

if (!fetchOnce) {
  setInterval(fetchTelemetry, 500); // ทุก 0.5 วินาที
  fetchTelemetry();
}
