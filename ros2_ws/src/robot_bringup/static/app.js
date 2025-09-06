const $ = (sel)=>document.querySelector(sel);

$("#btn-grid").addEventListener("click", async ()=>{
  const r = await fetch("/toggle/grid");
  $("#status").innerText = await r.text();
});

$("#btn-center").addEventListener("click", async ()=>{
  const r = await fetch("/toggle/center");
  $("#status").innerText = await r.text();
});

setInterval(()=>{
  $("#timestamp").innerText = new Date().toLocaleTimeString();
}, 1000);

// --- Telemetry scaffold (disabled by default) ---
// หากต้องการใช้งาน ให้เปลี่ยน fetchOnce=true -> false เพื่อเริ่มดึงข้อมูลแบบ polling
const fetchOnce = true;
async function fetchTelemetry(){
  try{
    const r = await fetch("/api/telemetry");
    const j = await r.json();
    $("#telemetry").textContent = JSON.stringify(j, null, 2);
  }catch(e){
    $("#telemetry").textContent = "fetch error: " + e;
  }
}
if(!fetchOnce){
  setInterval(fetchTelemetry, 1000);
  fetchTelemetry();
}
