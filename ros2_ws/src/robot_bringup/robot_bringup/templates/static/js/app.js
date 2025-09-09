// Utility functions
const $ = (sel) => document.querySelector(sel);

// Global state
let streamConnected = false;
let telemetryErrors = 0;

// Update timestamp every second
function updateTimestamp() {
    try {
        const now = new Date();
        const options = {
            year: 'numeric',
            month: '2-digit',
            day: '2-digit',
            hour: '2-digit',
            minute: '2-digit',
            second: '2-digit',
            hour12: false
        };
        $("#timestamp").innerText = now.toLocaleString('en-GB', options);
    } catch (e) {
        $("#timestamp").innerText = new Date().toLocaleString();
    }
}

// Start timestamp updates
setInterval(updateTimestamp, 1000);
updateTimestamp();

// Update status indicator
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

// Telemetry fetching
async function fetchTelemetry() {
    try {
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), 5000);
        
        const r = await fetch("/api/telemetry", {
            signal: controller.signal
        });
        
        clearTimeout(timeoutId);
        
        if (!r.ok) throw new Error(`HTTP ${r.status}`);
        
        const data = await r.json();
        
        // Update overlay data
        $("#fps").textContent = data.fps?.toFixed(1) || "0";
        
        // Reset error counter on success
        telemetryErrors = 0;
        
        // Update connection status
        if (!streamConnected) {
            streamConnected = true;
            updateStatusIndicator(true);
        }
        
    } catch (e) {
        telemetryErrors++;
        
        let errorMsg = `Telemetry error (${telemetryErrors}): `;
        
        if (e.name === 'AbortError') {
            errorMsg += "Request timeout - server may be busy";
        } else {
            errorMsg += e.message;
        }
        
        // Update connection status
        if (streamConnected) {
            streamConnected = false;
            updateStatusIndicator(false);
        }
        
        // If too many errors, slow down polling
        if (telemetryErrors > 5) {
            await new Promise(resolve => setTimeout(resolve, 2000));
        }
    }
}

// Start telemetry polling
const telemetryInterval = setInterval(fetchTelemetry, 500);
fetchTelemetry();

// Video stream handling
const videoElement = $("#video-stream");
const streamStatusEl = $("#stream-status");

videoElement.addEventListener("load", function() {
    console.log("Video stream loaded successfully");
    streamConnected = true;
    updateStatusIndicator(true);
});

videoElement.addEventListener("error", function() {
    console.log("Video stream error, attempting to reload...");
    streamConnected = false;
    updateStatusIndicator(false);
    
    // Wait a bit before retrying
    setTimeout(() => {
        const timestamp = new Date().getTime();
        this.src = "/video?" + timestamp;
    }, 2000);
});

videoElement.addEventListener("abort", function() {
    console.log("Video stream aborted");
    streamConnected = false;
    updateStatusIndicator(false);
});

// Check stream health periodically
setInterval(() => {
    if (!streamConnected && telemetryErrors > 3) {
        console.log("Attempting to reconnect video stream...");
        const timestamp = new Date().getTime();
        videoElement.src = "/video?" + timestamp;
    }
}, 10000);

// Page visibility handling
document.addEventListener("visibilitychange", function() {
    if (document.hidden) {
        console.log("Page hidden, reducing update frequency");
        clearInterval(telemetryInterval);
    } else {
        console.log("Page visible, resuming normal updates");
        fetchTelemetry();
        setInterval(fetchTelemetry, 500);
    }
});

// Set initial status
$("#status-dot").className = "status-indicator status-loading";
$("#stream-status").innerText = "Connecting...";

// Performance monitoring
let frameCount = 0;
setInterval(() => {
    frameCount++;
    if (frameCount % 120 === 0) { // Every 120 intervals (60 seconds)
        console.log(`Performance: ${frameCount} telemetry updates, Stream: ${streamConnected ? 'Connected' : 'Disconnected'}`);
    }
}, 500);