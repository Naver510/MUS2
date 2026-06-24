#ifndef WEB_UI_H
#define WEB_UI_H

#include <Arduino.h>

const char web_ui_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>GRUZIK Telemetry Dashboard</title>
    <style>
        :root {
            --bg-color: #0f172a;
            --panel-bg: #1e293b;
            --text-main: #f8fafc;
            --text-muted: #94a3b8;
            --accent-primary: #3b82f6;
            --accent-success: #10b981;
            --accent-danger: #ef4444;
            --accent-warn: #f59e0b;
            --accent-purple: #8b5cf6;
            --border-color: #334155;
            --shadow: 0 4px 6px -1px rgba(0,0,0,.1), 0 2px 4px -1px rgba(0,0,0,.06);
            --radius: 12px;
        }

        * { margin: 0; padding: 0; box-sizing: border-box; }

        body {
            font-family: 'Inter', -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
            background-color: var(--bg-color);
            color: var(--text-main);
            padding: 20px;
            display: flex;
            flex-direction: column;
            align-items: center;
        }

        /* ── Tabs ── */
        .tabs {
            display: flex;
            gap: 8px;
            margin-bottom: 20px;
            width: 100%;
            max-width: 1000px;
        }
        .tab-btn {
            flex: 1;
            padding: 10px 18px;
            border: 1px solid var(--border-color);
            border-radius: 8px;
            background: var(--panel-bg);
            color: var(--text-muted);
            font-size: .95rem;
            font-weight: 600;
            cursor: pointer;
            transition: all .2s;
        }
        .tab-btn.active {
            background: var(--accent-primary);
            color: #fff;
            border-color: var(--accent-primary);
        }
        .tab-btn:hover:not(.active) { color: var(--text-main); border-color: var(--accent-primary); }

        .tab-content { display: none; width: 100%; }
        .tab-content.active { display: block; }

        /* ── Layout grid ── */
        .container {
            max-width: 1000px;
            width: 100%;
            display: grid;
            gap: 20px;
            grid-template-columns: 1fr;
        }

        @media (min-width: 768px) {
            .container { grid-template-columns: repeat(2, 1fr); }
            .header-panel, .full-width { grid-column: span 2; }
        }

        /* ── Panels ── */
        .panel {
            background-color: var(--panel-bg);
            border-radius: var(--radius);
            padding: 20px;
            box-shadow: var(--shadow);
            border: 1px solid var(--border-color);
        }

        .panel-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 20px;
            border-bottom: 1px solid var(--border-color);
            padding-bottom: 10px;
        }

        h2 { font-size: 1.2rem; color: var(--text-main); font-weight: 600; }

        /* ── Connection badge ── */
        .status-badge {
            display: inline-flex; align-items: center; gap: 6px;
            padding: 6px 12px; border-radius: 20px; font-size: .85rem; font-weight: 600;
            background-color: rgba(239,68,68,.2); color: var(--accent-danger);
            transition: all .3s ease;
        }
        .status-badge.connected { background-color: rgba(16,185,129,.2); color: var(--accent-success); }
        .status-dot { width: 8px; height: 8px; border-radius: 50%; background-color: currentColor; }

        /* ── Telemetry data cards ── */
        .telemetry-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 15px; }

        .data-card {
            background-color: rgba(15,23,42,.5);
            padding: 15px; border-radius: 8px; border: 1px solid var(--border-color);
        }

        .data-label {
            font-size: .85rem; color: var(--text-muted);
            text-transform: uppercase; letter-spacing: .5px; margin-bottom: 5px;
        }

        .data-value {
            font-size: 1.8rem; font-weight: 700;
            color: var(--accent-primary); font-variant-numeric: tabular-nums;
        }
        .data-unit { font-size: .9rem; color: var(--text-muted); font-weight: normal; }

        /* ── Buttons ── */
        .btn-group { display: flex; gap: 10px; flex-wrap: wrap; margin-bottom: 20px; }

        button {
            flex: 1; min-width: 100px; padding: 12px 16px;
            border: none; border-radius: 8px; font-size: 1rem; font-weight: 600;
            cursor: pointer; transition: all .2s ease; color: white;
            background-color: var(--border-color);
        }
        button:active { transform: translateY(2px); }
        button:hover  { filter: brightness(1.1); }
        .btn-start   { background-color: var(--accent-success); }
        .btn-stop    { background-color: var(--accent-danger); }
        .btn-primary { background-color: var(--accent-primary); }

        /* ── Sliders ── */
        .slider-container { margin-bottom: 15px; }
        .slider-header {
            display: flex; justify-content: space-between;
            margin-bottom: 8px; font-size: .9rem; font-weight: 500;
        }
        .slider-val { color: var(--accent-primary); font-weight: 700; }

        input[type=range] { -webkit-appearance: none; width: 100%; background: transparent; }
        input[type=range]::-webkit-slider-thumb {
            -webkit-appearance: none; height: 20px; width: 20px; border-radius: 50%;
            background: var(--accent-primary); cursor: pointer; margin-top: -8px;
            box-shadow: 0 0 10px rgba(59,130,246,.5);
        }
        input[type=range]::-webkit-slider-runnable-track {
            width: 100%; height: 6px; cursor: pointer;
            background: var(--border-color); border-radius: 3px;
        }

        /* ── Debug panel specific ── */
        .hw-status-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 12px;
            margin-bottom: 20px;
        }

        .hw-card {
            padding: 16px; border-radius: 10px; border: 2px solid var(--border-color);
            background: rgba(15,23,42,.6);
            transition: border-color .3s;
        }
        .hw-card.ok    { border-color: var(--accent-success); }
        .hw-card.fail  { border-color: var(--accent-danger); }
        .hw-card.busy  { border-color: var(--accent-warn); }

        .hw-name {
            font-size: .8rem; text-transform: uppercase; letter-spacing: 1px;
            color: var(--text-muted); margin-bottom: 6px;
        }
        .hw-state {
            font-size: 1.1rem; font-weight: 700; color: var(--text-main);
            display: flex; align-items: center; gap: 8px;
        }
        .hw-dot {
            width: 10px; height: 10px; border-radius: 50%;
            background: var(--border-color); flex-shrink: 0;
            transition: background .3s;
        }
        .hw-card.ok   .hw-dot { background: var(--accent-success); box-shadow: 0 0 8px var(--accent-success); }
        .hw-card.fail .hw-dot { background: var(--accent-danger);  box-shadow: 0 0 8px var(--accent-danger); }
        .hw-card.busy .hw-dot { background: var(--accent-warn);    box-shadow: 0 0 8px var(--accent-warn); animation: blink 0.7s infinite; }

        @keyframes blink { 0%,100%{opacity:1} 50%{opacity:.3} }

        .hw-sub { font-size: .75rem; color: var(--text-muted); margin-top: 4px; }

        /* Progress bar (calibration) */
        .cal-bar-wrap {
            width: 100%; height: 6px; background: var(--border-color);
            border-radius: 3px; margin-top: 8px; overflow: hidden;
        }
        .cal-bar {
            height: 100%; border-radius: 3px;
            background: var(--accent-warn);
            transition: width .2s linear;
        }

        /* Encoder live bars */
        .enc-row { margin-bottom: 14px; }
        .enc-label { display: flex; justify-content: space-between; font-size: .85rem; margin-bottom: 4px; }
        .enc-label span:last-child { color: var(--accent-primary); font-weight: 700; font-variant-numeric: tabular-nums; }
        .enc-bar-wrap {
            width: 100%; height: 10px; background: var(--border-color);
            border-radius: 5px; overflow: hidden; position: relative;
        }
        /* center-origin bar for signed delta */
        .enc-bar-center {
            position: absolute; top:0; height:100%; border-radius: 5px;
            background: var(--accent-primary);
            transition: left .1s, width .1s;
        }

        /* IMU number table */
        .imu-table { width: 100%; border-collapse: collapse; font-size: .9rem; }
        .imu-table td { padding: 7px 10px; border-bottom: 1px solid var(--border-color); }
        .imu-table td:first-child { color: var(--text-muted); font-size: .8rem; text-transform: uppercase; }
        .imu-table td:last-child  { text-align: right; font-weight: 700; font-variant-numeric: tabular-nums; color: var(--accent-primary); }

        /* Odometry mini display */
        .odom-grid { display: grid; grid-template-columns: repeat(3,1fr); gap:10px; }
        .odom-card {
            text-align: center; padding: 10px; border-radius: 8px;
            background: rgba(15,23,42,.5); border: 1px solid var(--border-color);
        }
        .odom-card .lbl { font-size: .7rem; text-transform: uppercase; color: var(--text-muted); }
        .odom-card .val { font-size: 1.2rem; font-weight: 700; color: var(--accent-primary);
                          font-variant-numeric: tabular-nums; }
    </style>
</head>
<body>

    <!-- Connection Badge (always visible) -->
    <div style="width:100%;max-width:1000px;display:flex;justify-content:space-between;align-items:center;margin-bottom:12px;">
        <h2 style="font-size:1.4rem;">GRUZIK Dashboard</h2>
        <div id="conn-status" class="status-badge">
            <div class="status-dot"></div>
            <span>Disconnected</span>
        </div>
    </div>

    <!-- Tab buttons -->
    <div class="tabs">
        <button class="tab-btn active" id="tab-btn-main"  onclick="showTab('main')">📊 Telemetry</button>
        <button class="tab-btn"        id="tab-btn-ctrl"  onclick="showTab('ctrl')">🎮 Control</button>
        <button class="tab-btn"        id="tab-btn-debug" onclick="showTab('debug')">🔬 Debug</button>
        <button class="tab-btn"        id="tab-btn-map"   onclick="showTab('map')">🗺️ Map</button>
    </div>

    <!-- ═══════════════════════════════════════════════════════
         TAB: TELEMETRY
    ══════════════════════════════════════════════════════════ -->
    <div class="tab-content active" id="tab-main">
        <div class="container">
            <div class="panel full-width">
                <div class="panel-header"><h2>Real-Time Telemetry</h2></div>
                <div class="telemetry-grid">
                    <div class="data-card">
                        <div class="data-label">Left Speed</div>
                        <div class="data-value" id="val-speedL">0.00<span class="data-unit"> m/s</span></div>
                        <div class="data-label" style="font-size:.7rem;margin-top:5px;">Enc count: <span id="val-encL">0</span></div>
                    </div>
                    <div class="data-card">
                        <div class="data-label">Right Speed</div>
                        <div class="data-value" id="val-speedR">0.00<span class="data-unit"> m/s</span></div>
                        <div class="data-label" style="font-size:.7rem;margin-top:5px;">Enc count: <span id="val-encR">0</span></div>
                    </div>
                    <div class="data-card">
                        <div class="data-label">IMU Z-Gyro (corrected)</div>
                        <div class="data-value" id="val-gyro">0.0<span class="data-unit"> °/s</span></div>
                        <div class="data-label" style="font-size:.7rem;margin-top:5px;">Raw: <span id="val-gyroRaw">0.0</span> | Bias: <span id="val-gyroBias">—</span></div>
                    </div>
                    <div class="data-card">
                        <div class="data-label">Line Error</div>
                        <div class="data-value" id="val-error">0</div>
                    </div>
                    <div class="data-card" style="grid-column:span 2;">
                        <div class="data-label">Current State</div>
                        <div class="data-value" id="val-state" style="color:var(--text-main);font-size:1.4rem;">Idle</div>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <!-- ═══════════════════════════════════════════════════════
         TAB: CONTROL
    ══════════════════════════════════════════════════════════ -->
    <div class="tab-content" id="tab-ctrl">
        <div class="container">
            <div class="panel">
                <div class="panel-header"><h2>Robot Control</h2></div>

                <div class="btn-group">
                    <button class="btn-start" onclick="sendCmd('mode','start')">START</button>
                    <button class="btn-stop"  onclick="sendCmd('mode','stop')">STOP</button>
                </div>

                <div class="btn-group">
                    <button class="btn-primary"
                        onmousedown="sendCmd('mode','force_fwd')" onmouseup="sendCmd('mode','stop')"
                        ontouchstart="sendCmd('mode','force_fwd')" ontouchend="sendCmd('mode','stop')">
                        Force Forward (Hold)
                    </button>
                </div>

                <div class="btn-group">
                    <button class="btn-primary" onclick="sendCmd('cmd','cal')">Calibrate Line</button>
                    <button class="btn-primary" onclick="sendCmd('cmd','params')">Save Params</button>
                </div>

                <h3 style="font-size:1rem;margin:20px 0 10px 0;">Mapping Modes</h3>
                <div class="btn-group">
                    <button onclick="sendCmd('mode','mapping')"       style="background:#8b5cf6;">Record Map</button>
                    <button onclick="sendCmd('mode','unmapping')"     style="background:#f59e0b;">Play Map</button>
                    <button onclick="sendCmd('mode','speedunmapping')"style="background:#ec4899;">Fast Play Map</button>
                </div>
            </div>

            <div class="panel">
                <div class="panel-header"><h2>PID &amp; Speed Tuning</h2></div>

                <div class="slider-container">
                    <div class="slider-header"><span>Base Speed (m/s)</span><span class="slider-val" id="disp-baseSpeed">1.0</span></div>
                    <input type="range" id="sl-baseSpeed" min="0" max="2.5" step="0.1" value="1.0" oninput="updateSlider('baseSpeed')" onchange="sendParam('baseSpeed')">
                </div>
                <div class="slider-container">
                    <div class="slider-header"><span>Kp (Proportional)</span><span class="slider-val" id="disp-kp">0.25</span></div>
                    <input type="range" id="sl-kp" min="0" max="5" step="0.05" value="0.25" oninput="updateSlider('kp')" onchange="sendParam('kp')">
                </div>
                <div class="slider-container">
                    <div class="slider-header"><span>Kd (Derivative)</span><span class="slider-val" id="disp-kd">1.20</span></div>
                    <input type="range" id="sl-kd" min="0" max="10" step="0.05" value="1.2" oninput="updateSlider('kd')" onchange="sendParam('kd')">
                </div>
                <div class="slider-container">
                    <div class="slider-header"><span>Kp Scale Multiplier</span><span class="slider-val" id="disp-kpMult">1.000</span></div>
                    <input type="range" id="sl-kpMult" min="0.1" max="10.0" step="0.1" value="1.0" oninput="updateSlider('kpMult')" onchange="sendParam('kpMult')">
                </div>
                <div class="slider-container">
                    <div class="slider-header"><span>Kd Scale Multiplier</span><span class="slider-val" id="disp-kdMult">1.000</span></div>
                    <input type="range" id="sl-kdMult" min="0.1" max="10.0" step="0.1" value="1.0" oninput="updateSlider('kdMult')" onchange="sendParam('kdMult')">
                </div>
            </div>
        </div>
    </div>

    <!-- ═══════════════════════════════════════════════════════
         TAB: DEBUG
    ══════════════════════════════════════════════════════════ -->
    <div class="tab-content" id="tab-debug">
        <div class="container">

            <!-- Hardware Status Cards -->
            <div class="panel full-width">
                <div class="panel-header">
                    <h2>Hardware Status</h2>
                    <button id="btn-recal" class="btn-primary" style="flex:0;min-width:0;padding:8px 14px;font-size:.85rem;"
                            onclick="recalIMU()">↺ Recalibrate IMU</button>
                </div>

                <div class="hw-status-grid">
                    <!-- IMU card -->
                    <div class="hw-card" id="hw-imu">
                        <div class="hw-name">BMI160 IMU</div>
                        <div class="hw-state">
                            <div class="hw-dot" id="imu-dot"></div>
                            <span id="imu-state-text">Waiting…</span>
                        </div>
                        <div class="hw-sub" id="imu-sub">—</div>
                        <!-- calibration progress bar (hidden when not calibrating) -->
                        <div class="cal-bar-wrap" id="cal-bar-wrap" style="display:none;">
                            <div class="cal-bar" id="cal-bar" style="width:0%;"></div>
                        </div>
                    </div>

                    <!-- ENC1 card -->
                    <div class="hw-card" id="hw-enc1">
                        <div class="hw-name">Encoder 1 — Left (GPIO 6/7)</div>
                        <div class="hw-state">
                            <div class="hw-dot" id="enc1-dot"></div>
                            <span id="enc1-state-text">Waiting…</span>
                        </div>
                        <div class="hw-sub" id="enc1-sub">Count: 0 | Δ/tick: 0</div>
                    </div>

                    <!-- ENC2 card -->
                    <div class="hw-card" id="hw-enc2">
                        <div class="hw-name">Encoder 2 — Right (GPIO 8/9)</div>
                        <div class="hw-state">
                            <div class="hw-dot" id="enc2-dot"></div>
                            <span id="enc2-state-text">Waiting…</span>
                        </div>
                        <div class="hw-sub" id="enc2-sub">Count: 0 | Δ/tick: 0</div>
                    </div>
                </div>
            </div>

            <!-- IMU Detail -->
            <div class="panel">
                <div class="panel-header"><h2>IMU — Gyro Z Detail</h2></div>
                <table class="imu-table">
                    <tr><td>Raw reading</td>   <td><span id="dbg-gyroRaw">—</span> °/s</td></tr>
                    <tr><td>Bias offset</td>   <td><span id="dbg-gyroBias">—</span> °/s</td></tr>
                    <tr><td>Corrected</td>      <td><span id="dbg-gyroZ">—</span> °/s</td></tr>
                    <tr><td>Initialized</td>   <td><span id="dbg-imuInit">—</span></td></tr>
                    <tr><td>Calibrating</td>   <td><span id="dbg-imuCal">—</span></td></tr>
                    <tr><td>Cal progress</td>  <td><span id="dbg-calPct">—</span> %</td></tr>
                </table>
            </div>

            <!-- Encoder Detail -->
            <div class="panel">
                <div class="panel-header"><h2>Encoders — Live Counts</h2></div>

                <div class="enc-row">
                    <div class="enc-label">
                        <span>Left (ENC1)</span>
                        <span id="dbg-encL">0</span>
                    </div>
                </div>

                <div class="enc-row">
                    <div class="enc-label">
                        <span>Right (ENC2)</span>
                        <span id="dbg-encR">0</span>
                    </div>
                </div>

                <div style="margin-top:16px;">
                    <div class="data-label" style="margin-bottom:8px;">Delta per control tick (counts/ms) — should pulse when wheels spin</div>

                    <!-- ENC1 delta bar -->
                    <div class="enc-label">
                        <span>Left Δ</span><span id="dbg-encLDelta">0</span>
                    </div>
                    <div class="enc-bar-wrap" style="margin-bottom:10px;">
                        <div class="enc-bar-center" id="bar-encL" style="left:50%;width:0%;background:var(--accent-primary);"></div>
                    </div>

                    <!-- ENC2 delta bar -->
                    <div class="enc-label">
                        <span>Right Δ</span><span id="dbg-encRDelta">0</span>
                    </div>
                    <div class="enc-bar-wrap">
                        <div class="enc-bar-center" id="bar-encR" style="left:50%;width:0%;background:var(--accent-purple);"></div>
                    </div>
                </div>

                <div style="margin-top:16px;">
                    <div class="data-label" style="margin-bottom:8px;">Speed (m/s)</div>
                    <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;text-align:center;">
                        <div style="background:rgba(15,23,42,.5);padding:10px;border-radius:8px;border:1px solid var(--border-color);">
                            <div class="data-label">Left</div>
                            <div style="font-size:1.3rem;font-weight:700;color:var(--accent-primary);" id="dbg-speedL">0.00</div>
                        </div>
                        <div style="background:rgba(15,23,42,.5);padding:10px;border-radius:8px;border:1px solid var(--border-color);">
                            <div class="data-label">Right</div>
                            <div style="font-size:1.3rem;font-weight:700;color:var(--accent-purple);" id="dbg-speedR">0.00</div>
                        </div>
                    </div>
                </div>
            </div>

            <!-- Odometry -->
            <div class="panel full-width">
                <div class="panel-header"><h2>Odometry (fused encoder + gyro)</h2></div>
                <div class="odom-grid">
                    <div class="odom-card">
                        <div class="lbl">X (m)</div>
                        <div class="val" id="dbg-odomX">0.000</div>
                    </div>
                    <div class="odom-card">
                        <div class="lbl">Y (m)</div>
                        <div class="val" id="dbg-odomY">0.000</div>
                    </div>
                    <div class="odom-card">
                        <div class="lbl">Yaw (°)</div>
                        <div class="val" id="dbg-odomYaw">0.0</div>
                    </div>
                </div>
            </div>

        </div><!-- /container -->
    </div><!-- /tab-debug -->

    <!-- ═══════════════════════════════════════════════════════
         TAB: MAP
    ══════════════════════════════════════════════════════════ -->
    <div class="tab-content" id="tab-map">
        <div class="container">
            <div class="panel full-width">
                <div class="panel-header">
                    <h2>Map Viewer</h2>
                    <div>
                        <button style="padding:6px 12px;font-size:.9rem;" onclick="fetchMap('/map_raw')">Raw Map</button>
                        <button style="padding:6px 12px;font-size:.9rem;background:var(--accent-primary);" onclick="fetchMap('/map_opt')">Optimized Map</button>
                    </div>
                </div>
                <div style="width:100%;text-align:center;margin-top:10px;">
                    <canvas id="mapCanvas" width="400" height="400"
                            style="background:var(--bg-main);border-radius:8px;border:1px solid var(--border-color);width:100%;max-width:500px;"></canvas>
                </div>
            </div>
        </div>
    </div>

    <script>
        // ── Tab switching ──────────────────────────────────────────────────────
        function showTab(name) {
            document.querySelectorAll('.tab-content').forEach(t => t.classList.remove('active'));
            document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
            document.getElementById('tab-' + name).classList.add('active');
            document.getElementById('tab-btn-' + name).classList.add('active');
        }

        // ── WebSocket ──────────────────────────────────────────────────────────
        let ws;
        const statusBadge = document.getElementById('conn-status');
        const statusText  = statusBadge.querySelector('span');

        // Track last encoder counts to detect if they change (= encoder working)
        let prevEncL = null, prevEncR = null;
        let encLMoved = false, encRMoved = false;

        function initWebSocket() {
            const host = window.location.hostname || "192.168.4.1";
            ws = new WebSocket(`ws://${host}/ws`);

            ws.onopen = () => {
                statusBadge.classList.add('connected');
                statusText.innerText = 'Connected';
            };

            ws.onclose = () => {
                statusBadge.classList.remove('connected');
                statusText.innerText = 'Disconnected';
                setTimeout(initWebSocket, 2000);
            };

            ws.onmessage = (event) => {
                try {
                    const d = JSON.parse(event.data);
                    if (d.type === 'telemetry') updateTelemetry(d);
                    else if (d.type === 'params') updateParams(d);
                } catch(e) { console.error("Parse error", e); }
            };
        }

        // ── Telemetry update ───────────────────────────────────────────────────
        function updateTelemetry(d) {
            // --- Main tab ---
            set('val-speedL', d.speedL.toFixed(2) + '<span class="data-unit"> m/s</span>');
            set('val-speedR', d.speedR.toFixed(2) + '<span class="data-unit"> m/s</span>');
            set('val-gyro',   d.gyroZ.toFixed(1)  + '<span class="data-unit"> °/s</span>');
            setText('val-error',    d.lineErr.toFixed(0));
            setText('val-state',    d.state);
            if (d.encL !== undefined) setText('val-encL', d.encL);
            if (d.encR !== undefined) setText('val-encR', d.encR);
            if (d.gyroRaw  !== undefined) setText('val-gyroRaw',  d.gyroRaw.toFixed(2));
            if (d.gyroBias !== undefined) setText('val-gyroBias', d.gyroBias.toFixed(3));

            // --- Debug tab ---
            updateDebug(d);
        }

        function updateDebug(d) {
            // IMU status card
            const imuCard = document.getElementById('hw-imu');
            const imuText = document.getElementById('imu-state-text');
            const imuSub  = document.getElementById('imu-sub');
            const calWrap = document.getElementById('cal-bar-wrap');
            const calBar  = document.getElementById('cal-bar');

            if (!d.imuInit) {
                setHwCard('hw-imu', 'fail', 'NOT INITIALIZED');
                imuSub.innerText = 'Check SPI wiring and CS pin (GPIO14)';
                calWrap.style.display = 'none';
            } else if (d.imuCal) {
                setHwCard('hw-imu', 'busy', 'CALIBRATING…');
                imuSub.innerText = `Collecting bias samples — keep robot still`;
                calWrap.style.display = 'block';
                calBar.style.width = (d.calProgress || 0) + '%';
            } else {
                setHwCard('hw-imu', 'ok', 'OK — Running');
                imuSub.innerText = `Bias: ${d.gyroBias !== undefined ? d.gyroBias.toFixed(3) : '—'} °/s`;
                calWrap.style.display = 'none';
            }

            // IMU number table
            if (d.gyroRaw  !== undefined) setText('dbg-gyroRaw',  d.gyroRaw.toFixed(2));
            if (d.gyroBias !== undefined) setText('dbg-gyroBias', d.gyroBias.toFixed(3));
            if (d.gyroZ    !== undefined) setText('dbg-gyroZ',    d.gyroZ.toFixed(2));
            setText('dbg-imuInit', d.imuInit ? '✅ Yes' : '❌ No');
            setText('dbg-imuCal',  d.imuCal  ? '⏳ Yes' : 'No');
            setText('dbg-calPct',  d.calProgress !== undefined ? d.calProgress : '—');

            // Encoder counts
            const encL = d.encL !== undefined ? d.encL : 0;
            const encR = d.encR !== undefined ? d.encR : 0;
            const dL   = d.encLDelta !== undefined ? d.encLDelta : 0;
            const dR   = d.encRDelta !== undefined ? d.encRDelta : 0;

            setText('dbg-encL',      encL);
            setText('dbg-encR',      encR);
            setText('dbg-encLDelta', dL);
            setText('dbg-encRDelta', dR);

            // Detect if encoder is counting
            if (prevEncL !== null && encL !== prevEncL) encLMoved = true;
            if (prevEncR !== null && encR !== prevEncR) encRMoved = true;
            prevEncL = encL; prevEncR = encR;

            // Encoder card status: if never moved → warn, if moved → ok
            setHwCard('hw-enc1', encLMoved ? 'ok' : 'busy',
                      encLMoved ? 'COUNTING' : 'No pulses yet');
            setText('enc1-sub', `Count: ${encL} | Δ/tick: ${dL}`);

            setHwCard('hw-enc2', encRMoved ? 'ok' : 'busy',
                      encRMoved ? 'COUNTING' : 'No pulses yet');
            setText('enc2-sub', `Count: ${encR} | Δ/tick: ${dR}`);

            // Delta bars (centre-origin, ±5 counts maps to ±50%)
            drawDeltaBar('bar-encL', dL, 5);
            drawDeltaBar('bar-encR', dR, 5);

            // Speeds
            if (d.speedL !== undefined) setText('dbg-speedL', d.speedL.toFixed(3));
            if (d.speedR !== undefined) setText('dbg-speedR', d.speedR.toFixed(3));

            // Odometry
            if (d.odomX   !== undefined) setText('dbg-odomX',   d.odomX.toFixed(3));
            if (d.odomY   !== undefined) setText('dbg-odomY',   d.odomY.toFixed(3));
            if (d.odomYaw !== undefined) setText('dbg-odomYaw', d.odomYaw.toFixed(1));
        }

        // Draw a centre-anchored bar; maxVal maps to 50% width
        function drawDeltaBar(id, val, maxVal) {
            const bar = document.getElementById(id);
            const pct = Math.min(Math.abs(val) / maxVal * 50, 50);
            if (val >= 0) {
                bar.style.left  = '50%';
                bar.style.width = pct + '%';
            } else {
                bar.style.left  = (50 - pct) + '%';
                bar.style.width = pct + '%';
            }
        }

        function setHwCard(id, cls, text) {
            const card = document.getElementById(id);
            card.className = 'hw-card ' + cls;
            const txt = card.querySelector('[id$="-state-text"]');
            if (txt) txt.innerText = text;
        }

        function set(id, html) {
            const el = document.getElementById(id);
            if (el) el.innerHTML = html;
        }
        function setText(id, txt) {
            const el = document.getElementById(id);
            if (el) el.innerText = txt;
        }

        // ── Params update ──────────────────────────────────────────────────────
        function updateParams(d) {
            const pairs = [
                ['baseSpeed', 1], ['kp', 2], ['kd', 2], ['kpMult', 3], ['kdMult', 3]
            ];
            pairs.forEach(([k, dec]) => {
                if (d[k] === undefined) return;
                const sl = document.getElementById('sl-' + k);
                const dp = document.getElementById('disp-' + k);
                if (sl) sl.value = d[k];
                if (dp) dp.innerText = parseFloat(d[k]).toFixed(dec);
            });
        }

        // ── Controls ───────────────────────────────────────────────────────────
        function updateSlider(id) {
            const val = document.getElementById('sl-' + id).value;
            let dec = 2;
            if (id === 'baseSpeed') dec = 1;
            else if (id === 'kpMult' || id === 'kdMult') dec = 3;
            document.getElementById('disp-' + id).innerText = parseFloat(val).toFixed(dec);
        }

        function sendParam(id) {
            if (ws && ws.readyState === WebSocket.OPEN) {
                const val = parseFloat(document.getElementById('sl-' + id).value);
                ws.send(JSON.stringify({ type: 'set_param', param: id, value: val }));
            }
        }

        function sendCmd(type, action) {
            if (ws && ws.readyState === WebSocket.OPEN)
                ws.send(JSON.stringify({ type, action }));
        }

        function recalIMU() {
            if (ws && ws.readyState === WebSocket.OPEN)
                ws.send(JSON.stringify({ type: 'debug_cmd', action: 'recal_imu' }));
        }

        // ── Map viewer ─────────────────────────────────────────────────────────
        function fetchMap(url) {
            fetch(url)
                .then(r => { if (!r.ok) throw new Error("HTTP " + r.status); return r.text(); })
                .then(text => drawMap(text))
                .catch(e => alert("Could not fetch map: " + e.message));
        }

        function drawMap(text) {
            const lines = text.split('\n');
            let points = [];
            let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
            for (let line of lines) {
                line = line.trim();
                if (!line || line.startsWith('X')) continue;
                const parts = line.split(/[ \t]+/);
                if (parts.length >= 2) {
                    const x = parseFloat(parts[0]), y = parseFloat(parts[1]);
                    if (!isNaN(x) && !isNaN(y)) {
                        points.push({x, y});
                        if (x < minX) minX = x; if (x > maxX) maxX = x;
                        if (y < minY) minY = y; if (y > maxY) maxY = y;
                    }
                }
            }
            const canvas = document.getElementById('mapCanvas');
            const ctx = canvas.getContext('2d');
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            if (points.length === 0) {
                ctx.fillStyle = '#888'; ctx.font = '16px sans-serif'; ctx.textAlign = 'center';
                ctx.fillText("Map is empty or invalid", canvas.width/2, canvas.height/2);
                return;
            }
            const padding = 20;
            const w = maxX - minX || 1, h = maxY - minY || 1;
            const scale = Math.min((canvas.width - padding*2) / w, (canvas.height - padding*2) / h);
            const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
            ctx.strokeStyle = '#00ff9d'; ctx.lineWidth = 3;
            ctx.lineJoin = 'round'; ctx.lineCap = 'round';
            ctx.beginPath();
            for (let i = 0; i < points.length; i++) {
                const px = canvas.width/2  + (points[i].x - cx) * scale;
                const py = canvas.height/2 - (points[i].y - cy) * scale;
                if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
            }
            ctx.stroke();
            ctx.fillStyle = '#ff3366'; ctx.beginPath();
            const sx = canvas.width/2  + (points[0].x - cx) * scale;
            const sy = canvas.height/2 - (points[0].y - cy) * scale;
            ctx.arc(sx, sy, 6, 0, Math.PI*2); ctx.fill();
        }

        initWebSocket();
    </script>
</body>
</html>
)rawliteral";

#endif
