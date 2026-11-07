#ifndef INDEX_H
#define INDEX_H

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, viewport-fit=cover">
    <title>🤖 Robot Control</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        :root {
            --primary: #00ff88;
            --primary-dark: #00cc6a;
            --secondary: #00d4ff;
            --danger: #ff1744;
            --warning: #ffa600;
            --bg-dark: #0a0e27;
            --bg-darker: #050810;
            --surface: #1a1f3a;
            --surface-light: #252d4d;
            --text-primary: #e0e0e0;
            --text-secondary: #9ca3af;
            --border: #2d3748;
        }

        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, var(--bg-dark) 0%, var(--bg-darker) 100%);
            color: var(--text-primary);
            overflow: hidden;
            height: 100vh;
            display: flex;
            flex-direction: column;
        }

        /* STATUS BAR */
        .status-bar {
            background: var(--surface);
            border-bottom: 2px solid var(--border);
            padding: 12px 16px;
            display: grid;
            grid-template-columns: 1fr 1fr 1fr 1fr;
            gap: 12px;
            font-size: 12px;
            flex-shrink: 0;
        }

        .status-item {
            display: flex;
            align-items: center;
            gap: 8px;
            padding: 8px;
            background: var(--bg-dark);
            border: 1px solid var(--border);
            border-radius: 8px;
            min-height: 40px;
        }

        .status-indicator {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            animation: pulse 2s infinite;
        }

        .status-indicator.connected {
            background: var(--primary);
            box-shadow: 0 0 10px var(--primary);
        }

        .status-indicator.disconnected {
            background: var(--danger);
        }

        .status-indicator.warning {
            background: var(--warning);
            animation: pulse-warning 1s infinite;
        }

        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }

        @keyframes pulse-warning {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.7; }
        }

        .status-text {
            display: flex;
            flex-direction: column;
            gap: 2px;
        }

        .status-label {
            font-size: 10px;
            color: var(--text-secondary);
            text-transform: uppercase;
        }

        .status-value {
            font-weight: 600;
            color: var(--text-primary);
        }

        /* MAIN CONTAINER */
        .container {
            flex: 1;
            display: flex;
            flex-direction: column;
            overflow-y: auto;
            padding: 16px;
            gap: 16px;
        }

        /* JOYSTICK AREA */
        .joystick-container {
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 220px;
            touch-action: none;
        }

        .joystick {
            position: relative;
            width: 180px;
            height: 180px;
            background: radial-gradient(circle at 30% 30%, var(--surface-light), var(--surface));
            border: 3px solid var(--secondary);
            border-radius: 50%;
            box-shadow: 0 0 30px rgba(0, 212, 255, 0.3), inset 0 0 20px rgba(0, 0, 0, 0.5);
            display: flex;
            justify-content: center;
            align-items: center;
            cursor: grab;
        }

        .joystick:active {
            cursor: grabbing;
        }

        .joystick-handle {
            position: absolute;
            width: 60px;
            height: 60px;
            background: radial-gradient(circle at 35% 35%, var(--primary), var(--primary-dark));
            border: 2px solid var(--primary);
            border-radius: 50%;
            box-shadow: 0 0 20px rgba(0, 255, 136, 0.5);
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            transition: all 0.05s linear;
            pointer-events: none;
            z-index: 10;
        }

        .joystick-grid {
            position: absolute;
            width: 100%;
            height: 100%;
            opacity: 0.1;
            pointer-events: none;
        }

        .joystick-grid line {
            stroke: var(--text-secondary);
            stroke-width: 1;
        }

        /* CONTROLS SECTION */
        .controls-section {
            display: flex;
            gap: 12px;
            flex-wrap: wrap;
            justify-content: center;
            flex-shrink: 0;
        }

        .btn {
            padding: 12px 20px;
            border: 2px solid var(--border);
            border-radius: 8px;
            background: var(--surface);
            color: var(--text-primary);
            cursor: pointer;
            font-weight: 600;
            font-size: 13px;
            transition: all 0.2s;
            text-transform: uppercase;
            letter-spacing: 0.5px;
            min-width: 90px;
        }

        .btn:active {
            transform: scale(0.95);
        }

        .btn.primary {
            background: var(--primary);
            color: var(--bg-dark);
            border-color: var(--primary);
        }

        .btn.primary:hover {
            box-shadow: 0 0 20px rgba(0, 255, 136, 0.5);
        }

        .btn.danger {
            background: var(--danger);
            color: white;
            border-color: var(--danger);
        }

        .btn.danger:hover {
            box-shadow: 0 0 20px rgba(255, 23, 68, 0.5);
        }

        .btn.secondary {
            background: var(--secondary);
            color: var(--bg-dark);
            border-color: var(--secondary);
        }

        .btn.secondary:hover {
            box-shadow: 0 0 20px rgba(0, 212, 255, 0.5);
        }

        /* SETTINGS PANEL */
        .settings-panel {
            background: var(--surface);
            border: 2px solid var(--border);
            border-radius: 12px;
            padding: 16px;
            flex-shrink: 0;
        }

        .settings-title {
            font-size: 13px;
            font-weight: 700;
            text-transform: uppercase;
            color: var(--primary);
            margin-bottom: 12px;
            letter-spacing: 1px;
        }

        .setting-group {
            margin-bottom: 16px;
        }

        .setting-label {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 8px;
            font-size: 12px;
        }

        .setting-label-text {
            color: var(--text-secondary);
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }

        .setting-value {
            font-weight: 700;
            color: var(--primary);
            font-size: 14px;
        }

        .slider {
            width: 100%;
            height: 6px;
            border-radius: 3px;
            background: var(--bg-dark);
            outline: none;
            -webkit-appearance: none;
            appearance: none;
        }

        .slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 18px;
            height: 18px;
            border-radius: 50%;
            background: var(--primary);
            cursor: pointer;
            box-shadow: 0 0 10px rgba(0, 255, 136, 0.5);
            border: 2px solid var(--primary);
        }

        .toggle {
            display: inline-flex;
            align-items: center;
            gap: 8px;
            padding: 8px 12px;
            background: var(--bg-dark);
            border: 2px solid var(--border);
            border-radius: 8px;
            cursor: pointer;
            font-size: 12px;
            font-weight: 600;
        }

        .toggle.active {
            border-color: var(--primary);
            background: rgba(0, 255, 136, 0.1);
            color: var(--primary);
        }

        .toggle-dot {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            background: var(--text-secondary);
            transition: all 0.3s;
        }

        .toggle.active .toggle-dot {
            background: var(--primary);
            box-shadow: 0 0 8px var(--primary);
        }

        /* MODES */
        .modes-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 8px;
            margin-bottom: 12px;
        }

        .mode-btn {
            padding: 10px;
            font-size: 11px;
            background: var(--bg-dark);
            border: 2px solid var(--border);
            border-radius: 8px;
            color: var(--text-secondary);
            cursor: pointer;
            transition: all 0.2s;
        }

        .mode-btn.active {
            background: var(--primary);
            color: var(--bg-dark);
            border-color: var(--primary);
            box-shadow: 0 0 15px rgba(0, 255, 136, 0.5);
        }

        /* BIGGER INFO LOG TERMINAL */
        .info-log {
            background: var(--bg-dark);
            border: 1px solid var(--border);
            border-radius: 8px;
            padding: 12px;
            font-size: 12px;
            color: var(--text-secondary);
            height: 160px;
            max-height: 160px;
            overflow-y: auto;
            font-family: 'Courier New', monospace;
            flex-shrink: 0;
        }

        .log-entry {
            margin-bottom: 4px;
            word-break: break-all;
        }

        .log-entry.success { color: var(--primary); }
        .log-entry.error { color: var(--danger); }
        .log-entry.warning { color: var(--warning); }
    </style>
</head>
<body>
    <!-- STATUS BAR -->
    <div class="status-bar">
        <div class="status-item">
            <div class="status-indicator disconnected" id="wifiIndicator"></div>
            <div class="status-text">
                <span class="status-label">WiFi</span>
                <span class="status-value" id="wifiStatus">Offline</span>
            </div>
        </div>
        <div class="status-item">
            <div class="status-indicator" id="batteryIndicator"></div>
            <div class="status-text">
                <span class="status-label">Battery</span>
                <span class="status-value" id="batteryValue">5.0V</span>
            </div>
        </div>
        <div class="status-item">
            <div class="status-indicator warning" id="distanceIndicator"></div>
            <div class="status-text">
                <span class="status-label">Distance</span>
                <span class="status-value" id="distanceValue">--cm</span>
            </div>
        </div>
        <div class="status-item">
            <div style="width: 12px; height: 12px; border-radius: 50%; background: var(--secondary);"></div>
            <div class="status-text">
                <span class="status-label">State</span>
                <span class="status-value" id="robotState">Idle</span>
            </div>
        </div>
    </div>

    <!-- MAIN CONTAINER -->
    <div class="container">
        <!-- JOYSTICK -->
        <div class="joystick-container">
            <div class="joystick" id="joystick">
                <svg class="joystick-grid" viewBox="0 0 200 200">
                    <line x1="100" y1="0" x2="100" y2="200" stroke="white" stroke-width="1"/>
                    <line x1="0" y1="100" x2="200" y2="100" stroke="white" stroke-width="1"/>
                    <circle cx="100" cy="100" r="50" fill="none" stroke="white" stroke-width="1"/>
                    <circle cx="100" cy="100" r="100" fill="none" stroke="white" stroke-width="1"/>
                </svg>
                <div class="joystick-handle" id="joystickHandle"></div>
            </div>
        </div>

        <!-- CONTROL BUTTONS -->
        <div class="controls-section">
            <button class="btn primary" id="startBtn">▶ Start</button>
            <button class="btn secondary" id="pauseBtn">⏸ Pause</button>
            <button class="btn danger" id="stopBtn">⏹ Stop</button>
        </div>

        <!-- MODES & CONTROLS -->
        <div class="settings-panel">
            <div class="settings-title">🎮 Modes</div>
            <div class="modes-grid">
                <button class="mode-btn active" data-mode="manual">Manual</button>
                <button class="mode-btn" data-mode="line">Line Follow</button>
                <button class="mode-btn" data-mode="circle">Circle Demo</button>
                <button class="mode-btn" data-mode="square">Square Demo</button>
                <button class="mode-btn" data-mode="spiral">Spiral Demo</button>
                <button class="mode-btn" data-mode="test">Test Demo</button>
            </div>

            <div class="settings-title">⚙️ Settings</div>

            <div class="setting-group">
                <div class="setting-label">
                    <span class="setting-label-text">Base Speed</span>
                    <span class="setting-value" id="baseSpeedValue">95</span>
                </div>
                <input type="range" class="slider" id="baseSpeedSlider" min="50" max="200" value="95">
            </div>

            <div class="setting-group">
                <div class="setting-label">
                    <span class="setting-label-text">Pivot Speed</span>
                    <span class="setting-value" id="pivotSpeedValue">120</span>
                </div>
                <input type="range" class="slider" id="pivotSpeedSlider" min="80" max="200" value="120">
            </div>

            <div class="setting-group">
                <div class="setting-label">
                    <span class="setting-label-text">Obstacle Threshold (cm)</span>
                    <span class="setting-value" id="obstacleThreshValue">20</span>
                </div>
                <input type="range" class="slider" id="obstacleThreshSlider" min="10" max="40" value="20">
            </div>

            <div class="setting-group">
                <div class="setting-label">
                    <span class="setting-label-text">IR Sensors (Autonomous Line Follow)</span>
                </div>
                <div class="toggle" id="irToggle">
                    <span>OFF</span>
                    <div class="toggle-dot"></div>
                </div>
            </div>
        </div>

        <!-- BIGGER TERMINAL LOG -->
        <div class="info-log" id="infoLog"></div>
    </div>

    <script>
        const state = {
            robotIP: localStorage.getItem('robotIP') || 'robo.local',
            isConnected: false,
            isRunning: false,
            currentMode: 'manual',
            irEnabled: JSON.parse(localStorage.getItem('irEnabled') || 'false'),
            baseSpeed: parseInt(localStorage.getItem('baseSpeed') || '95'),
            pivotSpeed: parseInt(localStorage.getItem('pivotSpeed') || '120'),
            obstacleThreshold: parseInt(localStorage.getItem('obstacleThreshold') || '20'),
            telemetry: { distance: 0, battery: 5.0, state: 'Idle' }
        };

        let socket = null;
        let joystickX = 0, joystickY = 0;
        let isJoystickTouching = false;

        function log(message, type = 'info') {
            const logEl = document.getElementById('infoLog');
            const entry = document.createElement('div');
            entry.className = `log-entry ${type}`;
            entry.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
            logEl.appendChild(entry);
            logEl.scrollTop = logEl.scrollHeight;
            if (logEl.children.length > 25) { logEl.removeChild(logEl.firstChild); }
        }

        function initWebSocket() {
            try {
                const host = window.location.hostname || state.robotIP;
                log(`Connecting to ws://${host}:81/`, 'warning');
                socket = new WebSocket(`ws://${host}:81/`);

                socket.onopen = () => {
                    state.isConnected = true;
                    updateStatusBar();
                    log('✓ Connected to robot!', 'success');
                    sendSettings();
                };

                socket.onmessage = (event) => {
                    try {
                        const data = JSON.parse(event.data);
                        state.telemetry = { ...state.telemetry, ...data };
                        updateStatusBar();
                    } catch (e) {}
                };

                socket.onclose = () => {
                    state.isConnected = false;
                    updateStatusBar();
                    log('✗ Disconnected from robot', 'error');
                    setTimeout(initWebSocket, 3000);
                };

                socket.onerror = (err) => { log('WebSocket error', 'error'); };
            } catch (e) { log('Connection failed', 'error'); }
        }

        function sendCommand(cmd) {
            if (state.isConnected && socket && socket.readyState === WebSocket.OPEN) {
                socket.send(JSON.stringify(cmd));
            }
        }

        function sendSettings() {
            sendCommand({
                type: 'settings',
                baseSpeed: state.baseSpeed,
                pivotSpeed: state.pivotSpeed,
                obstacleThreshold: state.obstacleThreshold,
                irEnabled: state.irEnabled,
                mode: state.currentMode,
                isRunning: state.isRunning
            });
        }

        // JOYSTICK
        const joystickEl = document.getElementById('joystick');
        const handleEl = document.getElementById('joystickHandle');

        function updateJoystick(x, y) {
            joystickX = Math.max(-100, Math.min(100, x));
            joystickY = Math.max(-100, Math.min(100, y));

            const distance = Math.sqrt(joystickX ** 2 + joystickY ** 2);
            if (distance > 100) {
                const ratio = 100 / distance;
                joystickX *= ratio;
                joystickY *= ratio;
            }

            handleEl.style.transform = `translate(calc(-50% + ${joystickX}px), calc(-50% + ${joystickY}px))`;

            // Normalized -1 to 1
            sendCommand({
                type: 'joystick',
                x: (joystickX / 100),
                y: (joystickY / 100) // Up is negative, Down is positive
            });
        }

        joystickEl.addEventListener('touchstart', (e) => {
            isJoystickTouching = true;
            const rect = joystickEl.getBoundingClientRect();
            const touch = e.touches[0];
            updateJoystick(touch.clientX - rect.left - rect.width / 2, touch.clientY - rect.top - rect.height / 2);
        });

        document.addEventListener('touchmove', (e) => {
            if (isJoystickTouching) {
                const rect = joystickEl.getBoundingClientRect();
                const touch = e.touches[0];
                updateJoystick(touch.clientX - rect.left - rect.width / 2, touch.clientY - rect.top - rect.height / 2);
            }
        });

        document.addEventListener('touchend', () => {
            if (isJoystickTouching) {
                isJoystickTouching = false;
                handleEl.style.transform = 'translate(-50%, -50%)';
                joystickX = 0; joystickY = 0;
                sendCommand({ type: 'joystick', x: 0, y: 0 });
            }
        });

        // MOUSE SUPPORT FOR DESKTOP JOYSTICK
        let isMouseDown = false;
        joystickEl.addEventListener('mousedown', (e) => {
            isMouseDown = true;
            const rect = joystickEl.getBoundingClientRect();
            updateJoystick(e.clientX - rect.left - rect.width / 2, e.clientY - rect.top - rect.height / 2);
        });
        window.addEventListener('mousemove', (e) => {
            if (isMouseDown) {
                const rect = joystickEl.getBoundingClientRect();
                updateJoystick(e.clientX - rect.left - rect.width / 2, e.clientY - rect.top - rect.height / 2);
            }
        });
        window.addEventListener('mouseup', () => {
            if (isMouseDown) {
                isMouseDown = false;
                handleEl.style.transform = 'translate(-50%, -50%)';
                joystickX = 0; joystickY = 0;
                sendCommand({ type: 'joystick', x: 0, y: 0 });
            }
        });

        // MODES
        document.querySelectorAll('.mode-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
                btn.classList.add('active');
                state.currentMode = btn.dataset.mode;
                log(`Mode Selected: ${state.currentMode.toUpperCase()}`, 'success');
                sendSettings();
            });
        });

        // SLIDERS & TOGGLES
        document.getElementById('baseSpeedSlider').addEventListener('input', (e) => {
            state.baseSpeed = parseInt(e.target.value);
            document.getElementById('baseSpeedValue').textContent = state.baseSpeed;
            localStorage.setItem('baseSpeed', state.baseSpeed);
            sendSettings();
        });

        document.getElementById('pivotSpeedSlider').addEventListener('input', (e) => {
            state.pivotSpeed = parseInt(e.target.value);
            document.getElementById('pivotSpeedValue').textContent = state.pivotSpeed;
            localStorage.setItem('pivotSpeed', state.pivotSpeed);
            sendSettings();
        });

        document.getElementById('obstacleThreshSlider').addEventListener('input', (e) => {
            state.obstacleThreshold = parseInt(e.target.value);
            document.getElementById('obstacleThreshValue').textContent = state.obstacleThreshold;
            localStorage.setItem('obstacleThreshold', state.obstacleThreshold);
            sendSettings();
        });

        document.getElementById('irToggle').addEventListener('click', () => {
            state.irEnabled = !state.irEnabled;
            document.getElementById('irToggle').classList.toggle('active');
            document.getElementById('irToggle').querySelector('span').textContent = state.irEnabled ? 'ON' : 'OFF';
            localStorage.setItem('irEnabled', state.irEnabled);
            sendSettings();
            log(`IR Sensors: ${state.irEnabled ? 'ON' : 'OFF'}`, 'success');
        });

        // CONTROL BUTTONS
        document.getElementById('startBtn').addEventListener('click', () => {
            state.isRunning = true;
            log('Robot STARTED', 'success');
            sendSettings();
        });

        document.getElementById('pauseBtn').addEventListener('click', () => {
            state.isRunning = false;
            log('Robot PAUSED', 'warning');
            sendSettings();
        });

        document.getElementById('stopBtn').addEventListener('click', () => {
            state.isRunning = false;
            state.currentMode = 'manual';
            document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
            document.querySelector('[data-mode="manual"]').classList.add('active');
            log('Robot STOPPED', 'error');
            sendSettings();
        });

        function updateStatusBar() {
            const wifiIndicator = document.getElementById('wifiIndicator');
            const wifiStatus = document.getElementById('wifiStatus');
            if (state.isConnected) {
                wifiIndicator.className = 'status-indicator connected';
                wifiStatus.textContent = 'Online';
            } else {
                wifiIndicator.className = 'status-indicator disconnected';
                wifiStatus.textContent = 'Offline';
            }
            document.getElementById('batteryValue').textContent = state.telemetry.battery.toFixed(1) + 'V';
            document.getElementById('distanceValue').textContent = state.telemetry.distance + 'cm';
            document.getElementById('robotState').textContent = state.telemetry.state || 'Idle';

            const distIndicator = document.getElementById('distanceIndicator');
            if (state.telemetry.distance < state.obstacleThreshold && state.telemetry.distance > 0) {
                distIndicator.className = 'status-indicator danger';
            } else {
                distIndicator.className = 'status-indicator warning';
            }
        }

        function init() {
            document.getElementById('baseSpeedSlider').value = state.baseSpeed;
            document.getElementById('baseSpeedValue').textContent = state.baseSpeed;
            document.getElementById('pivotSpeedSlider').value = state.pivotSpeed;
            document.getElementById('pivotSpeedValue').textContent = state.pivotSpeed;
            document.getElementById('obstacleThreshSlider').value = state.obstacleThreshold;
            document.getElementById('obstacleThreshValue').textContent = state.obstacleThreshold;
            if (state.irEnabled) {
                document.getElementById('irToggle').classList.add('active');
                document.getElementById('irToggle').querySelector('span').textContent = 'ON';
            }
            updateStatusBar();
            log('🤖 Robot Control Ready!', 'success');
            initWebSocket();
        }

        document.addEventListener('DOMContentLoaded', init);
    </script>
</body>
</html>
)rawliteral";

#endif