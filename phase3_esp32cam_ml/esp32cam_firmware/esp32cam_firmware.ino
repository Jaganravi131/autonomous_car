/*
 * ============================================================================
 * PHASE 3 (PART B): ESP32-CAM FIRMWARE — VIDEO STREAMER + ML COMMAND BRIDGE
 * ============================================================================
 * Board Selection in Arduino IDE:
 *   - Board: "AI Thinker ESP32-CAM" (under Tools -> Board -> ESP32 Arduino)
 *   - Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"
 *
 * Wiring to Buck Converter & Arduino Uno (Matches full_wiring_schematic.png):
 *   - ESP32-CAM 5V            <- Buck Converter OUT+ (5.0V DC)
 *   - ESP32-CAM GND           <- Common GND
 *   - ESP32-CAM U0T (GPIO1)   -> Arduino Uno Pin A2 (SoftwareSerial RX)
 *   - ESP32-CAM IO13          <- Optional: Arduino Uno Pin A3 (via 1k/2k divider)
 *
 * Endpoints Provided by ESP32-CAM:
 *   - http://<ESP_IP>/         : Interactive Web Control Dashboard (WASD + Buttons)
 *   - http://<ESP_IP>:81/stream: Live MJPEG Video Stream (for Python ML & Browser)
 *   - http://<ESP_IP>/capture  : Single JPEG Frame Snapshot
 *   - http://<ESP_IP>/cmd?c=F  : Send Command ('F','G','I','L','R','B','S','1'-'9','A','C')
 *                                immediately over UART (9600 baud) to Arduino Uno!
 * ============================================================================
 */

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_http_server.h"

// -------------------- WI-FI CONFIGURATION --------------------------
// Enter your Wi-Fi / Phone Hotspot credentials here.
// If connection fails after 10 seconds, ESP32-CAM automatically starts its own
// Wi-Fi Access Point: SSID = "AutonomousCar-CAM", Password = "12345678" (IP: 192.168.4.1)
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

const char* AP_SSID   = "AutonomousCar-CAM";
const char* AP_PASS   = "12345678";

// -------------------- AI-THINKER ESP32-CAM PINOUT ------------------
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_LED_PIN      4

// -------------------- TELEMETRY LINK FROM THE ARDUINO UNO -------------------
// Arduino A3 (5V TX) --[1k ohm]--> ESP32-CAM IO13,  and 2k ohm IO13 -> GND
// UART1 is remapped onto IO13 so that UART0 (GPIO1/GPIO3) stays free for
// flashing and for sending commands.  IO13/IO14 are the pins shared with the
// microSD slot - this project does not use the SD card.
#define LINK_RX_PIN       13     // set -1 if you did not wire the A3 telemetry line
static HardwareSerial LinkSerial(1);

// -------------------- MJPEG STREAM BOUNDARY ------------------------
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t cmd_httpd    = NULL;
httpd_handle_t stream_httpd = NULL;

// -------------------- LIVE TELEMETRY CACHE (from Arduino) ------------------
// Line received from the Uno looks like:  #D:87,L:1,R:0,M:2,V:165\n
struct Telemetry {
  int          dist  = -1;    // cm from HC-SR04
  int          irL   = -1;    // 1 = obstacle seen
  int          irR   = -1;
  int          mode  = -1;    // 0 STOP, 1 MANUAL, 2 AUTO, 3 CAMERA/ML
  int          speed = -1;
  unsigned long lastMs = 0;   // when the last line arrived
};
static Telemetry tele;
static char    lineBuf[48];
static uint8_t lineLen = 0;

// -------------------- LIVE ML PREDICTIONS (pushed by the laptop) ----------
// The ML model runs on the laptop, not on the ESP32. While
// 3_run_autonomous_ml.py is driving the car it can report every prediction to
// this dashboard with:   GET /ml?p=F&c=94
// so you can watch what the model is thinking in real time.
struct MlState {
  char          pred[8]  = "";
  int           conf     = 0;    // 0..100
  unsigned long lastMs   = 0;
};
static MlState ml;

static bool mlIsFresh() {
  return (ml.lastMs != 0) && (millis() - ml.lastMs < 3000);
}

static void parseTelemetryLine(const char *s) {
  if (s[0] != '#') return;
  int d, l, r, m, v;
  if (sscanf(s, "#D:%d,L:%d,R:%d,M:%d,V:%d", &d, &l, &r, &m, &v) == 5) {
    tele.dist   = d;
    tele.irL    = l;
    tele.irR    = r;
    tele.mode   = m;
    tele.speed  = v;
    tele.lastMs = millis();
  }
}

static void pollTelemetry() {
  if (LINK_RX_PIN < 0) return;
  while (LinkSerial.available()) {
    char c = (char)LinkSerial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen) {
        lineBuf[lineLen] = '\0';
        parseTelemetryLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

// -------------------- BUILT-IN WEB DASHBOARD HTML ------------------
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta name="theme-color" content="#0b1020">
<title>4WD Autonomous Car — Control</title>
<style>
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;padding:0}
body{
  font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;
  background:
    radial-gradient(1100px 520px at 12% -8%,#1d2b64 0%,transparent 55%),
    radial-gradient(900px 500px at 105% 8%,#3a1c71 0%,transparent 50%),
    #0b1020;
  color:#e6edf7;min-height:100vh;padding:16px 12px 34px;
}
.app{max-width:560px;margin:0 auto;display:flex;flex-direction:column;gap:14px}

/* ---------- header ---------- */
header{display:flex;align-items:center;justify-content:space-between;gap:10px;padding:2px 4px}
.brand{display:flex;align-items:center;gap:10px;min-width:0}
.logo{width:38px;height:38px;border-radius:11px;flex:0 0 auto;display:grid;place-items:center;
  background:linear-gradient(135deg,#22d3ee,#8b5cf6);box-shadow:0 6px 18px #22d3ee40;font-size:19px}
h1{font-size:16px;margin:0;letter-spacing:.4px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.sub{font-size:11px;color:#8b9bb4;margin-top:2px}
.conn{display:flex;align-items:center;gap:6px;font-size:11px;color:#8b9bb4;flex:0 0 auto}
.dot{width:9px;height:9px;border-radius:50%;background:#64748b;box-shadow:0 0 0 0 #22c55e66}
.dot.on{background:#22c55e;animation:pulse 1.8s infinite}
.dot.off{background:#ef4444}
@keyframes pulse{0%{box-shadow:0 0 0 0 #22c55e88}70%{box-shadow:0 0 0 8px #22c55e00}100%{box-shadow:0 0 0 0 #22c55e00}}

/* ---------- cards ---------- */
.card{background:linear-gradient(180deg,#ffffff0d,#ffffff05);border:1px solid #ffffff14;
  border-radius:16px;padding:13px;backdrop-filter:blur(8px);box-shadow:0 10px 26px #00000045}
.card-title{font-size:10.5px;letter-spacing:1.4px;color:#8b9bb4;font-weight:700;
  margin:0 0 10px 2px;display:flex;align-items:center;gap:7px}
.card-title .tag{margin-left:auto;font-weight:600;letter-spacing:.3px;color:#22d3ee;
  background:#22d3ee14;border:1px solid #22d3ee33;border-radius:20px;padding:2px 8px;font-size:9.5px}

/* ---------- video ---------- */
.frame-wrap{position:relative;border-radius:12px;overflow:hidden;background:#05070f;
  border:1px solid #ffffff17;aspect-ratio:4/3}
#cam{width:100%;height:100%;object-fit:cover;display:block;background:#05070f}
.badge{position:absolute;font-size:9.5px;font-weight:800;letter-spacing:.7px;padding:4px 8px;
  border-radius:20px;background:#000000a8;backdrop-filter:blur(4px);color:#e6edf7;border:1px solid #ffffff1f}
.badge.live{top:8px;left:8px;color:#fecdd3;display:flex;align-items:center;gap:5px}
.badge.live i{width:6px;height:6px;border-radius:50%;background:#ef4444;animation:blink 1.3s infinite}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.25}}
.badge.res{top:8px;right:8px;color:#a5b4fc}
.badge.ip{bottom:8px;left:8px;right:8px;color:#94a3b8;font-weight:600;letter-spacing:.2px;
  text-align:center;font-size:9px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}

/* ---------- telemetry ---------- */
.tele-grid{display:grid;grid-template-columns:1fr 1fr;gap:9px}
.stat{background:#0f172ab8;border:1px solid #ffffff10;border-radius:12px;padding:9px 10px}
.stat .k{font-size:9.5px;color:#8b9bb4;letter-spacing:.8px;font-weight:700}
.stat .v{font-size:19px;font-weight:800;margin-top:3px;letter-spacing:.5px}
.stat .v small{font-size:10px;color:#8b9bb4;font-weight:600;letter-spacing:0}
.bar{height:5px;border-radius:4px;background:#ffffff12;margin-top:7px;overflow:hidden}
.bar span{display:block;height:100%;border-radius:4px;transition:width .25s ease}
.pill{display:inline-flex;align-items:center;gap:5px;padding:2px 8px;border-radius:20px;
  font-size:9.5px;font-weight:800;letter-spacing:.5px}
.pill.clear{background:#22c55e1f;color:#4ade80;border:1px solid #22c55e44}
.pill.hit{background:#ef44441f;color:#fca5a5;border:1px solid #ef444455}
.wide{grid-column:1/-1}
#tele-warn{font-size:11.5px;color:#94a3b8;line-height:1.6}
#tele-warn b{color:#a5b4fc}
kbd{background:#ffffff14;border:1px solid #ffffff22;border-bottom-width:2px;border-radius:5px;
  padding:1px 5px;font-size:10px;font-family:ui-monospace,Menlo,monospace;color:#cbd5e1}

/* ---------- modes ---------- */
.modes{display:grid;grid-template-columns:1fr 1fr;gap:9px}
.mode{position:relative;background:#0f172ab8;border:1.5px solid #ffffff16;color:#cbd5e1;
  border-radius:13px;padding:11px 10px;cursor:pointer;text-align:left;
  font-family:inherit;transition:transform .08s,border-color .18s,background .18s}
.mode:active{transform:scale(.975)}
.mode .ic{font-size:16px;display:block;margin-bottom:3px}
.mode .nm{font-size:12.5px;font-weight:800;letter-spacing:.3px;color:#e6edf7}
.mode .ds{font-size:9.5px;color:#8b9bb4;margin-top:2px;line-height:1.35}
.mode .key{position:absolute;top:8px;right:9px;font-size:9px;font-weight:800;color:#64748b}
.mode.active{border-color:#22c55e;background:linear-gradient(180deg,#22c55e26,#22c55e10);
  box-shadow:0 0 0 3px #22c55e1f,0 8px 20px #22c55e14}
.mode.active .nm{color:#4ade80}
.mode.active .key{color:#4ade80}
.mode.danger{border-color:#ef444455}
.mode.danger .nm{color:#fca5a5}
.mode.danger.active{border-color:#ef4444;background:linear-gradient(180deg,#ef444426,#ef444410);
  box-shadow:0 0 0 3px #ef44441f}

/* ---------- dpad ---------- */
.dpad{display:grid;grid-template-columns:repeat(3,1fr);gap:9px}
.pad{background:linear-gradient(180deg,#1e293b,#0f172a);border:1.5px solid #ffffff16;color:#e6edf7;
  border-radius:13px;padding:15px 4px;cursor:pointer;font-family:inherit;
  display:flex;flex-direction:column;align-items:center;gap:3px;
  transition:transform .08s,border-color .15s,background .15s;user-select:none}
.pad:active,.pad.held{transform:scale(.94);border-color:#22d3ee;background:linear-gradient(180deg,#0891b2,#0e7490)}
.pad .ar{font-size:17px;line-height:1}
.pad .lb{font-size:9.5px;font-weight:700;letter-spacing:.4px;color:#94a3b8}
.pad.stop{border-color:#ef444466;background:linear-gradient(180deg,#7f1d1d,#450a0a)}
.pad.stop:active,.pad.stop.held{border-color:#ef4444;background:linear-gradient(180deg,#dc2626,#991b1b)}
.pad.stop .lb{color:#fca5a5}
.pad.full{grid-column:1/-1}

/* ---------- speed ---------- */
.speed{display:flex;align-items:center;gap:11px}
.speed input{flex:1;accent-color:#22d3ee;height:5px}
.speed .sv{font-variant-numeric:tabular-nums;font-weight:800;font-size:14px;min-width:34px;
  text-align:right;color:#22d3ee}

/* ---------- ml ---------- */
.ml-row{display:flex;align-items:center;gap:12px}
.ml-letter{width:52px;height:52px;flex:0 0 auto;border-radius:14px;display:grid;place-items:center;
  font-size:23px;font-weight:900;letter-spacing:.5px;
  background:linear-gradient(135deg,#8b5cf6,#22d3ee);box-shadow:0 8px 20px #8b5cf640;color:#fff}
.ml-meta{flex:1;min-width:0}
.ml-cmd{font-size:12.5px;font-weight:700;color:#e6edf7;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.ml-conf{font-size:10px;color:#8b9bb4;margin-top:1px}
.bar.ml{height:7px;margin-top:7px}
.bar.ml span{background:linear-gradient(90deg,#22d3ee,#8b5cf6)}
.ml-idle{font-size:11.5px;color:#8b9bb4;line-height:1.6}
.ml-idle code{background:#ffffff14;padding:1px 5px;border-radius:4px;font-size:10.5px;color:#a5b4fc}

/* ---------- dev / footer ---------- */
details.dev summary{cursor:pointer;font-size:10.5px;letter-spacing:1.2px;color:#8b9bb4;font-weight:700;
  list-style:none;display:flex;align-items:center;gap:6px}
details.dev summary::-webkit-details-marker{display:none}
details.dev summary::before{content:"▸";color:#22d3ee;transition:transform .2s;display:inline-block}
details.dev[open] summary::before{transform:rotate(90deg)}
.api{margin-top:10px;font-size:10px;color:#94a3b8;line-height:1.85;
  font-family:ui-monospace,SFMono-Regular,Menlo,monospace;word-break:break-all}
.api b{color:#22d3ee;font-weight:700}
.hint{font-size:10.5px;color:#64748b;line-height:1.6;margin-top:9px}
footer{text-align:center;font-size:10px;color:#64748b;line-height:1.7;padding-top:2px}
footer .keys{display:flex;gap:5px;justify-content:center;flex-wrap:wrap;margin-bottom:6px}
</style>
</head>
<body>
<div class="app">

  <header>
    <div class="brand">
      <div class="logo">🚗</div>
      <div>
        <h1>4WD AUTONOMOUS CAR</h1>
        <div class="sub">ESP32-CAM · OV2640 · Arduino Uno</div>
      </div>
    </div>
    <div class="conn"><span class="dot" id="dot"></span><span id="conn-txt">…</span></div>
  </header>

  <!-- ============ VIDEO ============ -->
  <section class="card">
    <div class="frame-wrap">
      <img id="cam" src="" alt="camera stream">
      <div class="badge live"><i></i>LIVE</div>
      <div class="badge res" id="res">320×240</div>
      <div class="badge ip" id="ip">connecting…</div>
    </div>
  </section>

  <!-- ============ TELEMETRY ============ -->
  <section class="card">
    <div class="card-title">TELEMETRY <span class="tag" id="mode-tag">--</span></div>
    <div id="tele-body">
      <div id="tele-warn">Waiting for the Arduino…<br>
        Wire <b>A3 →[1kΩ]→ IO13</b> (+2kΩ to GND) for live sensor data.
      </div>
    </div>
  </section>

  <!-- ============ DRIVING MODE ============ -->
  <section class="card">
    <div class="card-title">DRIVING MODE</div>
    <div class="modes">
      <button class="mode" id="m-m" onclick="send('M')">
        <span class="key">M</span><span class="ic">🕹️</span>
        <span class="nm">MANUAL</span><span class="ds">You drive with the pad below</span></button>
      <button class="mode" id="m-a" onclick="send('A')">
        <span class="key">A</span><span class="ic">🤖</span>
        <span class="nm">AUTO</span><span class="ds">Servo radar + ultrasonic + IR</span></button>
      <button class="mode" id="m-c" onclick="send('C')">
        <span class="key">C</span><span class="ic">🧠</span>
        <span class="nm">CAMERA / ML</span><span class="ds">Laptop model drives the car</span></button>
      <button class="mode danger" id="m-s" onclick="send('S')">
        <span class="key">S</span><span class="ic">🛑</span>
        <span class="nm">STOP</span><span class="ds">Cut power to all 4 motors</span></button>
    </div>
  </section>

  <!-- ============ MACHINE LEARNING ============ -->
  <section class="card">
    <div class="card-title">MACHINE LEARNING <span class="tag" id="ml-tag">idle</span></div>
    <div id="ml-body">
      <div class="ml-idle">No model running. Start the laptop autopilot to watch predictions live:<br>
        <code>python3 3_run_autonomous_ml.py --ip &lt;IP&gt; --mode hybrid</code>
      </div>
    </div>
  </section>

  <!-- ============ MOVEMENT ============ -->
  <section class="card">
    <div class="card-title">MOVEMENT</div>
    <div class="dpad">
      <button class="pad" data-cmd="G"><span class="ar">↖</span><span class="lb">CURVE L</span></button>
      <button class="pad" data-cmd="F"><span class="ar">▲</span><span class="lb">FORWARD</span></button>
      <button class="pad" data-cmd="I"><span class="ar">↗</span><span class="lb">CURVE R</span></button>
      <button class="pad" data-cmd="L"><span class="ar">◄</span><span class="lb">SPIN L</span></button>
      <button class="pad stop" data-cmd="S"><span class="ar">■</span><span class="lb">STOP</span></button>
      <button class="pad" data-cmd="R"><span class="ar">►</span><span class="lb">SPIN R</span></button>
      <button class="pad full" data-cmd="B"><span class="ar">▼</span><span class="lb">REVERSE</span></button>
    </div>
    <div class="hint">Hold a button (or a key) to drive — release to brake. Ignored in AUTO: the sensors are driving.</div>
  </section>

  <!-- ============ SPEED ============ -->
  <section class="card">
    <div class="card-title">SPEED <span class="tag" id="spd-tag">5</span></div>
    <div class="speed">
      <input type="range" id="spd" min="1" max="9" value="5" step="1">
      <span class="sv" id="spd-v">5</span>
    </div>
    <div class="hint">Applies to MANUAL and CAMERA / ML. Keep it ≤5 on USB power.</div>
  </section>

  <!-- ============ DEV ============ -->
  <section class="card">
    <details class="dev">
      <summary>FOR ML DEVELOPMENT — STREAM &amp; API</summary>
      <div class="api" id="api"></div>
      <div class="hint">
        Push your model's prediction to this dashboard from Python and it appears live above:<br>
        <code>requests.get(f"http://{ip}/ml", params={"p":"F","c":94}, timeout=0.3)</code>
      </div>
    </details>
  </section>

  <footer>
    <div class="keys">
      <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> move
      <kbd>Q</kbd><kbd>E</kbd> curve
      <kbd>Space</kbd> stop
      <kbd>M</kbd><kbd>A</kbd><kbd>C</kbd> modes
      <kbd>1–9</kbd> speed
    </div>
    Arduino Uno + L298N + ESP32-CAM · framed protocol · 9600 baud
  </footer>
</div>

<script>
var HOST = location.hostname;
document.getElementById('ip').textContent = 'http://' + HOST;
document.getElementById('cam').src = location.protocol + '//' + HOST + ':81/stream';

var API = [
  ['MJPEG stream (use in OpenCV)', 'http://' + HOST + ':81/stream'],
  ['Single JPEG snapshot',         'http://' + HOST + '/capture'],
  ['Send a driving command',       'http://' + HOST + '/cmd?c=F'],
  ['Telemetry (JSON)',             'http://' + HOST + '/status'],
  ['Push ML prediction',           'http://' + HOST + '/ml?p=F&c=94']
];
document.getElementById('api').innerHTML = API.map(function (r) {
  return '<div><b>' + r[0] + '</b><br>' + r[1] + '</div>';
}).join('');

var MODE_NAMES = {0:'STOP', 1:'MANUAL', 2:'AUTO', 3:'CAMERA / ML'};
var MODE_BTN   = {0:'m-s', 1:'m-m', 2:'m-a', 3:'m-c'};
var CMD_NAMES  = {F:'FORWARD', G:'CURVE LEFT', I:'CURVE RIGHT',
                  L:'SPIN LEFT', R:'SPIN RIGHT', B:'REVERSE', S:'STOP'};

/* ---------------- sending ---------------- */
var held = null;
function send(c) {
  fetch('/cmd?c=' + encodeURIComponent(c)).catch(function () {});
}
function hold(c, el) {
  if (held === c) return;
  release();
  held = c;
  if (el) el.classList.add('held');
  send(c);
}
function release() {
  if (!held) return;
  held = null;
  var all = document.querySelectorAll('.pad.held');
  for (var i = 0; i < all.length; i++) all[i].classList.remove('held');
  send('S');
}

var pads = document.querySelectorAll('.pad');
for (var i = 0; i < pads.length; i++) (function (p) {
  var c = p.getAttribute('data-cmd');
  p.addEventListener('mousedown',  function (e) { e.preventDefault(); hold(c, p); });
  p.addEventListener('touchstart', function (e) { e.preventDefault(); hold(c, p); }, {passive:false});
})(pads[i]);
window.addEventListener('mouseup', release);
window.addEventListener('touchend', release);
window.addEventListener('blur', release);

/* ---------------- speed slider ---------------- */
var spd = document.getElementById('spd');
spd.addEventListener('input', function () {
  document.getElementById('spd-v').textContent = spd.value;
  document.getElementById('spd-tag').textContent = spd.value;
});
spd.addEventListener('change', function () { send(String(spd.value)); });

/* ---------------- keyboard ---------------- */
var KEYMAP = {W:'F', S:'B', A:'L', D:'R', Q:'G', E:'I', ' ':'S'};
var MODEKEYS = {M:'M', A:'A', C:'C'};
document.addEventListener('keydown', function (e) {
  if (e.metaKey || e.ctrlKey || e.altKey) return;
  var k = e.key.toUpperCase();
  if (k >= '1' && k <= '9') { spd.value = k; spd.dispatchEvent(new Event('input')); send(k); return; }
  if (MODEKEYS[k]) { send(MODEKEYS[k]); flashMode(MODEKEYS[k]); return; }
  if (KEYMAP[k]) {
    if (e.repeat) return;
    e.preventDefault();
    hold(KEYMAP[k], null);
    var el = document.querySelector('.pad[data-cmd="' + KEYMAP[k] + '"]');
    if (el) el.classList.add('held');
  }
});
document.addEventListener('keyup', function (e) {
  if (KEYMAP[e.key.toUpperCase()]) release();
});

/* ---------------- status polling ---------------- */
function flashMode(c) {
  var id = {M:'m-m', A:'m-a', C:'m-c', S:'m-s'}[c];
  var el = document.getElementById(id);
  if (!el) return;
  el.style.transition = 'none'; el.style.transform = 'scale(.94)';
  setTimeout(function () { el.style.transition = ''; el.style.transform = ''; }, 130);
}

function paintMode(m) {
  ['m-m', 'm-a', 'm-c', 'm-s'].forEach(function (id) {
    var el = document.getElementById(id);
    if (el) el.classList.toggle('active', id === MODE_BTN[m]);
  });
  document.getElementById('mode-tag').textContent = MODE_NAMES[m] || '--';
}

function barColor(cm) {
  if (cm < 0)  return '#64748b';
  if (cm < 20) return '#ef4444';
  if (cm < 45) return '#f59e0b';
  return '#22c55e';
}

function renderTele(d) {
  var box = document.getElementById('tele-body');
  if (!d.link) {
    box.innerHTML = '<div id="tele-warn">No telemetry from the Arduino.<br>' +
      'Optional wire: <b>A3 →[1kΩ]→ IO13</b> (+2kΩ to GND).<br>' +
      'Driving and mode buttons work fine without it.</div>';
    return;
  }
  var w = Math.max(0, Math.min(100, (d.dist / 200) * 100));
  var col = barColor(d.dist);
  box.innerHTML =
    '<div class="tele-grid">' +
      '<div class="stat"><div class="k">DISTANCE</div>' +
        '<div class="v">' + d.dist + '<small> cm</small></div>' +
        '<div class="bar"><span style="width:' + w + '%;background:' + col + '"></span></div></div>' +
      '<div class="stat"><div class="k">SPEED / MODE</div>' +
        '<div class="v">' + d.speed + '</div>' +
        '<div class="bar"><span style="width:' + (d.speed / 255 * 100) +
          '%;background:#22d3ee"></span></div></div>' +
      '<div class="stat wide"><div class="k">IR OBSTACLE SENSORS</div>' +
        '<div class="v" style="display:flex;gap:8px;margin-top:6px">' +
          '<span class="pill ' + (d.irL ? 'hit' : 'clear') + '">LEFT ' + (d.irL ? 'BLOCKED' : 'clear') + '</span>' +
          '<span class="pill ' + (d.irR ? 'hit' : 'clear') + '">RIGHT ' + (d.irR ? 'BLOCKED' : 'clear') + '</span>' +
        '</div></div>' +
    '</div>' +
    '<div class="hint">updated ' + d.age + ' ms ago</div>';
}

function renderMl(d) {
  var tag = document.getElementById('ml-tag');
  var body = document.getElementById('ml-body');
  if (!d.ml) {
    tag.textContent = 'idle';
    tag.style.color = '#22d3ee';
    body.innerHTML = '<div class="ml-idle">No model running. Start the laptop autopilot to watch ' +
      'predictions live:<br><code>python3 3_run_autonomous_ml.py --ip ' + HOST + ' --mode hybrid</code></div>';
    return;
  }
  var p = (d.mlPred || '?').toUpperCase();
  var c = Math.max(0, Math.min(100, d.mlConf || 0));
  tag.textContent = d.mlAge + ' ms ago';
  tag.style.color = '#4ade80';
  body.innerHTML =
    '<div class="ml-row">' +
      '<div class="ml-letter">' + p + '</div>' +
      '<div class="ml-meta">' +
        '<div class="ml-cmd">' + (CMD_NAMES[p] || p) + '</div>' +
        '<div class="ml-conf">confidence ' + c + '%</div>' +
        '<div class="bar ml"><span style="width:' + c + '%"></span></div>' +
      '</div>' +
    '</div>';
}

function poll() {
  fetch('/status').then(function (r) { return r.json(); }).then(function (d) {
    var dot = document.getElementById('dot');
    dot.className = 'dot on';
    document.getElementById('conn-txt').textContent = 'online';
    if (d.link) paintMode(d.mode); else document.getElementById('mode-tag').textContent = '--';
    renderTele(d);
    renderMl(d);
  }).catch(function () {
    document.getElementById('dot').className = 'dot off';
    document.getElementById('conn-txt').textContent = 'offline';
  });
}
setInterval(poll, 400);
poll();
</script>
</body>
</html>

)rawliteral";

// ============================================================================
// HTTP HANDLER: Dashboard Index Page (/)
// ============================================================================
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

// ============================================================================
// HTTP HANDLER: Single Frame JPEG Snapshot (/capture)
// ============================================================================
static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

// ============================================================================
// HTTP HANDLER: MJPEG Video Stream on Port 81 (/stream)
// ============================================================================
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      res = ESP_FAIL;
    } else {
      size_t hlen = snprintf(part_buf, 64, _STREAM_PART, fb->len);
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
      if (res == ESP_OK) res = httpd_resp_send_chunk(req, part_buf, hlen);
      if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
      esp_camera_fb_return(fb);
    }
    if (res != ESP_OK) break;
  }
  return res;
}

// ============================================================================
// HTTP HANDLER: Command Bridge (/cmd?c=F) -> UART Serial to Arduino Uno Pin A2
// ============================================================================
static esp_err_t cmd_handler(httpd_req_t *req) {
  char buf[32];
  size_t buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1 && buf_len < sizeof(buf)) {
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      char param[8];
      if (httpd_query_key_value(buf, "c", param, sizeof(param)) == ESP_OK) {
        // Send the command to the Arduino INSIDE A FRAME: '~' + cmd + '\n'
        // The Uno ONLY accepts framed commands, so the boot logs / debug text
        // this sketch prints on the same wire can never be read as a command.
        Serial.write('~');
        Serial.write(param[0]);
        Serial.write('\n');
      }
    }
  }
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "OK", 2);
}

// ============================================================================
// HTTP HANDLER: Live Telemetry JSON (/status)  <- read by the dashboard panel
// ============================================================================
static esp_err_t status_handler(httpd_req_t *req) {
  char json[224];
  bool fresh = (tele.lastMs != 0) && (millis() - tele.lastMs < 2000);
  unsigned long age   = tele.lastMs ? (millis() - tele.lastMs) : 0;
  unsigned long mlAge = mlIsFresh() ? (millis() - ml.lastMs) : 0;

  snprintf(json, sizeof(json),
           "{\"link\":%s,\"dist\":%d,\"irL\":%d,\"irR\":%d,\"mode\":%d,\"speed\":%d,"
           "\"age\":%lu,\"ml\":%s,\"mlPred\":\"%s\",\"mlConf\":%d,\"mlAge\":%lu}",
           fresh ? "true" : "false",
           tele.dist, tele.irL, tele.irR, tele.mode, tele.speed, age,
           mlIsFresh() ? "true" : "false",
           ml.pred, ml.conf, mlAge);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json, strlen(json));
}

// ============================================================================
// HTTP HANDLER: Push an ML prediction from the laptop  (/ml?p=F&c=94)
// Lets the dashboard show what your model is predicting WHILE it drives.
// ============================================================================
static esp_err_t ml_handler(httpd_req_t *req) {
  char buf[96];
  char p[8]  = "";
  char c[12] = "0";

  if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
    httpd_query_key_value(buf, "p", p, sizeof(p));
    httpd_query_key_value(buf, "c", c, sizeof(c));
  }

  if (p[0]) {
    snprintf(ml.pred, sizeof(ml.pred), "%c", (char)toupper((int)p[0]));
    ml.conf   = constrain(atoi(c), 0, 100);
    ml.lastMs = millis();
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "OK", 2);
}

// ============================================================================
// START HTTP SERVERS (PORT 80 FOR COMMANDS/UI, PORT 81 FOR MJPEG STREAM)
// ============================================================================
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri   = { .uri = "/",        .method = HTTP_GET, .handler = index_handler,   .user_ctx = NULL };
  httpd_uri_t cmd_uri     = { .uri = "/cmd",     .method = HTTP_GET, .handler = cmd_handler,     .user_ctx = NULL };
  httpd_uri_t capture_uri = { .uri = "/capture", .method = HTTP_GET, .handler = capture_handler, .user_ctx = NULL };
  httpd_uri_t status_uri  = { .uri = "/status",  .method = HTTP_GET, .handler = status_handler,  .user_ctx = NULL };
  httpd_uri_t ml_uri      = { .uri = "/ml",      .method = HTTP_GET, .handler = ml_handler,      .user_ctx = NULL };
  httpd_uri_t stream_uri  = { .uri = "/stream",  .method = HTTP_GET, .handler = stream_handler,  .user_ctx = NULL };

  if (httpd_start(&cmd_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(cmd_httpd, &index_uri);
    httpd_register_uri_handler(cmd_httpd, &cmd_uri);
    httpd_register_uri_handler(cmd_httpd, &capture_uri);
    httpd_register_uri_handler(cmd_httpd, &status_uri);
    httpd_register_uri_handler(cmd_httpd, &ml_uri);
  }

  config.server_port += 1; // Port 81
  config.ctrl_port   += 1;
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // Disable brownout detector to prevent resets during Wi-Fi startup spikes
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // IMPORTANT: 9600 baud matches Arduino Uno SoftwareSerial on Pin A2!
  Serial.begin(9600);

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // QVGA (320x240) gives fast 15-25 FPS low-latency streaming for ML control
  if (psramFound()) {
    config.frame_size   = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count     = 2;
  } else {
    config.frame_size   = FRAMESIZE_QVGA;
    config.jpeg_quality = 15;
    config.fb_count     = 1;
  }

  // Telemetry link from the Arduino Uno (UART1 remapped to IO13, RX only).
  // Safe to enable even if the optional wire is not connected.
  if (LINK_RX_PIN >= 0) {
    LinkSerial.begin(9600, SERIAL_8N1, LINK_RX_PIN, -1);
  }

  Serial.println();
  Serial.println(F("===== 4WD Autonomous Car - ESP32-CAM ====="));

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Camera init FAILED, error 0x%x\n", err);
    Serial.println(F("[CAM] Check: is the GPIO0 -> GND wire still attached? Remove it and reboot."));
    return;
  }
  Serial.println(F("[CAM] Camera OK"));

  // Try connecting to Wi-Fi Station first; fallback to Access Point if unavailable
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt < 10000)) {
    delay(400);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("[WiFi] Could not join \"%s\" -> starting own Access Point\n", WIFI_SSID);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("[WiFi] Join Wi-Fi \"%s\" (password \"%s\")\n", AP_SSID, AP_PASS);
    Serial.printf("[WiFi] Dashboard:  http://%s/\n", WiFi.softAPIP().toString().c_str());
    Serial.printf("[WiFi] Video:      http://%s:81/stream\n", WiFi.softAPIP().toString().c_str());
  } else {
    Serial.printf("[WiFi] Connected. Dashboard: http://%s/\n", WiFi.localIP().toString().c_str());
    Serial.printf("[WiFi] Video stream: http://%s:81/stream\n", WiFi.localIP().toString().c_str());
  }
  Serial.println(F("[CAM] Control page ready. Open the Dashboard URL in a browser."));

  // Blink flash LED briefly twice to indicate Wi-Fi & Camera Server are ready!
  for (int i = 0; i < 2; i++) {
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(80);
    digitalWrite(FLASH_LED_PIN, LOW);
    delay(120);
  }

  startCameraServer();
}

void loop() {
  pollTelemetry();   // read #D:.. telemetry lines coming from the Arduino Uno
  delay(50);
}
