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
 *   - ESP32-CAM U0R (GPIO3)   <- Optional: Arduino Uno Pin A3 (via 1k/2k divider)
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

// -------------------- MJPEG STREAM BOUNDARY ------------------------
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t cmd_httpd    = NULL;
httpd_handle_t stream_httpd = NULL;

// -------------------- BUILT-IN WEB DASHBOARD HTML ------------------
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>4WD Autonomous Car — ESP32-CAM Dashboard</title>
  <style>
    body { font-family: Arial, sans-serif; background: #0f172a; color: #f8fafc; text-align: center; margin: 0; padding: 16px; }
    h2 { color: #38bdf8; margin-bottom: 6px; }
    #stream-box { display: inline-block; border: 3px solid #38bdf8; border-radius: 10px; overflow: hidden; background: #000; margin: 10px 0; }
    img { width: 320px; height: 240px; display: block; }
    .grid { display: grid; grid-template-columns: repeat(3, 95px); gap: 10px; justify-content: center; margin: 14px auto; }
    button { background: #1e293b; color: #f8fafc; border: 2px solid #38bdf8; border-radius: 8px; padding: 14px 8px; font-size: 14px; font-weight: bold; cursor: pointer; user-select: none; }
    button:active { background: #0284c7; }
    .stop-btn { border-color: #ef4444; background: #7f1d1d; }
    .mode-bar { margin-top: 12px; display: flex; gap: 10px; justify-content: center; flex-wrap: wrap; }
    .status { color: #fde047; font-size: 14px; margin-top: 8px; }
  </style>
</head>
<body>
  <h2>4WD Autonomous Car — ESP32-CAM Controller</h2>
  <div id="stream-box"><img id="cam" src=""></div>
  <div class="status" id="status">Last Command: S (Hold W/A/S/D or Q/E keys to drive)</div>
  <div class="grid">
    <button onmousedown="send('G')" onmouseup="send('S')" ontouchstart="send('G')" ontouchend="send('S')">↖ Curve L (Q)</button>
    <button onmousedown="send('F')" onmouseup="send('S')" ontouchstart="send('F')" ontouchend="send('S')">▲ Forward (W)</button>
    <button onmousedown="send('I')" onmouseup="send('S')" ontouchstart="send('I')" ontouchend="send('S')">↗ Curve R (E)</button>
    <button onmousedown="send('L')" onmouseup="send('S')" ontouchstart="send('L')" ontouchend="send('S')">◄ Spin L (A)</button>
    <button class="stop-btn" onclick="send('S')">■ STOP (Space)</button>
    <button onmousedown="send('R')" onmouseup="send('S')" ontouchstart="send('R')" ontouchend="send('S')">► Spin R (D)</button>
    <div></div>
    <button onmousedown="send('B')" onmouseup="send('S')" ontouchstart="send('B')" ontouchend="send('S')">▼ Reverse (S)</button>
    <div></div>
  </div>
  <div class="mode-bar">
    <button onclick="send('C')">Mode: Camera / ML (C)</button>
    <button onclick="send('A')">Mode: On-Board Sensors (A)</button>
    <button onclick="send('5')">Speed: Medium (5)</button>
    <button onclick="send('8')">Speed: Fast (8)</button>
  </div>
  <script>
    document.getElementById('cam').src = window.location.protocol + '//' + window.location.hostname + ':81/stream';
    let activeKey = null;
    function send(c) {
      document.getElementById('status').innerText = 'Last Command: ' + c;
      fetch('/cmd?c=' + encodeURIComponent(c)).catch(()=>{});
    }
    document.addEventListener('keydown', (e) => {
      if (e.repeat) return;
      const k = e.key.toUpperCase();
      const map = {'W':'F', 'S':'B', 'A':'L', 'D':'R', 'Q':'G', 'E':'I', ' ':'S'};
      if (map[k]) { activeKey = k; send(map[k]); }
    });
    document.addEventListener('keyup', (e) => {
      const k = e.key.toUpperCase();
      if (k === activeKey) { activeKey = null; send('S'); }
    });
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
        // Send single command character directly over U0T (GPIO1) to Arduino A2
        Serial.write(param[0]);
        Serial.write('\n');
      }
    }
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
  httpd_uri_t stream_uri  = { .uri = "/stream",  .method = HTTP_GET, .handler = stream_handler,  .user_ctx = NULL };

  if (httpd_start(&cmd_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(cmd_httpd, &index_uri);
    httpd_register_uri_handler(cmd_httpd, &cmd_uri);
    httpd_register_uri_handler(cmd_httpd, &capture_uri);
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

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    return;
  }

  // Try connecting to Wi-Fi Station first; fallback to Access Point if unavailable
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt < 10000)) {
    delay(400);
  }

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
  }

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
  delay(100);
}
