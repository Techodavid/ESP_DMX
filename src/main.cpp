#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>

#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_timer.h"

// =================== CONFIG ===================
#define AP_SSID "ESP32-DMX-DEBUG"
#define AP_PASS "12345678"

#define LED_PIN 8
#define LED_COUNT 1

#define DMX_RX_PIN 5
static const uart_port_t DMX_UART_PORT = UART_NUM_1;

static const uint32_t IDLE_FALLBACK_MS = 4;
static const uint32_t READ_COUNT = 6;
static const uint32_t DMX_TIMEOUT_MS = 2000;
static uint8_t last_startcode = 0; 

// =================== GLOBALS ===================
Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
WebServer server(80);
Preferences prefs;

static QueueHandle_t dmx_queue = nullptr;

static uint16_t startAddr = 2;

// DMX buffers
static uint8_t dmx_universe[513];
static uint8_t frame_buf[600];
static uint16_t frame_len = 0;

// RAW / LATCH logic
static uint8_t rawCh[READ_COUNT];
static uint8_t lastRawCh[READ_COUNT];
static uint8_t latchedCh[READ_COUNT];
static uint8_t stableCnt[READ_COUNT];

// Stats
static uint32_t dmx_frames = 0;
static uint32_t dmx_breaks = 0;
static uint32_t dmx_bytes = 0;
static uint32_t dmx_overflows = 0;
static uint32_t last_frame_ms = 0;
static uint32_t last_rx_ms = 0;

// Debug
static uint32_t aliveCounter = 0;
static uint32_t lastTickMs = 0;
static float lastTempC = NAN;

static int64_t last_rx_us = 0;
static const int64_t FRAME_GAP_US = 12000; // 2 ms -> neuer Frame-Burst

static bool breakSeenRecently = false;
static uint32_t lastBreakMs = 0;
static const uint32_t BREAK_RECENT_MS = 300; // 2s Fenster

static uint8_t uartStopBits = 2; // 2 = DMX (8N2), 1 = 8N1 Test für Billig-Controller

// =================== WEB LOG ===================
#define LOG_LINES 120
#define LOG_LINE_LEN 96

static char logBuf[LOG_LINES][LOG_LINE_LEN];
static uint16_t logWriteIdx = 0;
static uint32_t logCounter = 0;

static void dmx_init();
static void dmx_reinit();

// =================== HELPERS ===================
static inline void setLED(uint8_t r, uint8_t g, uint8_t b) {
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}

static inline void setLED_DMX(uint8_t r, uint8_t g, uint8_t b) {
  led.setPixelColor(0, led.Color(g, r, b)); // <-- R/G Tausch
  led.show();
}

static inline uint16_t minFrameLen() {
  return startAddr + 3;   // RGB ist das Minimum
}


static void dmx_reinit() {
  uart_driver_delete(DMX_UART_PORT);
  dmx_queue = nullptr;
  frame_len = 0;
  last_rx_us = 0;
  breakSeenRecently = false;
  dmx_init();
}


static void logLine(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);

  char tmp[LOG_LINE_LEN];
  vsnprintf(tmp, sizeof(tmp), fmt, args);
  va_end(args);

  uint16_t idx = logWriteIdx % LOG_LINES;
  snprintf(logBuf[idx], LOG_LINE_LEN,
           "[%8lu ms] %s", millis(), tmp);

  logWriteIdx++;
  logCounter++;
}

// =================== DMX APPLY ===================



static void dmx_apply_frame() {
  if (frame_len == 0) return;

  // Startcode merken
  last_startcode = frame_buf[0];

  // Wenn du NUR DMX (Startcode 0) akzeptieren willst:
  // -> dann drop, aber NICHT Universe löschen
  if (last_startcode != 0x00) {
    logLine("DROP startcode=%u len=%u", last_startcode, frame_len);
    frame_len = 0;
    return;
  }

  // Variable frame length ist OK:
  // frame_buf[0]=Startcode, frame_buf[1..] = Slots
  // DMX Universe: [0]=Startcode, [1..512]=Kanäle
  dmx_universe[0] = last_startcode;

  uint16_t slots = (frame_len > 513) ? 513 : frame_len; // max bis Slot 512 + Startcode
  for (uint16_t i = 1; i < slots; i++) {
    dmx_universe[i] = frame_buf[i];
  }

  // RAW/LATCH aus dem Universe ziehen
  for (uint8_t i = 0; i < READ_COUNT; i++) {
    uint16_t slot = startAddr + i;
    uint8_t v = (slot <= 512) ? dmx_universe[slot] : 0;
    rawCh[i] = v;

    if (rawCh[i] == lastRawCh[i]) {
      if (stableCnt[i] < 255) stableCnt[i]++;
    } else {
      stableCnt[i] = 1;
      lastRawCh[i] = rawCh[i];
    }
    if (stableCnt[i] >= 2) latchedCh[i] = rawCh[i];
  }

  dmx_frames++;
  last_frame_ms = millis();
  frame_len = 0;
}

 


// =================== DMX INIT ===================
static void dmx_init() {
  uart_config_t cfg = {};
  cfg.baud_rate = 250000;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity    = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_2;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;
  cfg.stop_bits = (uartStopBits == 1) ? UART_STOP_BITS_1 : UART_STOP_BITS_2;

  uart_driver_install(DMX_UART_PORT, 2048, 0, 20, &dmx_queue, 0);
  uart_param_config(DMX_UART_PORT, &cfg);
  uart_set_pin(DMX_UART_PORT, UART_PIN_NO_CHANGE, DMX_RX_PIN,
               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  uart_set_rx_timeout(DMX_UART_PORT, 10);
  memset(dmx_universe, 0, sizeof(dmx_universe));
}

// =================== DMX PROCESS ===================
static void dmx_process() {
  if (!dmx_queue) return;

  uart_event_t ev;
  while (xQueueReceive(dmx_queue, &ev, 0) == pdTRUE) {

    if (ev.type == UART_BREAK) {
      dmx_breaks++;
      logLine("BREAK detected, frame_len=%u", frame_len);

      // Break = neue Frame-Grenze -> vorherigen Frame anwenden
      dmx_apply_frame();
      frame_len = 0;
      last_rx_ms = millis();
      continue;
    }

    if (ev.type == UART_DATA) {
      int remaining = ev.size;
      uint8_t buf[256];

      while (remaining > 0) {
        int chunk = (remaining > (int)sizeof(buf)) ? (int)sizeof(buf) : remaining;
        int n = uart_read_bytes(DMX_UART_PORT, buf, chunk, 0);
        if (n <= 0) break;

        dmx_bytes += (uint32_t)n;
        last_rx_ms = millis();

        for (int i = 0; i < n; i++) {
          if (frame_len < sizeof(frame_buf)) {
            frame_buf[frame_len++] = buf[i];
          }
        }

        remaining -= n;
      }
      continue;
    }

    if (ev.type == UART_FIFO_OVF || ev.type == UART_BUFFER_FULL) {
      dmx_overflows++;
      logLine("OVF: flush input + reset queue");
      uart_flush_input(DMX_UART_PORT);
      xQueueReset(dmx_queue);
      frame_len = 0;
      continue;
    }
  }

  // Idle-Fallback nur wenn Break fehlt
  if (frame_len > 0 && (millis() - last_rx_ms) > IDLE_FALLBACK_MS) {
    logLine("APPLY frame via IDLE (len=%u idle=%lu ms)",
            frame_len, (unsigned long)(millis() - last_rx_ms));
    dmx_apply_frame();
  }
}



// =================== WEB ===================
static void handleRoot() {
  String html;
  html.reserve(4000);

  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>ESP32 DMX Debug</title>";
  html += "<style>";
  html += "body{font-family:system-ui,Arial;margin:16px}";
  html += ".card{border:1px solid #ccc;border-radius:12px;padding:12px;margin-bottom:12px}";
  html += ".row{display:flex;justify-content:space-between}";
  html += "input{width:80px}";
  html += "</style></head><body>";

  html += "<h2>ESP32 DMX Debug</h2>";

  // Startadresse
  html += "<div class='card'>";
  html += "<h3>Start Address</h3>";
  html += "<input id='sa' type='number' min='1' max='512'>";
  html += "async function setSB(v){";
html += "await fetch('/set_sb?sb='+v);";
html += "}";
  html += "<button onclick='setSA()'>Set</button>";
  html += "<span id='sa_cur'></span>";
  html += "</div>";

html += "<div style='margin-top:8px'>";
html += "<button onclick=\"setSB(2)\">UART 8N2 (DMX)</button> ";
html += "<button onclick=\"setSB(1)\">UART 8N1 (Test)</button> ";
html += "<span id='sb_cur'></span>";
html += "</div>";

  // Status
  html += "<div class='card'><h3>Status</h3>";
  html += "<div class='row'><span>Frames</span><b id='fr'>-</b></div>";
  html += "<div class='row'><span>Breaks</span><b id='br'>-</b></div>";
  html += "<div class='row'><span>Bytes</span><b id='by'>-</b></div>";
  html += "<div class='row'><span>Overflows</span><b id='ov'>-</b></div>";
  html += "<div class='row'><span>Age</span><b id='age'>-</b></div>";
  html += "</div>";

  // Channels
 html += "<div class='card'><h3>Channels</h3>";
for (int i = 1; i <= READ_COUNT; i++) {
  html += "<div class='row'><span>CH" + String(i) +
          "</span><b id='c" + String(i) + "'>-</b></div>";
}
html += "</div>";

  html += "<div class='card'><h3>DMX Log</h3>";
html += "<pre id='log' style='height:240px;overflow:auto;"
        "background:#111;color:#0f0;padding:8px;"
        "font-size:12px'></pre>";
html += "</div>";

  // JS
 html += "<script>";
html += "let saInit=false;";

html += "async function upd(){";
html += "const j=await (await fetch('/json')).json();";
html += "fr.textContent=j.frames;";
html += "br.textContent=j.breaks;";
html += "by.textContent=j.bytes;";
html += "ov.textContent=j.overflows;";
html += "age.textContent=j.age+' ms';";
html += "for(let i=0;i<j.latch.length;i++){";
html += "const e=document.getElementById('c'+(i+1));";
html += "if(e) e.textContent=j.latch[i];}";
html += "const sb=document.getElementById('sb_cur');";
html += "if(sb) sb.textContent='Stopbits: '+j.sb;";
html += "if(!saInit){sa.value=j.start;sa_cur.textContent='Current: '+j.start;saInit=true;}";
html += "}";

html += "async function updLog(){";
html += "const t=await (await fetch('/log')).text();";
html += "const el=document.getElementById('log');";
html += "if(!el) return;";
html += "el.textContent=t;";
html += "el.scrollTop=el.scrollHeight;";
html += "}";

html += "async function setSA(){";
html += "await fetch('/set_start?addr='+sa.value);";
html += "saInit=false;";
html += "}";

html += "async function setSB(v){";
html += "await fetch('/set_sb?sb='+v);";
html += "}";

html += "setInterval(upd,500);";
html += "setInterval(updLog,700);";
html += "upd();updLog();";
html += "</script>";

  server.send(200, "text/html; charset=utf-8", html);
}


static void handleSetStart() {
  if (!server.hasArg("addr")) {
    server.send(400, "text/plain", "missing addr");
    return;
  }
  uint16_t a = server.arg("addr").toInt();
  if (a < 1 || a > 512) {
    server.send(400, "text/plain", "range 1..512");
    return;
  }
  startAddr = a;
  prefs.putUShort("start", startAddr);
  server.send(200, "text/plain", "ok");
}

static void handleSetStopBits() {
  if (!server.hasArg("sb")) { server.send(400, "text/plain", "missing sb"); return; }
  int sb = server.arg("sb").toInt();
  if (sb != 1 && sb != 2) { server.send(400, "text/plain", "sb must be 1 or 2"); return; }
  uartStopBits = (uint8_t)sb;
  dmx_reinit();
  server.send(200, "text/plain", "ok");
}

static void handleJson() {
  uint32_t now = millis();
  uint32_t age = now - last_frame_ms;

  String j = "{";
  j += "\"uptime\":" + String(now/1000) + ",";
  j += "\"temp\":" + String(lastTempC,1) + ",";
  j += "\"frames\":" + String(dmx_frames) + ",";
  j += "\"breaks\":" + String(dmx_breaks) + ",";
  j += "\"bytes\":" + String(dmx_bytes) + ",";
  j += "\"overflows\":" + String(dmx_overflows) + ",";
  j += "\"age\":" + String(age) + ",";
  j += "\"sb\":" + String(uartStopBits) + ",";
  j += "\"start\":" + String(startAddr) + ",";

  j += "\"raw\":[";
  for(int i=0;i<READ_COUNT;i++){ j+=String(rawCh[i])+(i<5?",":""); }
  j += "],\"latch\":[";
  for(int i=0;i<READ_COUNT;i++){ j+=String(latchedCh[i])+(i<5?",":""); }
  j += "],\"stable\":[";
  for(int i=0;i<READ_COUNT;i++){ j+=String(stableCnt[i])+(i<5?",":""); }
  j += "]}";

  server.send(200, "application/json", j);
}
static void handleLog() {
  String out;
  out.reserve(LOG_LINES * 120);

  uint16_t start =
    (logWriteIdx > LOG_LINES) ? (logWriteIdx - LOG_LINES) : 0;

  for (uint16_t i = start; i < logWriteIdx; i++) {
    uint16_t idx = i % LOG_LINES;
    out += logBuf[idx];
    out += "\n";
  }

  server.send(200, "text/plain; charset=utf-8", out);
}


// =================== SETUP ===================
void setup() {


  led.begin();
  setLED(0,0,30);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  prefs.begin("dmx", false);
  startAddr = prefs.getUShort("start", 2);

  dmx_init();
  server.on("/", handleRoot);
  server.on("/json", handleJson);
  server.on("/set_start", handleSetStart);
  server.on("/log", handleLog);
  server.on("/set_sb", handleSetStopBits);
  server.begin();

  setLED(0,30,0);
}

// =================== LOOP ===================
void loop() {
  server.handleClient();
  dmx_process();

  uint32_t now = millis();
  if (now - lastTickMs > 1000) {
    lastTickMs = now;
    aliveCounter++;
    lastTempC = temperatureRead();
  }

   bool clientConnected = (WiFi.softAPgetStationNum() > 0);
bool dmxAlive = ((now - last_frame_ms) <= DMX_TIMEOUT_MS);

if (clientConnected && dmxAlive) {
  // DMX Kanal 1/2/3 relativ zur Startadresse (gelatcht!)
  uint8_t r = latchedCh[0];
  uint8_t g = latchedCh[1];
  uint8_t b = latchedCh[2];
  setLED_DMX(r, g, b);
} 
else {
  if (!dmxAlive) {
    setLED(30, 0, 0); // DMX lost
  } else {
    setLED(0, 30, 0); // OK
  }
}
}


