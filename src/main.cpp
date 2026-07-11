/*
 ******************************************************************************
 * Humidity Monitor Firmware v1.1
 ******************************************************************************
 *
 *  Reads temperature + relative humidity INSIDE and OUTSIDE a container and
 *  serves them on a web dashboard. Monitor only — no relay, no heater, no
 *  control loop. (For the heater/drying state machine see HumidityController.)
 *
 *  TARGET BOARD:
 *    ESP32-S3-DevKitC-1  (env:esp32s3)
 *
 *  WHY TWO I2C BUSES:
 *    The AHT30 has a hard-wired I2C address of 0x38 with no address-select
 *    pin, so two of them can never share one bus. The ESP32-S3 has two full
 *    hardware I2C controllers (SOC_HP_I2C_NUM == 2), so each sensor gets its
 *    own bus and both keep address 0x38.
 *
 *    This is why the firmware is S3-only. On the ESP32-C6 there is just one
 *    high-power I2C controller; its `Wire1` maps to the pin-restricted
 *    low-power LP_I2C peripheral. Porting to C6 means an I2C mux or a
 *    different outside sensor (SHT31 @ 0x44, BME280 @ 0x76).
 *
 *  WIRING:
 *    Inside  AHT30 → SDA GPIO 8,  SCL GPIO 9    (Wire  / I2C0)
 *    Outside AHT30 → SDA GPIO 10, SCL GPIO 11   (Wire1 / I2C1)
 *    Both sensors: VIN → 3V3, GND → GND. Each bus needs its own pull-ups;
 *    most AHT30 breakouts already carry them.
 *
 *  HTTP API:
 *    GET /                current dashboard (HTML)
 *    GET /status         current readings + network + settings (small JSON)
 *    GET /history        downsampled history for the charts (JSON)
 *    GET /history.csv    full history, streamed as CSV
 *    GET /sensor/refresh force an immediate read, returns /status payload
 *    GET /health         "OK"
 *
 *  History is kept separate from /status on purpose: at 4032 points a combined
 *  payload would be hundreds of KB and would have to be serialized into a
 *  single heap String on every poll.
 *
 ******************************************************************************/

#define FIRMWARE_VERSION  "v1.1"

// ============================================================
//  PIN DEFINITIONS  (ESP32-S3)
//    Avoids strapping pins (0/3/45/46), USB (19/20) and the
//    flash/PSRAM pins (26-37) used by the N16R8 modules.
// ============================================================
#define SDA_INSIDE    8
#define SCL_INSIDE    9
#define SDA_OUTSIDE  10
#define SCL_OUTSIDE  11

#define I2C_HZ  100000UL   // AHT30 is rated to 400 kHz; 100 kHz tolerates long leads

// ============================================================
//  AP PORTAL / OTA
// ============================================================
#define AP_SSID   "ESP32-Setup"
#define AP_PASS   ""
#define OTA_USER  "admin"
#define OTA_PASS  "pw"

// ============================================================
//  DEFAULTS  (user-adjustable via web UI / NVS)
// ============================================================
#define DEFAULT_DEVICE_NAME  "HumidityMonitor"
#define DEFAULT_RH_ALERT     20.0f   // % — inside RH above this shows red

// ============================================================
//  TIMING
// ============================================================
#define READ_INTERVAL_MS     300000UL   // 5 min — matches the history span below
#define WIFI_RETRY_INTERVAL   30000UL

// ============================================================
//  SENSOR HISTORY
//    4032 x 5 min = 2 weeks. Readings are stored as tenths in
//    int16_t (12 bytes/point, ~48 KB total) rather than floats,
//    which would double the static footprint for no added
//    precision — the AHT30 resolves ~0.024 %RH / 0.01 degC.
// ============================================================
#define HISTORY_SIZE  4032
#define HIST_NA       INT16_MIN    // sentinel for "no reading"
#define HIST_MAX_PTS  480          // max points returned by /history

// ============================================================
//  INCLUDES
// ============================================================
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Wire.h>
#include <time.h>
#include <math.h>
#include <ArduinoJson.h>
#include <ElegantOTA.h>
#include <ArduinoOTA.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include <Adafruit_AHTX0.h>

// ============================================================
//  OBJECTS
// ============================================================
WebServer   server(80);
Preferences prefs;

// ============================================================
//  SENSORS
//    One AHT30 per I2C bus. Both live at 0x38.
// ============================================================
struct Sensor {
  Adafruit_AHTX0 dev;
  TwoWire*       bus;
  uint8_t        sda;
  uint8_t        scl;
  const char*    label;
  int32_t        id;       // Adafruit unified-sensor id, must be unique
  bool           online;
  float          hum;      // %RH
  float          tempF;    // degF
};

Sensor inside  = { {}, &Wire,  SDA_INSIDE,  SCL_INSIDE,  "inside",  0, false, NAN, NAN };
Sensor outside = { {}, &Wire1, SDA_OUTSIDE, SCL_OUTSIDE, "outside", 1, false, NAN, NAN };

// ============================================================
//  RUNTIME STATE
// ============================================================
String deviceName;
float  rhAlert = DEFAULT_RH_ALERT;

String        lastSensorTime = "Never";
unsigned long bootMillis     = 0;
unsigned long nextReadAt     = 0;
unsigned long lastWiFiRetry  = 0;

bool mdnsRunning = false;
bool inAPMode    = false;
bool otaRunning  = false;

String csrfToken;

// History ring buffer
struct HistPoint {
  int16_t  hIn, tIn, hOut, tOut;   // tenths
  uint32_t ts;                     // unix epoch, 0 = clock not yet synced
};
HistPoint history[HISTORY_SIZE];
int historyHead  = 0;
int historyCount = 0;

// ============================================================
//  FORWARD DECLARATIONS
// ============================================================
bool   connectWiFi();
void   startAPPortal();
void   startMDNS();
void   setupRoutes();
void   readSensors();
String sanitizeMDNS(const String &s);
String getTimeString();
String buildStatusJson();
bool   checkCsrf();

// ============================================================
//  NVS HELPERS
// ============================================================
void loadPrefs() {
  prefs.begin("system", true);
  deviceName = prefs.getString("name",    DEFAULT_DEVICE_NAME);
  rhAlert    = prefs.getFloat("rhAlert",  DEFAULT_RH_ALERT);
  prefs.end();
}

void saveSettings() {
  prefs.begin("system", false);
  prefs.putFloat("rhAlert", rhAlert);
  prefs.end();
}

void saveName(const String &name) {
  prefs.begin("system", false);
  prefs.putString("name", name);
  prefs.end();
}

void saveWiFiCreds(const String &ssid, const String &pass) {
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
}

void loadWiFiCreds(String &ssid, String &pass) {
  prefs.begin("wifi", true);
  ssid = prefs.getString("ssid", "");
  pass = prefs.getString("pass", "");
  prefs.end();
}

void clearWiFiCreds() {
  prefs.begin("wifi", false);
  prefs.remove("ssid");
  prefs.remove("pass");
  prefs.end();
}

// ============================================================
//  CSRF
// ============================================================
void generateCsrfToken() {
  csrfToken = "";
  for (int i = 0; i < 16; i++) csrfToken += String(esp_random() & 0xF, HEX);
}

bool checkCsrf() {
  String tok = server.hasArg("csrf") ? server.arg("csrf") : "";
  if (tok.isEmpty()) tok = server.header("X-CSRF-Token");
  if (tok != csrfToken) {
    server.send(403, "application/json", "{\"error\":\"Invalid or missing CSRF token\"}");
    return false;
  }
  return true;
}

// ============================================================
//  mDNS
// ============================================================
String sanitizeMDNS(const String &input) {
  String out = "";
  for (int i = 0; i < (int)input.length(); i++) {
    char c = tolower((unsigned char)input[i]);
    if (isalnum((unsigned char)c)) out += c;
    else if (c == '-' || c == ' ') out += '-';
  }
  while (out.startsWith("-")) out = out.substring(1);
  while (out.endsWith("-"))   out = out.substring(0, out.length() - 1);
  if (out.isEmpty())          out = DEFAULT_DEVICE_NAME;
  return out;
}

void startMDNS() {
  if (mdnsRunning) MDNS.end();
  String host = sanitizeMDNS(deviceName);
  if (MDNS.begin(host.c_str())) {
    MDNS.addService("http", "tcp", 80);
    mdnsRunning = true;
    Serial.printf("[mDNS] http://%s.local\n", host.c_str());
  } else {
    mdnsRunning = false;
    Serial.println("[mDNS] Failed to start");
  }
}

// ============================================================
//  TIME
// ============================================================
void setupTime() {
  configTzTime("EST5EDT,M3.2.0,M11.1.0", "pool.ntp.org", "time.nist.gov");
  Serial.println("[NTP] Sync requested");
}

bool timeSynced() { return time(nullptr) > 100000; }

String getTimeString() {
  if (!timeSynced()) return "Syncing...";
  time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);
  char buf[32];
  strftime(buf, sizeof(buf), "%m/%d/%Y %I:%M:%S %p", &t);
  return String(buf);
}

// ============================================================
//  POWER MANAGEMENT
// ============================================================
void setupPowerManagement() {
  esp_wifi_set_ps(WIFI_PS_NONE);
  Serial.println("[Power] Always-on mode");
}

// ============================================================
//  DERIVED VALUES
// ============================================================

// Magnus-Tetens dew point. Below the dew point, moisture condenses out of the
// air — comparing the outside dew point against the inside temperature tells
// you whether opening the container will fog its walls.
float dewPointF(float rh, float tF) {
  if (isnan(rh) || isnan(tF) || rh <= 0.0f) return NAN;
  const float a = 17.62f, b = 243.12f;
  float tC    = (tF - 32.0f) * 5.0f / 9.0f;
  float gamma = logf(rh / 100.0f) + (a * tC) / (b + tC);
  float dC    = (b * gamma) / (a - gamma);
  return dC * 9.0f / 5.0f + 32.0f;
}

// ============================================================
//  HISTORY ENCODING
// ============================================================
int16_t encTenths(float v) {
  if (isnan(v)) return HIST_NA;
  long t = lroundf(v * 10.0f);
  if (t >  32760) t =  32760;
  if (t < -32760) t = -32760;
  return (int16_t)t;
}

void histSetJson(JsonObject o, const char* key, int16_t enc) {
  if (enc == HIST_NA) o[key] = nullptr;
  else                o[key] = enc / 10.0f;
}

void pushHistory() {
  history[historyHead] = {
    encTenths(inside.hum),  encTenths(inside.tempF),
    encTenths(outside.hum), encTenths(outside.tempF),
    timeSynced() ? (uint32_t)time(nullptr) : 0u
  };
  historyHead = (historyHead + 1) % HISTORY_SIZE;
  if (historyCount < HISTORY_SIZE) historyCount++;
}

// ============================================================
//  SENSORS
// ============================================================
bool initSensor(Sensor &s, bool startBus) {
  if (startBus) {
    s.bus->begin(s.sda, s.scl, I2C_HZ);
    delay(50);
  }
  // begin() frees any previous i2c_dev, so re-init on failure does not leak.
  s.online = s.dev.begin(s.bus, s.id, AHTX0_I2CADDR_DEFAULT);
  Serial.printf("[AHT30/%s] %s\n", s.label, s.online ? "OK" : "Not found");
  return s.online;
}

// Reads one sensor, re-initializing once if the bus has dropped it (a loose
// lead or a brownout leaves the device wedged until it is re-addressed).
void readSensor(Sensor &s) {
  sensors_event_t humEv, tempEv;

  if (s.online && s.dev.getEvent(&humEv, &tempEv)) {
    s.hum   = humEv.relative_humidity;
    s.tempF = tempEv.temperature * 9.0f / 5.0f + 32.0f;
    return;
  }

  if (s.online) Serial.printf("[AHT30/%s] Read failed — re-init...\n", s.label);
  s.online = false;
  s.hum = s.tempF = NAN;

  if (initSensor(s, false) && s.dev.getEvent(&humEv, &tempEv)) {
    s.hum   = humEv.relative_humidity;
    s.tempF = tempEv.temperature * 9.0f / 5.0f + 32.0f;
  }
}

void readSensors() {
  readSensor(inside);
  readSensor(outside);
  lastSensorTime = getTimeString();

  if (!isnan(inside.hum) || !isnan(outside.hum)) pushHistory();

  Serial.printf("[Sensors] in %.1f%% / %.1fF   out %.1f%% / %.1fF\n",
                inside.hum, inside.tempF, outside.hum, outside.tempF);
}

// ============================================================
//  WiFi SIGNAL
// ============================================================
String wifiQuality(int rssi) {
  if (!WiFi.isConnected()) return "Disconnected";
  if (rssi > -50) return "Excellent";
  if (rssi > -60) return "Good";
  if (rssi > -70) return "Fair";
  return "Weak";
}

// ============================================================
//  STATUS JSON  (small — polled frequently)
// ============================================================
void addSensorJson(JsonObject o, const Sensor &s) {
  o["online"] = s.online;
  if (isnan(s.hum))   o["hum"]  = nullptr; else o["hum"]  = roundf(s.hum   * 10.0f) / 10.0f;
  if (isnan(s.tempF)) o["temp"] = nullptr; else o["temp"] = roundf(s.tempF * 10.0f) / 10.0f;
  float dp = dewPointF(s.hum, s.tempF);
  if (isnan(dp))      o["dew"]  = nullptr; else o["dew"]  = roundf(dp * 10.0f) / 10.0f;
}

String buildStatusJson() {
  int rssi = WiFi.isConnected() ? WiFi.RSSI() : 0;

  JsonDocument doc;
  doc["firmware"] = FIRMWARE_VERSION;
  doc["device"]   = deviceName;
  doc["mdns"]     = sanitizeMDNS(deviceName);
  doc["time"]     = getTimeString();
  doc["uptime"]   = (millis() - bootMillis) / 1000UL;
  doc["csrf"]     = csrfToken;
  doc["lastRead"] = lastSensorTime;

  addSensorJson(doc["inside"].to<JsonObject>(),  inside);
  addSensorJson(doc["outside"].to<JsonObject>(), outside);

  // Inside minus outside. Positive humidity delta = container is wetter than
  // the room, which is the condition that matters for a dry box.
  JsonObject d = doc["delta"].to<JsonObject>();
  if (!isnan(inside.hum) && !isnan(outside.hum))
    d["hum"]  = roundf((inside.hum - outside.hum) * 10.0f) / 10.0f;
  else d["hum"] = nullptr;
  if (!isnan(inside.tempF) && !isnan(outside.tempF))
    d["temp"] = roundf((inside.tempF - outside.tempF) * 10.0f) / 10.0f;
  else d["temp"] = nullptr;

  doc["rhAlert"]        = rhAlert;
  doc["readIntervalSec"] = READ_INTERVAL_MS / 1000UL;
  doc["historyCount"]   = historyCount;

  doc["ssid"]    = WiFi.isConnected() ? WiFi.SSID()               : "Not connected";
  doc["ip"]      = WiFi.isConnected() ? WiFi.localIP().toString() : "--";
  doc["mac"]     = WiFi.macAddress();
  doc["rssi"]    = rssi;
  doc["quality"] = wifiQuality(rssi);

  String out;
  serializeJson(doc, out);
  return out;
}

// ============================================================
//  HISTORY JSON  (downsampled — a full 4032-point payload would
//  be hundreds of KB in a single heap String)
// ============================================================
void handleHistory() {
  int maxPts = server.hasArg("max") ? server.arg("max").toInt() : HIST_MAX_PTS;
  if (maxPts < 2)            maxPts = 2;
  if (maxPts > HIST_MAX_PTS) maxPts = HIST_MAX_PTS;

  int stride = (historyCount + maxPts - 1) / maxPts;   // ceil
  if (stride < 1) stride = 1;

  JsonDocument doc;
  doc["count"]  = historyCount;
  doc["stride"] = stride;
  JsonArray pts = doc["pts"].to<JsonArray>();

  int start = (historyCount < HISTORY_SIZE) ? 0 : historyHead;
  for (int i = 0; i < historyCount; i += stride) {
    const HistPoint &p = history[(start + i) % HISTORY_SIZE];
    JsonObject o = pts.add<JsonObject>();
    o["ts"] = p.ts;
    histSetJson(o, "hi", p.hIn);
    histSetJson(o, "ti", p.tIn);
    histSetJson(o, "ho", p.hOut);
    histSetJson(o, "to", p.tOut);
  }

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// ============================================================
//  HISTORY CSV  (streamed in chunks — never built in one String)
// ============================================================
void appendTenths(String &dst, int16_t enc) {
  if (enc == HIST_NA) return;              // empty cell
  dst += String(enc / 10.0f, 1);
}

void handleHistoryCsv() {
  server.sendHeader("Content-Disposition", "attachment; filename=humidity-history.csv");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("epoch,datetime,inside_rh,inside_tempF,outside_rh,outside_tempF\n");

  String chunk;
  chunk.reserve(2048);

  int start = (historyCount < HISTORY_SIZE) ? 0 : historyHead;
  for (int i = 0; i < historyCount; i++) {
    const HistPoint &p = history[(start + i) % HISTORY_SIZE];

    chunk += String(p.ts);
    chunk += ',';
    if (p.ts) {
      time_t t = (time_t)p.ts;
      struct tm tmv;
      localtime_r(&t, &tmv);
      char buf[24];
      strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmv);
      chunk += buf;
    }
    chunk += ',';  appendTenths(chunk, p.hIn);
    chunk += ',';  appendTenths(chunk, p.tIn);
    chunk += ',';  appendTenths(chunk, p.hOut);
    chunk += ',';  appendTenths(chunk, p.tOut);
    chunk += '\n';

    if (chunk.length() > 1536) { server.sendContent(chunk); chunk = ""; }
  }
  if (chunk.length()) server.sendContent(chunk);
  server.sendContent("");   // terminate the chunked response
}

// ============================================================
//  WEB ROUTE HANDLERS
// ============================================================
void handleStatus() { server.send(200, "application/json", buildStatusJson()); }
void handleHealth() { server.send(200, "text/plain", "OK"); }

void handleSensorRefresh() {
  if (!checkCsrf()) return;
  readSensors();
  nextReadAt = millis() + READ_INTERVAL_MS;   // manual read resets the cadence
  server.send(200, "application/json", buildStatusJson());
}

void handleSettings() {
  if (!checkCsrf()) return;
  if (server.hasArg("rhAlert")) {
    float v = server.arg("rhAlert").toFloat();
    if (v >= 5.0f && v <= 90.0f) { rhAlert = v; saveSettings(); }
  }
  JsonDocument resp;
  resp["ok"]      = true;
  resp["rhAlert"] = rhAlert;
  String json; serializeJson(resp, json);
  server.send(200, "application/json", json);
}

void handleRename() {
  if (!checkCsrf()) return;
  if (!server.hasArg("name") || server.arg("name").isEmpty()) {
    server.send(400, "application/json", "{\"error\":\"Missing or empty name\"}");
    return;
  }
  String raw = server.arg("name"); raw.trim();
  deviceName = sanitizeMDNS(raw);
  saveName(deviceName);
  startMDNS();
  server.send(200, "application/json", "{\"ok\":true,\"name\":\"" + deviceName + "\"}");
}

void handleWiFiReset() {
  if (!checkCsrf()) return;
  server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Rebooting into setup mode.\"}");
  clearWiFiCreds();
  delay(1000);
  ESP.restart();
}

void handleWiFiChange() {
  if (!checkCsrf()) return;
  if (!server.hasArg("ssid") || server.arg("ssid").isEmpty()) {
    server.send(400, "application/json", "{\"error\":\"Missing SSID\"}");
    return;
  }
  saveWiFiCreds(server.arg("ssid"), server.hasArg("pass") ? server.arg("pass") : "");
  server.send(200, "application/json", "{\"ok\":true}");
  delay(800);
  ESP.restart();
}

void handleWiFiScanStart() {
  WiFi.scanNetworks(true);
  server.send(200, "application/json", "{\"ok\":true,\"status\":\"scanning\"}");
}

void handleWiFiScanResult() {
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) {
    server.send(200, "application/json", "{\"status\":\"scanning\"}");
    return;
  }
  JsonDocument doc;
  if (n < 0) {
    doc["status"] = "error";
  } else {
    doc["status"] = "done";
    JsonArray arr = doc["networks"].to<JsonArray>();
    for (int i = 0; i < n; i++) {
      JsonObject net = arr.add<JsonObject>();
      net["ssid"]   = WiFi.SSID(i);
      net["rssi"]   = WiFi.RSSI(i);
      net["secure"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
    }
  }
  WiFi.scanDelete();
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// ============================================================
//  AP SETUP PORTAL
// ============================================================
void startAPPortal() {
  inAPMode = true;

  // Append the last 2 bytes of the MAC so two freshly-flashed units don't both
  // broadcast the same "ESP32-Setup" — each board's AP is uniquely identifiable.
  String mac = WiFi.macAddress();          // AA:BB:CC:DD:EE:FF
  mac.replace(":", "");
  String apSsid = String(AP_SSID) + "-" + mac.substring(mac.length() - 4);

  Serial.printf("[WiFi] Starting AP: %s\n", apSsid.c_str());
  WiFi.mode(WIFI_AP);
  if (strlen(AP_PASS) >= 8) WiFi.softAP(apSsid.c_str(), AP_PASS);
  else                      WiFi.softAP(apSsid.c_str());
  Serial.printf("[WiFi] AP IP: %s\n", WiFi.softAPIP().toString().c_str());

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", R"HTML(
<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Humidity Monitor WiFi Setup</title>
<style>
body{background:#111;color:#eee;font-family:Arial,sans-serif;display:flex;
  justify-content:center;align-items:center;min-height:100vh;margin:0;padding:1rem;box-sizing:border-box}
.box{background:#1e1e2e;padding:2rem;border-radius:1rem;width:100%;max-width:340px;
  text-align:center;box-shadow:0 4px 24px rgba(0,0,0,.5)}
h2{color:#4f8ef7;margin-bottom:1.5rem}
input{width:100%;padding:.6rem;margin:.4rem 0 1rem;border-radius:.5rem;
  border:1px solid #444;background:#2a2a3e;color:#fff;font-size:1rem;box-sizing:border-box}
button{width:100%;padding:.7rem;background:#4f8ef7;color:#fff;border:none;
  border-radius:.5rem;font-size:1rem;cursor:pointer;margin-bottom:.6rem}
button:hover{background:#3a7de0}
.scan-btn{background:#252840}.scan-btn:hover{background:#333658}
.note{font-size:.8rem;color:#888;margin-top:.8rem}
.net-item{display:flex;justify-content:space-between;align-items:center;
  padding:.4rem .6rem;border-radius:.4rem;cursor:pointer;margin-bottom:.3rem;
  background:#252840;text-align:left}
.net-item:hover{background:#333658}
.lock{font-size:.75rem;margin-left:.4rem;color:#f7b84f}
#netList{margin-bottom:.8rem;max-height:220px;overflow-y:auto}
#scanning{color:#8892a4;font-size:.85rem;margin:.5rem 0}
</style></head><body>
<div class="box">
  <h2>WiFi Setup</h2>
  <button class="scan-btn" onclick="doScan()">Scan for Networks</button>
  <div id="scanning" style="display:none">Scanning...</div>
  <div id="netList"></div>
  <input id="ssidInput" placeholder="Selected or type SSID" autocomplete="off">
  <input id="passInput" placeholder="Password" type="password" autocomplete="new-password">
  <button onclick="doConnect()">Connect and Save</button>
  <p class="note">Device will restart and connect to your network.</p>
</div>
<script>
function doScan(){
  var list=document.getElementById('netList'),sc=document.getElementById('scanning');
  list.innerHTML='';sc.style.display='block';
  fetch('/wifi/scan').then(function(r){return r.json();}).then(function(nets){
    sc.style.display='none';
    if(!nets.length){list.innerHTML='<p style="color:#8892a4;font-size:.85rem">No networks found</p>';return;}
    nets.forEach(function(n){
      var d=document.createElement('div');d.className='net-item';
      d.innerHTML='<span>'+n.ssid+(n.secure?'<span class="lock">&#x1F512;</span>':'')+'</span>'
        +'<span style="font-size:.75rem;color:#8892a4">'+n.rssi+' dBm</span>';
      d.onclick=function(){document.getElementById('ssidInput').value=n.ssid;};
      list.appendChild(d);
    });
  }).catch(function(){sc.textContent='Scan failed — try again';});
}
function doConnect(){
  var s=document.getElementById('ssidInput').value.trim();
  var p=document.getElementById('passInput').value;
  if(!s){alert('Enter or select an SSID first.');return;}
  var fd=new FormData();fd.append('ssid',s);fd.append('password',p);
  fetch('/save',{method:'POST',body:fd}).then(function(r){return r.text();})
    .then(function(t){document.body.innerHTML=t;});
}
doScan();
</script></body></html>
)HTML");
  });

  server.on("/save", HTTP_POST, []() {
    if (!server.hasArg("ssid") || server.arg("ssid").isEmpty()) {
      server.send(400, "text/plain", "SSID is required");
      return;
    }
    String ssid = server.arg("ssid");
    String pass = server.hasArg("password") ? server.arg("password") : "";
    ssid.trim();
    saveWiFiCreds(ssid, pass);
    String mdnsName = sanitizeMDNS(deviceName);
    server.send(200, "text/html",
      String("<!DOCTYPE html><html><head>")
      + "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      + "<style>body{background:#111;color:#eee;font-family:Arial;text-align:center;padding-top:3rem}</style></head><body>"
      + "<h2 style='color:#3ecf8e'>Credentials Saved!</h2>"
      + "<p>Restarting...</p>"
      + "<p style='color:#888;font-size:.9rem'>Reconnect to your WiFi then visit<br>"
      + "<strong>http://" + mdnsName + ".local</strong></p></body></html>");
    delay(1500);
    ESP.restart();
  });

  server.on("/wifi/scan", HTTP_GET, []() {
    int n = WiFi.scanNetworks();
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < n; i++) {
      JsonObject net = arr.add<JsonObject>();
      net["ssid"]   = WiFi.SSID(i);
      net["rssi"]   = WiFi.RSSI(i);
      net["secure"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
    }
    WiFi.scanDelete();
    String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
  });

  server.onNotFound([]() {
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "");
  });

  server.begin();
}

// ============================================================
//  WiFi — STATION CONNECT
// ============================================================
bool connectWiFi() {
  String ssid, pass;
  loadWiFiCreds(ssid, pass);
  if (ssid.isEmpty()) { Serial.println("[WiFi] No saved credentials"); return false; }
  Serial.printf("[WiFi] Connecting to: %s\n", ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(ssid.c_str(), pass.c_str());
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(300); Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Connected  IP: %s  RSSI: %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return true;
  }
  Serial.println("[WiFi] Connection failed");
  return false;
}

// ============================================================
//  DASHBOARD
// ============================================================
void handleRoot() {
  server.send(200, "text/html", R"DASH(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Humidity Monitor</title>
<style>
:root{
  --bg:#0d0f1a;--card:#151724;--bdr:#252840;--acc:#4f8ef7;
  --grn:#3ecf8e;--red:#f76e6e;--warn:#f7b84f;--dim:#8892a4;--txt:#dde3f0;
  --in:#4fcfef;--out:#c08ef7;
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--txt);font-family:'Segoe UI',Arial,sans-serif;
     padding:1rem;max-width:600px;margin:auto}
@media(min-width:800px){body{max-width:900px}canvas{height:180px}}
h1{text-align:center;color:var(--acc);font-size:1.3rem;margin:.8rem 0 .3rem}
.card{background:var(--card);border:1px solid var(--bdr);border-radius:14px;
      padding:1.2rem 1.4rem;margin-bottom:1rem}
.card-hdr{display:flex;justify-content:space-between;align-items:center;margin-bottom:.9rem}
.card-hdr h2,.card>h2{font-size:.78rem;text-transform:uppercase;letter-spacing:.1em;
  color:var(--dim);margin-bottom:.9rem}
.row{display:flex;justify-content:space-between;align-items:center;
     padding:.35rem 0;border-bottom:1px solid var(--bdr);font-size:.9rem}
.row:last-child{border-bottom:none}
.lbl{color:var(--dim);font-size:.88rem}.val{font-weight:600;font-size:.92rem}
.twin{display:flex;gap:1rem}
.twin-item{flex:1;text-align:center;padding:.6rem .3rem;border-radius:10px;background:#11131f}
.twin-hdr{font-size:.72rem;letter-spacing:.08em;text-transform:uppercase;margin-bottom:.5rem;font-weight:700}
.twin-in{color:var(--in)}.twin-out{color:var(--out)}
.big{font-size:2rem;font-weight:700;line-height:1.15;transition:color .5s}
.sub{font-size:1.05rem;font-weight:600;color:var(--txt);margin-top:.15rem}
.dew{font-size:.72rem;color:var(--dim);margin-top:.35rem}
.badge{display:inline-block;padding:.2rem .75rem;border-radius:999px;
       font-size:.78rem;font-weight:600;background:#1e2535}
.badge.ok{color:var(--grn)}.badge.err{color:var(--red)}
.btn-row{display:flex;gap:.6rem;flex-wrap:wrap;margin-top:.85rem}
button{padding:.5rem .95rem;border:none;border-radius:8px;font-size:.86rem;
       font-weight:600;cursor:pointer;transition:opacity .15s}
button:hover{opacity:.83}button:active{opacity:.65}
.b-blue{background:var(--acc);color:#fff}
.b-red{background:var(--red);color:#fff}
.b-dim{background:var(--bdr);color:var(--txt)}
.b-sm{padding:.3rem .7rem;font-size:.78rem}
.irow{display:flex;gap:.5rem;margin-top:.7rem}
input[type=text],input[type=password],input[type=number]{flex:1;padding:.5rem .8rem;
  background:#1e2535;border:1px solid var(--bdr);border-radius:8px;
  color:var(--txt);font-size:.9rem;outline:none}
input:focus{border-color:var(--acc)}
.input-sm{width:5rem;flex:none;padding:.3rem .5rem;text-align:center}
.last-read{font-size:.72rem;color:var(--dim);text-align:right;margin-top:.5rem}
#upd{text-align:center;font-size:.73rem;color:var(--dim);margin-top:.4rem}
.net-list{margin-top:.7rem;max-height:180px;overflow-y:auto}
.net-item{display:flex;justify-content:space-between;align-items:center;
          padding:.4rem .6rem;border-radius:.4rem;cursor:pointer;
          margin-bottom:.3rem;background:#1e2535}
.net-item:hover{background:var(--bdr)}
.lock{font-size:.72rem;margin-left:.3rem;color:var(--warn)}
#scanStatus{font-size:.82rem;color:var(--dim);margin:.4rem 0;text-align:center}
canvas{width:100%;height:120px;display:block;margin-top:.2rem}
.uptime{font-size:.75rem;color:var(--dim);text-align:center;margin-bottom:.2rem}
.chart-hdr{display:flex;justify-content:space-between;align-items:center;
  font-size:.72rem;color:var(--dim);margin-top:.9rem;margin-bottom:.1rem}
.legend{display:flex;gap:.7rem}
.legend span{display:flex;align-items:center;gap:.28rem}
.dot{width:.55rem;height:.55rem;border-radius:50%;display:inline-block}
.dirty{color:var(--warn);font-size:.8rem;align-self:center;display:none}
.ota-link{display:block;text-align:center;color:var(--acc);text-decoration:none;
          font-size:.92rem;font-weight:600;padding:.4rem 0}
.net-card>summary{list-style:none;cursor:pointer;display:flex;align-items:center;gap:.6rem}
.net-card>summary::-webkit-details-marker{display:none}
.net-card>summary h2{margin-bottom:0}
.summ-info{margin-left:auto;font-size:.8rem;color:var(--dim);
           white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.chev{color:var(--dim);font-size:.7rem;transition:transform .2s;flex:none}
.net-card[open]>summary .chev{transform:rotate(90deg)}
.net-card[open]>summary .summ-info{display:none}
</style>
</head>
<body>

<h1 id="hTitle">Humidity Monitor</h1>
<div class="uptime" id="uptimeEl"></div>
<div class="uptime" id="versionEl"></div>

<!-- READINGS -->
<div class="card">
  <div class="card-hdr">
    <h2>Readings</h2>
    <button class="b-dim b-sm" onclick="refreshSensors()">&#x21BB; Refresh</button>
  </div>
  <div class="twin">
    <div class="twin-item">
      <div class="twin-hdr twin-in">Filament Box Sensor</div>
      <div class="big" id="inHum">--%</div>
      <div class="sub" id="inTemp">--&deg;F</div>
      <div class="dew">Dew point <span id="inDew">--</span></div>
    </div>
    <div class="twin-item">
      <div class="twin-hdr twin-out">Outside Air Sensor</div>
      <div class="big" id="outHum">--%</div>
      <div class="sub" id="outTemp">--&deg;F</div>
      <div class="dew">Dew point <span id="outDew">--</span></div>
    </div>
  </div>

  <div class="row" style="margin-top:.9rem"><span class="lbl">Difference (Filament Box &minus; Outside Air)</span>
    <span class="val" id="delta">--</span></div>
  <div class="row"><span class="lbl">Filament Box sensor</span>
    <span id="inBadge" class="badge">--</span></div>
  <div class="row"><span class="lbl">Outside Air sensor</span>
    <span id="outBadge" class="badge">--</span></div>
  <div class="last-read">Last read: <span id="lastRead">--</span></div>

  <div class="chart-hdr">
    <span>Humidity (%) &mdash; 2 weeks</span>
    <span class="legend">
      <span><i class="dot" style="background:#4fcfef"></i>Box</span>
      <span><i class="dot" style="background:#c08ef7"></i>Air</span>
    </span>
  </div>
  <canvas id="humChart"></canvas>

  <div class="chart-hdr">
    <span>Temperature (&deg;F) &mdash; 2 weeks</span>
    <span class="legend">
      <span><i class="dot" style="background:#4fcfef"></i>Box</span>
      <span><i class="dot" style="background:#c08ef7"></i>Air</span>
    </span>
  </div>
  <canvas id="tempChart"></canvas>

  <div class="btn-row">
    <button class="b-dim" onclick="location.href='/history.csv'">Download CSV</button>
    <span style="align-self:center;font-size:.75rem;color:var(--dim)"
          id="histCount"></span>
  </div>
</div>

<!-- NETWORK -->
<details class="card net-card">
  <summary>
    <h2>Network</h2>
    <span class="summ-info" id="netSummary">--</span>
    <span class="chev">&#x25B6;</span>
  </summary>
  <div class="net-body">
  <div class="row"><span class="lbl">SSID</span><span class="val" id="ssid">--</span></div>
  <div class="row"><span class="lbl">IP address</span><span class="val" id="ip">--</span></div>
  <div class="row"><span class="lbl">MAC address</span><span class="val" id="mac">--</span></div>
  <div class="row"><span class="lbl">RSSI</span>
    <span class="val"><span id="rssi">--</span> dBm</span></div>
  <div class="row"><span class="lbl">Signal</span><span class="val" id="quality">--</span></div>
  <div class="row"><span class="lbl">mDNS hostname</span>
    <span class="val" id="mdnsLabel">--</span></div>

  <h2 style="margin-top:1.1rem">Change WiFi Network</h2>
  <button class="b-dim" style="width:100%;margin-top:.5rem" onclick="doScan()">Scan for networks</button>
  <div id="scanStatus"></div>
  <div class="net-list" id="netList"></div>
  <div class="irow">
    <input type="text" id="wifiSSID" placeholder="Selected or type SSID" autocomplete="off">
    <input type="password" id="wifiPass" placeholder="Password" autocomplete="new-password">
  </div>
  <div class="btn-row" style="margin-top:.7rem">
    <button class="b-blue" onclick="changeWiFi()">Connect to network</button>
  </div>

  <h2 style="margin-top:1.1rem">Change Device Name / mDNS</h2>
  <div class="irow">
    <input type="text" id="renameInput" placeholder="new-hostname">
    <button class="b-blue" onclick="renameDevice()">Save</button>
  </div>
  <p style="color:var(--dim);font-size:.76rem;margin-top:.5rem">
    Lowercase letters, numbers and hyphens only. mDNS updates immediately.
  </p>
  <div class="btn-row" style="margin-top:1rem">
    <button class="b-red" onclick="resetWiFi()">Reset WiFi credentials</button>
  </div>
  </div>
</details>

<!-- OTA -->
<div class="card">
  <h2>Firmware Update</h2>
  <a class="ota-link" href="/update">Open OTA update page &#x2197;</a>
</div>

<div id="upd">Connecting...</div>

<script>
var $=function(id){return document.getElementById(id);};
var csrf='';
var C_IN='rgb(79,207,239)', C_OUT='rgb(192,142,247)';

function toast(msg,ms){
  ms=ms||2500;
  var d=document.createElement('div');
  d.textContent=msg;
  d.style.cssText='position:fixed;bottom:1.4rem;left:50%;transform:translateX(-50%);'
    +'background:#252840;color:#dde3f0;padding:.55rem 1.1rem;border-radius:8px;'
    +'font-size:.86rem;z-index:9999;box-shadow:0 2px 12px rgba(0,0,0,.4);opacity:1;transition:opacity .4s';
  document.body.appendChild(d);
  setTimeout(function(){d.style.opacity='0';setTimeout(function(){d.parentNode&&d.parentNode.removeChild(d);},500);},ms);
}

function api(url){
  var sep=url.indexOf('?')<0?'?':'&';
  return fetch(url+sep+'csrf='+encodeURIComponent(csrf));
}

function uptimeStr(s){
  var d=Math.floor(s/86400),h=Math.floor((s%86400)/3600),m=Math.floor((s%3600)/60),sec=s%60;
  if(d>0) return d+'d '+h+'h '+m+'m';
  if(h>0) return h+'h '+m+'m '+sec+'s';
  if(m>0) return m+'m '+sec+'s';
  return sec+'s';
}

function fmtTick(unixSec){
  var dt=new Date(unixSec*1000),h=dt.getHours(),m=dt.getMinutes();
  return (h<10?'0':'')+h+':'+(m<10?'0':'')+m;
}

// ── Two-series chart ──────────────────────────────────────
// pts: [{ts, a, b}] where a/b may be null (sensor offline at that time).
// Nulls break the line rather than interpolating across a gap.
function drawDual(canvasId,pts,unit){
  var cv=$(canvasId);
  if(!cv||!pts||pts.length<2) return;
  var dpr=window.devicePixelRatio||1;
  var LEFT=36,AXIS=18,CH=cv.offsetHeight||120;
  cv.width=cv.offsetWidth*dpr;cv.height=CH*dpr;
  var ctx=cv.getContext('2d');ctx.scale(dpr,dpr);
  var W=cv.offsetWidth,plotW=W-LEFT,plotH=CH-AXIS;

  var vals=[];
  pts.forEach(function(p){
    if(p.a!=null) vals.push(p.a);
    if(p.b!=null) vals.push(p.b);
  });
  if(vals.length<2) return;
  var mn=Math.min.apply(null,vals),mx=Math.max.apply(null,vals);
  var pad=Math.max((mx-mn)*0.2,2);mn-=pad;mx+=pad;
  if(mn===mx){mn-=1;mx+=1;}

  ctx.clearRect(0,0,W,CH);
  ctx.strokeStyle='rgba(255,255,255,0.06)';ctx.lineWidth=0.5;
  ctx.beginPath();ctx.moveTo(LEFT,0);ctx.lineTo(LEFT,plotH);ctx.stroke();

  ctx.fillStyle='rgba(136,146,164,0.9)';ctx.font='10px sans-serif';ctx.textAlign='right';
  function fmtY(v){return(mx-mn<20?v.toFixed(1):Math.round(v))+(unit||'');}
  ctx.fillText(fmtY(mx),LEFT-3,10);
  ctx.fillText(fmtY((mx+mn)/2),LEFT-3,plotH/2+4);
  ctx.fillText(fmtY(mn),LEFT-3,plotH-2);

  ctx.strokeStyle='rgba(255,255,255,0.04)';ctx.lineWidth=0.5;
  [0,0.5,1].forEach(function(frac){
    var y=plotH-(frac*(plotH-16))-4;
    ctx.beginPath();ctx.moveTo(LEFT,y);ctx.lineTo(W,y);ctx.stroke();
  });

  function xAt(i){return LEFT+(pts.length===1?0:i/(pts.length-1)*plotW);}
  function yAt(v){return plotH-(v-mn)/(mx-mn)*(plotH-16)-4;}

  function line(key,color){
    ctx.beginPath();
    var pen=false;
    pts.forEach(function(p,i){
      var v=p[key];
      if(v==null){pen=false;return;}
      if(!pen){ctx.moveTo(xAt(i),yAt(v));pen=true;}
      else ctx.lineTo(xAt(i),yAt(v));
    });
    ctx.strokeStyle=color;ctx.lineWidth=2;ctx.stroke();
  }
  line('a',C_IN);
  line('b',C_OUT);

  ctx.strokeStyle='rgba(255,255,255,0.07)';ctx.lineWidth=0.5;
  ctx.beginPath();ctx.moveTo(LEFT,plotH);ctx.lineTo(W,plotH);ctx.stroke();

  var labeled=pts.filter(function(p){return p.ts>0;});
  if(!labeled.length) return;
  ctx.fillStyle='rgba(136,146,164,0.9)';ctx.font='10px sans-serif';
  var numLabels=Math.min(6,pts.length);
  for(var li=0;li<numLabels;li++){
    var idx=Math.round(li/(numLabels-1)*(pts.length-1));
    if(!pts[idx].ts) continue;
    ctx.textAlign=li===0?'left':li===numLabels-1?'right':'center';
    ctx.fillText(fmtTick(pts[idx].ts),xAt(idx),CH-3);
  }
}

// ── Rendering ─────────────────────────────────────────────
function fmtNum(v,suffix,digits){
  return v==null?'--':v.toFixed(digits==null?1:digits)+suffix;
}

function applySide(prefix,s,alert){
  var h=$(prefix+'Hum'),t=$(prefix+'Temp');
  if(s.hum==null){h.textContent='--%';h.style.color='var(--dim)';}
  else{
    h.textContent=s.hum.toFixed(1)+'%';
    // Only the inside reading is judged against the alert threshold; the
    // outside reading is ambient and has no "good" or "bad" value.
    if(alert==null) h.style.color='var(--txt)';
    else h.style.color=s.hum>alert?'var(--red)':s.hum>alert*0.85?'var(--warn)':'var(--grn)';
  }
  t.textContent=fmtNum(s.temp,'°F');
  $(prefix+'Dew').textContent=fmtNum(s.dew,'°F');

  var b=$(prefix+'Badge');
  if(s.online){b.textContent='AHT30 Online';b.className='badge ok';}
  else{b.textContent='Sensor Offline';b.className='badge err';}
}

function applyStatus(d){
  csrf=d.csrf||csrf;
  $('hTitle').textContent=d.device||'Humidity Monitor';
  document.title=d.device||'Humidity Monitor';
  $('uptimeEl').textContent='Uptime: '+uptimeStr(d.uptime||0);
  $('versionEl').textContent='Firmware: '+(d.firmware||'');

  applySide('in',  d.inside,  null);
  applySide('out', d.outside, null);

  var dh=d.delta?d.delta.hum:null, dt=d.delta?d.delta.temp:null;
  var parts=[];
  if(dh!=null) parts.push((dh>0?'+':'')+dh.toFixed(1)+'% RH');
  if(dt!=null) parts.push((dt>0?'+':'')+dt.toFixed(1)+'°F');
  $('delta').textContent=parts.length?parts.join('  /  '):'--';

  $('lastRead').textContent=d.lastRead||'--';
  $('histCount').textContent=(d.historyCount||0)+' points logged';

  $('netSummary').textContent=(d.ssid||'--')+' · '+(d.ip||'--');
  $('ssid').textContent     =d.ssid;
  $('ip').textContent       =d.ip;
  $('mac').textContent      =d.mac;
  $('rssi').textContent     =d.rssi;
  $('quality').textContent  =d.quality;
  $('mdnsLabel').textContent=(d.mdns||'')+'.local';

  $('upd').textContent='Updated: '+new Date().toLocaleTimeString();
}

// Last /history payload, kept so a resize can redraw without re-fetching.
var histPts=null;

function drawCharts(){
  if(!histPts||histPts.length<2) return;
  drawDual('humChart', histPts.map(function(p){return{ts:p.ts,a:p.hi,b:p.ho};}), '%');
  drawDual('tempChart',histPts.map(function(p){return{ts:p.ts,a:p.ti,b:p.to};}), '°F');
}

function loadHistory(){
  fetch('/history').then(function(r){return r.json();}).then(function(d){
    histPts=d.pts||null;
    drawCharts();
  }).catch(function(){});
}

function update(){
  fetch('/status').then(function(r){return r.json();}).then(applyStatus)
    .catch(function(){$('upd').textContent='Connection lost — retrying...';});
}

// ── Actions ───────────────────────────────────────────────
function refreshSensors(){
  $('upd').textContent='Refreshing...';
  api('/sensor/refresh').then(function(r){return r.json();})
    .then(function(d){
      applyStatus(d);
      loadHistory();
      $('upd').textContent='Manually refreshed: '+new Date().toLocaleTimeString();
    })
    .catch(function(){toast('Refresh failed');});
}
function renameDevice(){
  var name=$('renameInput').value.trim();
  if(!name){toast('Enter a name first');return;}
  api('/device/rename?name='+encodeURIComponent(name))
    .then(function(r){return r.json();})
    .then(function(d){if(d.ok){toast('Renamed to: '+d.name);$('renameInput').value='';update();}
                     else toast('Error: '+(d.error||'unknown'));});
}
function changeWiFi(){
  var ssid=$('wifiSSID').value.trim(),pass=$('wifiPass').value;
  if(!ssid){toast('Enter or select a network first');return;}
  var fd=new FormData();fd.append('ssid',ssid);fd.append('pass',pass);fd.append('csrf',csrf);
  fetch('/wifi/change',{method:'POST',body:fd}).then(function(){toast('Saved. Rebooting...',4000);});
}
function resetWiFi(){
  api('/wifi/reset').then(function(){toast('Rebooting into setup mode...',4000);});
}

// ── WiFi scan ─────────────────────────────────────────────
var scanTimer=null;
function signalBars(r){return r>-50?'||||':r>-60?'||| ':r>-70?'||  ':'|   ';}
function doScan(){
  $('scanStatus').textContent='Starting scan...';$('netList').innerHTML='';
  fetch('/wifi/scan/start').then(function(){
    $('scanStatus').textContent='Scanning... (5-10 s)';
    clearInterval(scanTimer);scanTimer=setInterval(pollScan,2000);
  }).catch(function(){$('scanStatus').textContent='Scan request failed';});
}
function pollScan(){
  fetch('/wifi/scan/result').then(function(r){return r.json();}).then(function(d){
    if(d.status==='scanning') return;
    clearInterval(scanTimer);
    if(d.status==='error'||!d.networks){$('scanStatus').textContent='Scan failed';return;}
    $('scanStatus').textContent=d.networks.length+' network'+(d.networks.length!==1?'s':'')+' found';
    d.networks.forEach(function(n){
      var div=document.createElement('div');div.className='net-item';
      div.innerHTML='<span>'+n.ssid+(n.secure?'<span class="lock">&#x1F512;</span>':'')+'</span>'
        +'<span style="font-size:.75rem;color:var(--dim)">'+signalBars(n.rssi)+' '+n.rssi+' dBm</span>';
      div.onclick=function(){$('wifiSSID').value=n.ssid;$('wifiPass').focus();};
      $('netList').appendChild(div);
    });
  }).catch(function(){clearInterval(scanTimer);$('scanStatus').textContent='Poll failed';});
}

// Status is cheap; history is not. Poll them on separate cadences.
setInterval(update,      15000);
setInterval(loadHistory,300000);

// Redraw from the cached payload — resize fires continuously while dragging,
// and re-fetching history on each event would swamp the device.
var resizeTimer=null;
window.addEventListener('resize',function(){
  clearTimeout(resizeTimer);
  resizeTimer=setTimeout(drawCharts,150);
});

update();
loadHistory();
</script>
</body>
</html>
)DASH");
}

// ============================================================
//  ROUTE REGISTRATION
// ============================================================
void setupRoutes() {
  server.on("/",                 HTTP_GET,  handleRoot);
  server.on("/status",           HTTP_GET,  handleStatus);
  server.on("/history",          HTTP_GET,  handleHistory);
  server.on("/history.csv",      HTTP_GET,  handleHistoryCsv);
  server.on("/health",           HTTP_GET,  handleHealth);
  server.on("/sensor/refresh",   HTTP_GET,  handleSensorRefresh);
  server.on("/settings",         HTTP_GET,  handleSettings);
  server.on("/device/rename",    HTTP_GET,  handleRename);
  server.on("/wifi/scan/start",  HTTP_GET,  handleWiFiScanStart);
  server.on("/wifi/scan/result", HTTP_GET,  handleWiFiScanResult);
  server.on("/wifi/change",      HTTP_POST, handleWiFiChange);
  server.on("/wifi/reset",       HTTP_GET,  handleWiFiReset);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });

  // checkCsrf() falls back to this header when there is no ?csrf= arg, but
  // WebServer discards headers it was not told to keep.
  static const char* keep[] = { "X-CSRF-Token" };
  server.collectHeaders(keep, 1);
}

// ============================================================
//  ArduinoOTA
// ============================================================
void setupArduinoOTA() {
  ArduinoOTA.setHostname(sanitizeMDNS(deviceName).c_str());
  ArduinoOTA.onStart([]() {
    esp_task_wdt_delete(NULL);
    Serial.println("[ArduinoOTA] Start");
  });
  ArduinoOTA.onEnd([]()  { Serial.println("\n[ArduinoOTA] Done"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    if (t > 0) Serial.printf("[ArduinoOTA] %u%%\r", p * 100 / t);
  });
  ArduinoOTA.onError([](ota_error_t err) {
    const char* msgs[] = {"","Auth failed","Begin failed","Connect failed","Receive failed","End failed"};
    Serial.printf("[ArduinoOTA] Error[%u]: %s\n", err, err <= 5 ? msgs[err] : "Unknown");
  });
  ArduinoOTA.begin();
  otaRunning = true;
  Serial.println("[ArduinoOTA] Ready on port 3232");
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] ESP32-S3 Humidity Monitor " FIRMWARE_VERSION);
  bootMillis = millis();

  esp_task_wdt_config_t wdt_cfg = { .timeout_ms = 30000, .idle_core_mask = 0, .trigger_panic = true };
  esp_task_wdt_init(&wdt_cfg);
  esp_task_wdt_add(NULL);
  Serial.println("[WDT] Enabled — 30 s timeout");

  loadPrefs();
  generateCsrfToken();

  initSensor(inside,  true);
  initSensor(outside, true);

  String ssid, pass;
  loadWiFiCreds(ssid, pass);
  bool connected = !ssid.isEmpty() && connectWiFi();

  if (!connected) {
    Serial.println(ssid.isEmpty()
      ? "[WiFi] No credentials — starting AP portal"
      : "[WiFi] Could not connect — starting AP portal");
    startAPPortal();
    return;
  }

  setupTime();
  setupPowerManagement();
  startMDNS();
  setupArduinoOTA();
  setupRoutes();
  ElegantOTA.begin(&server, OTA_USER, OTA_PASS);
  server.begin();
  Serial.println("[HTTP] Server started");

  nextReadAt = 0;   // read immediately
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  server.handleClient();
  ElegantOTA.loop();
  if (otaRunning) ArduinoOTA.handle();
  esp_task_wdt_reset();

  if (!inAPMode
      && WiFi.getMode() == WIFI_STA
      && WiFi.status() != WL_CONNECTED
      && millis() - lastWiFiRetry >= WIFI_RETRY_INTERVAL)
  {
    lastWiFiRetry = millis();
    Serial.println("[WiFi] Reconnecting...");
    String ssid, pass;
    loadWiFiCreds(ssid, pass);
    WiFi.begin(ssid.c_str(), pass.c_str());
  }

  if (!inAPMode && millis() >= nextReadAt) {
    readSensors();
    nextReadAt = millis() + READ_INTERVAL_MS;
  }

  delay(50);
}
