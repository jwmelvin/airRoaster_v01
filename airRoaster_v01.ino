// ===========================================================================
// airRoaster v01 — ESP32-S3 hot-air coffee roaster controller
//
// Firmware version: see FW_VERSION below.
// Change history:   bottom of this file.
// (Intentionally no "last edited" date in this header — it only goes stale.
//  The top entry of the version history is the real last-changed date.)
// ===========================================================================
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Adafruit_MAX31855.h>
#include <Adafruit_MAX31865.h>

// ---------------------------------------------------------------------------
// Firmware identity
// ---------------------------------------------------------------------------
#define FW_VERSION  "0.2.1"

// ---------------------------------------------------------------------------
// WiFi credentials (defined in secrets.h — do not commit that file)
// ---------------------------------------------------------------------------
#include "secrets.h"

// ---------------------------------------------------------------------------
// Sensor SPI chip-select pins  (assign free GPIOs to suit your wiring)
// ---------------------------------------------------------------------------
#define CS_RTD_BT   10   // MAX31865 (product 3648) — bean temp RTD (connected probe)
#define CS_RTD_ET   9    // MAX31865 (product 3648) — exhaust/ET RTD (no probe yet — disabled below)
#define CS_TC_IN    8    // MAX31855 (product 269)  — inlet thermocouple

// Set to 1 when the ET RTD probe is installed and filtered
#define RTD_ET_ENABLED  0

// MAX31865 wiring type: MAX31865_2WIRE / MAX31865_3WIRE / MAX31865_4WIRE
#define RTD_WIRES   MAX31865_4WIRE

// PT1000 reference resistor on the 3648 board is 4300 Ω; nominal at 0 °C is 1000 Ω
#define RTD_REF     4300.0f
#define RTD_NOMINAL 1000.0f

// Sensor poll interval (ms) — much shorter than Artisan's sample rate
#define SENSOR_INTERVAL_MS  250

// Robust-read parameters (dimmer EMI tolerance — see hardware/emi.md)
#define RTD_VALID_MIN_C       -20.0f  // plausibility window for accepting a reading
#define RTD_VALID_MAX_C        600.0f
#define RTD_READ_DISAGREE_C    2.0f   // two same-cycle reads differing by more => SPI glitch, hold
#define SENSOR_FAULT_DEBOUNCE  4      // consecutive bad reads before a channel is declared faulted
#define SENSOR_REFAULT_LOG_MS  5000   // min interval between repeated fault-log entries (anti-flood)
#define RTD_MEDIAN_WINDOW      5      // samples in the median filter (set 1 to disable)
#define RTD_REASSERT_MS        5000   // re-assert VBIAS/auto-convert this often (silent-stall guard)

// ---------------------------------------------------------------------------
// DimmerLink I2C addresses and registers
// ---------------------------------------------------------------------------
#define HEAT_ADDR   0x51
#define FAN_ADDR    0x52

#define REG_STATUS  0x00
#define REG_ERROR   0x02
#define REG_LEVEL   0x10
#define REG_CURVE   0x11
#define REG_FREQ    0x20

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------
#define DUTY_STEP               5
#define WS_PORT                 81
#define DL_INIT_RETRY_DELAY_MS  500   // ms between initDL ready-check retries

// Interlock config
#define IL_FAN_MIN      48   // fan level below which heat is always 0
#define IL_FAN_FULL     55   // fan level at or above which heat is unrestricted
#define IL_HEAT_AT_MIN  30   // heat cap (%) when fan is exactly at IL_FAN_MIN (soft mode)

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
uint8_t requestedHeatLevel = 0;  // commanded heat level
uint8_t heatLevel = 0;           // actual heat level sent to device
uint8_t fanLevel = 0;
bool    interlockSoft = false;   // false = hard (binary), true = soft (linear ramp)

float btTemp  = 0.0;  // bean temp  — MAX31865 PT1000 (°C)
float etTemp  = 0.0;  // exhaust/ET — MAX31865 PT1000 (°C)
float inTemp  = 0.0;  // inlet temp — MAX31855 thermocouple (°C)

String inputBuffer = "";
volatile bool flagDisplayUpdate;

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------
static Adafruit_MAX31865 rtdBT(CS_RTD_BT);
static Adafruit_MAX31865 rtdET(CS_RTD_ET);
static Adafruit_MAX31855 tcIN(CS_TC_IN);

// Per-channel fault tracking (debounce + rate-limited logging) and, for the
// RTDs, a median-filter ring buffer and the stall-guard re-assert timer.
struct SensorFaultState {
    uint8_t  badCount;                // consecutive bad reads
    bool     faulted;                 // currently in the (logged) faulted state
    uint32_t lastLogMs;               // last time a fault was logged for this channel
    uint32_t lastReassertMs;          // last VBIAS/auto-convert re-assert (RTD only)
    float    hist[RTD_MEDIAN_WINDOW]; // recent accepted readings (RTD only)
    uint8_t  histHead;                // next write index into hist[]
    uint8_t  histCount;              // valid entries in hist[]
};
static SensorFaultState rtdBTfault = {0, false, 0, 0, {0}, 0, 0};
static SensorFaultState rtdETfault = {0, false, 0, 0, {0}, 0, 0};
static SensorFaultState tcINfault  = {0, false, 0, 0, {0}, 0, 0};

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);
#define COL1A     0
#define COL2A     64
#define ROWHEIGHT 8

enum rows_t { ROW1, ROW2, ROW3, ROW4, ROW5, ROW6, ROW7, ROW8 };

// ---------------------------------------------------------------------------
// WebSocket server
// ---------------------------------------------------------------------------
WebSocketsServer webSocket(WS_PORT);

// ---------------------------------------------------------------------------
// Error log (RAM only — no flash writes)
// ---------------------------------------------------------------------------
#define ERR_LOG_SIZE    8
#define ERR_MSG_LEN     64

static char  errLog[ERR_LOG_SIZE][ERR_MSG_LEN];
static uint8_t errHead = 0;   // next write index
static uint8_t errCount = 0;  // total entries stored (capped at ERR_LOG_SIZE)

void logError(const char* msg) {
    strncpy(errLog[errHead], msg, ERR_MSG_LEN - 1);
    errLog[errHead][ERR_MSG_LEN - 1] = '\0';
    errHead = (errHead + 1) % ERR_LOG_SIZE;
    if (errCount < ERR_LOG_SIZE) errCount++;

    Serial.print("[ERR] ");
    Serial.println(msg);

    char buf[ERR_MSG_LEN + 28];
    snprintf(buf, sizeof(buf), "{\"pushMessage\":\"error\",\"data\":\"%s\"}", msg);
    webSocket.broadcastTXT(buf);
}

void sendLog(uint8_t clientNum) {
    uint8_t start = (errCount < ERR_LOG_SIZE) ? 0 : errHead;
    for (uint8_t i = 0; i < errCount; i++) {
        uint8_t idx = (start + i) % ERR_LOG_SIZE;
        char buf[ERR_MSG_LEN + 32];
        snprintf(buf, sizeof(buf), "{\"pushMessage\":\"log\",\"data\":\"%s\"}", errLog[idx]);
        webSocket.sendTXT(clientNum, buf);
    }
    if (errCount == 0) {
        webSocket.sendTXT(clientNum, "{\"pushMessage\":\"log\",\"data\":\"no errors\"}");
    }
}

// ---------------------------------------------------------------------------
// JSON field helpers
// ---------------------------------------------------------------------------

// Extract a quoted string field from a flat JSON object.
// Returns "" if not found.
static String jsonString(const String& json, const char* key) {
    String search = String("\"") + key + "\"";
    int pos = json.indexOf(search);
    if (pos < 0) return "";
    int colon = json.indexOf(':', pos + search.length());
    if (colon < 0) return "";
    int q1 = json.indexOf('"', colon + 1);
    if (q1 < 0) return "";
    int q2 = json.indexOf('"', q1 + 1);
    if (q2 < 0) return "";
    return json.substring(q1 + 1, q2);
}

// Extract a numeric field from a flat JSON object.
// Returns NAN if not found.
static float jsonFloat(const String& json, const char* key) {
    String search = String("\"") + key + "\"";
    int pos = json.indexOf(search);
    if (pos < 0) return NAN;
    int colon = json.indexOf(':', pos + search.length());
    if (colon < 0) return NAN;
    int start = colon + 1;
    while (start < (int)json.length() && json[start] == ' ') start++;
    return json.substring(start).toFloat();
}

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void processCommand(String cmd, int8_t clientNum = -1);
void applyInterlock();
void broadcastStatus();
bool handleArtisanRequest(uint8_t clientNum, const String& msg);
void initSensors();
void updateSensors();
void serviceRtd(Adafruit_MAX31865 &dev, SensorFaultState &st, float &out, const char *name);

// ===========================================================================
// Sensors — init and periodic update
// ===========================================================================

void initSensors() {
    tcIN.begin();

    rtdBT.begin(RTD_WIRES);
    rtdET.begin(RTD_WIRES);

    // Clear any startup faults first, then enable continuous conversion.
    // Order matters: clearFault() resets config bits, so bias and autoConvert must come after.
    rtdBT.clearFault();
    rtdBT.enableBias(true);
    rtdBT.autoConvert(true);
#if RTD_ET_ENABLED
    rtdET.clearFault();
    rtdET.enableBias(true);
    rtdET.autoConvert(true);
#endif

    Serial.println("Sensors initialized");
}

// Median of the valid entries in a channel's history ring (order doesn't matter).
static float rtdMedian(const SensorFaultState &st) {
    float tmp[RTD_MEDIAN_WINDOW];
    uint8_t n = st.histCount;
    for (uint8_t i = 0; i < n; i++) tmp[i] = st.hist[i];
    for (uint8_t i = 1; i < n; i++) {          // insertion sort (n is tiny)
        float key = tmp[i];
        int8_t j = i - 1;
        while (j >= 0 && tmp[j] > key) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = key;
    }
    return tmp[n / 2];
}

// Robust RTD read. Rejects single-cycle SPI glitches (two disagreeing reads),
// median-filters accepted samples (rejects single-sample spikes), debounces
// genuine faults, holds the last good value through brief transients, periodically
// re-asserts VBIAS/auto-convert to guard against a silently-stalled converter, and
// keeps the converter alive after a fault. See hardware/emi.md for the noise story.
void serviceRtd(Adafruit_MAX31865 &dev, SensorFaultState &st, float &out, const char *name) {
    uint32_t now = millis();

    // Stall guard: a transient can silently clear VBIAS/auto-convert in the config
    // register, freezing conversions while temperature() keeps returning the last
    // latched (stale-but-plausible) value. Re-assert them periodically so the
    // converter always recovers within RTD_REASSERT_MS even with no fault flag set.
    if (now - st.lastReassertMs >= RTD_REASSERT_MS) {
        st.lastReassertMs = now;
        dev.enableBias(true);
        dev.autoConvert(true);
    }

    // Two reads this cycle: if they disagree wildly, the SPI frame was probably
    // corrupted by a dimmer transient — hold the last good value, retry next cycle.
    float t1 = dev.temperature(RTD_NOMINAL, RTD_REF);
    float t2 = dev.temperature(RTD_NOMINAL, RTD_REF);
    if (fabsf(t1 - t2) > RTD_READ_DISAGREE_C) return;

    float t = 0.5f * (t1 + t2);

    // Good reading — accept even if a transient fault flag was momentarily set.
    if (t > RTD_VALID_MIN_C && t < RTD_VALID_MAX_C) {
        st.hist[st.histHead] = t;                     // push into the median ring
        st.histHead = (st.histHead + 1) % RTD_MEDIAN_WINDOW;
        if (st.histCount < RTD_MEDIAN_WINDOW) st.histCount++;
        out = rtdMedian(st);                          // output the median, not the raw sample
        st.badCount = 0;
        st.faulted  = false;
        return;
    }

    // Out of range and consistent across both reads — candidate genuine fault.
    if (st.badCount < 255) st.badCount++;

    uint8_t fault = dev.readFault();
    dev.clearFault();              // recover the converter so it keeps running
    dev.enableBias(true);
    dev.autoConvert(true);
    // 'out' intentionally holds its last good value (do not zero) during the fault.

    if (st.badCount < SENSOR_FAULT_DEBOUNCE) return;  // ignore brief transients

    if (!st.faulted || (now - st.lastLogMs) >= SENSOR_REFAULT_LOG_MS) {
        st.faulted   = true;
        st.lastLogMs = now;
        char msg[48];
        snprintf(msg, sizeof(msg), "MAX31865 %s fault: 0x%02X", name, fault);
        logError(msg);
    }
}

void updateSensors() {
    serviceRtd(rtdBT, rtdBTfault, btTemp, "BT");
#if RTD_ET_ENABLED
    serviceRtd(rtdET, rtdETfault, etTemp, "ET");
#endif

    // --- MAX31855: inlet thermocouple (holds last good value on fault) ---
    double tc = tcIN.readCelsius();
    if (!isnan(tc)) {
        inTemp = (float)tc;
        tcINfault.badCount = 0;
        tcINfault.faulted  = false;
    } else {
        if (tcINfault.badCount < 255) tcINfault.badCount++;
        if (tcINfault.badCount >= SENSOR_FAULT_DEBOUNCE) {
            uint32_t now = millis();
            if (!tcINfault.faulted || (now - tcINfault.lastLogMs) >= SENSOR_REFAULT_LOG_MS) {
                tcINfault.faulted   = true;
                tcINfault.lastLogMs = now;
                char msg[48];
                snprintf(msg, sizeof(msg), "MAX31855 IN fault: 0x%02X", tcIN.readError());
                logError(msg);
            }
        }
    }
}

// ===========================================================================
// Setup
// ===========================================================================
void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(250);

    Serial.printf("\nairRoaster firmware v%s\n", FW_VERSION);

    // Display init
    display.begin(0x3C, true);
    display.clearDisplay();
    display.display();
    display.setTextSize(1);
    display.setTextColor(1, 0);
    display.setRotation(1);

    // DimmerLink init (before WiFi to avoid missing the ready window at power-on)
    initDL(HEAT_ADDR);
    initDL(FAN_ADDR);

    // Sensor init
    initSensors();

    // WiFi
    Serial.print("Connecting to WiFi");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());

    // WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);

    flagDisplayUpdate = true;
}

// ===========================================================================
// WebSocket event handler
// ===========================================================================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED:
            Serial.printf("[WS] Client %u connected\n", num);
            // Send current status to the newly connected client
            broadcastStatus();
            break;

        case WStype_DISCONNECTED:
            Serial.printf("[WS] Client %u disconnected\n", num);
            break;

        case WStype_TEXT: {
            String cmd = String((char*)payload, length);
            cmd.trim();
            Serial.printf("[WS] Command from client %u: %s\n", num, cmd.c_str());
            if (!handleArtisanRequest(num, cmd)) {
                // JSON command envelope: {"command":"OT1","value":60.0,...}
                if (cmd.startsWith("{")) {
                    String kw = jsonString(cmd, "command");
                    float val = jsonFloat(cmd, "value");
                    String plain = kw;
                    if (!isnan(val)) {
                        plain += " ";
                        plain += String((int)roundf(val));
                    }
                    processCommand(plain, (int8_t)num);
                } else {
                    processCommand(cmd, (int8_t)num);
                }
            }
            break;
        }

        case WStype_BIN:
            Serial.printf("[WS] Binary frame from client %u (%u bytes)\n", num, (unsigned)length);
            break;

        default:
            Serial.printf("[WS] Unhandled event type %u from client %u\n", (unsigned)type, num);
            break;
    }
}

// ===========================================================================
// Broadcast status JSON to all WebSocket clients
// ===========================================================================
void broadcastStatus() {
    char buf[96];
    snprintf(buf, sizeof(buf),
             "{\"pushMessage\":\"status\",\"data\":{\"heat\":%u,\"heatReq\":%u,\"fan\":%u,\"ilCap\":%u,\"ilSoft\":%s}}",
             heatLevel,
             requestedHeatLevel,
             fanLevel,
             interlockCap(),
             interlockSoft ? "true" : "false");
    webSocket.broadcastTXT(buf);
}

// ===========================================================================
// Artisan WebSocket request handler
// Artisan sends: {"command":"getData","id":12345,"machine":0}
// We respond:    {"id":12345,"data":{"BT":0.0,"ET":0.0,"IN":0.0}}
// ===========================================================================
bool handleArtisanRequest(uint8_t clientNum, const String& msg) {
    if (msg.indexOf("getData") < 0) return false;

    // Extract numeric id value
    int idPos = msg.indexOf("\"id\"");
    if (idPos < 0) return false;
    int colon = msg.indexOf(':', idPos);
    if (colon < 0) return false;
    int start = colon + 1;
    while (start < (int)msg.length() && msg[start] == ' ') start++;
    int end = start;
    while (end < (int)msg.length() && isDigit(msg[end])) end++;
    if (end == start) return false;

    char idStr[12];
    msg.substring(start, end).toCharArray(idStr, sizeof(idStr));

    char buf[96];
    snprintf(buf, sizeof(buf),
             "{\"id\":%s,\"data\":{\"BT\":%.1f,\"ET\":%.1f,\"IN\":%.1f}}",
             idStr, btTemp, etTemp, inTemp);
    webSocket.sendTXT(clientNum, buf);
    return true;
}

// ===========================================================================
// Display update
// ===========================================================================
void displayUpdate() {
    display.setCursor(COL1A, ROW1 * ROWHEIGHT);
    display.printf("airRoaster v%s", FW_VERSION);

    display.setCursor(COL1A, ROW2 * ROWHEIGHT);
    display.printf("Heat: %u", heatLevel);
    display.setCursor(COL2A, ROW2 * ROWHEIGHT);
    display.printf("Req: %u", requestedHeatLevel);

    display.setCursor(COL1A, ROW3 * ROWHEIGHT);
    display.printf("Fan: %u", fanLevel);

    display.setCursor(COL1A, ROW4 * ROWHEIGHT);
    {
        uint8_t cap = interlockCap();
        if (cap == 0) {
            display.printf("IL:%s BLOCKED", interlockSoft ? "S" : "H");
        } else if (cap < 100) {
            display.printf("IL:S cap=%u%%", cap);
        } else {
            display.printf("IL:%s ok", interlockSoft ? "S" : "H");
        }
    }

    display.setCursor(COL1A, ROW6 * ROWHEIGHT);
    if (WiFi.status() == WL_CONNECTED) {
        display.printf("%s", WiFi.localIP().toString().c_str());
    } else {
        display.printf("No WiFi");
    }

    display.setCursor(COL1A, ROW8 * ROWHEIGHT);
    display.printf("B:%.1f E:%.1f I:%.1f", btTemp, etTemp, inTemp);

    display.display();
}

// ===========================================================================
// DimmerLink helpers
// ===========================================================================
void initDL(uint8_t addr) {
    for (int attempt = 0; attempt < 3; attempt++) {
        if (isReady(addr)) {
            Serial.print(addr);
            Serial.println(" DimmerLink ready!");
            Serial.print("Mains frequency: ");
            Serial.print(getFrequency(addr));
            Serial.println(" Hz");
            Serial.print("setting curve...");
            Serial.println(setCurve(addr, 1));
            checkDLError(addr);
            return;
        }
        delay(DL_INIT_RETRY_DELAY_MS);
    }
    char msg[40];
    snprintf(msg, sizeof(msg), "DimmerLink 0x%02X not ready after 3 tries", addr);
    logError(msg);
}

bool isReady(uint8_t addr) {
    Wire.beginTransmission(addr);
    Wire.write(REG_STATUS);
    Wire.endTransmission(false);

    Wire.requestFrom(addr, 1);
    if (Wire.available()) {
        return (Wire.read() & 0x01) != 0;
    }
    return false;
}

bool setLevel(uint8_t addr, uint8_t level) {
    if (level > 100) return false;

    Wire.beginTransmission(addr);
    Wire.write(REG_LEVEL);
    Wire.write(level);
    return Wire.endTransmission() == 0;
}

int getLevel(uint8_t addr) {
    Wire.beginTransmission(addr);
    Wire.write(REG_LEVEL);
    Wire.endTransmission(false);

    Wire.requestFrom(addr, 1);
    if (Wire.available()) {
        return Wire.read();
    }
    return -1;
}

bool setCurve(uint8_t addr, uint8_t curve) {
    if (curve > 2) return false;

    Wire.beginTransmission(addr);
    Wire.write(REG_CURVE);
    Wire.write(curve);
    return Wire.endTransmission() == 0;
}

int getCurve(uint8_t addr) {
    Wire.beginTransmission(addr);
    Wire.write(REG_CURVE);
    Wire.endTransmission(false);

    Wire.requestFrom(addr, 1);
    if (Wire.available()) {
        return Wire.read();
    }
    return -1;
}

int getFrequency(uint8_t addr) {
    Wire.beginTransmission(addr);
    Wire.write(REG_FREQ);
    Wire.endTransmission(false);

    Wire.requestFrom(addr, 1);
    if (Wire.available()) {
        return Wire.read();
    }
    return -1;
}

uint8_t getError(uint8_t addr) {
    Wire.beginTransmission(addr);
    Wire.write(REG_ERROR);
    Wire.endTransmission(false);

    Wire.requestFrom(addr, 1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0xFF;
}

const char* dlErrorName(uint8_t code) {
    switch (code) {
        case 0x00: return nullptr;          // OK — no error
        case 0xF9: return "ERR_SYNTAX";
        case 0xFC: return "ERR_NOT_READY";
        case 0xFD: return "ERR_INDEX";
        case 0xFE: return "ERR_PARAM";
        default:   return "ERR_UNKNOWN";
    }
}

void checkDLError(uint8_t addr) {
    uint8_t code = getError(addr);
    const char* name = dlErrorName(code);
    if (name != nullptr) {
        char msg[48];
        snprintf(msg, sizeof(msg), "DimmerLink 0x%02X %s (0x%02X)", addr, name, code);
        logError(msg);
    }
}

// ===========================================================================
// Interlock
// ===========================================================================
// Returns the heat cap imposed by the interlock at the current fan level.
uint8_t interlockCap() {
    if (fanLevel < IL_FAN_MIN) return 0;
    if (!interlockSoft || fanLevel >= IL_FAN_FULL) return 100;
    // Linear ramp: IL_HEAT_AT_MIN% at IL_FAN_MIN, 100% at IL_FAN_FULL
    return (uint8_t)(IL_HEAT_AT_MIN +
        (uint32_t)(100 - IL_HEAT_AT_MIN) * (fanLevel - IL_FAN_MIN) / (IL_FAN_FULL - IL_FAN_MIN));
}

void applyInterlock() {
    uint8_t cap = interlockCap();
    uint8_t effective = (requestedHeatLevel < cap) ? requestedHeatLevel : cap;
    if (effective != heatLevel) {
        heatLevel = effective;
        setLevel(HEAT_ADDR, heatLevel);
        checkDLError(HEAT_ADDR);
        flagDisplayUpdate = true;
    }
}

// ===========================================================================
// Command parser
// ===========================================================================

// Split cmd on comma, space, semicolon, or equals; return token count (max 5)
int tokenize(const String &cmd, String tokens[], int maxTokens) {
    int count = 0;
    int start = 0;
    int len = cmd.length();
    for (int i = 0; i <= len && count < maxTokens; i++) {
        char c = (i < len) ? cmd[i] : '\0';
        if (c == ',' || c == ' ' || c == ';' || c == '=' || c == '\0') {
            if (i > start) {
                tokens[count++] = cmd.substring(start, i);
            }
            start = i + 1;
        }
    }
    return count;
}

void processCommand(String cmd, int8_t clientNum) {
    cmd.trim();
    if (cmd.length() == 0) return;

    String tokens[5];
    int n = tokenize(cmd, tokens, 5);
    if (n == 0) return;

    String kw = tokens[0];
    kw.toUpperCase();

    if (kw == "LOG") {
        if (clientNum >= 0) {
            sendLog((uint8_t)clientNum);
        } else {
            uint8_t start = (errCount < ERR_LOG_SIZE) ? 0 : errHead;
            if (errCount == 0) {
                Serial.println("[LOG] no errors");
            } else {
                for (uint8_t i = 0; i < errCount; i++) {
                    Serial.print("[LOG] ");
                    Serial.println(errLog[(start + i) % ERR_LOG_SIZE]);
                }
            }
        }
        return;

    } else if (kw == "OT1") {
        if (n < 2) return;
        String param = tokens[1];
        param.toUpperCase();
        if (param == "UP") {
            requestedHeatLevel = (requestedHeatLevel + DUTY_STEP > 100) ? 100 : requestedHeatLevel + DUTY_STEP;
        } else if (param == "DOWN") {
            requestedHeatLevel = (requestedHeatLevel >= DUTY_STEP) ? requestedHeatLevel - DUTY_STEP : 0;
        } else {
            int duty = param.toInt();
            if (duty >= 0 && duty <= 100) requestedHeatLevel = (uint8_t)duty;
        }
        flagDisplayUpdate = true;
        applyInterlock();
        broadcastStatus();

    } else if (kw == "OT2") {
        if (n < 2) return;
        String param = tokens[1];
        param.toUpperCase();
        if (param == "UP") {
            fanLevel = (fanLevel + DUTY_STEP > 100) ? 100 : fanLevel + DUTY_STEP;
        } else if (param == "DOWN") {
            fanLevel = (fanLevel >= DUTY_STEP) ? fanLevel - DUTY_STEP : 0;
        } else {
            int duty = param.toInt();
            if (duty >= 0 && duty <= 100) fanLevel = (uint8_t)duty;
        }
        setLevel(FAN_ADDR, fanLevel);
        checkDLError(FAN_ADDR);
        flagDisplayUpdate = true;
        applyInterlock();
        broadcastStatus();

    } else if (kw == "IL") {
        interlockSoft = !interlockSoft;
        flagDisplayUpdate = true;
        applyInterlock();
        broadcastStatus();
    }
}

// ===========================================================================
// Loop
// ===========================================================================
void loop() {
    webSocket.loop();

    // Serial commands
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            processCommand(inputBuffer, -1);
            inputBuffer = "";
        } else if (c != '\r') {
            if (inputBuffer.length() < 32) inputBuffer += c;
        }
    }

    // Sensor update (every SENSOR_INTERVAL_MS)
    static uint32_t lastSensorPoll = 0;
    uint32_t now = millis();
    if (now - lastSensorPoll >= SENSOR_INTERVAL_MS) {
        lastSensorPoll = now;
        updateSensors();
    }

    // Periodic DimmerLink error poll (every 5 s)
    static uint32_t lastErrorPoll = 0;
    if (now - lastErrorPoll >= 5000) {
        lastErrorPoll = now;
        checkDLError(HEAT_ADDR);
        checkDLError(FAN_ADDR);
    }

    // Recheck interlock every loop in case fan level changed outside OT2
    applyInterlock();

    // Display
    if (flagDisplayUpdate) {
        display.clearDisplay();
        displayUpdate();
        flagDisplayUpdate = false;
    }
}

// ===========================================================================
// Version history
// ---------------------------------------------------------------------------
// v0.2.1  2026-06-28  RTD read robustness: periodic VBIAS/auto-convert re-assert
//                     (guards against a silently-stalled converter), and a median
//                     filter over recent samples (rejects single-sample spikes).
//                     Also: BT mapped to the connected board (CS_RTD_BT=10); the
//                     empty board is ET (CS=9), gated off via RTD_ET_ENABLED.
// v0.2.0  2026-06-28  Sensor integration: 2x MAX31865 PT1000 (BT/ET) + MAX31855
//                     K-type (inlet), 3-channel Artisan output (BT/ET/IN),
//                     temps on OLED. Robust RTD reads — same-cycle glitch
//                     rejection, fault debounce, hold-last-good, rate-limited
//                     fault logging. ET RTD behind RTD_ET_ENABLED gate (no probe
//                     yet). BT/ET CS pins swapped to match wiring.
// v0.1.0  (baseline)  Dual DimmerLink heat/fan control, WebSocket + Artisan
//                     protocol, plain-text/serial commands, OLED status, fan
//                     interlock (hard/soft), RAM error log.
// ===========================================================================
