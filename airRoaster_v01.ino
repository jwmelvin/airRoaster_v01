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
#define FW_VERSION  "0.4.0"

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
// Inlet-temperature control (closed loop) — see the control section below and
// work.md for the design. Cooperative for now: controlStep() runs on a fixed
// cadence from loop(); the "seam" keeps the law liftable into a FreeRTOS task
// later without changing the math.
// ---------------------------------------------------------------------------
#define CONTROL_PERIOD_MS   250     // control cadence (Hz = 1000/this); matches sensor poll
#define INLET_SV_MIN_C      0.0f    // plausibility window for an inlet setpoint
#define INLET_SV_MAX_C      300.0f
#define PID_D_FILTER_TC     1.0f    // derivative low-pass time constant (s); 0 = off
#define PID_KAW             0.1f    // anti-windup back-calculation gain (1/s)

// Open-loop step-test autotune (TUNE command). Holds the current heat to measure
// a baseline, applies a heat step, records the inlet response, fits a first-order-
// plus-dead-time (FOPDT) model, and suggests PI gains (SIMC/lambda). Run it with
// fan in the normal range and the roaster roughly steady first.
#define TUNE_STEP_DELTA     15      // default heat step (% points) if none given
#define TUNE_STEP_DELTA_MIN 5
#define TUNE_STEP_DELTA_MAX 40
#define TUNE_BASELINE_MS    8000    // hold current heat this long to measure T0
#define TUNE_MAX_MS         180000  // hard cap on the step-observation phase
#define TUNE_MIN_TEST_MS    10000   // suppress settle detection before this
#define TUNE_SAMPLE_MS      500     // response sampling interval
#define TUNE_MAX_SAMPLES    360     // 180 s at TUNE_SAMPLE_MS
#define TUNE_SETTLE_SECS    15      // response flat this long => settled
#define TUNE_SETTLE_BAND_C  1.0f    // "flat" = max-min within this band (°C)
#define TUNE_MIN_RISE_C     3.0f    // require at least this rise to identify (°C)
#define TUNE_TEMP_ABORT_C   280.0f  // abort the test above this inlet temp (°C)

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
// Inlet control state
// ---------------------------------------------------------------------------
// Operating mode: MANUAL = heat is whatever OT1 last set; INLET = heat is
// modulated by the closed loop to hold inletSV. Marked volatile because a
// future FreeRTOS control task would read/write these across cores.
enum ctrlMode_t { MODE_MANUAL, MODE_INLET, MODE_TUNE };
volatile ctrlMode_t ctrlMode = MODE_MANUAL;
volatile float       inletSV  = 0.0f;   // inlet setpoint (°C), valid in MODE_INLET

// Autotune progress + last suggestion. Declared here (ahead of processCommand,
// which starts/aborts/applies a tune) rather than beside the tune internals.
enum tunePhase_t { TUNE_IDLE, TUNE_BASELINE, TUNE_STEP };
volatile tunePhase_t tunePhase   = TUNE_IDLE;
bool                 tuneHaveSug = false;   // a suggested-gain set is available
float                tuneSugKp   = 0.0f;    // SIMC "tight" suggestion from last tune
float                tuneSugKi   = 0.0f;

// PID gains — PLACEHOLDERS. The plant has NOT been characterized yet; these are
// deliberately gentle and are not expected to control well. Tune via the step
// test / TUNE routine (work.md phase 3) or live with the PID command before
// relying on closed-loop control. Starts PI-only (Kd = 0) — the inlet TC is
// noisy near the dimmers, so derivative is added only after characterization.
float pidKp = 1.5f;
float pidKi = 0.05f;   // 1/s
float pidKd = 0.0f;

// Cooperative-cadence jitter watch (worst control-step late-fire, µs). This is
// the evidence that decides whether a dedicated core is ever warranted — read
// and zeroed via the STAT command. Declared here (ahead of processCommand,
// which reports it) rather than beside the other PID working state below.
volatile uint32_t ctrlMaxJitterUs = 0;

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
float feedforward(float sv, uint8_t fan);
void  engageInlet(float sv);
void  controlStep(uint32_t dtMs);
void  startTune(float deltaPct);
void  abortTune(const char *why);
void  tuneStep(uint32_t dtMs);

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
    char buf[160];
    snprintf(buf, sizeof(buf),
             "{\"pushMessage\":\"status\",\"data\":{\"heat\":%u,\"heatReq\":%u,\"fan\":%u,\"ilCap\":%u,\"ilSoft\":%s,\"mode\":\"%s\",\"inSV\":%.1f}}",
             heatLevel,
             requestedHeatLevel,
             fanLevel,
             interlockCap(),
             interlockSoft ? "true" : "false",
             ctrlMode == MODE_INLET ? "inlet" : (ctrlMode == MODE_TUNE ? "tune" : "manual"),
             inletSV);
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

    display.setCursor(COL1A, ROW5 * ROWHEIGHT);
    if (ctrlMode == MODE_INLET) {
        display.printf("Inlet SV:%.0f", inletSV);
    } else if (ctrlMode == MODE_TUNE) {
        display.printf("Tuning...");
    } else {
        display.printf("Inlet: off");
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
        if (tunePhase != TUNE_IDLE) abortTune("manual override");
        ctrlMode = MODE_MANUAL;   // manual heat command is an instant override
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

    } else if (kw == "INLET") {
        // INLET <degC> — set inlet setpoint and engage closed-loop control.
        // INLET OFF      — disengage; heat holds at its current level.
        if (n < 2) return;
        if (tunePhase != TUNE_IDLE) abortTune("inlet override");
        String param = tokens[1];
        String upper = param;
        upper.toUpperCase();
        if (upper == "OFF") {
            ctrlMode = MODE_MANUAL;   // leave requestedHeatLevel where it is
        } else {
            engageInlet(param.toFloat());
        }
        flagDisplayUpdate = true;
        broadcastStatus();

    } else if (kw == "PID") {
        // PID            — report current gains/setpoint/mode.
        // PID <kp ki kd> — set gains live (for manual tuning).
        if (n >= 4) {
            pidKp = tokens[1].toFloat();
            pidKi = tokens[2].toFloat();
            pidKd = tokens[3].toFloat();
        }
        char b[112];
        snprintf(b, sizeof(b),
            "{\"pushMessage\":\"pid\",\"data\":{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"sv\":%.1f,\"mode\":\"%s\"}}",
            pidKp, pidKi, pidKd, inletSV,
            ctrlMode == MODE_INLET ? "inlet" : (ctrlMode == MODE_TUNE ? "tune" : "manual"));
        if (clientNum >= 0) webSocket.sendTXT((uint8_t)clientNum, b);
        Serial.println(b);

    } else if (kw == "STAT") {
        // Report the control-cadence jitter watch (worst late-fire, µs) and zero
        // it. Evidence on whether the cooperative loop ever needs a real core.
        char b[96];
        snprintf(b, sizeof(b),
            "{\"pushMessage\":\"stat\",\"data\":{\"ctrlMaxJitterUs\":%lu}}",
            (unsigned long)ctrlMaxJitterUs);
        if (clientNum >= 0) webSocket.sendTXT((uint8_t)clientNum, b);
        Serial.println(b);
        ctrlMaxJitterUs = 0;

    } else if (kw == "TUNE") {
        // TUNE              — start a step test (default step size).
        // TUNE <pct>        — start with a specific heat step (% points).
        // TUNE ABORT|OFF    — cancel a running test.
        // TUNE APPLY        — apply the last "tight" suggestion as PID gains.
        if (n >= 2) {
            String p = tokens[1];
            String up = p;
            up.toUpperCase();
            if (up == "OFF" || up == "ABORT") {
                if (tunePhase != TUNE_IDLE) abortTune("operator abort");
            } else if (up == "APPLY") {
                if (tuneHaveSug) {
                    pidKp = tuneSugKp;
                    pidKi = tuneSugKi;
                    pidKd = 0.0f;
                    char b[112];
                    snprintf(b, sizeof(b),
                        "{\"pushMessage\":\"tune\",\"data\":{\"applied\":true,\"kp\":%.3f,\"ki\":%.4f}}",
                        pidKp, pidKi);
                    webSocket.broadcastTXT(b);
                    Serial.println(b);
                }
            } else {
                if (tunePhase == TUNE_IDLE) startTune(p.toFloat());
            }
        } else if (tunePhase == TUNE_IDLE) {
            startTune((float)TUNE_STEP_DELTA);
        }
    }
}

// ===========================================================================
// Inlet temperature control (closed loop)
// ===========================================================================
// The entire control law lives in controlStep(), operating only on shared
// globals (inTemp, fanLevel, requestedHeatLevel, the PID state). That is the
// "seam": cooperative now (called on a fixed cadence from loop()), but liftable
// into a pinned FreeRTOS control task later with no change to the math — see
// work.md. In MODE_MANUAL it does nothing; OT1 drives heat directly.

// PID working state (file-local).
static float    pidITerm  = 0.0f;   // integral accumulator, in heat-% units
static float    pidPvPrev = 0.0f;   // previous process value (inTemp)
static float    pidDFilt  = 0.0f;   // filtered derivative term
static bool     ctrlPrimed = false; // false until the first step seeds pvPrev
// (ctrlMaxJitterUs is declared up in the state section, ahead of processCommand.)

// Static feedforward heat estimate. Phase-4 hook (work.md): returns 0 until the
// power map f(setpoint, fan) is calibrated. Wired into the loop now so the
// engage-seed and anti-windup math already account for it.
float feedforward(float sv, uint8_t fan) {
    (void)sv; (void)fan;
    return 0.0f;
}

// Engage closed-loop control with a bumpless transfer: seed the integrator so
// the first output equals the current heat level (u = Kp*e + iTerm + FF must
// equal heatLevel  =>  iTerm = heatLevel - Kp*e - FF). No step on engage.
void engageInlet(float sv) {
    if (sv < INLET_SV_MIN_C) sv = INLET_SV_MIN_C;
    if (sv > INLET_SV_MAX_C) sv = INLET_SV_MAX_C;

    // Already closed-loop: just retarget and keep the integrator. The setpoint
    // changes gradually during a roast (Artisan drives INLET repeatedly), so
    // re-seeding here would reset integral action on every update.
    if (ctrlMode == MODE_INLET) { inletSV = sv; return; }

    // Manual -> inlet: bumpless transfer. Seed the integrator so the first
    // output equals the current heat level (u = Kp*e + iTerm + FF == heatLevel).
    inletSV = sv;
    float e = sv - inTemp;
    pidITerm  = (float)heatLevel - pidKp * e - feedforward(sv, fanLevel);
    pidPvPrev = inTemp;
    pidDFilt  = 0.0f;
    ctrlPrimed = true;
    ctrlMode  = MODE_INLET;
}

// One control iteration. dtMs is the measured interval since the last call, so
// timing jitter doesn't bias the integral/derivative.
void controlStep(uint32_t dtMs) {
    if (ctrlMode == MODE_TUNE) { tuneStep(dtMs); return; }
    if (ctrlMode != MODE_INLET) { ctrlPrimed = false; return; }

    float dt = dtMs * 0.001f;
    if (dt <= 0.0f) return;
    if (dt > 1.0f) dt = 1.0f;   // clamp a pathological gap (e.g. post-stall)

    float pv = inTemp;
    if (!ctrlPrimed) { pidPvPrev = pv; ctrlPrimed = true; }  // belt-and-suspenders

    float e  = inletSV - pv;
    float P  = pidKp * e;
    float ff = feedforward(inletSV, fanLevel);

    // Derivative on measurement (no setpoint-change kick), low-pass filtered.
    float dRaw = -pidKd * (pv - pidPvPrev) / dt;
    if (PID_D_FILTER_TC > 0.0f) {
        float a = dt / (PID_D_FILTER_TC + dt);
        pidDFilt += a * (dRaw - pidDFilt);
    } else {
        pidDFilt = dRaw;
    }
    float D = pidDFilt;

    pidITerm += pidKi * e * dt;            // provisional integration
    float u = P + pidITerm + D + ff;

    // What the hardware will actually apply: clamp to 0..100 and the interlock.
    float uClamped = u < 0.0f ? 0.0f : (u > 100.0f ? 100.0f : u);
    float cap      = (float)interlockCap();
    float applied  = uClamped < cap ? uClamped : cap;

    // Back-calculation anti-windup against the *applied* value, so saturation —
    // including the fan interlock capping heat — doesn't wind the integrator up.
    pidITerm += PID_KAW * (applied - u) * dt;
    if (pidITerm < 0.0f)   pidITerm = 0.0f;
    if (pidITerm > 100.0f) pidITerm = 100.0f;

    pidPvPrev = pv;

    requestedHeatLevel = (uint8_t)lroundf(uClamped);
    applyInterlock();   // writes min(requestedHeatLevel, cap) to the heat dimmer
}

// ===========================================================================
// Open-loop step-test autotune (TUNE)
// ===========================================================================
// State machine, stepped from controlStep() at the control cadence while in
// MODE_TUNE: hold baseline -> step heat -> sample the inlet response -> fit an
// FOPDT model (two-point 28.3%/63.2% method) -> suggest PI gains (SIMC). The
// step goes through applyInterlock(), so inadequate airflow aborts the test.

static uint8_t  tuneU0        = 0;     // baseline applied heat (%)
static uint8_t  tuneCmdDelta  = 0;     // commanded step size (% points)
static uint8_t  tuneStepHeat  = 0;     // commanded heat during the step
static float    tuneStepDelta = 0.0f;  // actual applied heat delta (post-interlock)
static float    tuneT0        = 0.0f;  // measured baseline temp (°C)
static uint32_t tunePhaseMs   = 0;     // elapsed time in the current phase
static uint32_t tuneSampleAcc = 0;     // ms since the last response sample
static float    tuneBaseAcc   = 0.0f;  // baseline-temp accumulator
static uint16_t tuneBaseN     = 0;     // baseline samples
static float    tuneBuf[TUNE_MAX_SAMPLES];  // sampled response (buf[0] at step)
static uint16_t tuneCount     = 0;     // samples recorded

// Send a result line to every client and the serial console, then return to
// manual holding the pre-test baseline heat.
static void tuneReportAndRestore(const char *json) {
    webSocket.broadcastTXT(json);
    Serial.println(json);
    requestedHeatLevel = tuneU0;
    ctrlMode  = MODE_MANUAL;
    tunePhase = TUNE_IDLE;
    applyInterlock();
    flagDisplayUpdate = true;
    broadcastStatus();
}

void abortTune(const char *why) {
    char buf[128];
    snprintf(buf, sizeof(buf),
        "{\"pushMessage\":\"tune\",\"data\":{\"ok\":false,\"reason\":\"%s\"}}", why);
    tuneReportAndRestore(buf);
}

// First time (s, relative to the step) the response reaches `target`, linearly
// interpolated; -1 if it never does. Assumes a rising response (buf[0] ~ T0).
static float tuneCrossTime(float target) {
    for (uint16_t i = 0; i < tuneCount; i++) {
        if (tuneBuf[i] >= target) {
            if (i == 0) return 0.0f;
            float a = tuneBuf[i - 1], b = tuneBuf[i];
            float frac = (b > a) ? (target - a) / (b - a) : 0.0f;
            return ((float)(i - 1) + frac) * (TUNE_SAMPLE_MS * 0.001f);
        }
    }
    return -1.0f;
}

// SIMC (Skogestad) PI tuning for an FOPDT model. lambda is the closed-loop time
// constant (robustness knob: smaller = brisker). Outputs gains in our units —
// kc [%/°C], ki [%/(°C·s)] = kc / Ti.
static void tuneSimcPI(float Kp, float tau, float theta, float lambda,
                       float &kc, float &ki) {
    float denom = lambda + theta;
    if (denom < 1e-3f) denom = 1e-3f;
    kc = (1.0f / Kp) * (tau / denom);
    float Ti = (tau < 4.0f * denom) ? tau : 4.0f * denom;
    ki = (Ti > 1e-3f) ? kc / Ti : 0.0f;
}

// Fit the recorded response and report the model + suggested gains.
static void tuneFinish() {
    uint16_t navg = tuneCount < 8 ? tuneCount : 8;
    float sum = 0.0f;
    for (uint16_t i = tuneCount - navg; i < tuneCount; i++) sum += tuneBuf[i];
    float Tfinal = navg ? sum / navg : tuneT0;
    float dT = Tfinal - tuneT0;

    char buf[256];
    if (dT < TUNE_MIN_RISE_C || tuneStepDelta < 1.0f) {
        snprintf(buf, sizeof(buf),
            "{\"pushMessage\":\"tune\",\"data\":{\"ok\":false,\"reason\":\"insufficient response\",\"dT\":%.1f,\"step\":%.0f}}",
            dT, tuneStepDelta);
        tuneReportAndRestore(buf);
        return;
    }

    float Kp = dT / tuneStepDelta;                       // process gain (°C/%)
    float t28 = tuneCrossTime(tuneT0 + 0.283f * dT);
    float t63 = tuneCrossTime(tuneT0 + 0.632f * dT);
    if (t28 < 0.0f || t63 <= t28) {
        snprintf(buf, sizeof(buf),
            "{\"pushMessage\":\"tune\",\"data\":{\"ok\":false,\"reason\":\"could not fit FOPDT\",\"dT\":%.1f}}",
            dT);
        tuneReportAndRestore(buf);
        return;
    }
    float tau   = 1.5f * (t63 - t28);
    float theta = t63 - tau;
    if (theta < 0.0f) theta = 0.0f;

    // Two robustness levels: "tight" (brisk, per the operator's tracking-first
    // preference) and "cons" (conservative). Bound lambda so a tiny dead time
    // doesn't blow the gains up.
    float lamT = (theta > 0.1f * tau) ? theta : 0.1f * tau;
    float lamC = (3.0f * theta > 0.5f * tau) ? 3.0f * theta : 0.5f * tau;
    float kpT, kiT, kpC, kiC;
    tuneSimcPI(Kp, tau, theta, lamT, kpT, kiT);
    tuneSimcPI(Kp, tau, theta, lamC, kpC, kiC);

    tuneSugKp = kpT;
    tuneSugKi = kiT;
    tuneHaveSug = true;

    snprintf(buf, sizeof(buf),
        "{\"pushMessage\":\"tune\",\"data\":{\"ok\":true,\"fan\":%u,\"step\":%.0f,\"dT\":%.1f,"
        "\"Kp\":%.3f,\"tau\":%.1f,\"theta\":%.1f,"
        "\"tight\":{\"kp\":%.3f,\"ki\":%.4f},\"cons\":{\"kp\":%.3f,\"ki\":%.4f}}}",
        fanLevel, tuneStepDelta, dT, Kp, tau, theta, kpT, kiT, kpC, kiC);
    tuneReportAndRestore(buf);
}

// Begin a step test from the current operating point.
void startTune(float deltaPct) {
    if (deltaPct < TUNE_STEP_DELTA_MIN) deltaPct = TUNE_STEP_DELTA_MIN;
    if (deltaPct > TUNE_STEP_DELTA_MAX) deltaPct = TUNE_STEP_DELTA_MAX;
    tuneU0       = heatLevel;            // current applied heat is the baseline
    tuneCmdDelta = (uint8_t)deltaPct;
    tuneBaseAcc  = 0.0f;
    tuneBaseN    = 0;
    tunePhaseMs  = 0;
    tuneSampleAcc = 0;
    tuneCount    = 0;
    tunePhase    = TUNE_BASELINE;
    ctrlMode     = MODE_TUNE;
    flagDisplayUpdate = true;
    broadcastStatus();
}

void tuneStep(uint32_t dtMs) {
    const uint16_t settleSamples = (TUNE_SETTLE_SECS * 1000) / TUNE_SAMPLE_MS;

    if (inTemp > TUNE_TEMP_ABORT_C) { abortTune("over-temp"); return; }

    switch (tunePhase) {

    case TUNE_BASELINE:
        requestedHeatLevel = tuneU0;
        applyInterlock();
        tuneBaseAcc += inTemp;
        tuneBaseN++;
        tunePhaseMs += dtMs;
        if (tunePhaseMs >= TUNE_BASELINE_MS) {
            tuneT0 = tuneBaseN ? tuneBaseAcc / tuneBaseN : inTemp;
            uint16_t cmd = (uint16_t)tuneU0 + tuneCmdDelta;
            if (cmd > 100) cmd = 100;
            tuneStepHeat = (uint8_t)cmd;
            requestedHeatLevel = tuneStepHeat;
            applyInterlock();
            tuneStepDelta = (float)heatLevel - (float)tuneU0;   // actual, post-interlock
            if (tuneStepDelta < 1.0f) { abortTune("interlock capped step (raise fan)"); return; }
            tuneBuf[0] = inTemp;       // sample at t = 0 (≈ T0)
            tuneCount  = 1;
            tuneSampleAcc = 0;
            tunePhaseMs = 0;
            tunePhase = TUNE_STEP;
        }
        break;

    case TUNE_STEP:
        requestedHeatLevel = tuneStepHeat;
        applyInterlock();
        tunePhaseMs   += dtMs;
        tuneSampleAcc += dtMs;

        if (tuneSampleAcc >= TUNE_SAMPLE_MS) {
            tuneSampleAcc -= TUNE_SAMPLE_MS;
            if (tuneCount < TUNE_MAX_SAMPLES) tuneBuf[tuneCount++] = inTemp;

            // Settled? Response flat (max-min within band) over the last window.
            if (tuneCount >= settleSamples && tunePhaseMs >= TUNE_MIN_TEST_MS) {
                float mn = 1e9f, mx = -1e9f;
                for (uint16_t i = tuneCount - settleSamples; i < tuneCount; i++) {
                    float v = tuneBuf[i];
                    if (v < mn) mn = v;
                    if (v > mx) mx = v;
                }
                if (mx - mn < TUNE_SETTLE_BAND_C) { tuneFinish(); return; }
            }
        }

        if (tunePhaseMs >= TUNE_MAX_MS || tuneCount >= TUNE_MAX_SAMPLES) {
            tuneFinish();   // timeout — fit whatever we have
            return;
        }
        break;

    default:
        tunePhase = TUNE_IDLE;
        ctrlMode  = MODE_MANUAL;
        break;
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

    // Inlet control step (fixed cadence; the controlStep() seam). Uses the
    // measured interval so cooperative jitter doesn't bias the integral, and
    // records the worst late-fire as evidence on whether a dedicated core is
    // ever needed (read via STAT). controlStep() is a no-op in MODE_MANUAL.
    static uint32_t lastCtrl = 0;
    uint32_t ctrlDt = now - lastCtrl;
    if (ctrlDt >= CONTROL_PERIOD_MS) {
        if (lastCtrl != 0 && ctrlDt > CONTROL_PERIOD_MS) {
            uint32_t jitterUs = (ctrlDt - CONTROL_PERIOD_MS) * 1000UL;
            if (jitterUs > ctrlMaxJitterUs) ctrlMaxJitterUs = jitterUs;
        }
        lastCtrl = now;
        controlStep(ctrlDt);
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
// v0.4.0  2026-06-29  Open-loop step-test autotune (TUNE command). Holds the
//                     current heat to measure a baseline, applies a heat step,
//                     samples the inlet response, fits an FOPDT model (two-point
//                     28.3%/63.2% method) and suggests PI gains via SIMC at two
//                     robustness levels (tight/conservative). New MODE_TUNE; the
//                     step runs through applyInterlock() so inadequate airflow
//                     aborts it, with over-temp and operator-override aborts too.
//                     TUNE [pct] / TUNE ABORT / TUNE APPLY. Results broadcast as
//                     a "tune" push message. No external library — identification
//                     and tuning are inline (see work.md phase 3).
// v0.3.0  2026-06-29  Inlet-temperature closed-loop control (cooperative). New
//                     MODE_INLET: controlStep() modulates heat from measured
//                     inlet temp with a PI(D) loop — derivative-on-measurement,
//                     back-calculation anti-windup against the *applied* (post-
//                     interlock) heat, bumpless engage. Commands: INLET <degC> /
//                     INLET OFF, PID (report/set gains live), STAT (control
//                     jitter watch). OT1 is an instant manual override. Mode +
//                     setpoint added to the status broadcast and OLED row 5.
//                     Gains are untuned placeholders. Feedforward + sTune TUNE
//                     routine are the next phases (see work.md). The control law
//                     is isolated in controlStep() so it can later move to a
//                     pinned FreeRTOS task unchanged.
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
