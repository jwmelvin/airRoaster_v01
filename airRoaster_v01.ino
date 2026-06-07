#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <WiFi.h>
#include <WebSocketsServer.h>

// ---------------------------------------------------------------------------
// WiFi credentials (defined in secrets.h — do not commit that file)
// ---------------------------------------------------------------------------
#include "secrets.h"

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
#define FAN_INTERLOCK_THRESHOLD 30
#define WS_PORT                 81

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
uint8_t requestedHeatLevel = 0;  // commanded heat level
uint8_t heatLevel = 0;           // actual heat level sent to device
uint8_t fanLevel = 0;

String inputBuffer = "";
volatile bool flagDisplayUpdate;

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

    char buf[ERR_MSG_LEN + 12];
    snprintf(buf, sizeof(buf), "{\"error\":\"%s\"}", msg);
    webSocket.broadcastTXT(buf);
}

void sendLog(uint8_t clientNum) {
    uint8_t start = (errCount < ERR_LOG_SIZE) ? 0 : errHead;
    for (uint8_t i = 0; i < errCount; i++) {
        uint8_t idx = (start + i) % ERR_LOG_SIZE;
        char buf[ERR_MSG_LEN + 12];
        snprintf(buf, sizeof(buf), "{\"log\":\"%s\"}", errLog[idx]);
        webSocket.sendTXT(clientNum, buf);
    }
    if (errCount == 0) {
        webSocket.sendTXT(clientNum, "{\"log\":\"no errors\"}");
    }
}

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void processCommand(String cmd, int8_t clientNum = -1);
void applyInterlock();
void broadcastStatus();

// ===========================================================================
// Setup
// ===========================================================================
void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(250);

    // Display init
    display.begin(0x3C, true);
    display.clearDisplay();
    display.display();
    display.setTextSize(1);
    display.setTextColor(1, 0);
    display.setRotation(1);

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
    Serial.print("WebSocket server started on port ");
    Serial.println(WS_PORT);

    // DimmerLink init (after WebSocket so logError can broadcast)
    initDL(HEAT_ADDR);
    initDL(FAN_ADDR);

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
            processCommand(cmd, (int8_t)num);
            break;
        }

        default:
            break;
    }
}

// ===========================================================================
// Broadcast status JSON to all WebSocket clients
// ===========================================================================
void broadcastStatus() {
    char buf[80];
    snprintf(buf, sizeof(buf),
             "{\"heat\":%u,\"heatReq\":%u,\"fan\":%u,\"interlock\":%s}",
             heatLevel,
             requestedHeatLevel,
             fanLevel,
             (fanLevel < FAN_INTERLOCK_THRESHOLD) ? "true" : "false");
    webSocket.broadcastTXT(buf);
}

// ===========================================================================
// Display update
// ===========================================================================
void displayUpdate() {
    display.setCursor(COL1A, ROW2 * ROWHEIGHT);
    display.printf("Heat: %u", heatLevel);
    display.setCursor(COL2A, ROW2 * ROWHEIGHT);
    display.printf("Req: %u", requestedHeatLevel);

    display.setCursor(COL1A, ROW3 * ROWHEIGHT);
    display.printf("Fan: %u", fanLevel);

    display.setCursor(COL1A, ROW4 * ROWHEIGHT);
    if (fanLevel < FAN_INTERLOCK_THRESHOLD) {
        display.printf("INTERLOCK");
    }

    display.setCursor(COL1A, ROW6 * ROWHEIGHT);
    if (WiFi.status() == WL_CONNECTED) {
        display.printf("%s", WiFi.localIP().toString().c_str());
    } else {
        display.printf("No WiFi");
    }

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
        delay(200);
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
void applyInterlock() {
    uint8_t effective = (fanLevel >= FAN_INTERLOCK_THRESHOLD) ? requestedHeatLevel : 0;
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
        if (clientNum >= 0) sendLog((uint8_t)clientNum);
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

    // Periodic DimmerLink error poll (every 5 s)
    static uint32_t lastErrorPoll = 0;
    uint32_t now = millis();
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
