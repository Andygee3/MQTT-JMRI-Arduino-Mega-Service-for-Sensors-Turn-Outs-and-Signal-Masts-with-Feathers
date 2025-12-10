#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Wire.h>

// --- WiFi & MQTT ---
#include <WiFiEspAT.h>      // ESP8266 AT-command WiFi driver (ESP-01S)
#include <PubSubClient.h>

// --- Your project headers ---
#include "Turnout.h"
#include "Sensor.h"
#include "Signal.h"
#include "SignalMast.h"
#include "RFIDOutput.h"

// ================== CONFIG ==================
#define boardNo 0
#define sensorTopic "myTrains/track/sensor/"
#define noPWMBoards 2
#define NumberOfRFIDReaders 0      // set to 1 later when you want RFID active
#define stepSize 2
#define stepTime 3000

#define ESP_AT_BAUD 115200   // 9600 if you changed with AT+UART_DEF

static int  prevWifiStatus = WL_IDLE_STATUS;
static unsigned long lastStatusNote = 0;
static bool mqttBootPublished = false;

// MQTT broker
const char* mqtt_server = "192.168.0.35";
const int   mqtt_port   = 1883;
const char* mqtt_user = "mqttbroker";
const char* mqtt_pass = "mqttbroker";
// Topic to subscribe to
const char* mqtt_topic  = "myTrains/#";
// ===========================================

Adafruit_PWMServoDriver pwm0;
Adafruit_PWMServoDriver pwm1;

#if NumberOfRFIDReaders > 0
RFIDOutput TagReaders[NumberOfRFIDReaders] = {
  RFIDOutput(Serial2, "platform"),
};
#endif

int freeRam() {
  extern int __heap_start,*__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

unsigned long lastReadTime = 0;
const int debounceDelay = 50;

const int maxTurnout = 16;
Turnout turnouts[maxTurnout];

const int maxSensor = 32;
Sensor sensors[maxSensor];

const int maxSignal = 16;
Signal signals[maxSignal];

const int maxSignalMast = 16;
SignalMast signalMasts[maxSignalMast];

// ---- Signal Mast flashing state ----
bool mastFlashEnabled[maxSignalMast];
bool mastFlashCa[maxSignalMast];     // flash caution LED?
bool mastFlashCp[maxSignalMast];     // flash prelim-caution LED?
bool mastFlashOn[maxSignalMast];     // current ON/OFF phase
unsigned long mastLastFlashToggle[maxSignalMast];

// Per-mast off + per-LED lit brightness (PCA9685 "off" and "on" values)
uint16_t mastOffVal[maxSignalMast];
uint16_t mastPrLitVal[maxSignalMast];
uint16_t mastCaLitVal[maxSignalMast];
uint16_t mastCpLitVal[maxSignalMast];
uint16_t mastDaLitVal[maxSignalMast];

// (optional: per-feather brightness if you fancy later)
uint16_t mastAx1LitVal[maxSignalMast];
uint16_t mastAx2LitVal[maxSignalMast];
uint16_t mastAx3LitVal[maxSignalMast];
uint16_t mastAx4LitVal[maxSignalMast];
uint16_t mastAx5LitVal[maxSignalMast];
uint16_t mastAx6LitVal[maxSignalMast];

const unsigned long mastFlashInterval = 417;  // ms between toggles


// ---- WiFi / MQTT clients ----
WiFiClient wifiClient;
PubSubClient client(wifiClient);

// Variables to hold the split message parts
String part1, part2, part3;

// Forward declarations
void setupLayout(String message);
void setTurnOut(String turnout, String message);
void changeSignal(String signal, String message);
void checkSensors();
void moveTurnOut(int turnoutBoard, int turnoutPin, int newPWMVal);
void reconnectMQTT();
void PublishToMQTT(const String& topic, const String& message, bool retain);
void changeSignalMast(String mast, String message);
SignalMast* findSignalMastByName(String searchName);

int  getSignalMastIndexByName(const String& name);
void updateSignalMastFlashing();

String getMastIdFromTopic(const String& topic);
void handleJMRISignalMastMessage(const String& topic, const String& msg);
void handleJMRIJunctionFeatherMessage(const String& topic, const String& msg);


// ================== WIFI ATTACH & JOIN ==================
static bool espUartReady = false;

void ensureWiFiAttachedAndJoined() {
  if (!espUartReady) {
    Serial.println(F("[WiFi] Attaching ESP-01S on Serial3 @115200"));
    Serial3.begin(ESP_AT_BAUD);
    WiFi.init(Serial3);
    delay(300);

    Serial.print(F("[WiFi] Firmware: "));
    Serial.println(WiFi.firmwareVersion());


    uint8_t mac[6];
    WiFi.macAddress(mac); // WiFiEspAT returns MAC LSB->MSB
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
    Serial.print(F("[WiFi] STA MAC: "));
    Serial.println(macStr);

    espUartReady = true;
    delay(1000); // small settle
  }

  // Let the module auto-join using its saved credentials (CWAUTOCONN=1, CWJAP_DEF=...)
  unsigned long start = millis();
  Serial.print(F("[WiFi] Waiting for connection (ESP auto-join)..."));
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000UL) {
    Serial.print('.');
    delay(300);
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("[WiFi] Connected. IP: "));
    Serial.println(WiFi.localIP());
    Serial.print(F("[WiFi] RSSI: "));
    Serial.println(WiFi.RSSI());
  } else {
    Serial.println(F("[WiFi] Not connected yet (timeout). ESP may still join shortly."));
  }
}

// ================== MQTT CALLBACK ==================
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  String subTopic = topic;
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];

  Serial.println("Received MQTT message: " + message + " on topic: " + subTopic);

  // Prioritise setup messages
  if (subTopic.indexOf("/setup") != -1) {
    Serial.println(F("[callback] /setup detected, calling setupLayout()"));
    setupLayout(message);
    Serial.println(F("[callback] setupLayout() finished"));
    return;
  }
  
if (subTopic.indexOf("/signalmast") != -1) {
  // Use "|F" on the topic to detect junction feathers
  if (subTopic.endsWith("|F")) {
    handleJMRIJunctionFeatherMessage(subTopic, message);
  } else {
    // Everything else under /signalmast is a main mast aspect
    handleJMRISignalMastMessage(subTopic, message);
  }
  return;
}




  // Process other messages
  String delimiter = "/";
  int startIndex = 0;
  int splitIndex = subTopic.indexOf(delimiter);
  String parts[4];
  int i = 0;

  while (splitIndex != -1 && i < 4) {
    parts[i] = subTopic.substring(startIndex, splitIndex);
    startIndex = splitIndex + delimiter.length();
    splitIndex = subTopic.indexOf(delimiter, startIndex);
    i++;
  }
  if (i < 4 && startIndex < subTopic.length()) {
    parts[i] = subTopic.substring(startIndex);
  }

  // Expect topics like: myTrains / track / turnout / A12:T1
  if (i >= 3 && parts[2] == "turnout" && parts[3].length() > 0) {
    String id = parts[3];

    Serial.print(F("[callback] Control for id="));
    Serial.print(id);
    Serial.print(F(" message="));
    Serial.println(message);

    // First try turnout
    Turnout* t = findTurnoutByName(id);
    if (t != nullptr) {
      Serial.println(F("Turnout Command Received"));
      setTurnOut(id, message);
      return;
    }

    // Then try signal
    Signal* s = findSignalByName(id);
    if (s != nullptr) {
      Serial.println(F("Signal Command Received"));
      changeSignal(id, message);
      return;
    }
    
    SignalMast* m = findSignalMastByName(id);
    if (m != nullptr) {
      Serial.println(F("Signal Mast Command Received"));
      changeSignalMast(id, message);
      return;
    }

    
    // Nothing matched
    Serial.print(F("[callback] Unknown turnout/signal/signalMast id: "));
    Serial.println(id);
  }
}

// ================== FIND HELPERS ==================
Turnout* findTurnoutByName(String searchName) {
  for (int i = 0; i < maxTurnout; i++) {
    if (turnouts[i].getName() == searchName) return &turnouts[i];
  }
  return nullptr;
}

Signal* findSignalByName(String searchName) {
  for (int i = 0; i < maxSignal; i++) {
    if (signals[i].getName() == searchName) return &signals[i];
  }
  return nullptr;
}

SignalMast* findSignalMastByName(String searchName) {
  for (int i = 0; i < maxSignalMast; i++) {
    if (signalMasts[i].getName() == searchName) {
      return &signalMasts[i];
    }
  }
  return nullptr;
}

int getSignalMastIndexByName(const String& name) {
  for (int i = 0; i < maxSignalMast; i++) {
    if (signalMasts[i].getName() == name) {
      return i;
    }
  }
  return -1;
}

String getMastIdFromTopic(const String& topic) {
  int lastSlash = topic.lastIndexOf('/');
  if (lastSlash < 0 || lastSlash >= topic.length() - 1) return "";

  // Tail will be something like: "{0}B9|S1" or "{0}B9|S1|F"
  String tail = topic.substring(lastSlash + 1);

  // Strip "{0}" or similar prefix if present
  if (tail.startsWith("{")) {
    int close = tail.indexOf('}');
    if (close > 0 && close < tail.length() - 1) {
      tail = tail.substring(close + 1);   // "B9|S1" or "B9|S1|F"
    }
  }

  // If we have a trailing "|F" (feather), drop it so feathers & mast share ID
  int lastPipe = tail.lastIndexOf('|');
  if (lastPipe > 0) {
    String suffix = tail.substring(lastPipe + 1);
    suffix.toUpperCase();
    if (suffix == "F") {
      tail = tail.substring(0, lastPipe); // "B9|S1"
    }
  }

  return tail;  // e.g. "B9|S1"
}

// ================== SETUP LAYOUT ==================
void setupLayout(String message) {
  Serial.print(F("[setupLayout] raw message: "));
  Serial.println(message);

  StaticJsonDocument<256> doc;   // plenty for your setup JSON

  DeserializationError error = deserializeJson(doc, message);
  if (error) {
    Serial.print(F("[setupLayout] deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  // Dump parsed JSON structure for sanity (one-liner)
  Serial.print(F("[setupLayout] parsed doc: "));
  serializeJson(doc, Serial);
  Serial.println();

  // Access "setup" as an array safely
  JsonArray setupArr = doc["setup"].as<JsonArray>();
  if (setupArr.isNull()) {
    Serial.println(F("[setupLayout] 'setup' is null or not an array"));
    return;
  }
  if (setupArr.size() == 0) {
    Serial.println(F("[setupLayout] 'setup' array is empty"));
    return;
  }

  JsonObject setup0 = setupArr[0];
  if (setup0.isNull()) {
    Serial.println(F("[setupLayout] 'setup[0]' is null"));
    return;
  }

  // ---- robust string → int parsing ----
  const char* bStr = setup0["B"];
  int board = bStr ? atoi(bStr) : -1;

  String type = doc["type"] | "";

  Serial.print(F("[setupLayout] boardStr="));
  Serial.print(bStr ? bStr : "null");
  Serial.print(F(" board="));
  Serial.print(board);
  Serial.print(F(" type="));
  Serial.println(type);

  if (board != boardNo) {
    Serial.println(F("[setupLayout] board mismatch, ignoring"));
    return;
  }

  if (type == "turnout") {
    const char* turnoutName = doc["turnout"] | "";

    const char* tStr  = setup0["T"];
    const char* dStr  = setup0["D"];
    const char* pStr  = setup0["P"];
    const char* tvStr = setup0["Tv"];
    const char* cvStr = setup0["Cv"];

    int to  = tStr  ? atoi(tStr)  : -1;
    int pwm = dStr  ? atoi(dStr)  : 0;
    int pin = pStr  ? atoi(pStr)  : 0;
    int Tv  = tvStr ? atoi(tvStr) : 1500;
    int Cv  = cvStr ? atoi(cvStr) : 1500;

    Serial.print(F("[setupLayout] Turnout "));
    Serial.print(turnoutName);
    Serial.print(F(" to="));
    Serial.print(to);
    Serial.print(F(" pwm="));
    Serial.print(pwm);
    Serial.print(F(" pin="));
    Serial.println(pin);

    if (to >= 0 && to < maxTurnout) {
      turnouts[to] = Turnout(turnoutName, pwm, pin, Tv, Cv);
    } else {
      Serial.print(F("Turnout index out of bounds: "));
      Serial.println(to);
    }

  } else if (type == "sensor") {
    const char* sensorName = doc["sensor"] | "";

    const char* sStr = setup0["S"];
    const char* pStr = setup0["P"];
    const char* iStr = setup0["I"];

    int sensorNo = sStr ? atoi(sStr) : -1;
    int pin      = pStr ? atoi(pStr) : -1;
    int invert   = iStr ? atoi(iStr) : 0;

    Serial.print(F("[setupLayout] Sensor setup: index="));
    Serial.print(sensorNo);
    Serial.print(F(" name="));
    Serial.print(sensorName);
    Serial.print(F(" pin="));
    Serial.print(pin);
    Serial.print(F(" invert="));
    Serial.println(invert);

    if (sensorNo >= 0 && sensorNo < maxSensor) {
      sensors[sensorNo] = Sensor(sensorName, sensorNo, pin, invert);
      Serial.print(F("[setupLayout] Stored sensor "));
      Serial.print(sensorName);
      Serial.print(F(" at index "));
      Serial.print(sensorNo);
      Serial.print(F(" on pin "));
      Serial.println(pin);
    } else {
      Serial.print(F("Sensor index out of bounds: "));
      Serial.println(sensorNo);
    }

  } else if (type == "signal") {
    const char* signalHeadName = doc["head"] | "";

    const char* sStr  = setup0["S"];
    const char* dStr  = setup0["D"];
    const char* pStr  = setup0["P"];
    const char* tvStr = setup0["Tv"];
    const char* cvStr = setup0["Cv"];

    int to  = sStr  ? atoi(sStr)  : -1;
    int pwm = dStr  ? atoi(dStr)  : 0;
    int pin = pStr  ? atoi(pStr)  : 0;
    int Tv  = tvStr ? atoi(tvStr) : 1500;
    int Cv  = cvStr ? atoi(cvStr) : 1500;

    Serial.print(F("[setupLayout] Signal "));
    Serial.print(signalHeadName);
    Serial.print(F(" to="));
    Serial.print(to);
    Serial.print(F(" pwm="));
    Serial.print(pwm);
    Serial.print(F(" pin="));
    Serial.println(pin);

    if (to >= 0 && to < maxSignal) {
      signals[to] = Signal(signalHeadName, pwm, pin, Tv, Cv);
    } else {
      Serial.print(F("Signal index out of bounds: "));
      Serial.println(to);
    }
  
    } else if (type == "signalMast") {

    const char* mastName = doc["mast"] | "";
    const char* smStr = setup0["SM"];
    const char* dStr  = setup0["D"];
    const char* prStr = setup0["Pr"];
    const char* caStr = setup0["Ca"];
    const char* cpStr = setup0["Cp"];
    const char* daStr = setup0["Da"];
    const char* ovStr = setup0["Ov"];
    const char* lvStr = setup0["Lv"];
  const char* prLvStr = setup0["PrLv"];
  const char* caLvStr = setup0["CaLv"];
  const char* cpLvStr = setup0["CpLv"];
  const char* daLvStr = setup0["DaLv"];

    // Optional feathers
    const char* ax1Str = setup0["Ax1"];
    const char* ax2Str = setup0["Ax2"];
    const char* ax3Str = setup0["Ax3"];
    const char* ax4Str = setup0["Ax4"];
    const char* ax5Str = setup0["Ax5"];
    const char* ax6Str = setup0["Ax6"];

      const char* ax1LvStr = setup0["Ax1Lv"];
  const char* ax2LvStr = setup0["Ax2Lv"];
  const char* ax3LvStr = setup0["Ax3Lv"];
  const char* ax4LvStr = setup0["Ax4Lv"];
  const char* ax5LvStr = setup0["Ax5Lv"];
  const char* ax6LvStr = setup0["Ax6Lv"];

    int idx   = smStr ? atoi(smStr) : -1;
    int pwm   = dStr  ? atoi(dStr)  : 0;
    int prPin = prStr ? atoi(prStr) : -1;
    int caPin = caStr ? atoi(caStr) : -1;
    int cpPin = cpStr ? atoi(cpStr) : -1;
    int daPin = daStr ? atoi(daStr) : -1;
    int ov    = ovStr ? atoi(ovStr) : 0;
    int lv    = lvStr ? atoi(lvStr) : 4096;
      // Per-LED lit levels, fall back to global Lv if not specified
  uint16_t prLv = prLvStr ? atoi(prLvStr) : lv;
  uint16_t caLv = caLvStr ? atoi(caLvStr) : lv;
  uint16_t cpLv = cpLvStr ? atoi(cpLvStr) : lv;
  uint16_t daLv = daLvStr ? atoi(daLvStr) : lv;

    // Per-LED lit levels, fall back to global Lv if not specified
  uint16_t ax1Lv = ax1LvStr ? atoi(ax1LvStr) : lv;
  uint16_t ax2Lv = ax2LvStr ? atoi(ax2LvStr) : lv;
  uint16_t ax3Lv = ax3LvStr ? atoi(ax3LvStr) : lv;
  uint16_t ax4Lv = ax4LvStr ? atoi(ax4LvStr) : lv;
  uint16_t ax5Lv = ax5LvStr ? atoi(ax5LvStr) : lv;
  uint16_t ax6Lv = ax6LvStr ? atoi(ax6LvStr) : lv;

    int ax1Pin = ax1Str ? atoi(ax1Str) : -1;
    int ax2Pin = ax2Str ? atoi(ax2Str) : -1;
    int ax3Pin = ax3Str ? atoi(ax3Str) : -1;
    int ax4Pin = ax4Str ? atoi(ax4Str) : -1;
    int ax5Pin = ax5Str ? atoi(ax5Str) : -1;
    int ax6Pin = ax6Str ? atoi(ax6Str) : -1;

    Serial.print(F("[setupLayout] SignalMast "));
    Serial.print(mastName);
    Serial.print(F(" idx="));
    Serial.print(idx);
    Serial.print(F(" pwm="));
    Serial.print(pwm);
    Serial.print(F(" Pr=")); Serial.print(prPin);
    Serial.print(F(" Ca=")); Serial.print(caPin);
    Serial.print(F(" Cp=")); Serial.print(cpPin);
    Serial.print(F(" Da=")); Serial.print(daPin);
    Serial.print(F(" Ax1=")); Serial.print(ax1Pin);
    Serial.print(F(" Ax2=")); Serial.print(ax2Pin);
    Serial.print(F(" Ax3=")); Serial.print(ax3Pin);
    Serial.print(F(" Ax4=")); Serial.print(ax4Pin);
    Serial.print(F(" Ax5=")); Serial.print(ax5Pin);
    Serial.print(F(" Ax6=")); Serial.println(ax6Pin);

    if (idx >= 0 && idx < maxSignalMast) {
      signalMasts[idx] = SignalMast(
        mastName, pwm,
        prPin, caPin, cpPin, daPin,
        ov, lv,
        ax1Pin, ax2Pin, ax3Pin, ax4Pin, ax5Pin, ax6Pin
      );


    mastOffVal[idx]   = ov;
    mastPrLitVal[idx] = prLv;
    mastCaLitVal[idx] = caLv;
    mastCpLitVal[idx] = cpLv;
    mastDaLitVal[idx] = daLv;

    mastAx1LitVal[idx] = ax1Lv;
    mastAx2LitVal[idx] = ax2Lv;
    mastAx3LitVal[idx] = ax3Lv;
    mastAx4LitVal[idx] = ax4Lv;
    mastAx5LitVal[idx] = ax5Lv;
    mastAx6LitVal[idx] = ax6Lv;

      Serial.print(F("[setupLayout] Stored signal mast "));
      Serial.print(mastName);
      Serial.print(F(" @ index "));
      Serial.println(idx);
    } else {
      Serial.print(F("[setupLayout] Signal mast index out of bounds: "));
      Serial.println(idx);
    }
  }

  

  Serial.print(F("- SRAM left: "));
  Serial.println(freeRam());
}

// ================== SERVO MOVE ==================
void moveTurnOut(int turnoutBoard, int turnoutPin, int newPWMVal) {
  if (turnoutBoard == 0) {
    pwm0.writeMicroseconds(turnoutPin, newPWMVal);
  } else {
    pwm1.writeMicroseconds(turnoutPin, newPWMVal);
  }
}

void setTurnOut(String turnout, String message) {
  Serial.println("Turnout to move: " + turnout + " message: " + message);
  Turnout* turnoutDetail = findTurnoutByName(turnout);
  if (!turnoutDetail) return;

  if (message == "CLOSED") {
    turnoutDetail->requiredPWMVal = turnoutDetail->getCv();
    turnoutDetail->previousMillis = millis();
  } else if (message == "THROWN") {
    turnoutDetail->requiredPWMVal = turnoutDetail->getTv();
    turnoutDetail->previousMillis = millis();
  }
}

void changeSignal(String signal, String message) {
  Serial.println("Signal Change");
  Signal* signalDetail = findSignalByName(signal);
  if (!signalDetail) return;

  int signalBoard = signalDetail->getPwmBoard();
  int signalPin   = signalDetail->getPwmPin();
  int signalTv    = signalDetail->getTv();
  int signalCv    = signalDetail->getCv();

  if (message == "CLOSED") {
    Serial.println(signalPin);
    if (signalBoard == 0) pwm0.writeMicroseconds(signalPin, signalCv);
    else                  pwm1.writeMicroseconds(signalPin, signalCv);
  } else if (message == "THROWN") {
    Serial.println(signalPin);
    if (signalBoard == 0) pwm0.writeMicroseconds(signalPin, signalTv);
    else                  pwm1.writeMicroseconds(signalPin, signalTv);
  }
}
void changeSignalMast(String mastName, String message) {
  Serial.println(F("[SignalMast] changeSignalMast"));

  SignalMast* mast = findSignalMastByName(mastName);
  if (!mast) {
    Serial.print(F("[SignalMast] Unknown mast "));
    Serial.println(mastName);
    return;
  }

  int idx = getSignalMastIndexByName(mastName);
  if (idx < 0) {
    Serial.print(F("[SignalMast] No index for mast "));
    Serial.println(mastName);
    return;
  }

  // Normalise to upper case for easier matching
  message.toUpperCase();

 int board = mast->getPwmBoard();

// Use our per-mast brightness tables
uint16_t offVal  = mastOffVal[idx];
uint16_t prLv    = mastPrLitVal[idx];
uint16_t caLv    = mastCaLitVal[idx];
uint16_t cpLv    = mastCpLitVal[idx];
uint16_t daLv    = mastDaLitVal[idx];

uint16_t ax1Lv    = mastAx1LitVal[idx];
uint16_t ax2Lv    = mastAx2LitVal[idx];
uint16_t ax3Lv    = mastAx3LitVal[idx];
uint16_t ax4Lv    = mastAx4LitVal[idx];
uint16_t ax5Lv    = mastAx5LitVal[idx];
uint16_t ax6Lv    = mastAx6LitVal[idx];


auto setPin = [&](int pin, uint16_t litVal, bool on) {
  if (pin < 0) return; // allow missing LEDs
  uint16_t val = on ? litVal : offVal;
  if (board == 0) {
    pwm0.setPWM(pin, 0, val);
  } else {
    pwm1.setPWM(pin, 0, val);
  }
};


  // Reset any previous flashing state for this mast
  mastFlashEnabled[idx] = false;
  mastFlashCa[idx]      = false;
  mastFlashCp[idx]      = false;
  mastFlashOn[idx]      = false;

 // First: all off
setPin(mast->getPrPin(), prLv, false);
setPin(mast->getCaPin(), caLv, false);
setPin(mast->getCpPin(), cpLv, false);
setPin(mast->getDaPin(), daLv, false);

// Then apply aspect
if (message == "PROCEED" || message == "CLEAR" || message == "GREEN") {
  setPin(mast->getPrPin(), prLv, true);

} else if (message == "CAUTION" || message == "YELLOW" || message == "SINGLE_YELLOW") {
  setPin(mast->getCaPin(), caLv, true);

} else if (message == "PRELIM_CAUTION" || message == "DOUBLE_YELLOW" || message == "PRELIM") {
  setPin(mast->getCaPin(), caLv, true);
  setPin(mast->getCpPin(), cpLv, true);

} else if (message == "DANGER" || message == "STOP" || message == "RED") {
  setPin(mast->getDaPin(), daLv, true);

} else if (message == "TEST") {
  setPin(mast->getDaPin(), daLv, true);
  setPin(mast->getCaPin(), caLv, true);
  setPin(mast->getCpPin(), cpLv, true);
  setPin(mast->getPrPin(), prLv, true);

} else if (message == "OFF" || message == "DARK") {
    // All stay off

  } else if (message == "FLASH_CAUTION") {
    // Flash single yellow
    mastFlashEnabled[idx]      = true;
    mastFlashCa[idx]           = true;
    mastFlashCp[idx]           = false;
    mastFlashOn[idx]           = false;
    mastLastFlashToggle[idx]   = millis();

  } else if (message == "FLASH_PRELIM_CAUTION") {
    // Flash both yellows (preliminary caution)
    mastFlashEnabled[idx]      = true;
    mastFlashCa[idx]           = true;
    mastFlashCp[idx]           = true;
    mastFlashOn[idx]           = false;
    mastLastFlashToggle[idx]   = millis();

  } else {
    Serial.print(F("[SignalMast] Unknown aspect string: "));
    Serial.println(message);
  }
}

void handleJMRISignalMastMessage(const String& topic, const String& msg) {
  Serial.print(F("[JMRI Mast] topic="));
  Serial.println(topic);
  Serial.print(F("[JMRI Mast] msg="));
  Serial.println(msg);

  // Extract mastId from the topic (e.g. "B9|S1")
  String mastId = getMastIdFromTopic(topic);
  if (mastId.length() == 0) {
    Serial.println(F("[JMRI Mast] Could not extract mastId from topic"));
    return;
  }

  // Aspect is the first segment before ';'
  int semiPos = msg.indexOf(';');
  String aspect = (semiPos >= 0) ? msg.substring(0, semiPos) : msg;
  aspect.trim();   // e.g. "Test", "Caution", "Preliminary Caution"

  // Determine Lit / Unlit from the whole message
  String rightUpper = msg;
  rightUpper.toUpperCase();
  bool isLit = (rightUpper.indexOf("UNLIT") == -1);  // if "Unlit" present => dark

  // Normalise aspect string
  String upperAspect = aspect;
  upperAspect.toUpperCase();

  bool   isFlash    = false;
  String baseAspect = upperAspect;

  if (baseAspect.startsWith("FLASH")) {
    isFlash = true;
    baseAspect = baseAspect.substring(5); // after "FLASH"
    baseAspect.trim();
  }

  String normalized;

  if (!isLit) {
    normalized = "OFF";

  } else if (isFlash && baseAspect == "CAUTION") {
    normalized = "FLASH_CAUTION";

  } else if (isFlash && (baseAspect == "PRELIMINARY CAUTION" || baseAspect == "PRELIMINARY_CAUTION")) {
    normalized = "FLASH_PRELIM_CAUTION";

  } else if (baseAspect == "PROCEED") {
    normalized = "PROCEED";

  } else if (baseAspect == "CAUTION") {
    normalized = "CAUTION";

  } else if (baseAspect == "PRELIMINARY CAUTION" || baseAspect == "PRELIMINARY_CAUTION") {
    normalized = "PRELIM_CAUTION";

  } else if (baseAspect == "TEST") {
    // Your special all-lamps-on test aspect
    normalized = "TEST";

  } else {
    Serial.print(F("[JMRI Mast] Unknown aspect '"));
    Serial.print(aspect);
    Serial.println(F("', passing through raw"));
    normalized = baseAspect; // last resort
  }

  Serial.print(F("[JMRI Mast] mastId="));
  Serial.print(mastId);
  Serial.print(F(" aspect="));
  Serial.println(normalized);

  // Re-use your main mast logic
  changeSignalMast(mastId, normalized);
}

void handleJMRIJunctionFeatherMessage(const String& topic, const String& msg) {
  Serial.print(F("[JMRI Feather] topic="));
  Serial.println(topic);
  Serial.print(F("[JMRI Feather] msg="));
  Serial.println(msg);

  String mastId = getMastIdFromTopic(topic);  // e.g. "B9|S1"
  if (mastId.length() == 0) {
    Serial.println(F("[JMRI Feather] Cannot parse mast from topic"));
    return;
  }

  SignalMast* mast = findSignalMastByName(mastId);
  if (!mast) {
    Serial.print(F("[JMRI Feather] Unknown mast id for feathers: "));
    Serial.println(mastId);
    return;
  }

  int idx = getSignalMastIndexByName(mastId);
  if (idx < 0) {
    Serial.print(F("[JMRI Feather] No index for mast (feathers) "));
    Serial.println(mastId);
    return;
  }

  int      board  = mast->getPwmBoard();
  uint16_t offVal = mastOffVal[idx];

  auto setFeather = [&](int featherIndex, bool on) {
    int pin = -1;
    uint16_t litVal = offVal;

    switch (featherIndex) {
      case 1:
        pin    = mast->getFeatherPin(1);
        litVal = mastAx1LitVal[idx];
        break;
      case 2:
        pin    = mast->getFeatherPin(2);
        litVal = mastAx2LitVal[idx];
        break;
      case 3:
        pin    = mast->getFeatherPin(3);
        litVal = mastAx3LitVal[idx];
        break;
      case 4:
        pin    = mast->getFeatherPin(4);
        litVal = mastAx4LitVal[idx];
        break;
      case 5:
        pin    = mast->getFeatherPin(5);
        litVal = mastAx5LitVal[idx];
        break;
      case 6:
        pin    = mast->getFeatherPin(6);
        litVal = mastAx6LitVal[idx];
        break;
      default:
        return;  // invalid
    }

    if (pin < 0) return;

    uint16_t val = on ? litVal : offVal;
    if (board == 0) {
      pwm0.setPWM(pin, 0, val);
    } else {
      pwm1.setPWM(pin, 0, val);
    }
  };

  // First word of msg before ';' is the level: Zero/One/Two/...
  int semiPos = msg.indexOf(';');
  String level = (semiPos >= 0) ? msg.substring(0, semiPos) : msg;
  level.trim();
  level.toUpperCase();

  int featherIndex = 0;

  if      (level == "ZERO")   featherIndex = 0;
  else if (level == "ONE")    featherIndex = 1;
  else if (level == "TWO")    featherIndex = 2;
  else if (level == "THREE")  featherIndex = 3;
  else if (level == "FOUR")   featherIndex = 4;
  else if (level == "FIVE")   featherIndex = 5;
  else if (level == "SIX")    featherIndex = 6;
  else {
    Serial.print(F("[JMRI Feather] Unknown feather level: "));
    Serial.println(level);
    featherIndex = 0;  // safe: all off
  }

  Serial.print(F("[JMRI Feather] mastId="));
  Serial.print(mastId);
  Serial.print(F(" featherIndex="));
  Serial.println(featherIndex);

  // Turn all feathers off first
  for (int i = 1; i <= 6; i++) {
    setFeather(i, false);
  }

  // If non-zero, turn the selected feather on
  if (featherIndex > 0) {
    setFeather(featherIndex, true);
  }
}




// ================== MQTT HELPERS ==================
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    bool ok = client.connect("ArduinoMegaWiFi", mqtt_user, mqtt_pass);
    Serial.print(" result="); Serial.print(ok ? "OK" : "FAIL");
    Serial.print(" state="); Serial.println(client.state());  // 0 on success
    if (ok) {
      Serial.println("Connected to MQTT broker");
      client.subscribe(mqtt_topic);
      break;
    }
    delay(5000);
  }
}

void PublishToMQTT(const String& topic, const String& message, bool retain) {
  client.publish(topic.c_str(), message.c_str(), message.length(), retain);
}

// ================== PWM BOARDS ==================
void setupPWMBoards() {
  pwm0 = Adafruit_PWMServoDriver();
  pwm0.begin();
  pwm0.setPWMFreq(50);

  pwm1 = Adafruit_PWMServoDriver(0x41);
  pwm1.begin();
  pwm1.setPWMFreq(50);
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial2.begin(9600);  // RFID
  Wire.begin();

  Serial.println(F("[setup] Booting..."));

  // Init WiFi via ESP-01S on Serial3 and let it auto-join
  ensureWiFiAttachedAndJoined();

  // Init PWM boards
  setupPWMBoards();

  // Sensor shield inputs (22..50 inclusive)
  for (int i = 22; i < 51; i++) {
    pinMode(i, INPUT_PULLUP);
  }

  // MQTT
  client.setServer(mqtt_server, mqtt_port);
  wifiClient.setTimeout(10000);   // 10s read timeout for the TCP socket
  client.setSocketTimeout(20);    // MQTT waits (default 15s)
  client.setKeepAlive(25);        // keepalive seconds
  client.setBufferSize(512);      // increase if sending larger JSON
  client.setCallback(callback);

  Serial.println(F("[setup] Complete"));
}

// ================== SENSORS ==================
void checkSensors() {
  for (int i = 0; i < maxSensor; i++) {
    int sensorPin = sensors[i].getPin();
    const String& sensorName = sensors[i].getName();

    if (sensorName.length() == 0) continue;
    if (sensorPin < 22 || sensorPin > 50) continue;

    int currentState = sensors[i].getCurrentState();
    int newState     = digitalRead(sensorPin);

    // Only decide ACTIVE/INACTIVE if something actually changed
    if (newState != currentState) {
      String sensorState = (newState == 0) ? "ACTIVE" : "INACTIVE";

      // Minimal, single-line log per change
      Serial.print(F("[Sensor] "));
      Serial.print(sensorName);
      Serial.print(F(" (pin "));
      Serial.print(sensorPin);
      Serial.print(F(") -> "));
      Serial.println(sensorState);

      sensors[i].updateSensor(newState);
      String newTopic = String(sensorTopic) + sensorName;
      PublishToMQTT(newTopic, sensorState, true);
    }
  }
}
void updateSignalMastFlashing() {
  unsigned long now = millis();

  for (int i = 0; i < maxSignalMast; i++) {
    if (!mastFlashEnabled[i]) continue;
    if (now - mastLastFlashToggle[i] < mastFlashInterval) continue;

    mastLastFlashToggle[i] = now;
    mastFlashOn[i] = !mastFlashOn[i];

    SignalMast& mast = signalMasts[i];
    int board        = mast.getPwmBoard();
    uint16_t offVal  = mastOffVal[i];
    uint16_t caLv    = mastCaLitVal[i];
    uint16_t cpLv    = mastCpLitVal[i];

    auto setPin = [&](int pin, uint16_t litVal, bool on) {
      if (pin < 0) return;
      uint16_t val = on ? litVal : offVal;
      if (board == 0) {
        pwm0.setPWM(pin, 0, val);
      } else {
        pwm1.setPWM(pin, 0, val);
      }
    };

    // Green and red always off for flashing
    setPin(mast.getPrPin(), mastPrLitVal[i], false);
    setPin(mast.getDaPin(), mastDaLitVal[i], false);

    bool onPhase = mastFlashOn[i];

    if (mastFlashCa[i]) {
      setPin(mast.getCaPin(), caLv, onPhase);
    } else {
      setPin(mast.getCaPin(), caLv, false);
    }

    if (mastFlashCp[i]) {
      setPin(mast.getCpPin(), cpLv, onPhase);
    } else {
      setPin(mast.getCpPin(), cpLv, false);
    }
  }
}


// ================== LOOP ==================
void loop() {
  // Read WiFi status once per loop
  int s = WiFi.status();

  // Edge: WiFi just connected
  if (s == WL_CONNECTED && prevWifiStatus != WL_CONNECTED) {
    Serial.print(F("[WiFi] Connected. IP: "));
    Serial.println(WiFi.localIP());
    Serial.print(F("[WiFi] RSSI: "));
    Serial.println(WiFi.RSSI());

    // (Re)connect MQTT once WiFi is truly up
    reconnectMQTT();

    // Optional boot heartbeat
    if (client.connected() && !mqttBootPublished) {
      client.publish("myTrains/system/mega2560/status", "online", true);
      mqttBootPublished = true;
    }
  }

  // Not connected yet — don't spam; let ESP auto-join
  if (s != WL_CONNECTED) {
    if (millis() - lastStatusNote > 15000UL) {
      Serial.println(F("[WiFi] Not connected yet — waiting for ESP auto-join..."));
      lastStatusNote = millis();
    }
  }

  // MQTT service loop only when WiFi up
  if (s == WL_CONNECTED) {
    if (!client.connected()) reconnectMQTT();
    client.loop();
  }

  // RFID readers (currently disabled; set NumberOfRFIDReaders>0 AND rfidEnabled=true to use)
  bool rfidEnabled = false;
#if NumberOfRFIDReaders > 0
  if (rfidEnabled) {
    for (int r = 0; r < NumberOfRFIDReaders; r++) {
      bool newTag = TagReaders[r].CheckForTag();
      if (newTag) {
        Serial.println("Found " + TagReaders[r].Name);
        String topic = "myTrains/track/reporter/" + TagReaders[r].Name;
        PublishToMQTT(topic, String(TagReaders[r].Tag), false);
      }
    }
  }
#endif

  // Turnout stepping
  for (int t = 0; t < maxTurnout; t++) {
    int turnoutPWM         = turnouts[t].currentPWMVal;
    int turnoutRequiredPwm = turnouts[t].requiredPWMVal;

    if (turnoutPWM != turnoutRequiredPwm &&
        millis() - turnouts[t].previousMillis >= stepTime) {
      int stepDirection = (turnoutRequiredPwm > turnoutPWM) ? stepSize : -stepSize;
      int newPWMValue   = turnoutPWM + stepDirection;

      if ((stepDirection > 0 && newPWMValue > turnoutRequiredPwm) ||
          (stepDirection < 0 && newPWMValue < turnoutRequiredPwm)) {
        newPWMValue = turnoutRequiredPwm;
      }

      moveTurnOut(turnouts[t].pwmBoard, turnouts[t].pwmPin, newPWMValue);
      turnouts[t].currentPWMVal  = newPWMValue;
      turnouts[t].previousMillis = millis();
    }
  }

  // Debounced sensor read
  if (millis() - lastReadTime > debounceDelay) {
    lastReadTime = millis();
    checkSensors();
  }
  updateSignalMastFlashing();
  prevWifiStatus = s;
}
