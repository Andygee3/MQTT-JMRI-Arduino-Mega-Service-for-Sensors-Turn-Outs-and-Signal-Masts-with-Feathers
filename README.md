MQTT–JMRI–Arduino Mega Service for Sensors, Turnouts & Signal Masts (with Feathers)

This project links JMRI, MQTT, and multiple Arduino Mega 2560 boards to create a distributed, configurable control system for:

Slow-motion turnouts

Sensor feedback

Signal heads

Full signal masts with flashing aspects and junction feathers (1–6)

All device configuration (pins, PWM values, brightness, addresses, etc.) is stored and controlled in JMRI only, using Memory Variables that publish JSON configuration objects.
The Arduino boards require no individual programming changes when hardware is rearranged — simply update the JSON in JMRI.

This approach is inspired by Vintage80sModelRailway, but significantly extended to support full UK-style signalling, feathers, per-LED brightness, board auto-configuration, and robust JMRI → MQTT → Arduino state flow.

Features
🔧 Hardware / Control

Multiple Arduino Mega boards connected via WiFi (ESP-01S in AT mode)

One or more PCA9685 16-channel PWM drivers per board

Supports servos, LEDs, multi-aspect heads, and masts

Slow-motion turnout driver with stepped servo movement

Sensor inputs with debounce and invert support

🚦 Signalling

Signal heads mapped as turnout-like devices with independent brightness

Full Signal Mast implementation:

PROCEED (Green)

CAUTION (Yellow)

PRELIMINARY CAUTION (Double Yellow)

DANGER (Red)

TEST (all lamps on)

OFF/DARK

Flashing aspects:

FLASH_CAUTION

FLASH_PRELIM_CAUTION

Junction feathers 1–6 with independent brightness

🧠 Configuration Architecture

Each Arduino board identifies itself using a single value: B = Board ID

All configuration delivered via JSON from JMRI Memory Variables

No upload/programming needed to change:

Sensor pins

Servo values

LED brightness

Masts, feathers, or aspect logic

📡 MQTT Integration

All devices publish/receive MQTT messages under the myTrains/ namespace

JMRI drives signals and turnouts directly via MQTT topics

Arduino publishes sensor activity and online status

High-Level Architecture

              ┌──────────────────────┐
              │         JMRI         │
              │  (Turnouts, Masts,   │
              │   Sensors, Memory)   │
              └─────────┬────────────┘
                        │ MQTT (JSON Setup + Aspect Changes)
                        ▼
               ┌─────────────────────┐
               │    Mosquitto MQTT   │
               │     Broker/Pi       │
               └─────────┬───────────┘
                         │
            ┌────────────┴────────────┐
       ┌──────────┐            ┌──────────┐
       │ Arduino  │ WiFi/AT    │ Arduino  │ WiFi/AT
       │   Mega   ├────────────┤   Mega   ├──────────
       └──────────┘            └──────────┘
            │ PCA9685               │ PCA9685
            ▼                       ▼
       Turnouts / Sensors / Signals / Masts / Feathers


Hardware Requirements
Required

1+ Arduino Mega 2560

1x ESP-01S or ESP8266 running AT firmware
(connected to Mega Serial3)

1–2x PCA9685 16-channel PWM boards
(extendable; default build supports 2)

Stable 5V PSU for LEDs/servos

Sensor shield or direct wiring to pins 22–50

Optional

RFID readers (Serial2)

Additional PCA boards

Wiring overview

ESP-01S → Serial3 TX3/RX3

PCA9685 SDA/SCL → Mega SDA/SCL (Wire)

Sensors → Pins 22–50

LEDs/Servos → PCA9685 channels

JMRI JSON Setup Specification

Every hardware object is defined by a JSON object inside a JMRI Memory Variable.

The only setting configured on the Arduino is:

#define boardNo 0


JMRI sends JSON with "B":"0" so the board knows which items apply to it.

Turnout Setup
Example (Memory Variable: B10:T1)
{"type":"turnout","turnout":"B10:T1",
 "setup":[{"B":"0","D":"0","P":"1","T":"0","Tv":"1500","Cv":"2000"}]}

Field	Meaning
type	"turnout"
turnout	Unique traction ID (used in MQTT topics)
B	Board ID
D	PCA9685 board index (0–n)
P	PCA channel (0–15)
T	Turnout index for class array (0–15)
Tv	Thrown PWM
Cv	Closed PWM
Sensor Setup
Example
{"type":"sensor","sensor":"PD2",
 "setup":[{"B":"0","P":"31","I":"0","S":"9"}]}

Field	Meaning
type	"sensor"
sensor	ID used in MQTT topic
B	Board ID
P	Pin (22–50)
I	Invert (0/1)
S	Sensor index (0–32)
Signal Head Setup
{"type":"signal","head":"B9:S1:F",
 "setup":[{"B":"0","D":"0","P":"4","S":"9","Tv":"0","Cv":"4096"}]}


Cv = ON brightness
Tv = OFF brightness

Signal Mast Setup (with Feathers)
Example (Memory Variable: SM1)
{
  "type": "signalMast",
  "mast": "B9|S1",
  "setup": [{
    "B":"0","D":"0",
    "Pr":"15","Ca":"14","Cp":"13","Da":"12",
    "Ax1":"11",
    "SM":"0",
    "Ov":"4096","Lv":"1095",
    "PrLv":"300","CaLv":"2600","CpLv":"2600","DaLv":"3500",
    "Ax1Lv":"1095"
  }]
}

Field Definitions
Field	Description
Pr	Proceed lamp pin
Ca	Caution lamp pin
Cp	Preliminary Caution lamp pin
Da	Danger (Red) lamp pin
Ax1–Ax6	Feather pins 1–6
Ov	Off PWM value (typically 4096)
Lv	Default lit value
?Lv	Per-aspect override (e.g. PrLv, DaLv, Ax3Lv)

This allows full brightness tuning per lamp.

MQTT Topic Conventions
Purpose	Topic	Payload
Turnout command	myTrains/track/turnout/<id>	CLOSED, THROWN
Turnout setup	myTrains/turnout/setup/<id>	JSON
Sensor feedback	myTrains/track/sensor/<name>	ACTIVE, INACTIVE
Signal head	myTrains/track/turnout/<head>	CLOSED/THROWN
Signal mast aspect	myTrains/track/signalmast/{0}<mast>	e.g. Caution; Lit; Unheld
Feather	`myTrains/track/signalmast/{0}<mast>	F`
Board status	myTrains/system/mega2560/status	online
PWM / Brightness Model

The PCA9685 supports:

0–4095 = valid PWM duty cycle

4096 = full off

For signal masts:

Item	Value Source
OFF	Ov
Default ON	Lv
Per-lamp ON	e.g., PrLv, CaLv, DaLv
Per-feather ON	Ax1Lv … Ax6Lv

This allows:

dimmer green

bright red

balanced yellows

subtle feathers

no firmware changes required

Building & Uploading

Clone the repository.

Install required Arduino libraries:

Adafruit_PWMServoDriver

ArduinoJson

WiFiEspAT

PubSubClient

Edit in main.ino:

#define boardNo 0
const char* mqtt_server = "192.168.x.x";


Flash to the Arduino Mega.

Configure JMRI Memory Variables for turnouts, sensors, heads, and masts.

Reboot the Arduino — it will self-configure via MQTT JSON.

Credits

Based on ideas from Vintage80sModelRailway

Extended to support:

JMRI Signal Mast + Feather support

Per-lamp & per-feather brightness

Fully remote board configuration via JSON

Advanced flashing aspects

Multiple boards on one layout


# JMRI Script to Send Memories to MQTT

    import jmri
    import java
      from org.python.core.util import StringUtil

      memories = jmri.InstanceManager.getDefault(jmri.MemoryManager)

      mqttAdapter = jmri.InstanceManager.getDefault(jmri.jmrix.mqtt.MqttSystemConnectionMemo).getMqttAdapter()

      for memory in memories.getNamedBeanSet():
        if memory.getSystemName() not in ["IMRATEFACTOR", "IMCURRENTTIME"]:
        topic = "turnout/setup/"  
        mem = memory.getValue()
        memName = memory.getUserName()
        
        if memName and mem:  # Ensure values are not None
            topic += memName
            mqttAdapter.publish(topic, str(mem))  # Convert to string before publishing
