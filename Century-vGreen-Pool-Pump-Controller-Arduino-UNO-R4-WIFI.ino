/* VERSION: 1.1 - 2026-04-24 11:35 CT
============================================================
Pool Pump Controller – Arduino UNO R4 WiFi
============================================================

CHANGELOG:
- v1.1:
  - Added Resume Schedule command/button after STOP during active schedule
  - STOP now places active schedule into manual hold-off until Resume Schedule or schedule window changes
  - Run tab Stop button automatically changes to Resume Schedule while schedule hold-off is active

- v1.0:
  - First stable public release
  - Non-blocking control architecture finalized
  - Lightweight web UI with lazy loading
  - EEPROM settings persistence implemented
  - Freeze protection, schedules, overrides, and keepalive integrated

OVERVIEW:
This project implements a fully self-contained pool pump controller
using an Arduino UNO R4 WiFi. The controller communicates with a
variable-speed pump over RS-485 using a custom EPC-style protocol
(not Modbus) and provides a lightweight web-based user interface.

The system is designed for reliability and continuous operation,
with non-blocking control logic, watchdog-style recovery, and
reduced memory pressure to help prevent lockups.

ENTRY POINT:
- setup(): Initializes hardware, WiFi, RTC, EEPROM, and web server
- loop(): Runs cooperative non-blocking control tasks continuously

------------------------------------------------------------
CORE FEATURES:
------------------------------------------------------------
- Local Access Point + Web UI
- RS-485 pump communication using custom EPC protocol
- Prime-first startup for all run conditions
- Non-blocking start sequence and RPM ramping
- 3 programmable schedules with priority (1 > 2 > 3)
- Manual override modes (High / Low)
- Manual RPM update during active schedule/override operation
- Freeze protection based on ambient temperature
- Auxiliary relay control via pump configuration commands
- EEPROM-backed settings persistence
- Optional home WiFi connection and NTP time sync
- Lightweight JSON API endpoints for UI:
    /api/live
    /api/schedules
    /api/setup
    /api/faults

------------------------------------------------------------
HARDWARE INTERFACE:
------------------------------------------------------------
- Controller: Arduino UNO R4 WiFi
- Pump link: RS-485 over Serial1
- Serial settings: 9600 baud, 8-N-1
- RS-485 driver enable pin: GPIO 7 (HIGH = transmit)
- Pump address: 0x15

------------------------------------------------------------
SYSTEM ARCHITECTURE:
------------------------------------------------------------
The controller operates as a state-driven system with cooperative,
non-blocking processes:

1. COMMAND QUEUE
   - User actions are queued and executed sequentially
   - Prevents overlapping commands and command collisions

2. START SEQUENCE
   - Validates pump state
   - Configures auxiliary relay state
   - Stores configuration if needed
   - Starts prime cycle
   - Transitions automatically to target RPM

3. PRIME CONTROL
   - Every start begins with prime mode
   - Prime runs at configured RPM for configured duration
   - RPM changes are blocked until prime completes

4. RAMP ENGINE
   - Smooth RPM transitions up and down
   - Separate ramp behavior for acceleration and deceleration
   - Helps reduce faults from abrupt speed changes

5. KEEPALIVE SYSTEM
   - Periodically re-sends RUN or STOP commands
   - Prevents pump timeout from communication loss

6. SCHEDULE ENGINE
   - Supports 3 schedules with priority handling
   - Starts/stops pump automatically from RTC time
   - Handles overlap and real-time schedule updates

7. OVERRIDE MODES
   - High and Low override modes with duration timers
   - Overrides take priority over schedules
   - Returns to stop, schedule, or freeze mode when complete

8. FREEZE PROTECTION
   - Monitors ambient temperature from pump sensor
   - If below threshold for 30 minutes, starts the pump
   - Runs at fixed RPM until temperature recovers long enough

9. SENSOR + FAULT MONITORING
   - Reads pump status, RPM, power, temperature, and faults
   - Retrieves active and previous fault information
   - Updates UI through lightweight JSON endpoints

10. WATCHDOG-STYLE RECOVERY
   - Detects stalled or lost communication
   - Clears blocked web connections when needed
   - Helps maintain long-term runtime stability

------------------------------------------------------------
WEB UI DESIGN:
------------------------------------------------------------
- Lightweight HTML served from PROGMEM
- Tab-based layout with lazy loading
- Only the active tab polls data
- JSON responses are streamed instead of built as one large buffer
- Designed to reduce UI-related controller lockups

------------------------------------------------------------
PROTOCOL NOTES:
------------------------------------------------------------
- Protocol is custom EPC-style and is not standard Modbus
- CRC uses Modbus-style CRC16, but command framing is proprietary
- Do not use standard Modbus libraries for this controller

------------------------------------------------------------
IMPORTANT NOTES:
------------------------------------------------------------
- This system requires continuous communication with the pump (keepalive)
- Loss of communication for about 30 seconds may stop the pump
- All starts are prime-first by design
- Manual RPM updates are only allowed during active runs
- STOP inhibits auto-restart until Resume Schedule is pressed or the current schedule window changes

------------------------------------------------------------
CONFIGURATION:
------------------------------------------------------------
Before compiling, update:
- Device Access Point SSID/password
- Optional home WiFi credentials

------------------------------------------------------------
DISCLAIMER:
------------------------------------------------------------
This project is provided as-is. Use at your own risk.
Verify wiring, electrical safety, and pump compatibility before
deploying on live equipment.

============================================================
*/

#include <Arduino.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <EEPROM.h>
#include <string.h>
#include <math.h>
#include <RTC.h>

WiFiServer server(80);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);

// =======================
// WiFi CONFIG (REQUIRED: UPDATE BEFORE USE)
// =======================
const char* ssid         = "YOUR_DEVICE_AP_SSID";       // Example: "POOL_PUMP_CONTROLLER"
const char* password     = "YOUR_DEVICE_AP_PASSWORD";   // Leave "" only if intentionally open

const char* homeSsid     = "YOUR_HOME_WIFI_SSID";       // <-- replace with your WiFi name
const char* homePassword = "YOUR_HOME_WIFI_PASSWORD";   // <-- replace with your password

// =======================
// RS-485 / EPC PROTOCOL
// =======================
#define RS485_DE_PIN 7
const uint8_t PUMP_ADDR          = 0x15;
const uint8_t EPC_ACK_CMD        = 0x20;
const uint8_t EPC_ACK_REPLY      = 0x10;

const uint8_t EPC_FN_GO          = 0x41;
const uint8_t EPC_FN_STOP        = 0x42;
const uint8_t EPC_FN_STATUS      = 0x43;
const uint8_t EPC_FN_SETDEMAND   = 0x44;
const uint8_t EPC_FN_READSENSOR  = 0x45;
const uint8_t EPC_FN_CONFIG_RW   = 0x64;
const uint8_t EPC_FN_STORECFG    = 0x65;

const uint8_t EPC_MODE_SPEED     = 0x00;

// Sensor addresses
const uint8_t EPC_SENSOR_PAGE_AMBIENT_TEMP = 0;
const uint8_t EPC_SENSOR_ADDR_AMBIENT_TEMP = 0x07;
const uint8_t EPC_SENSOR_PAGE_PREV_FAULT   = 0;
const uint8_t EPC_SENSOR_ADDR_PREV_FAULT   = 0x09;
const uint8_t EPC_SENSOR_PAGE_FAULTS       = 1;
const uint8_t EPC_SENSOR_ADDR_FAULT1       = 0x15;
const uint8_t EPC_SENSOR_ADDR_FAULT2       = 0x16;
const uint8_t EPC_SENSOR_ADDR_FAULT3       = 0x17;
const uint8_t EPC_SENSOR_ADDR_FAULT4       = 0x18;
const uint8_t EPC_SENSOR_PAGE_RPM          = 0x03;
const uint8_t EPC_SENSOR_ADDR_RPM          = 0x04;
const uint8_t EPC_SENSOR_PAGE_S05          = 0x00;
const uint8_t EPC_SENSOR_ADDR_S05          = 0x05;

const uint8_t EPC_CONFIG_PAGE1_WRITE       = 0x81;
const uint8_t EPC_CONFIG_ADDR_AUX_RELAY    = 0x07;
const uint8_t EPC_AUX_ON_VALUE             = 0x00;
const uint8_t EPC_AUX_OFF_VALUE            = 0x01;

const unsigned long EPC_IDLE_US            = 4500;
const unsigned long EPC_REPLY_TIMEOUT_MS   = 150;
const unsigned long EPC_POST_TX_DELAY_US   = 300;

// =======================
// FREEZE / RAMP / TIMING
// =======================
const int FREEZE_RUN_RPM = 1000;
const unsigned long FREEZE_CONFIRM_MS = 30UL * 60UL * 1000UL;

const int RAMP_UP_STEP_RPM = 500;
const unsigned long RAMP_UP_DELAY_MS = 500;
const int RAMP_DOWN_STEP_RPM = 25;
const unsigned long RAMP_DOWN_DELAY_MS = 250;

const unsigned long KEEPALIVE_INTERVAL_MS = 1000;
const unsigned long STOP_KEEPALIVE_INTERVAL_MS = 1000;
const unsigned long STATUS_POLL_INTERVAL_MS = 5000;
const unsigned long TELEMETRY_POLL_INTERVAL_MS = 5000;
const unsigned long AUX_RETRY_INTERVAL_MS = 5000;
const unsigned long SETTINGS_CHECKPOINT_INTERVAL_MS = 60000UL;
const unsigned long HOME_WIFI_RETRY_INTERVAL_MS = 30000UL;
const unsigned long NTP_SYNC_INTERVAL_MS = 60UL * 60UL * 1000UL;
const unsigned long WEB_RECOVERY_INTERVAL_MS = 5000UL;
const unsigned long START_SEQUENCE_STEP_INTERVAL_MS = 25;

// =======================
// STRUCTS
// =======================
struct Schedule {
  bool enabled;
  int  speed;
  int  startHour;
  int  startMinute;
  char startAMPM[3];
  int  stopHour;
  int  stopMinute;
  char stopAMPM[3];
};

struct ClockTime {
  int  month;
  int  day;
  int  year;
  int  hour;
  int  minute;
  char ampm[3];
};

// =======================
// GLOBALS
// =======================
ClockTime clockTime;
Schedule sched[3];

bool  freezeEnabled;
int   freezeTemp;
int   auxRPM;
float auxHours;
bool  auxEnabled;
int   primeRPM;
int   primeMinutes;
int   overrideHighRPM;
float overrideHighHours;
int   overrideLowRPM;
float overrideLowHours;

// live state
uint8_t lastPumpStatus = 0xFF;
String  lastPumpStatusText = "Unknown";
String  lastPumpStatusHex  = "0xFF";
String  lastPumpReplyText  = "No command yet";

uint8_t lastPrevFaultCode = 0x00;
String  lastPrevFaultCodeHex = "0x00";
String  lastPrevFaultText = "None";

uint8_t lastFault1Code = 0x00;
String  lastFault1CodeHex = "0x00";
String  lastFault1Text = "None";
uint8_t lastFault2Code = 0x00;
String  lastFault2CodeHex = "0x00";
String  lastFault2Text = "None";
uint8_t lastFault3Code = 0x00;
String  lastFault3CodeHex = "0x00";
String  lastFault3Text = "None";
uint8_t lastFault4Code = 0x00;
String  lastFault4CodeHex = "0x00";
String  lastFault4Text = "None";

int16_t lastAmbientTempRaw = 0;
bool ambientTempValid = false;
uint16_t liveRPMRaw = 0;
uint16_t liveS05Raw = 0;
int livePumpRPM = 0;
int livePumpWatts = 0;

unsigned long lastTelemetryPollMillis = 0;
unsigned long lastStatusPollMillis = 0;
unsigned long lastKeepAliveMillis = 0;
unsigned long lastStopKeepAliveMillis = 0;
unsigned long lastGoodStatusMillis = 0;
unsigned long lastAuxRetryMillis = 0;
unsigned long lastSettingsCheckpointMillis = 0;
unsigned long lastHomeWiFiAttemptMillis = 0;
unsigned long lastNTPSyncMillis = 0;
unsigned long lastWebRecoveryMillis = 0;
unsigned long lastLoopHealthyMillis = 0;

bool auxRelayForcedOff = false;

// timer/session
bool runSessionActive = false;
unsigned long runSessionStartMillis = 0;
unsigned long lastRunSessionDurationMs = 0;
String lastStopReason = "None";

// state machine
enum PumpRunState {
  RUNSTATE_IDLE,
  RUNSTATE_PRIMING,
  RUNSTATE_RUNNING,
  RUNSTATE_FAULT
};
PumpRunState pumpRunState = RUNSTATE_IDLE;

bool pendingRunRequest = false;
unsigned long primeStartMillis = 0;
int requestedTargetRPM = 0;
int currentCommandedRPM = 0;

enum ActiveRunMode {
  ACTIVE_NONE,
  ACTIVE_OVERRIDE_HIGH,
  ACTIVE_OVERRIDE_LOW,
  ACTIVE_OVERRIDE_MANUAL,
  ACTIVE_FREEZE
};
ActiveRunMode activeRunMode = ACTIVE_NONE;
unsigned long activeRunStartMillis = 0;
unsigned long activeRunDurationMs = 0;

// freeze state
bool freezeProtectionActive = false;
unsigned long freezeBelowStartMillis = 0;
unsigned long freezeAboveStartMillis = 0;

// schedule state
uint8_t sensorMonitorStep = 0;
int activeScheduleIndex = -1;
int scheduleRequestedRPM = 0;
bool scheduleOwnsPump = false;
bool autoRunInhibited = false;

// active RPM override of active schedule / active override only
bool activeRPMUpdateEnabled = false;
int  activeRPMUpdateValue = 0;
int  activeRPMUpdateScheduleIndex = -1;

// home wifi / time sync state
bool homeWiFiConnected = false;
String homeWiFiIpText = "Not connected";
bool ntpStarted = false;
bool timeSyncOk = false;
String lastTimeSyncText = "Never";

// command queue
enum QueuedCommandType {
  QUEUED_CMD_NONE,
  QUEUED_CMD_STOP,
  QUEUED_CMD_RUN_HIGH,
  QUEUED_CMD_RUN_LOW,
  QUEUED_CMD_STATUS,
  QUEUED_CMD_AUX_ON,
  QUEUED_CMD_AUX_OFF,
  QUEUED_CMD_MANUAL_RPM,
  QUEUED_CMD_RESUME_SCHEDULE
};
QueuedCommandType queuedCommand = QUEUED_CMD_NONE;
int manualRequestedRPM = 1800;

// non-blocking start / ramp
enum StartSequenceState {
  START_IDLE,
  START_STATUS,
  START_AUX,
  START_STORE,
  START_SET_RPM,
  START_GO,
  START_COMPLETE,
  START_FAILED
};
StartSequenceState startSequenceState = START_IDLE;
bool startSequenceActive = false;
int startSequenceTargetRPM = 0;
unsigned long lastStartSequenceStepMillis = 0;

enum RampState {
  RAMP_IDLE,
  RAMP_ACTIVE
};
RampState rampState = RAMP_IDLE;
bool rampActive = false;
int rampTargetRPM = 0;
int rampCurrentRPM = 0;
unsigned long lastRampStepMillis = 0;

// =======================
// FORWARDS
// =======================
void servicePumpKeepAlive();
void servicePumpStopKeepAlive();
void processStartSequence();
void processRampSequence();
void processPrimeTransition();
void processActiveRunTimeout();
void processFreezeProtection();
void processSchedules();
void processPumpCommands();
void pollPumpStatusIfDue();
void refreshAllSensorMonitoring();
void refreshLiveTelemetry();
void updateAuxRelayControl();
void serviceWatchdogRecovery();
void syncClockFromNTPIfNeeded();
void serviceHomeWiFi();
void checkpointAllSettingsToEEPROM();
bool activeRPMUpdateAllowed();
void clearActiveRPMUpdate();
String jsonEscape(const String& in);

// =======================
// RTC HELPERS
// =======================
Month intToMonth(int m) {
  switch (m) {
    case 1: return Month::JANUARY;
    case 2: return Month::FEBRUARY;
    case 3: return Month::MARCH;
    case 4: return Month::APRIL;
    case 5: return Month::MAY;
    case 6: return Month::JUNE;
    case 7: return Month::JULY;
    case 8: return Month::AUGUST;
    case 9: return Month::SEPTEMBER;
    case 10: return Month::OCTOBER;
    case 11: return Month::NOVEMBER;
    default: return Month::DECEMBER;
  }
}

int monthToInt(Month m) {
  return Month2int(m);
}

int calcDayOfWeek(int year, int month, int day) {
  static int t[] = {0,3,2,5,0,3,5,1,4,6,2,4};
  if (month < 3) year -= 1;
  return (year + year/4 - year/100 + year/400 + t[month-1] + day) % 7;
}

DayOfWeek intToDayOfWeek(int dow) {
  switch (dow) {
    case 0: return DayOfWeek::SUNDAY;
    case 1: return DayOfWeek::MONDAY;
    case 2: return DayOfWeek::TUESDAY;
    case 3: return DayOfWeek::WEDNESDAY;
    case 4: return DayOfWeek::THURSDAY;
    case 5: return DayOfWeek::FRIDAY;
    default: return DayOfWeek::SATURDAY;
  }
}

String twoDigits(int v) {
  if (v < 10) return "0" + String(v);
  return String(v);
}

String rtcDateTimeString() {
  RTCTime now;
  RTC.getTime(now);

  String s = "";
  s += twoDigits(monthToInt(now.getMonth()));
  s += "/";
  s += twoDigits(now.getDayOfMonth());
  s += "/";
  s += String(now.getYear());
  s += " ";

  int hour24 = now.getHour();
  int hour12 = hour24 % 12;
  if (hour12 == 0) hour12 = 12;

  s += String(hour12);
  s += ":";
  s += twoDigits(now.getMinutes());
  s += ":";
  s += twoDigits(now.getSeconds());
  s += (hour24 >= 12) ? " PM" : " AM";
  return s;
}

void copyRTCToClockSettings() {
  RTCTime now;
  RTC.getTime(now);
  clockTime.month = monthToInt(now.getMonth());
  clockTime.day = now.getDayOfMonth();
  clockTime.year = now.getYear();
  int hour24 = now.getHour();
  int hour12 = hour24 % 12;
  if (hour12 == 0) hour12 = 12;
  clockTime.hour = hour12;
  clockTime.minute = now.getMinutes();
  strcpy(clockTime.ampm, (hour24 >= 12) ? "PM" : "AM");
}

void setRTCFromClockSettings() {
  int hour24 = clockTime.hour % 12;
  if (strcmp(clockTime.ampm, "PM") == 0) hour24 += 12;
  int dow = calcDayOfWeek(clockTime.year, clockTime.month, clockTime.day);
  RTCTime t(clockTime.day, intToMonth(clockTime.month), clockTime.year,
            hour24, clockTime.minute, 0,
            intToDayOfWeek(dow), SaveLight::SAVING_TIME_INACTIVE);
  RTC.setTime(t);
}

// =======================
// BASIC HELPERS
// =======================
bool isValidAMPM(const char* s) {
  return strcmp(s, "AM") == 0 || strcmp(s, "PM") == 0;
}

void setAMPM(char* dest, const String& src) {
  if (src == "PM") strcpy(dest, "PM");
  else strcpy(dest, "AM");
}

int16_t rawToSigned16(uint16_t raw) { return (int16_t)raw; }
float ambientTempC() { return ((float)lastAmbientTempRaw) / 128.0f; }
float ambientTempF() { return (ambientTempC() * 9.0f / 5.0f) + 32.0f; }
String formatFloat1(float v) { return String(v, 1); }

String formatDurationSeconds(unsigned long seconds) {
  unsigned long hours = seconds / 3600UL;
  unsigned long minutes = (seconds % 3600UL) / 60UL;
  unsigned long secs = seconds % 60UL;
  String s = "";
  if (hours > 0) { s += String(hours); s += "h "; }
  if (minutes > 0 || hours > 0) { s += String(minutes); s += "m "; }
  s += String(secs);
  s += "s";
  return s;
}

String jsonEscape(const String& in) {
  String out;
  out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '\\' || c == '"') {
      out += '\\'; out += c;
    } else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else out += c;
  }
  return out;
}

// =======================
// DEFAULTS / EEPROM
// =======================
void initDefaults() {
  clockTime.month = 4;
  clockTime.day = 21;
  clockTime.year = 2026;
  clockTime.hour = 12;
  clockTime.minute = 0;
  strcpy(clockTime.ampm, "PM");

  for (int i = 0; i < 3; i++) {
    sched[i].enabled = true;
    sched[i].speed = 1200;
    sched[i].startHour = 8 + i;
    sched[i].startMinute = 0;
    strcpy(sched[i].startAMPM, "AM");
    sched[i].stopHour = 10 + i;
    sched[i].stopMinute = 0;
    strcpy(sched[i].stopAMPM, "AM");
  }

  freezeEnabled = false;
  freezeTemp = 36;
  auxRPM = 2000;
  auxHours = 1.0;
  auxEnabled = true;
  primeRPM = 3000;
  primeMinutes = 3;
  overrideHighRPM = 3450;
  overrideHighHours = 1.0;
  overrideLowRPM = 1200;
  overrideLowHours = 1.0;
}

bool settingsLookValid() {
  if (clockTime.month < 1 || clockTime.month > 12) return false;
  if (clockTime.day < 1 || clockTime.day > 31) return false;
  if (clockTime.year < 2000 || clockTime.year > 2099) return false;
  if (clockTime.hour < 1 || clockTime.hour > 12) return false;
  if (clockTime.minute < 0 || clockTime.minute > 59) return false;
  if (!isValidAMPM(clockTime.ampm)) return false;

  for (int i = 0; i < 3; i++) {
    if (sched[i].speed < 600 || sched[i].speed > 3450) return false;
    if (sched[i].startHour < 1 || sched[i].startHour > 12) return false;
    if (sched[i].startMinute < 0 || sched[i].startMinute > 59) return false;
    if (!isValidAMPM(sched[i].startAMPM)) return false;
    if (sched[i].stopHour < 1 || sched[i].stopHour > 12) return false;
    if (sched[i].stopMinute < 0 || sched[i].stopMinute > 59) return false;
    if (!isValidAMPM(sched[i].stopAMPM)) return false;
  }

  if (freezeTemp < 32 || freezeTemp > 90) return false;
  if (auxRPM < 600 || auxRPM > 3450) return false;
  if (auxHours < 0.5 || auxHours > 24.0) return false;
  if (primeRPM < 1500 || primeRPM > 3450) return false;
  if (primeMinutes < 1 || primeMinutes > 10) return false;
  if (overrideHighRPM < 600 || overrideHighRPM > 3450) return false;
  if (overrideHighHours < 0.5 || overrideHighHours > 24.0) return false;
  if (overrideLowRPM < 600 || overrideLowRPM > 3450) return false;
  if (overrideLowHours < 0.5 || overrideLowHours > 24.0) return false;
  return true;
}

void saveSettings() {
  int addr = 0;
  EEPROM.put(addr, clockTime);         addr += sizeof(clockTime);
  for (int i = 0; i < 3; i++) { EEPROM.put(addr, sched[i]); addr += sizeof(Schedule); }
  EEPROM.put(addr, freezeEnabled);     addr += sizeof(freezeEnabled);
  EEPROM.put(addr, freezeTemp);        addr += sizeof(freezeTemp);
  EEPROM.put(addr, auxRPM);            addr += sizeof(auxRPM);
  EEPROM.put(addr, auxHours);          addr += sizeof(auxHours);
  EEPROM.put(addr, auxEnabled);        addr += sizeof(auxEnabled);
  EEPROM.put(addr, primeRPM);          addr += sizeof(primeRPM);
  EEPROM.put(addr, primeMinutes);      addr += sizeof(primeMinutes);
  EEPROM.put(addr, overrideHighRPM);   addr += sizeof(overrideHighRPM);
  EEPROM.put(addr, overrideHighHours); addr += sizeof(overrideHighHours);
  EEPROM.put(addr, overrideLowRPM);    addr += sizeof(overrideLowRPM);
  EEPROM.put(addr, overrideLowHours);  addr += sizeof(overrideLowHours);
}

void loadSettings() {
  int addr = 0;
  EEPROM.get(addr, clockTime);         addr += sizeof(clockTime);
  for (int i = 0; i < 3; i++) { EEPROM.get(addr, sched[i]); addr += sizeof(Schedule); }
  EEPROM.get(addr, freezeEnabled);     addr += sizeof(freezeEnabled);
  EEPROM.get(addr, freezeTemp);        addr += sizeof(freezeTemp);
  EEPROM.get(addr, auxRPM);            addr += sizeof(auxRPM);
  EEPROM.get(addr, auxHours);          addr += sizeof(auxHours);
  EEPROM.get(addr, auxEnabled);        addr += sizeof(auxEnabled);
  EEPROM.get(addr, primeRPM);          addr += sizeof(primeRPM);
  EEPROM.get(addr, primeMinutes);      addr += sizeof(primeMinutes);
  EEPROM.get(addr, overrideHighRPM);   addr += sizeof(overrideHighRPM);
  EEPROM.get(addr, overrideHighHours); addr += sizeof(overrideHighHours);
  EEPROM.get(addr, overrideLowRPM);    addr += sizeof(overrideLowRPM);
  EEPROM.get(addr, overrideLowHours);  addr += sizeof(overrideLowHours);

  if (!settingsLookValid()) {
    initDefaults();
    saveSettings();
  }
}

void checkpointAllSettingsToEEPROM() {
  copyRTCToClockSettings();
  saveSettings();
}

// =======================
// URL / PARAM HELPERS
// =======================
String urlDecode(const String& in) {
  String out = in;
  out.replace("+", " ");
  out.replace("%3A", ":"); out.replace("%3a", ":");
  out.replace("%2F", "/"); out.replace("%2f", "/");
  out.replace("%2E", "."); out.replace("%2e", ".");
  out.replace("%2D", "-"); out.replace("%2d", "-");
  out.replace("%5F", "_"); out.replace("%5f", "_");
  out.replace("%22", "\"");
  out.replace("%20", " ");
  return out;
}

String getParam(const String& req, const String& key) {
  String token = key + "=";
  int start = req.indexOf(token);
  if (start < 0) return "";
  start += token.length();
  int endAmp = req.indexOf('&', start);
  int endSpc = req.indexOf(' ', start);
  int end = -1;
  if (endAmp >= 0 && endSpc >= 0) end = min(endAmp, endSpc);
  else if (endAmp >= 0) end = endAmp;
  else if (endSpc >= 0) end = endSpc;
  else end = req.length();
  return urlDecode(req.substring(start, end));
}

bool getBoolParam(const String& req, const String& key) {
  String v = getParam(req, key);
  return (v == "1" || v == "true" || v == "on" || v == "yes");
}

int clampInt(int value, int minVal, int maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

float clampFloat(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

// =======================
// DEBUG / PROTOCOL
// =======================
void printBuffer(const char* label, const uint8_t* buf, size_t len) {
  Serial.print(label); Serial.print(": ");
  for (size_t i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print("0");
    Serial.print(buf[i], HEX); Serial.print(" ");
  }
  Serial.println();
}

uint16_t crc16Modbus(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)data[pos];
    for (int i = 0; i < 8; i++) {
      if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
      else crc >>= 1;
    }
  }
  return crc;
}

void rs485SetTransmit(bool enable) {
  digitalWrite(RS485_DE_PIN, enable ? HIGH : LOW);
}

void rs485FlushInput() {
  while (Serial1.available()) Serial1.read();
}

bool epcSendFrame(const uint8_t *payload, size_t payloadLen) {
  delayMicroseconds(EPC_IDLE_US);
  rs485FlushInput();
  printBuffer("TX", payload, payloadLen);
  rs485SetTransmit(true);
  for (size_t i = 0; i < payloadLen; i++) Serial1.write(payload[i]);
  Serial1.flush();
  delayMicroseconds(EPC_POST_TX_DELAY_US);
  rs485SetTransmit(false);
  return true;
}

bool epcReadFrame(uint8_t *buf, size_t expectedLen, unsigned long timeoutMs) {
  unsigned long start = millis();
  size_t idx = 0;
  while ((millis() - start) < timeoutMs) {
    while (Serial1.available()) {
      if (idx < expectedLen) buf[idx++] = (uint8_t)Serial1.read();
      else Serial1.read();
    }
    if (idx >= expectedLen) break;
  }
  if (idx != expectedLen) {
    Serial.print("RX: TIMEOUT, bytes received = ");
    Serial.println(idx);
    return false;
  }
  printBuffer("RX", buf, expectedLen);
  return true;
}

bool epcValidateReply(const uint8_t *buf, size_t len) {
  if (len < 5) return false;
  uint16_t crcCalc = crc16Modbus(buf, len - 2);
  uint16_t crcRecv = (uint16_t)buf[len - 2] | ((uint16_t)buf[len - 1] << 8);
  return crcCalc == crcRecv;
}

String decodePumpStatus(uint8_t status) {
  switch (status) {
    case 0x00: return "Stopped";
    case 0x09: return "Booting";
    case 0x0A: return "Accelerating";
    case 0x0B: return "Running";
    case 0x20: return "Fault";
    default: return "Unknown";
  }
}

String decodeStatusHex(uint8_t status) {
  String s = "0x";
  if (status < 0x10) s += "0";
  s += String(status, HEX);
  s.toUpperCase();
  return s;
}

String decodeNack(uint8_t nack) {
  switch (nack) {
    case 0x01: return "Illegal command";
    case 0x02: return "Operand out of range";
    case 0x03: return "Data out of range";
    case 0x04: return "General failure / fault mode";
    case 0x05: return "Incorrect command length";
    case 0x06: return "Command cannot execute now";
    case 0x09: return "Buffer error";
    case 0x0A: return "Running parameters incomplete";
    default:   return "Unknown NACK 0x" + String(nack, HEX);
  }
}

String faultCodeToHex(uint8_t code) {
  String s = "0x";
  if (code < 0x10) s += "0";
  s += String(code, HEX);
  s.toUpperCase();
  return s;
}

String decodeFaultCode(uint8_t code) {
  switch (code) {
    case 0x00: return "None";
    case 0x21: return "Software Overcurrent";
    case 0x22: return "DC Overvoltage";
    case 0x23: return "DC Undervoltage";
    case 0x26: return "Hardware Overcurrent";
    case 0x2A: return "Startup Failure";
    case 0x2D: return "Processor - Fatal";
    case 0x2E: return "IGBT Over Temperature";
    case 0x2F: return "Loss of Phase";
    case 0x30: return "Low Power";
    case 0x31: return "Processor - Registers";
    case 0x32: return "Processor - Program Counter";
    case 0x33: return "Processor - Interrupt/Execution";
    case 0x34: return "Processor - Clock";
    case 0x35: return "Processor - Flash Memory";
    case 0x36: return "RAS Fault";
    case 0x37: return "Processor - ADC";
    case 0x3C: return "Keypad Fault";
    case 0x3D: return "LVB Data Flash Fault";
    case 0x3E: return "Comm Loss Fault - LVB & Drive";
    case 0x3F: return "Generic Fault";
    case 0x40: return "Coherence Fault";
    case 0x41: return "UL Fault";
    case 0x42: return "SVRS Fault Type 1";
    case 0x43: return "SVRS Fault Type 2";
    case 0x44: return "SVRS Fault Type 13";
    default: return "Unknown";
  }
}

void updateRunSessionTimer() {
  bool pumpActiveNow = (lastPumpStatus == 0x09 || lastPumpStatus == 0x0A || lastPumpStatus == 0x0B);
  bool pumpStoppedNow = (lastPumpStatus == 0x00);
  bool pumpFaultNow = (lastPumpStatus == 0x20);

  if (!runSessionActive && pumpActiveNow) {
    runSessionActive = true;
    runSessionStartMillis = millis();
    lastStopReason = "Running";
  }
  if (runSessionActive && (pumpStoppedNow || pumpFaultNow)) {
    lastRunSessionDurationMs = millis() - runSessionStartMillis;
    runSessionActive = false;
    lastStopReason = pumpFaultNow ? "Fault Stop" : "Normal Stop";
  }
}

unsigned long getRunSessionSeconds() {
  if (runSessionActive) return (millis() - runSessionStartMillis) / 1000UL;
  return lastRunSessionDurationMs / 1000UL;
}

void updateCachedStatus(uint8_t status) {
  lastPumpStatus = status;
  lastPumpStatusText = decodePumpStatus(status);
  lastPumpStatusHex = decodeStatusHex(status);

  if (status == 0x20) {
    pumpRunState = RUNSTATE_FAULT;
    currentCommandedRPM = 0;
  } else if (status == 0x00 && pumpRunState != RUNSTATE_PRIMING) {
    pumpRunState = RUNSTATE_IDLE;
  } else if ((status == 0x09 || status == 0x0A || status == 0x0B) && pumpRunState != RUNSTATE_PRIMING) {
    pumpRunState = RUNSTATE_RUNNING;
  }
  updateRunSessionTimer();
}

// =======================
// EPC COMMANDS
// =======================
bool epcSendSimpleCommand(uint8_t functionCode, String &msg) {
  uint8_t tx[5];
  tx[0] = PUMP_ADDR; tx[1] = functionCode; tx[2] = EPC_ACK_CMD;
  uint16_t crc = crc16Modbus(tx, 3);
  tx[3] = (uint8_t)(crc & 0xFF);
  tx[4] = (uint8_t)((crc >> 8) & 0xFF);

  if (functionCode == EPC_FN_GO) Serial.println("CMD: GO");
  if (functionCode == EPC_FN_STOP) Serial.println("CMD: STOP");

  epcSendFrame(tx, sizeof(tx));
  uint8_t rx[5];
  if (!epcReadFrame(rx, sizeof(rx), EPC_REPLY_TIMEOUT_MS)) { msg = "Timeout waiting for reply"; return false; }
  if (!epcValidateReply(rx, sizeof(rx))) { msg = "CRC error on reply"; return false; }
  if (rx[0] != PUMP_ADDR) { msg = "Wrong reply address"; return false; }
  if (rx[1] == (functionCode | 0x80)) { msg = decodeNack(rx[2]); return false; }
  if (rx[1] != functionCode) { msg = "Unexpected function in reply"; return false; }
  if (rx[2] != EPC_ACK_REPLY) { msg = "Unexpected ACK byte"; return false; }
  msg = "OK";
  return true;
}

bool epcStoreConfig(String &msg) {
  uint8_t tx[5];
  tx[0] = PUMP_ADDR; tx[1] = EPC_FN_STORECFG; tx[2] = EPC_ACK_CMD;
  uint16_t crc = crc16Modbus(tx, 3);
  tx[3] = (uint8_t)(crc & 0xFF);
  tx[4] = (uint8_t)((crc >> 8) & 0xFF);
  Serial.println("CMD: STORE CONFIG");
  epcSendFrame(tx, sizeof(tx));
  uint8_t rx[5];
  if (!epcReadFrame(rx, sizeof(rx), EPC_REPLY_TIMEOUT_MS)) { msg = "Timeout waiting for STORE reply"; return false; }
  if (!epcValidateReply(rx, sizeof(rx))) { msg = "CRC error on STORE reply"; return false; }
  if (rx[0] != PUMP_ADDR) { msg = "Wrong STORE reply address"; return false; }
  if (rx[1] == (EPC_FN_STORECFG | 0x80)) { msg = decodeNack(rx[2]); return false; }
  if (rx[1] != EPC_FN_STORECFG || rx[2] != EPC_ACK_REPLY) { msg = "Unexpected STORE reply"; return false; }
  msg = "STORE OK";
  return true;
}

bool epcSetDemandRPM(int rpm, String &msg) {
  if (rpm < 600 || rpm > 3450) { msg = "RPM out of allowed range"; return false; }
  uint16_t demand = (uint16_t)(rpm * 4);
  Serial.print("CMD: SET RPM "); Serial.println(rpm);

  uint8_t tx[8];
  tx[0] = PUMP_ADDR; tx[1] = EPC_FN_SETDEMAND; tx[2] = EPC_ACK_CMD; tx[3] = EPC_MODE_SPEED;
  tx[4] = (uint8_t)(demand & 0xFF); tx[5] = (uint8_t)((demand >> 8) & 0xFF);
  uint16_t crc = crc16Modbus(tx, 6);
  tx[6] = (uint8_t)(crc & 0xFF); tx[7] = (uint8_t)((crc >> 8) & 0xFF);

  epcSendFrame(tx, sizeof(tx));
  uint8_t rx[8];
  if (!epcReadFrame(rx, sizeof(rx), EPC_REPLY_TIMEOUT_MS)) { msg = "Timeout waiting for Set Demand reply"; return false; }
  if (!epcValidateReply(rx, sizeof(rx))) { msg = "CRC error on Set Demand reply"; return false; }
  if (rx[0] != PUMP_ADDR) { msg = "Wrong reply address"; return false; }
  if (rx[1] == (EPC_FN_SETDEMAND | 0x80)) { msg = decodeNack(rx[2]); return false; }
  if (rx[1] != EPC_FN_SETDEMAND || rx[2] != EPC_ACK_REPLY) { msg = "Unexpected Set Demand reply"; return false; }
  if (rx[3] != EPC_MODE_SPEED) { msg = "Unexpected mode echoed"; return false; }
  uint16_t echoedDemand = (uint16_t)rx[4] | ((uint16_t)rx[5] << 8);
  if (echoedDemand != demand) { msg = "Demand echo mismatch"; return false; }
  msg = "Set Demand OK";
  return true;
}

bool epcGo(String &msg) { return epcSendSimpleCommand(EPC_FN_GO, msg); }
bool epcStop(String &msg) { return epcSendSimpleCommand(EPC_FN_STOP, msg); }

bool epcReadStatus(uint8_t &statusOut, String &msg) {
  Serial.println("CMD: STATUS");
  uint8_t tx[5];
  tx[0] = PUMP_ADDR; tx[1] = EPC_FN_STATUS; tx[2] = EPC_ACK_CMD;
  uint16_t crc = crc16Modbus(tx, 3);
  tx[3] = (uint8_t)(crc & 0xFF); tx[4] = (uint8_t)((crc >> 8) & 0xFF);
  epcSendFrame(tx, sizeof(tx));
  uint8_t rx[6];
  if (!epcReadFrame(rx, sizeof(rx), EPC_REPLY_TIMEOUT_MS)) { msg = "Timeout waiting for Status reply"; return false; }
  if (!epcValidateReply(rx, sizeof(rx))) { msg = "CRC error on Status reply"; return false; }
  if (rx[0] != PUMP_ADDR) { msg = "Wrong reply address"; return false; }
  if (rx[1] == (EPC_FN_STATUS | 0x80)) { msg = decodeNack(rx[2]); return false; }
  if (rx[1] != EPC_FN_STATUS || rx[2] != EPC_ACK_REPLY) { msg = "Unexpected Status reply"; return false; }
  statusOut = rx[3];
  msg = "Status OK";
  return true;
}

bool epcReadSensor(uint8_t page, uint8_t address, uint16_t &valueOut, String &msg) {
  uint8_t tx[7];
  tx[0] = PUMP_ADDR; tx[1] = EPC_FN_READSENSOR; tx[2] = EPC_ACK_CMD; tx[3] = page; tx[4] = address;
  uint16_t crc = crc16Modbus(tx, 5);
  tx[5] = (uint8_t)(crc & 0xFF); tx[6] = (uint8_t)((crc >> 8) & 0xFF);
  epcSendFrame(tx, sizeof(tx));
  uint8_t rx[9];
  if (!epcReadFrame(rx, sizeof(rx), EPC_REPLY_TIMEOUT_MS)) { msg = "Timeout waiting for Read Sensor reply"; return false; }
  if (!epcValidateReply(rx, sizeof(rx))) { msg = "CRC error on Read Sensor reply"; return false; }
  if (rx[0] != PUMP_ADDR) { msg = "Wrong reply address"; return false; }
  if (rx[1] == (EPC_FN_READSENSOR | 0x80)) { msg = decodeNack(rx[2]); return false; }
  if (rx[1] != EPC_FN_READSENSOR || rx[2] != EPC_ACK_REPLY) { msg = "Unexpected Read Sensor reply"; return false; }
  if (rx[3] != page || rx[4] != address) { msg = "Sensor page/address mismatch"; return false; }
  valueOut = (uint16_t)rx[5] | ((uint16_t)rx[6] << 8);
  msg = "Read Sensor OK";
  return true;
}

bool epcWriteAuxRelay(bool turnOn, String &msg) {
  uint8_t value = turnOn ? EPC_AUX_ON_VALUE : EPC_AUX_OFF_VALUE;
  uint8_t tx[9];
  tx[0] = PUMP_ADDR; tx[1] = EPC_FN_CONFIG_RW; tx[2] = EPC_ACK_CMD;
  tx[3] = EPC_CONFIG_PAGE1_WRITE; tx[4] = EPC_CONFIG_ADDR_AUX_RELAY; tx[5] = 0x00; tx[6] = value;
  uint16_t crc = crc16Modbus(tx, 7);
  tx[7] = (uint8_t)(crc & 0xFF); tx[8] = (uint8_t)((crc >> 8) & 0xFF);
  Serial.print("CMD: AUX "); Serial.println(turnOn ? "ON" : "OFF");
  epcSendFrame(tx, sizeof(tx));
  uint8_t rx[9];
  if (!epcReadFrame(rx, sizeof(rx), EPC_REPLY_TIMEOUT_MS)) { msg = "Timeout waiting for AUX reply"; return false; }
  if (!epcValidateReply(rx, sizeof(rx))) { msg = "CRC error on AUX reply"; return false; }
  if (rx[0] != PUMP_ADDR) { msg = "Wrong AUX reply address"; return false; }
  if (rx[1] == (EPC_FN_CONFIG_RW | 0x80)) { msg = decodeNack(rx[2]); return false; }
  if (rx[1] != EPC_FN_CONFIG_RW || rx[2] != EPC_ACK_REPLY) { msg = "Unexpected AUX reply"; return false; }
  msg = turnOn ? "Aux ON accepted" : "Aux OFF accepted";
  return true;
}

// =======================
// SENSOR / TELEMETRY
// =======================
bool refreshAmbientTemp(String &msg) {
  uint16_t value = 0;
  if (!epcReadSensor(EPC_SENSOR_PAGE_AMBIENT_TEMP, EPC_SENSOR_ADDR_AMBIENT_TEMP, value, msg)) return false;
  lastAmbientTempRaw = rawToSigned16(value);
  ambientTempValid = true;
  return true;
}

bool refreshPrevFault(String &msg) {
  uint16_t value = 0;
  if (!epcReadSensor(EPC_SENSOR_PAGE_PREV_FAULT, EPC_SENSOR_ADDR_PREV_FAULT, value, msg)) return false;
  lastPrevFaultCode = (uint8_t)(value & 0xFF);
  lastPrevFaultCodeHex = faultCodeToHex(lastPrevFaultCode);
  lastPrevFaultText = decodeFaultCode(lastPrevFaultCode);
  return true;
}

bool refreshCurrentFaults(String &msg) {
  uint16_t value = 0;
  switch (sensorMonitorStep) {
    case 2:
      if (!epcReadSensor(EPC_SENSOR_PAGE_FAULTS, EPC_SENSOR_ADDR_FAULT1, value, msg)) return false;
      lastFault1Code = (uint8_t)(value & 0xFF); lastFault1CodeHex = faultCodeToHex(lastFault1Code); lastFault1Text = decodeFaultCode(lastFault1Code); break;
    case 3:
      if (!epcReadSensor(EPC_SENSOR_PAGE_FAULTS, EPC_SENSOR_ADDR_FAULT2, value, msg)) return false;
      lastFault2Code = (uint8_t)(value & 0xFF); lastFault2CodeHex = faultCodeToHex(lastFault2Code); lastFault2Text = decodeFaultCode(lastFault2Code); break;
    case 4:
      if (!epcReadSensor(EPC_SENSOR_PAGE_FAULTS, EPC_SENSOR_ADDR_FAULT3, value, msg)) return false;
      lastFault3Code = (uint8_t)(value & 0xFF); lastFault3CodeHex = faultCodeToHex(lastFault3Code); lastFault3Text = decodeFaultCode(lastFault3Code); break;
    case 5:
      if (!epcReadSensor(EPC_SENSOR_PAGE_FAULTS, EPC_SENSOR_ADDR_FAULT4, value, msg)) return false;
      lastFault4Code = (uint8_t)(value & 0xFF); lastFault4CodeHex = faultCodeToHex(lastFault4Code); lastFault4Text = decodeFaultCode(lastFault4Code); break;
    default:
      break;
  }
  return true;
}

void refreshAllSensorMonitoring() {
  String msg;
  switch (sensorMonitorStep) {
    case 0: refreshAmbientTemp(msg); break;
    case 1: refreshPrevFault(msg); break;
    case 2:
    case 3:
    case 4:
    case 5: refreshCurrentFaults(msg); break;
  }
  sensorMonitorStep++;
  if (sensorMonitorStep > 5) sensorMonitorStep = 0;
}

void refreshLiveTelemetry() {
  if (millis() - lastTelemetryPollMillis < TELEMETRY_POLL_INTERVAL_MS) return;
  lastTelemetryPollMillis = millis();

  static bool readRPMNext = true;
  String msg;
  uint16_t value = 0;

  if (readRPMNext) {
    if (epcReadSensor(EPC_SENSOR_PAGE_RPM, EPC_SENSOR_ADDR_RPM, value, msg)) {
      liveRPMRaw = value;
      livePumpRPM = (int)(liveRPMRaw / 4);
    }
  } else {
    if (epcReadSensor(EPC_SENSOR_PAGE_S05, EPC_SENSOR_ADDR_S05, value, msg)) {
      liveS05Raw = value;
      livePumpWatts = (int)liveS05Raw;
    }
  }
  readRPMNext = !readRPMNext;
}

void updateAuxRelayControl() {
  bool pumpRunningLike = (lastPumpStatus == 0x09 || lastPumpStatus == 0x0A || lastPumpStatus == 0x0B);
  if (!pumpRunningLike) { auxRelayForcedOff = false; return; }
  if (auxEnabled) { auxRelayForcedOff = false; return; }

  bool needWrite = (!auxRelayForcedOff) || (millis() - lastAuxRetryMillis >= AUX_RETRY_INTERVAL_MS);
  if (!needWrite) return;

  String msg;
  if (epcWriteAuxRelay(false, msg)) {
    auxRelayForcedOff = true;
    lastAuxRetryMillis = millis();
  } else {
    lastAuxRetryMillis = millis();
    lastPumpReplyText = "Aux OFF failed: " + msg;
  }
}

// =======================
// KEEPALIVE / URGENCY
// =======================
bool shouldAssertRunCommand() {
  return (currentCommandedRPM >= 600 || pendingRunRequest || pumpRunState == RUNSTATE_PRIMING || pumpRunState == RUNSTATE_RUNNING);
}

void servicePumpKeepAlive() {
  if (millis() - lastKeepAliveMillis < KEEPALIVE_INTERVAL_MS) return;
  if (currentCommandedRPM < 600) return;

  String msg;
  if (!epcSetDemandRPM(currentCommandedRPM, msg)) {
    lastPumpReplyText = "Keepalive Set Demand failed: " + msg;
    lastKeepAliveMillis = millis();
    return;
  }
  if (!epcGo(msg)) {
    lastPumpReplyText = "Keepalive GO failed: " + msg;
    lastKeepAliveMillis = millis();
    return;
  }
  lastKeepAliveMillis = millis();
}

void servicePumpStopKeepAlive() {
  if (shouldAssertRunCommand()) return;
  if (millis() - lastStopKeepAliveMillis < STOP_KEEPALIVE_INTERVAL_MS) return;
  String msg;
  if (!epcStop(msg)) lastPumpReplyText = "Stop keepalive failed: " + msg;
  lastStopKeepAliveMillis = millis();
}

bool keepAliveUrgent() {
  return shouldAssertRunCommand() && ((millis() - lastKeepAliveMillis) >= (KEEPALIVE_INTERVAL_MS - 200));
}

bool stopKeepAliveUrgent() {
  return (!shouldAssertRunCommand()) && ((millis() - lastStopKeepAliveMillis) >= (STOP_KEEPALIVE_INTERVAL_MS - 200));
}

bool isSystemBusyWithMotorSequence() { return startSequenceActive || rampActive; }

bool controllerBusyForNewCommand() {
  return startSequenceActive || rampActive || pendingRunRequest || pumpRunState == RUNSTATE_PRIMING || queuedCommand != QUEUED_CMD_NONE;
}

bool queueOneCommand(QueuedCommandType cmd) {
  if (cmd == QUEUED_CMD_RESUME_SCHEDULE) {
    // Resume is lightweight: it only clears schedule hold-off and lets the schedule engine restart if still in-window.
    queuedCommand = QUEUED_CMD_RESUME_SCHEDULE;
    return true;
  }

  if (cmd == QUEUED_CMD_STOP) {
    // Emergency-style STOP: always allow it to preempt queued/start/ramp activity.
    queuedCommand = QUEUED_CMD_STOP;
    clearMotorSequences();
    pendingRunRequest = false;
    requestedTargetRPM = 0;
    currentCommandedRPM = 0;
    return true;
  }

  if (controllerBusyForNewCommand()) {
    lastPumpReplyText = "Controller busy - wait for current command to finish";
    return false;
  }
  queuedCommand = cmd;
  return true;
}

void clearMotorSequences() {
  startSequenceActive = false;
  startSequenceState = START_IDLE;
  rampActive = false;
  rampState = RAMP_IDLE;
}

// =======================
// RUN / START / RAMP
// =======================
bool refreshPumpStatus(String &msg) {
  uint8_t status;
  if (!epcReadStatus(status, msg)) return false;
  updateCachedStatus(status);
  lastGoodStatusMillis = millis();
  return true;
}

bool queueRampToRPM(int targetRPM) {
  if (targetRPM < 600 || targetRPM > 3450) {
    lastPumpReplyText = "RPM out of allowed range";
    return false;
  }

  if (rampActive) {
    rampTargetRPM = targetRPM;
    return true;
  }

  rampCurrentRPM = currentCommandedRPM;
  if (rampCurrentRPM <= 0) rampCurrentRPM = targetRPM;
  rampTargetRPM = targetRPM;

  if (rampCurrentRPM == rampTargetRPM) {
    currentCommandedRPM = rampTargetRPM;
    requestedTargetRPM = rampTargetRPM;
    return true;
  }

  rampState = RAMP_ACTIVE;
  rampActive = true;
  lastRampStepMillis = 0;
  return true;
}

bool queueStartSequence(int targetRPM) {
  if (startSequenceActive || rampActive) {
    lastPumpReplyText = "Motor sequence busy";
    return false;
  }
  startSequenceTargetRPM = targetRPM;
  startSequenceState = START_STATUS;
  startSequenceActive = true;
  lastStartSequenceStepMillis = 0;
  return true;
}

void processRampSequence() {
  if (!rampActive) return;
  unsigned long interval = (rampCurrentRPM < rampTargetRPM) ? RAMP_UP_DELAY_MS : RAMP_DOWN_DELAY_MS;
  if (millis() - lastRampStepMillis < interval) return;
  lastRampStepMillis = millis();

  int stepRPM = (rampTargetRPM > rampCurrentRPM) ? RAMP_UP_STEP_RPM : -RAMP_DOWN_STEP_RPM;
  int nextRPM = rampCurrentRPM + stepRPM;
  if (stepRPM > 0 && nextRPM > rampTargetRPM) nextRPM = rampTargetRPM;
  if (stepRPM < 0 && nextRPM < rampTargetRPM) nextRPM = rampTargetRPM;

  String msg;
  if (!epcSetDemandRPM(nextRPM, msg)) {
    lastPumpReplyText = "Ramp step failed: " + msg;
    rampActive = false;
    rampState = RAMP_IDLE;
    return;
  }

  rampCurrentRPM = nextRPM;
  currentCommandedRPM = nextRPM;
  requestedTargetRPM = rampTargetRPM;
  lastKeepAliveMillis = millis();

  if (rampCurrentRPM == rampTargetRPM) {
    rampActive = false;
    rampState = RAMP_IDLE;
    pumpRunState = RUNSTATE_RUNNING;
    pendingRunRequest = false;
    lastPumpReplyText = "Running at " + String(rampTargetRPM) + " RPM";
  }
}

void processStartSequence() {
  if (!startSequenceActive) return;
  if (millis() - lastStartSequenceStepMillis < START_SEQUENCE_STEP_INTERVAL_MS) return;
  lastStartSequenceStepMillis = millis();

  String msg;
  uint8_t status = 0;

  switch (startSequenceState) {
    case START_STATUS:
      if (!epcReadStatus(status, msg)) { lastPumpReplyText = "Start failed: status " + msg; startSequenceState = START_FAILED; break; }
      updateCachedStatus(status);
      lastGoodStatusMillis = millis();
      if (status == 0x20) { lastPumpReplyText = "Run blocked: motor fault"; startSequenceState = START_FAILED; break; }
      if (status == 0x09 || status == 0x0A || status == 0x0B) {
        if (!queueRampToRPM(startSequenceTargetRPM)) { startSequenceState = START_FAILED; break; }
        startSequenceState = START_COMPLETE;
      } else {
        startSequenceState = START_AUX;
      }
      break;

    case START_AUX:
      if (!epcWriteAuxRelay(auxEnabled, msg)) { lastPumpReplyText = "Start failed: AUX " + msg; startSequenceState = START_FAILED; break; }
      startSequenceState = START_STORE;
      break;

    case START_STORE:
      if (!epcStoreConfig(msg)) { lastPumpReplyText = "Start failed: STORE " + msg; startSequenceState = START_FAILED; break; }
      startSequenceState = START_SET_RPM;
      break;

    case START_SET_RPM:
      if (!epcSetDemandRPM(primeRPM, msg)) { lastPumpReplyText = "Start failed: prime demand " + msg; startSequenceState = START_FAILED; break; }
      currentCommandedRPM = primeRPM;
      requestedTargetRPM = startSequenceTargetRPM;
      startSequenceState = START_GO;
      break;

    case START_GO:
      if (!epcGo(msg)) { lastPumpReplyText = "Start failed: GO " + msg; startSequenceState = START_FAILED; break; }
      pendingRunRequest = true;
      primeStartMillis = millis();
      pumpRunState = RUNSTATE_PRIMING;
      lastKeepAliveMillis = millis();
      lastPumpReplyText = "Priming at " + String(primeRPM) + " RPM";
      startSequenceState = START_COMPLETE;
      break;

    case START_COMPLETE:
      startSequenceActive = false;
      startSequenceState = START_IDLE;
      break;

    case START_FAILED:
      clearMotorSequences();
      break;

    default:
      break;
  }
}

bool startOrChangeToRPM(int targetRPM) {
  String msg;
  uint8_t status;
  if (!epcReadStatus(status, msg)) { lastPumpReplyText = "Status failed: " + msg; return false; }
  updateCachedStatus(status);
  lastGoodStatusMillis = millis();

  if (status == 0x20) {
    pumpRunState = RUNSTATE_FAULT;
    currentCommandedRPM = 0;
    lastPumpReplyText = "Run blocked: motor fault";
    return false;
  }

  if (pumpRunState == RUNSTATE_PRIMING && pendingRunRequest) {
    lastPumpReplyText = "Prime in progress - wait for prime to finish";
    return false;
  }

  if (status == 0x00) return queueStartSequence(targetRPM);
  return queueRampToRPM(targetRPM);
}

void processPrimeTransition() {
  if (pumpRunState != RUNSTATE_PRIMING) return;
  if (!pendingRunRequest) return;
  unsigned long primeDurationMs = (unsigned long)primeMinutes * 60000UL;
  unsigned long elapsed = millis() - primeStartMillis;
  if (elapsed < primeDurationMs) return;
  if (!queueRampToRPM(requestedTargetRPM)) return;
  pendingRunRequest = false;
  lastKeepAliveMillis = millis();
  if (freezeProtectionActive && requestedTargetRPM == FREEZE_RUN_RPM) lastPumpReplyText = "Prime complete - freeze ramp queued";
  else lastPumpReplyText = "Prime complete - ramping to " + String(requestedTargetRPM) + " RPM";
}

void stopFreezeMode() {
  String msg;
  if (epcStop(msg)) {
    freezeProtectionActive = false;
    freezeBelowStartMillis = 0;
    freezeAboveStartMillis = 0;
    if (activeRunMode == ACTIVE_FREEZE) activeRunMode = ACTIVE_NONE;
    scheduleOwnsPump = false;
    pendingRunRequest = false;
    requestedTargetRPM = 0;
    currentCommandedRPM = 0;
    pumpRunState = RUNSTATE_IDLE;
    auxRelayForcedOff = false;
    lastKeepAliveMillis = 0;
    lastPumpReplyText = "Freeze protection cleared - pump stopped";
    uint8_t status;
    if (epcReadStatus(status, msg)) { updateCachedStatus(status); lastGoodStatusMillis = millis(); }
  } else {
    lastPumpReplyText = "Freeze stop failed: " + msg;
  }
}

void processActiveRunTimeout() {
  if (!(activeRunMode == ACTIVE_OVERRIDE_HIGH || activeRunMode == ACTIVE_OVERRIDE_LOW)) return;
  if (activeRunDurationMs == 0) return;
  unsigned long elapsed = millis() - activeRunStartMillis;
  if (elapsed < activeRunDurationMs) return;

  if (freezeProtectionActive) {
    activeRunMode = ACTIVE_FREEZE;
    activeRunStartMillis = 0;
    scheduleOwnsPump = false;
    activeRunDurationMs = 0;
    startOrChangeToRPM(FREEZE_RUN_RPM);
    lastPumpReplyText = "Override complete - returning to freeze protection";
    return;
  }

  String msg;
  if (epcStop(msg)) {
    activeRunMode = ACTIVE_NONE;
    activeRunStartMillis = 0;
    activeRunDurationMs = 0;
    requestedTargetRPM = 0;
    currentCommandedRPM = 0;
    pendingRunRequest = false;
    pumpRunState = RUNSTATE_IDLE;
    auxRelayForcedOff = false;
    lastKeepAliveMillis = 0;
    lastPumpReplyText = "Override run complete - pump stopped";
  } else {
    lastPumpReplyText = "Override stop failed: " + msg;
  }
}

void processFreezeProtection() {
  if (!ambientTempValid) return;

  if (!freezeEnabled) {
    freezeBelowStartMillis = 0;
    freezeAboveStartMillis = 0;
    if (freezeProtectionActive) stopFreezeMode();
    return;
  }

  float ambientF = ambientTempF();
  unsigned long now = millis();

  if (!freezeProtectionActive) {
    freezeAboveStartMillis = 0;
    if (ambientF < (float)freezeTemp) {
      if (freezeBelowStartMillis == 0) freezeBelowStartMillis = now;
      else if ((now - freezeBelowStartMillis) >= FREEZE_CONFIRM_MS) {
        if (startOrChangeToRPM(FREEZE_RUN_RPM)) {
          freezeProtectionActive = true;
          activeRunMode = ACTIVE_FREEZE;
          activeRunStartMillis = 0;
          activeRunDurationMs = 0;
          freezeBelowStartMillis = 0;
          freezeAboveStartMillis = 0;
          lastPumpReplyText = "Freeze protection active";
        }
      }
    } else {
      freezeBelowStartMillis = 0;
    }
    return;
  }

  freezeBelowStartMillis = 0;

  if (activeRunMode == ACTIVE_FREEZE && pumpRunState == RUNSTATE_RUNNING && requestedTargetRPM != FREEZE_RUN_RPM) {
    startOrChangeToRPM(FREEZE_RUN_RPM);
  }

  if (ambientF > (float)freezeTemp) {
    if (freezeAboveStartMillis == 0) freezeAboveStartMillis = now;
    else if ((now - freezeAboveStartMillis) >= FREEZE_CONFIRM_MS) stopFreezeMode();
  } else {
    freezeAboveStartMillis = 0;
  }
}

// =======================
// SCHEDULES
// =======================
int to24Hour(int hour12, int minute, const char* ampm) {
  int h = hour12 % 12;
  if (strcmp(ampm, "PM") == 0) h += 12;
  return h * 60 + minute;
}

int currentMinutesOfDay() {
  RTCTime now;
  RTC.getTime(now);
  return now.getHour() * 60 + now.getMinutes();
}

String scheduleTimeText(int hour, int minute, const char* ampm) {
  String s = String(hour);
  s += ":";
  s += twoDigits(minute);
  s += " ";
  s += ampm;
  return s;
}

String scheduleStatusText(int index) {
  if (index < 0 || index >= 3) return "Invalid";
  if (!sched[index].enabled) return "Disabled";
  if (activeScheduleIndex == index) return "Active";
  return "Armed";
}

void clearActiveRPMUpdate() {
  activeRPMUpdateEnabled = false;
  activeRPMUpdateValue = 0;
  activeRPMUpdateScheduleIndex = -1;
}

bool isPrimeLocked() {
  return (pumpRunState == RUNSTATE_PRIMING && pendingRunRequest);
}

bool pumpIsCurrentlyRunningLike() {
  return (pumpRunState == RUNSTATE_PRIMING || pumpRunState == RUNSTATE_RUNNING ||
          lastPumpStatus == 0x09 || lastPumpStatus == 0x0A || lastPumpStatus == 0x0B ||
          currentCommandedRPM > 0 || pendingRunRequest);
}

void stopPumpForNoDemand() {
  clearActiveRPMUpdate();
  autoRunInhibited = false;

  if (!pumpIsCurrentlyRunningLike()) {
    activeScheduleIndex = -1;
    scheduleRequestedRPM = 0;
    scheduleOwnsPump = false;
    pendingRunRequest = false;
    requestedTargetRPM = 0;
    currentCommandedRPM = 0;
    pumpRunState = RUNSTATE_IDLE;
    return;
  }

  String msg;
  if (epcStop(msg)) {
    activeScheduleIndex = -1;
    scheduleRequestedRPM = 0;
    scheduleOwnsPump = false;
    pendingRunRequest = false;
    requestedTargetRPM = 0;
    currentCommandedRPM = 0;
    pumpRunState = RUNSTATE_IDLE;
    auxRelayForcedOff = false;
    lastKeepAliveMillis = 0;
    lastPumpReplyText = "No active schedule or override - pump stopped";
  } else {
    lastPumpReplyText = "Auto stop failed: " + msg;
  }
}

bool activeRPMUpdateAllowed() {
  return (activeScheduleIndex >= 0) || (activeRunMode == ACTIVE_OVERRIDE_HIGH) || (activeRunMode == ACTIVE_OVERRIDE_LOW);
}

int getEffectiveScheduleTargetRPM(int scheduleIndex) {
  if (scheduleIndex < 0 || scheduleIndex >= 3) return 0;
  if (activeRPMUpdateEnabled && activeRPMUpdateScheduleIndex == scheduleIndex) return activeRPMUpdateValue;
  return sched[scheduleIndex].speed;
}

bool isScheduleActiveNow(const Schedule& s) {
  if (!s.enabled) return false;
  int nowMin = currentMinutesOfDay();
  int startMin = to24Hour(s.startHour, s.startMinute, s.startAMPM);
  int stopMin = to24Hour(s.stopHour, s.stopMinute, s.stopAMPM);

  if (startMin == stopMin) return false;
  if (startMin < stopMin) return (nowMin >= startMin && nowMin < stopMin);
  return (nowMin >= startMin || nowMin < stopMin); // overnight window
}

int getHighestPriorityActiveScheduleIndex() {
  for (int i = 0; i < 3; i++) {
    if (isScheduleActiveNow(sched[i])) return i;
  }
  return -1;
}

void processSchedules() {
  int previousActiveIndex = activeScheduleIndex;
  int newActiveIndex = getHighestPriorityActiveScheduleIndex();

  if (activeRPMUpdateEnabled && previousActiveIndex != newActiveIndex) clearActiveRPMUpdate();

  if (freezeProtectionActive || activeRunMode == ACTIVE_FREEZE) {
    activeScheduleIndex = newActiveIndex;
    scheduleRequestedRPM = (newActiveIndex >= 0) ? sched[newActiveIndex].speed : 0;
    scheduleOwnsPump = false;
    return;
  }

  if (activeRunMode == ACTIVE_OVERRIDE_HIGH || activeRunMode == ACTIVE_OVERRIDE_LOW) {
    activeScheduleIndex = newActiveIndex;
    scheduleRequestedRPM = (newActiveIndex >= 0) ? sched[newActiveIndex].speed : 0;
    scheduleOwnsPump = false;
    return;
  }

  activeScheduleIndex = newActiveIndex;
  scheduleRequestedRPM = (newActiveIndex >= 0) ? sched[newActiveIndex].speed : 0;

  if (newActiveIndex < 0) {
    stopPumpForNoDemand();
    return;
  }

  if (autoRunInhibited) {
    scheduleOwnsPump = false;
    pendingRunRequest = false;
    requestedTargetRPM = 0;
    currentCommandedRPM = 0;
    pumpRunState = RUNSTATE_IDLE;
    lastKeepAliveMillis = 0;
    return;
  }

  int targetRPM = getEffectiveScheduleTargetRPM(newActiveIndex);

  if (!scheduleOwnsPump) {
    if (startOrChangeToRPM(targetRPM)) {
      scheduleOwnsPump = true;
      lastPumpReplyText = "Schedule " + String(newActiveIndex + 1) + " starting with prime at " + String(primeRPM) + " RPM";
    }
    return;
  }

  if (pumpRunState == RUNSTATE_PRIMING && pendingRunRequest) return;

  if (currentCommandedRPM != targetRPM || requestedTargetRPM != targetRPM) {
    if (startOrChangeToRPM(targetRPM)) {
      scheduleOwnsPump = true;
      lastPumpReplyText = "Schedule " + String(newActiveIndex + 1) + " active at " + String(targetRPM) + " RPM";
    }
  }
}

// =======================
// COMMAND PROCESSING
// =======================
String runOwnerText() {
  if (freezeProtectionActive || activeRunMode == ACTIVE_FREEZE) return "Freeze";
  if (activeRunMode == ACTIVE_OVERRIDE_HIGH) return "High";
  if (activeRunMode == ACTIVE_OVERRIDE_LOW) return "Low";
  if (activeScheduleIndex == 0) return "Sch 1";
  if (activeScheduleIndex == 1) return "Sch 2";
  if (activeScheduleIndex == 2) return "Sch 3";
  return "";
}

String runStateText() {
  if (autoRunInhibited) return "Schedule Hold";
  String owner = runOwnerText();
  switch (pumpRunState) {
    case RUNSTATE_IDLE: return "Idle";
    case RUNSTATE_PRIMING: return owner.length() > 0 ? "Prime " + owner : "Priming";
    case RUNSTATE_RUNNING:
      if (owner == "Freeze") return "Run Freeze";
      return owner.length() > 0 ? "Run " + owner : "Running";
    case RUNSTATE_FAULT: return "Fault";
    default: return "Unknown";
  }
}

unsigned long getPrimeSecondsRemaining() {
  if (pumpRunState != RUNSTATE_PRIMING || !pendingRunRequest) return 0;
  unsigned long primeDurationMs = (unsigned long)primeMinutes * 60000UL;
  unsigned long elapsed = millis() - primeStartMillis;
  if (elapsed >= primeDurationMs) return 0;
  return (primeDurationMs - elapsed) / 1000UL;
}

String activeRunModeText() {
  switch (activeRunMode) {
    case ACTIVE_OVERRIDE_HIGH: return "Override High";
    case ACTIVE_OVERRIDE_LOW: return "Override Low";
    case ACTIVE_OVERRIDE_MANUAL: return "Manual";
    case ACTIVE_FREEZE: return "Freeze";
    default: return "None";
  }
}

unsigned long getActiveRunSecondsRemaining() {
  if (!(activeRunMode == ACTIVE_OVERRIDE_HIGH || activeRunMode == ACTIVE_OVERRIDE_LOW)) return 0;
  if (activeRunDurationMs == 0) return 0;
  unsigned long elapsed = millis() - activeRunStartMillis;
  if (elapsed >= activeRunDurationMs) return 0;
  return (activeRunDurationMs - elapsed) / 1000UL;
}

String freezeStateText() {
  if (!freezeEnabled) return "Disabled";
  if (freezeProtectionActive) return "Active";
  if (freezeBelowStartMillis > 0) return "Waiting to Start";
  if (freezeAboveStartMillis > 0) return "Waiting to Clear";
  return "Armed";
}

unsigned long getFreezeStartCountdownSeconds() {
  if (freezeProtectionActive || freezeBelowStartMillis == 0) return 0;
  unsigned long elapsed = millis() - freezeBelowStartMillis;
  if (elapsed >= FREEZE_CONFIRM_MS) return 0;
  return (FREEZE_CONFIRM_MS - elapsed) / 1000UL;
}

unsigned long getFreezeClearCountdownSeconds() {
  if (!freezeProtectionActive || freezeAboveStartMillis == 0) return 0;
  unsigned long elapsed = millis() - freezeAboveStartMillis;
  if (elapsed >= FREEZE_CONFIRM_MS) return 0;
  return (FREEZE_CONFIRM_MS - elapsed) / 1000UL;
}

void processPumpCommands() {
  if (queuedCommand == QUEUED_CMD_NONE) return;

  if (isPrimeLocked()) {
    if (queuedCommand == QUEUED_CMD_RUN_HIGH || queuedCommand == QUEUED_CMD_RUN_LOW || queuedCommand == QUEUED_CMD_MANUAL_RPM) {
      lastPumpReplyText = "Prime in progress - RPM changes blocked until prime completes";
      queuedCommand = QUEUED_CMD_NONE;
      return;
    }
  }

  QueuedCommandType cmd = queuedCommand;
  queuedCommand = QUEUED_CMD_NONE;

  if (cmd == QUEUED_CMD_RESUME_SCHEDULE) {
    autoRunInhibited = false;
    scheduleOwnsPump = false;
    activeRunMode = ACTIVE_NONE;
    activeRunStartMillis = 0;
    activeRunDurationMs = 0;
    activeScheduleIndex = -1;
    scheduleRequestedRPM = 0;
    pendingRunRequest = false;
    requestedTargetRPM = 0;
    currentCommandedRPM = 0;
    clearActiveRPMUpdate();
    lastPumpReplyText = "Resume Schedule OK - schedule may restart if still active";
    return;
  }

  if (cmd == QUEUED_CMD_STOP) {
    String msg;
    if (epcStop(msg)) {
      pendingRunRequest = false;
      requestedTargetRPM = 0;
      currentCommandedRPM = 0;
      pumpRunState = RUNSTATE_IDLE;
      activeRunMode = ACTIVE_NONE;
      activeRunStartMillis = 0;
      activeRunDurationMs = 0;
      scheduleOwnsPump = false;
      freezeProtectionActive = false;
      freezeBelowStartMillis = 0;
      freezeAboveStartMillis = 0;
      activeScheduleIndex = -1;
      scheduleRequestedRPM = 0;
      auxRelayForcedOff = false;
      lastKeepAliveMillis = 0;
      autoRunInhibited = true;
      clearMotorSequences();
      clearActiveRPMUpdate();
      lastPumpReplyText = "STOP OK - auto restart inhibited until current schedule window ends";
      uint8_t status;
      if (epcReadStatus(status, msg)) { updateCachedStatus(status); lastGoodStatusMillis = millis(); }
    } else lastPumpReplyText = "STOP failed: " + msg;
    return;
  }

  if (cmd == QUEUED_CMD_STATUS) {
    String msg;
    if (refreshPumpStatus(msg)) {
      refreshAllSensorMonitoring();
      refreshLiveTelemetry();
      lastPumpReplyText = "STATUS OK";
    } else lastPumpReplyText = "STATUS failed: " + msg;
    return;
  }

  if (cmd == QUEUED_CMD_AUX_ON) {
    String msg;
    if (epcWriteAuxRelay(true, msg)) {
      auxEnabled = true;
      auxRelayForcedOff = false;
      lastPumpReplyText = "Aux ON sent";
    } else lastPumpReplyText = "Aux ON failed: " + msg;
    return;
  }

  if (cmd == QUEUED_CMD_AUX_OFF) {
    String msg;
    if (epcWriteAuxRelay(false, msg)) {
      auxEnabled = false;
      auxRelayForcedOff = true;
      lastAuxRetryMillis = millis();
      lastPumpReplyText = "Aux OFF sent";
    } else lastPumpReplyText = "Aux OFF failed: " + msg;
    return;
  }

  if (cmd == QUEUED_CMD_RUN_HIGH) {
    autoRunInhibited = false;
    clearActiveRPMUpdate();
    if (startOrChangeToRPM(overrideHighRPM)) {
      scheduleOwnsPump = false;
      activeRunMode = ACTIVE_OVERRIDE_HIGH;
      activeRunStartMillis = millis();
      activeRunDurationMs = (unsigned long)(overrideHighHours * 3600000.0f);
    }
    return;
  }

  if (cmd == QUEUED_CMD_RUN_LOW) {
    autoRunInhibited = false;
    clearActiveRPMUpdate();
    if (startOrChangeToRPM(overrideLowRPM)) {
      scheduleOwnsPump = false;
      activeRunMode = ACTIVE_OVERRIDE_LOW;
      activeRunStartMillis = millis();
      activeRunDurationMs = (unsigned long)(overrideLowHours * 3600000.0f);
    }
    return;
  }

  if (cmd == QUEUED_CMD_MANUAL_RPM) {
    if (!activeRPMUpdateAllowed()) {
      lastPumpReplyText = "RPM update only works during an active schedule or HIGH/LOW override";
    } else if (startOrChangeToRPM(manualRequestedRPM)) {
      activeRPMUpdateEnabled = true;
      activeRPMUpdateValue = manualRequestedRPM;
      activeRPMUpdateScheduleIndex = activeScheduleIndex;
      if (activeScheduleIndex >= 0) scheduleOwnsPump = true;
      lastPumpReplyText = "Active RPM updated to " + String(manualRequestedRPM) + " RPM";
    }
    return;
  }
}

void pollPumpStatusIfDue() {
  if (millis() - lastStatusPollMillis < STATUS_POLL_INTERVAL_MS) return;
  lastStatusPollMillis = millis();

  String msg;
  uint8_t status;
  if (epcReadStatus(status, msg)) {
    updateCachedStatus(status);
    lastGoodStatusMillis = millis();
  }

  refreshAllSensorMonitoring();
  refreshLiveTelemetry();
  updateAuxRelayControl();
}

void serviceWatchdogRecovery() {
  if (lastGoodStatusMillis > 0 && (millis() - lastGoodStatusMillis > 20000UL)) {
    if (!shouldAssertRunCommand()) clearMotorSequences();
    if (millis() - lastWebRecoveryMillis >= WEB_RECOVERY_INTERVAL_MS) {
      lastWebRecoveryMillis = millis();
      WiFiClient drain = server.available();
      if (drain) drain.stop();
    }
  }
}

// =======================
// HOME WIFI / NTP
// =======================
bool isZeroIP(const IPAddress &ip) {
  return ip[0] == 0 && ip[1] == 0 && ip[2] == 0 && ip[3] == 0;
}

int nthSundayOfMonth(int year, int month, int nth) {
  int dowFirst = calcDayOfWeek(year, month, 1);
  int firstSunday = (dowFirst == 0) ? 1 : (8 - dowFirst);
  return firstSunday + (nth - 1) * 7;
}

bool isCentralDSTFromStandardLocal(int year, int month, int day, int hourStandard) {
  if (month < 3 || month > 11) return false;
  if (month > 3 && month < 11) return true;
  if (month == 3) {
    int secondSunday = nthSundayOfMonth(year, 3, 2);
    if (day > secondSunday) return true;
    if (day < secondSunday) return false;
    return hourStandard >= 2;
  }
  int firstSunday = nthSundayOfMonth(year, 11, 1);
  if (day < firstSunday) return true;
  if (day > firstSunday) return false;
  return hourStandard < 2;
}

void setRTCFromUTCToCentral(long epochUtc) {
  long standardEpoch = epochUtc - (6L * 3600L);
  time_t raw = (time_t)standardEpoch;
  struct tm* t = gmtime(&raw);
  if (!t) return;

  int year = t->tm_year + 1900;
  int month = t->tm_mon + 1;
  int day = t->tm_mday;
  int hourStd = t->tm_hour;
  bool dst = isCentralDSTFromStandardLocal(year, month, day, hourStd);
  long localEpoch = standardEpoch + (dst ? 3600L : 0L);

  time_t localRaw = (time_t)localEpoch;
  struct tm* lt = gmtime(&localRaw);
  if (!lt) return;

  RTCTime rtc(lt->tm_mday,
              intToMonth(lt->tm_mon + 1),
              lt->tm_year + 1900,
              lt->tm_hour,
              lt->tm_min,
              lt->tm_sec,
              intToDayOfWeek(lt->tm_wday),
              dst ? SaveLight::SAVING_TIME_ACTIVE : SaveLight::SAVING_TIME_INACTIVE);
  RTC.setTime(rtc);
  copyRTCToClockSettings();
  lastTimeSyncText = rtcDateTimeString();
  timeSyncOk = true;
}

void serviceHomeWiFi() {
  if (homeWiFiConnected) {
    IPAddress ip = WiFi.localIP();
    if (isZeroIP(ip) || WiFi.status() != WL_CONNECTED) {
      homeWiFiConnected = false;
      homeWiFiIpText = "Not connected";
    } else {
      homeWiFiIpText = ip.toString();
    }
    return;
  }

  if (millis() - lastHomeWiFiAttemptMillis < HOME_WIFI_RETRY_INTERVAL_MS) return;
  lastHomeWiFiAttemptMillis = millis();

  int status = WiFi.begin(homeSsid, homePassword);
  unsigned long start = millis();
  while (millis() - start < 4000UL) {
    if (WiFi.status() == WL_CONNECTED) break;
    delay(50);
  }

  if (WiFi.status() == WL_CONNECTED) {
    homeWiFiConnected = true;
    homeWiFiIpText = WiFi.localIP().toString();
    if (!ntpStarted) {
      timeClient.begin();
      ntpStarted = true;
    }
  } else {
    homeWiFiConnected = false;
    homeWiFiIpText = "Not connected";
  }
}

void syncClockFromNTPIfNeeded() {
  if (!homeWiFiConnected || !ntpStarted) return;
  if (millis() - lastNTPSyncMillis < NTP_SYNC_INTERVAL_MS && timeSyncOk) return;
  if (timeClient.update() || timeClient.forceUpdate()) {
    setRTCFromUTCToCentral(timeClient.getEpochTime());
    lastNTPSyncMillis = millis();
    checkpointAllSettingsToEEPROM();
  }
}

// =======================
// WEB UI - SMALL SHELL PAGE
// =======================
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1,viewport-fit=cover'>
<title>VGreen 270 Pump Controller</title>
<style>
:root{
  --bg:#edf3f9; --panel:#ffffff; --line:#d6e0ea; --text:#132235; --muted:#5d6b7d;
  --primary:#0b63c8; --primarySoft:#dceafb; --danger:#c53a3a; --dangerSoft:#fde7e7;
  --ok:#177245; --warn:#9a6700; --shadow:0 6px 18px rgba(15,35,60,.08);
  --radius:16px; --radiusSm:12px;
}
*{box-sizing:border-box}
html,body{margin:0;padding:0;background:var(--bg);color:var(--text);font-family:Arial,sans-serif}
body{padding:10px}
.shell{max-width:1180px;margin:0 auto}
.topbar{display:grid;grid-template-columns:1.2fr .9fr 1.5fr;gap:10px;align-items:stretch}
.hero{background:linear-gradient(135deg,#0b63c8,#3e88df);color:#fff;border-radius:18px;padding:16px;box-shadow:var(--shadow)}
.hero h1{margin:0 0 4px 0;font-size:26px}
.hero .sub{font-size:13px;opacity:.92}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:var(--radius);padding:14px;box-shadow:var(--shadow)}
.panel h2{font-size:17px;margin:0 0 10px 0}
.miniLabel{font-size:12px;color:var(--muted);font-weight:700;text-transform:uppercase;letter-spacing:.04em}
.bigValue{font-size:22px;font-weight:700;margin-top:4px;word-break:break-word}
.replyBox{min-height:70px}
.statusStrip{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px;margin-top:10px}
.stat{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:12px;box-shadow:var(--shadow)}
.stat .miniLabel{margin-bottom:4px}
.stat .num{font-size:24px;font-weight:700}
.stat .small{font-size:14px;color:var(--muted)}
.tabs{display:flex;gap:8px;flex-wrap:wrap;margin:14px 0 10px}
.tabbtn{appearance:none;border:none;background:var(--primarySoft);color:#123;padding:11px 14px;border-radius:999px;font-size:15px;font-weight:700;cursor:pointer}
.tabbtn.active{background:var(--primary);color:#fff}
.tab{display:none}
.tab.active{display:block}
.section{display:grid;gap:10px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:var(--radius);padding:14px;box-shadow:var(--shadow)}
.cardTitle{font-size:16px;font-weight:700;margin:0 0 10px 0}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:10px}
.grid.tight{grid-template-columns:repeat(auto-fit,minmax(140px,1fr))}
.field{display:flex;flex-direction:column;gap:6px}
.label{font-weight:700;font-size:13px;color:var(--muted)}
.value{font-size:20px;font-weight:700}
.value.small{font-size:16px;font-weight:600}
.row{display:flex;flex-wrap:wrap;gap:10px}
.actions{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px}
button,input,select{font:inherit}
button{appearance:none;border:none;background:var(--primary);color:#fff;padding:12px 14px;border-radius:14px;font-weight:700;cursor:pointer;min-height:46px}
button.alt{background:#607286}
button.stop{background:var(--danger)}
button.ghost{background:#eef4fa;color:#18314f;border:1px solid var(--line)}
input,select{width:100%;padding:12px;border-radius:12px;border:1px solid #bccad9;background:#fff;color:var(--text);min-height:46px}
.notice{font-size:13px;color:var(--muted);line-height:1.35}
.hr{height:1px;background:var(--line);margin:12px 0}
.pillRow{display:flex;flex-wrap:wrap;gap:8px}
.pill{display:inline-flex;align-items:center;padding:8px 12px;border-radius:999px;background:#f3f7fb;border:1px solid var(--line);font-size:13px;font-weight:700;color:#234}
.scheduleCards{display:grid;gap:10px}
.scheduleCard{background:#f8fbfe;border:1px solid var(--line);border-radius:14px;padding:12px}
.scheduleHead{display:flex;justify-content:space-between;gap:8px;align-items:center;margin-bottom:8px}
.scheduleName{font-size:16px;font-weight:700}
.footerNote{font-size:12px;color:var(--muted);padding:6px 2px 20px}
@media (max-width:900px){
  .topbar{grid-template-columns:1fr}
  .actions{grid-template-columns:repeat(2,minmax(0,1fr))}
  .statusStrip{grid-template-columns:repeat(2,minmax(0,1fr))}
}
@media (max-width:560px){
  body{padding:8px}
  .hero h1{font-size:22px}
  .bigValue{font-size:19px}
  .value{font-size:18px}
  .actions,.statusStrip,.grid,.grid.tight{grid-template-columns:1fr}
  .tabbtn{flex:1 1 calc(50% - 8px);text-align:center}
}
</style></head><body>
<div class='shell'>
  <div class='topbar'>
    <div class='hero'>
      <h1>VGreen 270 Pump Controller</h1>
      <div class='sub'>Light UI build • lazy tabs • mobile + desktop layout</div>
    </div>
    <div class='panel'>
      <div class='miniLabel'>Controller Time</div>
      <div id='rtc' class='bigValue'>--</div>
      <div style='height:10px'></div>
      <div class='miniLabel'>Controller State</div>
      <div id='controllerState' class='bigValue'>--</div>
    </div>
    <div class='panel replyBox'>
      <div class='miniLabel'>Last Reply</div>
      <div id='replyText' class='bigValue' style='font-size:17px;line-height:1.25'>--</div>
    </div>
  </div>

  <div class='statusStrip'>
    <div class='stat'><div class='miniLabel'>Live RPM</div><div id='quickRpm' class='num'>--</div><div class='small'>Pump speed</div></div>
    <div class='stat'><div class='miniLabel'>Watts</div><div id='quickWatts' class='num'>--</div><div class='small'>Raw S05</div></div>
    <div class='stat'><div class='miniLabel'>Ambient °F</div><div id='quickTemp' class='num'>--</div><div class='small'>Controller ambient</div></div>
    <div class='stat'><div class='miniLabel'>Run Owner</div><div id='quickOwner' class='num' style='font-size:20px'>--</div><div class='small'>Schedule / override</div></div>
  </div>

  <div class='tabs'>
    <button class='tabbtn active' onclick='showTab("run",this)'>Run</button>
    <button class='tabbtn' onclick='showTab("live",this)'>Live</button>
    <button class='tabbtn' onclick='showTab("schedules",this)'>Schedules</button>
    <button class='tabbtn' onclick='showTab("setup",this)'>Setup</button>
    <button class='tabbtn' onclick='showTab("faults",this)'>Faults</button>
  </div>

  <div id='run' class='tab active'>
    <div class='section'>
      <div class='card'>
        <div class='cardTitle'>Pump Commands</div>
        <div class='actions'>
          <button onclick='cmd("run_high")'>Run High</button>
          <button onclick='cmd("run_low")'>Run Low</button>
          <button id='stopResumeBtn' class='stop' onclick='stopResume()'>Stop</button>
          <button class='alt' onclick='cmd("status")'>Refresh Status</button>
        </div>
        <div class='hr'></div>
        <div class='grid tight'>
          <div class='field'>
            <div class='label'>Manual RPM</div>
            <input id='manualRpm' type='number' min='600' max='3450' step='25' value='1800'>
          </div>
          <div class='field' style='justify-content:flex-end'>
            <div class='label'>&nbsp;</div>
            <button onclick='updateRPM()'>Update RPM</button>
          </div>
        </div>
        <div class='notice' style='margin-top:10px'>Manual RPM update only applies while an active schedule or HIGH/LOW override is running. If STOP is pressed during a schedule, this button changes to Resume Schedule.</div>
      </div>

      <div class='card'>
        <div class='cardTitle'>Current Run Summary</div>
        <div class='grid'>
          <div><div class='label'>Prime Remaining</div><div id='primeRemaining' class='value'>--</div></div>
          <div><div class='label'>Override Remaining</div><div id='overrideRemaining' class='value'>--</div></div>
          <div><div class='label'>Freeze State</div><div id='freezeState' class='value'>--</div></div>
          <div><div class='label'>Run Session</div><div id='runSession' class='value'>--</div></div>
        </div>
      </div>
    </div>
  </div>

  <div id='live' class='tab'>
    <div class='section'>
      <div class='card'>
        <div class='cardTitle'>Live Pump Data</div>
        <div class='grid'>
          <div><div class='label'>Pump Status</div><div id='pumpStatus' class='value'>--</div></div>
          <div><div class='label'>Status Hex</div><div id='pumpStatusHex' class='value'>--</div></div>
          <div><div class='label'>Live RPM</div><div id='liveRpm' class='value'>--</div></div>
          <div><div class='label'>Live Watts</div><div id='liveWatts' class='value'>--</div></div>
          <div><div class='label'>Ambient °F</div><div id='ambientF' class='value'>--</div></div>
          <div><div class='label'>Owner</div><div id='runOwner' class='value'>--</div></div>
        </div>
      </div>
    </div>
  </div>

  <div id='schedules' class='tab'>
    <div class='section'>
      <div class='card'>
        <div class='cardTitle'>Schedule Overview</div>
        <div id='scheduleSummary' class='pillRow'>Loading...</div>
      </div>
      <div class='card'>
        <div class='cardTitle'>Edit Schedules</div>
        <form onsubmit='saveSchedules(event)'>
          <div id='scheduleFormArea' class='scheduleCards'></div>
          <div class='row' style='margin-top:10px'>
            <button type='submit'>Save Schedules</button>
          </div>
        </form>
      </div>
    </div>
  </div>

  <div id='setup' class='tab'>
    <div class='section'>
      <div class='card'>
        <div class='cardTitle'>Run Setup</div>
        <form onsubmit='saveSetup(event)'>
          <div class='grid'>
            <div class='field'><div class='label'>Prime RPM</div><input id='primeRPM' type='number' min='1500' max='3450' step='25'></div>
            <div class='field'><div class='label'>Prime Minutes</div><input id='primeMinutes' type='number' min='1' max='10' step='1'></div>
            <div class='field'><div class='label'>High RPM</div><input id='highRPM' type='number' min='600' max='3450' step='25'></div>
            <div class='field'><div class='label'>High Hours</div><input id='highHours' type='number' min='0.5' max='24' step='0.5'></div>
            <div class='field'><div class='label'>Low RPM</div><input id='lowRPM' type='number' min='600' max='3450' step='25'></div>
            <div class='field'><div class='label'>Low Hours</div><input id='lowHours' type='number' min='0.5' max='24' step='0.5'></div>
            <div class='field'><div class='label'>Freeze Enabled</div><select id='freezeEnabled'><option value='0'>Off</option><option value='1'>On</option></select></div>
            <div class='field'><div class='label'>Freeze Temp °F</div><input id='freezeTemp' type='number' min='32' max='90' step='1'></div>
            <div class='field'><div class='label'>Aux Enabled</div><select id='auxEnabled'><option value='0'>Off</option><option value='1'>On</option></select></div>
          </div>
          <div class='row' style='margin-top:10px'>
            <button type='submit'>Save Setup</button>
          </div>
        </form>
      </div>

      <div class='card'>
        <div class='cardTitle'>Clock Settings</div>
        <form onsubmit='saveClock(event)'>
          <div class='grid tight'>
            <div class='field'><div class='label'>Hour</div><input id='clkHour' type='number' min='1' max='12'></div>
            <div class='field'><div class='label'>Minute</div><input id='clkMinute' type='number' min='0' max='59'></div>
            <div class='field'><div class='label'>AM / PM</div><select id='clkAmpm'><option value='AM'>AM</option><option value='PM'>PM</option></select></div>
            <div class='field'><div class='label'>Month</div><input id='clkMonth' type='number' min='1' max='12'></div>
            <div class='field'><div class='label'>Day</div><input id='clkDay' type='number' min='1' max='31'></div>
            <div class='field'><div class='label'>Year</div><input id='clkYear' type='number' min='2024' max='2099'></div>
          </div>
          <div class='row' style='margin-top:10px'>
            <button type='submit'>Save Clock</button>
            <button type='button' class='ghost' onclick='cmd("status")'>Sync From Controller</button>
          </div>
        </form>
      </div>
    </div>
  </div>

  <div id='faults' class='tab'>
    <div class='section'>
      <div class='card'>
        <div class='cardTitle'>Fault History</div>
        <div class='grid'>
          <div><div class='label'>Previous Fault</div><div id='prevFault' class='value small'>--</div></div>
          <div><div class='label'>Fault 1</div><div id='fault1' class='value small'>--</div></div>
          <div><div class='label'>Fault 2</div><div id='fault2' class='value small'>--</div></div>
          <div><div class='label'>Fault 3</div><div id='fault3' class='value small'>--</div></div>
          <div><div class='label'>Fault 4</div><div id='fault4' class='value small'>--</div></div>
        </div>
      </div>
    </div>
  </div>

  <div class='footerNote'>UI kept intentionally light so the controller stays responsive while pump communications continue.</div>
</div>

<script>
let activeTab='run';
let pollTimer=null;
function byId(id){return document.getElementById(id)}
function fillQuick(d){
  if(byId('quickRpm')) byId('quickRpm').textContent=d.liveRPM ?? '--';
  if(byId('quickWatts')) byId('quickWatts').textContent=d.liveWatts ?? '--';
  if(byId('quickTemp')) byId('quickTemp').textContent=d.ambientF ?? '--';
  if(byId('quickOwner')) byId('quickOwner').textContent=d.runOwner ?? '--';
}
function showTab(name,btn){
  activeTab=name;
  document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));
  document.querySelectorAll('.tabbtn').forEach(t=>t.classList.remove('active'));
  byId(name).classList.add('active');
  btn.classList.add('active');
  loadActiveTab(true);
}
async function j(url){const r=await fetch(url,{cache:'no-store'}); return await r.json();}
async function cmd(act){ await fetch('/cmd?act='+encodeURIComponent(act),{cache:'no-store'}); setTimeout(()=>loadActiveTab(true),150); }
async function stopResume(){ const b=byId('stopResumeBtn'); const act=(b&&b.dataset.mode==='resume')?'resume_schedule':'stop'; await cmd(act); }
async function updateRPM(){ const rpm=byId('manualRpm').value||1800; await fetch('/cmd?act=manual_rpm&rpm='+encodeURIComponent(rpm),{cache:'no-store'}); setTimeout(()=>loadActiveTab(true),150); }
async function saveSetup(ev){ ev.preventDefault(); const qs=new URLSearchParams({primeRPM:byId('primeRPM').value,primeMinutes:byId('primeMinutes').value,highRPM:byId('highRPM').value,highHours:byId('highHours').value,lowRPM:byId('lowRPM').value,lowHours:byId('lowHours').value,freezeEnabled:byId('freezeEnabled').value,freezeTemp:byId('freezeTemp').value,auxEnabled:byId('auxEnabled').value}); await fetch('/save_setup?'+qs.toString(),{cache:'no-store'}); setTimeout(()=>loadActiveTab(true),150); }
async function saveClock(ev){ ev.preventDefault(); const qs=new URLSearchParams({hour:byId('clkHour').value,minute:byId('clkMinute').value,ampm:byId('clkAmpm').value,month:byId('clkMonth').value,day:byId('clkDay').value,year:byId('clkYear').value}); await fetch('/save_clock?'+qs.toString(),{cache:'no-store'}); setTimeout(()=>loadActiveTab(true),150); }
async function saveSchedules(ev){ ev.preventDefault(); const fd=new FormData(ev.target); const qs=new URLSearchParams(fd); await fetch('/save_schedules?'+qs.toString(),{cache:'no-store'}); setTimeout(()=>loadActiveTab(true),150); }
function fillCommon(d){
  byId('rtc').textContent=d.rtc ?? '--';
  byId('controllerState').textContent=d.controllerState ?? '--';
  byId('replyText').textContent=d.replyText ?? '--';
  const sr=byId('stopResumeBtn');
  if(sr){
    if(d.autoRunInhibited){ sr.textContent='Resume Schedule'; sr.dataset.mode='resume'; sr.classList.remove('stop'); sr.classList.add('alt'); }
    else { sr.textContent='Stop'; sr.dataset.mode='stop'; sr.classList.add('stop'); sr.classList.remove('alt'); }
  }
  fillQuick(d);
}
async function loadRun(){ const d=await j('/api/live'); fillCommon(d); byId('primeRemaining').textContent=d.primeRemaining; byId('overrideRemaining').textContent=d.overrideRemaining; byId('freezeState').textContent=d.freezeState; byId('runSession').textContent=d.runSession; }
async function loadLive(){ const d=await j('/api/live'); fillCommon(d); byId('pumpStatus').textContent=d.pumpStatus; byId('pumpStatusHex').textContent=d.pumpStatusHex; byId('liveRpm').textContent=d.liveRPM; byId('liveWatts').textContent=d.liveWatts; byId('ambientF').textContent=d.ambientF; byId('runOwner').textContent=d.runOwner; }
async function loadSetup(){ const d=await j('/api/setup'); fillCommon(d); byId('primeRPM').value=d.primeRPM; byId('primeMinutes').value=d.primeMinutes; byId('highRPM').value=d.highRPM; byId('highHours').value=d.highHours; byId('lowRPM').value=d.lowRPM; byId('lowHours').value=d.lowHours; byId('freezeEnabled').value=d.freezeEnabled?1:0; byId('freezeTemp').value=d.freezeTemp; byId('auxEnabled').value=d.auxEnabled?1:0; byId('clkHour').value=d.clock.hour; byId('clkMinute').value=d.clock.minute; byId('clkAmpm').value=d.clock.ampm; byId('clkMonth').value=d.clock.month; byId('clkDay').value=d.clock.day; byId('clkYear').value=d.clock.year; }
async function loadSchedules(){
  const d=await j('/api/schedules'); fillCommon(d);
  byId('scheduleSummary').innerHTML=d.summaryHtml;
  let html='';
  d.schedules.forEach((s,i)=>{
    const n=i+1;
    html += `<div class='scheduleCard'>`;
    html += `<div class='scheduleHead'><div class='scheduleName'>Schedule ${n}</div><div class='pill'>${s.enabled?'Enabled':'Disabled'}</div></div>`;
    html += `<div class='grid'>`;
    html += `<div class='field'><div class='label'>Enabled</div><select name='s${n}en'><option value='0' ${!s.enabled?'selected':''}>Off</option><option value='1' ${s.enabled?'selected':''}>On</option></select></div>`;
    html += `<div class='field'><div class='label'>RPM</div><input name='s${n}rpm' type='number' min='600' max='3450' step='25' value='${s.speed}'></div>`;
    html += `<div class='field'><div class='label'>Start Hour</div><input name='s${n}sh' type='number' min='1' max='12' value='${s.startHour}'></div>`;
    html += `<div class='field'><div class='label'>Start Minute</div><input name='s${n}sm' type='number' min='0' max='59' value='${s.startMinute}'></div>`;
    html += `<div class='field'><div class='label'>Start AM/PM</div><select name='s${n}sap'><option value='AM' ${s.startAMPM=='AM'?'selected':''}>AM</option><option value='PM' ${s.startAMPM=='PM'?'selected':''}>PM</option></select></div>`;
    html += `<div class='field'><div class='label'>Stop Hour</div><input name='s${n}eh' type='number' min='1' max='12' value='${s.stopHour}'></div>`;
    html += `<div class='field'><div class='label'>Stop Minute</div><input name='s${n}em' type='number' min='0' max='59' value='${s.stopMinute}'></div>`;
    html += `<div class='field'><div class='label'>Stop AM/PM</div><select name='s${n}eap'><option value='AM' ${s.stopAMPM=='AM'?'selected':''}>AM</option><option value='PM' ${s.stopAMPM=='PM'?'selected':''}>PM</option></select></div>`;
    html += `</div></div>`;
  });
  byId('scheduleFormArea').innerHTML=html;
}
async function loadFaults(){ const d=await j('/api/faults'); fillCommon(d); byId('prevFault').textContent=d.prevFault; byId('fault1').textContent=d.fault1; byId('fault2').textContent=d.fault2; byId('fault3').textContent=d.fault3; byId('fault4').textContent=d.fault4; }
async function loadActiveTab(force=false){
  try{
    if(activeTab==='run') await loadRun();
    else if(activeTab==='live') await loadLive();
    else if(activeTab==='setup') await loadSetup();
    else if(activeTab==='schedules') await loadSchedules();
    else if(activeTab==='faults') await loadFaults();
  }catch(e){ console.log(e); }
  if(pollTimer) clearTimeout(pollTimer);
  pollTimer=setTimeout(()=>loadActiveTab(), activeTab==='run'||activeTab==='live' ? 3000 : 6000);
}
loadActiveTab(true);
</script>
</body></html>
)HTML";

// =======================
// HTTP HELPERS
// =======================
void serviceRealtimeTasksWhileWaiting() {
  processStartSequence();
  processRampSequence();
  processPrimeTransition();
  servicePumpKeepAlive();
  servicePumpStopKeepAlive();
}

bool readHttpRequestLine(WiFiClient &client, String &req) {
  unsigned long start = millis();
  req = "";
  while (millis() - start < 12UL) {
    while (client.available()) {
      char c = client.read();
      if (c == '\n') return req.length() > 0;
      if (c != '\r') req += c;
      if (req.length() > 220) return true;
    }
    serviceRealtimeTasksWhileWaiting();
  }
  return false;
}

void drainHttpHeadersQuick(WiFiClient &client) {
  unsigned long start = millis();
  while (millis() - start < 5UL) {
    while (client.available()) client.read();
    serviceRealtimeTasksWhileWaiting();
    if (!client.available()) break;
  }
}

void sendHttpHeader(WiFiClient &client, const char* contentType) {
  client.print(F("HTTP/1.1 200 OK\r\n"));
  client.print(F("Content-Type: "));
  client.print(contentType);
  client.print(F("\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n"));
}

void sendRedirectHome(WiFiClient &client) {
  client.print(F("HTTP/1.1 302 Found\r\nLocation: /\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n"));
}

void sendPlainOk(WiFiClient &client, const String& msg) {
  sendHttpHeader(client, "text/plain; charset=UTF-8");
  client.print(msg);
}

void streamIndexPage(WiFiClient &client) {
  sendHttpHeader(client, "text/html; charset=UTF-8");
  client.print(INDEX_HTML);
}

// lightweight JSON streaming helpers
void jsonBool(WiFiClient &client, bool v) { client.print(v ? F("true") : F("false")); }
void jsonStrKV(WiFiClient &client, const char* k, const String& v, bool comma=true) {
  client.print('"'); client.print(k); client.print(F("\":\"")); client.print(jsonEscape(v)); client.print('"'); if (comma) client.print(',');
}
void jsonIntKV(WiFiClient &client, const char* k, long v, bool comma=true) {
  client.print('"'); client.print(k); client.print(F("\":")); client.print(v); if (comma) client.print(',');
}
void jsonBoolKV(WiFiClient &client, const char* k, bool v, bool comma=true) {
  client.print('"'); client.print(k); client.print(F("\":")); jsonBool(client, v); if (comma) client.print(',');
}

String faultCombinedText(const String& hexCode, const String& text) {
  return hexCode + " - " + text;
}

void sendApiLive(WiFiClient &client) {
  sendHttpHeader(client, "application/json; charset=UTF-8");
  client.print('{');
  jsonStrKV(client, "rtc", rtcDateTimeString());
  jsonStrKV(client, "controllerState", runStateText());
  jsonStrKV(client, "replyText", lastPumpReplyText);
  jsonStrKV(client, "pumpStatus", lastPumpStatusText);
  jsonStrKV(client, "pumpStatusHex", lastPumpStatusHex);
  jsonIntKV(client, "liveRPM", livePumpRPM);
  jsonIntKV(client, "liveWatts", livePumpWatts);
  jsonStrKV(client, "ambientF", ambientTempValid ? formatFloat1(ambientTempF()) : String("--"));
  jsonStrKV(client, "runOwner", runOwnerText().length() ? runOwnerText() : String("None"));
  jsonStrKV(client, "primeRemaining", formatDurationSeconds(getPrimeSecondsRemaining()));
  jsonStrKV(client, "overrideRemaining", formatDurationSeconds(getActiveRunSecondsRemaining()));
  jsonStrKV(client, "freezeState", freezeStateText());
  jsonStrKV(client, "runSession", formatDurationSeconds(getRunSessionSeconds()));
  jsonIntKV(client, "requestedTargetRPM", requestedTargetRPM);
  jsonIntKV(client, "currentCommandedRPM", currentCommandedRPM);
  jsonBoolKV(client, "autoRunInhibited", autoRunInhibited);
  jsonIntKV(client, "activeScheduleIndex", activeScheduleIndex);
  jsonBoolKV(client, "commHealthy", (millis() - lastGoodStatusMillis) < 15000UL, false);
  client.print('}');
}

void sendApiFaults(WiFiClient &client) {
  sendHttpHeader(client, "application/json; charset=UTF-8");
  client.print('{');
  jsonStrKV(client, "rtc", rtcDateTimeString());
  jsonStrKV(client, "controllerState", runStateText());
  jsonStrKV(client, "replyText", lastPumpReplyText);
  jsonBoolKV(client, "autoRunInhibited", autoRunInhibited);
  jsonIntKV(client, "activeScheduleIndex", activeScheduleIndex);
  jsonStrKV(client, "prevFault", faultCombinedText(lastPrevFaultCodeHex, lastPrevFaultText));
  jsonStrKV(client, "fault1", faultCombinedText(lastFault1CodeHex, lastFault1Text));
  jsonStrKV(client, "fault2", faultCombinedText(lastFault2CodeHex, lastFault2Text));
  jsonStrKV(client, "fault3", faultCombinedText(lastFault3CodeHex, lastFault3Text));
  jsonStrKV(client, "fault4", faultCombinedText(lastFault4CodeHex, lastFault4Text), false);
  client.print('}');
}

void sendApiSetup(WiFiClient &client) {
  sendHttpHeader(client, "application/json; charset=UTF-8");
  client.print('{');
  jsonStrKV(client, "rtc", rtcDateTimeString());
  jsonStrKV(client, "controllerState", runStateText());
  jsonStrKV(client, "replyText", lastPumpReplyText);
  jsonBoolKV(client, "autoRunInhibited", autoRunInhibited);
  jsonIntKV(client, "activeScheduleIndex", activeScheduleIndex);
  jsonIntKV(client, "primeRPM", primeRPM);
  jsonIntKV(client, "primeMinutes", primeMinutes);
  jsonIntKV(client, "highRPM", overrideHighRPM);
  jsonStrKV(client, "highHours", String(overrideHighHours,1));
  jsonIntKV(client, "lowRPM", overrideLowRPM);
  jsonStrKV(client, "lowHours", String(overrideLowHours,1));
  jsonBoolKV(client, "freezeEnabled", freezeEnabled);
  jsonIntKV(client, "freezeTemp", freezeTemp);
  jsonBoolKV(client, "auxEnabled", auxEnabled);
  client.print(F("\"clock\":{"));
  jsonIntKV(client, "month", clockTime.month);
  jsonIntKV(client, "day", clockTime.day);
  jsonIntKV(client, "year", clockTime.year);
  jsonIntKV(client, "hour", clockTime.hour);
  jsonIntKV(client, "minute", clockTime.minute);
  jsonStrKV(client, "ampm", String(clockTime.ampm), false);
  client.print(F("}}"));
}

void sendApiSchedules(WiFiClient &client) {
  sendHttpHeader(client, "application/json; charset=UTF-8");
  client.print('{');
  jsonStrKV(client, "rtc", rtcDateTimeString());
  jsonStrKV(client, "controllerState", runStateText());
  jsonStrKV(client, "replyText", lastPumpReplyText);
  jsonBoolKV(client, "autoRunInhibited", autoRunInhibited);
  jsonIntKV(client, "activeScheduleIndex", activeScheduleIndex);
  client.print(F("\"summaryHtml\":\""));
  String summary = "Active schedule: ";
  if (activeScheduleIndex >= 0) summary += String(activeScheduleIndex + 1);
  else summary += "None";
  summary += "<br>Schedule owner: ";
  summary += scheduleOwnsPump ? "Yes" : "No";
  summary += "<br>Schedule hold-off: ";
  summary += autoRunInhibited ? "Yes - press Resume Schedule to restart" : "No";
  client.print(jsonEscape(summary));
  client.print(F("\",\"schedules\":["));
  for (int i = 0; i < 3; i++) {
    if (i) client.print(',');
    client.print('{');
    jsonBoolKV(client, "enabled", sched[i].enabled);
    jsonIntKV(client, "speed", sched[i].speed);
    jsonIntKV(client, "startHour", sched[i].startHour);
    jsonIntKV(client, "startMinute", sched[i].startMinute);
    jsonStrKV(client, "startAMPM", String(sched[i].startAMPM));
    jsonIntKV(client, "stopHour", sched[i].stopHour);
    jsonIntKV(client, "stopMinute", sched[i].stopMinute);
    jsonStrKV(client, "stopAMPM", String(sched[i].stopAMPM));
    jsonStrKV(client, "status", scheduleStatusText(i), false);
    client.print('}');
  }
  client.print(F("]}"));
}

// =======================
// HTTP ACTION ROUTES
// =======================
void handleCommandRoute(WiFiClient &client, const String& req) {
  String act = getParam(req, "act");
  if (act == "stop") queueOneCommand(QUEUED_CMD_STOP);
  else if (act == "resume_schedule") queueOneCommand(QUEUED_CMD_RESUME_SCHEDULE);
  else if (act == "run_high") queueOneCommand(QUEUED_CMD_RUN_HIGH);
  else if (act == "run_low") queueOneCommand(QUEUED_CMD_RUN_LOW);
  else if (act == "status") queueOneCommand(QUEUED_CMD_STATUS);
  else if (act == "aux_on") queueOneCommand(QUEUED_CMD_AUX_ON);
  else if (act == "aux_off") queueOneCommand(QUEUED_CMD_AUX_OFF);
  else if (act == "manual_rpm") {
    manualRequestedRPM = clampInt(getParam(req, "rpm").toInt(), 600, 3450);
    queueOneCommand(QUEUED_CMD_MANUAL_RPM);
  }
  sendPlainOk(client, "OK");
}

void handleSaveSetupRoute(WiFiClient &client, const String& req) {
  primeRPM = clampInt(getParam(req, "primeRPM").toInt(), 1500, 3450);
  primeMinutes = clampInt(getParam(req, "primeMinutes").toInt(), 1, 10);
  overrideHighRPM = clampInt(getParam(req, "highRPM").toInt(), 600, 3450);
  overrideHighHours = clampFloat(getParam(req, "highHours").toFloat(), 0.5f, 24.0f);
  overrideLowRPM = clampInt(getParam(req, "lowRPM").toInt(), 600, 3450);
  overrideLowHours = clampFloat(getParam(req, "lowHours").toFloat(), 0.5f, 24.0f);
  freezeEnabled = getBoolParam(req, "freezeEnabled");
  freezeTemp = clampInt(getParam(req, "freezeTemp").toInt(), 32, 90);
  auxEnabled = getBoolParam(req, "auxEnabled");
  saveSettings();
  lastPumpReplyText = "Setup saved";
  sendPlainOk(client, "OK");
}

void handleSaveClockRoute(WiFiClient &client, const String& req) {
  clockTime.hour = clampInt(getParam(req, "hour").toInt(), 1, 12);
  clockTime.minute = clampInt(getParam(req, "minute").toInt(), 0, 59);
  setAMPM(clockTime.ampm, getParam(req, "ampm"));
  clockTime.month = clampInt(getParam(req, "month").toInt(), 1, 12);
  clockTime.day = clampInt(getParam(req, "day").toInt(), 1, 31);
  clockTime.year = clampInt(getParam(req, "year").toInt(), 2024, 2099);
  setRTCFromClockSettings();
  saveSettings();
  lastPumpReplyText = "Clock saved";
  sendPlainOk(client, "OK");
}

void handleSaveSchedulesRoute(WiFiClient &client, const String& req) {
  for (int i = 0; i < 3; i++) {
    String n = String(i + 1);
    sched[i].enabled = getBoolParam(req, "s" + n + "en");
    sched[i].speed = clampInt(getParam(req, "s" + n + "rpm").toInt(), 600, 3450);
    sched[i].startHour = clampInt(getParam(req, "s" + n + "sh").toInt(), 1, 12);
    sched[i].startMinute = clampInt(getParam(req, "s" + n + "sm").toInt(), 0, 59);
    setAMPM(sched[i].startAMPM, getParam(req, "s" + n + "sap"));
    sched[i].stopHour = clampInt(getParam(req, "s" + n + "eh").toInt(), 1, 12);
    sched[i].stopMinute = clampInt(getParam(req, "s" + n + "em").toInt(), 0, 59);
    setAMPM(sched[i].stopAMPM, getParam(req, "s" + n + "eap"));
  }
  saveSettings();
  lastPumpReplyText = "Schedules saved";
  sendPlainOk(client, "OK");
}

void handleClient(WiFiClient &client) {
  if (!client) return;
  String req;
  if (!readHttpRequestLine(client, req)) { client.stop(); return; }
  drainHttpHeadersQuick(client);

  if (req.startsWith("GET /api/live")) sendApiLive(client);
  else if (req.startsWith("GET /api/faults")) sendApiFaults(client);
  else if (req.startsWith("GET /api/setup")) sendApiSetup(client);
  else if (req.startsWith("GET /api/schedules")) sendApiSchedules(client);
  else if (req.startsWith("GET /cmd")) handleCommandRoute(client, req);
  else if (req.startsWith("GET /save_setup")) handleSaveSetupRoute(client, req);
  else if (req.startsWith("GET /save_clock")) handleSaveClockRoute(client, req);
  else if (req.startsWith("GET /save_schedules")) handleSaveSchedulesRoute(client, req);
  else if (req.startsWith("GET / ") || req.startsWith("GET /HTTP")) streamIndexPage(client);
  else sendRedirectHome(client);

  client.flush();
  client.stop();
}

void serviceWebServer() {
  if (keepAliveUrgent() || stopKeepAliveUrgent() || isSystemBusyWithMotorSequence()) return;
  WiFiClient client = server.available();
  if (client) handleClient(client);
}

// =======================
// SETUP / LOOP
// =======================
void setup() {
  pinMode(RS485_DE_PIN, OUTPUT);
  rs485SetTransmit(false);

  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("Booting Pool Pump Controller v1.9 light UI build...");

  Serial1.begin(9600);
  RTC.begin();

  loadSettings();
  copyRTCToClockSettings();

  WiFi.beginAP(ssid, password);
  delay(500);
  server.begin();

  Serial.print("AP IP: ");
  Serial.println(WiFi.localIP());

  String msg;
  uint8_t status;
  if (epcReadStatus(status, msg)) {
    updateCachedStatus(status);
    lastGoodStatusMillis = millis();
  } else {
    lastPumpReplyText = "Startup status failed: " + msg;
  }

  refreshAllSensorMonitoring();
  refreshLiveTelemetry();
  lastSettingsCheckpointMillis = millis();
}

void loop() {
  lastLoopHealthyMillis = millis();

  processStartSequence();
  processRampSequence();
  processPrimeTransition();
  processPumpCommands();
  servicePumpKeepAlive();
  servicePumpStopKeepAlive();
  pollPumpStatusIfDue();
  processActiveRunTimeout();
  processFreezeProtection();
  processSchedules();
  serviceHomeWiFi();
  syncClockFromNTPIfNeeded();
  serviceWatchdogRecovery();

  if (millis() - lastSettingsCheckpointMillis >= SETTINGS_CHECKPOINT_INTERVAL_MS) {
    lastSettingsCheckpointMillis = millis();
    checkpointAllSettingsToEEPROM();
  }

  serviceWebServer();
}
