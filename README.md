# Century vGreen Pool Pump Controller



\

---

## ⚡ Quick Start

1. Upload the `.ino` to Arduino UNO R4 WiFi
2. Connect to WiFi: **POOL PUMP**
3. Open browser:

```
http://192.168.4.1
```

4. Configure settings and start pump

---

## 🚀 Overview

 This project is a **robust, standalone pool pump controller** for my Century / Regal Beloit vGreen 270 variable-speed pump. This controller may be compatible with other model Century / Regal Beloit vGreen variable-speed pumps, **implement at your own risk**.  

Built on the **Arduino UNO R4 WiFi**, the system communicates over **RS-485 using a custom EPC protocol** and provides a **lightweight web-based UI** for full local control.

Designed for **continuous operation**, the controller uses:

* Non-blocking architecture
* Command queue execution
* Watchdog-style recovery
* Reliable keepalive communication

---

## 🌐 Device Access & WiFi Modes

> ⚠️ **WiFi Compatibility Note**
> Arduino UNO R4 WiFi supports **2.4 GHz only** (not 5 GHz)

---

### 🔵 Access Point Mode (Default)

* **SSID:** `POOL PUMP`
* **Password:** *(open network by default)*

Open:

```
http://192.168.4.1
```

**Notes:**

* No internet required
* “No Internet” warning is normal
* SSID/password configurable in code

```cpp
const char* ssid     = "POOL PUMP";
const char* password = "";
```

---

### 🟢 Home WiFi Mode (Optional)

```cpp
const char* homeSsid     = "YOUR_HOME_WIFI_SSID";
const char* homePassword = "YOUR_HOME_WIFI_PASSWORD";
```

* IP assigned automatically (DHCP)
* IP may change over time

---

### 📌 Recommended: Static IP via Router

Assign a DHCP reservation to keep a consistent IP:

Example:

```
192.168.1.50
```

---

### 🔁 Mode Behavior

* AP mode always available
* Home WiFi runs in parallel
* AP remains fallback

---

### 🕒 Clock & Time Synchronization

#### AP Mode

* No internet → no time sync
* RTC used only
* Time may drift

#### Home WiFi Mode

* NTP sync enabled
* Accurate time maintained

📌 Recommendation:

* Use WiFi for accurate schedules
* Or manually update clock

---

## 🖥️ Web Interface

### ▶️ Run Control

* Start / Stop
* Override modes
* Manual RPM control

---

### 📊 Live Telemetry

* RPM
* Watts
* Temperature
* Pump state

---

### 📅 Scheduling

* 3 schedules
* Priority-based
* RTC driven
* Prime always enforced

---

### ⚙️ Setup

* Prime settings
* Overrides
* Freeze protection
* Clock
* Aux control

---

### ⚠️ Faults

* Active faults
* Previous faults
* Descriptions

---

## 🔌 Hardware

### UNO R4 WiFi Controller

### RS-485 Shield

* Set switches for UART + RS485\

---

### 🔗 RS-485 Wiring

| Controller | Pump |
| ---------- | ---- |
| A / D+     | A    |
| B / D-     | B    |
| GND        | GND  |

📌 Notes:

* Swap A/B if no communication
* Use twisted pair
* Keep wires short

---

### Pump Interface

### Optional TTL Adapter

---

## 🧩 Architecture

* Command queue prevents collisions
* Prime-first startup always enforced
* Ramp engine smooths transitions
* Keepalive prevents timeout
* Schedule engine uses RTC
* Freeze protection monitors temp
* Watchdog prevents lockups

---

## ❄️ Freeze Protection

* Below setpoint (30 min) → start pump
* Runs at ~1000 RPM
* Above setpoint (30 min) → stop

---

## 🔁 Control Behavior

* Start = **Prime → Run**
* Prime cannot be interrupted
* Overrides > schedules
* Schedules > idle
* STOP blocks restart

---

## 📡 Protocol Notes

* Custom EPC protocol (not Modbus)
* CRC16 (0xA001)
* Pump address: `0x15`
* Requires continuous communication

Reference:
https://www.troublefreepool.com/threads/century-regal-vgreen-motor-automation.238733/

---

## 🛠️ Setup

### 1. Upload Code

* Select UNO R4 WiFi
* Upload sketch

---

### 2. Configure WiFi

```cpp
const char* ssid     = "POOL PUMP";
const char* password = "";
```

Optional:

```cpp
const char* homeSsid     = "...";
const char* homePassword = "...";
```

---

### 3. Connect

**Direct:**

```
http://192.168.4.1
```

**Home WiFi:**

```
http://<assigned-ip>
```

---

### 4. Configure System

* Set clock
* Set schedules
* Set prime
* Test pump

---

## 🛠️ Troubleshooting

### No Communication with Pump

* Swap A/B wires
* Verify RS485 switch settings
* Confirm Serial1 wiring (pins 0/1)
* Check pump address (0x15)

---

### Cannot Connect to WiFi

* Ensure 2.4 GHz network
* Try AP mode first
* Verify credentials

---

### UI Freezes / Pump Stops

* Avoid rapid commands
* Controller may be busy
* Allow time between actions

---

### Time Incorrect

* AP mode does not sync time
* Use WiFi for NTP sync
* Manually update clock

---

## 📁 Project Structure

```
Century-vGreen-Pool-Pump-Controller-Arduino-UNO-R4-WIFI/
├── .ino
├── README.md
├── images/
├── docs/
├── hardware/
```

---

## 🚧 Engineering Notes

* Aux relay tied to pump config
* Strict keepalive required
* UI polling impacts performance
* Ramp-down sensitive to faults
* EEPROM + RTC sync optimized

---

## ⚠️ Disclaimer

This project controls electrical equipment.
Use caution and verify all wiring before operation.
