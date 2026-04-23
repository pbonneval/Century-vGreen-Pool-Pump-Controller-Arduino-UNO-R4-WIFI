# Century vGreen Pool Pump Controller

![Platform](https://img.shields.io/badge/Platform-Arduino%20UNO%20R4%20WiFi-blue)
![Interface](https://img.shields.io/badge/Interface-RS--485-orange)
![Protocol](https://img.shields.io/badge/Protocol-Custom%20EPC-critical)
![Status](https://img.shields.io/badge/Status-Stable-success)

---

## 🚀 Overview

This project is a **robust, standalone pool pump controller** for my Century / Regal Beloit vGreen 270 variable-speed pump. This controller may be compatible with other model Century / Regal Beloit vGreen 270 variable-speed pumps, **implement at your own risk**.

Built on the **Arduino UNO R4 WiFi**, the system communicates over **RS-485 using a custom EPC protocol** and provides a **lightweight web-based UI** for full local control.

Designed for **continuous operation**, the controller uses:

* Non-blocking architecture
* Command queue execution
* Watchdog-style recovery
* Reliable keepalive communication

---

## 🌐 Device Access & WiFi Modes

> ⚠️ **WiFi Compatibility Note**
> The Arduino UNO R4 WiFi supports **2.4 GHz networks only**.
> It is **not compatible with 5 GHz WiFi networks**.

---

### 🔵 Access Point Mode (Default)

The controller starts in **Access Point (AP) mode**.

* **SSID:** `POOL PUMP` *(default — configurable in code)*
* **Password:** *(blank by default → open network)*

Open in browser:

```id="apurl1"
http://192.168.4.1
```

**Notes:**

* Direct connection to controller
* No internet required
* “No Internet” warning is normal
* SSID/password configurable in code:

```cpp id="apcode"
const char* ssid     = "POOL PUMP";
const char* password = "";
```

---

### 🟢 Home WiFi Mode (Optional)

Controller can connect to your home network:

```cpp id="homewifi"
const char* homeSsid     = "YOUR_HOME_WIFI_SSID";
const char* homePassword = "YOUR_HOME_WIFI_PASSWORD";
```

**Behavior:**

* IP is assigned automatically (DHCP)
* IP may change after reboot or router reset

---

### 📌 Recommended: DHCP Reservation

To keep a consistent IP:

1. Open router settings
2. Find device in connected devices
3. Reserve its IP using MAC address

Example:

```id="staticip"
192.168.1.50
```

---

### 🔁 Mode Behavior

* AP mode is always available
* Home WiFi runs in parallel
* AP remains fallback if home WiFi fails

---

### 🕒 Clock & Time Synchronization

The controller uses the onboard RTC (Real-Time Clock) for scheduling and timekeeping.

#### 🔵 Access Point Mode (AP Only)
- When operating only in AP mode, the controller **does not have access to internet time sources**
- The clock will use the **last stored RTC value**
- Over time, the clock **may drift and become inaccurate**

#### 🟢 Home WiFi Mode
- When connected to a home WiFi network, the controller will:
  - Automatically **synchronize time using NTP**
  - Maintain accurate time for schedules and system operation

#### 📌 Important Notes
- Time synchronization occurs **only when connected to WiFi with internet access**
- After power loss, the RTC will retain time, but may not be perfectly accurate
- If operating in AP-only mode long-term, periodic clock updates are recommended

#### ✅ Recommendation
For best accuracy:
- Connect the controller to your home WiFi network  
- Or periodically update the clock via the web UI  

> ⚠️ If used exclusively in AP mode, schedules may gradually shift due to clock drift.

---

## 🖥️ Web Interface

### ▶️ Run Control

![Run](images/run.png)

* Start / Stop pump
* Override modes
* Manual RPM adjustment

---

### 📊 Live Telemetry

![Live](images/live.png)

Displays:

* Pump state
* RPM
* Power (watts)
* Temperature
* Controller state

---

### 📅 Scheduling

![Schedules](images/schedules.png)

* 3 schedules
* Priority-based (1 > 2 > 3)
* RTC-driven
* Prime always enforced

---

### ⚙️ System Setup

![Setup](images/setup.png)

Configure:

* Prime settings
* Override settings
* Freeze protection
* Aux relay behavior
* Clock

---

### ⚠️ Fault Monitoring

![Faults](images/faults.png)

* Active faults
* Previous faults
* Decoded fault descriptions

---

## 🔌 Hardware

### UNO R4 WIFI Controller
![UNO R4](images/unor4.png)
### RS-485 Shield
-Ensure switches on shield are set for UART and RS485
![RS485 Shield](images/serialshield.png)

---

### 🔗 RS-485 Wiring - Serial Shield to Pump 

![RS485 Wiring](images/wiring.png)

**Connections:**

| Controller | Pump / RS-485 |
|----------|----------------|
| A / D+   | A              |
| B / D-   | B              |
| GND      | GND (if available) |

📌 **Notes:**
- Ensure A/B lines are not swapped  
- Some pumps label A/B differently → may require swapping  
- Use twisted pair wiring for best signal integrity  
- Keep cable runs as short as practical  

---

### Pump Interface Reference
![Pump Wiring](images/pump485.png)
### Optional TTL to RS-485 Converter in place of Serial Shield
-Pin 0 and Pin 1 on the UNO for TTL RXD and TXD
-Caution the VCC and GND colors may be reversed
![TTL Adapter](images/ttlto485.png)
## 🧩 Architecture

* Command queue prevents collisions
* Start sequence enforces proper startup
* Prime always runs first
* Ramp engine prevents faults
* Keepalive prevents timeout
* Schedule engine uses RTC
* Freeze protection monitors temp
* Watchdog prevents lockups

---

## ❄️ Freeze Protection

* Below setpoint for 30 min → start pump
* Runs at ~1000 RPM
* Above setpoint for 30 min → stop

---

## 🔁 Control Behavior

* All starts: **Prime → Run**
* Prime cannot be interrupted
* Overrides > schedules
* Schedules > idle
* STOP blocks restart until schedule change

---

## 📡 Protocol Notes

* Custom EPC protocol (not Modbus)
* CRC16 (0xA001 polynomial)
* Pump address: `0x15`
* Requires continuous communication

Reference:
https://www.troublefreepool.com/threads/century-regal-vgreen-motor-automation.238733/

---

## 🛠️ Setup

### 1. Upload Code

* Open `.ino` in Arduino IDE
* Select UNO R4 WiFi
* Upload

---

### 2. Configure WiFi

```cpp
const char* ssid     = "POOL PUMP";
const char* password = "";
```

Optional:

```cpp
const char* homeSsid     = "YOUR_HOME_WIFI_SSID";
const char* homePassword = "YOUR_HOME_WIFI_PASSWORD";
```

---

### 3. Connect

#### Direct (Recommended)

1. Connect to `POOL PUMP`
2. Open:

```id="apurl2"
http://192.168.4.1
```

---

#### Home Network

1. Find IP in router
2. Open:

```
http://<assigned-ip>
```

---

### 4. Recommended Network Setup

✔ Set DHCP reservation
✔ Use fixed IP
✔ Bookmark controller

---

### 5. Configure System

* Set clock
* Set schedules
* Set prime
* Verify pump control

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
* EEPROM + RTC synchronization refined

---

## 📜 Disclaimer

Use at your own risk.
Verify wiring, safety, and compatibility before deployment.
