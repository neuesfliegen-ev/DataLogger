# 🛩️ Flight Data Logger

This project is a **Flight Data Logger** designed for UAVs and model aircraft. It logs sensor and GPS data at 1 Hz to an SD card in CSV format and streams live telemetry to a server over a cellular (3G/4G) link. The system uses an IMU, a barometric sensor and a GPS module, and shows live status updates on an OLED screen.

## 📦 Features

- Logs data to SD card in `.csv` format
- Captures:
  - 9-axis IMU data (acceleration, gyroscope, magnetometer)
  - Barometric pressure, altitude and temperature
  - GPS coordinates, altitude, speed and satellite count
- OLED display for live status updates
- **Live telemetry over cellular**: an ESP32 gateway forwards data to a web server through a SIM7600 modem (see [Live Telemetry](#-live-telemetry-esp32--sim7600))
- Live dashboard for the telemetry data
- Modular and extensible design for future sensor integration

## 🧠 Core Components

| Component           | Description |
|---------------------|-------------|
| **Arduino**         | Primary microcontroller (Nano 33 BLE Sense REV2): sensors, SD logging, OLED |
| **ESP32**           | Telemetry gateway between the Arduino and the cellular modem |
| **SIM7600**         | Cellular modem (with SIM card), controlled by AT commands over UART |
| **BMI270_BMM150**   | 9-DOF IMU (acceleration, gyroscope, magnetometer) |
| **LPS22HB**         | Barometric pressure sensor |
| **TinyGPS++**       | GPS data parser |
| **SH1106 OLED**     | I2C 128x64 screen for live feedback |
| **SD Card Module**  | Stores `.csv` data logs |
| **Voltage Divider** | Scales down battery voltage for safe ADC reading |

---

## 📡 Live Telemetry (ESP32 + SIM7600)

Code: [`esp32SimNano.ino`](esp32SimNano.ino)

```
Arduino Nano ──I2C (JSON frames)──► ESP32 ──UART (AT commands)──► SIM7600 ──HTTP POST──► Server ──► Dashboard
     ▲                                │
     └────── READY / ACK (GPIO) ──────┘
```

- **Arduino → ESP32:** the ESP32 runs as an I2C slave (address `0x42`) and receives each data frame as a JSON payload.
- **Handshake:** two GPIO lines coordinate the transfer:
  - `READY` (ESP32 → Nano, GPIO 4): high when the modem is up and the ESP32 can accept a new frame.
  - `ACK` (ESP32 → Nano, GPIO 5): high when the last frame was delivered successfully (HTTP 200).
- **ESP32 → SIM7600:** UART2 at 115200 baud (ESP32 TX = GPIO 17, RX = GPIO 16). On startup the ESP32:
  1. waits until the modem answers `AT`
  2. disables echo (`ATE0`)
  3. opens the network (`AT+NETOPEN`)
  4. starts an HTTP session (`AT+HTTPINIT`, recovering with `AT+HTTPTERM` if a session is stuck)
  5. sets the server URL and `application/json` content type (`AT+HTTPPARA`)
- **Sending:** each frame is uploaded with `AT+HTTPDATA` and sent as an HTTP POST (`AT+HTTPACTION=1`). The ESP32 then waits for an HTTP 200 response.
- **Health check:** the ESP32 polls the modem with `AT` every 200 ms and updates the `READY` line.
- **Modes:** the payload carries a `"mode"` field (`REALTIME` or `BATCH`), which the ESP32 parses.

---

## 🔧 Circuit Diagram

[Wiring diagram](Datenlogger_Files/Datalogger_schematic%20v2.pdf)

> Note: the schematic still shows a 430 Ω resistor in the voltage divider. Use **510 Ω** as described below.

---

## 🔋 ADC Voltage Divider

To safely read the battery voltage with the Arduino's 3.3 V ADC pin, use the following voltage divider:

- **Input voltage**: up to 4.2 V (LiPo 1S max)
- **Top resistor (battery side)**: 510 Ω
- **Bottom resistor (to GND, ADC reads across it)**: 1.8 kΩ

### 📐 Voltage Divider Calculation

```
Vout = Vin × R_bottom / (R_top + R_bottom)
     = 4.2 V × 1800 / (510 + 1800)
     ≈ 3.27 V
```

---

## 📂 Data Format

Logged data is stored as a CSV file (`log.csv`) with the following headers:

```
timestamp,accX,accY,accZ,gyroX,gyroY,gyroZ,magX,magY,magZ,
latitude,longitude,gpsAltitude,Speed,SatCount,roll,pitch,yaw,
pressure,temperature,paltitude
```

Example row:

```
1542,-0.01,0.02,9.81,0.01,-0.02,0.00,30.12,11.23,-45.66,
52.5200,13.4050,120.5,10.4,9,0,0,0,100.3,25.5,132.2
```

---

## 🛠️ Getting Started

### 🔌 Wiring

- Connect sensors and modules via I2C/SPI/Serial as per the diagram.
- Ensure GPS is connected to `Serial1` (typically `TX`/`RX` on the Arduino). (IMPORTANT)
- For telemetry: connect the Arduino to the ESP32 over I2C plus the `READY`/`ACK` lines, and the ESP32 to the SIM7600 over UART (pins above). Insert an active SIM card.
- Insert a **FAT32-formatted SD card**.

### 🧪 Uploading

1. Install dependencies (Arduino):
   - `Arduino_BMI270_BMM150`
   - `Arduino_LPS22HB`
   - `TinyGPSPlus`
   - `U8g2`
2. Compile and upload the main sketch to the Arduino.
3. Upload `esp32SimNano.ino` to the ESP32 (ESP32 board package, uses `Wire` and `HardwareSerial`). Set the server URL in `initSIM7600()` if needed.
4. Open the Serial Monitor at **115200 baud** to see real-time debug info (the ESP32 also echoes modem responses there).

---

## 📊 OLED Display

The OLED shows system status:
- Sensor initialization
- GPS satellite count
- Errors during SD or sensor setup
- Team number

> To keep the display clear and reduce power, messages are brief and limited to startup and GPS updates.

---

## 🧩 Planned Improvements

- Add orientation calculation (roll, pitch, yaw)
- Support logging at configurable rates (5 Hz, 10 Hz)
- Add battery voltage and current sensors

---

## 👨‍💻 Credits

- Developed by the **Neues Fliegen Avionics Team**
- Based on libraries by Arduino, Adafruit and Mikal Hart (TinyGPS++)
