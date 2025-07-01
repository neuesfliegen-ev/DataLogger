# 🛩️ Flight Data Logger

This project is a **Flight Data Logger** designed for UAVs and model aircrafts. It logs sensor and GPS data at 1 Hz to an SD card in CSV format. The system uses various sensors including an IMU, barometric sensor, and GPS module, and displays live status updates on an OLED screen.

## 📦 Features

- Logs data to SD card in `.csv` format
- Captures:
  - 9-axis IMU data (acceleration, gyroscope, magnetometer)
  - Barometric pressure, altitude, and temperature
  - GPS coordinates, altitude, speed, and satellite count
- OLED display for live status updates
- Live telemetry Using 3G SIM card
- Modular and extensible design for future sensor integration

## 🧠 Core Components

| Component                  | Description |
|---------------------------|-------------|
| **Arduino**               | Primary microcontroller (Nano Ble 33 Sense REV2) |
| **BMI270_BMM150**         | 9-DOF IMU (acceleration, gyroscope, magnetometer) |
| **LPS22HB**               | Barometric pressure sensor |
| **TinyGPS++**             | GPS data parser |
| **SH1106 OLED**           | I2C 128x64 screen for live feedback |
| **SD Card Module**        | Stores `.csv` data logs |
| **Voltage Divider**       | Scales down battery voltage for safe ADC reading |

---

## 🔧 Circuit Diagram
needs to replace resistor 430 with 510 ohms

[Wiring diagram](Datenlogger_Files/Datalogger_schematic v2.pdf)

---

## 🔋 ADC Voltage Divider

To safely read battery voltage levels using the Arduino's 3.3V ADC pin, use the following voltage divider:

- **Input Voltage**: Up to 4.2V (LiPo 1S max)
- **Divider Resistors**:
  - R1: **1.8kΩ**
  - R2: **510**

### 📐 Voltage Divider Calculation

Vout = Vin × (1 - (R2 / (R1 + R2))
= 4.2 × (1- (510 / (1800 + 510)) ≈ 3.272V


---

## 📂 Data Format

Logged data is stored as a CSV file (`log.csv`) with the following headers:

timestamp,accX,accY,accZ,gyroX,gyroY,gyroZ,magX,magY,magZ,
latitude,longitude,gpsAltitude,Speed,SatCount,roll,pitch,yaw,
pressure,temperature,paltitude


Example row:
1542,-0.01,0.02,9.81,0.01,-0.02,0.00,30.12,11.23,-45.66,
52.5200,13.4050,120.5,10.4,9,0,0,0,100.3,25.5,132.2


---

## 🛠️ Getting Started

### 🔌 Wiring

- Connect sensors and modules via I2C/SPI/Serial as per the diagram.
- Ensure GPS is connected to `Serial1` (typically `TX`/`RX` on Arduino). (IMPORTANT)
- Insert a formatted **FAT32 SD card**.

### 🧪 Uploading

1. Install dependencies:
   - `Arduino_BMI270_BMM150`
   - `Arduino_LPS22HB`
   - `TinyGPSPlus`
   - `U8g2`
2. Compile and upload the sketch to your board.
3. Open the Serial Monitor at **115200 baud** to see real-time debug info.

---

## 📊 OLED Display

The OLED shows system status:
- Sensor initialization
- GPS satellites count
- Errors during SD or sensor setup
- Team number

> To keep the display clear and reduce power, update messages are brief and limited to startup and GPS updates.

---

## 🧩 Planned Improvements

- Add orientation calculation (roll, pitch, yaw)
- Support logging at configurable rates (5Hz, 10Hz)
- Add battery voltage and current sensors
- Add SIM card for real time telemetry

---

## 👨‍💻 Credits

- Developed by **Neus Fliegen Avionics Team**
- Based on libraries by Arduino, Adafruit, and Mikal Hart (TinyGPS++)

---

## 📷 License

- currently internal private use only!

---

