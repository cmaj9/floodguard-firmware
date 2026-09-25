# FloodGuard Firmware (IoT Telemetry Node System)

เฟิร์มแวร์สำหรับระบบตรวจวัดระดับน้ำและแจ้งเตือนภัยน้ำท่วม **FloodGuard** โดยออกแบบการทำงานเป็นสถาปัตยกรรม **Master-Subnode Architecture** ที่เชื่อมต่อข้อมูลผ่านสายสัญญาณมาตรฐานอุตสาหกรรม **RS485** และส่งข้อมูลระยะไกลขึ้นคลาวด์ผ่าน **LoRaWAN (OTAA)**

---

## 📐 สถาปัตยกรรมระบบ (System Architecture)

```text
               +-------------------------------------------+
               |             FloodGuard System             |
               +-------------------------------------------+
                                     │
      RS485 (Industrial Serial)      │ LoRaWAN (AS923 / OTAA)
   ┌─────────────────────────────┐   │
   ▼                             ▼   ▼
┌──────────────────┐          ┌──────────────────────────────────┐
│  sensor-subnode  │ ──RS485─►│           master-node            │ ──LoRaWAN─► Cloud / TTN
│    (ESP32-C3)    │          │     (Heltec WiFi LoRa 32 V3)     │
└──────────────────┘          └──────────────────────────────────┘
  - Ultrasonic (A01NYUB)        - LoRaWAN OTAA Transmitter
  - Gyro / Tilt (GY-25)         - GPS (Epoch Timestamping)
  - RS485 Slave Transmitter     - Flash Backlog Storage (LittleFS)
                                - SHT30 Temp / Humidity
                                - OLED Display & Battery Monitor
                                - External Power Switch (Relay Pin 26)
```

---

## 📁 โครงสร้างโปรเจกต์ (Repository Structure)

```text
floodguard-firmware/
├── .gitignore
├── README.md
│
├── master-node/                # โหนดแม่: Heltec WiFi LoRa 32 V3
│   ├── platformio.ini          # ค่าคอนฟิกบอร์ด, LoRaWAN flag, libraries
│   ├── src/main.cpp            # โค้ดหลัก (LoRaWAN State Machine, Backlog, RS485 Bridge)
│   └── include/
│       ├── credentials.h.example  # Template สำหรับระบุ LoRaWAN Keys
│       └── credentials.h          # [Local only] ไฟล์เก็บ Keys จริง (ไม่ถูก commit ขึ้น Git)
│
└── sensor-subnode/             # โหนดลูก: ESP32-C3
    ├── platformio.ini          # ค่าคอนฟิก ESP32-C3
    └── src/main.cpp            # โค้ดอ่าน Ultrasonic A01NYUB + Gyro GY-25 ส่งผ่าน RS485
```

---

## ⚡ รายละเอียดของแต่ละโหนด

### 1. `sensor-subnode` (ESP32-C3 - โหนดลูก)
- **วัดระดับน้ำ (Ultrasonic A01NYUB)**: อ่านระยะห่างผิวน้ำผ่าน UART1 พร้อมระบบตรวจสอบ Checksum
- **วัดความเอียง (GY-25)**: อ่านค่าแกน X, Y เพื่อตรวจสอบองศาการเอียงของทุ่น/เสาวัด (`OK`, `WARN`, `TILT`)
- **การส่งข้อมูล (RS485)**: รับคำขอ (`REQ`) จากโหนดแม่ และส่งชุดข้อมูล Packet (`RawLevel, GyroX, GyroY, TiltStatus`) กลับพร้อมรอรับ `ACK`

### 2. `master-node` (Heltec WiFi LoRa 32 V3 - โหนดแม่)
- **LoRaWAN OTAA (AS923)**: ส่งข้อมูลแบบ Confirmed Uplink พร้อมระบบ Auto Retry สูงสุด 2 ครั้ง ล็อก DataRate ที่ DR 2 (SF10) เพื่อความเสถียร
- **GPS Timestamp (TinyGPSPlus)**: ดึงเวลาสากล Unix Epoch จาก NMEA RMC ฝังลงใน Payload
- **Flash Backlog (LittleFS)**: จัดเก็บข้อมูลประวัติขนาด 26 ไบต์ลง Flash ทันทีหากออฟไลน์หรือเครือข่ายขัดข้อง และจะทยอยส่งออกเมื่อระบบกลับมาเชื่อมต่อได้
- **เซนเซอร์สภาพแวดล้อมและพลังงาน**: SHT30 (อุณหภูมิ/ความชื้น) และวงจรวัดโวลต์แบตเตอรี่พร้อม Median + EMA Filter
- **ระบบประหยัดพลังงาน**: ควบคุมเปิด-ปิดไฟเลี้ยงวงจรภายนอกและหน้าจอผ่าน Relay (Pin 26)

---

## 🚀 การติดตั้งและพัฒนา (Getting Started)

### 1. เครื่องมือที่ต้องใช้
- [Visual Studio Code](https://code.visualstudio.com/)
- [PlatformIO IDE Extension](https://platformio.org/)

### 2. ตั้งค่า LoRaWAN Credentials
ก่อนจะ Compile บอร์ด `master-node` ให้สร้างไฟล์ `credentials.h` จาก Template:

1. คัดลอกไฟล์เทมเพลต:
   ```bash
   cp master-node/include/credentials.h.example master-node/include/credentials.h
   ```
2. แก้ไขไฟล์ `master-node/include/credentials.h` โดยใส่คีย์จาก The Things Network (TTN) หรือ Network Server ของคุณ:
   ```cpp
   uint8_t devEui[] = { 0x.. };
   uint8_t appEui[] = { 0x.. };
   uint8_t appKey[] = { 0x.. };
   ```

### 3. คำสั่ง Build & Flash Firmware
คุณสามารถคลิกปุ่มบน PlatformIO Toolbar หรือใช้คำสั่ง Terminal:

- **Flash โหนดลูก (sensor-subnode)**:
  ```bash
  pio run -d sensor-subnode -t upload
  ```
- **Flash โหนดแม่ (master-node)**:
  ```bash
  pio run -d master-node -t upload
  ```
