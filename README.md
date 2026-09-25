# IoT Sensor Node (ESP32-C3 + Heltec LoRaWAN V3)

ระบบตรวจวัดระดับน้ำและมุมเอียง พร้อมส่งข้อมูลระยะไกลผ่านเครือข่าย LoRaWAN (OTAA) โดยทำงานร่วมกันระหว่างบอร์ดอ่านเซนเซอร์ (ESP32-C3) และบอร์ดส่งสัญญาณวิทยุ (Heltec WiFi LoRa 32 V3)

---

## 📐 โครงสร้างระบบ (Architecture)

```text
Project_Node/
├── esp32-c3/               # Firmware อ่านเซนเซอร์ (UART / SoftwareSerial)
│   ├── src/main.cpp        # Ultrasonic (A01NYUB) + Gyro/Tilt (GY-25)
│   └── platformio.ini
│
└── heltec-lora32/          # Firmware จัดการ LoRaWAN, GPS, Backlog, Display
    ├── src/main.cpp        # OTAA Join, LittleFS Flash Backlog, Power Control
    ├── include/
    │   ├── credentials.h.example  # Template สำหรับตั้งค่า LoRaWAN Keys
    │   └── credentials.h          # [Local only] ไฟล์เก็บ Keys จริง (ถูก ignore จาก Git)
    └── platformio.ini
```

---

## ⚡ หน้าที่ของแต่ละบอร์ด

### 1. `esp32-c3` (Sensor Acquisition Unit)
- **Ultrasonic (A01NYUB)**: อ่านระยะห่างผิวน้ำ/ระดับน้ำผ่าน UART1 (สลับ Bus)
- **Gyroscope/Inclinometer (GY-25)**: อ่านค่ามุมเอียงแกน X, Y เพื่อคำนวณสถานะความเอียง (OK / WARN / TILT)
- **Serial Bridge**: สื่อสารกับบอร์ด Heltec ผ่าน SoftwareSerial (คำสั่ง REQ / ข้อมูล Payload + ACK)

### 2. `heltec-lora32` (LoRaWAN Gateway Node / Transmitter)
- **LoRaWAN OTAA (AS923)**: ส่งข้อมูลแบบ Confirmed Uplink พร้อมระบบ Retry สูงสุด 2 ครั้ง
- **GPS (TinyGPSPlus)**: ดึงพิกัดตำแหน่งและ Unix Epoch Timestamp จาก NMEA RMC
- **SHT30**: ตรวจวัดอุณหภูมิและความชื้นสัมพัทธ์ในกล่องอุปกรณ์
- **Battery Monitor**: วัดแรงดันและเปอร์เซ็นต์แบตเตอรี่ผ่าน ADC
- **LittleFS Flash Backlog**: เก็บสะสมประวัติข้อมูลขนาด 26 ไบต์ลง Flash อัตโนมัติหากอยู่ในช่วง Offline หรือ Join ไม่ผ่าน และจะทยอยส่งออกเมื่อเชื่อมต่อได้
- **Power Management (Relay Pin 26)**: ควบคุมไฟเลี้ยงเซนเซอร์ภายนอกเพื่อประหยัดพลังงานในโหมด Sleep

---

## 🚀 การติดตั้งและใช้งาน (Getting Started)

### 1. เครื่องมือที่ต้องใช้
- [VS Code](https://code.visualstudio.com/)
- [PlatformIO IDE Extension](https://platformio.org/)

### 2. ตั้งค่า LoRaWAN Credentials (สำคัญมาก)
ไฟล์ `credentials.h` ที่มีคีย์จริงถูกยกเว้นจาก Git เพื่อความปลอดภัย ให้ทำตามขั้นตอนนี้:

1. คัดลอกไฟล์เทมเพลต:
   ```bash
   cp heltec-lora32/include/credentials.h.example heltec-lora32/include/credentials.h
   ```
2. เปิดไฟล์ `heltec-lora32/include/credentials.h` แล้วใส่คีย์ที่ได้จาก The Things Network (TTN) หรือ LoRaWAN Network Server:
   ```cpp
   uint8_t devEui[] = { 0x.. };
   uint8_t appEui[] = { 0x.. };
   uint8_t appKey[] = { 0x.. };
   ```

### 3. การ Build & Flash Firmware
ใช้คำสั่ง PlatformIO ผ่าน Terminal หรือปุ่มบน VS Code:

- **บอร์ด ESP32-C3**:
  ```bash
  pio run -d esp32-c3 -t upload
  ```
- **บอร์ด Heltec LoRa32**:
  ```bash
  pio run -d heltec-lora32 -t upload
  ```
