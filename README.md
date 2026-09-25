# FloodGuard Firmware

เฟิร์มแวร์ระบบตรวจวัดระดับน้ำ FloodGuard เชื่อมต่อระหว่างโหนดลูกและโหนดแม่ผ่านสายสัญญาณ RS485 และส่งข้อมูลไปยังระบบคลาวด์ด้วย LoRaWAN OTAA

---

## System Overview

```text
┌─────────────────┐           ┌─────────────────────────┐
│ sensor-subnode  │ ──RS485──►│       master-node       │ ──LoRaWAN──► Cloud / TTN
│   (ESP32-C3)    │           │ (Heltec WiFi LoRa 32 V3)│
└─────────────────┘           └─────────────────────────┘
  - Ultrasonic                  - LoRaWAN OTAA
  - Gyro / Tilt                 - GPS Unix Epoch
                                - Flash Backlog
                                - SHT30 Temp / Humidity
```

---

## Project Structure

```text
floodguard-firmware/
├── master-node/        # โหนดแม่ Heltec LoRa32 V3
└── sensor-subnode/     # โหนดลูก ESP32-C3
```

---

## Setup & Build

1. คัดลอกไฟล์คอนฟิกคีย์ก่อนเริ่ม build
```bash
cp master-node/include/credentials.h.example master-node/include/credentials.h
```

2. คำสั่ง build และ flash ผ่าน PlatformIO
```bash
pio run -d sensor-subnode -t upload
pio run -d master-node -t upload
```
