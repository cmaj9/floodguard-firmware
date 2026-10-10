# FloodGuard Master Node: Comprehensive Technical Context & Architecture Documentation

เอกสารฉบับนี้รวบรวมบริบท รายละเอียดทางเทคนิค สถาปัตยกรรมฮาร์ดแวร์ ซอฟต์แวร์ โปรโตคอลการสื่อสาร และอัลกอริทึมทั้งหมดของ **FloodGuard Master Node** (รวมถึงส่วนเชื่อมโยงกับ Subnode, LoRaWAN Network และ Backend)

---

## 1. ภาพรวมสถาปัตยกรรมระบบ (System Architecture)

ระบบฝั่งภาคสนาม (Field Node) ออกแบบเป็น **สถาปัตยกรรม 2 ชิป (Dual-MCU System)** เพื่อแยกหน้าที่ประมวลผลและการสื่อสารทางไกล:

```
+-------------------------------------------------------------+
|                      Sensor Subnode                         |
|                       (ESP32-C3)                            |
|  - อัลตราโซนิก A01NYUB (วัดระยะผิวน้ำ)                      |
|  - MPU6050 Gyro/Accelerometer (วัดมุมเอียง X, Y)             |
+------------------------------+------------------------------+
                               |
                               | UART Bridge (9600 baud)
                               | REQ -> Data -> ACK
                               v
+-------------------------------------------------------------+
|                       Master Node                           |
|               (Heltec WiFi LoRa 32 V3)                      |
|  - ESP32-S3 Dual Core + Semtech SX1262 LoRa                |
|  - GPS (Neo-6M/8M): พิกัด Lat/Lng + NMEA RMC UTC Epoch      |
|  - SHT30: อุณหภูมิ / ความชื้นสัมพัทธ์                        |
|  - LiFePO4 4S 12.8V: 3-Layer SoC Stabilization (RTC SRAM)    |
|  - SSD1306 OLED (128x64): แสดงสถานะ                         |
|  - LittleFS Flash Backlog: 720 packets FIFO Safe            |
|  - Power Management: MOSFET Relay 26 + Deep Sleep 9.25 นาที  |
+------------------------------+------------------------------+
                               |
                               | LoRaWAN Confirmed Uplink
                               | AS923-TH (923.4 MHz, DR 2 / SF10)
                               v
+-------------------------------------------------------------+
|                     LoRaWAN Gateway                         |
+------------------------------+------------------------------+
                               |
                               v
+-------------------------------------------------------------+
|                   ChirpStack Network Server                 |
+------------------------------+------------------------------+
                               |
                               v
+-------------------------------------------------------------+
|               Railway Backend & Database                    |
|  - REST API & MQTT Integration                              |
|  - PostgreSQL Database                                      |
+------------------------------+------------------------------+
                               |
                               v
+-------------------------------------------------------------+
|                   Frontend Web Dashboard                    |
|  - React + TypeScript + Tailwind CSS                        |
+-------------------------------------------------------------+
```

---

## 2. การเชื่อมต่อฮาร์ดแวร์และ Pinout ของ Master Node

| ขา GPIO | หน้าที่ / อุปกรณ์ที่เชื่อมต่อ | รายละเอียดการทำงาน |
| :---: | :--- | :--- |
| **GPIO 26** | **MOSFET Power Relay (`RELAY_PIN`)** | สวิตช์หลักคุมไฟเลี้ยง: `HIGH` = จ่ายไฟให้เซนเซอร์และวงจรภายนอกทั้งหมด, `LOW` = ตัดไฟเพื่อเข้า Deep Sleep |
| **GPIO 2** | **ADC Battery (`CUSTOM_BAT_PIN`)** | ต่อเข้า Voltage Divider ($R_1 = 100\text{k}\Omega, R_2 = 20\text{k}\Omega$, Divider Ratio = 6.0) วัดแรงดันแบตเตอรี่ LiFePO4 4S |
| **GPIO 36** | **Vext Rail (`VEXT_PIN`)** | รางจ่ายไฟ Heltec สำหรับโมดูล I2C / อุปกรณ์ต่อพ่วงภายนอก |
| **GPIO 21** | **OLED Reset (`OLED_RESET`)** | ขารีเซ็ตจอแสดงผล SSD1306 OLED (128x64) บน I2C บัส `Wire` (SDA: 17, SCL: 18) |
| **GPIO 0** | **ปุ่ม PRG (`BUTTON_PIN`)** | ปุ่มกดหน้าบอร์ด: ใช้ปลุกจอ OLED ชั่วคราว และปลุกชิปจาก Deep Sleep ผ่าน Hardware RTC Ext0 Wakeup |
| **GPIO 19, 20**| **Hardware Serial 1 (`GPS_RX`, `GPS_TX`)** | เชื่อมต่อโมดูล GPS (Neo-6M/Neo-8M) ที่ความเร็ว 9600 baud ดึงพิกัดและเวลาสากล $GPRMC |
| **GPIO 47, 48**| **Software/Secondary I2C (`I2CSHT`)** | เชื่อมต่อเซนเซอร์อุณหภูมิและความชื้น **SHT30** (Address `0x44`) |
| **GPIO 41, 42**| **Hardware Serial 2 (`ESP32Serial`)** | ลิงก์ UART สื่อสารกับบอร์ดย่อย ESP32-C3 Subnode (Baudrate 9600) |
| **GPIO 4, 5** | **LED Indicators** | GPIO 4 = LED แดง (Active Phase / กำลังทำงาน), GPIO 5 = LED เขียว (สถานะส่ง/ได้รับ ACK) |

---

## 3. โปรโตคอลการสื่อสาร Master $\leftrightarrow$ Subnode (ESP32-C3 Bridge)

การแลกเปลี่ยนข้อมูลผ่าน UART2 ทำงานในรูปแบบ **Request-Response แบบมี Handshake ปลอดภัย**:
* **การร้องขอ:** Master Node ส่งคำว่า `REQ\n`
* **การส่งข้อมูลจาก Subnode:** Subnode ส่งสตริงในรูปแบบ:
  ```text
  <Level>,<GyroX>,<GyroY>,<TiltStatus>\n
  ```
  *(ตัวอย่าง: `164.0,-0.8,-7.8,OK\n`)*
* **การตอบรับ:** Master ตอบกลับ `ACK\n` เมื่อสามารถ Parse ข้อมูลได้ถูกต้อง
* **Tilt Angle Correction:** Master จะนำมุมเอียงมาคำนวณปรับแก้ระดับน้ำทันที:
  $$Level_{corrected} = Level_{raw} \times \cos\left(\sqrt{\text{GyroX}^2 + \text{GyroY}^2}\right)$$
  *(ป้องกันปัญหาระดับน้ำเพี้ยนเมื่อเกิดลมแรง เสาเอียง หรือกระแสน้ำพัด)*

---

## 4. โครงสร้างข้อมูล LoRaWAN Payload (ขนาดคงที่ 26 Bytes)

แพ็กเก็ตข้อมูลที่ Master ส่งขึ้น ChirpStack ถูกบีบอัดเป็น Binary Big-Endian ดังนี้:

| ไบต์ที่ | ชนิดข้อมูล | ตัวแปร | ตัวคูณ / สเกล | รายละเอียด |
| :---: | :---: | :--- | :---: | :--- |
| `0` | `uint8_t` | `station_id` | - | รหัสสถานี (เช่น `1` สำหรับ `ST-001`) |
| `1 - 2` | `uint16_t` | `water_level` | $\times 10$ | ระดับน้ำ (เซนติเมตร) |
| `3 - 6` | `int32_t` | `latitude` | $\times 1,000,000$ | ละติจูด (องศา) |
| `7 - 10`| `int32_t` | `longitude` | $\times 1,000,000$ | ลองจิจูด (องศา) |
| `11 - 12`| `uint16_t` | `battery_voltage`| $\times 100$ | แรงดันแบตเตอรี่ (โวลต์) |
| `13` | `uint8_t` | `battery_percent`| - | เปอร์เซ็นต์แบตเตอรี่ ($0 - 100\%$) |
| `14 - 15`| `int16_t` | `temperature` | $\times 10$ | อุณหภูมิ (°C) |
| `16 - 17`| `uint16_t` | `humidity` | $\times 10$ | ความชื้นสัมพัทธ์ ($\%$) |
| `18 - 19`| `int16_t` | `gyro_x` | $\times 100$ | มุมเอียงแกน X (องศา) |
| `20 - 21`| `int16_t` | `gyro_y` | $\times 100$ | มุมเอียงแกน Y (องศา) |
| `22 - 25`| `uint32_t` | `epoch` | - | **UNIX Timestamp (GPS Epoch)** จาก NMEA $GPRMC รับประกันว่าข้อมูลตกค้างจะมีเวลาตรวจวัดแท้จริงติดไปด้วยเสมอ |

---

## 5. วงรอบพลังงานและระบบ Deep Sleep (Power & Timing Cycle)

* **ความถี่รอบการทำงาน (Transmission Duty Cycle):** **ทุกๆ 10 นาที (600 วินาที)**
* **ช่วงทำงาน (Active Phase - 45 วินาที):**
  * เปิด MOSFET Relay ขา 26 จ่ายไฟให้ Subnode, GPS, SHT30
  * เปิดจอ OLED และรัน `sensorTask` (Core 0) โพลล์ค่าเซนเซอร์ทุก 5 วินาที
  * ส่ง LoRaWAN Uplink แบบ Confirmed (รอ ACK)
* **ช่วงจำศีล (Deep Sleep Phase - 9.25 นาที / 555 วินาที):**
  * ตัดไฟ MOSFET Relay ขา 26 (ดับเซนเซอร์ทั้งหมด ป้องกันไฟรั่ว)
  * ปิดจอภาพ OLED (`SSD1306_DISPLAYOFF`), ดับไฟ LED ทุกดวง
  * เปิดใช้งาน Hardware Interrupt Wakeup:
    ```cpp
    esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_PIN, 0);
    Mcu.addwakeio((uint8_t)BUTTON_PIN);
    ```
    เพื่อให้ปุ่ม PRG (GPIO 0) สามารถกดปลุกจอและจ่ายไฟชั่วคราวได้ทันทีแม้ชิปหลับอยู่
  * เข้าสู่โหมด Deep Sleep ใช้พลังงานในระดับไมโครแอมป์ ($\mu\text{A}$) ทำให้แบตเตอรี่ LiFePO4 สามารถเลี้ยงระบบได้นานต่อเนื่องแม้ไม่มีแดดชาร์จนานหลายสัปดาห์

---

## 6. การตั้งค่า LoRaWAN และระบบ Flash Backlog ป้องกันข้อมูลสูญหาย

1. **การล็อค DataRate ป้องกัน DOWNLINK_GATEWAY TOO_EARLY:**
   * บังคับให้ระบบใช้ **DR 2 (SF10 / BW 125 kHz)** บนความถี่ AS923 Channel 0 (923.4 MHz) แบบถาวร เพื่อให้ช่วงเวลาเปิดหน้าต่าง RX1/RX2 ของชิปตรงกับระยะเวลาของ Gateway
2. **Confirmed Uplink with Retry:**
   * ส่งข้อมูลแบบ Confirmed Uplink รอ ACK สูงสุด 10 วินาที หากไม่ได้ ACK จะลองส่งซ้ำ (Retry) สูงสุด 2 ครั้ง
3. **Atomic Per-Packet Flash Drain (FIFO Safe):**
   * หากขาดการเชื่อมต่อหรือ Gateway ออฟไลน์ ข้อมูลจะถูกเขียนลง LittleFS (`/backlog.bin`) ทันที (รองรับได้สูงสุด 720 แพ็กเก็ต หรือราว 5 วันเต็ม)
   * เมื่อ Gateway กลับมาออนไลน์ ระบบจะทยอยส่งระบาย Backlog จากเก่าสุดไปใหม่สุดทีละแพ็กเก็ต (เว้นช่วง 3 วินาที)
   * **ทุกครั้งที่แพ็กเก็ตใดได้รับ ACK สำเร็จ จะตัดยอดออกจาก Flash ทันที (Atomic Commit)** ป้องกันการส่งข้อมูลซ้ำกรณีสัญญาณขาดตอนกลางคัน

---

## 7. ระบบคำนวณและกรองแบตเตอรี่ (3-Layer Stabilization - v1.0.6)

ออกแบบเฉพาะสำหรับ **แบตเตอรี่ LiFePO4 4S (12.8V)** ซึ่งมีคุณสมบัติ Flat Discharge Plateau แคบมาก ($13.05\text{V} - 13.20\text{V}$):

### OCV Piecewise Curve Refinement
```cpp
float calculateLiFePO4Percent(float v) {
    if (v >= 13.60f) return 100.0f; // ช่วงชาร์จเต็ม หรือมีแดดชาร์จ (13.6V - 14.4V)
    if (v >= 13.40f) return 98.0f + (v - 13.40f) * (2.0f / 0.20f);  // 13.40V = 98% (Resting Full หลังหมดแดด)
    if (v >= 13.30f) return 90.0f + (v - 13.30f) * (8.0f / 0.10f);  // 13.30V = 90%
    if (v >= 13.20f) return 80.0f + (v - 13.20f) * (10.0f / 0.10f); // 13.20V = 80%
    if (v >= 13.15f) return 70.0f + (v - 13.15f) * (10.0f / 0.05f); // 13.15V = 70%
    if (v >= 13.10f) return 60.0f + (v - 13.10f) * (10.0f / 0.05f); // 13.10V = 60% (เกลี่ย Plateau)
    if (v >= 13.05f) return 50.0f + (v - 13.05f) * (10.0f / 0.05f); // 13.05V = 50%
    if (v >= 12.95f) return 35.0f + (v - 12.95f) * (15.0f / 0.10f); // 12.95V = 35%
    if (v >= 12.85f) return 20.0f + (v - 12.85f) * (15.0f / 0.10f); // 12.85V = 20%
    if (v >= 12.50f) return 10.0f + (v - 12.50f) * (10.0f / 0.35f); // 12.50V = 10%
    if (v >= 12.00f) return 5.0f  + (v - 12.00f) * (5.0f / 0.50f);  // 12.00V = 5%
    if (v >= 11.20f) return 0.0f  + (v - 11.20f) * (5.0f / 0.80f);  // 11.20V = 0% (BMS Cutoff)
    return 0.0f;
}
```

### การทำงาน 3 ชั้น (3 Layers):
1. **Layer 0 (Hardware & RTC Fast SRAM):**
   * วัดแรงดัน **No-Load OCV ก่อนเปิด Relay 26** ใน `setup()` และรอบ Join
   * หากวัดขณะ Relay เปิดอยู่ จะชดเชยค่าแรงดันตกคร่อม (Load Sag) $+0.08\text{V}$
   * จัดเก็บค่าเปอร์เซ็นต์คงที่ไว้ใน `RTC_DATA_ATTR float rtcLastStableBattPct` เพื่อรักษาค่าข้ามรอบ Deep Sleep 9.25 นาทีโดยไม่ต้องเขียน Flash ซ้ำๆ
2. **Layer 1 (Solar Charging Detection):**
   * เมื่อวัดแรงดันได้ $\ge 13.45\text{V}$ ระบบจะถือว่าโซลาร์เซลล์กำลังอัดประจุจริง และยอมให้เปอร์เซ็นต์เพิ่มขึ้นอย่างนุ่มนวลผ่าน IIR Filter ($70:30$)
3. **Layer 2 (Monotonic Clamping):**
   * ในเวลากลางคืนหรือไม่มีแดด ($V < 13.45\text{V}$) บังคับใช้กฎห้ามตัวเลขเปอร์เซ็นต์เด้งกลับขึ้นมาเด็ดขาด 100%
4. **Layer 3 (Dynamic Rate Limiter):**
   * จำกัดการลดลงสูงสุดไม่เกิน **$2.0\%$ ต่อรอบ 10 นาที** เพื่อซับการคายประจุแรงดันผิว (Surface Charge Decay) หลังหมดแดด
   * **Critical Safety Bypass:** หากแบตเตอรี่ลดต่ำกว่า $12.50\text{V}$ ระบบจะปลดล็อคตัวหน่วงและลดลงตามจริงทันทีเพื่อส่งสัญญาณเตือนภัยฉุกเฉิน

---

## 8. ประวัติ Version Tags ใน Git Repository

* **`v1.0.2`:** ปรับรอบการทำงานเป็น 10 นาที, เพิ่ม OCV Piecewise Curve, รองรับ Continuous Backlog Drain และ Dynamic Timestamp
* **`v1.0.3`:** แก้ไขบั๊ก Heltec Library กระโดดไป DR 5 ด้วยการล็อค DR 2 ถาวร และเปลี่ยนคิวเป็น Early Flash Append (FIFO Safe)
* **`v1.0.4`:** เพิ่มฮาร์ดแวร์ RTC `ext0` Wakeup และ ISR ทำให้ปุ่ม PRG (GPIO 0) กดปลุกเครื่องได้ขณะ Deep Sleep
* **`v1.0.5`:** ระบบ Atomic Per-Packet Flash Commit ตัดยอดข้อมูลออกจาก Flash ทันทีทุกครั้งที่ได้รับ ACK ระหว่างระบาย Backlog
* **`v1.0.6` (เวอร์ชันล่าสุด):** อัลกอริทึม **3-Layer Battery SoC Stabilization** แก้ปัญหาเปอร์เซ็นต์แบตเตอรี่แกว่งในเวลากลางคืน พร้อมบันทึกสถานะข้ามรอบ Deep Sleep ด้วย RTC Fast SRAM
