#include <Arduino.h>
#include <SoftwareSerial.h>

// ================= PIN DEFINITIONS =================
// A01NYUB & GY-25 ใช้ UART1 สลับกัน
#define SENSOR_RX_PIN   4    // A01NYUB TX -> ESP32 RX
#define GY25_RX_PIN     5    // GY25 TX -> ESP32 RX
#define GY25_TX_PIN     6    // ESP32 TX -> GY25 RX

// Heltec ใช้ SoftwareSerial
#define HELTEC_RX_PIN   3
#define HELTEC_TX_PIN   7
#define HELTEC_BAUD     9600
#define ACK_TIMEOUT_MS  1000

HardwareSerial sensorBus(1);          // UART1 - สลับใช้ A01NYUB / GY25
SoftwareSerial HeltecSerial;          // ส่งไป Heltec

// ================= GLOBAL VARIABLES =================
float currentLevel    = 0.0;
float correctedLevel  = 0.0;
float gyroX = 0, gyroY = 0;
float totalTilt = 0;
String tiltStatus = "OK";

// GY-25 buffer
uint8_t gy25Buffer[8];
uint8_t gy25Counter = 0;

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  HeltecSerial.begin(HELTEC_BAUD, SWSERIAL_8N1, HELTEC_RX_PIN, HELTEC_TX_PIN, false);
  Serial.println("=== SETUP DONE ===");
}

// ================= READ A01NYUB ULTRASONIC (UART1) =================
bool readA01NYUB() {
  sensorBus.begin(9600, SERIAL_8N1, SENSOR_RX_PIN, -1);

  while (sensorBus.available()) sensorBus.read();

  unsigned long startWait = millis();
  bool success = false;

  while (millis() - startWait < 500) {
    if (sensorBus.available() > 0) {
      int b = sensorBus.read();
      if (b == 0xFF) {
        uint8_t frame[4];
        frame[0] = 0xFF;
        unsigned long startFrame = millis();
        while (sensorBus.available() < 3) {
          if (millis() - startFrame > 50) break;
        }
        if (sensorBus.available() >= 3) {
          frame[1] = sensorBus.read();
          frame[2] = sensorBus.read();
          frame[3] = sensorBus.read();

          uint8_t checksum = (frame[0] + frame[1] + frame[2]) & 0xFF;
          if (checksum == frame[3]) {
            uint16_t distanceMM = ((uint16_t)frame[1] << 8) | frame[2];
            if (distanceMM > 280) {
              currentLevel = distanceMM / 10.0;
              success = true;
              break;
            } else {
              // Ultrasonic distance below min range (< 280mm): report 0.0
              Serial.println("A01NYUB: TOO CLOSE -> ส่ง 0");
              currentLevel = 0.0;
              success = true;
              break;
            }
          }
        }
      }
    }
    delay(1);
  }

  if (!success) {
    Serial.println("A01NYUB: TIMEOUT");
    currentLevel = 0.0;  // Timeout fallback: report 0.0
  }

  sensorBus.end();
  return success;
}

// ================= READ GY-25 (UART1) =================
void readGY25() {
  while (sensorBus.available()) {
    gy25Buffer[gy25Counter] = (unsigned char)sensorBus.read();

    if (gy25Counter == 0 && gy25Buffer[0] != 0xAA) {
      return;
    }

    gy25Counter++;

    if (gy25Counter == 8) {
      gy25Counter = 0;
      if (gy25Buffer[0] == 0xAA && gy25Buffer[7] == 0x55) {
        gyroY = (int16_t)(gy25Buffer[3] << 8 | gy25Buffer[4]) / 100.0;
        gyroX = (int16_t)(gy25Buffer[5] << 8 | gy25Buffer[6]) / 100.0;
      }
    }
  }
}

void readGY25Data() {
  sensorBus.begin(9600, SERIAL_8N1, GY25_RX_PIN, GY25_TX_PIN);
  delay(100);

  while (sensorBus.available()) sensorBus.read(); // flush
  gy25Counter = 0;

  unsigned long gyroStart = millis();
  while (millis() - gyroStart < 500) {
    readGY25();
    delay(1);
  }

  sensorBus.end();
}

// ================= READ ALL SENSORS =================
void readSensorsQuick() {
  // ---- 1. Ultrasonic (UART1) ----
  if (readA01NYUB()) {
    Serial.printf("Lvl: %.1f cm\n", currentLevel);
  }

  // ---- 2. GY-25 (UART1, สลับมา) ----
  readGY25Data();

  // ---- 3. Tilt status (คำนวณทั้ง totalTilt และ tiltStatus สำหรับใช้ในการส่ง) ----
  totalTilt = sqrt(gyroX * gyroX + gyroY * gyroY);

  if (totalTilt <= 5.0f) tiltStatus = "OK";
  else if (totalTilt <= 10.0f) tiltStatus = "WARN";
  else tiltStatus = "TILT";

  // Send raw level directly; tilt correction handled downstream if needed
}

// ================= PRINT TO TERMINAL =================
void printSensorData() {
  Serial.println("===== SENSOR DATA =====");
  Serial.printf("LEVEL  -> Raw: %.1f cm  Corrected: %.1f cm\n", currentLevel, correctedLevel);
  Serial.printf("TILT   -> X: %.1f  Y: %.1f  Total: %.1f  Status: %s\n",
                gyroX, gyroY, totalTilt, tiltStatus.c_str());
  Serial.println("========================");
}

// ================= BUILD & SEND PAYLOAD =================
String buildPayload() {
  String payload = "";
  // Send raw level
  payload += String(currentLevel, 1) + ",";
  payload += String(gyroX, 1) + ",";
  payload += String(gyroY, 1) + ",";
  payload += tiltStatus;
  return payload;
}

bool sendPayloadToHeltec() {
  String payload = buildPayload();

  while (HeltecSerial.available()) HeltecSerial.read();

  HeltecSerial.println(payload);
  Serial.println("TX -> " + payload);

  unsigned long start = millis();
  String resp = "";
  while (millis() - start < ACK_TIMEOUT_MS) {
    if (HeltecSerial.available()) {
      char c = HeltecSerial.read();
      if (c == '\n') break;
      resp += c;
    }
  }

  if (resp.indexOf("ACK") >= 0) {
    Serial.println("Heltec: ส่งสำเร็จ (ACK received)");
    return true;
  } else {
    Serial.println("Heltec: ส่งไม่สำเร็จ (No ACK / Timeout)");
    return false;
  }
}

// ================= MAIN LOOP =================
void loop() {

  // ── รับ REQ จาก Serial Monitor (ทดสอบ) ──
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.replace("\r", "");
    cmd.trim();
    Serial.println("CMD: [" + cmd + "]");
    if (cmd == "REQ") {
      Serial.println("[DEBUG] REQ from Serial Monitor");
      readSensorsQuick();
      printSensorData();
    }
  }

  // ── รับ REQ จาก Heltec ──
  if (HeltecSerial.available()) {
    String cmd = HeltecSerial.readStringUntil('\n');
    cmd.replace("\r", "");
    cmd.trim();
    Serial.println("[HELTEC CMD]: " + cmd);

    if (cmd == "REQ") {
      Serial.println("REQ received → reading sensors...");
      readSensorsQuick();
      printSensorData();

      String payload = buildPayload();
      HeltecSerial.println(payload);
      Serial.println("TX -> " + payload);

      unsigned long start = millis();
      bool gotAck = false;
      while (millis() - start < 1000) {
        if (HeltecSerial.available()) {
          String resp = HeltecSerial.readStringUntil('\n');
          resp.trim();
          if (resp == "ACK") {
            Serial.println("Heltec: ส่งสำเร็จ");
            gotAck = true;
            break;
          }
        }
        delay(1);
      }
      if (!gotAck) Serial.println("Heltec: ไม่ได้รับ ACK");
    }
  }

  // ── อ่าน sensor + แสดงผลอัตโนมัติ ──
  static unsigned long lastRead = 0;
  if (millis() - lastRead > 2000) {
    lastRead = millis();
    Serial.println("--- Auto read ---");
    readSensorsQuick();
    printSensorData();
  }
}
