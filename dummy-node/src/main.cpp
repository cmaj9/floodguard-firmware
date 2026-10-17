/*
 * HELTEC V3 LoRaWAN - DUMMY SENSOR NODE
 * 
 * คุณสมบัติ:
 * 1. สุ่มค่าข้อมูลจำลอง (Dummy) ตามโครงสร้าง 26-Byte Payload เดิม 100%
 * 2. ส่ง LoRaWAN ทุกๆ 5 นาที (สามารถตั้งค่ารอบเวลาได้ง่ายที่ TX_INTERVAL_MS)
 * 3. ปุ่ม PRG (GPIO 0) กดเพื่อส่งข้อมูลทันที (Manual Trigger) พร้อม Cooldown 1 นาที (60 วินาที)
 *    - หากกดซ้ำก่อน 1 นาที จะไม่ส่งข้อมูล และแสดงจอเตือน Cooldown "WAIT XXs"
 *    - หากกดหลัง 1 นาที จะส่งข้อมูลทันที พร้อมรีเซ็ตเวลารอบส่ง Auto 5 นาทีใหม่
 * 4. จอแสดงผลระหว่างส่งข้อมูล (Visual Feedback):
 *    - แอนิเมชันเปิดม่านขยายจากกึ่งกลาง (Curtain / Box Expand) สไตล์ Sci-Fi
 *    - แสดงตัวโต "SENDING..." พร้อมไอคอนคลื่นวิทยุกระพริบ
 *    - เมื่อได้รับ ACK แสดง "ACK OK!" (หรือ "NO ACK!") ตัวโตค้างไว้ 3 วินาที
 *    - ปิดม่านเปลี่ยนกลับสู่หน้าจอ Dashboard แสดงค่าเซนเซอร์เดิม 100%
 * 5. ส่งแบบ Confirmed Uplink (รอ ACK จาก Gateway, ล็อก DR 2 / SF10, AS923)
 * 6. หน้าจอ OLED เปิดแสดงผลตลอดเวลา (Always-On)
 * 7. ไม่มีการ Sleep (ESP32 ทำงานต่อเนื่อง)
 * 8. ตัดส่วนฮาร์ดแวร์เซนเซอร์จริง, LittleFS Backlog, และ Deep Sleep ออกทั้งหมด
 */

#include "LoRaWan_APP.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

// ==============================================================================
// 1. การตั้งค่ารอบเวลา และโหนด (SETTINGS & CONFIGURATION)
// ==============================================================================
#define STATION_ID                4
#define TX_INTERVAL_MS            300000UL    // << รอบเวลาส่งอัตโนมัติ (5 นาที = 300000 ms)
#define MANUAL_TX_COOLDOWN_MS     60000UL     // << Cooldown ปุ่ม PRG (1 นาที = 60000 ms)
#define CONFIRMED_ACK_TIMEOUT_MS  5000UL      // เวลารอ ACK จาก Gateway สูงสุด (5 วินาที)

// ระบบซิงค์เวลาจริงอัตโนมัติจาก LoRaWAN Gateway ผ่านคำสั่ง MAC DeviceTimeReq (ไม่ใช้ Mockup)

// ==============================================================================
// 2. ช่วงค่าจำลองเซนเซอร์ (DUMMY SENSOR RANGES) - ปรับเปลี่ยนได้ที่นี่
// ==============================================================================
const float MOCK_LEVEL_MIN    = 50.0f;     // ระดับน้ำต่ำสุด (cm)
const float MOCK_LEVEL_MAX    = 300.0f;    // ระดับน้ำสูงสุด (cm)

const float MOCK_TEMP_MIN     = 25.0f;     // อุณหภูมิต่ำสุด (°C)
const float MOCK_TEMP_MAX     = 35.0f;     // อุณหภูมิสูงสุด (°C)

const float MOCK_HUM_MIN      = 40.0f;     // ความชื้นต่ำสุด (%)
const float MOCK_HUM_MAX      = 90.0f;     // ความชื้นสูงสุด (%)

const float MOCK_GYRO_MIN     = -15.0f;    // มุมเอียง Gyro X, Y ต่ำสุด (องศา)
const float MOCK_GYRO_MAX     = 15.0f;     // มุมเอียง Gyro X, Y สูงสุด (องศา)

const float MOCK_BAT_V_MIN    = 12.50f;    // แรงดันแบตเตอรี่ต่ำสุด (V)
const float MOCK_BAT_V_MAX    = 13.60f;    // แรงดันแบตเตอรี่สูงสุด (V)

// 1. กำหนดพิกัดศูนย์กลางให้ตรงกับจุดอ้างอิงของสถานี ST-004
const float MOCK_LAT_CENTER   = 14.035269f;  // << ใส่ Latitude จุดอ้างอิงของสถานีจริง
const float MOCK_LNG_CENTER   = 100.729226f; // << ใส่ Longitude จุดอ้างอิงของสถานีจริง

// 2. ปรับระยะแกว่งไม่เกิน 40 เมตร (แนะนำ 0.00025f หรือ 0.00020f)
const float MOCK_GPS_JITTER   = 0.00025f;  // << แกว่งแกนละประมาณ +/- 27 เมตร (ระยะรวมไม่เกิน 40 เมตรแน่นอน)

// ================= LoRaWAN Config (OTAA) =================
#if __has_include("credentials.h")
#include "credentials.h"
#else
// Placeholder credentials (Copy include/credentials.h.example to include/credentials.h and set your keys)
uint8_t devEui[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t appEui[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t appKey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t  nwkSKey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t  appSKey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint32_t devAddr   = (uint32_t)0x00000000;
#endif

uint16_t userChannelsMask[6] = {0x0002, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
DeviceClass_t   loraWanClass  = CLASS_A;

bool     overTheAirActivation = true;
bool     loraWanAdr           = false;
bool     isTxConfirmed        = true;      // Confirmed Uplink (รอ ACK จาก Gateway)
uint8_t  appPort              = 2;
uint8_t  confirmedNbTrials    = 1;
uint32_t appTxDutyCycle       = TX_INTERVAL_MS;
int8_t   loraWanDatarate      = DR_2;      // DR 2 (SF10) สำหรับ AS923
extern int8_t defaultDrForNoAdr;
extern int8_t currentDrForNoAdr;

// ==============================================================================
// 4. HARDWARE DEFINITIONS (HELTEC WIFI LORA 32 V3)
// ==============================================================================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    21
#define VEXT_PIN      36
#define BUTTON_PIN    0       // ปุ่ม PRG บนบอร์ด Heltec V3 (GPIO 0)
#define LED_RED_PIN   4
#define LED_GREEN_PIN 5

#define PAYLOAD_TOTAL_SIZE 26

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ==============================================================================
// 5. GLOBAL SENSOR VARIABLES, TIMERS & BUTTON
// ==============================================================================
float    currentLevel        = 100.0f;
float    currentLat          = MOCK_LAT_CENTER;
float    currentLng          = MOCK_LNG_CENTER;
int      batPercentage       = 100;
float    finalBatteryVoltage = 13.30f;
float    currentTemp         = 28.0f;
float    currentHum          = 65.0f;
float    gyroX               = 0.0f;
float    gyroY               = 0.0f;
String   tiltStatus          = "OK";
String   currentStatus       = "BOOT";

unsigned long cycleStartTime     = 0;
unsigned long lastDisplayUpdate  = 0;
unsigned long lastTxTime         = 0; // บันทึกเวลาที่ส่งสำเร็จครั้งล่าสุด (สำหรับ Cooldown)
bool          isJoinedNetwork    = false;

volatile bool prgPressed            = false;
volatile unsigned long lastPrgIntTime = 0;

volatile bool waitingForAck       = false;
volatile bool lastConfirmedAcked  = false;

// ISR สำหรับปุ่ม PRG (Debounce 400ms)
void IRAM_ATTR prgButtonISR() {
    unsigned long now = millis();
    if (now - lastPrgIntTime > 400) {
        prgPressed = true;
        lastPrgIntTime = now;
    }
}

// ==============================================================================
// 6. HELPER FUNCTIONS (POWER & BATTERY)
// ==============================================================================
void VextON() {
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, LOW); // จ่ายไฟเลี้ยงหน้าจอ OLED
}

// คำนวณ % แบตเตอรี่ LiFePO4 4S ตามกราฟ OCV จริง
float calculateLiFePO4Percent(float v) {
    if (v >= 13.60f) return 100.0f;
    if (v >= 13.40f) return 98.0f + (v - 13.40f) * (2.0f / 0.20f);
    if (v >= 13.30f) return 90.0f + (v - 13.30f) * (8.0f / 0.10f);
    if (v >= 13.20f) return 80.0f + (v - 13.20f) * (10.0f / 0.10f);
    if (v >= 13.15f) return 70.0f + (v - 13.15f) * (10.0f / 0.05f);
    if (v >= 13.10f) return 60.0f + (v - 13.10f) * (10.0f / 0.05f);
    if (v >= 13.05f) return 50.0f + (v - 13.05f) * (10.0f / 0.05f);
    if (v >= 12.95f) return 35.0f + (v - 12.95f) * (15.0f / 0.10f);
    if (v >= 12.85f) return 20.0f + (v - 12.85f) * (15.0f / 0.10f);
    if (v >= 12.50f) return 10.0f + (v - 12.50f) * (10.0f / 0.35f);
    if (v >= 12.00f) return 5.0f  + (v - 12.00f) * (5.0f / 0.50f);
    if (v >= 11.20f) return 0.0f  + (v - 11.20f) * (5.0f / 0.80f);
    return 0.0f;
}

// ==============================================================================
// 7. DUMMY DATA GENERATOR
// ==============================================================================
void generateDummyValues() {
    // 1. ระดับน้ำ (cm)
    long lvlMin10 = (long)(MOCK_LEVEL_MIN * 10);
    long lvlMax10 = (long)(MOCK_LEVEL_MAX * 10);
    currentLevel = (float)random(lvlMin10, lvlMax10 + 1) / 10.0f;

    // 2. อุณหภูมิ & ความชื้น
    long tMin10 = (long)(MOCK_TEMP_MIN * 10);
    long tMax10 = (long)(MOCK_TEMP_MAX * 10);
    currentTemp = (float)random(tMin10, tMax10 + 1) / 10.0f;

    long hMin10 = (long)(MOCK_HUM_MIN * 10);
    long hMax10 = (long)(MOCK_HUM_MAX * 10);
    currentHum  = (float)random(hMin10, hMax10 + 1) / 10.0f;

    // 3. Gyro X, Y
    long gMin100 = (long)(MOCK_GYRO_MIN * 100);
    long gMax100 = (long)(MOCK_GYRO_MAX * 100);
    gyroX = (float)random(gMin100, gMax100 + 1) / 100.0f;
    gyroY = (float)random(gMin100, gMax100 + 1) / 100.0f;

    float totalTilt = sqrt(gyroX * gyroX + gyroY * gyroY);
    tiltStatus = (totalTilt >= 10.0f) ? "TILT" : "OK";

    // 4. แบตเตอรี่
    long bMin100 = (long)(MOCK_BAT_V_MIN * 100);
    long bMax100 = (long)(MOCK_BAT_V_MAX * 100);
    finalBatteryVoltage = (float)random(bMin100, bMax100 + 1) / 100.0f;
    batPercentage = (int)(calculateLiFePO4Percent(finalBatteryVoltage) + 0.5f);

    // 5. พิกัด GPS พร้อม Jitter เล็กน้อย
    long jE6 = (long)(MOCK_GPS_JITTER * 1000000.0f);
    currentLat = MOCK_LAT_CENTER + ((float)random(-jE6, jE6 + 1) / 1000000.0f);
    currentLng = MOCK_LNG_CENTER + ((float)random(-jE6, jE6 + 1) / 1000000.0f);

    Serial.println(F("============================================="));
    Serial.printf("[DUMMY DATA GENERATED] Time: %lu s\n", (unsigned long)time(nullptr));
    Serial.printf("  Level: %.1f cm\n", currentLevel);
    Serial.printf("  Temp:  %.1f C | Hum: %.1f %%\n", currentTemp, currentHum);
    Serial.printf("  Gyro:  X:%.1f Y:%.1f (%s)\n", gyroX, gyroY, tiltStatus.c_str());
    Serial.printf("  Batt:  %.2f V (%d %%)\n", finalBatteryVoltage, batPercentage);
    Serial.printf("  GPS:   Lat:%.6f Lng:%.6f\n", currentLat, currentLng);
    Serial.println(F("============================================="));
}

// ==============================================================================
// 8. 26-BYTE PACKET BUILDER (ตรงตามโครงสร้างเดิมทุกประการ)
// ==============================================================================
void buildSensorPayload(uint8_t *out26) {
    uint16_t water_level = (uint16_t)(currentLevel * 10.0f);
    int32_t  latInt      = (int32_t)(currentLat * 1000000.0f);
    int32_t  lngInt      = (int32_t)(currentLng * 1000000.0f);
    uint32_t latU        = (uint32_t)latInt;
    uint32_t lngU        = (uint32_t)lngInt;
    uint16_t vBatInt     = (uint16_t)(finalBatteryVoltage * 100.0f);
    int16_t  temperature = (int16_t)(currentTemp * 10.0f);
    uint16_t humidity    = (uint16_t)(currentHum * 10.0f);
    int16_t  gyro_x      = (int16_t)(gyroX * 100.0f);
    int16_t  gyro_y      = (int16_t)(gyroY * 100.0f);
    uint32_t epochLocal  = (uint32_t)time(nullptr);

    out26[0]  = (uint8_t)STATION_ID;
    out26[1]  = (water_level >> 8) & 0xFF;
    out26[2]  = water_level & 0xFF;
    out26[3]  = (latU >> 24) & 0xFF;
    out26[4]  = (latU >> 16) & 0xFF;
    out26[5]  = (latU >> 8)  & 0xFF;
    out26[6]  = latU & 0xFF;
    out26[7]  = (lngU >> 24) & 0xFF;
    out26[8]  = (lngU >> 16) & 0xFF;
    out26[9]  = (lngU >> 8)  & 0xFF;
    out26[10] = lngU & 0xFF;
    out26[11] = (vBatInt >> 8) & 0xFF;
    out26[12] = vBatInt & 0xFF;
    out26[13] = (uint8_t)batPercentage;
    out26[14] = (temperature >> 8) & 0xFF;
    out26[15] = temperature & 0xFF;
    out26[16] = (humidity >> 8) & 0xFF;
    out26[17] = humidity & 0xFF;
    out26[18] = (gyro_x >> 8) & 0xFF;
    out26[19] = gyro_x & 0xFF;
    out26[20] = (gyro_y >> 8) & 0xFF;
    out26[21] = gyro_y & 0xFF;
    out26[22] = (epochLocal >> 24) & 0xFF;
    out26[23] = (epochLocal >> 16) & 0xFF;
    out26[24] = (epochLocal >> 8)  & 0xFF;
    out26[25] = epochLocal & 0xFF;
}

// ==============================================================================
// 9. OLED DISPLAY (DASHBOARD LAYOUT เดิม 100%)
// ==============================================================================
void updateDisplay() {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);

    // --- บรรทัดที่ 1 (Y=0): สถานะ (0), แบตเตอรี่ (34), เวลานับถอยหลัง (98) ---
    oled.setCursor(0, 0);
    oled.print(currentStatus);

    char batStr[16];
    sprintf(batStr, "%4.1fV%3d%%", finalBatteryVoltage, batPercentage);
    oled.setCursor(34, 0);
    oled.print(batStr);

    oled.setCursor(98, 0);
    if (!isJoinedNetwork && currentStatus == "JOIN") {
        oled.print("--:--");
    } else {
        unsigned long elapsed = millis() - cycleStartTime;
        unsigned long remain = (TX_INTERVAL_MS > elapsed) ? ((TX_INTERVAL_MS - elapsed) / 1000UL) : 0;
        char timeStr[8];
        sprintf(timeStr, "%02d:%02d", (int)(remain / 60), (int)(remain % 60));
        oled.print(timeStr);
    }

    // --- เส้นคั่น (Y=10) ---
    oled.drawLine(0, 10, 128, 10, SSD1306_WHITE);

    // --- บรรทัดที่ 2 (Y=12-13): ระดับน้ำ (ตัวเลขใหญ่ขนาด 2) ---
    oled.setCursor(0, 13);
    oled.print("Lvl:");
    oled.setCursor(28, 12);
    oled.setTextSize(2);
    oled.print(currentLevel, 1);
    oled.setTextSize(1);
    oled.print(" cm");

    // --- บรรทัดที่ 3 (Y=30): พิกัด GPS ละติจูด ลองจิจูด ---
    oled.setCursor(0, 30);
    char gpsStr[32];
    sprintf(gpsStr, "%.2f  %.2f", currentLat, currentLng);
    oled.print(gpsStr);

    // --- บรรทัดที่ 4 (Y=41): อุณหภูมิ และความชื้น SHT30 ---
    oled.setCursor(0, 41);
    char shtStr[24];
    sprintf(shtStr, "T:%.1fC  H:%.0f%%", currentTemp, currentHum);
    oled.print(shtStr);

    // --- บรรทัดที่ 5 (Y=54): สถานะมุมเอียง และค่า Gyro X, Y ---
    oled.setCursor(0, 54);
    char gyStr[32];
    sprintf(gyStr, "%s  X:%.0f  Y:%.0f", tiltStatus.c_str(), gyroX, gyroY);
    oled.print(gyStr);

    oled.display();
}

// ==============================================================================
// 10. TRANSITION ANIMATIONS & VISUAL FEEDBACK (ลูกเล่นเปิดม่าน Sci-Fi)
// ==============================================================================

// แอนิเมชันเปิดม่านกล่องขยายจากกึ่งกลางจอ (Curtain / Box Expand)
void animBoxExpand() {
    for (int step = 0; step <= 16; step++) {
        int w = step * 8;       // ขยายความกว้าง 0 -> 128
        int h = step * 4;       // ขยายความสูง 0 -> 64
        int x = (128 - w) / 2;
        int y = (64 - h) / 2;
        oled.clearDisplay();
        oled.drawRect(x, y, w, h, SSD1306_WHITE);
        oled.display();
        delay(10);
    }
}

// วาดกรอบสไตล์ Sci-Fi พร้อมมุมเหลี่ยมเท่ๆ
void drawSciFiFrame() {
    oled.drawRect(0, 0, 128, 64, SSD1306_WHITE);
    // กรอบมุมตัด (Corner Brackets)
    oled.drawLine(0, 5, 5, 0, SSD1306_WHITE);
    oled.drawLine(127, 5, 122, 0, SSD1306_WHITE);
    oled.drawLine(0, 58, 5, 63, SSD1306_WHITE);
    oled.drawLine(127, 58, 122, 63, SSD1306_WHITE);
    // จุดตกแต่งตรงกลางขอบ
    oled.drawFastHLine(58, 2, 12, SSD1306_WHITE);
    oled.drawFastHLine(58, 61, 12, SSD1306_WHITE);
}

// จอแสดงผลระหว่างกำลังส่งข้อมูล (ตัวโต พร้อมคลื่นวิทยุกระพริบ)
void drawSendingScreen(int waveStep) {
    oled.clearDisplay();
    drawSciFiFrame();
    oled.setTextColor(SSD1306_WHITE);

    oled.setTextSize(1);
    oled.setCursor(24, 7);
    oled.print("[ LORAWAN TX ]");

    oled.setTextSize(2);
    oled.setCursor(8, 21);
    oled.print("SENDING...");

    // ไอคอนเสาส่งสัญญาณและคลื่นวิทยุขยายตัว
    int cx = 64, cy = 54;
    oled.drawFastVLine(cx, cy - 6, 8, SSD1306_WHITE);
    oled.fillCircle(cx, cy - 7, 2, SSD1306_WHITE);

    if (waveStep >= 0) oled.drawCircle(cx, cy - 7, 6,  SSD1306_WHITE);
    if (waveStep >= 1) oled.drawCircle(cx, cy - 7, 12, SSD1306_WHITE);
    if (waveStep >= 2) oled.drawCircle(cx, cy - 7, 18, SSD1306_WHITE);

    oled.display();
}

// จอแสดงผลเมื่อได้รับ ACK หรือ Timeout (ตัวโต แสดงค้างไว้ 3 วินาที)
void showResultScreen(bool success) {
    animBoxExpand();
    oled.clearDisplay();
    drawSciFiFrame();
    oled.setTextColor(SSD1306_WHITE);

    if (success) {
        oled.setTextSize(1);
        oled.setCursor(24, 8);
        oled.print("[ STATUS: OK ]");

        oled.setTextSize(2);
        oled.setCursor(18, 24);
        oled.print("ACK OK!");

        oled.setTextSize(1);
        oled.setCursor(14, 48);
        oled.print("Gateway Confirmed");
    } else {
        oled.setTextSize(1);
        oled.setCursor(18, 8);
        oled.print("[ STATUS: FAIL ]");

        oled.setTextSize(2);
        oled.setCursor(18, 24);
        oled.print("NO ACK!");

        oled.setTextSize(1);
        oled.setCursor(18, 48);
        oled.print("Timeout / Retry");
    }
    oled.display();

    // แสดงผลข้อความตัวโตค้างไว้ 3 วินาทีตามที่ผู้ใช้ต้องการ
    delay(3000);

    // เปิดม่านกลับสู่หน้า Dashboard เซนเซอร์เดิม
    animBoxExpand();
    updateDisplay();
}

// จอเตือน Cooldown หากกดปุ่ม PRG ซ้ำก่อน 1 นาที
void showCooldownWarning(unsigned long remainSec) {
    oled.clearDisplay();
    drawSciFiFrame();
    oled.setTextColor(SSD1306_WHITE);

    oled.setTextSize(1);
    oled.setCursor(28, 8);
    oled.print("RATE LIMITED");

    oled.setTextSize(2);
    oled.setCursor(14, 24);
    oled.printf("WAIT %lus", remainSec);

    oled.setTextSize(1);
    oled.setCursor(18, 48);
    oled.print("Cooldown: 1 Min");

    oled.display();
    delay(1500); // แสดงเตือน 1.5 วินาที
    updateDisplay();
}

// ==============================================================================
// 11. LORAWAN CONFIRMED TRANSMISSION & CALLBACK
// ==============================================================================
extern "C" void downLinkAckHandle() {
    Serial.println(F("[LORAWAN] *** ได้รับ ACK จาก Gateway สำเร็จ ***"));
    if (waitingForAck) {
        lastConfirmedAcked = true;
        waitingForAck      = false;
    }
}

bool isTimeSynced = false;

extern "C" void dev_time_updated() {
    TimerSysTime_t sysTimeCurrent = TimerGetSysTime();
    if (sysTimeCurrent.Seconds > 1700000000) {
        struct timeval tv = { .tv_sec = (time_t)sysTimeCurrent.Seconds, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        isTimeSynced = true;
        Serial.printf("[TIME SYNC] ซิงค์เวลาจาก LoRaWAN Gateway สำเร็จ! Epoch: %lu\n", (unsigned long)sysTimeCurrent.Seconds);
    }
}

extern "C" void downLinkDataHandle(McpsIndication_t *mcpsIndication) {
    Serial.printf("[LORAWAN] Downlink Data: Port %d, RSSI %d, SNR %d\n",
                  mcpsIndication->Port, mcpsIndication->Rssi, mcpsIndication->Snr);
    TimerSysTime_t sysTimeCurrent = TimerGetSysTime();
    if (sysTimeCurrent.Seconds > 1700000000) {
        struct timeval tv = { .tv_sec = (time_t)sysTimeCurrent.Seconds, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        isTimeSynced = true;
        Serial.printf("[TIME SYNC] อัปเดตเวลาจริงจาก LoRaWAN สำเร็จ! Epoch: %lu\n", (unsigned long)sysTimeCurrent.Seconds);
    }
}

// ฟังก์ชันส่ง Probe ขอเวลาจริงจาก Gateway ทันทีที่ Join สำเร็จก่อนเริ่มส่งแพ็กเก็ตแรก
void syncTimeFromGateway() {
    if (isTimeSynced) return;

    Serial.println(F("[TIME] โหนดเพิ่งบูต -> เริ่มขอเวลาจริงจาก LoRaWAN Gateway (DeviceTimeReq)..."));
    currentStatus = "SYNC";
    updateDisplay();

    LoRaWAN.setDefaultDR(DR_2);
    defaultDrForNoAdr = DR_2;
    currentDrForNoAdr = DR_2;

    MibRequestConfirm_t mibReq;
    mibReq.Type = MIB_CHANNELS_DATARATE;
    mibReq.Param.ChannelsDatarate = DR_2;
    LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_CHANNELS_DEFAULT_DATARATE;
    mibReq.Param.ChannelsDefaultDatarate = DR_2;
    LoRaMacMibSetRequestConfirm(&mibReq);

    MlmeReq_t mlmeTimeReq;
    mlmeTimeReq.Type = MLME_DEVICE_TIME;
    LoRaMacMlmeRequest(&mlmeTimeReq);

    uint8_t dummyByte = 0x00;
    memcpy(appData, &dummyByte, 1);
    appDataSize = 1;
    appPort = 2;
    isTxConfirmed = false;
    LoRaWAN.send();
    isTxConfirmed = true;

    Serial.println(F("[TIME] ส่ง Time Sync Probe ออกไปแล้ว รอรับ DeviceTimeAns จาก Gateway (1-4 วินาที)..."));

    unsigned long startWait = millis();
    while (!isTimeSynced && (millis() - startWait < 5000)) {
        Mcu.timerhandler();
        Radio.IrqProcess();

        TimerSysTime_t sysTime = TimerGetSysTime();
        if (sysTime.Seconds > 1700000000) {
            struct timeval tv = { .tv_sec = (time_t)sysTime.Seconds, .tv_usec = 0 };
            settimeofday(&tv, NULL);
            isTimeSynced = true;
            break;
        }
        delay(10);
    }

    if (isTimeSynced) {
        Serial.printf("[TIME] *** ซิงค์เวลาจริงสำเร็จ! Real Epoch: %lu ***\n", (unsigned long)time(nullptr));
    } else {
        Serial.println(F("[TIME] !!! Gateway ยังไม่ตอบรับเวลาจริง (จะขอเวลาซ้ำในแพ็กเก็ตถัดไป) !!!"));
    }
}

bool sendConfirmedAndWait(const uint8_t *payload, uint8_t size, uint32_t ackTimeoutMs) {
    memcpy(appData, payload, size);
    appDataSize = size;

    lastConfirmedAcked = false;
    waitingForAck      = true;

    // บังคับล็อค DataRate ให้เป็น DR 2 (SF10) ตามโค้ดเดิม
    LoRaWAN.setDefaultDR(DR_2);
    defaultDrForNoAdr = DR_2;
    currentDrForNoAdr = DR_2;

    MibRequestConfirm_t mibReq;
    mibReq.Type = MIB_CHANNELS_DATARATE;
    mibReq.Param.ChannelsDatarate = DR_2;
    LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_CHANNELS_DEFAULT_DATARATE;
    mibReq.Param.ChannelsDefaultDatarate = DR_2;
    LoRaMacMibSetRequestConfirm(&mibReq);

    // ขอซิงค์เวลาจาก ChirpStack / LoRaWAN Gateway หากยังไม่ได้ซิงค์
    if (!isTimeSynced) {
        MlmeReq_t mlmeTimeReq;
        mlmeTimeReq.Type = MLME_DEVICE_TIME;
        LoRaMacMlmeRequest(&mlmeTimeReq);
        Serial.println(F("[TIME] แนบคำขอซิงค์เวลาจาก Gateway (DeviceTimeReq) ไปกับ Uplink..."));
    }

    LoRaWAN.send();
    Serial.printf("[LORAWAN] กำลังส่ง Uplink Confirmed (%d bytes) ที่ DR %d รอ ACK...\n", size, currentDrForNoAdr);

    unsigned long waitStart = millis();
    int waveStep = 0;
    unsigned long lastAnimTime = 0;

    while (waitingForAck && (millis() - waitStart) < ackTimeoutMs) {
        // ประมวลผล Radio IRQ และ Timers โดยตรง ไม่เรียก Mcu.sleep เพื่อไม่ให้เกิด Deep Sleep
        Mcu.timerhandler();
        Radio.IrqProcess();

        // ตรวจสอบเวลาคู่ขนานขณะรอ ACK
        if (!isTimeSynced) {
            TimerSysTime_t sysTime = TimerGetSysTime();
            if (sysTime.Seconds > 1700000000) {
                struct timeval tv = { .tv_sec = (time_t)sysTime.Seconds, .tv_usec = 0 };
                settimeofday(&tv, NULL);
                isTimeSynced = true;
                Serial.printf("[TIME SYNC] ซิงค์เวลาจริงระหว่างรอ ACK สำเร็จ! Epoch: %lu\n", (unsigned long)sysTime.Seconds);
            }
        }

        // อัปเดตแอนิเมชันคลื่นวิทยุขณะรอ ACK ทุก 180 ms
        if (millis() - lastAnimTime >= 180) {
            lastAnimTime = millis();
            drawSendingScreen(waveStep);
            waveStep = (waveStep + 1) % 3;
        }
        delay(10);
    }

    if (waitingForAck) {
        Serial.println(F("[LORAWAN] หมดเวลารอ ACK (5 วินาที) -> ไม่ได้รับ ACK"));
    }

    waitingForAck = false;
    return lastConfirmedAcked;
}

// ==============================================================================
// 12. SETUP
// ==============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println(F("\n============================================="));
    Serial.println(F(" HELTEC V3 - DUMMY SENSOR NODE (PRG TRIGGER) "));
    Serial.println(F("============================================="));

    // สุ่มค่าเริ่มต้นจากฮาร์ดแวร์ ESP32
    randomSeed(esp_random());

    Serial.println(F("[TIME] โหมดเวลาจริง: รอดึงเวลาจาก LoRaWAN Gateway (DeviceTimeReq) - ไม่ใช้ Mockup"));

    // สร้างค่าจำลองรอบแรกทันทีเพื่อพร้อมแสดงผลบนจอ
    generateDummyValues();

    // เปิดไฟเลี้ยงหน้าจอ OLED
    VextON();
    delay(100);

    // กำหนดขา LED และปุ่ม PRG (GPIO 0)
    pinMode(LED_RED_PIN,   OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    digitalWrite(LED_RED_PIN,   HIGH);
    digitalWrite(LED_GREEN_PIN, LOW);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), prgButtonISR, FALLING);
    Serial.println(F("[BUTTON] PRG Pin (GPIO 0) initialized with interrupt."));

    // เริ่มต้นระบบฮาร์ดแวร์ Heltec LoRaWAN
    overTheAirActivation = true;
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

    // ล็อก Channel 0 (923.4 MHz) ให้ใช้ DR 2 (SF10) ตามโค้ดเดิม
    LoRaMacChannelAdd(0, (ChannelParams_t){923400000, 0, {((DR_2 << 4) | DR_2)}, 0});

    // เริ่มต้น I2C และหน้าจอ OLED
    Wire.begin(17, 18);
    pinMode(OLED_RESET, OUTPUT);
    digitalWrite(OLED_RESET, LOW);
    delay(10);
    digitalWrite(OLED_RESET, HIGH);
    delay(10);

    if (oled.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) {
        currentStatus = "BOOT";
        animBoxExpand();
        updateDisplay();
        delay(1000);
    }

    Serial.println(F("[SYSTEM] Init Complete -> Entering LoRaWAN State Machine"));
    deviceState = DEVICE_STATE_INIT;
}

// ==============================================================================
// 13. LOOP (ALWAYS-ON STATE MACHINE WITH PRG TRIGGER)
// ==============================================================================
void loop() {
    unsigned long now = millis();

    // 1. ดักจับการกดปุ่ม PRG เพื่อส่งข้อมูลทันที
    if (prgPressed) {
        prgPressed = false;
        Serial.println(F("[BUTTON] กดปุ่ม PRG ตรวจสอบสิทธิ์การส่ง..."));

        if (!isJoinedNetwork && deviceState != DEVICE_STATE_SLEEP) {
            Serial.println(F("[BUTTON] กำลัง Join เครือข่ายอยู่ ยังไม่สามารถส่งได้"));
        } else if (deviceState == DEVICE_STATE_SEND) {
            Serial.println(F("[BUTTON] กำลังอยู่ในกระบวนการส่งข้อมูลอยู่แล้ว"));
        } else {
            // ตรวจสอบ Cooldown 1 นาที (60 วินาที) จากการส่งครั้งล่าสุด
            unsigned long elapsedSinceLastTx = now - lastTxTime;
            if (lastTxTime > 0 && elapsedSinceLastTx < MANUAL_TX_COOLDOWN_MS) {
                unsigned long remainSec = (MANUAL_TX_COOLDOWN_MS - elapsedSinceLastTx + 999) / 1000UL;
                Serial.printf("[BUTTON] ติด Cooldown! กรุณารออีก %lu วินาที (ห้ามส่งถี่กว่า 1 นาที)\n", remainSec);
                showCooldownWarning(remainSec);
            } else {
                Serial.println(F("[BUTTON] Cooldown ครบถ้วน -> สั่งส่งข้อมูลทันที!"));
                deviceState = DEVICE_STATE_SEND;
            }
        }
    }

    // 2. อัปเดตหน้าจอ OLED ทุกๆ 1 วินาทีอย่างต่อเนื่อง (Dashboard ปกติ)
    if (now - lastDisplayUpdate >= 1000) {
        lastDisplayUpdate = now;

        // หลังจากแสดงผล ACK หรือ NACK ผ่านไป 5 วินาที ให้เปลี่ยนสถานะเป็น "WAIT"
        if ((currentStatus == "ACK" || currentStatus == "NACK") && (now - cycleStartTime >= 5000)) {
            currentStatus = "WAIT";
        }
        updateDisplay();
    }

    // 3. ตรวจสอบรอบเวลาส่งอัตโนมัติ (Always-On จัดการเวลาด้วย millis() โดยตรง ไม่ใช้ Deep Sleep)
    if (isJoinedNetwork && deviceState == DEVICE_STATE_SLEEP) {
        if (now - cycleStartTime >= TX_INTERVAL_MS) {
            Serial.println(F("[CYCLE] ครบกำหนดรอบเวลา -> เริ่มการส่งข้อมูลรอบใหม่อัตโนมัติ!"));
            deviceState = DEVICE_STATE_SEND;
        }
    }

    // 4. LoRaWAN State Machine
    switch (deviceState) {

        case DEVICE_STATE_INIT: {
            #if (LORAWAN_DEVEUI_AUTO)
                LoRaWAN.generateDeveuiByChipID();
            #endif
            LoRaWAN.init(loraWanClass, loraWanRegion);
            LoRaWAN.setDefaultDR(DR_2);
            defaultDrForNoAdr = DR_2;
            currentDrForNoAdr = DR_2;
            break;
        }

        case DEVICE_STATE_JOIN: {
            currentStatus = "JOIN";
            updateDisplay();
            Serial.println(F("[JOIN] Sending Join Request to Gateway..."));
            LoRaWAN.join();
            break;
        }

        case DEVICE_STATE_SEND: {
            isJoinedNetwork = true;
            digitalWrite(LED_RED_PIN, LOW); // ดับไฟแดงเมื่อ Join สำเร็จ

            // 1. หากเพิ่งบูตเครื่องและยังไม่ได้ซิงค์เวลาจริง ให้ขอเวลาก่อนส่งแพ็กเก็ตแรก
            if (!isTimeSynced) {
                syncTimeFromGateway();
            }

            // 2. สุ่มค่าข้อมูลจำลองรอบใหม่ ณ เวลาที่จะส่งจริง
            generateDummyValues();

            // 3. จัดรูปแบบ 26-Byte Payload โดยใช้เวลาจริง ณ วินาทีนี้ (time(nullptr))
            uint8_t payload[PAYLOAD_TOTAL_SIZE];
            buildSensorPayload(payload);

            currentStatus = "SEND";
            digitalWrite(LED_GREEN_PIN, HIGH);

            // 3. แอนิเมชันเปิดม่านเข้าสู่หน้าจอส่งข้อมูล
            animBoxExpand();
            drawSendingScreen(0);

            // 4. ส่ง Confirmed Uplink และรอ ACK สูงสุด 5 วินาที (พร้อมแสดงคลื่นวิทยุเคลื่อนไหว)
            bool acked = sendConfirmedAndWait(payload, PAYLOAD_TOTAL_SIZE, CONFIRMED_ACK_TIMEOUT_MS);
            digitalWrite(LED_GREEN_PIN, LOW);

            // บันทึกเวลาที่ส่งสำเร็จ และรีเซ็ตรอบใหม่
            lastTxTime     = millis();
            cycleStartTime = millis();

            if (acked) {
                currentStatus = "ACK";
                Serial.println(F("[SEND] *** ส่งสำเร็จ ได้รับ ACK จาก Gateway ***"));
            } else {
                currentStatus = "NACK";
                Serial.println(F("[SEND] !!! ส่งออกไปแล้ว แต่ไม่ได้รับ ACK จาก Gateway (Timeout) !!!"));
            }

            // 5. แสดงผลลัพธ์ตัวโตค้างไว้ 3 วินาที พร้อมม่านปิดเปลี่ยนกลับ
            showResultScreen(acked);

            deviceState = DEVICE_STATE_CYCLE;
            break;
        }

        case DEVICE_STATE_CYCLE: {
            TimerStop(&TxNextPacketTimer); // หยุด Timer ของไลบรารีเพื่อป้องกันไม่ให้สั่งเข้า Deep Sleep เด็ดขาด
            Radio.Sleep();                 // ให้ภาครับ-ส่งวิทยุพัก (Standby) ระหว่างรอ
            deviceState = DEVICE_STATE_SLEEP;
            Serial.printf("[CYCLE] เข้าสู่รอบการรอส่งถัดไป (อีก %lu วินาที / %.1f นาที) [Always-On ไม่ Sleep]\n",
                          TX_INTERVAL_MS / 1000UL, (float)TX_INTERVAL_MS / 60000.0f);
            break;
        }

        case DEVICE_STATE_SLEEP: {
            // ทำงานต่อเนื่อง Always-On ไม่เรียก Mcu.sleep โดยเด็ดขาด
            // ประมวลผลเฉพาะ Radio IRQ และ Timers เพื่อรอ Join-Accept หรือ Downlink
            Mcu.timerhandler();
            Radio.IrqProcess();
            delay(10);
            break;
        }

        default: {
            deviceState = DEVICE_STATE_INIT;
            break;
        }
    }
}
