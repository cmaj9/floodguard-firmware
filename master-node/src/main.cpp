/*
 * HELTEC V3 LoRaWAN - MASTER DEPLOYMENT MODE (ESP32-C3 Sensor Bridge)
 * 1. ส่ง LoRaWAN ทุกๆ 10 นาที พร้อมเวลานับถอยหลังบนจอ
 * 2. GPS (UART) + Direct RMC Parser -> ฝัง Unix Epoch Timestamp ถาวรใน Payload ไบต์ 22-25
 * 3. หน้าจอ OLED ปิดอัตโนมัติเมื่อเข้าสู่ Sleep (ปลุกด้วยปุ่ม PRG)
 * 4. ระดับน้ำ (ultrasonic) + Gyro: รับค่าจาก ESP32-C3 ผ่าน UART (REQ/ACK)
 * 5. แบตเตอรี่: อ่านผ่าน ADC ขา 2 พร้อมระบบ Median + EMA Filter
 * 6. Relay ควบคุมไฟวงจรภายนอกผ่าน GPIO 26 (HIGH = เปิดจ่ายไฟ, LOW = ตัดไฟเมื่อ Sleep)
 * 7. Flash Backlog: บันทึกข้อมูลขนาด 26 ไบต์ลง LittleFS สะสมต่อเนื่องทนต่อการ Reboot
 * 8. ป้องกัน NACK: เพิ่ม Delay หลัง Join, เพิ่มระบบ Retry 2 ครั้ง, ล็อก DR 2 ถาวร
 */

#include "LoRaWan_APP.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <Adafruit_SHT31.h>
#include <math.h>
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <LittleFS.h>
#include <time.h>
#include <sys/time.h>
#include "esp_sleep.h"
#include "soc/rtc.h"
#include "esp_private/esp_clk.h"

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

// ==============================================================================
// การตั้งค่ารอบเวลาทำงาน (CYCLE) และเวลาตื่น (ACTIVE PHASE)
// ==============================================================================
//  จุดที่ 1: กำหนดรอบเวลารวมทั้งหมด (Total Cycle)
//    - ส่งข้อมูลทุก 10 นาที = 600000UL ms (ค่าปัจจุบัน)
//    - ส่งข้อมูลทุก 15 นาที = 900000UL ms
//    - ส่งข้อมูลทุก  5 นาที = 300000UL ms
#define TOTAL_CYCLE_MS       600000UL    // << [แก้รอบส่งทั้งหมดที่นี่] (มิลลิวินาที)

//  จุดที่ 2: กำหนดเวลาตื่นทำงาน (Active Phase)
//    เวลาที่เปิด Relay 26 เลี้ยงวงจรเซนเซอร์, เปิดหน้าจอ OLED, อ่านค่าเซนเซอร์ และส่ง LoRaWAN
//    - 45 วินาที = 45000UL ms (ค่าปัจจุบัน)
//    - 60 วินาที = 60000UL ms
//    - 30 วินาที = 30000UL ms
#define ACTIVE_DURATION      45000UL     // << [แก้เวลาตื่นที่นี่] (มิลลิวินาที)

//  จุดที่ 3: กำหนดเวลาเปิดหน้าจอ OLED (Screen On Duration)
//    - กำหนดเวลาเปิดหน้าจอ OLED ให้ผู้ใช้ตรวจดูค่า (25 วินาที = 25000UL ms) ก่อนจะดับจอเพื่อประหยัดแบตเตอรี่
//    - การดับจอที่เวลานี้ จะไม่มีผลกระทบต่อรอบเวลาทำงาน 1 cycle (Relay เลี้ยงเซนเซอร์ และ LoRaWAN ยังทำงานครบ 45 วิตามปกติ)
#define SCREEN_ON_DURATION_MS 25000UL    // << [แก้เวลาเปิดหน้าจอที่นี่] (มิลลิวินาที)
#define INSPECTION_DURATION_MS SCREEN_ON_DURATION_MS // เวลาตื่นตรวจเช็คด้วยปุ่ม PRG (25 วินาที)

// ------------------------------------------------------------------------------
//  ตัวแปรระบบ (คำนวณเวลานอน Sleep ให้อัตโนมัติจาก 2 ค่าด้านบน ไม่ต้องแก้ไขเอง):
//    - เวลานอน Sleep = TOTAL_CYCLE_MS - ACTIVE_DURATION (เช่น 600,000 - 45,000 = 555,000 ms หรือ 9.25 นาที)
// ------------------------------------------------------------------------------
uint32_t appTxDutyCycle      = TOTAL_CYCLE_MS;
uint32_t currentTxWait       = TOTAL_CYCLE_MS;
#define  JOIN_CYCLE_TOTAL_MS  ACTIVE_DURATION
#define  JOIN_SLEEP_MS        (TOTAL_CYCLE_MS - ACTIVE_DURATION)

// ================= OTAA Join Control =================
#define JOIN_ATTEMPT_TIMEOUT_MS  30000UL  // 30 วิ: ถ้า Join ไม่ติด ให้เซฟค่าลง Flash ทันที

bool    overTheAirActivation = true;
bool    loraWanAdr           = false;     // ปิด ADR เพื่อให้คงค่า DR 2 ตลอดเวลา
bool    isTxConfirmed        = true;
uint8_t appPort              = 2;
uint8_t confirmedNbTrials    = 1;

// กำหนด DataRate ระดับโกลบอลของไลบรารีให้เป็น DR 2 (SF10)
int8_t  loraWanDatarate      = DR_2;
extern int8_t defaultDrForNoAdr;
extern int8_t currentDrForNoAdr;

bool          joinTimerStarted    = false;
unsigned long joinCycleStartTime  = 0;
bool          joinFallbackStarted = false;
bool          joinSleepPhase      = false;
unsigned long joinSleepStartTime  = 0;

// ================= ตัวแปรจับเวลา =================
unsigned long activeStartTime    = 0;
unsigned long screenTimer        = 0;
unsigned long lastDisplayUpdate  = 0;
unsigned long relayPowerOnTime   = 0;  // จับเวลาตั้งแต่เปิดไฟ Relay 26 เลี้ยงเซนเซอร์
bool          isActivePhase      = false;

// โหมดตรวจเช็คเซนเซอร์ด้วยปุ่ม PRG (25 วินาที, ไม่ส่ง LoRaWAN)
volatile bool isInspectionMode        = false;
unsigned long inspectionStartTime     = 0;
volatile bool pendingTransmission     = false;
volatile unsigned long lastPrgInterruptTime = 0;

const unsigned long SCREEN_TIMEOUT = SCREEN_ON_DURATION_MS;

// ================= Hardware Config =================
#define RELAY_PIN     26    // สวิตช์ MOSFET ควบคุมไฟวงจรภายนอก (HIGH=เปิด, LOW=ตัดไฟ)
#define LED_RED_PIN   4
#define LED_GREEN_PIN 5
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    21
#define VEXT_PIN      36
#define BUTTON_PIN    0

volatile bool prgButtonPressed = false;
void IRAM_ATTR prgButtonISR()
{
    unsigned long now = millis();
    if (now - lastPrgInterruptTime > 500) { // 500ms software debounce
        prgButtonPressed = true;
        lastPrgInterruptTime = now;
    }
}

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool isScreenOn = true;

// ================= เซนเซอร์ =================
#define GPS_RX_PIN     19
#define GPS_TX_PIN     20
#define CUSTOM_BAT_PIN 2

// ================= UART รับค่าจาก ESP32-C3 =================
#define ESP32_RX_PIN   41
#define ESP32_TX_PIN   42
#define ESP32_BAUD     9600
HardwareSerial ESP32Serial(2);

float gyroX = 0, gyroY = 0;

#define LVL_MIN_VALID  5.0f
#define LVL_MAX_VALID  1000.0f

#define  ESP32_MAX_FAIL    3

// ================= SHT30 =================
#define SHT30_SDA 6
#define SHT30_SCL 7
TwoWire          I2CSHT = TwoWire(1);
Adafruit_SHT31   sht31  = Adafruit_SHT31(&I2CSHT);
float currentTemp = NAN;
float currentHum  = NAN;

TinyGPSPlus gps;
String tiltStatus = "OK";

// ================= Global Sensor Values =================
float    currentLevel        = 0.0;
float    currentLat          = 0.0;
float    currentLng          = 0.0;
int      batPercentage       = 0;
float    finalBatteryVoltage = 0.0;
float    calibrationOffset   = 0.00f;
String   currentStatus       = "Booting...";
int      station_id          = 1;
uint32_t currentEpoch        = 0;

bool     gpsValidCached = false;

// ================= Battery (LiFePO4 4S 12.8V) =================
const float BAT_R1        = 100000.0f;
const float BAT_R2        =  20000.0f;
const float BAT_DIV_RATIO = (BAT_R1 + BAT_R2) / BAT_R2;

// RTC Memory: คงค่าไว้ข้าม Deep Sleep ไม่หายไปหลังหลับ และไม่ทำให้ Flash เสื่อม
RTC_DATA_ATTR float    rtcLastStableBattPct     = -1.0f;
RTC_DATA_ATTR uint32_t rtcCycleTargetEpoch      = 0;      // เวลาเป้าหมายที่ต้องตื่นเริ่ม Active รอบถัดไป (Epoch หรือ relative sec)
bool newCycleSoCAllowed = true;

// คำนวณ % แบตเตอรี่ LiFePO4 4S ด้วย OCV Piecewise Curve (11.20V - 14.00V)
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

// ฟังก์ชันอ่านค่าแรงดันแบตเตอรี่แบบ Oversampling (31 samples, Median/Mean Trimmed)
float readRawBatteryVoltage() {
    int samples[31];
    for (int i = 0; i < 31; i++) {
        samples[i] = analogReadMilliVolts(CUSTOM_BAT_PIN);
        delay(2);
    }
    for (int i = 0; i < 30; i++) {
        for (int j = i + 1; j < 31; j++) {
            if (samples[i] > samples[j]) {
                int tmp = samples[i];
                samples[i] = samples[j];
                samples[j] = tmp;
            }
        }
    }
    long sum = 0;
    for (int i = 10; i <= 20; i++) { sum += samples[i]; }
    float adcMv = sum / 11.0f;
    return ((adcMv / 1000.0f) * BAT_DIV_RATIO) + calibrationOffset;
}

// 3-Layer Battery SoC Stabilization Algorithm
int processBatterySoC(float measuredVoltage, bool isNoLoad, bool forceNewCycle = false) {
    // ชั้นที่ 1: ชดเชยแรงดันตกคร่อมหากวัดขณะ Relay ON (+0.08V)
    float effectiveVoltage = isNoLoad ? measuredVoltage : (measuredVoltage + 0.08f);
    float rawPct = calculateLiFePO4Percent(effectiveVoltage);

    // Initial / Cold boot: ดึงค่าตามแรงดันจริงทันที
    if (rtcLastStableBattPct < 0.0f || rtcLastStableBattPct > 100.0f) {
        rtcLastStableBattPct = rawPct;
        newCycleSoCAllowed = false;
        return (int)(constrain(rtcLastStableBattPct, 0.0f, 100.0f) + 0.5f);
    }

    // ชั้นที่ 2: ตรวจจับแสงแดด (Solar Charging Phase: V >= 13.45V)
    if (effectiveVoltage >= 13.45f) {
        // แดดชาร์จจริง -> อนุญาตให้เปอร์เซ็นต์เพิ่มขึ้นได้
        if (rawPct > rtcLastStableBattPct) {
            rtcLastStableBattPct = (rtcLastStableBattPct * 0.70f) + (rawPct * 0.30f);
        } else {
            rtcLastStableBattPct = rawPct;
        }
        return (int)(constrain(rtcLastStableBattPct, 0.0f, 100.0f) + 0.5f);
    }

    // ชั้นที่ 3: กลางคืน / Discharging Phase (V < 13.45V)
    // 3.1 Safety Bypass: หากแบตเตอรี่ต่ำวิกฤต (< 12.50V) ให้ลดลงตามจริงทันทีเพื่อความปลอดภัย
    if (effectiveVoltage < 12.50f) {
        rtcLastStableBattPct = min(rtcLastStableBattPct, rawPct);
        return (int)(constrain(rtcLastStableBattPct, 0.0f, 100.0f) + 0.5f);
    }

    // 3.2 Monotonic Clamping: ห้าม % เด้งกลับขึ้นมาเด็ดขาดในตอนกลางคืน
    if (rawPct > rtcLastStableBattPct) {
        rawPct = rtcLastStableBattPct;
    }

    // 3.3 Dynamic Rate Limiter: จำกัดการลดลงสูงสุดตามสัดส่วนความยาวรอบส่ง (ฐาน 2.0% ต่อ 10 นาที = 600,000 ms)
    // ใช้สิทธิ์การลดลงเฉพาะรอบใหม่เท่านั้น ไม่ลดลงซ้ำๆ ในระหว่างการโพลล์หน้าจอ
    if (forceNewCycle || newCycleSoCAllowed) {
        float maxAllowedDrop = 2.0f * ((float)TOTAL_CYCLE_MS / 600000.0f);
        if (maxAllowedDrop < 0.5f) maxAllowedDrop = 0.5f; // ค่าเผื่อขั้นต่ำ 0.5% สำหรับรอบส่งสั้นมาก

        if ((rtcLastStableBattPct - rawPct) > maxAllowedDrop) {
            rtcLastStableBattPct -= maxAllowedDrop;
        } else {
            rtcLastStableBattPct = rawPct;
        }
        newCycleSoCAllowed = false;
    }

    return (int)(constrain(rtcLastStableBattPct, 0.0f, 100.0f) + 0.5f);
}

// ================= Background Sensor Task =================
#define SENSOR_READ_INTERVAL_MS      5000
#define SENSOR_IDLE_POLL_MS          500
#define SENSOR_FIRST_READ_TIMEOUT_MS 8000

TaskHandle_t      sensorTaskHandle = NULL;
SemaphoreHandle_t sensorMutex      = NULL;

volatile bool sensorTaskActive = true;
volatile bool sensorDataReady  = false;
extern uint16_t backlogCount;

void VextON() {
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, LOW);
}

void VextOFF() {
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, HIGH);
}

void updateDisplay()
{
    if (!isScreenOn) return;

    float  lvlLocal, gxLocal, gyLocal, tempLocal, humLocal, latLocal, lngLocal, battVLocal;
    int    battPctLocal;
    bool   gpsOkLocal;
    String tiltLocal;

    if (sensorMutex != NULL && xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        lvlLocal     = currentLevel;
        gxLocal      = gyroX;
        gyLocal      = gyroY;
        tempLocal    = currentTemp;
        humLocal     = currentHum;
        latLocal     = currentLat;
        lngLocal     = currentLng;
        battVLocal   = finalBatteryVoltage;
        battPctLocal = batPercentage;
        gpsOkLocal   = gpsValidCached;
        tiltLocal    = tiltStatus;
        xSemaphoreGive(sensorMutex);
    } else {
        lvlLocal = currentLevel; gxLocal = gyroX; gyLocal = gyroY;
        tempLocal = currentTemp; humLocal = currentHum;
        latLocal = currentLat; lngLocal = currentLng;
        battVLocal = finalBatteryVoltage; battPctLocal = batPercentage;
        gpsOkLocal = gpsValidCached; tiltLocal = tiltStatus;
    }

    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);

    String shortStatus = currentStatus;
    if      (isInspectionMode || currentStatus == "Check...") shortStatus = "CHK";
    else if (currentStatus == "Booting...")  shortStatus = "BOOT";
    else if (currentStatus == "Joining...")  shortStatus = "JOIN";
    else if (currentStatus == "Sending...") {
        shortStatus = (backlogCount > 0) ? "DRAN" : "SEND";
    }
    else if (currentStatus == "Monitor...")  shortStatus = "MON";
    else if (currentStatus == "Reading...")  shortStatus = "READ";
    else if (currentStatus == "No Join...")  shortStatus = "NOJN";
    else if (currentStatus == "Sleep")       shortStatus = "SLP";

    oled.setCursor(0, 0);
    oled.print(shortStatus);

    char batStr[16];
    sprintf(batStr, "%4.1fV%3d%%", battVLocal, battPctLocal);
    oled.setCursor(34, 0);
    oled.print(batStr);

    oled.setCursor(98, 0);
    if (isInspectionMode) {
        unsigned long insElapsed = millis() - inspectionStartTime;
        unsigned long remain = (INSPECTION_DURATION_MS > insElapsed)
                               ? (INSPECTION_DURATION_MS - insElapsed) / 1000 : 0;
        char timeStr[8];
        sprintf(timeStr, "%02d:%02d", (int)(remain / 60), (int)(remain % 60));
        oled.print(timeStr);
    } else if (isActivePhase) {
        if (backlogCount > 0 && currentStatus == "Sending...") {
            char bkStr[8];
            sprintf(bkStr, "B:%3d", backlogCount);
            oled.print(bkStr);
        } else {
            unsigned long activeElapsed = millis() - activeStartTime;
            unsigned long remain = (ACTIVE_DURATION > activeElapsed)
                                   ? (ACTIVE_DURATION - activeElapsed) / 1000 : 0;
            char timeStr[8];
            sprintf(timeStr, "%02d:%02d", (int)(remain / 60), (int)(remain % 60));
            oled.print(timeStr);
        }
    } else {
        oled.print("--:--");
    }

    oled.drawLine(0, 10, 128, 10, SSD1306_WHITE);

    oled.setCursor(0, 13);
    oled.print("Lvl:");
    oled.setCursor(28, 12);
    oled.setTextSize(2);
    oled.print(lvlLocal , 1);
    oled.setTextSize(1);
    oled.print(" cm");

    oled.setCursor(0, 30);
    if (gpsOkLocal) {
        char gpsStr[32];
        sprintf(gpsStr, "%.2f  %.2f", latLocal, lngLocal);
        oled.print(gpsStr);
    } else {
        oled.print("GPS: Searching...");
    }

    oled.setCursor(0, 41);
    if (!isnan(tempLocal) && !isnan(humLocal)) {
        char shtStr[24];
        sprintf(shtStr, "T:%.1fC  H:%.0f%%", tempLocal, humLocal);
        oled.print(shtStr);
    } else {
        oled.print("T: ---  H: ---");
    }

    oled.setCursor(0, 54);
    char gyStr[32];
    sprintf(gyStr, "%s  X:%.0f  Y:%.0f",
            tiltLocal.c_str(), gxLocal, gyLocal);
    oled.print(gyStr);

    oled.display();
}

// ================= FCnt Flash Storage =================
uint32_t loadFCnt() {
    Preferences p;
    p.begin("lora_fcnt", true);
    uint32_t val = p.getUInt("fcnt", 0);
    p.end();
    Serial.printf("[FCNT] Load: %lu\n", val);
    return val;
}

void saveFCnt(uint32_t fcnt) {
    Preferences p;
    p.begin("lora_fcnt", false);
    p.putUInt("fcnt", fcnt);
    p.end();
    Serial.printf("[FCNT] Save: %lu\n", fcnt);
}

// ================= BACKLOG STORAGE SYSTEM (GPS EPOCH MASTER) =================
#define PAYLOAD_TOTAL_SIZE      26
#define BACKLOG_QUEUE_CAPACITY  720
#define FLASH_BACKLOG_FILE      "/backlog.bin"
#define FLASH_BACKLOG_MAX       720

struct QueuedPacket {
    uint8_t payload[PAYLOAD_TOTAL_SIZE];
};

QueuedPacket backlogQueue[BACKLOG_QUEUE_CAPACITY];
uint16_t     backlogHead  = 0;
uint16_t     backlogCount = 0;

void enqueueBacklog(const uint8_t *payload26)
{
    if (backlogCount == BACKLOG_QUEUE_CAPACITY) {
        Serial.println("[BACKLOG] คิวเต็ม (720) -> ทิ้งค่าเก่าสุด 1 รายการ");
        backlogHead = (backlogHead + 1) % BACKLOG_QUEUE_CAPACITY;
        backlogCount--;
    }
    uint16_t tailIdx = (backlogHead + backlogCount) % BACKLOG_QUEUE_CAPACITY;
    memcpy(backlogQueue[tailIdx].payload, payload26, PAYLOAD_TOTAL_SIZE);
    backlogCount++;
}

void dequeueBacklog()
{
    if (backlogCount == 0) return;
    backlogHead = (backlogHead + 1) % BACKLOG_QUEUE_CAPACITY;
    backlogCount--;
}

void parseNmeaRmc(const String &line, uint32_t &outEpoch)
{
    if (!line.startsWith("$GP") && !line.startsWith("$GN") && !line.startsWith("$BD")) return;
    if (line.indexOf("RMC") < 0) return;

    int commaIdx[12];
    int count = 0;
    for (int i = 0; i < line.length() && count < 12; i++) {
        if (line[i] == ',') commaIdx[count++] = i;
    }
    if (count < 10) return;

    char status = line[commaIdx[1] + 1];
    if (status != 'A') return;

    String tStr = line.substring(commaIdx[0] + 1, commaIdx[1]);
    String dStr = line.substring(commaIdx[8] + 1, commaIdx[9]);

    if (tStr.length() >= 6 && dStr.length() == 6) {
        int hour = tStr.substring(0, 2).toInt();
        int min  = tStr.substring(2, 4).toInt();
        int sec  = tStr.substring(4, 6).toInt();

        int day  = dStr.substring(0, 2).toInt();
        int mon  = dStr.substring(2, 4).toInt();
        int year = 2000 + dStr.substring(4, 6).toInt();

        if (year >= 2024 && mon >= 1 && mon <= 12 && day >= 1 && day <= 31) {
            const int daysBeforeMonth[] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };

            long totalDays = 0;
            for (int currYear = 1970; currYear < year; currYear++) {
                bool isLeap = ((currYear % 4 == 0 && currYear % 100 != 0) || (currYear % 400 == 0));
                totalDays += isLeap ? 366 : 365;
            }

            totalDays += daysBeforeMonth[mon - 1];

            bool isCurrentLeap = ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0));
            if (mon > 2 && isCurrentLeap) {
                totalDays += 1;
            }

            totalDays += (day - 1);

            uint32_t ep = (uint32_t)(totalDays * 86400UL + hour * 3600UL + min * 60UL + sec);

            if (ep > 1700000000) {
                outEpoch = ep;
                time_t oldTime = time(nullptr);
                int64_t timeShift = (int64_t)ep - (int64_t)oldTime;
                struct timeval tv = { .tv_sec = (time_t)ep, .tv_usec = 0 };
                settimeofday(&tv, NULL);
                if (rtcCycleTargetEpoch > 0 && abs((long)timeShift) > 2) {
                    rtcCycleTargetEpoch += timeShift;
                    Serial.printf("[GPS TIME] ซิงค์เวลาดาวเทียมสำเร็จ: Epoch %lu -> ปรับเป้าหมายรอบถัดไปเป็น: %lu\n",
                                  (unsigned long)ep, (unsigned long)rtcCycleTargetEpoch);
                }
            }
        }
    }
}

void buildSensorPayload(uint8_t *out26)
{
    float    lvlLocal = 0, latLocal = 0, lngLocal = 0, battVLocal = 0, tempLocal = 0, humLocal = 0, gxLocal = 0, gyLocal = 0;
    int      battPctLocal = 0;
    uint32_t epochLocal = 0;

    if (sensorMutex != NULL && xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
        lvlLocal     = currentLevel;
        latLocal     = currentLat;
        lngLocal     = currentLng;
        battVLocal   = finalBatteryVoltage;
        battPctLocal = batPercentage;
        tempLocal    = currentTemp;
        humLocal     = currentHum;
        gxLocal      = gyroX;
        gyLocal      = gyroY;
        epochLocal   = currentEpoch;
        xSemaphoreGive(sensorMutex);
    }

    time_t now = time(nullptr);
    if (now > 1700000000) {
        epochLocal = (uint32_t)now;
    } else if (epochLocal <= 1700000000) {
        epochLocal = 0;
    }

    uint16_t water_level = (uint16_t)(lvlLocal * 10);
    int32_t  latInt      = (int32_t)(latLocal * 1000000.0f);
    int32_t  lngInt      = (int32_t)(lngLocal * 1000000.0f);
    uint32_t latU        = (uint32_t)latInt;
    uint32_t lngU        = (uint32_t)lngInt;
    uint16_t vBatInt     = (uint16_t)(battVLocal * 100);
    int16_t  temperature = (int16_t)(tempLocal * 10);
    uint16_t humidity    = (uint16_t)(humLocal * 10);
    int16_t  gyro_x      = (int16_t)(gxLocal * 100);
    int16_t  gyro_y      = (int16_t)(gyLocal * 100);

    out26[0]  = (uint8_t)station_id;
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
    out26[13] = (uint8_t)battPctLocal;
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

void saveNextFCntBeforeSend()
{
    MibRequestConfirm_t mibReq;
    mibReq.Type = MIB_UPLINK_COUNTER;
    if (LoRaMacMibGetRequestConfirm(&mibReq) == LORAMAC_STATUS_OK) {
        uint32_t currentFcnt = mibReq.Param.UpLinkCounter;
        saveFCnt(currentFcnt + 1);
    }
}

uint16_t getFlashBacklogCount()
{
    if (!LittleFS.exists(FLASH_BACKLOG_FILE)) return 0;
    File f = LittleFS.open(FLASH_BACKLOG_FILE, "r");
    if (!f) return 0;

    uint16_t count = 0;
    if (f.read((uint8_t *)&count, sizeof(count)) != sizeof(count)) count = 0;
    f.close();

    if (count > FLASH_BACKLOG_MAX) {
        LittleFS.remove(FLASH_BACKLOG_FILE);
        return 0;
    }
    return count;
}

void appendPacketToFlash(const uint8_t *payload26)
{
    uint16_t existingCount = 0;

    if (LittleFS.exists(FLASH_BACKLOG_FILE)) {
        File f = LittleFS.open(FLASH_BACKLOG_FILE, "r");
        if (f) {
            f.read((uint8_t *)&existingCount, sizeof(existingCount));
            if (existingCount > FLASH_BACKLOG_MAX) existingCount = 0;
            for (uint16_t i = 0; i < existingCount; i++) {
                f.read(backlogQueue[i].payload, PAYLOAD_TOTAL_SIZE);
            }
            f.close();
        }
    }

    if (existingCount >= FLASH_BACKLOG_MAX) {
        Serial.println("[FLASH] คิวเต็ม (720) -> ทิ้งแพ็กเก็ตเก่าสุด 1 รายการ");
        for (uint16_t i = 0; i < FLASH_BACKLOG_MAX - 1; i++) {
            backlogQueue[i] = backlogQueue[i + 1];
        }
        existingCount = FLASH_BACKLOG_MAX - 1;
    }

    memcpy(backlogQueue[existingCount].payload, payload26, PAYLOAD_TOTAL_SIZE);
    existingCount++;

    File f = LittleFS.open(FLASH_BACKLOG_FILE, "w");
    if (!f) {
        Serial.println("[FLASH] ERROR: เปิดไฟล์เขียนไม่ได้");
        return;
    }

    f.write((uint8_t *)&existingCount, sizeof(existingCount));
    for (uint16_t i = 0; i < existingCount; i++) {
        f.write(backlogQueue[i].payload, PAYLOAD_TOTAL_SIZE);
    }
    f.close();

    Serial.printf("[FLASH] บันทึกสะสมลง Flash สำเร็จ (รวมข้อมูลตกค้างทั้งหมด: %d packet)\n", existingCount);
}

void loadBacklogFromFlash()
{
    backlogCount = 0;
    backlogHead  = 0;

    if (!LittleFS.exists(FLASH_BACKLOG_FILE)) {
        Serial.println("[FLASH] ไม่มีข้อมูลตกค้างใน Flash (0 packet)");
        return;
    }

    File f = LittleFS.open(FLASH_BACKLOG_FILE, "r");
    if (!f) return;

    uint16_t savedCount = 0;
    f.read((uint8_t *)&savedCount, sizeof(savedCount));

    if (savedCount == 0 || savedCount > FLASH_BACKLOG_MAX) {
        f.close();
        LittleFS.remove(FLASH_BACKLOG_FILE);
        return;
    }

    for (uint16_t i = 0; i < savedCount; i++) {
        if (f.read(backlogQueue[i].payload, PAYLOAD_TOTAL_SIZE) != PAYLOAD_TOTAL_SIZE) {
            savedCount = i;
            break;
        }
    }
    f.close();

    backlogHead  = 0;
    backlogCount = savedCount;
    Serial.printf("[FLASH] โหลดข้อมูลตกค้างจาก Flash ทั้งหมด %d packet เข้า RAM สำเร็จ\n", backlogCount);
}

void saveRemainingBacklogToFlash()
{
    if (backlogCount == 0) {
        if (LittleFS.exists(FLASH_BACKLOG_FILE)) {
            LittleFS.remove(FLASH_BACKLOG_FILE);
            Serial.println("[FLASH] Backlog ว่างแล้ว (0 packet) -> ลบไฟล์ Flash เรียบร้อย");
        }
        return;
    }

    File f = LittleFS.open(FLASH_BACKLOG_FILE, "w");
    if (!f) {
        Serial.println("[FLASH] ERROR: เปิดไฟล์เขียนไม่ได้");
        return;
    }

    f.write((uint8_t *)&backlogCount, sizeof(backlogCount));
    for (uint16_t i = 0; i < backlogCount; i++) {
        uint16_t idx = (backlogHead + i) % BACKLOG_QUEUE_CAPACITY;
        f.write(backlogQueue[idx].payload, PAYLOAD_TOTAL_SIZE);
    }
    f.close();
    Serial.printf("[FLASH] ตัดยอดสำเร็จ: บันทึกคงเหลือค้างใน Flash จริง %d packet\n", backlogCount);
}

// ================= CONFIRMED UPLINK =================
#define CONFIRMED_ACK_TIMEOUT_MS   5000  // ประหยัดแบตเตอรี่: หน้าต่าง RX1/RX2 จบภายใน 2.5 วินาที รอสูงสุด 5 วินาทีเพียงพอ 100%
#define BACKLOG_SEND_SPACING_MS    3000  // เว้นระยะห่างระหว่างแต่ละ packet 3 วินาที

volatile bool waitingForAck     = false;
volatile bool lastConfirmedAcked = false;

void downLinkAckHandle()
{
    Serial.println("[LORAWAN] *** ได้รับ ACK จาก Gateway สำเร็จ ***");
    if (waitingForAck) {
        lastConfirmedAcked = true;
        waitingForAck       = false;
    }
}

bool sendConfirmedAndWait(const uint8_t *payload, uint8_t size, uint32_t ackTimeoutMs)
{
    memcpy(appData, payload, size);
    appDataSize = size;

    lastConfirmedAcked = false;
    waitingForAck        = true;

    // บังคับล็อค DataRate ให้เป็น DR 2 (SF10) ทุกตัวแปรในไลบรารี Heltec และ LoRaMac
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

    LoRaWAN.send();
    Serial.printf("[LORAWAN] ส่ง uplink (%d bytes) ที่ DR %d รอ ACK...\n", size, currentDrForNoAdr);

    unsigned long waitStart = millis();
    while (waitingForAck && (millis() - waitStart) < ackTimeoutMs) {
        LoRaWAN.sleep(loraWanClass);
        delay(10);
    }

    if (waitingForAck) {
        Serial.println("[LORAWAN] หมดเวลารอ ACK (5 วินาที) -> ไม่ได้รับ ACK ในรอบนี้");
    }

    waitingForAck = false;
    return lastConfirmedAcked;
}

// ================= ESP32-C3 BRIDGE =================
bool parseESP32Payload(String payload, float lastKnownLevel,
                        float &outLevel, float &outGX, float &outGY, String &outTilt,
                        float &outTotalTilt, float &outCorrectedLevel)
{
    payload.trim();
    if (payload.length() == 0) return false;

    int idx1 = payload.indexOf(',');
    int idx2 = payload.indexOf(',', idx1 + 1);
    int idx3 = payload.indexOf(',', idx2 + 1);

    if (idx1 < 0 || idx2 < 0 || idx3 < 0) return false;

    float  newLevel = payload.substring(0, idx1).toFloat();
    float  newGX    = payload.substring(idx1 + 1, idx2).toFloat();
    float  newGY    = payload.substring(idx2 + 1, idx3).toFloat();
    String newTilt  = payload.substring(idx3 + 1);

    if (newLevel >= LVL_MIN_VALID && newLevel < LVL_MAX_VALID) {
        outLevel = newLevel;
    } else {
        outLevel = lastKnownLevel;
    }

    outGX   = newGX;
    outGY   = newGY;
    outTilt = newTilt;

    float total        = sqrt(outGX * outGX + outGY * outGY);
    float tiltClamped  = min(total, 90.0f);
    float tiltRad       = tiltClamped * (PI / 180.0f);

    outTotalTilt      = total;
    outCorrectedLevel = outLevel * cos(tiltRad);

    return true;
}

bool readFromESP32(float lastKnownLevel, uint8_t &failCountRef, bool &linkLostRef,
                    float &outLevel, float &outGX, float &outGY, String &outTilt,
                    float &outTotalTilt, float &outCorrectedLevel)
{
    while (ESP32Serial.available()) ESP32Serial.read();

    ESP32Serial.println("REQ");

    String line = "";
    unsigned long startWait = millis();

    while (millis() - startWait < 3000) {
        while (ESP32Serial.available()) {
            char c = (char)ESP32Serial.read();
            if (c == '\n') {
                if (parseESP32Payload(line, lastKnownLevel, outLevel, outGX, outGY,
                                       outTilt, outTotalTilt, outCorrectedLevel)) {
                    ESP32Serial.println("ACK");
                    failCountRef = 0;
                    linkLostRef  = false;
                    return true;
                }
                line = "";
            } else if (c != '\r') {
                line += c;
                if (line.length() > 60) line = "";
            }
        }
        delay(1);
    }

    failCountRef++;
    outTilt = "NO_LINK";
    if (failCountRef >= ESP32_MAX_FAIL) {
        linkLostRef = true;
    }
    return false;
}

// ================= อ่านเซนเซอร์ 1 รอบเต็ม =================
void readSensorsQuick()
{
    static uint8_t esp32FailCountLocal   = 0;
    static bool    esp32LinkLostLocal    = false;
    static float   lastValidLevel        = 0.0f;
    static float   lastGX = 0.0f, lastGY = 0.0f;
    static float   lastTotalTilt = 0.0f, lastCorrectedLevel = 0.0f;
    static String  lastTiltStatus = "OK";
    static float   tempLocal = NAN, humLocal = NAN;

    // 1. อ่าน GPS (พิกัด + NMEA RMC วันเวลา)
    String   nmeaLine = "";
    uint32_t rmcEpoch = 0;
    unsigned long startWait = millis();

    while (millis() - startWait < 2500) {
        while (Serial1.available()) {
            char c = Serial1.read();
            gps.encode(c);

            if (c == '\n') {
                parseNmeaRmc(nmeaLine, rmcEpoch);
                nmeaLine = "";
            } else if (c != '\r') {
                if (nmeaLine.length() < 100) nmeaLine += c;
                else nmeaLine = "";
            }
        }
        delay(1);
    }

    bool     gpsValidLocal = gps.location.isValid();
    float    latLocal      = gpsValidLocal ? gps.location.lat() : currentLat;
    float    lngLocal      = gpsValidLocal ? gps.location.lng() : currentLng;
    uint32_t satsLocal     = gps.satellites.value();

    uint32_t epochLocal = 0;
    if (rmcEpoch > 1700000000) {
        epochLocal = rmcEpoch;
    } else {
        time_t now = time(nullptr);
        epochLocal = (now > 1700000000) ? (uint32_t)now : 0;
    }

    // 2. แบตเตอรี่ (วัดขณะ Relay ON -> ชดเชย Load Sag และกรองด้วย 3-Layer Stabilization)
    float battVoltLocal = readRawBatteryVoltage();
    int   battPctLocal  = processBatterySoC(battVoltLocal, false, false);

    // 3. ESP32-C3
    float  levelLocal      = lastValidLevel;
    float  gxLocal         = lastGX;
    float  gyLocal         = lastGY;
    float  totalTiltLocal  = lastTotalTilt;
    float  correctedLevel  = lastCorrectedLevel;
    String tiltLocal       = lastTiltStatus;

    readFromESP32(lastValidLevel, esp32FailCountLocal, esp32LinkLostLocal,
                  levelLocal, gxLocal, gyLocal, tiltLocal, totalTiltLocal, correctedLevel);

    lastValidLevel      = levelLocal;
    lastGX               = gxLocal;
    lastGY               = gyLocal;
    lastTotalTilt        = totalTiltLocal;
    lastCorrectedLevel   = correctedLevel;
    lastTiltStatus       = tiltLocal;

    // 4. SHT30
    float t = sht31.readTemperature();
    float h = sht31.readHumidity();
    if (!isnan(t) && t > -40.0f && t < 125.0f) tempLocal = t;
    if (!isnan(h) && h >= 0.0f && h <= 100.0f)  humLocal  = h;

    Serial.println(F("---------------------------------------------"));
    Serial.printf ("[SENSOR %lu ms]\n", millis());
    Serial.printf ("  BATT   %.2fV  %d%%\n", battVoltLocal, battPctLocal);
    if (gpsValidLocal) {
        Serial.printf("  GPS    OK   Lat: %.6f  Lng: %.6f  Sats: %lu\n", latLocal, lngLocal, (unsigned long)satsLocal);
    } else {
        Serial.println("  GPS    NO FIX");
    }
    Serial.printf ("  TIME   Epoch: %lu\n", (unsigned long)epochLocal);
    Serial.printf ("  SHT30  T: %.1fC  H: %.1f%%\n", tempLocal, humLocal);
    Serial.printf ("  LEVEL  Raw: %.1fcm  Corrected: %.1fcm\n", levelLocal, correctedLevel);
    Serial.printf ("  GYRO   X: %.1f  Y: %.1f  Total: %.1f  Status: %s\n", gxLocal, gyLocal, totalTiltLocal, tiltLocal.c_str());
    Serial.println(F("---------------------------------------------"));

    // 5. Commit ข้อมูลเข้า Global ภายใต้ Mutex
    if (sensorMutex != NULL && xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
        currentLevel        = levelLocal;
        gyroX                = gxLocal;
        gyroY                = gyLocal;
        tiltStatus           = tiltLocal;
        currentTemp          = tempLocal;
        currentHum           = humLocal;
        currentLat           = latLocal;
        currentLng           = lngLocal;
        gpsValidCached       = gpsValidLocal;
        finalBatteryVoltage  = battVoltLocal;
        batPercentage        = battPctLocal;
        currentEpoch         = epochLocal;
        sensorDataReady       = true;
        xSemaphoreGive(sensorMutex);
    }
}

void sensorTask(void *pvParameters)
{
    for (;;) {
        if (sensorTaskActive) {
            readSensorsQuick();
            vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
        } else {
            vTaskDelay(pdMS_TO_TICKS(SENSOR_IDLE_POLL_MS));
        }
    }
}

// ==============================================================================
// โหมดตรวจเช็คเซนเซอร์ 25 วินาทีเมื่อตื่นจาก Deep Sleep ด้วยปุ่ม PRG (GPIO 0)
// ห้ามส่ง LoRaWAN, ไม่แตะต้อง Flash Backlog, ไม่เปิดสัญญาณวิทยุ, ปิด LED ทั้งหมด
// ==============================================================================
void runPrgInspectionMode()
{
    Serial.println(F("\n================================================================="));
    Serial.println(F("=== [PRG WAKE] ตื่นจาก Deep Sleep เข้าสู่โหมดตรวจเช็ค (25s) ==="));
    Serial.println(F("=== [PRG WAKE] ห้ามส่ง LoRaWAN / ไม่แตะ Flash / ปิด LED ทุกดวง  ==="));
    Serial.println(F("================================================================="));

    time_t wakeEpoch = time(nullptr);
    Serial.printf("[PRG] เวลาตื่นปัจจุบัน Epoch:%lu | เป้าหมายรอบถัดไป Epoch:%lu\n",
                  (unsigned long)wakeEpoch, (unsigned long)rtcCycleTargetEpoch);

    VextON();
    delay(100);

    analogReadResolution(12);

    // 1. วัด No-Load OCV แบตเตอรี่ก่อนเปิด Relay 26
    float noLoadV   = readRawBatteryVoltage();
    int   noLoadPct = processBatterySoC(noLoadV, true, true);
    finalBatteryVoltage = noLoadV;
    batPercentage       = noLoadPct;
    Serial.printf("[BATT] PRG Inspection No-load: %.2fV | SoC: %d%%\n", noLoadV, noLoadPct);

    // 2. ปิดไฟ LED ทั้งหมดเพื่อประหยัดพลังงาน
    pinMode(LED_RED_PIN,   OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    digitalWrite(LED_RED_PIN,   LOW);
    digitalWrite(LED_GREEN_PIN, LOW);

    // 3. เปิด Relay 26 เลี้ยงวงจรเซนเซอร์
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);

    // 4. เริ่มต้นการสื่อสารกับเซนเซอร์
    Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    ESP32Serial.begin(ESP32_BAUD, SERIAL_8N1, ESP32_RX_PIN, ESP32_TX_PIN);

    Wire.begin(17, 18);
    I2CSHT.begin(SHT30_SDA, SHT30_SCL);
    delay(50);
    sht31.begin(0x44);

    pinMode(OLED_RESET, OUTPUT);
    digitalWrite(OLED_RESET, LOW);
    delay(10);
    digitalWrite(OLED_RESET, HIGH);
    delay(10);

    if (oled.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) {
        oled.clearDisplay();
        oled.setTextColor(SSD1306_WHITE);
        oled.setTextSize(1);
    }
    isScreenOn = true;

    // 5. ตั้งค่าปุ่มกด PRG สำหรับกดซ้ำเพื่อขยายเวลาตรวจเช็ค
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    prgButtonPressed = false;
    lastPrgInterruptTime = millis();
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), prgButtonISR, FALLING);

    // 6. เริ่มต้น Task อ่านเซนเซอร์บน Core 0
    if (sensorMutex == NULL) {
        sensorMutex = xSemaphoreCreateMutex();
    }
    sensorTaskActive = true;
    if (sensorTaskHandle == NULL) {
        xTaskCreatePinnedToCore(
            sensorTask,
            "SensorTask",
            8192,
            NULL,
            1,
            &sensorTaskHandle,
            0
        );
    }

    // 7. เข้าสู่โหมดตรวจเช็ค 25 วินาที
    isInspectionMode    = true;
    inspectionStartTime = millis();
    currentStatus       = "Check...";
    lastDisplayUpdate   = 0;

    Serial.println(F("[PRG] เริ่มนับถอยหลังตรวจเช็ค 25 วินาทีบนหน้าจอ OLED..."));

    while (millis() - inspectionStartTime < INSPECTION_DURATION_MS) {
        // หากผู้ใช้กดปุ่ม PRG ซ้ำขณะกำลังตรวจเช็ค -> รีเซ็ตเวลากลับเป็น 25 วินาทีใหม่
        if (prgButtonPressed) {
            prgButtonPressed = false;
            inspectionStartTime = millis();
            Serial.println(F("[PRG] กดปุ่มซ้ำ -> รีเซ็ตเวลานับถอยหลัง 25 วินาทีใหม่"));
        }

        if (millis() - lastDisplayUpdate >= 1000) {
            lastDisplayUpdate = millis();
            updateDisplay();
        }
        delay(20);
    }

    // 8. ครบเวลาตรวจเช็ค 25 วินาที
    Serial.println(F("=== [PRG WAKE] จบการตรวจเช็ค 25 วินาที ==="));

    isInspectionMode = false;
    isScreenOn       = false;
    sensorTaskActive = false;
    if (sensorTaskHandle != NULL) {
        vTaskDelete(sensorTaskHandle);
        sensorTaskHandle = NULL;
    }
    oled.ssd1306_command(SSD1306_DISPLAYOFF);
    digitalWrite(RELAY_PIN, LOW); // ตัดไฟเซนเซอร์ทันที

    // 9. คำนวณเวลานอนที่เหลือตามเป้าหมาย rtcCycleTargetEpoch
    time_t nowEpoch = time(nullptr);
    int64_t remainingSleepSec = 0;

    if (rtcCycleTargetEpoch > 0) {
        remainingSleepSec = (int64_t)rtcCycleTargetEpoch - (int64_t)nowEpoch;
    } else {
        remainingSleepSec = 555;
    }

    Serial.printf("[PRG] สรุปเวลา: ปัจจุบัน Epoch:%lu | เป้าหมาย Epoch:%lu | คงเหลือ %lld วินาที (%.2f นาที)\n",
                  (unsigned long)nowEpoch, (unsigned long)rtcCycleTargetEpoch,
                  remainingSleepSec, (float)remainingSleepSec / 60.0f);

    if (remainingSleepSec > 2) {
        // เหลือเวลา -> กลับเข้าสู่ Deep Sleep ต่อจนครบเวลาเป้าหมายจริง (Drift = 0s)
        Serial.printf("[PRG] กลับเข้าสู่ Deep Sleep ต่ออีก %lld วินาที (%.2f นาที)...\n",
                      remainingSleepSec, (float)remainingSleepSec / 60.0f);
        Radio.Sleep();
        esp_sleep_enable_timer_wakeup((uint64_t)remainingSleepSec * 1000000ULL);
        esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_PIN, 0);
        Mcu.addwakeio((uint8_t)BUTTON_PIN);
        VextOFF();
        esp_deep_sleep_start();
    } else {
        // เวลาหมดพอดีหรือเกินรอบส่งแล้ว (Q2: A) -> ออกจากฟังก์ชัน ให้ setup() ทำงานต่อเพื่อส่ง LoRaWAN ปกติ
        Serial.println(F("[PRG] ถึงเวลาเริ่มรอบส่ง LoRaWAN แล้ว -> เปลี่ยนผ่านเข้าสู่การส่งข้อมูลทันที"));
        return;
    }
}

void setup()
{
    Serial.begin(115200);

    // ตรวจจับการตื่นจาก Deep Sleep ด้วยปุ่ม PRG ทันทีที่บรรทัดแรก
    esp_sleep_wakeup_cause_t wakeupReason = esp_sleep_get_wakeup_cause();
    if (wakeupReason == ESP_SLEEP_WAKEUP_EXT0 || wakeupReason == ESP_SLEEP_WAKEUP_GPIO) {
        runPrgInspectionMode();
    } else if (wakeupReason != ESP_SLEEP_WAKEUP_TIMER) {
        // เป็นการเปิดเครื่องใหม่ หรือกดปุ่ม Reset ฮาร์ดแวร์ (Cold boot / HW Reset)
        rtcCycleTargetEpoch = 0;
        Serial.println(F("[SYSTEM] ฮาร์ดแวร์ Reset หรือเปิดเครื่องใหม่ -> รีเซ็ตจับเวลาเริ่ม Cycle ใหม่"));
    }

    if (!LittleFS.begin(true)) {
        Serial.println("[FLASH] LittleFS mount FAILED");
    } else {
        Serial.println("[FLASH] LittleFS mounted OK");
        uint16_t stored = getFlashBacklogCount();
        Serial.printf("[FLASH] ตรวจสอบความจำ: พบข้อมูลค้างใน Flash %d packet\n", stored);
    }

    Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    VextON();
    delay(100);

    analogReadResolution(12);

    // วัด No-Load OCV แบตเตอรี่ก่อนเปิด Relay (ก่อนจ่ายไฟให้วงจรและเซนเซอร์)
    float preRelayV   = readRawBatteryVoltage();
    int   preRelayPct = processBatterySoC(preRelayV, true, true);
    finalBatteryVoltage = preRelayV;
    batPercentage       = preRelayPct;
    Serial.printf("[BATT] Pre-relay No-load OCV: %.2fV | SoC: %d%%\n", preRelayV, preRelayPct);

    pinMode(RELAY_PIN,    OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);  // เปิดจ่ายไฟให้วงจรและเซนเซอร์ตั้งแต่เริ่ม
    relayPowerOnTime = millis();

    pinMode(LED_RED_PIN,  OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    digitalWrite(LED_RED_PIN,  HIGH);
    digitalWrite(LED_GREEN_PIN, LOW);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), prgButtonISR, FALLING);
    screenTimer = millis();

    overTheAirActivation = true;
    Serial.println("[BOOT] Mode: OTAA");

    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

    // บังคับ Channel 1 (923.4MHz) ให้ใช้ DR 2 (SF10) ถาวร
    LoRaMacChannelAdd(0, (ChannelParams_t){923400000, 0, {((DR_2 << 4) | DR_2)}, 0});

    {
        uint32_t savedFcnt = loadFCnt();
        MibRequestConfirm_t mibReq;
        mibReq.Type = MIB_UPLINK_COUNTER;
        mibReq.Param.UpLinkCounter = savedFcnt;
        LoRaMacMibSetRequestConfirm(&mibReq);
        Serial.printf("[BOOT] FCnt set to: %lu\n", savedFcnt);
    }

    Wire.begin(17, 18);

    I2CSHT.begin(SHT30_SDA, SHT30_SCL);
    delay(50);
    if (sht31.begin(0x44)) {
        delay(20);
        sht31.readTemperature();
        sht31.readHumidity();
        delay(50);
        Serial.println("SHT30 READY");
    }

    pinMode(OLED_RESET, OUTPUT);
    digitalWrite(OLED_RESET, LOW);
    delay(10);
    digitalWrite(OLED_RESET, HIGH);
    delay(10);

    if (oled.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) {
        oled.clearDisplay();
        oled.setTextColor(SSD1306_WHITE);
        oled.setTextSize(2);
        oled.setCursor(10, 20);
        oled.println("SYSTEM OK!");
        oled.display();
        delay(1000);
    }

    ESP32Serial.begin(ESP32_BAUD, SERIAL_8N1, ESP32_RX_PIN, ESP32_TX_PIN);

    if (sensorMutex == NULL) {
        sensorMutex = xSemaphoreCreateMutex();
    }
    sensorTaskActive = true;
    if (sensorTaskHandle == NULL) {
        xTaskCreatePinnedToCore(
            sensorTask,
            "SensorTask",
            8192,
            NULL,
            1,
            &sensorTaskHandle,
            0
        );
        Serial.println("[SENSOR TASK] Started on core 0");
    }
}

void loop()
{
    // ================================================================
    // 1. ตัวจัดการ JOIN CYCLE — ทำงานต่อเนื่อง ไม่ขึ้นกับสถานะ Library
    // ================================================================
    if (joinTimerStarted && !joinSleepPhase) {
        unsigned long jElapsed = millis() - joinCycleStartTime;

        if (isScreenOn && millis() - lastDisplayUpdate > 1000) {
            lastDisplayUpdate = millis();
            updateDisplay();
        }

        // ครบ 30 วินาที: ออฟไลน์ -> เซฟสะสมลง Flash ทันที
        if (!joinFallbackStarted && jElapsed >= JOIN_ATTEMPT_TIMEOUT_MS) {
            joinFallbackStarted = true;
            currentStatus = "No Join...";
            updateDisplay();
            Serial.println("[JOIN] 30s Timeout -> Gateway ออฟไลน์: บันทึกข้อมูลสะสมลง Flash ทันที");

            if (!sensorDataReady) {
                unsigned long wStart = millis();
                while (!sensorDataReady && millis() - wStart < 3000) {
                    delay(50);
                }
            }

            uint8_t payload26[PAYLOAD_TOTAL_SIZE];
            buildSensorPayload(payload26);
            appendPacketToFlash(payload26);
            sensorTaskActive = false;
        }

        // ครบ 45 วินาที: จบ Active Phase -> ดับจอ, ตัดไฟ Relay ขา 26, ดับ LED, เข้าสู่ Sleep 9.25 นาที
        if (jElapsed >= JOIN_CYCLE_TOTAL_MS) {
            joinSleepPhase     = true;
            joinSleepStartTime = millis();
            isActivePhase      = false;
            isScreenOn         = false;
            sensorTaskActive   = false;
            oled.ssd1306_command(SSD1306_DISPLAYOFF);
            currentStatus = "Sleep";

            digitalWrite(RELAY_PIN, LOW);     // ตัดไฟวงจรภายนอกและเซนเซอร์
            digitalWrite(LED_RED_PIN, LOW);   // ดับไฟ LED สีแดง
            digitalWrite(LED_GREEN_PIN, LOW); // ดับไฟ LED สีเขียว
            deviceState = DEVICE_STATE_SLEEP;
            Serial.printf("[JOIN] จบ Active Phase (%lu s) -> ตัดไฟวงจรภายนอก, ดับ LED -> เข้าโหมด Sleep (%.2f นาที)\n",
                          (unsigned long)(ACTIVE_DURATION / 1000UL),
                          (float)(JOIN_SLEEP_MS) / 60000.0f);
        }
    }

    // นับเวลา Sleep เมื่อ Gateway ออฟไลน์
    if (joinSleepPhase) {
        if (millis() - joinSleepStartTime >= JOIN_SLEEP_MS) {
            joinSleepPhase      = false;
            joinTimerStarted    = false;
            joinFallbackStarted = false;
            sensorDataReady     = false;
            digitalWrite(LED_RED_PIN, HIGH);  // เปิดไฟ LED แสดงสถานะ Active
            deviceState         = DEVICE_STATE_JOIN;
            Serial.printf("[JOIN] ครบระยะ Sleep (%.2f นาที) -> ตื่นจาก Sleep -> เริ่มรอบใหม่อัตโนมัติ\n",
                          (float)(JOIN_SLEEP_MS) / 60000.0f);
        }
    }

    // เคลียร์ Join State เฉพาะตอน Join สำเร็จจริง
    if (deviceState == DEVICE_STATE_SEND && joinTimerStarted) {
        Serial.println("[JOIN] *** Join สำเร็จ *** -> เคลียร์ Join Timer");
        joinTimerStarted    = false;
        joinFallbackStarted = false;
        joinSleepPhase      = false;
        sensorDataReady     = false;
        sensorTaskActive    = true;
    }

    // ================================================================
    // 2. MAIN STATE MACHINE
    // ================================================================
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
            if (!joinTimerStarted) {
                newCycleSoCAllowed = true;
                // วัด No-Load OCV ก่อนเปิด Relay
                float noLoadV = readRawBatteryVoltage();
                int   noLoadPct = processBatterySoC(noLoadV, true, true);
                finalBatteryVoltage = noLoadV;
                batPercentage       = noLoadPct;

                digitalWrite(RELAY_PIN, HIGH);    // เปิดไฟเลี้ยงวงจรเซนเซอร์รอบใหม่
                digitalWrite(LED_RED_PIN, HIGH);  // เปิดไฟ LED สีแดง
                isScreenOn = true;
                screenTimer = millis();
                activeStartTime = millis();
                isActivePhase   = true;
                oled.ssd1306_command(SSD1306_DISPLAYON);

                joinCycleStartTime  = millis();
                joinTimerStarted    = true;
                joinFallbackStarted = false;
                sensorDataReady     = false;
                sensorTaskActive    = true;
                saveFCnt(0);

                uint16_t storedFlash = getFlashBacklogCount();
                Serial.printf("[JOIN] เริ่มต้น Cycle ใหม่ | ข้อมูลรอส่งใน Flash: %d packet\n", storedFlash);
                Serial.println("[JOIN] ส่ง Join-Request (1 ครั้งต่อรอบ)");
                LoRaWAN.join();
            }

            currentStatus = joinFallbackStarted ? "No Join..." : "Joining...";
            break;
        }

        case DEVICE_STATE_SEND: {
            if (isInspectionMode) {
                // หากกำลังอยู่ในโหมดตรวจเช็ค 25 วินาที ให้ชะลอการส่งไว้ก่อนจนกว่าจะดูค่าครบ
                pendingTransmission = true;
                deviceState = DEVICE_STATE_SLEEP;
                break;
            }

            newCycleSoCAllowed = true;
            digitalWrite(RELAY_PIN, HIGH);

            // รอให้วงจรเซนเซอร์, ESP32-C3, GPS, SHT30 อุ่นเครื่องจนค่าเสถียรแน่นอน (การันตีจ่ายไฟอย่างน้อย 5 วินาที)
            Serial.println("[SEND] ตรวจสอบความพร้อมเซนเซอร์ (การันตีเปิดไฟอุ่นเครื่องอย่างน้อย 5 วินาที)...");
            while (millis() - relayPowerOnTime < 5000UL) {
                delay(100);
            }

            // 1. อ่านค่าเซนเซอร์รอบปัจจุบันก่อนเสมอ (รับประกันว่า sensorTask มีข้อมูลพร้อมใช้งาน 100%)
            if (!sensorDataReady) {
                unsigned long waitStart = millis();
                while (!sensorDataReady && millis() - waitStart < SENSOR_FIRST_READ_TIMEOUT_MS) {
                    delay(50);
                }
            }

            uint8_t currentPayload[PAYLOAD_TOTAL_SIZE];
            buildSensorPayload(currentPayload);

            // 2. บันทึกข้อมูลปัจจุบันต่อท้ายคิว Flash ทันที (FIFO Safe: บันทึกลงหน่วยความจำก่อนส่งเสมอ ป้องกันข้อมูลหลุด)
            appendPacketToFlash(currentPayload);
            Serial.println("[SEND] บันทึกข้อมูลรอบปัจจุบันต่อท้ายคิว Flash สำเร็จ (FIFO Safe)");

            // 3. โหลดคิวทั้งหมด (ข้อมูลเก่าสะสม + ข้อมูลปัจจุบันล่าสุด) จาก Flash เข้าสู่ RAM
            loadBacklogFromFlash();

            activeStartTime  = millis();
            isActivePhase    = true;
            sensorDataReady  = false;
            sensorTaskActive = true;

            // กำหนดเป้าหมายเวลาของรอบถัดไป (นับ 10 นาทีเต็มจากจุดเริ่ม Active นี้)
            time_t nowEpoch = time(nullptr);
            uint32_t intervalSec = TOTAL_CYCLE_MS / 1000UL;
            if (rtcCycleTargetEpoch == 0) {
                rtcCycleTargetEpoch = (uint32_t)nowEpoch + intervalSec;
                Serial.printf("[CYCLE] เริ่มต้น Cycle แรก (T0 Epoch:%lu) -> กำหนดรอบถัดไปที่ Epoch:%lu (อีก %lu วินาที / %.2f นาที)\n",
                              (unsigned long)nowEpoch, (unsigned long)rtcCycleTargetEpoch,
                              (unsigned long)intervalSec, (float)intervalSec / 60.0f);
            } else {
                if (rtcCycleTargetEpoch + intervalSec < (uint32_t)nowEpoch) {
                    rtcCycleTargetEpoch = (uint32_t)nowEpoch + intervalSec;
                } else {
                    rtcCycleTargetEpoch += intervalSec;
                }
                Serial.printf("[CYCLE] เริ่มต้น Cycle รอบใหม่ (Epoch:%lu) -> กำหนดรอบถัดไปที่ Epoch:%lu (อีก %lu วินาที / %.2f นาที)\n",
                              (unsigned long)nowEpoch, (unsigned long)rtcCycleTargetEpoch,
                              (unsigned long)intervalSec, (float)intervalSec / 60.0f);
            }

            currentStatus = "Sending...";
            isScreenOn    = true;
            screenTimer   = millis();
            oled.ssd1306_command(SSD1306_DISPLAYON);
            updateDisplay();

            for (int i = 0; i < 3; i++) {
                digitalWrite(LED_GREEN_PIN, HIGH); delay(150);
                digitalWrite(LED_GREEN_PIN, LOW);  delay(150);
            }
            digitalWrite(LED_GREEN_PIN, HIGH);

            // 4. ระบายคิวทั้งหมดจากเก่าสุด (หัวแถว) ไปใหม่สุด (ท้ายแถว) อย่างต่อเนื่อง
            bool allBacklogSent = true;

            if (backlogCount > 0) {
                Serial.printf("[MODE] กำลังส่งข้อมูลสะสมต่อเนื่องทั้งหมด %d packet (เรียงจากเก่าไปใหม่)...\n", backlogCount);

                while (backlogCount > 0) {
                    bool acked = false;

                    // ลองส่งแพ็กเก็ตเดิมได้สูงสุด 2 ครั้ง หากไม่ได้ ACK (ประหยัดพลังงาน)
                    for (int retry = 0; retry < 2; retry++) {
                        saveNextFCntBeforeSend();
                        acked = sendConfirmedAndWait(backlogQueue[backlogHead].payload, PAYLOAD_TOTAL_SIZE, CONFIRMED_ACK_TIMEOUT_MS);

                        if (acked) {
                            break; // ได้รับ ACK แล้ว ออกจากลูป retry
                        } else {
                            if (retry < 1) {
                                Serial.printf("[BACKLOG] รอบที่ %d ไม่ได้รับ ACK -> รอ 3 วิแล้วลองใหม่รอบที่ %d...\n", retry + 1, retry + 2);
                                delay(3000);
                            } else {
                                Serial.printf("[BACKLOG] รอบที่ %d ไม่ได้รับ ACK (ครบ 2 ครั้งแล้ว)\n", retry + 1);
                            }
                        }
                    }

                    if (acked) {
                        dequeueBacklog();
                        saveRemainingBacklogToFlash(); // ตัดยอดออกจาก Flash ทันทีทุกแพ็กเก็ตที่ได้รับ ACK (Atomic Commit)
                        Serial.printf("[BACKLOG] ส่งสำเร็จ 1 packet (คงเหลือค้าง %d packet)\n", backlogCount);
                        updateDisplay();

                        if (backlogCount > 0) {
                            delay(BACKLOG_SEND_SPACING_MS); // เว้นระยะ 3 วินาทีระหว่างแพ็กเก็ต
                        }
                    } else {
                        Serial.println("[BACKLOG] NACK ครบ 2 ครั้ง (สัญญาณขาดจริง) → บันทึกค้างใน Flash แล้วหยุดส่งทันที");
                        allBacklogSent = false;
                        break;
                    }
                }
            }

            // 5. จัดการผลลัพธ์หลังเสร็จสิ้นการส่ง
            if (!allBacklogSent) {
                // หาก NACK 2 ครั้ง บันทึกแพ็กเก็ตที่เหลือทั้งหมด (รวมข้อมูลรอบปัจจุบันหากยังส่งไปไม่ถึง) ค้างไว้ใน Flash ต่อไป
                saveRemainingBacklogToFlash();
                Serial.printf("[BACKLOG] NACK ขาดการเชื่อมต่อ -> เซฟคิวที่เหลือลง Flash แล้วตัดเข้า Sleep รอบใหม่ (เหลือค้าง %d packet)\n", backlogCount);
            } else {
                // เมื่อส่งสำเร็จครบถ้วนทุกรายการ (backlogCount == 0) ลบไฟล์ Flash ทิ้งทันที
                if (LittleFS.exists(FLASH_BACKLOG_FILE)) {
                    LittleFS.remove(FLASH_BACKLOG_FILE);
                    Serial.println("[FLASH] ส่งข้อมูลครบทั้งหมดแล้ว -> ลบไฟล์ Flash สำเร็จ");
                }
                Serial.println("[SEND] ส่งข้อมูลสำเร็จครบถ้วนทุกรายการ (คิวว่าง 0 packet)!");
            }

            delay(1000);
            deviceState = DEVICE_STATE_CYCLE;
            break;
        }

        case DEVICE_STATE_CYCLE: {
            digitalWrite(LED_GREEN_PIN, LOW);

            time_t nowEpoch = time(nullptr);
            int64_t remainingSec = 0;
            if (rtcCycleTargetEpoch > 0) {
                remainingSec = (int64_t)rtcCycleTargetEpoch - (int64_t)nowEpoch;
            } else {
                remainingSec = TOTAL_CYCLE_MS / 1000UL;
            }
            if (remainingSec < 10) remainingSec = 10;
            currentTxWait   = (uint32_t)(remainingSec * 1000UL);
            txDutyCycleTime = currentTxWait;

            Serial.printf("[CYCLE] ปัจจุบัน Epoch:%lu | เป้าหมายเริ่มรอบถัดไป Epoch:%lu | เวลาคงเหลือในรอบ %lld วินาที (%.2f นาที)\n",
                          (unsigned long)nowEpoch, (unsigned long)rtcCycleTargetEpoch, remainingSec, (float)remainingSec / 60.0f);

            isScreenOn      = true;
            screenTimer     = millis();
            oled.ssd1306_command(SSD1306_DISPLAYON);
            currentStatus    = "Monitor...";
            sensorDataReady  = false;
            sensorTaskActive = true;

            LoRaWAN.cycle(txDutyCycleTime);
            deviceState = DEVICE_STATE_SLEEP;
            break;
        }

        case DEVICE_STATE_SLEEP: {

            // ดักจับการกดปุ่ม PRG (GPIO 0) ผ่าน Flag One-shot จาก ISR พร้อม Debounce (Q3: A ปลุกจอได้ตลอดเวลา)
            if (prgButtonPressed) {
                prgButtonPressed = false;

                if (isInspectionMode) {
                    // หากกดซ้ำขณะตรวจเช็คอยู่ -> รีเซ็ตเวลากลับเป็น 25 วินาทีใหม่เพื่อยืดเวลาดูต่อ
                    inspectionStartTime = millis();
                    Serial.println(F("[PRG] กดปุ่มซ้ำ -> รีเซ็ตเวลาตรวจเช็คให้นับ 25 วินาทีใหม่"));
                } else {
                    Serial.println(F("=== กดปุ่ม PRG -> ปลุกหน้าจอ OLED ตรวจเช็คเซนเซอร์ (25s) ==="));

                    // 1. อ่านค่าแรงดัน No-Load OCV
                    float noLoadV = readRawBatteryVoltage();
                    int   noLoadPct = processBatterySoC(noLoadV, true, true);
                    finalBatteryVoltage = noLoadV;
                    batPercentage       = noLoadPct;
                    Serial.printf("[BATT] PRG Inspection No-load: %.2fV | SoC: %d%%\n", noLoadV, noLoadPct);

                    // 2. ปิดไฟ LED ทั้งหมดเพื่อประหยัดพลังงาน
                    digitalWrite(LED_RED_PIN, LOW);
                    digitalWrite(LED_GREEN_PIN, LOW);

                    // 3. เปิด Relay 26 จ่ายไฟให้เซนเซอร์ และเปิดหน้าจอ OLED
                    digitalWrite(RELAY_PIN, HIGH);
                    isScreenOn = true;
                    oled.ssd1306_command(SSD1306_DISPLAYON);

                    // 4. เข้าสู่โหมดตรวจเช็ค 25 วินาที
                    isInspectionMode    = true;
                    inspectionStartTime = millis();
                    currentStatus       = "Check...";
                    sensorDataReady     = false;
                    sensorTaskActive    = true;

                    updateDisplay();
                }
            }

            // จัดการการทำงานในโหมดตรวจเช็ค 25 วินาที
            if (isInspectionMode) {
                unsigned long insElapsed = millis() - inspectionStartTime;

                if (insElapsed >= INSPECTION_DURATION_MS) {
                    // ครบ 25 วินาที: สิ้นสุดโหมดตรวจเช็ค
                    isInspectionMode = false;
                    Serial.println(F("=== สิ้นสุดโหมดตรวจเช็ค 25 วินาที ==="));

                    time_t nowEpoch = time(nullptr);
                    if (rtcCycleTargetEpoch > 0 && (uint32_t)nowEpoch >= rtcCycleTargetEpoch) {
                        // ถึงรอบส่ง LoRaWAN พอดีหรือเกินรอบส่งแล้ว (Q2: A) -> เริ่มส่งข้อมูลทันที
                        Serial.println(F("[INSPECTION] ถึง/เกินเป้าหมายรอบส่งแล้ว -> เริ่มการส่งข้อมูลรอบถัดไปทันที"));
                        isScreenOn = false;
                        oled.ssd1306_command(SSD1306_DISPLAYOFF);
                        deviceState = DEVICE_STATE_SEND;
                        break;
                    }

                    if (isActivePhase) {
                        // ยังอยู่ในช่วง Active Phase (45s แรก) ปล่อยให้ Active Phase จัดการต่อ
                        Serial.println(F("[INSPECTION] ยังอยู่ในช่วง Active Phase -> กลับสู่การทำงาน Active ปกติ"));
                    } else {
                        // พ้น Active Phase แล้ว -> ดับจอ ตัดไฟเซนเซอร์ แล้วกลับเข้าสู่ Deep Sleep ต่อเนื่อง
                        isScreenOn = false;
                        oled.ssd1306_command(SSD1306_DISPLAYOFF);
                        digitalWrite(RELAY_PIN, LOW);     // ตัดไฟวงจรเซนเซอร์
                        sensorTaskActive = false;
                        currentStatus    = "Sleep";
                        Serial.println(F("[INSPECTION] ตัดไฟ Relay 26 ดับจอ -> เตรียมเข้าสู่ Deep Sleep ต่อเนื่อง"));
                    }
                } else {
                    if (isScreenOn && millis() - lastDisplayUpdate > 1000) {
                        lastDisplayUpdate = millis();
                        updateDisplay();
                    }
                }
                break;
            }

            // ปั๊ม Radio เฉพาะตอนรอ Join-Accept เท่านั้น
            if (joinTimerStarted && !joinSleepPhase) {
                LoRaWAN.sleep(loraWanClass);
            }

            // ใน Active Phase (45 วินาที): ให้หน้าจอและเซนเซอร์ทำงานต่อเนื่อง ห้ามเข้า Deep Sleep
            if (isActivePhase) {
                // ดับหน้าจอ OLED เมื่อถึงเวลา SCREEN_ON_DURATION_MS (25 วินาที) เพื่อประหยัดแบตเตอรี่
                if (isScreenOn && !isInspectionMode && (millis() - activeStartTime >= SCREEN_ON_DURATION_MS)) {
                    isScreenOn = false;
                    oled.ssd1306_command(SSD1306_DISPLAYOFF);
                    Serial.println("=== ดับหน้าจอ OLED (ครบ 25 วินาที) เพื่อประหยัดพลังงาน | วงจรเซนเซอร์ยังทำงานใน 1 Cycle ===");
                }

                if (millis() - activeStartTime >= ACTIVE_DURATION) {
                    isActivePhase    = false;
                    isScreenOn       = false;
                    sensorTaskActive = false;
                    oled.ssd1306_command(SSD1306_DISPLAYOFF);
                    currentStatus    = "Sleep";
                    screenTimer      = millis();
                    digitalWrite(RELAY_PIN, LOW);     // ตัดไฟวงจรภายนอกและเซนเซอร์
                    digitalWrite(LED_RED_PIN, LOW);   // ดับไฟ LED สีแดง
                    digitalWrite(LED_GREEN_PIN, LOW); // ดับไฟ LED สีเขียว
                    Serial.printf("=== สิ้นสุด Active Phase (%lu s) -> ดับจอ, ตัดไฟวงจรเซนเซอร์, ดับ LED ===\n",
                                  (unsigned long)(ACTIVE_DURATION / 1000UL));
                } else {
                    if (isScreenOn && millis() - lastDisplayUpdate > 1000) {
                        lastDisplayUpdate = millis();
                        updateDisplay();
                    }
                }
                break;
            }

            // พ้น 45 วินาทีแล้ว จึงเข้าสู่ Deep Sleep สำหรับเวลาที่เหลือจนถึง rtcCycleTargetEpoch
            esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_PIN, 0);
            Mcu.addwakeio((uint8_t)BUTTON_PIN);

            time_t nowEpoch = time(nullptr);
            int64_t remainingSleepSec = 0;
            if (rtcCycleTargetEpoch > 0) {
                remainingSleepSec = (int64_t)rtcCycleTargetEpoch - (int64_t)nowEpoch;
            } else {
                remainingSleepSec = (appTxDutyCycle > ACTIVE_DURATION) ? (appTxDutyCycle - ACTIVE_DURATION) / 1000L : 555;
            }

            if (remainingSleepSec > 2) {
                Serial.printf("=== เริ่ม Deep Sleep %lld วินาที (%.2f นาที) จนถึงเป้าหมายรอบถัดไป Epoch:%lu (Drift = 0s) ===\n",
                              remainingSleepSec, (float)remainingSleepSec / 60.0f, (unsigned long)rtcCycleTargetEpoch);
                Radio.Sleep();
                esp_sleep_enable_timer_wakeup((uint64_t)remainingSleepSec * 1000000ULL);
                VextOFF();
                esp_deep_sleep_start();
            } else {
                // ถึงรอบส่งถัดไปพอดี
                Serial.println(F("[SLEEP] ถึงเวลาส่งรอบถัดไปแล้ว -> เริ่มส่งข้อมูลทันที"));
                deviceState = DEVICE_STATE_SEND;
            }

            break;
        }

        default: {
            deviceState = DEVICE_STATE_INIT;
            break;
        }
    }
}
