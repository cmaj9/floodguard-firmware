/*
 * HELTEC V3 LoRaWAN - MASTER DEPLOYMENT MODE (ESP32-C3 Sensor Bridge)
 * 1. ส่ง LoRaWAN ทุกๆ 15 นาที พร้อมเวลานับถอยหลังบนจอ
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

uint32_t appTxDutyCycle      = 900000;    // รอบ 15 นาที
#define  ACTIVE_DURATION      45000UL     // Active phase 45 วินาที

bool    overTheAirActivation = true;
bool    loraWanAdr           = false;     // ปิด ADR เพื่อให้คงค่า DR 2 ตลอดเวลา
bool    isTxConfirmed        = true;
uint8_t appPort              = 2;
uint8_t confirmedNbTrials    = 1;

// กำหนด DataRate ระดับโกลบอลของไลบรารีให้เป็น DR 2 (SF10)
int8_t  loraWanDatarate      = DR_2;

uint32_t currentTxWait = 900000;

// ================= OTAA Join Control =================
#define JOIN_ATTEMPT_TIMEOUT_MS  30000UL   // 30 วิ: ออฟไลน์ -> เซฟลง Flash ทันที
#define JOIN_CYCLE_TOTAL_MS      45000UL   // 45 วิ: จบ Active Phase -> ตัดไฟ Relay เข้า Sleep
#define JOIN_SLEEP_MS            855000UL  // 855 วิ: Sleep 14.25 นาที (45s + 855s = 15 นาที)
#define ACTIVE_BUDGET_MS         38000UL   // เวลาโควตาสูงสุดในการส่งข้อมูลต่อรอบ (38 วิ)

bool          joinTimerStarted    = false;
unsigned long joinCycleStartTime  = 0;
bool          joinFallbackStarted = false;
bool          joinSleepPhase      = false;
unsigned long joinSleepStartTime  = 0;

// ================= ตัวแปรจับเวลา =================
unsigned long activeStartTime    = 0;
unsigned long screenTimer        = 0;
unsigned long lastDisplayUpdate  = 0;
bool          isActivePhase      = false;

const unsigned long SCREEN_TIMEOUT = 20000;

// ================= Hardware Config =================
#define RELAY_PIN     26    // สวิตช์ MOSFET ควบคุมไฟวงจรภายนอก (HIGH=เปิด, LOW=ตัดไฟ)
#define LED_RED_PIN   4
#define LED_GREEN_PIN 5
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    21
#define VEXT_PIN      36
#define BUTTON_PIN    0

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

// ================= Battery =================
const float BAT_R1        = 100000.0f;
const float BAT_R2        =  20000.0f;
const float BAT_DIV_RATIO = (BAT_R1 + BAT_R2) / BAT_R2;
const float BAT_EMPTY_V   = 11.0f;
const float BAT_FULL_V    = 13.6f;

// ================= Background Sensor Task =================
#define SENSOR_READ_INTERVAL_MS      5000
#define SENSOR_IDLE_POLL_MS          500
#define SENSOR_FIRST_READ_TIMEOUT_MS 8000

TaskHandle_t      sensorTaskHandle = NULL;
SemaphoreHandle_t sensorMutex      = NULL;

volatile bool sensorTaskActive = true;
volatile bool sensorDataReady  = false;

void VextON() {
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, LOW);
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
    if      (currentStatus == "Booting...")  shortStatus = "BOOT";
    else if (currentStatus == "Joining...")  shortStatus = "JOIN";
    else if (currentStatus == "Sending...")  shortStatus = "SEND";
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
    if (isActivePhase) {
        unsigned long activeElapsed = millis() - activeStartTime;
        unsigned long remain = (ACTIVE_DURATION > activeElapsed)
                               ? (ACTIVE_DURATION - activeElapsed) / 1000 : 0;
        char timeStr[8];
        sprintf(timeStr, "%02d:%02d", (int)(remain / 60), (int)(remain % 60));
        oled.print(timeStr);
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
                struct timeval tv = { .tv_sec = (time_t)ep, .tv_usec = 0 };
                settimeofday(&tv, NULL);
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

    if (epochLocal <= 1700000000) {
        time_t now = time(nullptr);
        if (now > 1700000000) epochLocal = (uint32_t)now;
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
        LittleFS.remove(FLASH_BACKLOG_FILE);
        return;
    }

    File f = LittleFS.open(FLASH_BACKLOG_FILE, "w");
    if (!f) return;

    f.write((uint8_t *)&backlogCount, sizeof(backlogCount));
    for (uint16_t i = 0; i < backlogCount; i++) {
        uint16_t idx = (backlogHead + i) % BACKLOG_QUEUE_CAPACITY;
        f.write(backlogQueue[idx].payload, PAYLOAD_TOTAL_SIZE);
    }
    f.close();
    Serial.printf("[FLASH] บันทึกแพ็กเก็ตที่ยังส่งไม่หมด %d packet ค้างไว้ใน Flash ต่อไป\n", backlogCount);
}

// ================= CONFIRMED UPLINK =================
#define CONFIRMED_ACK_TIMEOUT_MS   10000
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

    // บังคับส่งด้วย DR 2 (SF10) ทุกครั้งเพื่อความเสถียรของหน้าต่าง RX
    MibRequestConfirm_t mibReq;
    mibReq.Type = MIB_CHANNELS_DATARATE;
    mibReq.Param.ChannelsDatarate = DR_2;
    LoRaMacMibSetRequestConfirm(&mibReq);

    LoRaWAN.send();
    Serial.printf("[LORAWAN] ส่ง uplink (%d bytes) รอ ACK...\n", size);

    unsigned long waitStart = millis();
    while (waitingForAck && (millis() - waitStart) < ackTimeoutMs) {
        LoRaWAN.sleep(loraWanClass);
        delay(10);
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
    static float   smoothedVoltageLocal  = 0.0f;
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

    // 2. แบตเตอรี่
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
    float adcMv             = sum / 11.0f;
    float currentAvgVoltage = (adcMv / 1000.0f) * BAT_DIV_RATIO;

    smoothedVoltageLocal = (smoothedVoltageLocal == 0.0f)
                           ? currentAvgVoltage
                           : (currentAvgVoltage * 0.10f) + (smoothedVoltageLocal * 0.90f);

    float battVoltLocal = smoothedVoltageLocal + calibrationOffset;
    float percent = ((battVoltLocal - BAT_EMPTY_V) / (BAT_FULL_V - BAT_EMPTY_V)) * 100.0f;
    int   battPctLocal  = (int)(constrain(percent, 0.0f, 100.0f) + 0.5f);

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
    Serial.printf ("[CYCLE %lu ms]\n", millis());
    Serial.printf ("  BATT   %.2fV  %d%%\n", battVoltLocal, battPctLocal);
    if (gpsValidLocal) {
        Serial.printf("  GPS    OK   Lat:%.6f  Lng:%.6f  Sats:%lu\n", latLocal, lngLocal, (unsigned long)satsLocal);
    } else {
        Serial.println("  GPS    NO FIX");
    }
    Serial.printf ("  TIME   Epoch:%lu\n", (unsigned long)epochLocal);
    Serial.printf ("  LEVEL  Raw:%.1fcm  Corrected:%.1fcm\n", levelLocal, correctedLevel);
    Serial.printf ("  GYRO   X:%.1f  Y:%.1f  Total:%.1f  Status:%s\n", gxLocal, gyLocal, totalTiltLocal, tiltLocal.c_str());
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

void setup()
{
    Serial.begin(115200);

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

    pinMode(RELAY_PIN,    OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);  // เปิดจ่ายไฟให้วงจรและเซนเซอร์ตั้งแต่เริ่ม

    pinMode(LED_RED_PIN,  OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    digitalWrite(LED_RED_PIN,  HIGH);
    digitalWrite(LED_GREEN_PIN, LOW);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
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

    analogReadResolution(12);

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

    sensorMutex = xSemaphoreCreateMutex();
    sensorTaskActive = true;
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

            if (sensorDataReady) {
                uint8_t payload26[PAYLOAD_TOTAL_SIZE];
                buildSensorPayload(payload26);
                appendPacketToFlash(payload26);
            }
            sensorTaskActive = false;
        }

        // ครบ 45 วินาที: จบ Active Phase -> ดับจอ, ตัดไฟ Relay ขา 26, ดับ LED, เข้าสู่ Sleep 14.25 นาที
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
            Serial.println("[JOIN] จบ Active Phase (45 วินาที) -> ตัดไฟวงจรภายนอก, ดับ LED -> เข้าโหมด Sleep 14.25 นาที");
        }
    }

    // นับเวลา Sleep 14.25 นาที (855 วิ)
    if (joinSleepPhase) {
        if (millis() - joinSleepStartTime >= JOIN_SLEEP_MS) {
            joinSleepPhase      = false;
            joinTimerStarted    = false;
            joinFallbackStarted = false;
            digitalWrite(LED_RED_PIN, HIGH);  // เปิดไฟ LED แสดงสถานะ Active
            deviceState         = DEVICE_STATE_JOIN;
            Serial.println("[JOIN] ครบ 14.25 นาที -> ตื่นจาก Sleep -> เริ่ม Cycle ใหม่อัตโนมัติ");
        }
    }

    // เคลียร์ Join State เฉพาะตอน Join สำเร็จจริง
    if (deviceState == DEVICE_STATE_SEND && joinTimerStarted) {
        Serial.println("[JOIN] *** Join สำเร็จ *** -> เคลียร์ Join Timer");
        joinTimerStarted    = false;
        joinFallbackStarted = false;
        joinSleepPhase      = false;
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
            break;
        }

        case DEVICE_STATE_JOIN: {
            if (!joinTimerStarted) {
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
            digitalWrite(RELAY_PIN, HIGH);

            // หน่วงเวลา 4 วินาทีหลัง Join สำเร็จ เพื่อให้ Gateway & Network Server พร้อมรับ Uplink
            Serial.println("[SEND] รอ Gateway ตั้งค่า Session ให้เสร็จสมบูรณ์ (4 วินาที)...");
            delay(4000);

            // 1. โหลดข้อมูลตกค้างทั้งหมดจาก Flash เข้า RAM
            loadBacklogFromFlash();

            activeStartTime  = millis();
            isActivePhase    = true;
            sensorTaskActive = true;

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

            // 2. ทยอยส่ง Backlog จาก Flash พร้อมระบบ Retry 2 ครั้งต่อแพ็กเก็ต
            if (backlogCount > 0) {
                Serial.printf("[MODE] ต่อ Gateway ติด! กำลังทยอยส่ง Backlog ย้อนหลัง %d packet...\n", backlogCount);

                while (backlogCount > 0 && (millis() - activeStartTime) < ACTIVE_BUDGET_MS) {
                    bool acked = false;

                    // ลองส่งแพ็กเก็ตเดิมได้สูงสุด 2 ครั้ง หากไม่ได้ ACK
                    for (int retry = 0; retry < 2; retry++) {
                        saveNextFCntBeforeSend();
                        acked = sendConfirmedAndWait(backlogQueue[backlogHead].payload, PAYLOAD_TOTAL_SIZE, CONFIRMED_ACK_TIMEOUT_MS);

                        if (acked) {
                            break; // ได้รับ ACK แล้ว ออกจากลูป retry
                        } else {
                            Serial.printf("[BACKLOG] ครั้งที่ %d ไม่ได้รับ ACK -> รอ 3 วิแล้วลองใหม่...\n", retry + 1);
                            delay(3000);
                        }
                    }

                    if (acked) {
                        dequeueBacklog();
                        Serial.printf("[BACKLOG] ส่งสำเร็จ 1 packet (คงเหลือ %d packet)\n", backlogCount);
                        updateDisplay();

                        if (backlogCount > 0 && (millis() - activeStartTime) < ACTIVE_BUDGET_MS) {
                            delay(BACKLOG_SEND_SPACING_MS); // เว้นระยะ 4 วินาทีระหว่างแพ็กเก็ต
                        }
                    } else {
                        Serial.println("[BACKLOG] NACK ครบ 2 ครั้ง (สัญญาณขาดจริง) → หยุดส่ง Backlog รอบนี้");
                        break;
                    }
                }
            }

            // 3. จัดการกรณี Backlog หมด หรือส่งไม่หมด
            if (backlogCount == 0) {
                if (LittleFS.exists(FLASH_BACKLOG_FILE)) {
                    LittleFS.remove(FLASH_BACKLOG_FILE);
                    Serial.println("[FLASH] ส่ง Backlog ย้อนหลังครบทั้งหมดแล้ว -> ลบไฟล์ Flash เรียบร้อย");
                }

                Serial.println("[SEND] Backlog ว่างแล้ว -> เตรียมอ่านและส่งข้อมูลรอบปัจจุบัน...");
                if (!sensorDataReady) {
                    unsigned long waitStart = millis();
                    while (!sensorDataReady && millis() - waitStart < SENSOR_FIRST_READ_TIMEOUT_MS) {
                        delay(50);
                    }
                }

                uint8_t currentPayload[PAYLOAD_TOTAL_SIZE];
                buildSensorPayload(currentPayload);

                saveNextFCntBeforeSend();
                bool curAcked = sendConfirmedAndWait(currentPayload, PAYLOAD_TOTAL_SIZE, CONFIRMED_ACK_TIMEOUT_MS);
                if (curAcked) {
                    Serial.println("[SEND] ส่งข้อมูลรอบปัจจุบันสำเร็จ!");
                } else {
                    Serial.println("[SEND] ส่งข้อมูลปัจจุบันไม่สำเร็จ (NACK) -> บันทึกลง Flash รอส่งรอบหน้า");
                    appendPacketToFlash(currentPayload);
                }
            } else {
                Serial.printf("[SEND] ส่ง Backlog ยังไม่หมด (ค้าง %d packet) -> บันทึกค่าปัจจุบันรอบนี้เพิ่มเข้า Flash\n", backlogCount);

                if (!sensorDataReady) {
                    unsigned long waitStart = millis();
                    while (!sensorDataReady && millis() - waitStart < SENSOR_FIRST_READ_TIMEOUT_MS) {
                        delay(50);
                    }
                }

                uint8_t currentPayload[PAYLOAD_TOTAL_SIZE];
                buildSensorPayload(currentPayload);
                enqueueBacklog(currentPayload);

                saveRemainingBacklogToFlash();
                Serial.printf("[SEND] บันทึกลง Flash สำเร็จ รวมข้อมูลตกค้างทั้งหมด: %d packet\n", backlogCount);
            }

            delay(1000);
            deviceState = DEVICE_STATE_CYCLE;
            break;
        }

        case DEVICE_STATE_CYCLE: {
            digitalWrite(LED_GREEN_PIN, LOW);

            unsigned long elapsed   = millis() - activeStartTime;
            long          remaining = (long)appTxDutyCycle - (long)elapsed;
            if (remaining < 10000L) remaining = 10000L;

            long rndMs = remaining + (long)randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
            if (rndMs < 10000L) rndMs = 10000L;
            currentTxWait   = (uint32_t)rndMs;
            txDutyCycleTime = currentTxWait;

            isScreenOn      = true;
            screenTimer     = millis();
            oled.ssd1306_command(SSD1306_DISPLAYON);
            currentStatus    = "Monitor...";
            sensorTaskActive = true;

            LoRaWAN.cycle(txDutyCycleTime);
            deviceState = DEVICE_STATE_SLEEP;
            break;
        }

        case DEVICE_STATE_SLEEP: {

            // ปั๊ม Radio เฉพาะตอนรอ Join-Accept เท่านั้น
            if (joinTimerStarted && !joinSleepPhase) {
                LoRaWAN.sleep(loraWanClass);
            }

            // ใน Active Phase (45 วินาที): ให้หน้าจอและเซนเซอร์ทำงานต่อเนื่อง ห้ามเข้า Deep Sleep
            if (isActivePhase) {
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
                    Serial.println("=== สิ้นสุด Active Phase (45 วินาที) -> ดับจอ, ตัดไฟวงจรเซนเซอร์, ดับ LED -> เริ่ม Sleep 14.25 นาที ===");
                } else {
                    if (millis() - lastDisplayUpdate > 1000) {
                        lastDisplayUpdate = millis();
                        updateDisplay();
                    }
                }
                break;
            }

            // พ้น 45 วินาทีแล้ว จึงเข้าสู่ Deep Sleep สำหรับเวลาที่เหลือ (~14.25 นาที)
            LoRaWAN.sleep(loraWanClass);

            // ปุ่ม PRG ปลุกจอชั่วคราว
            if (digitalRead(BUTTON_PIN) == LOW) {
                digitalWrite(RELAY_PIN, HIGH);
                digitalWrite(LED_RED_PIN, HIGH);
                isScreenOn      = true;
                isActivePhase   = true;
                sensorTaskActive = true;
                activeStartTime = millis();
                screenTimer     = millis();
                oled.ssd1306_command(SSD1306_DISPLAYON);
                currentStatus = "Monitor...";
                Serial.println("=== กดปุ่ม PRG -> ปลุกจอและจ่ายไฟวงจรชั่วคราว ===");
                delay(200);
            }

            if (isScreenOn && millis() - screenTimer > SCREEN_TIMEOUT) {
                oled.ssd1306_command(SSD1306_DISPLAYOFF);
                isScreenOn = false;
            }

            break;
        }

        default: {
            deviceState = DEVICE_STATE_INIT;
            break;
        }
    }
}
