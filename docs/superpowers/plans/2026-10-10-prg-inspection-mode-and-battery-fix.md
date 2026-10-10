# Implementation Plan: Deep Sleep PRG Inspection Mode & Battery Fix

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Intercept ESP32 Deep Sleep wake-up at `setup()` to route PRG button presses (`ESP_SLEEP_WAKEUP_EXT0`) directly to `runPrgInspectionMode()` (25s OLED check, Relay 26 ON, LEDs OFF, no LoRaWAN transmission, no Flash check), and resume Deep Sleep for remaining cycle time.

**Architecture:**
1. Store RTC cycle sleep timestamps (`rtcSleepStartRtcMs`, `rtcTargetSleepDurationMs`) before calling `LoRaWAN.sleep()`.
2. In `setup()`, check `esp_sleep_get_wakeup_cause()`. If `EXT0` or `GPIO`, execute `runPrgInspectionMode()`.
3. In `runPrgInspectionMode()`, initialize only sensors & OLED (keep SX1262 LoRa asleep, do not mount backlog). Display `CHK` and countdown `00:25` -> `00:00`.
4. If PRG is pressed again, reset 25s countdown.
5. After 25s, if remaining cycle time > 5s, deep sleep for the remaining duration. Otherwise, return to `setup()` for normal scheduled LoRaWAN transmission.

## Global Constraints
- Target board: Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262)
- Inspect duration: Exactly 25 seconds (`INSPECTION_DURATION_MS = SCREEN_ON_DURATION_MS = 25000UL`)
- PRG button: GPIO 0 (`BUTTON_PIN`)
- Relay switch: GPIO 26 (`RELAY_PIN`), HIGH = sensor power, LOW = power off
- Battery ADC: GPIO 2 (`CUSTOM_BAT_PIN`), Divider Ratio = 6.0
- LED Red: GPIO 4 (`LED_RED_PIN`), LED Green: GPIO 5 (`LED_GREEN_PIN`) - both MUST remain LOW during PRG inspection
- PlatformIO executable path: `C:\Users\Bruger\.platformio\penv\Scripts\pio.exe run -e heltec_wifi_lora_32_V3`

---

### Task 1: Add RTC Sleep Tracking & `runPrgInspectionMode()` Prototype

**Files:**
- Modify: `src/main.cpp` (add RTC tracking variables and `runPrgInspectionMode()` implementation)

- [ ] **Step 1: Declare RTC sleep timing variables**

```cpp
RTC_DATA_ATTR uint64_t rtcSleepStartRtcMs = 0;
RTC_DATA_ATTR uint32_t rtcTargetSleepDurationMs = 0;
```

- [ ] **Step 2: Implement `runPrgInspectionMode()` function**

Implement `runPrgInspectionMode()` before `setup()`:
- Measure No-load OCV before turning on Relay 26.
- Turn on Relay 26, keep LEDs LOW, initialize Wire, SHT30, GPS, ESP32-C3 UART, OLED.
- Run 25-second inspection loop with repeat PRG press reset.
- Calculate remaining sleep duration using `RtcGetTimerValue()`.
- Return to deep sleep if > 5000 ms remains, or return to `setup()` if expired.

- [ ] **Step 3: Branch at beginning of `setup()`**

At the very top of `setup()`:
```cpp
    esp_sleep_wakeup_cause_t wakeupReason = esp_sleep_get_wakeup_cause();
    if (wakeupReason == ESP_SLEEP_WAKEUP_EXT0 || wakeupReason == ESP_SLEEP_WAKEUP_GPIO) {
        runPrgInspectionMode();
    }
```

- [ ] **Step 4: Record sleep timestamps before `LoRaWAN.sleep()` in `DEVICE_STATE_SLEEP`**

In `case DEVICE_STATE_SLEEP:`, before `LoRaWAN.sleep(loraWanClass)`:
```cpp
    rtcSleepStartRtcMs = RtcGetTimerValue();
    rtcTargetSleepDurationMs = (appTxDutyCycle > ACTIVE_DURATION) ? (appTxDutyCycle - ACTIVE_DURATION) : 10000;
```

- [ ] **Step 5: Compile with PlatformIO**

Run: `& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e heltec_wifi_lora_32_V3`
Expected: SUCCESS

- [ ] **Step 6: Commit changes**

```bash
git add src/main.cpp
git commit -m "feat(prg): intercept deep sleep EXT0 wake in setup to run dedicated inspection mode"
```
