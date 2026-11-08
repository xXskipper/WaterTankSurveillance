// ============================================
//  ESP32 #1 – SENDER (at tank) with Deep Sleep
//  Heltec WiFi LoRa 32 V4 (ESP32-S3 / SX1262) + JSN-SR04T + Solar
// ============================================
//  Required libraries (Arduino Library Manager):
//    - RadioLib         (by jgromes)
//    - Adafruit SSD1306 (by Adafruit)
//    - Adafruit GFX Library (by Adafruit)
// ============================================
//  Flow (any wakeup):
//    1. OLED on immediately with live progress feedback
//    2. Measure distance with ultrasonic sensor
//    3. Transmit fill level via LoRa
//    4. Button wakeup only: wait up to 3 s for ACK → show link status
//    5. OLED off, deep sleep
// ============================================

// ============================================
//  FEATURE FLAGS  (1 = on, 0 = off)
// ============================================
#define FEATURE_PRESSURE  0   // Measure water-line pressure and send it with the fill level
#define FEATURE_VALVE     0   // Open the solenoid valve automatically (needs FEATURE_PRESSURE)

#define DEBUG_CONTINUOUS  0   // 1 = measure continuously, print to Serial, never sleep (set 0 for normal operation)

#if FEATURE_VALVE && !FEATURE_PRESSURE
#error "FEATURE_VALVE needs FEATURE_PRESSURE"
#endif

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RadioLib.h>
#include <algorithm>

// --- Heltec WiFi LoRa 32 V4 (ESP32-S3) – LoRa Pins ---
#define LORA_SCK    9
#define LORA_MISO   11
#define LORA_MOSI   10
#define LORA_CS     8
#define LORA_RST    12
#define LORA_DIO1   14
#define LORA_BUSY   13

// --- OLED (SSD1306 128x64, I2C) ---
#define OLED_SDA    17
#define OLED_SCL    18
#define OLED_RST    21
#define OLED_ADDR   0x3C
#define SCREEN_W    128
#define SCREEN_H    64

// --- PRG / BOOT button (active LOW) ---
#define BUTTON_PIN  0

// --- Vext: external power rail that feeds the OLED (active LOW) ---
#define VEXT_PIN    36

// --- JSN-SR04T Ultrasonic Sensor ---
#define TRIG_PIN    4
#define ECHO_PIN    5

// --- Tank Parameters (ADJUST to your tank!) ---
#define TANK_HEIGHT_CM    116   // Usable water height (floor → 100 % level) in cm
#define SENSOR_OFFSET_CM    25   // Gap from sensor face to the 100 % level in cm

// --- Deep Sleep Interval ---
#define SLEEP_MINUTES      30   // 30 min
#define US_PER_MINUTE      60000000ULL

// --- Pressure sensor (analog transmitter, ADJUST to your sensor!) ---
#define PRESSURE_ADC_PIN     2      // ADC1 pin (check against the Heltec V4 pinout)
#define PRESSURE_V_MIN       0.5f   // Sensor output at 0 bar (V)
#define PRESSURE_V_MAX       4.5f   // Sensor output at full scale (V)
#define PRESSURE_FULLSCALE   10.0f  // Pressure at PRESSURE_V_MAX (bar)
#define PRESSURE_DIVIDER     1.5f   // Voltage divider ratio (Vsensor / Vadc), e.g. 10k + 20k = 1.5
#define PRESSURE_OPEN_BAR    1.0f   // Minimum pressure to OPEN the valve
#define PRESSURE_KEEP_BAR    0.3f   // Valve closes again when pressure drops below this

// --- Solenoid valve (via MOSFET/relay module, normally closed valve recommended) ---
#define VALVE_PIN            6      // Output pin (check against the Heltec V4 pinout)
#define VALVE_ON             HIGH   // Pin level that opens the valve
#define VALVE_OFF            LOW
#define VALVE_OPEN_LEVEL     30     // Open when fill level <= this (%)
#define VALVE_CLOSE_LEVEL    95     // Close when fill level >= this (%) (blind zone always closes)
#define VALVE_MAX_OPEN_MIN   240    // Safety limit: close after this many minutes (4 h)
#define FILL_CHECK_MINUTES   10     // Wake interval while the valve is open

// --- Sensor averaging ---
#define NUM_MEASUREMENTS   30   // max. shots per measurement
#define SHOT_PAUSE_MS      20   // pause between shots (echo of a ~1 m target is gone after ~6 ms)
#define MEASURE_RETRIES     3   // full measurements before reporting an error
#define TARGET_VALID        4   // stop early once this many valid shots are collected

// Valid sensor range – readings outside this window are rejected as noise.
// Min: the JSN-SR04T never reports less than ~20 cm, even in its blind zone –
//      anything below this is a noise spike on the echo pin (e.g. no sensor).
// Max: a little beyond "tank fully empty" (height + offset + margin).
#define SENSOR_MIN_CM  10
#define SENSOR_MAX_CM  (TANK_HEIGHT_CM + SENSOR_OFFSET_CM + 15)

// Blind zone: the JSN-SR04T is unreliable closer than this → report "tank full".
#define SENSOR_BLIND_CM    25
// Spread check: shots further than SPREAD_TOL_CM from the median are outliers.
// If fewer than MIN_AGREE shots agree, the reading is reported as unstable
// (typical for multi-echoes when the water is too close to the sensor).
#define SPREAD_TOL_CM       3
#define MIN_AGREE           3

// --- Special LoRa payload codes (must match the receiver) ---
#define CODE_UNSTABLE     253
#define CODE_TANK_FULL    254
#define CODE_SENSOR_ERROR 255

enum MeasureResult { MEAS_OK, MEAS_NO_ECHO, MEAS_BLIND, MEAS_UNSTABLE };

// --- LoRa preamble (must match on both boards) ---
// The sender transmits a long preamble so the receiver can run its radio in
// RX duty-cycle mode (sleep most of the time, briefly listen for a preamble)
// instead of staying in continuous RX (~5 mA).
#define LORA_PREAMBLE_SYMBOLS  512   // ≈ 0.52 s airtime at SF7 / 125 kHz

// --- ACK wait after button wakeup ---
#define ACK_TIMEOUT_MS     3000

// --- Battery ADC (Heltec V4: ~390k/100k divider (x4.9) on GPIO 1, enable via GPIO 37) ---
#define BAT_ADC_PIN   1
#define BAT_CTRL_PIN  37
#define BAT_DIVIDER   4.9f   // measured: x2 showed 1.5 V on a charged LiPo

// ============================================
//  State that must survive deep sleep → RTC memory
// ============================================
RTC_DATA_ATTR bool     valveOpen    = false;
RTC_DATA_ATTR bool     valveFault   = false;   // Timeout hit → locked until refilled or button press
RTC_DATA_ATTR uint32_t valveOpenMin = 0;

// ============================================

SPIClass loraSPI(FSPI);
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSPI);
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, OLED_RST);

bool oledReady = false;

volatile bool rxFlag = false;

// ============================================
//  Battery
// ============================================

float readBatteryVoltage() {
    pinMode(BAT_CTRL_PIN, OUTPUT);
    digitalWrite(BAT_CTRL_PIN, HIGH);
    delay(10);
    long sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += analogReadMilliVolts(BAT_ADC_PIN);
        delay(2);
    }
    digitalWrite(BAT_CTRL_PIN, LOW);
    return (sum / 16.0f / 1000.0f) * BAT_DIVIDER;
}

// LiPo discharge curve (resting voltage → state of charge). A LiPo is far from
// linear: it sits on a long plateau around 3.7–3.9 V and only falls off steeply
// at the very top and bottom. Values between table points are interpolated.
// 0 % is set to 3.27 V (practical cut-off, the protection PCB trips lower).
static const float BAT_CURVE[][2] = {   // { volts, percent }
    {4.20f, 100}, {4.15f, 95}, {4.11f, 90}, {4.08f, 85}, {4.02f, 80},
    {3.98f,  75}, {3.95f, 70}, {3.91f, 65}, {3.87f, 60}, {3.85f, 55},
    {3.84f,  50}, {3.82f, 45}, {3.80f, 40}, {3.79f, 35}, {3.77f, 30},
    {3.75f,  25}, {3.73f, 20}, {3.71f, 15}, {3.69f, 10}, {3.61f,  5},
    {3.27f,   0},
};

int batteryPercent(float v) {
    const int n = sizeof(BAT_CURVE) / sizeof(BAT_CURVE[0]);
    if (v >= BAT_CURVE[0][0])     return 100;
    if (v <= BAT_CURVE[n - 1][0]) return 0;
    for (int i = 1; i < n; i++) {
        if (v >= BAT_CURVE[i][0]) {
            float span = BAT_CURVE[i - 1][0] - BAT_CURVE[i][0];
            float frac = (v - BAT_CURVE[i][0]) / span;
            return (int)roundf(BAT_CURVE[i][1] + frac * (BAT_CURVE[i - 1][1] - BAT_CURVE[i][1]));
        }
    }
    return 0;
}

IRAM_ATTR void setRxFlag() { rxFlag = true; }

// ============================================
//  OLED
// ============================================

bool initOLED() {
    // Power on the OLED via Vext (active LOW)
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, LOW);
    delay(50);

    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(20);
    digitalWrite(OLED_RST, HIGH);
    delay(20);
    Wire.begin(OLED_SDA, OLED_SCL);
    oledReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    return oledReady;
}

// Renders one consistent layout to the OLED and pushes it.
//   fillLevel  : -1 = not yet measured, 0-100 = measured value
//   errorText  : non-null → shown large instead of fill level (e.g. "Sen ERR!")
//   status     : short string on the bottom line
//   battV      : battery voltage (V); <0 = omit battery row
void updateDisplay(int fillLevel, const char* errorText, const char* status, float battV = -1.0f) {
    if (!oledReady) return;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // Header
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Tank Sender");
    display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

    if (errorText) {
        display.setTextSize(2);
        display.setCursor(10, 18);
        display.print(errorText);
    } else if (fillLevel < 0) {
        // Not yet measured – just show a waiting message
        display.setTextSize(1);
        display.setCursor(22, 28);
        display.print("Please wait...");
    } else {
        // Fill level (large) + bar
        display.setTextSize(2);
        display.setCursor(0, 13);
        display.print("Fill: ");
        display.print(fillLevel);
        display.print("%");

        int barW = (int)(fillLevel * 126.0f / 100.0f);
        display.drawRect(0, 33, 128, 10, SSD1306_WHITE);
        if (barW > 0) display.fillRect(1, 34, barW, 8, SSD1306_WHITE);
    }

    // Battery row (only when battV has been measured)
    if (battV > 0.0f) {
        char batLine[20];
        snprintf(batLine, sizeof(batLine), "Bat: %.1fV (%d%%)", battV, batteryPercent(battV));
        display.setTextSize(1);
        display.setCursor(0, 44);
        display.print(batLine);
    }

    if (status && status[0]) {
        display.setTextSize(1);
        display.setCursor(0, 55);
        display.print(status);
    }

    display.display();
}

// ============================================
//  Measurement & radio
// ============================================

static long echoMicros() {
    unsigned long t0 = micros();
    while (digitalRead(ECHO_PIN) == LOW) {
        if ((micros() - t0) > 3000) {
            Serial.print(" -> no rising edge");
            return 0;
        }
    }
    // Measure the HIGH pulse (50 ms max ≈ 8.5 m)
    unsigned long t1 = micros();
    Serial.printf(" [rise after %lu us]", t1 - t0);
    while (digitalRead(ECHO_PIN) == HIGH) {
        if ((micros() - t1) > 50000) break;
    }
    return (long)(micros() - t1);
}

// Live feedback during a measurement: shot counter, valid hits and last result.
static void showShotProgress(int shot, int valid, bool hit, float lastCm) {
    if (!oledReady) return;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Measuring");
    display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(0, 14);
    display.printf("Shot %d/%d", shot, NUM_MEASUREMENTS);
    display.setCursor(0, 34);
    display.printf("Hits %d/%d", valid, TARGET_VALID);
    display.setTextSize(1);
    display.setCursor(0, 55);
    if (hit) display.printf("last: %.1f cm", lastCm);
    else     display.print("last: no echo");
    display.display();
}

// Fires NUM_MEASUREMENTS shots, rejects outliers around the median and
// writes the averaged distance to distanceCm (only valid for MEAS_OK).
MeasureResult measureDistance(float &distanceCm) {
    float readings[NUM_MEASUREMENTS];
    int valid = 0, shots = 0;
    for (int i = 0; i < NUM_MEASUREMENTS && valid < TARGET_VALID; i++) {
        shots++;
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(4);
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(50);
        digitalWrite(TRIG_PIN, LOW);

        Serial.printf("  [sensor] shot %d:", i + 1);
        long duration = echoMicros();
        bool lastValidShot = false;
        float lastD = 0;
        if (duration > 0) {
            float d = duration * 0.034f / 2.0f;
            lastD = d;
            if (d >= SENSOR_MIN_CM && d <= SENSOR_MAX_CM) {
                lastValidShot = true;
                Serial.printf(" %ld us -> %.1f cm OK", duration, d);
                readings[valid++] = d;
            } else {
                Serial.printf(" %ld us -> %.1f cm OUT OF RANGE", duration, d);
            }
        }
        Serial.println();
        if (shots % 3 == 0) showShotProgress(shots, valid, duration > 0 && lastValidShot, lastD);   // OLED refresh costs ~25 ms   // no-op while the OLED is off
        delay(SHOT_PAUSE_MS);
    }
    if (valid == 0) {
        Serial.println("  [sensor] valid=0");
        return MEAS_NO_ECHO;
    }

    // Median is robust against single outliers
    std::sort(readings, readings + valid);
    float median = (valid % 2) ? readings[valid / 2]
                               : (readings[valid / 2 - 1] + readings[valid / 2]) / 2.0f;

    // Average only the shots that agree with the median
    float sum = 0;
    int agree = 0;
    for (int i = 0; i < valid; i++) {
        if (fabsf(readings[i] - median) <= SPREAD_TOL_CM) {
            sum += readings[i];
            agree++;
        }
    }
    Serial.printf("  [sensor] valid=%d/%d shots  median=%.1f cm  agree=%d  spread=%.1f cm\n",
                  valid, shots, median, agree, readings[valid - 1] - readings[0]);

    if (median < SENSOR_BLIND_CM) return MEAS_BLIND;
    if (agree < MIN_AGREE)        return MEAS_UNSTABLE;
    distanceCm = sum / agree;
    return MEAS_OK;
}

int calcFillLevel(float distanceCm) {
    float waterHeight = TANK_HEIGHT_CM - (distanceCm - SENSOR_OFFSET_CM);
    return constrain((int)((waterHeight / TANK_HEIGHT_CM) * 100.0f), 0, 100);
}

#if FEATURE_PRESSURE
// Returns the line pressure in bar, or -1 on a sensor fault (e.g. broken wire:
// output below the sensor's 0 bar level).
float readPressureBar() {
    long sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += analogReadMilliVolts(PRESSURE_ADC_PIN);
        delay(2);
    }
    float vSensor = (sum / 16.0f / 1000.0f) * PRESSURE_DIVIDER;
    if (vSensor < PRESSURE_V_MIN - 0.25f) return -1.0f;
    float bar = (vSensor - PRESSURE_V_MIN) / (PRESSURE_V_MAX - PRESSURE_V_MIN) * PRESSURE_FULLSCALE;
    return max(bar, 0.0f);
}
#endif

#if FEATURE_VALVE
// Decides whether the valve should be open from now on. Every uncertain case closes it.
void updateValve(MeasureResult meas, int fillLevel, float pressureBar, bool buttonWakeup) {
    if (buttonWakeup) valveFault = false;
    if (meas == MEAS_OK && fillLevel > VALVE_OPEN_LEVEL) valveFault = false;

    if (valveOpen) {
        valveOpenMin += FILL_CHECK_MINUTES;
        const char* closeReason = nullptr;
        if (meas == MEAS_BLIND)                          closeReason = "tank full";
        else if (meas != MEAS_OK)                        closeReason = "no valid level reading";
        else if (fillLevel >= VALVE_CLOSE_LEVEL)         closeReason = "level reached";
        else if (pressureBar < PRESSURE_KEEP_BAR)        closeReason = "pressure lost";
        else if (valveOpenMin >= VALVE_MAX_OPEN_MIN) {
            closeReason = "time limit";
            valveFault  = true;
        }
        if (closeReason) {
            valveOpen = false;
            Serial.printf("Valve CLOSED: %s (open %u min)\n", closeReason, valveOpenMin);
        }
    } else if (!valveFault && meas == MEAS_OK && fillLevel <= VALVE_OPEN_LEVEL
               && pressureBar >= PRESSURE_OPEN_BAR) {
        valveOpen    = true;
        valveOpenMin = 0;
        Serial.printf("Valve OPENED: level %d%%, pressure %.1f bar\n", fillLevel, pressureBar);
    }
    digitalWrite(VALVE_PIN, valveOpen ? VALVE_ON : VALVE_OFF);
}
#endif

bool waitForAck() {
    rxFlag = false;
    radio.setDio1Action(setRxFlag);
    radio.startReceive();

    unsigned long start = millis();
    while (!rxFlag && (millis() - start) < ACK_TIMEOUT_MS) {
        delay(10);
    }

    if (!rxFlag) {
        radio.standby();
        return false;
    }

    String data;
    int state = radio.readData(data);
    radio.standby();
    return (state == RADIOLIB_ERR_NONE && data.startsWith("ACK"));
}

void goToSleep() {
    pinMode(TRIG_PIN, INPUT);
    pinMode(ECHO_PIN, INPUT);

    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, HIGH);

    // While the valve is open, wake up often to check the fill level
    int sleepMinutes = valveOpen ? FILL_CHECK_MINUTES : SLEEP_MINUTES;

#if FEATURE_VALVE
    // Keep the valve pin at its level during deep sleep
    digitalWrite(VALVE_PIN, valveOpen ? VALVE_ON : VALVE_OFF);
    gpio_hold_en((gpio_num_t)VALVE_PIN);
    gpio_deep_sleep_hold_en();
#endif

    Serial.printf("Sleeping for %d minutes...\n", sleepMinutes);
    Serial.flush();

    esp_sleep_enable_timer_wakeup(sleepMinutes * US_PER_MINUTE);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);

    esp_deep_sleep_start();
}

// ============================================

void setup() {
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);   // never block when no serial monitor is attached

    esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();
    bool buttonWakeup = (wakeReason == ESP_SLEEP_WAKEUP_EXT0);
    Serial.println(buttonWakeup ? "\n=== Sender: button wakeup ===" : "\n=== Sender: timer wakeup ===");

#if FEATURE_VALVE
    // Re-apply the valve level first, then release the sleep hold (no glitch)
    pinMode(VALVE_PIN, OUTPUT);
    digitalWrite(VALVE_PIN, valveOpen ? VALVE_ON : VALVE_OFF);
    gpio_hold_dis((gpio_num_t)VALVE_PIN);
    gpio_deep_sleep_hold_dis();
#endif

    // Vext powers the sensor and OLED from the switched 3.3 V LDO.
    // goToSleep() shuts it off, so the sensor draws nothing during deep sleep.
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, LOW);

    // OLED only on button press
    if (buttonWakeup) {
        initOLED();
        updateDisplay(-1, nullptr, "Booting...");
    }

    // ── Measure ─────────────────────────────────────────────────────────
    if (buttonWakeup) updateDisplay(-1, nullptr, "Measuring...");
    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);
    pinMode(ECHO_PIN, INPUT_PULLDOWN);
    delay(200);

#if DEBUG_CONTINUOUS
    // Debug: measure forever, show on OLED + Serial, no LoRa, no deep sleep
    initOLED();
    while (true) {
        float d = 0;
        updateDisplay(-1, nullptr, "Measuring...");
        unsigned long t0 = millis();
        MeasureResult m = measureDistance(d);
        Serial.printf("  [sensor] took %lu ms\n", millis() - t0);
        const char* names[] = { "OK", "NO_ECHO", "BLIND", "UNSTABLE" };
        char line[24];
        if (m == MEAS_OK) {
            int f = calcFillLevel(d);
            Serial.printf(">>> %s  dist=%.1f cm  fill=%d%%\n", names[m], d, f);
            snprintf(line, sizeof(line), "Dist: %.1f cm", d);
            updateDisplay(f, nullptr, line);
        } else {
            Serial.printf(">>> %s\n", names[m]);
            updateDisplay(-1, m == MEAS_BLIND ? "FULL!" : (m == MEAS_UNSTABLE ? "Unstable" : "No echo"), "Debug mode");
        }
        delay(500);
    }
#endif

    float         distance  = 0;
    MeasureResult meas      = MEAS_NO_ECHO;
    // Weak echoes are common with this sensor → retry a few times before giving up
    for (int attempt = 1; attempt <= MEASURE_RETRIES; attempt++) {
        meas = measureDistance(distance);
        if (meas == MEAS_OK || meas == MEAS_BLIND) break;
        Serial.printf("Measurement attempt %d/%d failed, retrying...\n", attempt, MEASURE_RETRIES);
        delay(500);
    }
    int           fillLevel = 0;
    int           payload   = CODE_SENSOR_ERROR;
    const char*   errorText = nullptr;

    switch (meas) {
        case MEAS_OK:
            fillLevel = calcFillLevel(distance);
            payload   = fillLevel;
            Serial.printf("Distance: %.1f cm  |  Fill: %d%%\n", distance, fillLevel);
            break;
        case MEAS_BLIND:
            payload   = CODE_TANK_FULL;
            errorText = "FULL!";
            Serial.println("Water within sensor blind zone – reporting FULL.");
            break;
        case MEAS_UNSTABLE:
            payload   = CODE_UNSTABLE;
            errorText = "Unstable";
            Serial.println("Readings scatter too much – reporting UNSTABLE.");
            break;
        case MEAS_NO_ECHO:
            errorText = "Sen ERR!";
            Serial.println("ERROR: No valid sensor reading.");
            break;
    }
    // ── Pressure & valve ─────────────────────────────────────────────────
    int valveState = 0;   // 0 = closed, 1 = open, 2 = locked (time limit hit)
#if FEATURE_PRESSURE
    float pressureBar = readPressureBar();
    Serial.printf("Pressure: %.1f bar%s\n", pressureBar, pressureBar < 0 ? " (sensor fault)" : "");
#endif
#if FEATURE_VALVE
    updateValve(meas, fillLevel, pressureBar, buttonWakeup);
    valveState = valveOpen ? 1 : (valveFault ? 2 : 0);
#endif

    if (buttonWakeup) updateDisplay(fillLevel, errorText, "Transmitting...");

    // ── LoRa ─────────────────────────────────────────────────────────────
    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    int  loraState = radio.begin(868.0, 125.0, 7, 5);
    bool loraOk    = (loraState == RADIOLIB_ERR_NONE);
    if (!loraOk) Serial.printf("LoRa init failed: %d\n", loraState);

    bool        txOk        = false;
    bool        linked      = false;
    const char* finalStatus = "LoRa init failed!";

    if (loraOk) {
        String msg  = String(payload);
#if FEATURE_PRESSURE
        // "level;pressure;valve" – the receiver's toInt() still reads the level
        msg += ";" + String(pressureBar, 1) + ";" + String(valveState);
#endif
        txOk       = (radio.transmit(msg) == RADIOLIB_ERR_NONE);
        finalStatus = txOk ? "Transmitted OK" : "Transmit failed!";
        Serial.println(finalStatus);

        if (txOk && buttonWakeup && meas != MEAS_NO_ECHO) {
            updateDisplay(fillLevel, errorText, "Checking link...");
            linked      = waitForAck();
            finalStatus = linked ? "Link: OK" : "Link: No signal";
            Serial.printf("Link: %s\n", linked ? "OK" : "No signal");
        }

        radio.sleep();
    }
    loraSPI.end();

    // ── Final result on OLED (button only) ───────────────────────────────
    if (buttonWakeup) {
        float battV = readBatteryVoltage();
        updateDisplay(fillLevel, errorText, finalStatus, battV);
        delay(5000);
        display.ssd1306_command(SSD1306_DISPLAYOFF);
    }

    goToSleep();
}

void loop() {
    // Never reached – ESP32 sleeps at the end of setup()
}
