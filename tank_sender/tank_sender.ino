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

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RadioLib.h>

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
#define TANK_HEIGHT_CM    300   // Inner height of the tank in cm
#define SENSOR_OFFSET_CM   10   // Gap from sensor face to the top of the tank in cm

// --- Deep Sleep Interval ---
#define SLEEP_MINUTES      10
#define US_PER_MINUTE      60000000ULL
#define SLEEP_DURATION     (SLEEP_MINUTES * US_PER_MINUTE)

// --- Sensor averaging ---
#define NUM_MEASUREMENTS   10

// Valid sensor range – readings outside this window are rejected as noise.
// Min: sensor can't be closer than a few cm to the mount point.
// Max: a little beyond "tank fully empty" (height + offset + margin).
#define SENSOR_MIN_CM  (SENSOR_OFFSET_CM - 5)
#define SENSOR_MAX_CM  (TANK_HEIGHT_CM + SENSOR_OFFSET_CM + 15)

// --- ACK wait after button wakeup ---
#define ACK_TIMEOUT_MS     3000

// --- Battery ADC (Heltec V4: 1:2 resistor divider on GPIO 1, enable via GPIO 37) ---
#define BAT_ADC_PIN   1
#define BAT_CTRL_PIN  37

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
    for (int i = 0; i < 8; i++) {
        sum += analogReadMilliVolts(BAT_ADC_PIN);
        delay(5);
    }
    digitalWrite(BAT_CTRL_PIN, LOW);
    return (sum / 8.0f / 1000.0f) * 2.0f;
}

int batteryPercent(float v) {
    // LiPo: 3.0 V = 0 %, 4.2 V = 100 %
    float pct = (v - 3.0f) / (4.2f - 3.0f) * 100.0f;
    return (int)constrain(pct, 0.0f, 100.0f);
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
//   sensorError: true → shows "Sensor ERROR!" instead of fill level
//   status     : short string on the bottom line
//   battV      : battery voltage (V); <0 = omit battery row
void updateDisplay(int fillLevel, bool sensorError, const char* status, float battV = -1.0f) {
    if (!oledReady) return;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // Header
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Tank Sender");
    display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

    if (sensorError) {
        display.setTextSize(2);
        display.setCursor(10, 18);
        display.print("Sen ERR!");
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
        if ((micros() - t0) > 5000) {
            Serial.print(" -> no rising edge");
            return 0;
        }
    }
    // Measure the HIGH pulse (50 ms max ≈ 8.5 m)
    unsigned long t1 = micros();
    while (digitalRead(ECHO_PIN) == HIGH) {
        if ((micros() - t1) > 50000) break;
    }
    return (long)(micros() - t1);
}

float measureDistance() {
    float sum = 0;
    int valid = 0;
    for (int i = 0; i < NUM_MEASUREMENTS; i++) {
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(4);
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);

        Serial.printf("  [sensor] shot %d:", i + 1);
        long duration = echoMicros();
        if (duration > 0) {
            float d = duration * 0.034f / 2.0f;
            if (d >= SENSOR_MIN_CM && d <= SENSOR_MAX_CM) {
                Serial.printf(" %ld us -> %.1f cm OK", duration, d);
                sum += d;
                valid++;
            } else {
                Serial.printf(" %ld us -> %.1f cm OUT OF RANGE", duration, d);
            }
        }
        Serial.println();
        delay(60);
    }
    Serial.printf("  [sensor] valid=%d/%d  avg=%.1f cm\n",
                  valid, NUM_MEASUREMENTS, valid > 0 ? sum / valid : 0.0f);
    return (valid == 0) ? -1.0f : (sum / valid);
}

int calcFillLevel(float distanceCm) {
    float waterHeight = TANK_HEIGHT_CM - (distanceCm - SENSOR_OFFSET_CM);
    return constrain((int)((waterHeight / TANK_HEIGHT_CM) * 100.0f), 0, 100);
}

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

    Serial.printf("Sleeping for %d minutes...\n", SLEEP_MINUTES);
    Serial.flush();

    esp_sleep_enable_timer_wakeup(SLEEP_DURATION);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);

    esp_deep_sleep_start();
}

// ============================================

void setup() {
    Serial.begin(115200);

    esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();
    bool buttonWakeup = (wakeReason == ESP_SLEEP_WAKEUP_EXT0);
    Serial.println(buttonWakeup ? "\n=== Sender: button wakeup ===" : "\n=== Sender: timer wakeup ===");

    // Vext powers the sensor and OLED from the switched 3.3 V LDO.
    // goToSleep() shuts it off, so the sensor draws nothing during deep sleep.
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, LOW);

    // OLED only on button press
    if (buttonWakeup) {
        initOLED();
        updateDisplay(-1, false, "Booting...");
    }

    // ── Measure ─────────────────────────────────────────────────────────
    if (buttonWakeup) updateDisplay(-1, false, "Measuring...");
    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);
    pinMode(ECHO_PIN, INPUT_PULLDOWN);
    delay(200);

    float distance   = measureDistance();
    bool sensorError = (distance < 0);
    int  fillLevel   = sensorError ? 0 : calcFillLevel(distance);

    if (sensorError) {
        Serial.println("ERROR: No valid sensor reading.");
        if (buttonWakeup) updateDisplay(0, true, "Transmitting...");
    } else {
        Serial.printf("Distance: %.1f cm  |  Fill: %d%%\n", distance, fillLevel);
        if (buttonWakeup) updateDisplay(fillLevel, false, "Transmitting...");
    }

    // ── LoRa ─────────────────────────────────────────────────────────────
    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    int  loraState = radio.begin(868.0, 125.0, 7, 5);
    bool loraOk    = (loraState == RADIOLIB_ERR_NONE);
    if (!loraOk) Serial.printf("LoRa init failed: %d\n", loraState);

    bool        txOk        = false;
    bool        linked      = false;
    const char* finalStatus = "LoRa init failed!";

    if (loraOk) {
        String payload = sensorError ? "255" : String(fillLevel);
        txOk        = (radio.transmit(payload) == RADIOLIB_ERR_NONE);
        finalStatus = txOk ? "Transmitted OK" : "Transmit failed!";
        Serial.println(finalStatus);

        if (txOk && buttonWakeup && !sensorError) {
            updateDisplay(fillLevel, false, "Checking link...");
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
        updateDisplay(sensorError ? 0 : fillLevel, sensorError, finalStatus, battV);
        delay(5000);
        display.ssd1306_command(SSD1306_DISPLAYOFF);
    }

    goToSleep();
}

void loop() {
    // Never reached – ESP32 sleeps at the end of setup()
}
