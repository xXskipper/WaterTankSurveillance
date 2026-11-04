// ============================================
//  ESP32 #2 – RECEIVER (indoors)
//  Heltec WiFi LoRa 32 V4 (ESP32-S3 / SX1262) + WiFi + Email
// ============================================
//  Required libraries (Arduino Library Manager):
//    - RadioLib         (by jgromes)
//    - Adafruit SSD1306 (by Adafruit)
//    - Adafruit GFX Library (by Adafruit)
//    - ReadyMail        (by Mobizt)
// ============================================
//  Deep sleep strategy (adapted from DeepSleepWakeUpByLora example):
//    - SX1262 stays in continuous RX mode while ESP32 is in deep sleep.
//    - DIO1 (GPIO 14) goes HIGH when a packet arrives → wakes ESP32 via EXT1.
//    - PRG button (GPIO 0, active LOW)            → wakes ESP32 via EXT0.
//
//  Wakeup cases:
//    EXT1 (DIO1 HIGH)  – LoRa packet arrived:
//        OLED on immediately → read packet → send ACK →
//        send alarm email if critical → OLED off → sleep.
//    EXT0 (GPIO 0 LOW) – User pressed button:
//        OLED on → connect WiFi → show fill + WiFi + link stats → sleep.
//        Held for 3 s → additionally sends a test email.
//    First boot:
//        Full init → startup screen → sleep.
// ============================================

// ============================================
//  FEATURE FLAGS  (1 = on, 0 = off)
// ============================================
#define FEATURE_EMAIL        1   // Master switch: 0 = no WiFi, no ReadyMail, no mails at all
#define FEATURE_EMAIL_LOW    1   // Mail when the tank is (almost) empty
#define FEATURE_EMAIL_FULL   1   // Mail once when the tank is full
#define FEATURE_EMAIL_TEST   0   // Test mail on long button press
#define FEATURE_EMAIL_VALVE  0   // Mails when the valve opens / the tank is filled / the time limit hits

#define EMAIL_VALVE_ON (FEATURE_EMAIL && FEATURE_EMAIL_VALVE)
#define EMAIL_LOW_ON   (FEATURE_EMAIL && FEATURE_EMAIL_LOW)
#define EMAIL_FULL_ON  (FEATURE_EMAIL && FEATURE_EMAIL_FULL)
#define EMAIL_TEST_ON  (FEATURE_EMAIL && FEATURE_EMAIL_TEST)

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RadioLib.h>
#include <WiFi.h>
#if FEATURE_EMAIL
#include <WiFiClientSecure.h>
#define ENABLE_SMTP
#include <ReadyMail.h>
#endif
#include "driver/rtc_io.h"

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
#define BUTTON_PIN     0
#define LONG_PRESS_MS  3000   // Hold this long after wakeup → send a test email

// --- Vext: external power rail that feeds the OLED (active LOW) ---
#define VEXT_PIN    36

// --- WiFi, SMTP server & email credentials: see secrets.h ---
#include "secrets.h"

// Port 587 uses STARTTLS (plain connect, then upgrade); everything else direct SSL
#define SMTP_STARTTLS  (SMTP_PORT == 587)

// --- Alarm thresholds (ADJUST!) ---
#define ALARM_LEVEL    20   // Send email when fill level drops below 20 %
#define RESET_LEVEL    35   // Clear the alarm flag once level rises above 35 %
#define FULL_REARM_LEVEL 85 // Re-arm the "tank full" mail once level drops below 85 %

// --- Special LoRa payload codes (must match the sender) ---
#define CODE_UNSTABLE     253   // Readings scattered too much – ignored
#define CODE_TANK_FULL    254   // Water within the sensor's blind zone
#define CODE_SENSOR_ERROR 255   // No valid echo at all

// --- Battery ADC (Heltec V4: ~390k/100k divider (x4.9) on GPIO 1, enable via GPIO 37) ---
#define BAT_ADC_PIN   1
#define BAT_CTRL_PIN  37
#define BAT_DIVIDER   4.9f   // measured: x2 showed 1.5 V on a charged LiPo

// ============================================
//  State that must survive deep sleep → RTC memory
// ============================================
RTC_DATA_ATTR bool     alarmSent   = false;
RTC_DATA_ATTR bool     fullSent    = false;
RTC_DATA_ATTR int      lastFill    = -1;   // -1 = no packet received yet
RTC_DATA_ATTR int      lastRssi    = 0;
RTC_DATA_ATTR float    lastPressure = -2.0f;   // -2 = sender sends no pressure, -1 = sensor fault
RTC_DATA_ATTR int      lastValve   = -1;       // -1 = unknown, 0 = closed, 1 = open, 2 = locked
RTC_DATA_ATTR uint32_t packetCount = 0;

// ============================================

SPIClass loraSPI(FSPI);
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSPI);
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, OLED_RST);
#if FEATURE_EMAIL
WiFiClientSecure ssl_client;

// Called by ReadyMail after the STARTTLS command to upgrade the connection
void tlsHandshake(bool &success) { success = ssl_client.startTLS(); }

SMTPClient smtp(ssl_client, tlsHandshake, SMTP_STARTTLS);
#endif

bool oledReady = false;

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
    return (sum / 8.0f / 1000.0f) * BAT_DIVIDER;
}

int batteryPercent(float v) {
    // LiPo: 3.0 V = 0 %, 4.2 V = 100 %
    float pct = (v - 3.0f) / (4.2f - 3.0f) * 100.0f;
    return (int)constrain(pct, 0.0f, 100.0f);
}

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

// Renders the current state (from RTC memory) plus a live status line.
// Shows "No data yet" when lastFill == -1 (nothing received since boot).
//   status : short string on the bottom line
//   battV  : battery voltage (V); <0 = show pkts/rssi instead of battery row
void updateDisplay(const char* status, float battV = -1.0f) {
    if (!oledReady) return;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // Header
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Tank Receiver");
    if (lastValve >= 0) {
        char pv[12];
        if (lastPressure >= 0) snprintf(pv, sizeof(pv), "%.1fb", lastPressure);
        else                   snprintf(pv, sizeof(pv), "--b");
        display.setCursor(84, 0);
        display.print(pv);
        display.print(lastValve == 1 ? " ON" : (lastValve == 2 ? " ERR" : ""));
    }
    display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

    if (lastFill < 0) {
        display.setTextSize(1);
        display.setCursor(0, 22);
        display.print("No data received yet.");
        if (battV >= 0.0f) {
            char batLine[20];
            snprintf(batLine, sizeof(batLine), "Bat: %.1fV (%d%%)", battV, batteryPercent(battV));
            display.setCursor(0, 35);
            display.print(batLine);
        }
    } else {
        bool full = (lastFill == CODE_TANK_FULL);
        display.setTextSize(2);
        display.setCursor(0, 13);
        display.print("Fill: ");
        if (full) {
            display.print("FULL");
        } else {
            display.print(lastFill);
            display.print("%");
        }

        int barW = full ? 126 : (int)(lastFill * 126.0f / 100.0f);
        display.drawRect(0, 33, 128, 10, SSD1306_WHITE);
        if (barW > 0) display.fillRect(1, 34, barW, 8, SSD1306_WHITE);

        display.setTextSize(1);
        display.setCursor(0, 46);
        if (battV > 0.0f) {
            char batLine[22];
            snprintf(batLine, sizeof(batLine), "Bat:%.1fV(%d%%) Pkt:%u",
                     battV, batteryPercent(battV), packetCount);
            display.print(batLine);
        } else {
            display.print("Pkts:");
            display.print(packetCount);
            display.print(" RSSI:");
            display.print(lastRssi);
            display.print("dBm");
        }
    }

    // Status line
    if (status && status[0]) {
        display.setTextSize(1);
        display.setCursor(0, 56);
        display.print(status);
    }

    display.display();
}

// ============================================
//  WiFi & Email
// ============================================

void connectWiFi() {
#if !FEATURE_EMAIL
    return;   // WiFi is only needed for mails
#endif
    if (strlen(WIFI_PASSWORD) > 0) {
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    } else {
        WiFi.begin(WIFI_SSID);
    }
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        attempts++;
    }
}

#if FEATURE_EMAIL
void smtpCallback(SMTPStatus status) {
    Serial.println(status.text);
}

// Sends one email to all RECIPIENT_EMAILS. Returns true on success.
bool sendEmail(const String& subject, const String& body) {
    if (WiFi.status() != WL_CONNECTED) { Serial.println("No WiFi – email skipped."); return false; }

    ssl_client.setInsecure();
    if (SMTP_STARTTLS) ssl_client.setPlainStart();

    smtp.connect(SMTP_HOST, SMTP_PORT, smtpCallback, true /* SSL */);
    if (!smtp.isConnected()) { Serial.println("SMTP connect failed."); return false; }

    smtp.authenticate(SENDER_EMAIL, SENDER_PASSWORD, readymail_auth_password);
    if (!smtp.isAuthenticated()) { Serial.println("SMTP auth failed."); return false; }

    SMTPMessage msg;
    msg.headers.add(rfc822_from,    "Water Tank Monitor <" + String(SENDER_EMAIL) + ">");
    for (const char* rcpt : RECIPIENT_EMAILS) {
        msg.headers.add(rfc822_to, String(rcpt));
    }
    msg.headers.add(rfc822_subject, subject);
    msg.text.body(body);

    return smtp.send(msg);
}

void sendAlertEmail(int fillLevel) {
    String body  = "Hello,\r\n\r\n";
    body += "The water tank needs to be refilled!\r\n\r\n";
    body += "Current fill level : " + String(fillLevel) + " %\r\n";
    body += "Alarm threshold    : " + String(ALARM_LEVEL) + " %\r\n\r\n";
    body += "Please refill soon.\r\n\r\n";
    body += "-- Water Tank Monitor --";
    sendEmail("Water tank needs refilling!", body);
}

void sendFullEmail() {
    String body  = "Hello,\r\n\r\n";
    body += "The water tank is full again.\r\n\r\n";
    body += "-- Water Tank Monitor --";
    sendEmail("Water tank is full", body);
}

void sendValveEmail(int valve, int fillLevel, float pressure) {
    String subject, intro;
    if (valve == 1) {
        subject = "Water tank: refilling started";
        intro   = "The valve was opened, the tank is being refilled.";
    } else if (valve == 2) {
        subject = "Water tank: refill TIME LIMIT reached!";
        intro   = "The valve was closed after the safety time limit and is locked.\r\n"
                  "Please check the water line and the level sensor.";
    } else {
        subject = "Water tank: refilling finished";
        intro   = "The valve was closed.";
    }
    String body  = "Hello,\r\n\r\n" + intro + "\r\n\r\n";
    if (fillLevel == CODE_TANK_FULL) body += "Fill level : FULL\r\n";
    else                             body += "Fill level : " + String(fillLevel) + " %\r\n";
    if (pressure >= 0) body += "Pressure   : " + String(pressure, 1) + " bar\r\n";
    body += "\r\n-- Water Tank Monitor --";
    sendEmail(subject, body);
}

bool sendTestEmail() {
    String body  = "Hello,\r\n\r\n";
    body += "This is a test email from the Water Tank Monitor.\r\n";
    body += "If you can read this, email alerts are working.\r\n\r\n";
    if (lastFill == CODE_TANK_FULL) {
        body += "Last fill level    : FULL\r\n";
    } else if (lastFill >= 0) {
        body += "Last fill level    : " + String(lastFill) + " %\r\n";
    } else {
        body += "Last fill level    : no data received yet\r\n";
    }
    body += "Packets received   : " + String(packetCount) + "\r\n";
    body += "Last RSSI          : " + String(lastRssi) + " dBm\r\n\r\n";
    body += "-- Water Tank Monitor --";
    return sendEmail("Water tank monitor: test email", body);
}
#endif   // FEATURE_EMAIL

// ============================================
//  LoRa helpers
// ============================================

void initSPI() {
    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    pinMode(LORA_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH);
}

void goToSleep() {
    // Cut Vext power so the OLED draws nothing during sleep
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, HIGH);

    radio.startReceive();

#ifdef LORA_PA_EN
    rtc_gpio_hold_en((gpio_num_t)LORA_PA_EN);
#endif

    rtc_gpio_pullup_dis(GPIO_NUM_14);
    rtc_gpio_pulldown_en(GPIO_NUM_14);
    esp_sleep_enable_ext1_wakeup(1ULL << LORA_DIO1, ESP_EXT1_WAKEUP_ANY_HIGH);

    rtc_gpio_pullup_en(GPIO_NUM_0);
    rtc_gpio_pulldown_dis(GPIO_NUM_0);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);

    Serial.flush();
    esp_deep_sleep_start();
}

// ============================================
//  Wakeup handlers
// ============================================

void handleLoRaWakeup() {
    Serial.begin(115200);

#ifdef LORA_PA_EN
    rtc_gpio_hold_dis((gpio_num_t)LORA_PA_EN);
#endif

    initSPI();

    // Read packet BEFORE radio.begin() so the FIFO is not cleared by reset
    String received;
    int rxState = radio.readData(received);
    int rssi    = (int)radio.getRSSI();

    radio.begin(868.0, 125.0, 7, 5);
    radio.transmit("ACK");

    if (rxState == RADIOLIB_ERR_NONE) {
        received.trim();
        int fillLevel = received.toInt();
        Serial.printf("Fill: %d %%  RSSI: %d dBm\n", fillLevel, rssi);

        if (fillLevel == CODE_SENSOR_ERROR) {
            Serial.println("Sender reported a sensor error.");
        } else if (fillLevel == CODE_UNSTABLE) {
            Serial.println("Sender reported unstable readings – ignored.");
        } else {
            // CODE_TANK_FULL is stored as-is: it is above RESET_LEVEL (clears
            // the alarm) and never below ALARM_LEVEL.
            if (fillLevel == CODE_TANK_FULL) Serial.println("Tank full (water in sensor blind zone).");
            packetCount++;
            lastFill = fillLevel;
            lastRssi = rssi;

            // Optional extension from the sender: "level;pressure;valve"
            int newValve = -1;
            int sep1 = received.indexOf(';');
            int sep2 = received.indexOf(';', sep1 + 1);
            if (sep1 > 0 && sep2 > sep1) {
                lastPressure = received.substring(sep1 + 1, sep2).toFloat();
                newValve     = received.substring(sep2 + 1).toInt();
                Serial.printf("Pressure: %.1f bar  Valve: %d\n", lastPressure, newValve);
            }

#if EMAIL_VALVE_ON
            // Mail on valve state changes only; the very first report just sets the state
            if (newValve >= 0 && lastValve >= 0 && newValve != lastValve) {
                Serial.println("Valve state changed – sending email...");
                connectWiFi();
                sendValveEmail(newValve, fillLevel, lastPressure);
                WiFi.disconnect(true);
            }
#endif
            if (newValve >= 0) lastValve = newValve;

            if (fillLevel >= RESET_LEVEL && alarmSent) {
                alarmSent = false;
                Serial.println("Alarm cleared – tank refilled.");
            }

            if (fillLevel < FULL_REARM_LEVEL && fullSent) {
                fullSent = false;
            }

#if EMAIL_FULL_ON
            if (fillLevel == CODE_TANK_FULL && !fullSent) {
                Serial.println("Tank full! Sending email...");
                connectWiFi();
                sendFullEmail();
                WiFi.disconnect(true);
                fullSent = true;
            }
#endif

#if EMAIL_LOW_ON
            if (fillLevel <= ALARM_LEVEL && !alarmSent) {
                Serial.println("Fill level critical! Sending email...");
                connectWiFi();
                sendAlertEmail(fillLevel);
                WiFi.disconnect(true);
                alarmSent = true;
            }
#endif
        }
    } else {
        Serial.printf("LoRa read error: %d\n", rxState);
    }

    goToSleep();
}

void handleButtonWakeup() {
    Serial.begin(115200);
    Serial.println("Button wakeup – showing status.");

#ifdef LORA_PA_EN
    rtc_gpio_hold_dis((gpio_num_t)LORA_PA_EN);
#endif

    initOLED();

    // Long press (button still held after LONG_PRESS_MS) → send a test email.
    // GPIO 0 is still an RTC pin after the EXT0 wakeup – hand it back to GPIO.
    rtc_gpio_deinit(GPIO_NUM_0);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    bool longPress = false;
#if EMAIL_TEST_ON
    updateDisplay("Hold 3s: test mail");
    unsigned long pressStart = millis();
    while (digitalRead(BUTTON_PIN) == LOW && (millis() - pressStart) < LONG_PRESS_MS) {
        delay(20);
    }
    longPress = (digitalRead(BUTTON_PIN) == LOW);
#endif
    Serial.println(longPress ? "Long press – test email." : "Short press.");

    updateDisplay("WiFi: connecting...");

    float battV = readBatteryVoltage();

    initSPI();
    radio.begin(868.0, 125.0, 7, 5);

    connectWiFi();

    char wifiLine[22];
    if (!FEATURE_EMAIL) {
        snprintf(wifiLine, sizeof(wifiLine), "Email: disabled");
    } else if (WiFi.status() == WL_CONNECTED) {
        snprintf(wifiLine, sizeof(wifiLine), "WiFi: %s",
                 WiFi.localIP().toString().c_str());
    } else {
        snprintf(wifiLine, sizeof(wifiLine), "WiFi: no connection");
    }
    updateDisplay(wifiLine, battV);

#if EMAIL_TEST_ON
    if (longPress) {
        if (WiFi.status() == WL_CONNECTED) {
            updateDisplay("Sending test mail...", battV);
            bool ok = sendTestEmail();
            Serial.println(ok ? "Test email sent." : "Test email failed.");
            updateDisplay(ok ? "Test mail sent" : "Mail failed!", battV);
        } else {
            updateDisplay("Mail: no WiFi!", battV);
        }
    }
#endif

    delay(6000);
    display.ssd1306_command(SSD1306_DISPLAYOFF);

    WiFi.disconnect(true);
    goToSleep();
}

void handleFirstBoot() {
    Serial.begin(115200);
    Serial.println("\n=== Water Tank Receiver – First Boot ===");

    initOLED();
    updateDisplay("Booting...");

    initSPI();
    int loraState = radio.begin(868.0, 125.0, 7, 5);
    bool loraOk   = (loraState == RADIOLIB_ERR_NONE);
    if (!loraOk) Serial.printf("LoRa init failed: %d\n", loraState);

    updateDisplay("WiFi: connecting...");
    connectWiFi();
    
    char statusLine[22];
    if (!FEATURE_EMAIL) {
        snprintf(statusLine, sizeof(statusLine), "NoMail  LoRa %s",
                 loraOk ? "OK" : "FAIL");
    } else if (WiFi.status() == WL_CONNECTED) {
        snprintf(statusLine, sizeof(statusLine), "WiFi OK  LoRa %s",
                 loraOk ? "OK" : "FAIL");
    } else {
        snprintf(statusLine, sizeof(statusLine), "WiFi FAIL  LoRa %s",
                 loraOk ? "OK" : "FAIL");
    }
    updateDisplay(statusLine);

    delay(4000);
    display.ssd1306_command(SSD1306_DISPLAYOFF);

    WiFi.disconnect(true);
    Serial.println("Going to sleep – waiting for LoRa packet or button.");
    goToSleep();
}

// ============================================

void setup() {
    auto cause = esp_sleep_get_wakeup_cause();

    switch (cause) {
        case ESP_SLEEP_WAKEUP_EXT1: handleLoRaWakeup();  break;
        case ESP_SLEEP_WAKEUP_EXT0: handleButtonWakeup(); break;
        default:                    handleFirstBoot();    break;
    }
}

void loop() {
    // Never reached – the device sleeps at the end of every setup() path
}
