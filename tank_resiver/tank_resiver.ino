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
//    First boot:
//        Full init → startup screen → sleep.
// ============================================

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#define ENABLE_SMTP
#include <ReadyMail.h>
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
#define BUTTON_PIN  0

// --- Vext: external power rail that feeds the OLED (active LOW) ---
#define VEXT_PIN    36

// --- WiFi (ADJUST!) ---
#define WIFI_SSID       "YourNetworkName"
#define WIFI_PASSWORD   "YourWiFiPassword"   // Leave empty ("") for open networks

// --- Email Settings (ADJUST!) ---
// For Gmail: create an App Password at
// Google Account → Security → App Passwords
#define SMTP_HOST         "smtp.gmail.com"
#define SMTP_PORT         465
#define SENDER_EMAIL      "your.address@gmail.com"
#define SENDER_PASSWORD   "xxxx xxxx xxxx xxxx"   // Gmail App Password
#define RECIPIENT_EMAIL   "recipient@email.com"

// --- Alarm thresholds (ADJUST!) ---
#define ALARM_LEVEL    20   // Send email when fill level drops below 20 %
#define RESET_LEVEL    35   // Clear the alarm flag once level rises above 35 %

// --- Battery ADC (Heltec V4: 1:2 resistor divider on GPIO 1, enable via GPIO 37) ---
#define BAT_ADC_PIN   1
#define BAT_CTRL_PIN  37

// ============================================
//  State that must survive deep sleep → RTC memory
// ============================================
RTC_DATA_ATTR bool     alarmSent   = false;
RTC_DATA_ATTR int      lastFill    = -1;   // -1 = no packet received yet
RTC_DATA_ATTR int      lastRssi    = 0;
RTC_DATA_ATTR uint32_t packetCount = 0;

// ============================================

SPIClass loraSPI(FSPI);
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSPI);
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, OLED_RST);
WiFiClientSecure ssl_client;
SMTPClient smtp(ssl_client);

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
    return (sum / 8.0f / 1000.0f) * 2.0f;
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
        display.setTextSize(2);
        display.setCursor(0, 13);
        display.print("Fill: ");
        display.print(lastFill);
        display.print("%");

        int barW = (int)(lastFill * 126.0f / 100.0f);
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

void smtpCallback(SMTPStatus status) {
    Serial.println(status.text);
}

void sendAlertEmail(int fillLevel) {
    if (WiFi.status() != WL_CONNECTED) return;

    ssl_client.setInsecure();

    smtp.connect(SMTP_HOST, SMTP_PORT, smtpCallback, true /* SSL */);
    if (!smtp.isConnected()) { Serial.println("SMTP connect failed."); return; }

    smtp.authenticate(SENDER_EMAIL, SENDER_PASSWORD, readymail_auth_password);
    if (!smtp.isAuthenticated()) { Serial.println("SMTP auth failed."); return; }

    SMTPMessage msg;
    msg.headers.add(rfc822_from,    "Water Tank Monitor <" + String(SENDER_EMAIL) + ">");
    msg.headers.add(rfc822_to,      "Owner <" + String(RECIPIENT_EMAIL) + ">");
    msg.headers.add(rfc822_subject, "Water tank needs refilling!");

    String body  = "Hello,\r\n\r\n";
    body += "The water tank needs to be refilled!\r\n\r\n";
    body += "Current fill level : " + String(fillLevel) + " %\r\n";
    body += "Alarm threshold    : " + String(ALARM_LEVEL) + " %\r\n\r\n";
    body += "Please refill soon.\r\n\r\n";
    body += "-- Water Tank Monitor --";
    msg.text.body(body);

    smtp.send(msg);
}

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

        if (fillLevel != 255) {
            packetCount++;
            lastFill = fillLevel;
            lastRssi = rssi;

            if (fillLevel >= RESET_LEVEL && alarmSent) {
                alarmSent = false;
                Serial.println("Alarm cleared – tank refilled.");
            }

            if (fillLevel <= ALARM_LEVEL && !alarmSent) {
                Serial.println("Fill level critical! Sending email...");
                connectWiFi();
                sendAlertEmail(fillLevel);
                WiFi.disconnect(true);
                alarmSent = true;
            }
        } else {
            Serial.println("Sender reported a sensor error.");
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
    updateDisplay("WiFi: connecting...");

    float battV = readBatteryVoltage();

    initSPI();
    radio.begin(868.0, 125.0, 7, 5);

    connectWiFi();

    char wifiLine[22];
    if (WiFi.status() == WL_CONNECTED) {
        snprintf(wifiLine, sizeof(wifiLine), "WiFi: %s",
                 WiFi.localIP().toString().c_str());
    } else {
        snprintf(wifiLine, sizeof(wifiLine), "WiFi: no connection");
    }
    updateDisplay(wifiLine, battV);

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
    if (WiFi.status() == WL_CONNECTED) {
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
