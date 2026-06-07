// ============================================
//  ESP32 #2 – EMPFÄNGER (im Haus)
//  TTGO LoRa32 + WLAN + E-Mail Benachrichtigung
// ============================================
//  Benötigte Bibliotheken (Arduino Library Manager):
//    - LoRa       (by Sandeep Mistry)
//    - ESP Mail Client  (by Mobizt)
// ============================================

#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <ESP_Mail_Client.h>

// --- TTGO LoRa32 Pin-Definitionen ---
#define LORA_SCK    5
#define LORA_MISO   19
#define LORA_MOSI   27
#define LORA_SS     18
#define LORA_RST    14
#define LORA_DIO0   26

// --- WLAN (ANPASSEN!) ---
#define WIFI_SSID       "DeinNetzwerkname"
#define WIFI_PASSWORT   "DeinWLAN-Passwort"

// --- E-Mail Einstellungen (ANPASSEN!) ---
// Hinweis: Bei Gmail ein "App-Passwort" erstellen unter:
// Google-Konto → Sicherheit → App-Passwörter
#define SMTP_HOST         "smtp.gmail.com"
#define SMTP_PORT         465
#define ABSENDER_EMAIL    "deine.adresse@gmail.com"
#define ABSENDER_PASSWORT "xxxx xxxx xxxx xxxx"   // Gmail App-Passwort
#define EMPFAENGER_EMAIL  "empfaenger@email.com"

// --- Schwellwert für Benachrichtigung (ANPASSEN!) ---
#define FUELLSTAND_ALARM_PROZENT  20   // E-Mail bei unter 20%
#define FUELLSTAND_RESET_PROZENT  35   // Alarm zurücksetzen wenn wieder über 35%

// ============================================

SMTPSession smtp;
bool emailGesendet = false;

// ============================================

void verbindeWLAN() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORT);
  Serial.print("WLAN verbinden");
  int versuche = 0;
  while (WiFi.status() != WL_CONNECTED && versuche < 20) {
    delay(500);
    Serial.print(".");
    versuche++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWLAN verbunden! IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWLAN-Verbindung fehlgeschlagen!");
  }
}

void sendeEmail(int fuellstand) {
  // WLAN prüfen und ggf. neu verbinden
  if (WiFi.status() != WL_CONNECTED) {
    verbindeWLAN();
  }

  Session_Config config;
  config.server.host_name = SMTP_HOST;
  config.server.port = SMTP_PORT;
  config.login.email = ABSENDER_EMAIL;
  config.login.password = ABSENDER_PASSWORT;
  config.login.user_domain = "";
  config.secure.startTLS = false;

  SMTP_Message message;
  message.sender.name = "Wassertank Monitor";
  message.sender.email = ABSENDER_EMAIL;
  message.subject = "Wassertank muss nachgefüllt werden!";
  message.addRecipient("Eigentümer", EMPFAENGER_EMAIL);

  // E-Mail Text
  String textInhalt = "Hallo,\n\n";
  textInhalt += "der Wassertank muss nachgefüllt werden!\n\n";
  textInhalt += "Aktueller Füllstand: " + String(fuellstand) + "%\n";
  textInhalt += "Alarm-Schwelle: " + String(FUELLSTAND_ALARM_PROZENT) + "%\n\n";
  textInhalt += "Bitte bald nachfüllen.\n\n";
  textInhalt += "-- Wassertank Monitor --";
  message.text.content = textInhalt.c_str();
  message.text.charSet = "utf-8";

  if (!smtp.connect(&config)) {
    Serial.println("FEHLER: SMTP Verbindung fehlgeschlagen – " + smtp.errorReason());
    return;
  }

  if (!MailClient.sendMail(&smtp, &message)) {
    Serial.println("FEHLER beim Senden: " + smtp.errorReason());
  } else {
    Serial.println("E-Mail erfolgreich gesendet!");
  }
  smtp.closeSession();
}

// ============================================

void setup() {
  Serial.begin(115200);

  verbindeWLAN();

  // LoRa initialisieren
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(868E6)) {
    Serial.println("FEHLER: LoRa konnte nicht gestartet werden!");
    while (true);
  }

  // Gleiche Einstellungen wie Sender!
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);

  Serial.println("Empfänger bereit – warte auf Daten...");
}

// ============================================

void loop() {
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    String empfangen = "";
    while (LoRa.available()) {
      empfangen += (char)LoRa.read();
    }
    empfangen.trim();

    int fuellstand = empfangen.toInt();
    int signalStaerke = LoRa.packetRssi();

    Serial.println("----------------------------");
    Serial.print("Füllstand empfangen: ");
    Serial.print(fuellstand);
    Serial.println("%");
    Serial.print("Signalstärke (RSSI): ");
    Serial.print(signalStaerke);
    Serial.println(" dBm");

    // Alarm zurücksetzen wenn Tank wieder aufgefüllt wurde
    if (fuellstand >= FUELLSTAND_RESET_PROZENT && emailGesendet) {
      emailGesendet = false;
      Serial.println("Alarm zurückgesetzt (Tank wurde nachgefüllt).");
    }

    // E-Mail senden wenn Füllstand unter Schwelle (nur einmal pro Ereignis)
    if (fuellstand <= FUELLSTAND_ALARM_PROZENT && !emailGesendet) {
      Serial.println("Füllstand kritisch! Sende E-Mail...");
      sendeEmail(fuellstand);
      emailGesendet = true;
    }
  }
}
