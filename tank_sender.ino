// ============================================
//  ESP32 #1 – SENDER (am Tank) mit Deep Sleep
//  TTGO LoRa32 V2 + JSN-SR04T + Solar
// ============================================
//  Ablauf:
//  1. Aufwachen
//  2. Ultraschall messen
//  3. Per LoRa senden
//  4. Zurück in Deep Sleep
// ============================================

#include <SPI.h>
#include <LoRa.h>

// --- TTGO LoRa32 V2 Pin-Definitionen ---
#define LORA_SCK    5
#define LORA_MISO   19
#define LORA_MOSI   27
#define LORA_SS     18
#define LORA_RST    14
#define LORA_DIO0   26

// --- JSN-SR04T Pins ---
#define TRIG_PIN    13
#define ECHO_PIN    12

// --- Tank-Parameter (ANPASSEN!) ---
#define TANK_HOEHE_CM     150   // Innenhöhe des Tanks in cm
#define SENSOR_OFFSET_CM   10   // Abstand Sensor bis Tankoberkante in cm

// --- Deep Sleep Intervall ---
#define SCHLAF_MINUTEN    10    // Alle 10 Minuten aufwachen und messen
#define uS_PRO_MINUTE     60000000ULL
#define SCHLAF_DAUER      (SCHLAF_MINUTEN * uS_PRO_MINUTE)

// --- Anzahl Messungen für Mittelwert ---
#define ANZAHL_MESSUNGEN  5

// ============================================

float messeAbstand() {
  float summe = 0;
  int gueltigeMessungen = 0;

  for (int i = 0; i < ANZAHL_MESSUNGEN; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long dauer = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout 30 ms

    if (dauer > 0) {
      summe += dauer * 0.034 / 2.0;
      gueltigeMessungen++;
    }
    delay(50);
  }

  if (gueltigeMessungen == 0) return -1; // Fehler
  return summe / gueltigeMessungen;
}

int berechneFuellstand(float abstandCm) {
  float wasserHoehe = TANK_HOEHE_CM - (abstandCm - SENSOR_OFFSET_CM);
  int prozent = (int)((wasserHoehe / TANK_HOEHE_CM) * 100.0);
  return constrain(prozent, 0, 100);
}

bool sendeLoRa(int fuellstand) {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(868E6)) {
    Serial.println("FEHLER: LoRa Start fehlgeschlagen!");
    return false;
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  delay(100);

  LoRa.beginPacket();
  LoRa.print(fuellstand);
  LoRa.endPacket();
  delay(100);

  // LoRa ausschalten vor dem Schlafen → spart Strom
  LoRa.sleep();
  SPI.end();

  return true;
}

void geheSchlafen() {
  pinMode(TRIG_PIN, INPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.print("Gehe schlafen für ");
  Serial.print(SCHLAF_MINUTEN);
  Serial.println(" Minuten...");
  Serial.flush();

  esp_sleep_enable_timer_wakeup(SCHLAF_DAUER);
  esp_deep_sleep_start();
  // Ab hier läuft kein Code mehr – nächster Start bei setup()
}

// ============================================

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Tank-Sender aufgewacht ===");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  delay(200);

  // --- Messen ---
  float abstand = messeAbstand();

  if (abstand < 0) {
    Serial.println("FEHLER: Keine gültige Messung! Sende Fehlercode 255.");
    sendeLoRa(255); // 255 = Fehlerwert
  } else {
    int fuellstand = berechneFuellstand(abstand);

    Serial.print("Abstand zur Oberfläche: ");
    Serial.print(abstand, 1);
    Serial.println(" cm");
    Serial.print("Füllstand: ");
    Serial.print(fuellstand);
    Serial.println("%");

    if (sendeLoRa(fuellstand)) {
      Serial.println("Daten erfolgreich gesendet.");
    } else {
      Serial.println("Senden fehlgeschlagen.");
    }
  }

  // --- Schlafen ---
  geheSchlafen();
}

void loop() {
  // Wird nie ausgeführt – ESP32 schläft nach setup()
}
