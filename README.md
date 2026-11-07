# Water Tank Surveillance

A battery/solar powered water tank monitor built from two **Heltec WiFi LoRa 32 V4** boards.

- The **sender** sits at the tank, measures the fill level with an ultrasonic sensor and transmits it over LoRa (868 MHz).
- The **receiver** sits indoors, shows the level on its OLED and sends **e-mail alerts** when the tank is almost empty or full.
- *Optional:* the sender can monitor the water-line pressure and open a **solenoid valve** to refill the tank automatically.

Both boards spend almost all of their time in deep sleep.

```
 ┌──────────── at the tank ────────────┐          ┌────────── indoors ──────────┐
 │ JSN-SR04T ─► Sender (ESP32-S3)      │  LoRa    │ Receiver (ESP32-S3) ─► WiFi │─► e-mail
 │ [pressure sensor] [solenoid valve]  │ ───────► │ OLED + PRG button           │
 └─────────────────────────────────────┘ ◄─ ACK ──┴─────────────────────────────┘
```

## Features

- Fill level in % via a JSN-SR04T waterproof ultrasonic sensor (median filter, outlier rejection, blind-zone detection)
- Measurement every 30 minutes, deep sleep in between, designed for a LiPo plus a small solar panel
- Receiver sleeps while the SX1262 runs in RX duty-cycle mode (the sender transmits a long preamble) and is woken by the incoming packet (no polling) – ~30× less current than continuous RX
- Battery percentage uses a LiPo discharge curve (non-linear), not a straight line
- E-mail when the level drops below a threshold, and once when the tank is full (with hysteresis, no repeat spam)
- OLED status on button press (level, battery, link quality, WiFi); long press sends a test e-mail
- Optional pressure monitoring and automatic refill valve with several safety limits
- Compile-time **feature flags** to switch functions on and off

## Hardware

See [hardware.txt](hardware.txt) for the full bill of materials and pin mapping.

| Part | Qty |
|---|---|
| Heltec WiFi LoRa 32 V4 (ESP32-S3, SX1262, 0.96" OLED) | 2 |
| JSN-SR04T waterproof ultrasonic distance sensor | 1 |
| 3.7 V 3000 mAh LiPo (JST 1.25) | 2 |
| 5 V mini solar panel | 2 |
| *Optional:* analog pressure transmitter, solenoid valve, MOSFET/relay driver | 1 each |

## Software setup

1. Install the ESP32 board package (tested with `esp32:esp32` 3.3.x) and the libraries below via the Arduino IDE or `arduino-cli`:
   - [RadioLib](https://github.com/jgromes/RadioLib)
   - Adafruit SSD1306 and Adafruit GFX Library
   - [ReadyMail](https://github.com/mobizt/ReadyMail) (receiver only)
2. Create the receiver credentials:
   ```
   cp tank_receiver/secrets.h.example tank_receiver/secrets.h
   ```
   and fill in WiFi and SMTP data. `secrets.h` is git-ignored.
3. Adjust the tank geometry at the top of [tank_sender/tank_sender.ino](tank_sender/tank_sender.ino):
   `TANK_HEIGHT_CM` (usable water height) and `SENSOR_OFFSET_CM` (sensor face to the 100 % level).
4. Adjust the alarm levels in [tank_receiver/tank_receiver.ino](tank_receiver/tank_receiver.ino): `ALARM_LEVEL`, `RESET_LEVEL`.
5. Build and flash, for example:
   ```
   arduino-cli compile --upload -p /dev/ttyACM0 tank_sender
   arduino-cli compile --upload -p /dev/ttyACM1 tank_receiver
   ```
   The board is preselected in each `sketch.yaml` (`heltec_wifi_lora_32_V4`).

> To flash a board that is in deep sleep, hold the PRG button to wake it up first.

## Feature flags

Set at the top of each sketch (`1` = on, `0` = off). Disabled code is not compiled.

| Sketch | Flag | Default | Purpose |
|---|---|---|---|
| Sender | `FEATURE_PRESSURE` | 0 | Measure line pressure and send it with the level |
| Sender | `FEATURE_VALVE` | 0 | Automatic refill valve (needs `FEATURE_PRESSURE`) |
| Receiver | `FEATURE_EMAIL` | 1 | Master switch: off = no WiFi, no mails |
| Receiver | `FEATURE_EMAIL_LOW` | 1 | Mail when the tank is almost empty |
| Receiver | `FEATURE_EMAIL_FULL` | 1 | Mail once when the tank is full |
| Receiver | `FEATURE_EMAIL_TEST` | 1 | Test mail on long button press |
| Receiver | `FEATURE_EMAIL_VALVE` | 1 | Mails on valve events (only active with a valve) |

## Automatic refill (optional)

With `FEATURE_VALVE` enabled the sender opens the valve when the fill level is ≤ 30 % **and** the line pressure is ≥ 1.0 bar. While it is open the sender checks every 2 minutes. The valve is closed again when:

- the tank is full (water in the sensor's blind zone) or the level reaches 95 %,
- the pressure drops below 0.3 bar,
- the level reading is invalid,
- or after 240 minutes (safety limit). The valve is then locked until the tank is refilled above 30 % or the button is pressed.

A normally-closed valve is strongly recommended so that a crash or empty battery leaves the water shut off. All thresholds are `#define`s at the top of the sender sketch. **Pin numbers and sensor values are placeholders** – verify them against your hardware before enabling the flags.

## LoRa payload

Plain text, sent by the sender every wake-up:

```
45            fill level in % (FEATURE_PRESSURE off)
45;3.2;1      level;pressure in bar;valve  (valve: 0 closed, 1 open, 2 locked)
```

Both boards must use the same `LORA_PREAMBLE_SYMBOLS` (sender transmits a long preamble, receiver duty-cycles its radio) – flash both after changing it.

Special level codes: `253` unstable reading (ignored), `254` tank full, `255` sensor error. The receiver answers every packet with `ACK`.

## Repository layout

```
tank_sender/     sender sketch (at the tank)
tank_receiver/   receiver sketch (indoors) + secrets.h.example
hardware.txt     bill of materials and pin mapping
```

## Safety note

This is a hobby project. If a valve can flood something, add your own independent safeguards (e.g. a mechanical float valve or a water stop) and do not rely on this firmware alone.

## License

[MIT](LICENSE)
