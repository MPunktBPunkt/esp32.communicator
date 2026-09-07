# esp32.communicator

![Version](https://img.shields.io/badge/version-1.8.0-blue)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Donate](https://img.shields.io/badge/Donate-PayPal-00457C.svg?logo=paypal)](https://www.paypal.com/donate/?business=martin%40bchmnn.de&currency_code=EUR)

> **ESP32 Messenger** — Nachrichten zwischen ESP32-Boards per ESP-NOW, parallel zu WiFi und [iobroker.esp-hub](https://github.com/MPunktBPunkt/iobroker.esp-hub).

---

## Überblick

Mehrere ESP32-Geräte tauschen Textnachrichten und Anruf-Signale direkt aus — ohne Router, mit ~200 m Reichweite im Freien. Gleichzeitig bleiben sie per WiFi im Heimnetz und melden Statistiken an den ESP-Hub. Web-UI mit Chat, Peer-Liste und Live-Updates via SSE.

---

## Features

- **Messenger-UI** — Kontaktliste + Chat-Thread (Desktop-Split), Broadcast „Alle“
- **ESP-NOW Messaging** — Broadcast oder Unicast an bekannte MAC
- **Peer-Discovery** — automatisches Hello alle 5 s
- **WebRTC-Anrufe** — Klingeln/Annehmen über ESP-NOW; Audio/Video im Browser via Hub-Signaling
- **SSE** — Echtzeit-Updates ohne Polling
- **WiFi + ESP-NOW parallel** — Hub-Integration bleibt aktiv
- **OTA** — Hub-Push oder Browser

> **Hinweis Anrufe:** Browser brauchen für Kamera/Mikrofon einen Secure Context. Unter `http://` im LAN ggf. Origin freigeben oder HTTPS. Beide Web-UIs müssen während des Anrufs geöffnet sein.

---

## Voraussetzungen

| Typ | Details |
|-----|---------|
| **Board** | ESP32, Arduino ESP32 **≥ 3.0** |
| **WiFiManager** | tablatronix / tzapu |
| **ArduinoJson** | bblanchon v6 oder v7 |
| **ioBroker** | [iobroker.esp-hub](https://github.com/MPunktBPunkt/iobroker.esp-hub) **≥ 0.5.8** |

---

## Quickstart

1. Pro Gerät eindeutigen Namen setzen:

```cpp
#define DEVICE_NAME   "Communicator-1"
#define HUB_HOST      "192.168.178.113"
#define HUB_PORT      8093
```

2. Sketch auf alle ESPs flashen
3. Hotspot **`ESP-Comm-Setup`** → WLAN + Hub-IP
4. `http://<ESP-IP>/` oder `http://espcomm-<name>.local/`

> **Wichtig:** Alle Communicatoren müssen auf dem **selben WiFi-Kanal** sein (BSSID-Lock im Status-Tab).

---

## Vorkompilierte Firmware

Schema: `{name}.{version}.{family}.bin` (`esp32` = klassisch/D1 Mini, `esp32s3` = ESP32-S3)

| Datei | Board |
|-------|-------|
| `communicator.1.8.0.esp32.bin` | ESP32 / D1 Mini |
| `communicator.1.8.0.esp32s3.bin` | ESP32-S3 |

> Hinweis: Vorkompilierte Bins entsprechen Sketch-Stand **1.8.0**.

---

## Web-Oberfläche

| Tab | Inhalt |
|-----|--------|
| **Chat** | Nachrichten senden/empfangen, Emojis, Name ändern |
| **Geräte** | Bekannte Peers, Online-Status, Anrufen |
| **Status** | RSSI/Heap-Gauges, Kanal, Hub-Info |
| **OTA** | Firmware-Update |

---

## ioBroker-Integration (ESP-Hub)

```
esp-hub.0.devices.<MAC>.ios/
├── peers    ← bekannte ESPs
├── sent     ← gesendete Nachrichten
├── recv     ← empfangene Nachrichten
└── unread   ← ungelesen
```

---

## API (Auszug)

| Methode | Pfad | Beschreibung |
|---------|------|--------------|
| GET | `/api/messages` | Posteingang |
| POST | `/api/send` | `{to, text}` senden |
| POST | `/api/call` | Anruf-Signal |
| GET | `/api/peers` | Geräteliste |
| GET | `/api/sse` | Server-Sent Events |

---

## Lizenz

GNU General Public License v3.0 © MPunktBPunkt — siehe [LICENSE](LICENSE)

[![Donate](https://img.shields.io/badge/Donate-PayPal-00457C.svg?logo=paypal)](https://www.paypal.com/donate/?business=martin%40bchmnn.de&currency_code=EUR)
