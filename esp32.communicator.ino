// ╔══════════════════════════════════════════════════════════════╗
// ║  esp32.communicator.ino                                     ║
// ║  Version: 1.5.0                                             ║
// ║  Basis: esp-hub-base v1.5.0                                 ║
// ╠══════════════════════════════════════════════════════════════╣
// ║  Kommunikation:                                             ║
// ║    ESP-NOW  — direkte Nachrichten ohne Router               ║
// ║               Broadcast (alle) oder Unicast (gezielt)       ║
// ║               Reichweite: ~200m offen, ~50m Gebäude         ║
// ║    WiFi     — Web-UI + ioBroker Hub-Heartbeat               ║
// ║    mDNS     — http://<name>.local/                          ║
// ╠══════════════════════════════════════════════════════════════╣
// ║  Bibliotheken (Arduino Library Manager):                    ║
// ║    - WiFiManager  von tablatronix / tzapu                   ║
// ║    - ArduinoJson  von bblanchon (v6 oder v7)                ║
// ║  Built-in (kein separater Download nötig):                  ║
// ║    - esp_now, WiFi, HTTPClient, Update                      ║
// ║    - WebServer, Preferences, ESPmDNS                        ║
// ╠══════════════════════════════════════════════════════════════╣
// ║  Hinweis: Arduino ESP32 Board-Paket >= 3.0 erforderlich     ║
// ║  (esp_now_recv_info_t Callback-Signatur)                    ║
// ╠══════════════════════════════════════════════════════════════╣
// ║  ESP-NOW + WiFi gleichzeitig:                               ║
// ║    ESP-NOW nutzt automatisch den WiFi-Kanal.                ║
// ║    ESPs ohne WiFi: esp_wifi_set_channel(CH, WIFI_SECOND_    ║
// ║    CHAN_NONE) auf denselben Kanal setzen wie der Router.    ║
// ╠══════════════════════════════════════════════════════════════╣
// ║  Quickstart:                                                ║
// ║    1. DEVICE_NAME + HUB_HOST anpassen                       ║
// ║    2. Flashen → Captive Portal → WLAN konfigurieren         ║
// ║    3. Web-UI: http://<ESP-IP>/ oder http://<mdns>.local/    ║
// ║    4. Weitere ESPs flashen → erscheinen automatisch         ║
// ║       in der Geräteliste sobald sie Hello senden            ║
// ╠══════════════════════════════════════════════════════════════╣
// ║  WLAN zurücksetzen:                                         ║
// ║    BOOT-Taste beim Einschalten 3 Sek. gedrückt halten       ║
// ╚══════════════════════════════════════════════════════════════╝

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>

// ================================================================
//  KONFIGURATION  <-- Diese Werte anpassen
// ================================================================

// Gerätename — erscheint im Dashboard und als Absender
#define DEVICE_NAME            "Communicator-1"

// Firmware-Version
#define FW_VERSION             "1.5.0 communicator"

// ioBroker Host-IP (für Hub-Heartbeat)
#define HUB_HOST               "192.168.178.113"
#define HUB_PORT               8093

// WLAN-Hotspot-Name beim Erststart
#define WIFI_AP_NAME           "ESP-Comm-Setup"
#define WIFI_PORTAL_TIMEOUT_S  180

// Heartbeat-Intervall (wird vom Hub dynamisch überschrieben)
#define DEFAULT_INTERVAL_S     30
// Kein Watchdog-Reset — ESP-NOW + Nachrichten/Log bleiben ohne Hub erhalten

// BOOT-Taste (GPIO0 auf fast allen ESP32-Boards)
#define RESET_BUTTON_PIN       0
#define RESET_HOLD_SEC         3

// ================================================================
//  NACHRICHTEN-LED  <-- Bei neuer Nachricht blinkt diese LED
// ================================================================

// GPIO2 = Onboard-LED auf den meisten ESP32-Boards
// LOW = AN (invertierte Logik)
#define MSG_LED_PIN      2
#define MSG_LED_INVERTED true

// ================================================================
//  FORWARD DECLARATIONS
//  (Arduino IDE löst Reihenfolge nicht automatisch auf)
// ================================================================
void     sendHeartbeat();
void     performOta(const String& url);
void     ssePush(const char* event, const String& data);
String   handleApiStatusJson();
String   inboxToJson();
String   peersToJson();
String   eventLogToJson();
void     espNowCall(const uint8_t* toMac, uint8_t callType);

// ================================================================
//  INTERNE KONSTANTEN
// ================================================================

#define MAX_INBOX      30      // Max. Nachrichten im Posteingang
#define MAX_PEERS      20      // Max. bekannte Geräte
#define MSG_TYPE_TEXT   0x01   // Textnachricht
#define MSG_TYPE_HELLO  0x02   // Peer-Discovery-Beacon
#define MSG_TYPE_SCAN   0x03   // RSSI-Scan Ergebnisse
#define MSG_TYPE_CALL   0x04   // Anruf-Anfrage
#define MSG_TYPE_CALL_A 0x05   // Anruf angenommen
#define MSG_TYPE_CALL_R 0x06   // Anruf abgelehnt
#define MSG_TYPE_CALL_E 0x07   // Anruf beendet
#define HELLO_INTERVAL 5000UL  // Alle 5s Hello senden
#define SCAN_INTERVAL  60000UL // Alle 60s Scan-Ergebnisse tauschen
#define HUB_CHECK_INTERVAL_MS 300000UL // Alle 5min Hub-Check wenn kein Zugang
#define LED_BLINK_MS   180     // LED-Blinkintervall ms
#define SLOW_BLINK_MS  4000UL  // Langsames Blinken wenn ungelesen

// ================================================================
//  ESP-NOW Paket-Struktur (max 250 Bytes)
// ================================================================
typedef struct __attribute__((packed)) {
    uint8_t  type;          // MSG_TYPE_TEXT / MSG_TYPE_HELLO
    char     fromName[32];  // Absender-Name
    uint8_t  toMac[6];      // Ziel-MAC (0xFF*6 = broadcast)
    char     text[200];     // Nachrichtentext
    uint32_t msgId;         // Fortlaufende ID
} EspNowPacket;             // = 243 Bytes ✓ (< 250)

// Scan-Ergebnis-Eintrag (8 Bytes)
struct ApRssiEntry {
    uint8_t  bssid[6];  // AP MAC
    int8_t   rssi;      // RSSI in dBm
    uint8_t  channel;   // WiFi-Kanal
};

// Scan-Broadcast-Paket (max 5 APs) — 3 + 5*8 = 43 Bytes
typedef struct __attribute__((packed)) {
    uint8_t      type;          // MSG_TYPE_SCAN
    char         fromName[32];  // Absender
    uint8_t      apCount;       // Anzahl APs (max 5)
    ApRssiEntry  aps[5];        // Scan-Ergebnisse
} ScanPacket;                   // = 83 Bytes ✓


// ================================================================
//  Datenstrukturen
// ================================================================

struct InboxEntry {
    char     fromName[32];
    char     fromMac[18];
    char     text[200];
    uint32_t timestamp;
    uint32_t msgId;
    bool     isRead;
    bool     isBroadcast;
    bool     isSent;        // true = eigene gesendete Nachricht
};

struct PeerEntry {
    char     name[32];
    uint8_t  mac[6];
    uint32_t lastSeen;   // seconds since boot
};

// ================================================================
//  Globale Variablen
// ================================================================

Preferences  prefs;
WiFiManager  wifiManager;
WebServer    webServer(80);

String       deviceName = DEVICE_NAME;
String       hubHost    = HUB_HOST;
int          hubPort    = HUB_PORT;

// Inbox (circular buffer)
InboxEntry   inbox[MAX_INBOX];
int          inboxCount = 0;
int          inboxNext  = 0;
uint32_t     unreadCount = 0;

// Bekannte Peers
PeerEntry    knownPeers[MAX_PEERS];
int          peerCount  = 0;

// Event-Log
#define MAX_EVENTS 50
struct EventEntry {
    char     msg[120];
    uint32_t timestamp;
};
EventEntry   eventLog[MAX_EVENTS];
int          eventCount = 0;
int          eventNext  = 0;

void logEvent(const char* msg) {
    EventEntry& e = eventLog[eventNext % MAX_EVENTS];
    strncpy(e.msg, msg, 119); e.msg[119] = 0;
    e.timestamp = millis() / 1000UL;
    eventNext++;
    if (eventCount < MAX_EVENTS) eventCount++;
    Serial.printf("[EVT] %s\n", msg);
}

String eventLogToJson() {
    String json = "[";
    uint32_t nowSec = millis() / 1000UL;
    for (int i = eventCount - 1; i >= 0; i--) {
        int idx = (eventNext - eventCount + i + MAX_EVENTS) % MAX_EVENTS;
        EventEntry& e = eventLog[idx];
        if (i < eventCount - 1) json += ",";
        uint32_t age = (nowSec >= e.timestamp) ? (nowSec - e.timestamp) : 0;
        json += "{\"age\":" + String(age) + ",\"msg\":\"" + escJson(String(e.msg)) + "\"}";
    }
    return json + "]";
}

// Nachrichtenstatistik
uint32_t     msgIdCounter = 0;
uint32_t     sentTotal    = 0;
uint32_t     recvTotal    = 0;
bool         espNowReady  = false;

// Deferred Peer-Registrierung (esp_now_add_peer aus Callback nicht thread-sicher)
struct PendingPeer { uint8_t mac[6]; char name[32]; };
static PendingPeer pendingPeers[8];
static volatile int pendingPeerCount = 0;

// WiFi-Reconnect: non-blocking
unsigned long lastReconnectAttempt = 0;

// LED (non-blocking)
bool          ledBlinking   = false;
int           ledBlinksDone = 0;
int           ledBlinksMax  = 0;
unsigned long ledBlinkNext  = 0;
unsigned long slowBlinkNext = 0;

// Hub-Heartbeat
unsigned long lastHeartbeat     = 0;
unsigned long heartbeatInterval = (unsigned long)DEFAULT_INTERVAL_S * 1000UL;
unsigned long lastSuccess       = 0;
bool          otaPending        = false;
String        otaUrl            = "";

// ESP-NOW Discovery
unsigned long lastHello = 0;

// BSSID-Lock: gespeicherter AP (Hex-String "AABBCCDDEEFF" oder leer)
String lockedBssid = "";

// Call-State
struct CallState {
    bool     active       = false; // Anruf läuft
    bool     incoming     = false; // Eingehend (true) oder ausgehend (false)
    char     peerName[32] = {};
    uint8_t  peerMac[6]   = {};
    uint32_t startTime    = 0;     // millis() bei Anruf-Start
    uint32_t ringStart    = 0;     // millis() bei erstem Ring
};
CallState callState;
#define CALL_RING_TIMEOUT_MS 30000UL // 30s klingeln dann abbrechen

// Consensus: gesammelte RSSI-Votes anderer Peers
#define MAX_VOTE_APS  10
struct VoteEntry {
    uint8_t bssid[6];
    uint8_t channel;
    int     rssiSum;    // Summe aller gemeldeten RSSIs
    int     voteCount;  // Anzahl Peer-Reports
};
VoteEntry    voteTable[MAX_VOTE_APS];
int          voteCount = 0;
unsigned long lastScanBroadcast = 0;

// Hub-Check bei AP-Wechsel
bool          hubOnCurrentAp   = true;   // Ist Hub auf aktuellem AP erreichbar?
unsigned long lastHubCheck     = 0;
String        hubBssidStr      = "";  // BSSID des AP mit Hub-Zugang


// ================================================================
//  Hilfsfunktionen
// ================================================================

String getMac() {
    String m = WiFi.macAddress();
    m.replace(":", "");
    m.toUpperCase();
    return m;
}

String getMacFmt() {
    String m = WiFi.macAddress();
    m.toUpperCase();
    return m;
}

String getLocalIp() {
    return WiFi.localIP().toString();
}

String fmtUptime(unsigned long s) {
    if (s < 60)   return String(s) + "s";
    if (s < 3600) return String(s / 60) + "min " + String(s % 60) + "s";
    return String(s / 3600) + "h " + String((s % 3600) / 60) + "min";
}

String toMdnsName(const String& name) {
    String r = name;
    r.toLowerCase();
    for (int i = 0; i < (int)r.length(); i++) {
        char c = r[i];
        if (c == ' ' || c == '_') { r[i] = '-'; }
        else if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-')) {
            r.remove(i, 1); i--;
        }
    }
    while (r.startsWith("-")) r.remove(0, 1);
    while (r.endsWith("-"))   r.remove(r.length()-1, 1);
    if (r.length() > 63) r = r.substring(0, 63);
    if (r.length() == 0) r = "esp32";
    return r;
}

void macBytesToStr(const uint8_t* mac, char* buf) {
    sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

bool parseMacStr(const String& s, uint8_t* mac) {
    unsigned int v[6];
    if (sscanf(s.c_str(), "%x:%x:%x:%x:%x:%x",
               &v[0],&v[1],&v[2],&v[3],&v[4],&v[5]) != 6) return false;
    for (int i = 0; i < 6; i++) mac[i] = (uint8_t)v[i];
    return true;
}

bool isBroadcastMac(const uint8_t* mac) {
    for (int i = 0; i < 6; i++) if (mac[i] != 0xFF) return false;
    return true;
}

String escJson(const String& s) {
    String r = s;
    r.replace("\\", "\\\\");
    r.replace("\"", "\\\"");
    r.replace("\n", "\\n");
    r.replace("\r", "");
    return r;
}

String escHtml(const String& s) {
    String r = s;
    r.replace("&", "&amp;");
    r.replace("<", "&lt;");
    r.replace(">", "&gt;");
    r.replace("\"", "&quot;");
    return r;
}

// ================================================================
//  LED (non-blocking)
// ================================================================

void ledSetup() {
    pinMode(MSG_LED_PIN, OUTPUT);
#if MSG_LED_INVERTED
    digitalWrite(MSG_LED_PIN, HIGH);  // OFF
#else
    digitalWrite(MSG_LED_PIN, LOW);   // OFF
#endif
}

void ledSet(bool on) {
#if MSG_LED_INVERTED
    digitalWrite(MSG_LED_PIN, on ? LOW : HIGH);
#else
    digitalWrite(MSG_LED_PIN, on ? HIGH : LOW);
#endif
}

void ledBlink(int count = 3) {
    ledBlinksMax  = count * 2;
    ledBlinksDone = 0;
    ledBlinking   = true;
    ledBlinkNext  = millis();
}

// Slow-blink State: 0=warte, 1=LED an
static int slowBlinkState = 0;
static unsigned long slowBlinkTimer = 0;
static bool slowBlinkInit = false;

void ledTick() {
    unsigned long now = millis();

    // Schnelles Blinken (neue Nachricht) — hat Vorrang
    if (ledBlinking) {
        if (now >= ledBlinkNext) {
            ledSet(ledBlinksDone % 2 == 0);
            ledBlinksDone++;
            ledBlinkNext = now + LED_BLINK_MS;
            if (ledBlinksDone >= ledBlinksMax) {
                ledBlinking = false;
                ledSet(false);
                slowBlinkState = 0;
            }
        }
        return;
    }

    // Langsames Blinken (ungelesene Nachrichten) — komplett non-blocking
    if (unreadCount > 0) {
        if (!slowBlinkInit) { slowBlinkTimer = now + 1000UL; slowBlinkInit = true; }  // 1s Anlaufzeit
        if (slowBlinkState == 0 && now >= slowBlinkTimer) {
            ledSet(true);
            slowBlinkTimer = now + 80UL;   // 80ms an
            slowBlinkState = 1;
        } else if (slowBlinkState == 1 && now >= slowBlinkTimer) {
            ledSet(false);
            slowBlinkTimer = now + SLOW_BLINK_MS;
            slowBlinkState = 0;
        }
    } else {
        slowBlinkState = 0;
        slowBlinkInit = false;
        ledSet(false);
    }
}

// ================================================================
//  Inbox
// ================================================================

void inboxAdd(const char* fromName, const char* fromMac,
              const char* text, bool isBcast, uint32_t msgId) {
    InboxEntry& e = inbox[inboxNext % MAX_INBOX];
    strncpy(e.fromName, fromName, 31); e.fromName[31] = 0;
    strncpy(e.fromMac,  fromMac,  17); e.fromMac[17]  = 0;
    strncpy(e.text,     text,    199); e.text[199]    = 0;
    e.timestamp   = millis() / 1000UL;
    e.msgId       = msgId;
    e.isRead      = false;
    e.isBroadcast = isBcast;
    e.isSent      = false;
    inboxNext++;
    if (inboxCount < MAX_INBOX) inboxCount++;
    unreadCount++;
    recvTotal++;
    char evtbuf2[120];
    snprintf(evtbuf2, sizeof(evtbuf2), "Nachricht von %s: %.60s", fromName, text);
    logEvent(evtbuf2);
    Serial.printf("[MSG] NEU von %s: %.60s\n", fromName, text);
    ledBlink(4);
    // SSE Push an Browser
    ssePush("messages", inboxToJson());
    ssePush("status", handleApiStatusJson());
}

void inboxMarkAllRead() {
    for (int i = 0; i < inboxCount; i++) {
        int idx = (inboxNext - inboxCount + i + MAX_INBOX) % MAX_INBOX;
        inbox[idx].isRead = true;
    }
    unreadCount = 0;
}

String inboxToJson() {
    String json = "[";
    bool first = true;
    for (int i = inboxCount - 1; i >= 0; i--) {
        int idx = (inboxNext - inboxCount + i + MAX_INBOX) % MAX_INBOX;
        InboxEntry& e = inbox[idx];
        if (!first) json += ",";
        first = false;
        uint32_t nowI = millis() / 1000UL;
        uint32_t ageI = (nowI >= e.timestamp) ? (nowI - e.timestamp) : 0;
        json += "{\"from\":\"" + escJson(String(e.fromName)) + "\","
             +  "\"mac\":\""  + String(e.fromMac) + "\","
             +  "\"text\":\"" + escJson(String(e.text)) + "\","
             +  "\"age\":"     + String(ageI) + ","
             +  "\"read\":"   + String(e.isRead ? "true" : "false") + ","
             +  "\"bc\":"     + String(e.isBroadcast ? "true" : "false") + ","
             +  "\"sent\":"   + String(e.isSent ? "true" : "false") + "}";
    }
    return json + "]";
}

// ================================================================
//  Peer-Verwaltung
// ================================================================

// processPendingPeers: nur noch logEvent aus Callback-Kontext in Hauptloop schreiben
// (die eigentliche Peer-Registrierung passiert jetzt direkt im Callback)
void processPendingPeers() {
    if (pendingPeerCount == 0) return;
    int cnt = pendingPeerCount;
    pendingPeerCount = 0;
    for (int i = 0; i < cnt && i < 8; i++) {
        char ms[18]; macBytesToStr(pendingPeers[i].mac, ms);
        Serial.printf("[PEER] Neu: %s (%s)\n", pendingPeers[i].name, ms);
        char evtbuf[120];
        snprintf(evtbuf, sizeof(evtbuf), "Gerät entdeckt: %s (%s)", pendingPeers[i].name, ms);
        logEvent(evtbuf);
        ssePush("peers", peersToJson());
    }
}

void peerUpdate(const char* name, const uint8_t* mac) {
    // Bekannten Peer sofort aktualisieren (lastSeen)
    for (int i = 0; i < peerCount; i++) {
        if (memcmp(knownPeers[i].mac, mac, 6) == 0) {
            strncpy(knownPeers[i].name, name, 31);
            knownPeers[i].lastSeen = millis() / 1000UL;
            return;
        }
    }
    // Neuer Peer → in Queue stellen (sicher aus Callback aufrufbar)
    int slot = pendingPeerCount;
    if (slot < 8) {
        memcpy(pendingPeers[slot].mac, mac, 6);
        strncpy(pendingPeers[slot].name, name, 31);
        pendingPeers[slot].name[31] = 0;
        pendingPeerCount = slot + 1;
    }
}

String peersToJson() {
    String json = "[";
    uint32_t now = millis() / 1000UL;
    for (int i = 0; i < peerCount; i++) {
        PeerEntry& p = knownPeers[i];
        char ms[18]; macBytesToStr(p.mac, ms);
        bool online = (now - p.lastSeen) < 150;
        if (i > 0) json += ",";
        json += "{\"name\":\"" + escJson(String(p.name)) + "\","
             +  "\"mac\":\""   + String(ms) + "\","
             +  "\"ls\":"      + String(p.lastSeen) + ","
             +  "\"online\":"  + String(online ? "true" : "false") + "}";
    }
    return json + "]";
}

// ================================================================
//  ESP-NOW Callbacks
// ================================================================

// ESP32 Core >= 3.3.x: erstes Argument ist wifi_tx_info_t* statt uint8_t*
void onDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
    Serial.printf("[ESP-NOW] TX: %s\n",
                  status == ESP_NOW_SEND_SUCCESS ? "OK" : "FEHLER");
}

// Arduino ESP32 Core 3.x Signatur
void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (len < (int)sizeof(EspNowPacket)) return;
    const EspNowPacket* pkt = (const EspNowPacket*)data;
    char ms[18]; macBytesToStr(info->src_addr, ms);

    if (pkt->type == MSG_TYPE_HELLO) {
        // HELLO: Peer sofort registrieren (sicher — esp_now_add_peer ist re-entrant für neue Peers)
        // Bekannten Peer nur lastSeen aktualisieren
        bool found = false;
        for (int i = 0; i < peerCount; i++) {
            if (memcmp(knownPeers[i].mac, info->src_addr, 6) == 0) {
                strncpy(knownPeers[i].name, pkt->fromName, 31);
                knownPeers[i].lastSeen = millis() / 1000UL;
                found = true; break;
            }
        }
        if (!found && peerCount < MAX_PEERS) {
            // Neuen Peer direkt eintragen
            PeerEntry& p = knownPeers[peerCount];
            strncpy(p.name, pkt->fromName, 31); p.name[31] = 0;
            memcpy(p.mac, info->src_addr, 6);
            p.lastSeen = millis() / 1000UL;
            peerCount++;
            // ESP-NOW Peer registrieren (für Unicast-Antworten)
            if (!esp_now_is_peer_exist(info->src_addr)) {
                esp_now_peer_info_t pi = {};
                memcpy(pi.peer_addr, info->src_addr, 6);
                pi.channel = 0;
                pi.ifidx   = WIFI_IF_STA;  // Core 3.x
                pi.encrypt = false;
                esp_now_add_peer(&pi);
            }
            // Event für Hauptloop vormerken (nur logEvent, kein API-Call)
            int slot = pendingPeerCount;
            if (slot < 8) {
                memcpy(pendingPeers[slot].mac, info->src_addr, 6);
                strncpy(pendingPeers[slot].name, pkt->fromName, 31);
                pendingPeers[slot].name[31] = 0;
                pendingPeerCount = slot + 1;
            }
        }
        return;
    }
    if (pkt->type == MSG_TYPE_SCAN) {
        // Scan-Ergebnisse eines Peers verarbeiten
        const ScanPacket* sp = (const ScanPacket*)data;
        if (len >= (int)sizeof(ScanPacket)) {
            for (int i = 0; i < sp->apCount && i < 5; i++) {
                voteAddAp(sp->aps[i].bssid, sp->aps[i].channel, sp->aps[i].rssi);
            }
            Serial.printf("[VOTE] %d AP-Votes von %s\n", sp->apCount, sp->fromName);
        }
        return;
    }
    if (pkt->type == MSG_TYPE_TEXT) {
        bool bc = isBroadcastMac(pkt->toMac);
        peerUpdate(pkt->fromName, info->src_addr);
        inboxAdd(pkt->fromName, ms, pkt->text, bc, pkt->msgId);
    }
    // Call-Signaling
    if (pkt->type == MSG_TYPE_CALL) {
        if (!callState.active) {
            callState.active   = true;
            callState.incoming = true;
            callState.ringStart = millis();
            strncpy(callState.peerName, pkt->fromName, 31);
            memcpy(callState.peerMac, info->src_addr, 6);
            peerUpdate(pkt->fromName, info->src_addr);
            char evtbuf[80];
            snprintf(evtbuf, sizeof(evtbuf), "Eingehender Anruf von %s", pkt->fromName);
            logEvent(evtbuf);
            ledBlink(8);  // Schnell blinken
            ssePush("call", "{\"type\":\"ring\",\"from\":\"" + escJson(String(pkt->fromName)) + "\",\"mac\":\"" + String(ms) + "\"}");
        }
    }
    if (pkt->type == MSG_TYPE_CALL_A) {
        if (callState.active && !callState.incoming) {
            callState.startTime = millis();
            char evtbuf[80];
            snprintf(evtbuf, sizeof(evtbuf), "Anruf angenommen von %s", pkt->fromName);
            logEvent(evtbuf);
            ssePush("call", "{\"type\":\"accept\",\"from\":\"" + escJson(String(pkt->fromName)) + "\"}");
        }
    }
    if (pkt->type == MSG_TYPE_CALL_R || pkt->type == MSG_TYPE_CALL_E) {
        if (callState.active) {
            const char* reason = pkt->type == MSG_TYPE_CALL_R ? "abgelehnt" : "beendet";
            char evtbuf[80];
            snprintf(evtbuf, sizeof(evtbuf), "Anruf %s von %s", reason, pkt->fromName);
            logEvent(evtbuf);
            ssePush("call", "{\"type\":\"end\",\"reason\":\"" + String(reason) + "\"}");
            memset(&callState, 0, sizeof(callState));
        }
    }
}

// ================================================================
//  ESP-NOW: Init + Senden
// ================================================================

void setupEspNow() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Init fehlgeschlagen!");
        espNowReady = false;
        return;
    }
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    // Broadcast-Peer registrieren
    uint8_t bc[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    esp_now_peer_info_t pi = {};
    memcpy(pi.peer_addr, bc, 6);
    pi.channel = 0;
    pi.ifidx   = WIFI_IF_STA;  // Core 3.x: explizit Station-Interface
    pi.encrypt = false;
    esp_now_add_peer(&pi);

    espNowReady = true;
    uint8_t ch; wifi_second_chan_t sc;
    esp_wifi_get_channel(&ch, &sc);
    Serial.printf("[ESP-NOW] OK — MAC: %s, Kanal: %d\n",
                  WiFi.macAddress().c_str(), ch);
    char chbuf[60];
    snprintf(chbuf, sizeof(chbuf),
             "ESP-NOW aktiv, Kanal %d, MAC %s — alle Communicatoren muessen Kanal %d haben!",
             ch, WiFi.macAddress().c_str(), ch);
    logEvent(chbuf);
    Serial.printf("[ESP-NOW] WICHTIG: Kanal %d — alle ESPs muessen denselben Kanal nutzen!\n", ch);
}

bool espNowSend(const uint8_t* toMac, const String& text) {
    if (!espNowReady) return false;
    EspNowPacket pkt = {};
    pkt.type  = MSG_TYPE_TEXT;
    pkt.msgId = ++msgIdCounter;
    strncpy(pkt.fromName, deviceName.c_str(), 31);
    memcpy(pkt.toMac, toMac, 6);
    strncpy(pkt.text, text.c_str(), 199);
    bool ok = esp_now_send(toMac, (uint8_t*)&pkt, sizeof(pkt)) == ESP_OK;
    if (ok) {
        sentTotal++;
        // Gesendete Nachricht im Chat-Verlauf speichern
        InboxEntry& se = inbox[inboxNext % MAX_INBOX];
        strncpy(se.fromName, deviceName.c_str(), 31); se.fromName[31] = 0;
        strncpy(se.fromMac, getMacFmt().c_str(), 17); se.fromMac[17] = 0;
        strncpy(se.text, text.c_str(), 199); se.text[199] = 0;
        se.timestamp   = millis() / 1000UL;
        se.msgId       = pkt.msgId;
        se.isRead      = true;
        se.isBroadcast = isBroadcastMac(toMac);
        se.isSent      = true;
        inboxNext++;
        if (inboxCount < MAX_INBOX) inboxCount++;
        ssePush("messages", inboxToJson());
        ssePush("status", handleApiStatusJson());
    }
    return ok;
}

void espNowCall(const uint8_t* toMac, uint8_t callType) {
    if (!espNowReady) return;
    EspNowPacket pkt = {};
    pkt.type  = callType;
    pkt.msgId = ++msgIdCounter;
    strncpy(pkt.fromName, deviceName.c_str(), 31);
    memcpy(pkt.toMac, toMac, 6);
    // text-Feld: Kurzstatus für spätere Erweiterungen
    strncpy(pkt.text, callType == MSG_TYPE_CALL ? "ring" :
                      callType == MSG_TYPE_CALL_A ? "accept" :
                      callType == MSG_TYPE_CALL_R ? "reject" : "end", 15);
    esp_now_send(toMac, (uint8_t*)&pkt, sizeof(pkt));
}

void espNowHello() {
    if (!espNowReady) return;
    EspNowPacket pkt = {};
    pkt.type  = MSG_TYPE_HELLO;
    pkt.msgId = ++msgIdCounter;
    strncpy(pkt.fromName, deviceName.c_str(), 31);
    uint8_t bc[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    esp_now_send(bc, (uint8_t*)&pkt, sizeof(pkt));
}



// ================================================================
//  RSSI-Consensus: Scan-Ergebnisse verteilen + besten AP wählen
// ================================================================

void voteAddAp(const uint8_t* bssid, uint8_t ch, int rssi) {
    // Vorhandenen Eintrag aktualisieren
    for (int i = 0; i < voteCount; i++) {
        if (memcmp(voteTable[i].bssid, bssid, 6) == 0) {
            voteTable[i].rssiSum += rssi;
            voteTable[i].voteCount++;
            return;
        }
    }
    // Neuer Eintrag
    if (voteCount >= MAX_VOTE_APS) return;
    memcpy(voteTable[voteCount].bssid, bssid, 6);
    voteTable[voteCount].channel   = ch;
    voteTable[voteCount].rssiSum   = rssi;
    voteTable[voteCount].voteCount = 1;
    voteCount++;
}

// Eigene Scan-Ergebnisse per ESP-NOW broadcasten
void broadcastScan() {
    if (!espNowReady) return;
    int n = WiFi.scanNetworks(false, false, false, 200);
    if (n <= 0) { WiFi.scanDelete(); return; }

    // Eigene Scan-Votes eintragen
    for (int i = 0; i < n; i++) {
        voteAddAp(WiFi.BSSID(i), (uint8_t)WiFi.channel(i), WiFi.RSSI(i));
    }

    // Kompakt senden: max 5 APs pro Paket
    ScanPacket pkt = {};
    pkt.type = MSG_TYPE_SCAN;
    strncpy(pkt.fromName, deviceName.c_str(), 31);
    int sent = 0;
    for (int i = 0; i < n && i < 5; i++) {
        memcpy(pkt.aps[i].bssid, WiFi.BSSID(i), 6);
        int rssiVal = WiFi.RSSI(i);
        pkt.aps[i].rssi    = (int8_t)(rssiVal < -128 ? -128 : rssiVal);
        pkt.aps[i].channel = (uint8_t)WiFi.channel(i);
        sent++;
    }
    pkt.apCount = sent;
    WiFi.scanDelete();

    uint8_t bc[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    esp_now_send(bc, (uint8_t*)&pkt, sizeof(ScanPacket));
    Serial.printf("[VOTE] Scan gesendet: %d APs\n", sent);
}

// Besten AP laut Consensus ermitteln
// Kriterium: höchste mittlere RSSI mit mindestens 2 Votes (eigener + 1 Peer)
bool consensusBestAp(uint8_t* bestBssid, uint8_t* bestCh, float* bestAvg) {
    if (voteCount == 0) return false;
    *bestAvg = -999;
    bool found = false;
    for (int i = 0; i < voteCount; i++) {
        VoteEntry& v = voteTable[i];
        if (v.voteCount < 2) continue; // Mindestens 2 übereinstimmende Reports
        float avg = (float)v.rssiSum / v.voteCount;
        if (avg > *bestAvg) {
            *bestAvg = avg;
            memcpy(bestBssid, v.bssid, 6);
            *bestCh = v.channel;
            found = true;
        }
    }
    return found;
}

// Consensus auswerten und ggf. AP wechseln
void evaluateConsensus() {
    uint8_t bestBssid[6]; uint8_t bestCh; float bestAvg;
    if (!consensusBestAp(bestBssid, &bestCh, &bestAvg)) {
        Serial.println("[VOTE] Kein Consensus — nicht genug Daten");
        return;
    }
    // Aktueller AP
    uint8_t* cur = WiFi.BSSID();
    if (cur && memcmp(cur, bestBssid, 6) == 0) {
        Serial.printf("[VOTE] Bereits auf bestem AP (Ø %.0f dBm)\n", bestAvg);
        return;
    }
    char bssidStr[18];
    snprintf(bssidStr, sizeof(bssidStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             bestBssid[0],bestBssid[1],bestBssid[2],
             bestBssid[3],bestBssid[4],bestBssid[5]);
    char evtbuf[120];
    snprintf(evtbuf, sizeof(evtbuf),
             "Consensus: Wechsle zu %s Kanal %d (Ø %.0f dBm, %d Votes)",
             bssidStr, bestCh, bestAvg,
             voteTable[0].voteCount); // approximate
    logEvent(evtbuf);
    Serial.println("[VOTE] " + String(evtbuf));

    // Zu bestem AP wechseln
    String ssid = WiFi.SSID();
    String psk  = WiFi.psk();
    WiFi.disconnect(false); delay(200);
    WiFi.begin(ssid.c_str(), psk.c_str(), 0, bestBssid);
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis()-t < 10000) {
        delay(200); webServer.handleClient();
    }
    if (WiFi.status() != WL_CONNECTED) {
        logEvent("Consensus-Wechsel fehlgeschlagen — reconnect...");
        WiFi.begin(ssid.c_str(), psk.c_str());
        t = millis();
        while (WiFi.status() != WL_CONNECTED && millis()-t < 10000) delay(200);
    } else {
        WiFi.setSleep(false);
        esp_wifi_set_ps(WIFI_PS_NONE);
        // BSSID-Lock setzen
        char lockStr[13];
        snprintf(lockStr, sizeof(lockStr), "%02X%02X%02X%02X%02X%02X",
                 bestBssid[0],bestBssid[1],bestBssid[2],
                 bestBssid[3],bestBssid[4],bestBssid[5]);
        lockedBssid = String(lockStr);
        prefs.begin("espcomm", false);
        prefs.putString("bssid", lockedBssid);
        prefs.end();
        uint8_t ch; wifi_second_chan_t sc;
        esp_wifi_get_channel(&ch, &sc);
        snprintf(evtbuf, sizeof(evtbuf),
                 "Consensus-AP aktiv: Kanal %d, BSSID-Lock gespeichert", ch);
        logEvent(evtbuf);
    }
    // Votes zurücksetzen für nächste Runde
    voteCount = 0;
    memset(voteTable, 0, sizeof(voteTable));
}

// ================================================================
//  Hub-Reconnect: periodisch Hub-Check wenn AP gewechselt
// ================================================================

void hubReconnectCheck() {
    if (hubOnCurrentAp) return;  // Hub erreichbar — nichts tun
    if (WiFi.status() != WL_CONNECTED) return;

    String ssid = WiFi.SSID();
    String psk  = WiFi.psk();
    uint8_t* curBssid = WiFi.BSSID();

    // Merke aktuellen AP
    uint8_t savedBssid[6];
    if (curBssid) memcpy(savedBssid, curBssid, 6);
    else return;

    // Zu Hub-AP wechseln
    uint8_t hubBssid[6]; bool hubBssidOk = false;
    if (hubBssidStr.length() == 12) {
        for (int i = 0; i < 6; i++) {
            String b = hubBssidStr.substring(i*2, i*2+2);
            hubBssid[i] = (uint8_t)strtol(b.c_str(), nullptr, 16);
        }
        hubBssidOk = true;
    }

    logEvent("Hub-Check: kurz zu Hub-AP wechseln...");
    WiFi.disconnect(false); delay(200);
    if (hubBssidOk)
        WiFi.begin(ssid.c_str(), psk.c_str(), 0, hubBssid);
    else
        WiFi.begin(ssid.c_str(), psk.c_str());

    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis()-t < 8000) delay(200);

    if (WiFi.status() == WL_CONNECTED) {
        WiFi.setSleep(false);
        esp_wifi_set_ps(WIFI_PS_NONE);
        // Hub-Heartbeat senden (inkl. OTA-Check)
        sendHeartbeat();
        logEvent("Hub-Check: Heartbeat gesendet, kehre zu Kommunikations-AP zurueck");
    }

    // Zurück zum Kommunikations-AP
    WiFi.disconnect(false); delay(200);
    WiFi.begin(ssid.c_str(), psk.c_str(), 0, savedBssid);
    t = millis();
    while (WiFi.status() != WL_CONNECTED && millis()-t < 8000) delay(200);
    if (WiFi.status() == WL_CONNECTED) {
        WiFi.setSleep(false);
        esp_wifi_set_ps(WIFI_PS_NONE);
    }
}

// ================================================================
//  Auto-Channel-Sync
//  Wenn nach PEER_SEARCH_TIMEOUT_S keine Peers gefunden wurden:
//  Alle APs mit gleicher SSID durchprobieren bis Peers antworten
// ================================================================

#define PEER_SEARCH_TIMEOUT_S  90   // 90s ohne Peer → AP-Suche starten
#define PEER_SEARCH_HELLO_MS   2000 // 2s Hello-Pause pro AP zum Antworten

bool     channelSyncRunning  = false;
unsigned long channelSyncStart = 0;   // Wann wurde gestartet
unsigned long lastPeerFound    = 0;   // Zeitpunkt letzter Peer

void doChannelSync() {
    if (!espNowReady) return;
    String currentSsid = WiFi.SSID();
    String currentBssid = "";
    uint8_t* cb = WiFi.BSSID();
    if (cb) {
        char buf[18];
        snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
                 cb[0],cb[1],cb[2],cb[3],cb[4],cb[5]);
        currentBssid = String(buf);
    }

    Serial.println("[SYNC] Starte AP-Kanalsuche — SSID: " + currentSsid);
    logEvent(("Kanalsuche: scanne APs mit SSID " + currentSsid).c_str());

    // WiFi-Netzwerke scannen
    int n = WiFi.scanNetworks(false, false, false, 300);
    if (n <= 0) {
        Serial.println("[SYNC] Keine Netzwerke gefunden");
        return;
    }

    // Alle APs mit gleicher SSID sammeln
    struct ApInfo { uint8_t bssid[6]; int ch; int rssi; };
    ApInfo candidates[10];
    int cCount = 0;

    for (int i = 0; i < n && cCount < 10; i++) {
        if (WiFi.SSID(i) != currentSsid) continue;
        char bssidStr[18];
        uint8_t* b = WiFi.BSSID(i);
        snprintf(bssidStr, sizeof(bssidStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                 b[0],b[1],b[2],b[3],b[4],b[5]);
        if (String(bssidStr) == currentBssid) continue; // Aktuellen AP überspringen
        memcpy(candidates[cCount].bssid, b, 6);
        candidates[cCount].ch   = WiFi.channel(i);
        candidates[cCount].rssi = WiFi.RSSI(i);
        cCount++;
        Serial.printf("[SYNC] Kandidat: %s Kanal %d (%d dBm)\n",
                      bssidStr, WiFi.channel(i), WiFi.RSSI(i));
    }
    WiFi.scanDelete();

    if (cCount == 0) {
        Serial.println("[SYNC] Kein anderer AP mit gleicher SSID gefunden");
        logEvent(("Kanalsuche: kein anderer AP gefunden — bleibe auf Kanal " + String(WiFi.channel())).c_str());
        return;
    }

    // Jeden Kandidaten kurz ausprobieren
    String savedPsk = WiFi.psk();
    for (int i = 0; i < cCount; i++) {
        char bssidStr[18];
        snprintf(bssidStr, sizeof(bssidStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                 candidates[i].bssid[0], candidates[i].bssid[1],
                 candidates[i].bssid[2], candidates[i].bssid[3],
                 candidates[i].bssid[4], candidates[i].bssid[5]);

        Serial.printf("[SYNC] Versuche AP %s Kanal %d...\n", bssidStr, candidates[i].ch);
        WiFi.disconnect(false);
        delay(200);
        WiFi.begin(currentSsid.c_str(), savedPsk.c_str(), 0, candidates[i].bssid);

        unsigned long t = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t < 8000) {
            delay(200);
            webServer.handleClient(); // UI reaktionsfähig halten
        }

        if (WiFi.status() != WL_CONNECTED) {
            Serial.printf("[SYNC] AP %s nicht erreichbar\n", bssidStr);
            continue;
        }

        // Verbunden — ESP-NOW Power Save sicherstellen
        WiFi.setSleep(false);
        esp_wifi_set_ps(WIFI_PS_NONE);
        uint8_t ch; wifi_second_chan_t sc;
        esp_wifi_get_channel(&ch, &sc);
        Serial.printf("[SYNC] Verbunden mit %s, Kanal %d\n", bssidStr, ch);

        // Hello senden und auf Peers warten
        espNowHello();
        delay(PEER_SEARCH_HELLO_MS);
        espNowHello();
        delay(PEER_SEARCH_HELLO_MS);

        if (peerCount > 0) {
            // Peers gefunden! Diesen AP als Lock speichern
            char lockStr[13];
            snprintf(lockStr, sizeof(lockStr), "%02X%02X%02X%02X%02X%02X",
                     candidates[i].bssid[0], candidates[i].bssid[1],
                     candidates[i].bssid[2], candidates[i].bssid[3],
                     candidates[i].bssid[4], candidates[i].bssid[5]);
            lockedBssid = String(lockStr);
            prefs.begin("espcomm", false);
            prefs.putString("bssid", lockedBssid);
            prefs.end();
            char evtbuf[120];
            snprintf(evtbuf, sizeof(evtbuf),
                     "Kanal-Sync: %d Peer(s) auf Kanal %d gefunden! AP %s gespeichert.",
                     peerCount, ch, bssidStr);
            logEvent(evtbuf);
            Serial.println("[SYNC] " + String(evtbuf));
            lastPeerFound = millis() / 1000UL;
            return; // Fertig
        }
        Serial.printf("[SYNC] Keine Peers auf Kanal %d\n", ch);
    }

    // Keinen passenden AP gefunden — zum ursprünglichen zurück
    Serial.println("[SYNC] Kein AP mit Peers — kehre zurück zu: " + currentBssid);
    logEvent("Kanalsuche: kein AP mit Peers — bleibe beim ursprünglichen AP");
    if (cb) {
        WiFi.disconnect(false);
        delay(200);
        WiFi.begin(currentSsid.c_str(), savedPsk.c_str(), 0, cb);
        unsigned long t = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t < 8000) delay(200);
        WiFi.setSleep(false);
        esp_wifi_set_ps(WIFI_PS_NONE);
    }
}

// ================================================================
//  Web-UI Hauptseite
// ================================================================

// ================================================================
//  Web-UI als Raw String (kein Escape-Aufwand, OTA-kompatibel)
// ================================================================

// SSE: Liste aktiver Clients
#define MAX_SSE_CLIENTS 4
struct SseClient {
    WiFiClient client;
    bool       active = false;
};
SseClient sseClients[MAX_SSE_CLIENTS];

// SSE-Event an alle verbundenen Browser senden
void ssePush(const char* event, const String& data) {
    for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
        if (!sseClients[i].active) continue;
        if (!sseClients[i].client.connected()) {
            sseClients[i].active = false;
            continue;
        }
        sseClients[i].client.printf("event: %s\ndata: %s\n\n", event, data.c_str());
    }
}

// SSE-Verbindung aufbauen
void handleSse() {
    // Freien Slot suchen
    int slot = -1;
    for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
        if (!sseClients[i].active || !sseClients[i].client.connected()) {
            slot = i; break;
        }
    }
    if (slot < 0) {
        webServer.send(503, "text/plain", "Zu viele SSE-Verbindungen");
        return;
    }
    WiFiClient client = webServer.client();
    client.print(
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/event-stream\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: keep-alive\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "\r\n"
    );
    sseClients[slot].client = client;
    sseClients[slot].active = true;
    Serial.printf("[SSE] Client %d verbunden\n", slot);
    // Sofort aktuellen Status senden
    ssePush("status", handleApiStatusJson());
    ssePush("messages", inboxToJson());
    ssePush("peers", peersToJson());
}

// Status als JSON String (für SSE + REST)
String handleApiStatusJson() {
    uint8_t chN; wifi_second_chan_t scN;
    esp_wifi_get_channel(&chN, &scN);
    String json = "{";
    json += "\"name\":\"" + escJson(deviceName) + "\",";
    json += "\"mac\":\"" + getMacFmt() + "\",";
    json += "\"ip\":\"" + getLocalIp() + "\",";
    json += "\"uptime\":" + String(millis() / 1000UL) + ",";
    json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
    json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
    json += "\"espNow\":" + String(espNowReady ? "true" : "false") + ",";
    json += "\"channel\":" + String(chN) + ",";
    json += "\"peers\":" + String(peerCount) + ",";
    json += "\"sent\":" + String(sentTotal) + ",";
    json += "\"recv\":" + String(recvTotal) + ",";
    json += "\"unread\":" + String(unreadCount) + ",";
    json += "\"bssid\":\"" + lockedBssid + "\",";
    json += "\"version\":\"" + String(FW_VERSION) + "\"";
    json += "}";
    return json;
}

void handleRoot() {
    // Peer-Dropdown Optionen vorab bauen (C++ Seite)
    String peerOptions = "<option value='bc'>\xF0\x9F\x93\xA2 An alle (Broadcast)</option>";
    for (int i = 0; i < peerCount; i++) {
        char ms[18]; macBytesToStr(knownPeers[i].mac, ms);
        peerOptions += "<option value='" + String(ms) + "'>"
                     + escHtml(String(knownPeers[i].name))
                     + " (" + String(ms).substring(12) + ")</option>";
    }

    // BSSID-Lock Button
    String bssidSection;
    if (lockedBssid.length() == 12) {
        bssidSection = "<span style='color:#3fb950;font-family:monospace'>" + lockedBssid + "</span>"
                       " <button class='btn-sm' onclick='clearBssid()'>&#128275; Aufheben</button>";
    } else {
        bssidSection = "<span style='color:#8b949e'>nicht gesetzt</span>"
                       " <button class='btn-sm' onclick='lockBssid()'>&#128274; AP sperren</button>";
    }

    // Kanal ermitteln
    uint8_t chD; wifi_second_chan_t scD;
    esp_wifi_get_channel(&chD, &scD);

    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    webServer.send(200, "text/html; charset=utf-8", "");

    // ── Chunk 1: HEAD + CSS ──────────────────────────────────────
    webServer.sendContent(R"rawhtml(<!DOCTYPE html>
<html lang="de">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP Communicator</title>
<script src="https://cdn.rawgit.com/Mikhus/canvas-gauges/gh-pages/download/2.1.7/all/gauge.min.js"></script>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d1117;color:#e6edf3;font-family:system-ui,sans-serif;font-size:14px}
header{background:#161b22;border-bottom:1px solid #30363d;padding:12px 20px;
  display:flex;align-items:center;gap:10px;flex-wrap:wrap}
h1{font-size:17px;color:#58a6ff;display:flex;align-items:center;gap:8px}
.badge{padding:2px 8px;border-radius:10px;font-size:11px;font-weight:600;white-space:nowrap}
.bc-blue{background:rgba(88,166,255,.15);color:#58a6ff;border:1px solid rgba(88,166,255,.3)}
.bc-green{background:rgba(63,185,80,.15);color:#3fb950;border:1px solid rgba(63,185,80,.3)}
.bc-red{background:rgba(248,81,73,.15);color:#f85149;border:1px solid rgba(248,81,73,.3)}
.bc-purple{background:rgba(188,140,255,.15);color:#bc8cff;border:1px solid rgba(188,140,255,.3)}
.bc-yellow{background:rgba(227,179,65,.15);color:#e3b341;border:1px solid rgba(227,179,65,.3)}
.tabs{display:flex;background:#161b22;border-bottom:1px solid #30363d;overflow-x:auto;gap:0}
.tab{padding:10px 18px;cursor:pointer;border-bottom:3px solid transparent;
  color:#8b949e;font-weight:500;white-space:nowrap;user-select:none;transition:color .15s}
.tab.active,.tab:hover{color:#58a6ff;border-bottom-color:#58a6ff}
.pane{display:none;padding:16px;max-width:900px;margin:0 auto}
.pane.active{display:block}
.card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:14px;margin-bottom:12px}
.card h3{font-size:11px;color:#8b949e;text-transform:uppercase;letter-spacing:.5px;margin-bottom:10px}
input,textarea,select{background:#0d1117;border:1px solid #30363d;color:#e6edf3;
  border-radius:6px;padding:8px 10px;font-size:13px;width:100%;outline:none}
input:focus,textarea:focus,select:focus{border-color:#58a6ff}
textarea{resize:vertical;min-height:70px;font-family:inherit}
.btn-send{background:#238636;color:#fff;border:none;padding:10px;border-radius:6px;
  cursor:pointer;font-size:14px;font-weight:600;width:100%;margin-top:8px}
.btn-send:hover{background:#2ea043}
.btn-sm{background:#21262d;color:#8b949e;border:1px solid #30363d;
  padding:4px 10px;font-size:12px;border-radius:6px;cursor:pointer}
.btn-sm:hover{border-color:#58a6ff;color:#58a6ff}
.btn-danger{background:#6e1b1b;color:#f85149;border:1px solid #f85149;
  padding:4px 10px;font-size:12px;border-radius:6px;cursor:pointer}
.chat-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:8px}
.emoji-bar{display:flex;gap:4px;flex-wrap:wrap;padding:6px 0;border-bottom:1px solid #21262d;margin-bottom:8px}
.emoji-btn{cursor:pointer;padding:2px 5px;border-radius:4px;background:none;
  border:none;font-size:18px;line-height:1}
.emoji-btn:hover{background:#21262d}
.msg-list{display:flex;flex-direction:column;gap:6px}
.msg{background:#0d1117;border:1px solid #21262d;border-radius:8px;padding:10px 12px}
.msg.unread{border-left:3px solid #58a6ff}
.msg.sent{background:#0d2016;border-left:3px solid #3fb950}
.msg-head{display:flex;justify-content:space-between;align-items:center;margin-bottom:5px;flex-wrap:wrap;gap:4px}
.msg-from{font-weight:600;font-size:13px}
.msg-from.me{color:#3fb950}
.msg-meta{font-size:11px;color:#8b949e;display:flex;gap:6px;align-items:center}
.msg-text{color:#c9d1d9;font-size:13px;line-height:1.5;word-break:break-word}
.peer-card{background:#0d1117;border:1px solid #21262d;border-radius:8px;
  padding:10px 12px;display:flex;align-items:center;gap:10px;cursor:pointer;margin-bottom:6px}
.peer-card:hover{border-color:#30363d}
.peer-dot{width:9px;height:9px;border-radius:50%;flex-shrink:0}
.peer-name{font-weight:600;font-size:13px}
.peer-mac{font-size:11px;color:#8b949e;font-family:monospace}
.stat-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(140px,1fr));gap:8px;margin-bottom:14px}
.stat-box{background:#0d1117;border:1px solid #21262d;border-radius:8px;padding:12px;text-align:center}
.stat-val{font-size:26px;font-weight:700;color:#58a6ff;line-height:1}
.stat-lbl{font-size:11px;color:#8b949e;margin-top:4px}
table{width:100%;border-collapse:collapse;font-size:13px}
td{padding:7px 8px;border-bottom:1px solid #21262d}
td:first-child{color:#8b949e;width:42%}
.gauge-row{display:flex;justify-content:center;gap:24px;flex-wrap:wrap;padding:8px 0}
.gauge-box{text-align:center}
.gauge-lbl{font-size:12px;color:#8b949e;margin-top:4px}
.log-entry{display:flex;justify-content:space-between;padding:7px 8px;
  border-bottom:1px solid #21262d;font-size:13px;gap:8px}
.log-msg{color:#c9d1d9;flex:1}
.log-age{color:#8b949e;flex-shrink:0;font-size:11px}
.empty{color:#8b949e;text-align:center;padding:24px;font-size:13px}
.warn{background:rgba(227,179,65,.08);border:1px solid rgba(227,179,65,.25);
  border-radius:6px;padding:8px 12px;color:#e3b341;font-size:12px;margin-bottom:8px}
#toast{position:fixed;bottom:20px;left:50%;transform:translateX(-50%);
  background:#238636;color:#fff;padding:8px 20px;border-radius:20px;
  font-size:13px;opacity:0;transition:opacity .3s;pointer-events:none;z-index:99}
#toast.show{opacity:1}
.modal-overlay{display:none;position:fixed;inset:0;background:rgba(0,0,0,.75);
  z-index:100;align-items:center;justify-content:center}
.modal-overlay.show{display:flex}
.modal{background:#161b22;border:1px solid #30363d;border-radius:12px;
  padding:28px;text-align:center;max-width:320px;width:90%;
  box-shadow:0 8px 32px rgba(0,0,0,.5)}
.modal h2{font-size:18px;margin-bottom:6px;color:#e6edf3}
.modal p{color:#8b949e;font-size:13px;margin-bottom:20px}
.call-avatar{font-size:52px;margin-bottom:12px;animation:pulse 1s infinite}
@keyframes pulse{0%,100%{transform:scale(1)}50%{transform:scale(1.08)}}
.call-btns{display:flex;gap:12px;justify-content:center}
.btn-call{background:#238636;color:#fff;border:none;padding:12px 24px;
  border-radius:24px;cursor:pointer;font-size:15px;font-weight:600}
.btn-call:hover{background:#2ea043}
.btn-hangup{background:#b91c1c;color:#fff;border:none;padding:12px 24px;
  border-radius:24px;cursor:pointer;font-size:15px;font-weight:600}
.btn-hangup:hover{background:#dc2626}
.btn-call-sm{background:#1a3a2a;color:#3fb950;border:1px solid #3fb950;
  padding:5px 12px;border-radius:16px;cursor:pointer;font-size:12px;
  font-weight:600;display:inline-flex;align-items:center;gap:4px}
.btn-call-sm:hover{background:#238636;color:#fff}
.name-form{display:flex;gap:8px;margin-top:8px}
.name-form input{flex:1;padding:6px 10px;font-size:13px}
.name-form button{background:#238636;color:#fff;border:none;
  padding:6px 14px;border-radius:6px;cursor:pointer;white-space:nowrap}
</style>
</head>
<body>
)rawhtml");

    // ── Chunk 2: Header + Tabs (mit dynamischen Werten) ──────────
    String h2;
    h2.reserve(1200);
    h2 = "<header><h1>&#128172; ";
    h2 += escHtml(deviceName);
    h2 += "</h1><span class='badge bc-blue'>v";
    h2 += FW_VERSION;
    h2 += "</span>";
    h2 += espNowReady ?
          "<span class='badge bc-green'>ESP-NOW &#9679;</span>" :
          "<span class='badge bc-red'>ESP-NOW &#9679;</span>";
    if (unreadCount > 0) {
        h2 += "<span class='badge bc-yellow' id='ubadge'>&#9993; ";
        h2 += String(unreadCount);
        h2 += " neu</span>";
    }
    h2 += "</header>\n";
    h2 += "<div class='tabs'>"
          "<div class='tab active' onclick='showTab(\"chat\",this)'>&#128172; Chat</div>"
          "<div class='tab' onclick='showTab(\"devs\",this)'>&#128241; Ger&auml;te</div>"
          "<div class='tab' onclick='showTab(\"stat\",this)'>&#128202; Status</div>"
          "<div class='tab' onclick='showTab(\"log\",this)'>&#128221; Log</div>"
          "<a class='tab' href='/ota'>&#128640; OTA</a>"
          "</div>\n";
    webServer.sendContent(h2);

    // ── Chunk 3: Chat-Pane ───────────────────────────────────────
    String h3;
    h3.reserve(1500);
    h3 = "<div id='pane-chat' class='pane active'>"
         "<div class='card'>"
         "<select id='sel-to' style='margin-bottom:8px'>";
    h3 += peerOptions;
    h3 += "</select>"
           "<div class='name-form'>"
           "<input id='my-name-input' placeholder='Mein Name' maxlength='31' value='";
    h3 += escHtml(deviceName);
    h3 += "'";
    h3 +=           "'>"
           "<button onclick='setMyName()'>&#128336; Speichern</button>"
           "</div>"
          "<div class='emoji-bar'>"
          "<button class='emoji-btn' onclick='addEmoji(\"&#128512;\")' title='Smile'>&#128512;</button>"
          "<button class='emoji-btn' onclick='addEmoji(\"&#128077;\")' title='Daumen hoch'>&#128077;</button>"
          "<button class='emoji-btn' onclick='addEmoji(\"&#128078;\")' title='Daumen runter'>&#128078;</button>"
          "<button class='emoji-btn' onclick='addEmoji(\"&#128293;\")' title='Feuer'>&#128293;</button>"
          "<button class='emoji-btn' onclick='addEmoji(\"&#10084;\")' title='Herz'>&#10084;</button>"
          "<button class='emoji-btn' onclick='addEmoji(\"&#128075;\")' title='Winken'>&#128075;</button>"
          "<button class='emoji-btn' onclick='addEmoji(\"&#128514;\")' title='Lachen'>&#128514;</button>"
          "<button class='emoji-btn' onclick='addEmoji(\"&#128532;\")' title='Hm'>&#128532;</button>"
          "<button class='emoji-btn' onclick='addEmoji(\"&#9989;\")' title='Check'>&#9989;</button>"
          "<button class='emoji-btn' onclick='addEmoji(\"&#10060;\")' title='X'>&#10060;</button>"
          "<button class='emoji-btn' onclick='addEmoji(\"&#128276;\")' title='Glocke'>&#128276;</button>"
          "<button class='emoji-btn' onclick='addEmoji(\"&#127881;\")' title='Party'>&#127881;</button>"
          "</div>"
          "<textarea id='msg-text' placeholder='Nachricht...' maxlength='199' rows='2'></textarea>"
          "<button class='btn-send' onclick='sendMsg()'>&#9658; Senden</button>"
          "</div>"
          "<div class='card'>"
          "<div class='chat-header'>"
          "<h3 style='margin:0'>&#128172; Chat (<span id='ucnt'>";
    h3 += String(unreadCount);
    h3 += "</span> ungelesen)</h3>"
          "<div style='display:flex;gap:6px'>"
          "<button class='btn-sm' onclick='markRead()'>&#10003; Gelesen</button>"
          "<button class='btn-danger' onclick='clearChat()'>&#128465; Leeren</button>"
          "</div></div>"
          "<div class='msg-list' id='msg-list'>"
          "<div class='empty'>&#8987; Lade...</div>"
          "</div></div></div>\n";
    webServer.sendContent(h3);

    // ── Chunk 4: Geräte-Pane ─────────────────────────────────────
    String h4;
    h4.reserve(1200);
    h4 = "<div id='pane-devs' class='pane'>"
         "<div class='card'>"
         "<div class='chat-header'>"
         "<h3 style='margin:0'>Bekannte Ger&auml;te (<span id='peer-cnt'>";
    h4 += String(peerCount);
    h4 += "</span>)</h3>"
          "<button class='btn-sm' onclick='loadPeers()'>&#8635; Aktualisieren</button>"
          "</div><br>"
          "<div id='peer-list'>";
    if (peerCount == 0) {
        h4 += "<div class='empty'>Noch keine Ger&auml;te entdeckt</div>";
    } else {
        uint32_t nowP = millis() / 1000UL;
        for (int i = 0; i < peerCount; i++) {
            PeerEntry& p = knownPeers[i];
            char ms[18]; macBytesToStr(p.mac, ms);
            bool online = (nowP - p.lastSeen) < 150;
            h4 += "<div class='peer-card' onclick='selectPeer(\"";
            h4 += String(ms);
            h4 += "\",\"";
            h4 += escHtml(String(p.name));
            h4 += "\")'><div class='peer-dot' style='background:";
            h4 += online ? "#3fb950" : "#555";
            h4 += "'></div><div style='flex:1'><div class='peer-name'>";
            h4 += escHtml(String(p.name));
            h4 += "</div><div class='peer-mac'>";
            h4 += String(ms);
            h4 += "</div></div><span class='badge ";
            h4 += online ? "bc-green" : "bc-red";
            h4 += "'>";
            h4 += online ? "online" : "offline";
            h4 += "</span></div>";
        }
    }
    h4 += "</div></div></div>\n";
    webServer.sendContent(h4);

    // ── Chunk 5: Status-Pane mit canvas-gauges ───────────────────
    String h5;
    h5.reserve(1600);
    h5 = "<div id='pane-stat' class='pane'>"
         "<div class='stat-grid'>"
         "<div class='stat-box'><div class='stat-val' id='s-ch'>";
    h5 += String(chD);
    h5 += "</div><div class='stat-lbl'>WiFi-Kanal</div></div>"
          "<div class='stat-box'><div class='stat-val' id='s-peers'>";
    h5 += String(peerCount);
    h5 += "</div><div class='stat-lbl'>Bekannte ESPs</div></div>"
          "<div class='stat-box'><div class='stat-val' id='s-recv'>";
    h5 += String(recvTotal);
    h5 += "</div><div class='stat-lbl'>Empfangen</div></div>"
          "<div class='stat-box'><div class='stat-val' id='s-sent'>";
    h5 += String(sentTotal);
    h5 += "</div><div class='stat-lbl'>Gesendet</div></div>"
          "<div class='stat-box'><div class='stat-val' id='s-unread'>";
    h5 += String(unreadCount);
    h5 += "</div><div class='stat-lbl'>Ungelesen</div></div>"
          "</div>"
          // canvas-gauges: RSSI + Heap
          "<div class='card'>"
          "<h3>Radio &amp; Speicher</h3>"
          "<div class='gauge-row'>"
          "<div class='gauge-box'><canvas id='g-rssi'></canvas><div class='gauge-lbl'>WLAN RSSI (dBm)</div></div>"
          "<div class='gauge-box'><canvas id='g-heap'></canvas><div class='gauge-lbl'>Freier Heap (KB)</div></div>"
          "</div></div>"
          "<div class='card'><h3>Ger&auml;t</h3><table>"
          "<tr><td>Name</td><td id='s-name'>";
    h5 += escHtml(deviceName);
    h5 += "</td></tr><tr><td>Chip</td><td>";
    h5 += String(ESP.getChipModel());
    h5 += "</td></tr><tr><td>MAC (WiFi)</td><td style='font-family:monospace'>";
    h5 += getMacFmt();
    h5 += "</td></tr><tr><td>IP-Adresse</td><td style='font-family:monospace' id='s-ip'>";
    h5 += getLocalIp();
    h5 += "</td></tr><tr><td>WLAN RSSI</td><td id='s-rssi'>";
    h5 += String(WiFi.RSSI());
    h5 += " dBm</td></tr><tr><td>Uptime</td><td id='s-uptime'>";
    h5 += fmtUptime(millis() / 1000UL);
    h5 += "</td></tr><tr><td>Firmware</td><td>";
    h5 += FW_VERSION;
    h5 += "</td></tr></table></div>"
          "<div class='card'><h3>ESP-NOW &amp; WiFi</h3><table>"
          "<tr><td>ESP-NOW</td><td>";
    h5 += espNowReady ? "<span style='color:#3fb950'>&#9679; Aktiv</span>"
                      : "<span style='color:#f85149'>&#9679; Inaktiv</span>";
    h5 += "</td></tr><tr><td>WiFi-Kanal</td><td id='s-ch2'>";
    h5 += String(chD);
    h5 += " <small style='color:#e3b341'>(alle ESPs = gleicher Kanal)</small>"
          "</td></tr><tr><td>BSSID-Lock</td><td>";
    h5 += bssidSection;
    h5 += "</td></tr><tr><td>Peers</td><td>";
    h5 += String(peerCount);
    h5 += " / ";
    h5 += String(MAX_PEERS);
    h5 += "</td></tr></table>"
          "<div style='margin-top:8px;display:flex;gap:8px;flex-wrap:wrap'>"
          "<button class='btn-sm' onclick='manualSync()'>&#128259; Kanal-Sync</button>"
          "</div></div></div>\n";
    webServer.sendContent(h5);

    // ── Chunk 6: Log-Pane ────────────────────────────────────────
    webServer.sendContent(
        "<div id='pane-log' class='pane'>"
        "<div class='card'>"
        "<div class='chat-header'><h3 style='margin:0'>&#128221; Event-Log</h3>"
        "<button class='btn-sm' onclick='loadLog()'>&#8635; Aktualisieren</button></div><br>"
        "<div id='log-list'><div class='empty'>&#8987; Lade...</div></div>"
        "</div></div>\n"
    );

    // ── Chunk 7: Toast + Script ──────────────────────────────────
    webServer.sendContent(R"rawjs(<div id="toast"></div>
<script>
// ── Tab-Switching ──────────────────────────────────────────────
function showTab(id,btn){
  document.querySelectorAll('.pane').forEach(p=>p.classList.remove('active'));
  document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));
  document.getElementById('pane-'+id).classList.add('active');
  if(btn)btn.classList.add('active');
}

// ── Toast ──────────────────────────────────────────────────────
function toast(msg,ok){
  var t=document.getElementById('toast');
  t.textContent=msg;
  t.style.background=ok?'#238636':'#b91c1c';
  t.classList.add('show');
  setTimeout(()=>t.classList.remove('show'),2500);
}

// ── Emoji ──────────────────────────────────────────────────────
function addEmoji(e){
  var t=document.getElementById('msg-text');
  t.value+=e; t.focus();
}

// ── Senden ────────────────────────────────────────────────────
function sendMsg(){
  var to=document.getElementById('sel-to').value;
  var txt=document.getElementById('msg-text').value.trim();
  if(!txt){toast('Bitte Text eingeben',false);return;}
  fetch('/api/send',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({to,text:txt})})
  .then(r=>r.json()).then(d=>{
    if(d.ok){toast('\u2714 Gesendet!',true);document.getElementById('msg-text').value='';}
    else toast('Fehler: '+d.error,false);
  }).catch(()=>toast('Verbindungsfehler',false));
}

// ── Gelesen / Chat leeren ──────────────────────────────────────
function markRead(){
  fetch('/api/mark-read',{method:'POST'}).then(()=>{
    document.getElementById('ucnt').textContent='0';
    var ub=document.getElementById('ubadge');if(ub)ub.remove();
  });
}
function clearChat(){
  if(!confirm('Chat-Verlauf wirklich l\u00f6schen?'))return;
  fetch('/api/clear-chat',{method:'POST'}).then(()=>{
    document.getElementById('msg-list').innerHTML='<div class="empty">Chat geleert</div>';
    document.getElementById('ucnt').textContent='0';
    var ub=document.getElementById('ubadge');if(ub)ub.remove();
  });
}

// ── Peer-Liste ────────────────────────────────────────────────
function loadPeers(){
  fetch('/api/peers').then(r=>r.json()).then(peers=>{
    var el=document.getElementById('peer-list');
    var pc=document.getElementById('peer-cnt');
    if(pc)pc.textContent=peers.length;
    if(!peers.length){el.innerHTML='<div class="empty">Noch keine Ger\u00e4te</div>';return;}
    el.innerHTML=peers.map(p=>
      `<div class="peer-card" onclick="selectPeer('${p.mac}','${p.name.replace(/'/g,'')}')">
        <div class="peer-dot" style="background:${p.online?'#3fb950':'#555'}"></div>
        <div style="flex:1"><div class="peer-name">${p.name}</div>
        <div class="peer-mac">${p.mac}</div></div>
        <span class="badge ${p.online?'bc-green':'bc-red'}">${p.online?'online':'offline'}</span>
      </div>`
    ).join('');
    // Dropdown aktualisieren
    var sel=document.getElementById('sel-to');
    var cur=sel.value;
    sel.innerHTML='<option value="bc">\uD83D\uDCE2 An alle (Broadcast)</option>'
      +peers.map(p=>`<option value="${p.mac}">${p.name} (${p.mac.substring(12)})</option>`).join('');
    if([...sel.options].some(o=>o.value===cur))sel.value=cur;
  });
}
function selectPeer(mac,name){
  document.getElementById('sel-to').value=mac;
  showTab('chat',document.querySelector('.tab'));
  document.getElementById('msg-text').focus();
  toast('\u2192 Empf\u00e4nger: '+name,true);
}

// ── Chat rendern ──────────────────────────────────────────────
function renderChat(msgs){
  var el=document.getElementById('msg-list');
  if(!msgs||!msgs.length){
    el.innerHTML='<div class="empty">Noch keine Nachrichten<br><small>Warte auf andere ESPs...</small></div>';
    return;
  }
  el.innerHTML=msgs.map(m=>{
    var age=m.age||0;
    var as=age<60?age+'s':age<3600?Math.floor(age/60)+'min':Math.floor(age/3600)+'h';
    var isSent=m.sent===true;
    var cls='msg'+(isSent?' sent':(m.read?'':' unread'));
    var badge=isSent?'<span class="badge bc-green">gesendet</span>'
      :(m.bc?'<span class="badge bc-purple">broadcast</span>':'<span class="badge bc-blue">direkt</span>');
    var name=isSent?'<span class="msg-from me">Ich</span>':`<span class="msg-from">${m.from}</span>`;
    return `<div class="${cls}">
      <div class="msg-head">${name}
        <div class="msg-meta">${badge}<span>${as}</span></div></div>
      <div class="msg-text">${m.text}</div></div>`;
  }).join('');
}

// ── canvas-gauges ─────────────────────────────────────────────
var rssiGauge, heapGauge;
function initGauges(){
  rssiGauge = new RadialGauge({
    renderTo:'g-rssi', width:160, height:160,
    minValue:-100, maxValue:0, majorTicks:['-100','-80','-60','-40','-20','0'],
    minorTicks:4, strokeTicks:true,
    highlights:[
      {from:-100,to:-80,color:'rgba(248,81,73,.4)'},
      {from:-80, to:-60,color:'rgba(227,179,65,.4)'},
      {from:-60, to:0,  color:'rgba(63,185,80,.4)'}
    ],
    colorPlate:'#161b22', colorNeedle:'#58a6ff', colorNeedleEnd:'#58a6ff',
    colorMajorTicks:'#8b949e', colorMinorTicks:'#444',
    colorNumbers:'#e6edf3', colorTitle:'#8b949e',
    colorUnits:'#8b949e', colorValueText:'#e6edf3',
    colorValueBoxBackground:'#0d1117',
    borders:false, needleType:'arrow', needleWidth:2,
    animationDuration:500, animationRule:'linear',
    valueDecimals:0, units:'dBm',
    value:)rawjs");

    webServer.sendContent(String(WiFi.RSSI()));

    webServer.sendContent(R"rawjs(
  }).draw();

  heapGauge = new LinearGauge({
    renderTo:'g-heap', width:120, height:200,
    minValue:0, maxValue:320,
    majorTicks:['0','64','128','192','256','320'],
    minorTicks:4, strokeTicks:true,
    highlights:[
      {from:0,  to:64, color:'rgba(248,81,73,.4)'},
      {from:64, to:128,color:'rgba(227,179,65,.4)'},
      {from:128,to:320,color:'rgba(63,185,80,.4)'}
    ],
    colorPlate:'#161b22', colorBar:'#21262d', colorBarProgress:'#58a6ff',
    colorMajorTicks:'#8b949e', colorMinorTicks:'#444',
    colorNumbers:'#e6edf3', colorValueText:'#e6edf3',
    colorValueBoxBackground:'#0d1117',
    borders:false, barStroke:0, barWidth:8,
    needleType:null, animationDuration:500, animationRule:'linear',
    valueDecimals:0, units:'KB',
    value:)rawjs");

    webServer.sendContent(String(ESP.getFreeHeap() / 1024));

    webServer.sendContent(R"rawjs(
  }).draw();
}
// Gauges erst initialisieren wenn Status-Tab geöffnet wird
document.querySelectorAll('.tab')[2].addEventListener('click',function(){
  if(!rssiGauge)setTimeout(initGauges,100);
});

// ── Call-Logik ────────────────────────────────────────────────
var callMac=null, callTimer=null, callSeconds=0;

function startCall(mac,name){
  if(callMac){toast('Bereits im Gespräch',false);return;}
  callMac=mac;
  document.getElementById('active-call-name').textContent='Rufe an: '+name+'...';
  document.getElementById('active-call-duration').textContent='Klingelt...';
  document.getElementById('active-call-modal').classList.add('show');
  fetch('/api/call',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({to:mac,action:'ring'})})
  .catch(()=>toast('Fehler',false));
}

function acceptCall(){
  document.getElementById('call-modal').classList.remove('show');
  var mac=document.getElementById('call-modal').dataset.mac;
  var name=document.getElementById('call-from').textContent;
  callMac=mac; callSeconds=0;
  document.getElementById('active-call-name').textContent='Verbunden: '+name;
  startCallTimer();
  document.getElementById('active-call-modal').classList.add('show');
  fetch('/api/call',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({to:mac,action:'accept'})})
  .catch(()=>{});
}

function rejectCall(){
  var mac=document.getElementById('call-modal').dataset.mac;
  document.getElementById('call-modal').classList.remove('show');
  fetch('/api/call',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({to:mac,action:'reject'})})
  .catch(()=>{});
  callMac=null;
}

function endCall(){
  document.getElementById('active-call-modal').classList.remove('show');
  if(callTimer){clearInterval(callTimer);callTimer=null;}
  if(callMac){
    fetch('/api/call',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({to:callMac,action:'end'})})
    .catch(()=>{});
  }
  callMac=null; callSeconds=0;
}

function startCallTimer(){
  callSeconds=0;
  if(callTimer)clearInterval(callTimer);
  callTimer=setInterval(()=>{
    callSeconds++;
    var m=Math.floor(callSeconds/60), s=callSeconds%60;
    document.getElementById('active-call-duration').textContent=
      (m<10?'0':'')+m+':'+(s<10?'0':'')+s;
  },1000);
}

// SSE Call-Events verarbeiten
function handleCallEvent(data){
  var d=JSON.parse(data);
  if(d.type==='ring'){
    // Eingehender Anruf
    var modal=document.getElementById('call-modal');
    modal.dataset.mac=d.mac;
    document.getElementById('call-title').textContent='Eingehender Anruf';
    document.getElementById('call-from').textContent=d.from;
    document.getElementById('call-avatar').textContent='\u{1F4DE}';
    modal.classList.add('show');
  } else if(d.type==='accept'){
    // Anruf angenommen (outgoing)
    document.getElementById('active-call-name').textContent='Verbunden mit '+d.from;
    startCallTimer();
  } else if(d.type==='end'||d.type==='timeout'){
    document.getElementById('call-modal').classList.remove('show');
    document.getElementById('active-call-modal').classList.remove('show');
    if(callTimer){clearInterval(callTimer);callTimer=null;}
    callMac=null;
    toast(d.type==='timeout'?'Kein Annehmen':'Anruf beendet',false);
  }
}

// ── Name ändern ────────────────────────────────────────────────
function setMyName(){
  var inp=document.getElementById('my-name-input');
  var name=inp.value.trim();
  if(!name||name.length>31){toast('Name 1-31 Zeichen',false);return;}
  fetch('/api/set-name',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({name})})
  .then(r=>r.json()).then(d=>{
    if(d.ok){toast('\u2714 Name gespeichert: '+d.name,true);}
    else toast('Fehler: '+d.error,false);
  }).catch(()=>toast('Fehler',false));
}

// ── BSSID + Kanal-Sync ────────────────────────────────────────
function lockBssid(){
  fetch('/api/bssid-lock',{method:'POST'}).then(r=>r.json()).then(d=>{
    if(d.ok){toast('\u2714 AP gesperrt: '+d.bssid,true);setTimeout(()=>location.reload(),1500);}
    else toast('Fehler',false);
  }).catch(()=>toast('Fehler',false));
}
function clearBssid(){
  fetch('/api/bssid-clear',{method:'POST'}).then(()=>{
    toast('Lock aufgehoben',true);setTimeout(()=>location.reload(),1500);
  }).catch(()=>toast('Fehler',false));
}
function manualSync(){
  if(!confirm('Kanal-Sync starten? Dauert ca. 30s.'))return;
  toast('Suche\u2026',true);
  fetch('/api/channel-sync',{method:'POST'}).then(r=>r.json()).then(d=>{
    if(d.peers>0){toast('\u2714 Peers auf Kanal '+d.channel+'!',true);setTimeout(()=>location.reload(),2000);}
    else toast('Kein passender AP gefunden',false);
  }).catch(()=>toast('Fehler',false));
}

// ── SSE: Server-Sent Events ───────────────────────────────────
var sse;
function connectSse(){
  sse=new EventSource('/api/sse');
  sse.addEventListener('messages',e=>{
    try{renderChat(JSON.parse(e.data));}catch(ex){}
    fetch('/api/status').then(r=>r.json()).then(updateStatus);
  });
  sse.addEventListener('peers',e=>{
    try{
      var peers=JSON.parse(e.data);
      renderPeerList(peers);
      updateDropdown(peers);
    }catch(ex){}
  });
  sse.addEventListener('status',e=>{
    try{updateStatus(JSON.parse(e.data));}catch(ex){}
  });
  sse.addEventListener('log',e=>{
    try{renderLogList(JSON.parse(e.data));}catch(ex){}
  });
  sse.addEventListener('call',e=>{
    try{handleCallEvent(e.data);}catch(ex){console.error('Call SSE:',ex);}
  });
  sse.onerror=()=>{
    setTimeout(connectSse,3000); // Reconnect
  };
}

function updateStatus(d){
  var upd=(id,v)=>{var el=document.getElementById(id);if(el)el.textContent=v;};
  upd('s-ch',d.channel||'?');
  upd('s-ch2',d.channel||'?');
  upd('s-peers',d.peers||0);
  upd('s-recv',d.recv||0);
  upd('s-sent',d.sent||0);
  upd('s-unread',d.unread||0);
  upd('s-rssi',(d.rssi||0)+' dBm');
  upd('ucnt',d.unread||0);
  var uptime=d.uptime||0;
  var ut=uptime<60?uptime+'s':uptime<3600?Math.floor(uptime/60)+'min '+uptime%60+'s':Math.floor(uptime/3600)+'h '+Math.floor((uptime%3600)/60)+'min';
  upd('s-uptime',ut);
  // Gauges aktualisieren
  if(rssiGauge)rssiGauge.value=d.rssi||0;
  if(heapGauge)heapGauge.value=Math.round((d.freeHeap||0)/1024);
  // Unread-Badge
  var ub=document.getElementById('ubadge');
  if(d.unread>0){
    if(!ub){var hdr=document.querySelector('header');var s=document.createElement('span');
      s.id='ubadge';s.className='badge bc-yellow';hdr.appendChild(s);}
    document.getElementById('ubadge').textContent='\u2709 '+d.unread+' neu';
  } else if(ub) ub.remove();
}

function renderPeerList(peers){
  var el=document.getElementById('peer-list');
  var pc=document.getElementById('peer-cnt');if(pc)pc.textContent=peers.length;
  if(!peers.length){el.innerHTML='<div class="empty">Noch keine Ger\u00e4te</div>';return;}
  el.innerHTML=peers.map(p=>
    `<div class="peer-card" onclick="selectPeer('${p.mac}','${p.name.replace(/'/g,'')}')">
      <div class="peer-dot" style="background:${p.online?'#3fb950':'#555'}"></div>
      <div style="flex:1"><div class="peer-name">${p.name}</div>
      <div class="peer-mac">${p.mac}</div></div>
      <span class="badge ${p.online?'bc-green':'bc-red'}">${p.online?'online':'offline'}</span>
    </div>`
  ).join('');
}

function updateDropdown(peers){
  var sel=document.getElementById('sel-to');
  var cur=sel.value;
  sel.innerHTML='<option value="bc">\uD83D\uDCE2 An alle (Broadcast)</option>'
    +peers.map(p=>`<option value="${p.mac}">${p.name} (${p.mac.substring(12)})</option>`).join('');
  if([...sel.options].some(o=>o.value===cur))sel.value=cur;
}

function renderLogList(evts){
  var el=document.getElementById('log-list');
  if(!evts||!evts.length){el.innerHTML='<div class="empty">Keine Ereignisse</div>';return;}
  el.innerHTML=evts.map(e=>{
    var age=e.age||0;
    var as=age<60?age+'s':age<3600?Math.floor(age/60)+'min':Math.floor(age/3600)+'h';
    return `<div class="log-entry"><span class="log-msg">${e.msg}</span><span class="log-age">${as}</span></div>`;
  }).join('');
}

function loadPeers(){
  fetch('/api/peers').then(r=>r.json()).then(p=>{renderPeerList(p);updateDropdown(p);});
}
function loadLog(){
  fetch('/api/events').then(r=>r.json()).then(renderLogList);
}

// ── Start ─────────────────────────────────────────────────────
connectSse();
// Initial load via REST als Fallback
fetch('/api/messages').then(r=>r.json()).then(renderChat);
loadPeers();
loadLog();
// Polling als Fallback wenn SSE nicht verbunden
setInterval(()=>{
  if(sse&&sse.readyState===1)return; // SSE aktiv - kein Polling
  fetch('/api/messages').then(r=>r.json()).then(renderChat);
  fetch('/api/status').then(r=>r.json()).then(updateStatus);
},5000);
// Status-Polling immer (für Uptime-Anzeige)
setInterval(()=>{
  fetch('/api/status').then(r=>r.json()).then(updateStatus);
},10000);
</script>
</body>
</html>
)rawjs");

    webServer.sendContent("");
}


void handleOtaPage() {
    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    webServer.send(200, "text/html; charset=utf-8", "");
    String h = R"rawhtml(<!DOCTYPE html>
<html lang="de"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OTA Update</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d1117;color:#e6edf3;font-family:system-ui,sans-serif;font-size:14px}
header{background:#161b22;border-bottom:1px solid #30363d;padding:12px 20px;display:flex;align-items:center;gap:10px}
h1{font-size:17px;color:#58a6ff}
.tab{padding:10px 18px;color:#8b949e;font-weight:500;text-decoration:none;
  border-bottom:2px solid transparent;display:inline-block}
.tab.active{color:#58a6ff;border-bottom-color:#58a6ff}
.tabs{background:#161b22;border-bottom:1px solid #30363d}
.content{padding:20px;max-width:600px;margin:0 auto}
.card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:16px}
.card h3{font-size:11px;color:#8b949e;text-transform:uppercase;margin-bottom:14px}
.drop{border:2px dashed #30363d;border-radius:8px;padding:28px;text-align:center;
  cursor:pointer;color:#8b949e}
.drop:hover,.drop.over{border-color:#58a6ff;color:#e6edf3}
input[type=file]{display:none}
.btn{background:#238636;color:#fff;border:none;padding:10px;border-radius:6px;
  cursor:pointer;font-size:14px;font-weight:600;margin-top:12px;width:100%}
.btn:hover{background:#2ea043}
.prog{display:none;margin-top:12px}
.bar{height:8px;background:#21262d;border-radius:4px;overflow:hidden}
.fill{height:100%;background:#58a6ff;border-radius:4px;width:0;transition:width .3s}
.msg{margin-top:8px;font-size:13px;color:#8b949e;text-align:center}
.warn{background:rgba(227,179,65,.1);border:1px solid rgba(227,179,65,.3);
  border-radius:6px;padding:10px;color:#e3b341;font-size:13px;margin-bottom:12px}
</style></head><body>
<header><h1>&#128172; )rawhtml";
    h += escHtml(deviceName);
    h += R"rawhtml(</h1></header>
<div class="tabs">
  <a class="tab" href="/">&#128172; Chat</a>
  <a class="tab active" href="/ota">&#128640; OTA</a>
</div>
<div class="content"><div class="card">
<h3>&#128640; Firmware hochladen</h3>
<div class="warn">&#9888; Nach dem Flashen startet der ESP automatisch neu.</div>
<div class="drop" id="drop" onclick="document.getElementById('fw').click()">
  &#128190; <b>.bin</b> Datei hierher ziehen oder klicken
</div>
<input type="file" id="fw" accept=".bin">
<div id="fn" style="margin-top:6px;font-size:12px;color:#8b949e;text-align:center"></div>
<button class="btn" onclick="doUpload()">&#9889; Flashen</button>
<div class="prog" id="prog">
  <div class="bar"><div class="fill" id="fill"></div></div>
  <div class="msg" id="msg">Lade hoch...</div>
</div>
</div></div>
<script>
var inp=document.getElementById('fw'),drop=document.getElementById('drop');
inp.addEventListener('change',function(){
  if(inp.files[0])document.getElementById('fn').textContent=inp.files[0].name+' ('+Math.round(inp.files[0].size/1024)+' KB)';
});
drop.addEventListener('dragover',e=>{e.preventDefault();drop.classList.add('over');});
drop.addEventListener('dragleave',()=>drop.classList.remove('over'));
drop.addEventListener('drop',e=>{
  e.preventDefault();drop.classList.remove('over');
  var f=e.dataTransfer.files[0];
  if(f&&f.name.endsWith('.bin')){
    var dt=new DataTransfer();dt.items.add(f);inp.files=dt.files;
    document.getElementById('fn').textContent=f.name+' ('+Math.round(f.size/1024)+' KB)';
  }
});
function doUpload(){
  if(!inp.files[0]){alert('Bitte .bin Datei wählen!');return;}
  var fd=new FormData();fd.append('firmware',inp.files[0]);
  var xhr=new XMLHttpRequest();xhr.open('POST','/ota-upload');
  document.getElementById('prog').style.display='block';
  xhr.upload.onprogress=e=>{
    if(e.lengthComputable){
      var p=Math.round(e.loaded/e.total*100);
      document.getElementById('fill').style.width=p+'%';
      document.getElementById('msg').textContent='Hochladen: '+p+'%';
    }
  };
  xhr.onload=()=>{
    var ok=xhr.status===200;
    document.getElementById('fill').style.background=ok?'#3fb950':'#f85149';
    document.getElementById('fill').style.width='100%';
    document.getElementById('msg').textContent=ok?'Erfolgreich! ESP startet neu...':'Fehler: '+xhr.responseText;
  };
  xhr.onerror=()=>document.getElementById('msg').textContent='Verbindungsfehler';
  xhr.send(fd);
}
</script>
</body></html>
)rawhtml";
    webServer.sendContent(h);
    webServer.sendContent("");
}

void handleOtaUpload() {
    HTTPUpload& upload = webServer.upload();
    if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("[OTA] Start: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
            Update.printError(Serial);
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) Serial.printf("[OTA] OK! %u Bytes\n", upload.totalSize);
        else Update.printError(Serial);
    }
}

void handleOtaUploadFinish() {
    if (Update.hasError())
        webServer.send(500, "text/plain", "OTA fehlgeschlagen!");
    else
        webServer.send(200, "text/plain", "OK");
    delay(500);
    ESP.restart();
}

// ================================================================
//  API-Handler
// ================================================================

void handleApiSend() {
    if (!webServer.hasArg("plain")) {
        webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"Kein Body\"}");
        return;
    }
    String body = webServer.arg("plain");

#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
#else
    DynamicJsonDocument doc(512);
#endif

    if (deserializeJson(doc, body) != DeserializationError::Ok) {
        webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"JSON ungueltig\"}");
        return;
    }

    String toStr = doc["to"].as<String>();
    String text  = doc["text"].as<String>();

    if (text.length() == 0) {
        webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"Text leer\"}");
        return;
    }
    if (text.length() > 199) text = text.substring(0, 199);

    uint8_t toMac[6];
    if (toStr == "bc" || toStr == "FF:FF:FF:FF:FF:FF") {
        memset(toMac, 0xFF, 6);
    } else {
        if (!parseMacStr(toStr, toMac)) {
            webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"MAC ungueltig\"}");
            return;
        }
    }

    bool ok = espNowSend(toMac, text);
    if (ok) {
        webServer.send(200, "application/json", "{\"ok\":true}");
    } else {
        webServer.send(500, "application/json", "{\"ok\":false,\"error\":\"ESP-NOW Fehler\"}");
    }
}

void handleApiMessages() {
    webServer.send(200, "application/json", inboxToJson());
}

void handleApiMarkRead() {
    inboxMarkAllRead();
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiClearChat() {
    inboxCount = 0; inboxNext = 0; unreadCount = 0;
    memset(inbox, 0, sizeof(inbox));
    logEvent("Chat-Verlauf geleert");
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiPeers() {
    webServer.send(200, "application/json", peersToJson());
}

void handleApiChannelSync() {
    if (channelSyncRunning) {
        webServer.send(429, "application/json", "{\"ok\":false,\"error\":\"laeuft bereits\"}");
        return;
    }
    channelSyncRunning = true;
    lastPeerFound = 0;  // Reset damit Sync wirklich läuft
    doChannelSync();
    channelSyncRunning = false;
    lastPeerFound = (peerCount > 0) ? millis()/1000UL : 1;
    uint8_t ch; wifi_second_chan_t sc;
    esp_wifi_get_channel(&ch, &sc);
    String resp = "{\"ok\":true,\"peers\":" + String(peerCount) +
                  ",\"channel\":" + String(ch) + "}";
    webServer.send(200, "application/json", resp);
}

void handleApiBssidLock() {
    uint8_t* b = WiFi.BSSID();
    if (!b) { webServer.send(400, "application/json", "{\"ok\":false}"); return; }
    char s[13];
    snprintf(s, sizeof(s), "%02X%02X%02X%02X%02X%02X", b[0],b[1],b[2],b[3],b[4],b[5]);
    lockedBssid = String(s);
    prefs.begin("espcomm", false);
    prefs.putString("bssid", lockedBssid);
    prefs.end();
    char msg[80];
    snprintf(msg, sizeof(msg), "BSSID-Lock gesetzt: %s (Kanal %d)", s, WiFi.channel());
    logEvent(msg);
    webServer.send(200, "application/json", "{\"ok\":true,\"bssid\":\"" + lockedBssid + "\"}");
}

void handleApiBssidClear() {
    lockedBssid = "";
    prefs.begin("espcomm", false);
    prefs.putString("bssid", "");
    prefs.end();
    logEvent("BSSID-Lock aufgehoben — verbindet sich beim naechsten Start an besten AP");
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiEvents() {
    webServer.send(200, "application/json", eventLogToJson());
}

void handleApiStatus() {
    webServer.send(200, "application/json", handleApiStatusJson());
}

// POST /api/call  {to: "MAC", action: "ring"|"accept"|"reject"|"end"}
void handleApiCall() {
    if (!webServer.hasArg("plain")) {
        webServer.send(400, "application/json", "{"ok":false}"); return;
    }
#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
#else
    DynamicJsonDocument doc(256);
#endif
    if (deserializeJson(doc, webServer.arg("plain")) != DeserializationError::Ok) {
        webServer.send(400, "application/json", "{"ok":false}"); return;
    }
    String action = doc["action"].as<String>();
    String toStr  = doc["to"].as<String>();

    uint8_t toMac[6];
    if (!parseMacStr(toStr, toMac)) {
        webServer.send(400, "application/json", "{"ok":false,"error":"MAC ungueltig"}");
        return;
    }

    uint8_t callType = 0;
    if (action == "ring")   { callType = MSG_TYPE_CALL;
        if (!callState.active) {
            callState.active = true; callState.incoming = false;
            callState.ringStart = millis();
            memcpy(callState.peerMac, toMac, 6);
            // Name aus Peer-Liste
            for (int i = 0; i < peerCount; i++) {
                if (memcmp(knownPeers[i].mac, toMac, 6) == 0) {
                    strncpy(callState.peerName, knownPeers[i].name, 31); break;
                }
            }
        }
    }
    else if (action == "accept") { callType = MSG_TYPE_CALL_A;
        callState.startTime = millis();
    }
    else if (action == "reject") { callType = MSG_TYPE_CALL_R;
        memset(&callState, 0, sizeof(callState));
    }
    else if (action == "end")    { callType = MSG_TYPE_CALL_E;
        memset(&callState, 0, sizeof(callState));
    }
    else {
        webServer.send(400, "application/json", "{"ok":false,"error":"Unbekannte Aktion"}");
        return;
    }
    espNowCall(toMac, callType);
    char evtbuf[80];
    snprintf(evtbuf, sizeof(evtbuf), "Anruf %s an %s", action.c_str(), callState.peerName);
    logEvent(evtbuf);
    webServer.send(200, "application/json", "{"ok":true}");
}

// POST /api/set-name  {name: "Neuer Name"}
void handleApiSetName() {
    if (!webServer.hasArg("plain")) {
        webServer.send(400, "application/json", "{"ok":false}"); return;
    }
#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
#else
    DynamicJsonDocument doc(128);
#endif
    if (deserializeJson(doc, webServer.arg("plain")) != DeserializationError::Ok) {
        webServer.send(400, "application/json", "{"ok":false}"); return;
    }
    String newName = doc["name"].as<String>();
    newName.trim();
    if (newName.length() == 0 || newName.length() > 31) {
        webServer.send(400, "application/json", "{"ok":false,"error":"Name 1-31 Zeichen"}");
        return;
    }
    deviceName = newName;
    prefs.begin("espcomm", false);
    prefs.putString("name", newName);
    prefs.end();
    char evtbuf[60];
    snprintf(evtbuf, sizeof(evtbuf), "Name geaendert zu: %s", newName.c_str());
    logEvent(evtbuf);
    // mDNS neu starten mit neuem Namen
    String mdns = "espcomm-" + toMdnsName(deviceName);
    MDNS.begin(mdns.c_str());
    ssePush("status", handleApiStatusJson());
    webServer.send(200, "application/json", "{"ok":true,"name":"" + escJson(newName) + ""}");
}

void handleNotFound() {
    webServer.sendHeader("Location", "/", true);
    webServer.send(302, "text/plain", "");
}

// ================================================================
//  Web-Server Setup
// ================================================================

void setupWebServer() {
    webServer.on("/",              HTTP_GET,  handleRoot);
    webServer.on("/ota",           HTTP_GET,  handleOtaPage);
    webServer.on("/ota-upload",    HTTP_POST, handleOtaUploadFinish, handleOtaUpload);
    webServer.on("/api/send",      HTTP_POST, handleApiSend);
    webServer.on("/api/messages",  HTTP_GET,  handleApiMessages);
    webServer.on("/api/mark-read",  HTTP_POST, handleApiMarkRead);
    webServer.on("/api/clear-chat", HTTP_POST, handleApiClearChat);
    webServer.on("/api/peers",     HTTP_GET,  handleApiPeers);
    webServer.on("/api/status",    HTTP_GET,  handleApiStatus);
    webServer.on("/api/events",    HTTP_GET,  handleApiEvents);
    webServer.on("/api/sse",       HTTP_GET,  handleSse);
    webServer.on("/api/channel-sync", HTTP_POST, handleApiChannelSync);
    webServer.on("/api/bssid-lock",  HTTP_POST, handleApiBssidLock);
    webServer.on("/api/bssid-clear", HTTP_POST, handleApiBssidClear);
    webServer.on("/api/call",         HTTP_POST, handleApiCall);
    webServer.on("/api/set-name",     HTTP_POST, handleApiSetName);
    webServer.onNotFound(handleNotFound);
    webServer.begin();
    Serial.println("[WEB] http://" + getLocalIp() + "/");
}

// ================================================================
//  Hub-Heartbeat (aus esp-hub-base)
// ================================================================

String buildHeartbeat() {
#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
#else
    DynamicJsonDocument doc(512);
#endif
    doc["mac"]        = getMac();
    doc["name"]       = deviceName;
    doc["hwType"]     = "esp32";
    doc["chipModel"]  = ESP.getChipModel();
    doc["version"]    = FW_VERSION;
    doc["ip"]         = getLocalIp();
    doc["rssi"]       = WiFi.RSSI();
    doc["uptime"]     = millis() / 1000UL;
    doc["freeHeap"]   = ESP.getFreeHeap();
    doc["freeSketch"] = ESP.getFreeSketchSpace();

    // IO-Werte: Kommunikationsstatistik
    JsonObject ios   = doc["ios"].to<JsonObject>();
    JsonObject peers = ios["peers"].to<JsonObject>();
    peers["type"]  = "sensor";
    peers["value"] = peerCount;
    JsonObject sent = ios["sent"].to<JsonObject>();
    sent["type"]  = "sensor";
    sent["value"] = sentTotal;
    JsonObject recv = ios["recv"].to<JsonObject>();
    recv["type"]  = "sensor";
    recv["value"] = recvTotal;
    JsonObject unread = ios["unread"].to<JsonObject>();
    unread["type"]  = "sensor";
    unread["value"] = unreadCount;
    uint8_t chHb; wifi_second_chan_t scHb;
    esp_wifi_get_channel(&chHb, &scHb);
    JsonObject chan = ios["channel"].to<JsonObject>();
    chan["type"]  = "sensor";
    chan["value"] = chHb;

    String out; serializeJson(doc, out); return out;
}

void sendHeartbeat() {
    if (WiFi.status() != WL_CONNECTED) return;
    String payload = buildHeartbeat();
    String url = "http://" + hubHost + ":" + String(hubPort) + "/api/register";

    HTTPClient http;
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(3000);  // TWDT = 5s → max 3s blockieren
    esp_task_wdt_reset();   // TWDT zurücksetzen vor HTTP-Call
    int code = http.POST(payload);

    if (code == 200) {
        String body = http.getString();
        lastSuccess = millis();

#if ARDUINOJSON_VERSION_MAJOR >= 7
        JsonDocument resp;
#else
        DynamicJsonDocument resp(512);
#endif
        if (deserializeJson(resp, body) == DeserializationError::Ok) {
            if (resp.containsKey("interval")) {
                unsigned long ni = (unsigned long)(int)resp["interval"] * 1000UL;
                if (ni != heartbeatInterval && ni >= 5000UL)
                    heartbeatInterval = ni;
            }
            if (resp.containsKey("otaUrl") && !resp["otaUrl"].isNull()) {
                String u = resp["otaUrl"].as<String>();
                if (u.length() > 0) { otaPending = true; otaUrl = u; }
            }
            // Name vom Hub übernehmen (wenn im Dashboard umbenannt)
            if (resp.containsKey("name") && !resp["name"].isNull()) {
                String newName = resp["name"].as<String>();
                if (newName.length() > 0 && newName != deviceName) {
                    Serial.println("[HB] Name geändert: " + newName);
                    deviceName = newName;
                    prefs.begin("espcomm", false);
                    prefs.putString("name", newName);
                    prefs.end();
                    char evtbuf3[80];
                    snprintf(evtbuf3, sizeof(evtbuf3), "Name geändert zu: %s", newName.c_str());
                    logEvent(evtbuf3);
                }
            }
        }
        Serial.println("[HB] OK");
    } else {
        Serial.printf("[HB] HTTP %d\n", code);
        if (code < 0) hubOnCurrentAp = false;  // Verbindungsfehler = Hub nicht erreichbar
    }
    http.end();
}

// ================================================================
//  HTTP OTA (vom Hub)
// ================================================================

void performOta(const String& url) {
    Serial.println("[OTA] " + url);
    HTTPClient http;
    http.begin(url);
    http.setTimeout(30000);
    esp_task_wdt_reset();  // OTA darf länger dauern
    int code = http.GET();
    if (code != 200) { http.end(); return; }
    int total = http.getSize();
    if (!Update.begin(total > 0 ? total : UPDATE_SIZE_UNKNOWN))
        { Update.printError(Serial); http.end(); return; }
    WiFiClient* s = http.getStreamPtr();
    uint8_t buf[512]; size_t w = 0;
    while (http.connected() && (total <= 0 || w < (size_t)total)) {
        size_t a = s->available();
        if (!a) { delay(1); continue; }
        size_t r = s->readBytes(buf, min(a, sizeof(buf)));
        if (!r) break;
        Update.write(buf, r); w += r;
    }
    if (Update.end(true)) { http.end(); delay(500); ESP.restart(); }
    else Update.printError(Serial);
    http.end();
}

// ================================================================
//  Reset-Taste
// ================================================================

void checkResetButton() {
    pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
    if (digitalRead(RESET_BUTTON_PIN) == HIGH) return;
    Serial.println("[RESET] Halte " + String(RESET_HOLD_SEC) + "s...");
    unsigned long start = millis();
    while (digitalRead(RESET_BUTTON_PIN) == LOW) {
        if (millis() - start > (unsigned long)RESET_HOLD_SEC * 1000UL) {
            Serial.println("[RESET] WLAN gelöscht!");
            wifiManager.resetSettings();
            prefs.begin("espcomm", false);
            prefs.clear();
            prefs.end();
            delay(500);
            ESP.restart();
        }
        delay(100);
    }
}

// ================================================================
//  WiFiManager Setup
// ================================================================

void setupWifi() {
    prefs.begin("espcomm", false);
    String sName  = prefs.getString("name",     deviceName);
    String sHost  = prefs.getString("hub_host", hubHost);
    int    sPort  = prefs.getInt   ("hub_port", hubPort);
    lockedBssid   = prefs.getString("bssid",    "");
    prefs.end();
    deviceName = sName; hubHost = sHost; hubPort = sPort;

    // BSSID-Lock: direkt ohne Portal verbinden wenn bekannte BSSID gespeichert
    if (lockedBssid.length() == 12) {
        uint8_t bssid[6];
        for (int i = 0; i < 6; i++) {
            String byteStr = lockedBssid.substring(i*2, i*2+2);
            bssid[i] = (uint8_t)strtol(byteStr.c_str(), nullptr, 16);
        }
        Serial.print("[WiFi] BSSID-Lock: ");
        Serial.println(lockedBssid);
        // Verbinde mit gespeichertem SSID+Passwort, aber erzwinge BSSID
        WiFi.mode(WIFI_STA);
        WiFi.begin(WiFi.SSID().c_str(), WiFi.psk().c_str(), 0, bssid);
        unsigned long t = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) delay(200);
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("[WiFi] BSSID-Lock verbunden! Kanal: " + String(WiFi.channel()));
            // Weiter mit normaler Konfiguration (kein Portal nötig)
            deviceName = sName; hubHost = sHost; hubPort = sPort;
            WiFi.setSleep(false);
            esp_wifi_set_ps(WIFI_PS_NONE);
            Serial.println("[WiFi] IP: " + WiFi.localIP().toString());
            return;
        }
        // Fehlgeschlagen — BSSID löschen und normal verbinden
        Serial.println("[WiFi] BSSID-Lock fehlgeschlagen — normaler Connect");
        prefs.begin("espcomm", false);
        prefs.putString("bssid", "");
        prefs.end();
        lockedBssid = "";
    }

    WiFiManagerParameter pName("name",     "Geraetename", deviceName.c_str(), 32);
    WiFiManagerParameter pHost("hub_host", "ESP-Hub IP",  hubHost.c_str(),    40);
    WiFiManagerParameter pPort("hub_port", "ESP-Hub Port",String(hubPort).c_str(), 6);

    wifiManager.addParameter(&pName);
    wifiManager.addParameter(&pHost);
    wifiManager.addParameter(&pPort);
    wifiManager.setConfigPortalTimeout(WIFI_PORTAL_TIMEOUT_S);
    wifiManager.setAPCallback([](WiFiManager*) {
        Serial.println("[WiFi] Portal: " WIFI_AP_NAME);
    });

    if (!wifiManager.autoConnect(WIFI_AP_NAME)) {
        Serial.println("[WiFi] Timeout - Neustart");
        delay(1000); ESP.restart();
    }

    prefs.begin("espcomm", false);
    prefs.putString("name",     String(pName.getValue()));
    prefs.putString("hub_host", String(pHost.getValue()));
    prefs.putInt   ("hub_port", String(pPort.getValue()).toInt());
    prefs.end();

    deviceName = String(pName.getValue());
    hubHost    = String(pHost.getValue());
    hubPort    = String(pPort.getValue()).toInt();

    // Nach erstem Connect: BSSID automatisch speichern
    uint8_t* bssidPtr = WiFi.BSSID();
    if (bssidPtr) {
        char bssidStr[13];
        snprintf(bssidStr, sizeof(bssidStr), "%02X%02X%02X%02X%02X%02X",
                 bssidPtr[0], bssidPtr[1], bssidPtr[2],
                 bssidPtr[3], bssidPtr[4], bssidPtr[5]);
        lockedBssid = String(bssidStr);
        prefs.begin("espcomm", false);
        prefs.putString("bssid", lockedBssid);
        prefs.end();
        Serial.println("[WiFi] BSSID gespeichert: " + lockedBssid + " (Kanal " + String(WiFi.channel()) + ")");
    }

    // Power Save MUSS aus sein — sonst verpasst der ESP32 ESP-NOW Broadcasts!
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    Serial.println("[WiFi] Power Save: OFF");
    Serial.println("[WiFi] IP: " + WiFi.localIP().toString());
}

// ================================================================
//  Setup & Loop
// ================================================================

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== ESP Communicator v" FW_VERSION " ===");

    ledSetup();

    checkResetButton();
    setupWifi();
    ledSetup();  // GPIO2 nach WiFiManager neu initialisieren

    // ESP-NOW NACH WiFi init (nutzt denselben Kanal)
    setupEspNow();

    // mDNS
    String mdns = "espcomm-" + toMdnsName(deviceName);
    if (MDNS.begin(mdns.c_str())) {
        MDNS.addService("http", "tcp", 80);
        Serial.println("[mDNS] http://" + mdns + ".local/");
    }

    setupWebServer();

    lastSuccess = millis();
    sendHeartbeat();
    lastHeartbeat = millis();

    // Erster Hello-Broadcast
    espNowHello();
    lastHello = millis();

    // Startblinken
    ledBlink(2);
    logEvent("ESP gestartet — Web-UI + ESP-NOW aktiv (Auto-Kanal-Sync nach 90s)");
    Serial.println("[READY] Web-UI + ESP-NOW aktiv");
}

void loop() {
    webServer.handleClient();
    ledTick();

    unsigned long now = millis();

    // Hub-Heartbeat
    if (now - lastHeartbeat >= heartbeatInterval) {
        lastHeartbeat = now;
        sendHeartbeat();
    }

    // ESP-NOW Hello-Broadcast (Peer-Discovery)
    if (now - lastHello >= HELLO_INTERVAL) {
        lastHello = now;
        espNowHello();
    }

    // RSSI-Scan broadcasten (alle 60s wenn Peers vorhanden)
    if (peerCount > 0 && now - lastScanBroadcast >= SCAN_INTERVAL) {
        lastScanBroadcast = now;
        broadcastScan();
    }

    // Consensus auswerten (alle 120s wenn genug Votes)
    static unsigned long lastConsensus = 0;
    if (voteCount >= 2 && now - lastConsensus >= 120000UL) {
        lastConsensus = now;
        evaluateConsensus();
    }

    // Hub-Check: alle 5min kurz zum Hub wenn AP gewechselt
    if (!hubOnCurrentAp && now - lastHubCheck >= HUB_CHECK_INTERVAL_MS) {
        lastHubCheck = now;
        hubReconnectCheck();
    }

    // OTA vom Hub
    if (otaPending) {
        otaPending = false;
        performOta(otaUrl);
        otaUrl = "";
    }

    // WiFi-Reconnect (non-blocking)
    if (WiFi.status() != WL_CONNECTED) {
        if (now - lastReconnectAttempt > 10000UL) {
            lastReconnectAttempt = now;
            Serial.println("[WiFi] Reconnect...");
            WiFi.reconnect();
        }
    }

    // Deferred Peer-Registrierung (aus ESP-NOW Callback gequeuet)
    processPendingPeers();

    // Call-Timeout: Klingeln nach 30s abbrechen
    if (callState.active && callState.incoming &&
        millis() - callState.ringStart > CALL_RING_TIMEOUT_MS) {
        logEvent("Anruf abgelaufen (kein Annehmen)");
        ssePush("call", "{\"type\":\"timeout\"}");
        memset(&callState, 0, sizeof(callState));
    }

    // Auto-Channel-Sync: wenn nach PEER_SEARCH_TIMEOUT_S keine Peers
    if (peerCount == 0 && !channelSyncRunning &&
        (millis() / 1000UL) > PEER_SEARCH_TIMEOUT_S &&
        lastPeerFound == 0) {
        channelSyncRunning = true;
        doChannelSync();
        channelSyncRunning = false;
        lastPeerFound = 1;  // Verhindert Dauerschleife wenn kein AP passt
    }

    delay(5);
}

// ================================================================
//  ▲▲▲  ENDE
// ================================================================
