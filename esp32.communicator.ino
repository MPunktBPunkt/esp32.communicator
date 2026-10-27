// ╔══════════════════════════════════════════════════════════════╗
// ║  esp32.communicator.ino                                     ║
// ║  Version: 1.8.0                                             ║
// ║  Basis: esp-hub-base v1.7.0                                 ║
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
#include "esphub/HardwareInfo.h"
#include "esphub/CpuLoad.h"
#include "web/UiPages.h"

// ================================================================
//  KONFIGURATION  <-- Diese Werte anpassen
// ================================================================

// Gerätename — erscheint im Dashboard und als Absender
#define DEVICE_NAME            "Communicator-1"

// Firmware-Version
#define FW_VERSION             "1.9.0"

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
    char     fromMac[18];   // Absender-MAC (bei gesendet: eigene)
    char     peerMac[18];   // Gesprächspartner, oder "bc" bei Broadcast
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
HardwareInfo hwInfo;
String       hubHost    = HUB_HOST;
int          hubPort    = HUB_PORT;

// PIN / Session (Web-UI Schutz)
String       pinHashStored = "";
#define MAX_SESSIONS 4
#define SESSION_TTL_MS (86400000UL)  // 24h
struct WebSession {
    char     token[17];
    uint32_t created;
    bool     used;
};
WebSession   webSessions[MAX_SESSIONS];

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
    r.replace("\\", "\\");
    r.replace("\"", "\"");
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
    if (isBcast) {
        strncpy(e.peerMac, "bc", 17);
    } else {
        strncpy(e.peerMac, fromMac, 17);
    }
    e.peerMac[17] = 0;
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

static bool peerMacEquals(const char* a, const char* b) {
    if (!a || !b) return false;
    if (strcmp(a, "bc") == 0 || strcmp(b, "bc") == 0)
        return strcmp(a, "bc") == 0 && strcmp(b, "bc") == 0;
    // compare ignoring separators / case
    char na[13] = {0}, nb[13] = {0};
    int ia = 0, ib = 0;
    for (int i = 0; a[i] && ia < 12; i++) {
        char c = a[i];
        if (c >= 'a' && c <= 'f') c -= 32;
        if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F')) na[ia++] = c;
    }
    for (int i = 0; b[i] && ib < 12; i++) {
        char c = b[i];
        if (c >= 'a' && c <= 'f') c -= 32;
        if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F')) nb[ib++] = c;
    }
    return ia == 12 && ib == 12 && memcmp(na, nb, 12) == 0;
}

void inboxRebuildFrom(InboxEntry* tmp, int n) {
    memset(inbox, 0, sizeof(inbox));
    inboxCount = 0;
    inboxNext = 0;
    unreadCount = 0;
    for (int i = 0; i < n; i++) {
        inbox[inboxNext % MAX_INBOX] = tmp[i];
        if (!tmp[i].isRead && !tmp[i].isSent) unreadCount++;
        inboxNext++;
        if (inboxCount < MAX_INBOX) inboxCount++;
    }
}

bool inboxDeleteById(uint32_t id) {
    if (inboxCount <= 0) return false;
    static InboxEntry tmp[MAX_INBOX];
    int n = 0;
    bool found = false;
    for (int i = 0; i < inboxCount; i++) {
        int idx = (inboxNext - inboxCount + i + MAX_INBOX) % MAX_INBOX;
        if (inbox[idx].msgId == id) { found = true; continue; }
        tmp[n++] = inbox[idx];
    }
    if (!found) return false;
    inboxRebuildFrom(tmp, n);
    return true;
}

int inboxClearPeer(const char* peer) {
    if (inboxCount <= 0) return 0;
    static InboxEntry tmp[MAX_INBOX];
    int n = 0, removed = 0;
    for (int i = 0; i < inboxCount; i++) {
        int idx = (inboxNext - inboxCount + i + MAX_INBOX) % MAX_INBOX;
        if (peerMacEquals(inbox[idx].peerMac, peer)) { removed++; continue; }
        tmp[n++] = inbox[idx];
    }
    if (removed) inboxRebuildFrom(tmp, n);
    return removed;
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
        json += "{\"id\":" + String(e.msgId) + ","
             +  "\"from\":\"" + escJson(String(e.fromName)) + "\","
             +  "\"mac\":\""  + String(e.fromMac) + "\","
             +  "\"peer\":\"" + String(e.peerMac) + "\","
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
        if (isBroadcastMac(toMac)) {
            strncpy(se.peerMac, "bc", 17);
        } else {
            char peerMs[18]; macBytesToStr(toMac, peerMs);
            strncpy(se.peerMac, peerMs, 17);
        }
        se.peerMac[17] = 0;
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
    if (!requireAuth()) return;
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
    json += "\"name\":\"" + escJson(deviceName) + "\"";
    json += ",\"mac\":\"" + getMacFmt() + "\"";
    json += ",\"macFmt\":\"" + getMacFmt() + "\"";
    json += ",\"ip\":\"" + getLocalIp() + "\"";
    json += ",\"uptime\":" + String(millis() / 1000UL);
    json += ",\"uptimeFmt\":\"" + fmtUptime(millis() / 1000UL) + "\"";
    json += ",\"heap\":" + String(hwInfo.freeHeap());
    json += ",\"freeHeap\":" + String(hwInfo.freeHeap());
    json += ",\"psram\":" + String(hwInfo.freePsram());
    json += ",\"psramSize\":" + String(hwInfo.psramSize());
    json += ",\"sketch\":" + String(hwInfo.freeSketchSpace());
    json += ",\"rssi\":" + String(WiFi.RSSI());
    json += ",\"espNow\":" + String(espNowReady ? "true" : "false");
    json += ",\"channel\":" + String(chN);
    json += ",\"peers\":" + String(peerCount);
    json += ",\"sent\":" + String(sentTotal);
    json += ",\"recv\":" + String(recvTotal);
    json += ",\"unread\":" + String(unreadCount);
    json += ",\"bssid\":\"" + lockedBssid + "\"";
    json += ",\"hubHost\":\"" + escJson(hubHost) + "\"";
    json += ",\"hubPort\":" + String(hubPort);
    json += ",\"hubUrl\":\"http://" + escJson(hubHost) + ":" + String(hubPort) + "\"";
    json += ",\"version\":\"" FW_VERSION "\"";
    json += ",\"chip\":\"" + hwInfo.chipModel() + "\"";
    json += ",\"pinEnabled\":" + String(pinEnabled() ? "true" : "false");
    hwInfo.appendStatusFields(json);
    CpuLoad::appendStatusFields(json);
    json += "}";
    return json;
}




// ================================================================
//  PIN / Session Auth
// ================================================================

String hashPin(const String& pin) {
    // FNV-1a 32-bit mit Salt (ausreichend fuer lokales Geraete-PIN)
    uint32_t h = 2166136261u;
    const char* salt = "espcomm-pin-v1";
    for (const char* p = salt; *p; ++p) { h ^= (uint8_t)(*p); h *= 16777619u; }
    for (size_t i = 0; i < pin.length(); i++) { h ^= (uint8_t)pin[i]; h *= 16777619u; }
    char buf[9];
    snprintf(buf, sizeof(buf), "%08X", (unsigned)h);
    return String(buf);
}

bool pinEnabled() {
    return pinHashStored.length() == 8;
}

void loadPinFromPrefs() {
    prefs.begin("espcomm", true);
    pinHashStored = prefs.getString("pinhash", "");
    prefs.end();
}

void savePinHash(const String& hash) {
    pinHashStored = hash;
    prefs.begin("espcomm", false);
    if (hash.length()) prefs.putString("pinhash", hash);
    else prefs.remove("pinhash");
    prefs.end();
}

String makeSessionToken() {
    char tok[17];
    for (int i = 0; i < 16; i++) {
        uint8_t v = (uint8_t)(esp_random() & 0xF);
        tok[i] = "0123456789ABCDEF"[v];
    }
    tok[16] = 0;
    return String(tok);
}

String createSession() {
    // expire old
    uint32_t now = millis();
    for (int i = 0; i < MAX_SESSIONS; i++) {
        if (webSessions[i].used && (now - webSessions[i].created) > SESSION_TTL_MS)
            webSessions[i].used = false;
    }
    int slot = -1;
    for (int i = 0; i < MAX_SESSIONS; i++) {
        if (!webSessions[i].used) { slot = i; break; }
    }
    if (slot < 0) slot = 0; // overwrite oldest slot 0
    String tok = makeSessionToken();
    strncpy(webSessions[slot].token, tok.c_str(), 16);
    webSessions[slot].token[16] = 0;
    webSessions[slot].created = now;
    webSessions[slot].used = true;
    return tok;
}

void destroySessionToken(const String& tok) {
    for (int i = 0; i < MAX_SESSIONS; i++) {
        if (webSessions[i].used && tok.equals(webSessions[i].token))
            webSessions[i].used = false;
    }
}

String getCookieValue(const String& name) {
    if (!webServer.hasHeader("Cookie")) return "";
    String c = webServer.header("Cookie");
    String key = name + "=";
    int p = c.indexOf(key);
    if (p < 0) return "";
    p += key.length();
    int e = c.indexOf(';', p);
    if (e < 0) e = c.length();
    String v = c.substring(p, e);
    v.trim();
    return v;
}

bool sessionValid() {
    if (!pinEnabled()) return true;
    String tok = getCookieValue("comm_sess");
    if (tok.length() != 16) return false;
    uint32_t now = millis();
    for (int i = 0; i < MAX_SESSIONS; i++) {
        if (!webSessions[i].used) continue;
        if ((now - webSessions[i].created) > SESSION_TTL_MS) {
            webSessions[i].used = false;
            continue;
        }
        if (tok.equals(webSessions[i].token)) return true;
    }
    return false;
}

bool requireAuth() {
    if (!pinEnabled()) return true;
    if (sessionValid()) return true;
    webServer.send(401, "application/json",
                   "{\"ok\":false,\"error\":\"login erforderlich\",\"auth\":true}");
    return false;
}

void sendSessionCookie(const String& tok) {
    // 24h
    webServer.sendHeader("Set-Cookie",
        "comm_sess=" + tok + "; Path=/; Max-Age=86400; SameSite=Lax", true);
}

void clearSessionCookie() {
    webServer.sendHeader("Set-Cookie",
        "comm_sess=; Path=/; Max-Age=0; SameSite=Lax", true);
}

void handleLoginPage() {
    webServer.sendHeader(F("Content-Encoding"), F("gzip"));
    webServer.sendHeader(F("Cache-Control"), F("no-store"));
    webServer.send_P(200, "text/html", (PGM_P)PAGE_LOGIN_GZ, PAGE_LOGIN_GZ_LEN);
}

void handleApiAuthStatus() {
    bool logged = sessionValid();
    String j = "{";
    j += "\"ok\":true,";
    j += "\"pinEnabled\":"; j += pinEnabled() ? "true" : "false"; j += ",";
    j += "\"loggedIn\":"; j += logged ? "true" : "false";
    j += "}";
    webServer.send(200, "application/json", j);
}

void handleApiLogin() {
    if (!pinEnabled()) {
        webServer.send(200, "application/json", "{\"ok\":true,\"pinEnabled\":false}");
        return;
    }
    if (!webServer.hasArg("plain")) {
        webServer.send(400, "application/json", "{\"ok\":false}"); return;
    }
#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
#else
    DynamicJsonDocument doc(128);
#endif
    if (deserializeJson(doc, webServer.arg("plain")) != DeserializationError::Ok) {
        webServer.send(400, "application/json", "{\"ok\":false}"); return;
    }
    String pin = doc["pin"].as<String>();
    pin.trim();
    if (hashPin(pin) != pinHashStored) {
        delay(300);
        webServer.send(401, "application/json", "{\"ok\":false,\"error\":\"Falsche PIN\"}");
        return;
    }
    String tok = createSession();
    sendSessionCookie(tok);
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiLogout() {
    String tok = getCookieValue("comm_sess");
    if (tok.length()) destroySessionToken(tok);
    clearSessionCookie();
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiSetPin() {
    if (!requireAuth()) return;
    if (!webServer.hasArg("plain")) {
        webServer.send(400, "application/json", "{\"ok\":false}"); return;
    }
#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
#else
    DynamicJsonDocument doc(192);
#endif
    if (deserializeJson(doc, webServer.arg("plain")) != DeserializationError::Ok) {
        webServer.send(400, "application/json", "{\"ok\":false}"); return;
    }
    String action = doc["action"].as<String>(); // set | clear | change
    String pin = doc["pin"].as<String>();
    String oldPin = doc["oldPin"].as<String>();
    pin.trim(); oldPin.trim();

    if (action == "clear") {
        if (!pinEnabled()) { webServer.send(200, "application/json", "{\"ok\":true}"); return; }
        if (hashPin(oldPin) != pinHashStored) {
            webServer.send(401, "application/json", "{\"ok\":false,\"error\":\"Aktuelle PIN falsch\"}");
            return;
        }
        savePinHash("");
        // alle Sessions invalidieren
        for (int i = 0; i < MAX_SESSIONS; i++) webSessions[i].used = false;
        clearSessionCookie();
        logEvent("PIN entfernt");
        webServer.send(200, "application/json", "{\"ok\":true,\"pinEnabled\":false}");
        return;
    }

    // set or change
    if (pin.length() < 4 || pin.length() > 8) {
        webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"PIN 4-8 Zeichen\"}");
        return;
    }
    for (size_t i = 0; i < pin.length(); i++) {
        if (pin[i] < '0' || pin[i] > '9') {
            webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"Nur Ziffern\"}");
            return;
        }
    }
    if (pinEnabled()) {
        if (hashPin(oldPin) != pinHashStored) {
            webServer.send(401, "application/json", "{\"ok\":false,\"error\":\"Aktuelle PIN falsch\"}");
            return;
        }
    }
    savePinHash(hashPin(pin));
    String tok = createSession();
    sendSessionCookie(tok);
    logEvent("PIN gesetzt/geaendert");
    webServer.send(200, "application/json", "{\"ok\":true,\"pinEnabled\":true}");
}

void handleRoot() {
    if (pinEnabled() && !sessionValid()) { handleLoginPage(); return; }
    webServer.sendHeader(F("Content-Encoding"), F("gzip"));
    webServer.sendHeader(F("Cache-Control"), F("no-store"));
    webServer.send_P(200, "text/html", (PGM_P)PAGE_MAIN_GZ, PAGE_MAIN_GZ_LEN);
}

void handleOtaPage() {
    if (pinEnabled() && !sessionValid()) { handleLoginPage(); return; }
    webServer.sendHeader(F("Content-Encoding"), F("gzip"));
    webServer.sendHeader(F("Cache-Control"), F("no-store"));
    webServer.send_P(200, "text/html", (PGM_P)PAGE_OTA_GZ, PAGE_OTA_GZ_LEN);
}

void handleOtaUpload() {
    if (!requireAuth()) return;
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
    if (!requireAuth()) return;
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
    if (!requireAuth()) return;
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
    if (!requireAuth()) return;
    webServer.send(200, "application/json", inboxToJson());
}

void handleApiMarkRead() {
    if (!requireAuth()) return;
    inboxMarkAllRead();
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiClearChat() {
    if (!requireAuth()) return;
    String peerFilter = "";
    if (webServer.hasArg("plain")) {
#if ARDUINOJSON_VERSION_MAJOR >= 7
        JsonDocument doc;
#else
        DynamicJsonDocument doc(192);
#endif
        if (deserializeJson(doc, webServer.arg("plain")) == DeserializationError::Ok) {
            if (!doc["peer"].isNull()) peerFilter = doc["peer"].as<String>();
        }
    }
    if (peerFilter.length() > 0) {
        int n = inboxClearPeer(peerFilter.c_str());
        char buf[64];
        snprintf(buf, sizeof(buf), "Chat-Thread geleert (%d)", n);
        logEvent(buf);
        ssePush("messages", inboxToJson());
        ssePush("status", handleApiStatusJson());
        webServer.send(200, "application/json", "{\"ok\":true,\"removed\":" + String(n) + "}");
        return;
    }
    inboxCount = 0; inboxNext = 0; unreadCount = 0;
    memset(inbox, 0, sizeof(inbox));
    logEvent("Chat-Verlauf geleert");
    ssePush("messages", inboxToJson());
    ssePush("status", handleApiStatusJson());
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiDeleteMsg() {
    if (!requireAuth()) return;
    if (!webServer.hasArg("plain")) {
        webServer.send(400, "application/json", "{\"ok\":false}"); return;
    }
#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
#else
    DynamicJsonDocument doc(128);
#endif
    if (deserializeJson(doc, webServer.arg("plain")) != DeserializationError::Ok) {
        webServer.send(400, "application/json", "{\"ok\":false}"); return;
    }
    uint32_t id = doc["id"].as<uint32_t>();
    if (!id) {
        webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"id fehlt\"}"); return;
    }
    if (!inboxDeleteById(id)) {
        webServer.send(404, "application/json", "{\"ok\":false,\"error\":\"nicht gefunden\"}"); return;
    }
    ssePush("messages", inboxToJson());
    ssePush("status", handleApiStatusJson());
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiPeers() {
    if (!requireAuth()) return;
    webServer.send(200, "application/json", peersToJson());
}

void handleApiChannelSync() {
    if (!requireAuth()) return;
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
    if (!requireAuth()) return;
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
    if (!requireAuth()) return;
    lockedBssid = "";
    prefs.begin("espcomm", false);
    prefs.putString("bssid", "");
    prefs.end();
    logEvent("BSSID-Lock aufgehoben — verbindet sich beim naechsten Start an besten AP");
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiEvents() {
    if (!requireAuth()) return;
    webServer.send(200, "application/json", eventLogToJson());
}

void handleApiStatus() {
    if (!requireAuth()) return;
    webServer.send(200, "application/json", handleApiStatusJson());
}

// POST /api/call  {to: "MAC", action: "ring"|"accept"|"reject"|"end"}
void handleApiCall() {
    if (!requireAuth()) return;
    if (!webServer.hasArg("plain")) {
        webServer.send(400, "application/json", "{\"ok\":false}"); return;
    }
#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
#else
    DynamicJsonDocument doc(256);
#endif
    if (deserializeJson(doc, webServer.arg("plain")) != DeserializationError::Ok) {
        webServer.send(400, "application/json", "{\"ok\":false}"); return;
    }
    String action = doc["action"].as<String>();
    String toStr  = doc["to"].as<String>();

    uint8_t toMac[6];
    if (!parseMacStr(toStr, toMac)) {
        webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"MAC ungueltig\"}");
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
        webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"Unbekannte Aktion\"}");
        return;
    }
    espNowCall(toMac, callType);
    char evtbuf[80];
    snprintf(evtbuf, sizeof(evtbuf), "Anruf %s an %s", action.c_str(), callState.peerName);
    logEvent(evtbuf);
    webServer.send(200, "application/json", "{\"ok\":true}");
}

// POST /api/set-name  {name: "Neuer Name"}
void handleApiSetName() {
    if (!requireAuth()) return;
    if (!webServer.hasArg("plain")) {
        webServer.send(400, "application/json", "{\"ok\":false}"); return;
    }
#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
#else
    DynamicJsonDocument doc(128);
#endif
    if (deserializeJson(doc, webServer.arg("plain")) != DeserializationError::Ok) {
        webServer.send(400, "application/json", "{\"ok\":false}"); return;
    }
    String newName = doc["name"].as<String>();
    newName.trim();
    if (newName.length() == 0 || newName.length() > 31) {
        webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"Name 1-31 Zeichen\"}");
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
    webServer.send(200, "application/json", "{\"ok\":true,\"name\":\"" + escJson(newName) + "\"}");
}

void handleNotFound() {
    webServer.sendHeader("Location", "/", true);
    webServer.send(302, "text/plain", "");
}

// ================================================================
//  Web-Server Setup
// ================================================================

void setupWebServer() {
    // Cookie-Header fuer Session-Auth sammeln
    const char* hdrs[] = {"Cookie"};
    webServer.collectHeaders(hdrs, 1);

    webServer.on("/",              HTTP_GET,  handleRoot);
    webServer.on("/ota",           HTTP_GET,  handleOtaPage);
    webServer.on("/ota-upload",    HTTP_POST, handleOtaUploadFinish, handleOtaUpload);
    webServer.on("/api/auth-status", HTTP_GET,  handleApiAuthStatus);
    webServer.on("/api/login",       HTTP_POST, handleApiLogin);
    webServer.on("/api/logout",      HTTP_POST, handleApiLogout);
    webServer.on("/api/set-pin",     HTTP_POST, handleApiSetPin);
    webServer.on("/api/send",      HTTP_POST, handleApiSend);
    webServer.on("/api/messages",  HTTP_GET,  handleApiMessages);
    webServer.on("/api/mark-read",  HTTP_POST, handleApiMarkRead);
    webServer.on("/api/clear-chat", HTTP_POST, handleApiClearChat);
    webServer.on("/api/delete-msg", HTTP_POST, handleApiDeleteMsg);
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
    doc["chipModel"]  = hwInfo.chipModel();
    doc["version"]    = FW_VERSION;
    doc["ip"]         = getLocalIp();
    doc["rssi"]       = WiFi.RSSI();
    doc["uptime"]     = millis() / 1000UL;
    doc["freeHeap"]   = hwInfo.freeHeap();
    doc["freeSketch"] = hwInfo.freeSketchSpace();
    doc["hwSummary"]  = hwInfo.summaryLine();
    doc["module"]     = hwInfo.moduleGuess();
    {
        const int c0 = CpuLoad::core0Percent();
        const int c1 = CpuLoad::core1Percent();
        doc["cpu"] = (c1 >= 0) ? ((c0 + c1) / 2) : c0;
    }

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
                newName.trim();
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
    pinHashStored = prefs.getString("pinhash",  "");
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
    hwInfo.probe();
    hwInfo.logToSerial();
    CpuLoad::begin();

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
    CpuLoad::poll();
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
