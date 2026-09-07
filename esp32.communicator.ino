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

// ================================================================
//  KONFIGURATION  <-- Diese Werte anpassen
// ================================================================

// Gerätename — erscheint im Dashboard und als Absender
#define DEVICE_NAME            "Communicator-1"

// Firmware-Version
#define FW_VERSION             "1.8.0 communicator"

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
    json += "\"hubHost\":\"" + escJson(hubHost) + "\",";
    json += "\"hubPort\":" + String(hubPort) + ",";
    json += "\"version\":\"" + String(FW_VERSION) + "\",";
    json += "\"pinEnabled\":" + String(pinEnabled() ? "true" : "false");
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
    String html = R"rawhtml(<!DOCTYPE html>
<html lang="de"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Anmelden · Communicator</title>
<link href="https://fonts.googleapis.com/css2?family=Figtree:wght@500;700&family=Syne:wght@800&display=swap" rel="stylesheet">
<style>
body{margin:0;min-height:100vh;display:grid;place-items:center;font-family:Figtree,system-ui,sans-serif;
background:radial-gradient(800px 400px at 20% 0%,rgba(26,155,135,.22),transparent 55%),linear-gradient(165deg,#eef6f2,#dceae4);color:#14221f}
.card{width:min(380px,92vw);background:rgba(247,252,250,.9);border:1px solid #c9d8d2;border-radius:20px;padding:28px 22px;box-shadow:0 12px 30px rgba(20,34,31,.08)}
h1{font-family:Syne,system-ui,sans-serif;font-size:1.4rem;margin:0 0 6px;letter-spacing:-.02em}
p{color:#5c716b;font-size:14px;margin:0 0 18px}
input{width:100%;padding:12px 14px;border-radius:12px;border:1px solid #c9d8d2;font:inherit;margin-bottom:12px;box-sizing:border-box}
button{width:100%;padding:12px;border:none;border-radius:12px;font-weight:700;font:inherit;color:#fff;
background:linear-gradient(135deg,#0d6b63,#1a9b87);cursor:pointer}
.err{color:#b42318;font-size:13px;min-height:18px;margin-bottom:8px}
</style></head><body>
<div class="card">
  <h1>)rawhtml";
    html += escHtml(deviceName);
    html += R"rawhtml(</h1>
  <p>Geraete-PIN eingeben, um Chat und Anrufe zu oeffnen.</p>
  <div class="err" id="err"></div>
  <input id="pin" type="password" inputmode="numeric" maxlength="8" placeholder="PIN" autofocus
    onkeydown="if(event.key==='Enter')login()">
  <button onclick="login()">Anmelden</button>
</div>
<script>
function login(){
  var pin=document.getElementById('pin').value;
  var err=document.getElementById('err');
  err.textContent='';
  fetch('/api/login',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({pin:pin}),credentials:'same-origin'})
  .then(function(r){return r.json().then(function(d){return {ok:r.ok,d:d};});})
  .then(function(x){
    if(x.ok&&x.d.ok) location.href='/';
    else err.textContent=(x.d&&x.d.error)||'Falsche PIN';
  }).catch(function(){err.textContent='Verbindungsfehler';});
}
</script>
</body></html>
)rawhtml";
    webServer.send(200, "text/html; charset=utf-8", html);
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
    String bssidSection;
    if (lockedBssid.length() == 12) {
        bssidSection = "<span class='mono ok'>" + lockedBssid + "</span>"
                       " <button class='btn ghost' onclick='clearBssid()'>Aufheben</button>";
    } else {
        bssidSection = "<span class='muted'>nicht gesetzt</span>"
                       " <button class='btn ghost' onclick='lockBssid()'>AP sperren</button>";
    }
    uint8_t chD; wifi_second_chan_t scD;
    esp_wifi_get_channel(&chD, &scD);

    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    webServer.send(200, "text/html; charset=utf-8", "");

    webServer.sendContent(R"rawhtml(<!DOCTYPE html>
<html lang="de"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0a4f4a">
<title>Communicator</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Figtree:wght@400;500;600;700&family=Syne:wght@700;800&display=swap" rel="stylesheet">
<style>
:root{
  --ink:#14221f;--muted:#5c716b;--line:#c9d8d2;--bg0:#dceae4;--bg1:#eef6f2;
  --panel:#f7fcfa;--teal:#0d6b63;--sea:#1a9b87;--deep:#0a4f4a;
  --danger:#b42318;--ok:#1a7a4c;--warn:#a65d1a;--bubble:#dff3ee;
  --shadow:0 10px 30px rgba(20,34,31,.08);
  --font:"Figtree",system-ui,sans-serif;--display:"Syne",system-ui,sans-serif;
}
*{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%;}
body{
  color:var(--ink);font:15px/1.45 var(--font);
  background:
    radial-gradient(900px 420px at 0% 0%,rgba(26,155,135,.2),transparent 55%),
    radial-gradient(700px 380px at 100% 0%,rgba(13,107,99,.14),transparent 50%),
    linear-gradient(165deg,var(--bg1),var(--bg0));
}
button,input,textarea{font:inherit}
button{cursor:pointer}
.mono{font-family:ui-monospace,Menlo,monospace;font-size:12px}
.muted{color:var(--muted)} .ok{color:var(--ok)}
.app{
  max-width:1100px;margin:0 auto;height:100%;min-height:100dvh;
  display:grid;grid-template-columns:1fr;position:relative;
}
@media(min-width:860px){
  body{padding:18px}
  .app{
    height:calc(100dvh - 36px);min-height:560px;border-radius:22px;overflow:hidden;
    border:1px solid rgba(201,216,210,.9);box-shadow:var(--shadow);
    grid-template-columns:340px 1fr;background:rgba(247,252,250,.72);backdrop-filter:blur(14px);
  }
}
.pane{display:none;flex-direction:column;min-height:0;height:100%;background:rgba(247,252,250,.82)}
.pane.show{display:flex}
@media(min-width:860px){
  #view-list{display:flex !important;border-right:1px solid var(--line)}
  #view-chat{display:flex !important}
  #view-more.overlay{position:absolute;inset:0;z-index:20;background:rgba(12,28,26,.35);padding:24px;align-items:center;justify-content:center}
  #view-more.overlay .sheet{width:min(520px,100%);max-height:90%;overflow:auto;border-radius:18px;background:var(--panel);border:1px solid var(--line);box-shadow:var(--shadow)}
}
.top{
  display:flex;align-items:center;gap:12px;padding:14px 16px;border-bottom:1px solid var(--line);
  background:rgba(255,255,255,.55);backdrop-filter:blur(10px);flex-shrink:0;
}
.logo{
  width:40px;height:40px;border-radius:12px;flex-shrink:0;position:relative;
  background:conic-gradient(from 200deg,var(--teal),var(--sea),#7ed7c6,var(--teal));
  animation:spin 16s linear infinite;
}
.logo:after{content:"";position:absolute;inset:6px;border-radius:8px;background:var(--panel)}
.logo i{position:absolute;inset:0;margin:auto;width:8px;height:8px;border-radius:50%;background:var(--teal);z-index:1;animation:pulse 2.2s ease-in-out infinite}
.brand .k{font-size:10px;letter-spacing:.14em;text-transform:uppercase;color:var(--muted);font-weight:700}
.brand h1{font:800 1.15rem/1.1 var(--display);letter-spacing:-.03em}
.icon-btn{
  width:40px;height:40px;border-radius:12px;border:1px solid var(--line);background:#fff;
  color:var(--ink);display:grid;place-items:center;font-size:18px;flex-shrink:0;
}
.icon-btn:hover{border-color:var(--teal);color:var(--teal)}
.search{
  margin:12px 14px 6px;padding:10px 12px;border-radius:12px;border:1px solid var(--line);
  background:#fff;width:calc(100% - 28px);
}
.clist{flex:1;overflow:auto;padding:6px 10px 18px}
.crow{
  display:flex;gap:12px;align-items:center;padding:12px;border-radius:14px;cursor:pointer;
  border:1px solid transparent;transition:background .15s,border-color .15s,transform .15s;
  background:rgba(255,255,255,.35);
}
.crow:hover{background:#fff;border-color:var(--line);transform:translateY(-1px)}
.crow.active{background:#fff;border-color:var(--teal);box-shadow:0 0 0 2px rgba(13,107,99,.12)}
.crow.offline{opacity:.72}
.av{
  width:48px;height:48px;border-radius:16px;flex-shrink:0;display:grid;place-items:center;
  font:800 15px/1 var(--display);color:#fff;background:linear-gradient(135deg,var(--teal),var(--sea));
  position:relative;box-shadow:0 4px 12px rgba(13,107,99,.18);
}
.av.bc{background:linear-gradient(135deg,#355c57,#6aa89c);box-shadow:none}
.av .dot{
  position:absolute;right:-2px;bottom:-2px;width:14px;height:14px;border-radius:50%;
  border:2.5px solid #fff;background:#9aada6;
}
.av .dot.on{background:#12b886;box-shadow:0 0 0 4px rgba(18,184,134,.22);animation:pulse 2s ease-in-out infinite}
.cmeta{flex:1;min-width:0}
.cmeta .n{font-weight:700;font-size:15px;display:flex;justify-content:space-between;gap:8px;align-items:center}
.cmeta .p{color:var(--muted);font-size:13px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;margin-top:3px}
.cmeta .subline{display:flex;gap:8px;align-items:center;margin-top:4px;flex-wrap:wrap}
.st{
  font-size:11px;font-weight:800;letter-spacing:.02em;text-transform:uppercase;
  padding:3px 8px;border-radius:8px;border:1px solid transparent;
}
.st.on{color:#0b7a55;background:rgba(18,184,134,.14);border-color:rgba(18,184,134,.35)}
.st.off{color:#6b7280;background:rgba(107,114,128,.1);border-color:rgba(107,114,128,.22)}
.st.bc{color:var(--teal-deep);background:rgba(13,107,99,.1);border-color:rgba(13,107,99,.25)}
.macs{font-family:ui-monospace,Menlo,monospace;font-size:11px;color:var(--muted)}
.pill{
  min-width:20px;height:20px;padding:0 6px;border-radius:999px;background:var(--teal);color:#fff;
  font-size:11px;font-weight:700;display:inline-grid;place-items:center;
}
.chat-head{gap:10px}
.chat-head .who{flex:1;min-width:0}
.chat-head .who .n{font:700 1rem/1.15 var(--display);letter-spacing:-.02em}
.chat-head .who .s{font-size:12px;color:var(--muted)}
.btn{
  border:none;border-radius:12px;padding:10px 14px;font-weight:700;
  background:linear-gradient(135deg,var(--teal),var(--sea));color:#fff;
}
.btn.ghost{background:#fff;color:var(--muted);border:1px solid var(--line)}
.btn.ghost:hover{color:var(--teal);border-color:var(--teal)}
.btn.danger{background:var(--danger)}
.btn.call{padding:10px 16px}
.thread{
  flex:1;overflow:auto;padding:16px 14px 10px;display:flex;flex-direction:column;gap:8px;
  background:
    linear-gradient(180deg,rgba(255,255,255,.15),transparent 40%),
    repeating-linear-gradient(0deg,transparent,transparent 27px,rgba(20,34,31,.03) 28px);
}
.day{align-self:center;font-size:11px;font-weight:700;color:var(--muted);background:rgba(255,255,255,.7);
  border:1px solid var(--line);padding:4px 10px;border-radius:999px;margin:6px 0}
.bubble{
  max-width:min(78%,420px);padding:10px 12px;border-radius:16px;font-size:14px;line-height:1.45;
  word-break:break-word;animation:rise .28s ease;border:1px solid transparent;
}
.bubble.in{align-self:flex-start;background:#fff;border-color:var(--line);border-bottom-left-radius:6px}
.bubble.out{align-self:flex-end;background:var(--bubble);border-color:#bfe6dc;border-bottom-right-radius:6px}
.bubble .meta{display:flex;gap:8px;align-items:center;margin-top:5px;font-size:11px;color:var(--muted)}
.bubble .fn{font-weight:700;font-size:12px;color:var(--teal);margin-bottom:2px}
.bubble .del{
  margin-left:auto;border:none;background:transparent;color:var(--muted);cursor:pointer;
  font-size:12px;font-weight:700;padding:2px 6px;border-radius:6px;
}
.bubble .del:hover{color:var(--danger);background:rgba(180,35,24,.08)}
.chat-actions{display:flex;gap:6px;align-items:center;flex-shrink:0}
.empty{
  margin:auto;text-align:center;color:var(--muted);padding:28px 18px;max-width:320px;
}
.empty strong{display:block;font:700 1.15rem/1.2 var(--display);color:var(--ink);margin-bottom:8px}
.composer{
  display:flex;flex-direction:column;gap:8px;padding:12px;border-top:1px solid var(--line);background:rgba(255,255,255,.7);
  backdrop-filter:blur(10px);flex-shrink:0;
}
.composer-row{display:flex;gap:8px;align-items:flex-end}
.composer textarea{
  flex:1;resize:none;min-height:44px;max-height:120px;padding:11px 12px;border-radius:14px;
  border:1px solid var(--line);background:#fff;outline:none;
}
.composer textarea:focus{border-color:var(--teal);box-shadow:0 0 0 3px rgba(13,107,99,.12)}
.composer .send{width:48px;height:48px;border-radius:14px;flex-shrink:0}
.emoji-bar{display:flex;gap:4px;flex-wrap:wrap;max-height:0;overflow:hidden;opacity:0;transition:max-height .2s,opacity .2s}
.emoji-bar.open{max-height:120px;opacity:1}
.emoji-btn{border:1px solid transparent;background:transparent;border-radius:8px;padding:4px 6px;font-size:18px;line-height:1;cursor:pointer}
.emoji-btn:hover{background:#fff;border-color:var(--line)}
.pin-box{background:#fff;border:1px solid var(--line);border-radius:14px;padding:12px 14px;margin-bottom:12px}
.pin-box h3{font-size:13px;margin:0 0 8px}
.pin-box .hint{font-size:12px;color:var(--muted);margin-bottom:10px}
.pin-box input{width:100%;padding:10px 12px;border-radius:10px;border:1px solid var(--line);margin-bottom:8px;box-sizing:border-box}
.sheet{padding:18px}
.sheet h2{font:800 1.2rem/1.1 var(--display);margin-bottom:4px}
.sheet .sub{color:var(--muted);font-size:13px;margin-bottom:16px}
.statgrid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:14px}
.stat{background:#fff;border:1px solid var(--line);border-radius:12px;padding:12px;text-align:center}
.stat b{display:block;font:800 1.25rem/1 var(--display);color:var(--teal)}
.stat span{font-size:11px;color:var(--muted);font-weight:700;text-transform:uppercase;letter-spacing:.04em}
.rows{background:#fff;border:1px solid var(--line);border-radius:14px;overflow:hidden;margin-bottom:12px}
.row{display:flex;justify-content:space-between;gap:12px;padding:12px 14px;border-bottom:1px solid var(--line);font-size:13px}
.row:last-child{border-bottom:0}
.row span:first-child{color:var(--muted);font-weight:600}
.name-form{display:flex;gap:8px;margin-top:8px}
.name-form input{flex:1;padding:10px 12px;border-radius:12px;border:1px solid var(--line);background:#fff}
.logbox{max-height:180px;overflow:auto;font-size:12px}
.logline{display:flex;justify-content:space-between;gap:8px;padding:7px 0;border-bottom:1px solid var(--line)}
#toast{
  position:fixed;left:50%;bottom:22px;transform:translateX(-50%) translateY(8px);
  background:var(--deep);color:#fff;padding:10px 16px;border-radius:12px;font-weight:600;font-size:13px;
  opacity:0;transition:.25s;z-index:50;pointer-events:none;
}
#toast.show{opacity:1;transform:translateX(-50%) translateY(0)}
.modal-overlay{display:none;position:fixed;inset:0;background:rgba(12,28,26,.55);z-index:60;
  align-items:center;justify-content:center;padding:16px;backdrop-filter:blur(6px)}
.modal-overlay.show{display:flex}
.modal{background:#fff;border:1px solid var(--line);border-radius:20px;padding:24px;width:min(420px,100%);text-align:center;animation:rise .28s ease}
.modal h2{font:800 1.2rem/1.15 var(--display);margin-bottom:6px}
.modal p{color:var(--muted);margin-bottom:16px}
.call-av{width:64px;height:64px;margin:0 auto 12px;border-radius:18px;display:grid;place-items:center;
  color:#fff;font-size:28px;background:linear-gradient(135deg,var(--teal),var(--sea));animation:pulse 1.4s ease-in-out infinite}
.call-btns{display:flex;gap:8px;justify-content:center;flex-wrap:wrap}
.rtc-media{display:flex;gap:8px;justify-content:center;margin:12px 0;flex-wrap:wrap}
.rtc-media video{width:148px;height:111px;object-fit:cover;border-radius:12px;background:#14221f;border:1px solid var(--line)}
#rtc-local{transform:scaleX(-1)}
.rtc-hint{font-size:11px;color:var(--muted);margin-top:8px}
@keyframes rise{from{opacity:0;transform:translateY(8px)}to{opacity:1;transform:none}}
@keyframes pulse{0%,100%{transform:scale(1)}50%{transform:scale(1.07)}}
@keyframes spin{to{transform:rotate(360deg)}}
@media(max-width:859px){
  #view-chat.show,#view-list.show,#view-more.show{position:fixed;inset:0;z-index:5}
  #view-more .sheet{height:100%;overflow:auto;background:var(--panel)}
}
</style></head><body>
<div class="app" id="app">
)rawhtml");

    // Dynamic list header
    {
        String h;
        h.reserve(500);
        h = "<section class='pane show' id='view-list'>"
            "<div class='top'><div class='logo' aria-hidden='true'><i></i></div>"
            "<div class='brand'><div class='k'>ESP Mesh Link</div><h1>";
        h += escHtml(deviceName);
        h += "</h1></div>"
             "<button class='icon-btn' title='Mehr' onclick='showMore()' aria-label='Mehr'>&#8943;</button>"
             "</div>"
             "<input class='search' id='q' placeholder='Kontakt suchen...' oninput='renderList()'>"
             "<div class='clist' id='clist'></div></section>";
        webServer.sendContent(h);
    }

    webServer.sendContent(R"rawhtml(
<section class="pane" id="view-chat">
  <div class="top chat-head">
    <button class="icon-btn back" onclick="showList()" aria-label="Zurueck">&#8592;</button>
    <div class="who"><div class="n" id="chat-name">Chat</div><div class="s" id="chat-sub"> </div></div>
    <div class="chat-actions">
      <button class="btn ghost" id="btn-clear" title="Chat leeren" onclick="clearThread()">Leeren</button>
      <button class="btn call" id="btn-call" onclick="callCurrent()">Anruf</button>
    </div>
  </div>
  <div class="thread" id="thread"></div>
  <div class="composer">
    <div class="emoji-bar" id="emoji-bar">
      <button type="button" class="emoji-btn" onclick="addEmoji('😀')">😀</button>
      <button type="button" class="emoji-btn" onclick="addEmoji('😂')">😂</button>
      <button type="button" class="emoji-btn" onclick="addEmoji('😍')">😍</button>
      <button type="button" class="emoji-btn" onclick="addEmoji('👍')">👍</button>
      <button type="button" class="emoji-btn" onclick="addEmoji('👎')">👎</button>
      <button type="button" class="emoji-btn" onclick="addEmoji('❤️')">❤️</button>
      <button type="button" class="emoji-btn" onclick="addEmoji('🔥')">🔥</button>
      <button type="button" class="emoji-btn" onclick="addEmoji('🎉')">🎉</button>
      <button type="button" class="emoji-btn" onclick="addEmoji('👋')">👋</button>
      <button type="button" class="emoji-btn" onclick="addEmoji('🙏')">🙏</button>
      <button type="button" class="emoji-btn" onclick="addEmoji('✅')">✅</button>
      <button type="button" class="emoji-btn" onclick="addEmoji('❌')">❌</button>
      <button type="button" class="emoji-btn" onclick="addEmoji('⭐')">⭐</button>
      <button type="button" class="emoji-btn" onclick="addEmoji('💡')">💡</button>
      <button type="button" class="emoji-btn" onclick="addEmoji('🏠')">🏠</button>
      <button type="button" class="emoji-btn" onclick="addEmoji('☕')">☕</button>
    </div>
    <div class="composer-row">
      <button class="btn ghost" type="button" onclick="toggleEmoji()" title="Emoji" style="width:48px;height:48px;padding:0;flex-shrink:0">☺</button>
      <textarea id="msg" rows="1" maxlength="199" placeholder="Nachricht..." onkeydown="if(event.key==='Enter'&&!event.shiftKey){event.preventDefault();sendMsg();}"></textarea>
      <button class="btn send" onclick="sendMsg()" aria-label="Senden">&#9654;</button>
    </div>
  </div>
</section>
<section class="pane" id="view-more">
  <div class="sheet">
    <div class="top" style="padding:0 0 14px;border:0;background:transparent">
      <button class="icon-btn" onclick="hideMore()" aria-label="Schliessen">&#10005;</button>
      <div class="brand"><div class="k">Geraet</div><h2 style="font:800 1.15rem/1.1 var(--display)">Status &amp; Einstellungen</h2></div>
    </div>
)rawhtml");

    {
        String s;
        s.reserve(1200);
        s = "<div class='statgrid'>"
            "<div class='stat'><b id='s-peers'>";
        s += String(peerCount);
        s += "</b><span>Peers</span></div>"
             "<div class='stat'><b id='s-unread'>";
        s += String(unreadCount);
        s += "</b><span>Ungelesen</span></div>"
             "<div class='stat'><b id='s-ch'>";
        s += String(chD);
        s += "</b><span>Kanal</span></div></div>"
             "<div class='rows'>"
             "<div class='row'><span>Firmware</span><span>";
        s += FW_VERSION;
        s += "</span></div>"
             "<div class='row'><span>IP</span><span class='mono' id='s-ip'>";
        s += getLocalIp();
        s += "</span></div>"
             "<div class='row'><span>MAC</span><span class='mono'>";
        s += getMacFmt();
        s += "</span></div>"
             "<div class='row'><span>RSSI</span><span id='s-rssi'>";
        s += String(WiFi.RSSI());
        s += " dBm</span></div>"
             "<div class='row'><span>Uptime</span><span id='s-uptime'>";
        s += fmtUptime(millis() / 1000UL);
        s += "</span></div>"
             "<div class='row'><span>ESP-NOW</span><span id='s-espnow'>";
        s += espNowReady ? "<span class='ok'>Aktiv</span>" : "Offline";
        s += "</span></div>"
             "<div class='row'><span>Gesendet / Empfangen</span><span><span id='s-sent'>";
        s += String(sentTotal);
        s += "</span> / <span id='s-recv'>";
        s += String(recvTotal);
        s += "</span></span></div>"
             "<div class='row'><span>BSSID-Lock</span><span>";
        s += bssidSection;
        s += "</span></div></div>"
             "<div class='rows' style='padding:12px 14px'>"
             "<div style='font-weight:700;margin-bottom:6px'>Anzeigename</div>"
             "<div class='name-form'><input id='my-name' maxlength='31' value='";
        s += escHtml(deviceName);
        s += "'><button class='btn' onclick='setMyName()'>Speichern</button></div></div>"
             "<div class='pin-box' id='pin-box'>"
             "<h3>Geraete-PIN</h3>"
             "<div class='hint' id='pin-hint'>Schuetzt die Web-UI. ESP-NOW bleibt geraeteweit.</div>"
             "<input id='pin-old' type='password' inputmode='numeric' maxlength='8' placeholder='Aktuelle PIN' style='display:none'>"
             "<input id='pin-new' type='password' inputmode='numeric' maxlength='8' placeholder='Neue PIN (4-8 Ziffern)'>"
             "<div style='display:flex;gap:8px;flex-wrap:wrap'>"
             "<button class='btn' onclick='savePin()'>PIN speichern</button>"
             "<button class='btn ghost' id='pin-clear-btn' onclick='clearPin()' style='display:none'>PIN entfernen</button>"
             "<button class='btn ghost' onclick='doLogout()'>Abmelden</button>"
             "</div></div>"
             "<div style='display:flex;gap:8px;flex-wrap:wrap;margin-bottom:12px'>"
             "<button class='btn ghost' onclick='manualSync()'>Kanal-Sync</button>"
             "<a class='btn ghost' href='/ota' style='text-decoration:none;display:inline-block'>OTA Update</a>"
             "</div>"
             "<div class='rows' style='padding:12px 14px'>"
             "<div style='font-weight:700;margin-bottom:8px'>Ereignisse</div>"
             "<div class='logbox' id='log-list'></div></div>"
             "</div></section>";
        webServer.sendContent(s);
    }

    {
        String cfg;
        cfg.reserve(220);
        cfg = "<script>window.COMM_CFG={mac:'";
        cfg += getMac();
        cfg += "',macFmt:'";
        cfg += getMacFmt();
        cfg += "',name:'";
        cfg += escJson(deviceName);
        cfg += "',hub:'http://";
        cfg += hubHost;
        cfg += ":";
        cfg += String(hubPort);
        cfg += "'};</script>\n";
        webServer.sendContent(cfg);
    }

    webServer.sendContent(R"rawjs(
<div id="call-modal" class="modal-overlay"><div class="modal">
  <div class="call-av" id="call-avatar">&#9742;</div>
  <h2 id="call-title">Eingehender Anruf</h2>
  <p id="call-from">-</p>
  <div class="call-btns">
    <button class="btn" onclick="acceptCall()">Annehmen</button>
    <button class="btn danger" onclick="rejectCall()">Ablehnen</button>
  </div>
</div></div>
<div id="active-call-modal" class="modal-overlay"><div class="modal">
  <h2 id="active-call-name">Anruf</h2>
  <p id="active-call-duration">00:00</p>
  <p id="rtc-status" class="rtc-hint" style="margin:0 0 8px">Verbinde...</p>
  <div id="rtc-warn" class="rtc-hint" style="display:none;color:var(--warn);margin-bottom:8px"></div>
  <div class="rtc-media">
    <video id="rtc-remote" autoplay playsinline></video>
    <video id="rtc-local" autoplay playsinline muted></video>
  </div>
  <audio id="rtc-remote-audio" autoplay playsinline></audio>
  <div class="call-btns">
    <button class="btn ghost" id="rtc-cam-btn" onclick="toggleCam()">Kamera</button>
    <button class="btn ghost" id="rtc-mic-btn" onclick="toggleMic()">Mikro</button>
    <button class="btn danger" onclick="endCall()">Auflegen</button>
  </div>
  <div class="rtc-hint">Audio/Video im Browser · Signaling ueber ESP-Hub</div>
</div></div>
<div id="toast"></div>
<script>
var CFG=window.COMM_CFG||{};
var MY_MAC=normMac(CFG.mac||'');
var HUB_BASE=CFG.hub||'';
var peers=[], msgs=[], logs=[], activePeer=null, activeName='';
var desktop=window.matchMedia('(min-width:860px)').matches;
window.addEventListener('resize',function(){desktop=window.matchMedia('(min-width:860px)').matches;syncViews();});

function normMac(m){return String(m||'').replace(/[^a-fA-F0-9]/g,'').toUpperCase();}
function sameMac(a,b){
  if(a==='bc'||b==='bc') return String(a).toLowerCase()==='bc'&&String(b).toLowerCase()==='bc';
  return normMac(a)===normMac(b);
}
function peerKey(p){return p==='bc'?'bc':normMac(p);}
function toast(msg,ok){
  var t=document.getElementById('toast');
  t.textContent=msg; t.style.background=ok?'#0a4f4a':'#b42318';
  t.classList.add('show'); setTimeout(function(){t.classList.remove('show');},2400);
}
function initials(name){
  var p=String(name||'?').trim().split(/\s+/);
  return ((p[0]||'?')[0]+(p[1]?p[1][0]:'')).toUpperCase();
}
function ageStr(age){
  age=age||0;
  return age<60?age+'s':age<3600?Math.floor(age/60)+'min':Math.floor(age/3600)+'h';
}
function msgPeer(m){
  if(m.peer) return m.peer;
  if(m.bc) return 'bc';
  return m.sent ? null : m.mac;
}
function threadMsgs(key){
  return msgs.filter(function(m){
    var p=msgPeer(m); if(!p) return false;
    return sameMac(p,key);
  });
}
function unreadFor(key){
  return threadMsgs(key).filter(function(m){return !m.sent && !m.read;}).length;
}
function lastPreview(key){
  var t=threadMsgs(key);
  if(!t.length) return key==='bc' ? 'Nachricht an alle Geraete' : 'Tippen zum Chatten';
  var m=t[0];
  var prefix=m.sent?'Du: ':(m.bc?(m.from+': '):'');
  return prefix+(m.text||'');
}
function macTail(mac){
  if(!mac||mac==='bc') return '';
  var s=String(mac).toUpperCase();
  return s.length>=5 ? s.slice(-5) : s;
}

function syncViews(){
  var list=document.getElementById('view-list');
  var chat=document.getElementById('view-chat');
  if(desktop){
    list.classList.add('show'); chat.classList.add('show');
    document.querySelector('.back').style.visibility='hidden';
  } else {
    document.querySelector('.back').style.visibility='visible';
  }
}
function showList(){
  if(desktop){activePeer=null;renderList();renderThread();return;}
  document.getElementById('view-list').classList.add('show');
  document.getElementById('view-chat').classList.remove('show');
  document.getElementById('view-more').classList.remove('show');
}
function openChat(mac,name){
  activePeer=mac; activeName=name||mac;
  document.getElementById('chat-name').textContent=activeName;
  var peer=peers.find(function(p){return sameMac(p.mac,mac);});
  if(mac==='bc'){
    document.getElementById('chat-sub').innerHTML='Broadcast an alle Teilnehmer';
  } else {
    var on=peer&&peer.online;
    document.getElementById('chat-sub').innerHTML=
      '<span class="st '+(on?'on':'off')+'">'+(on?'Online':'Offline')+'</span> · <span class="macs">'+esc(mac)+'</span>';
  }
  document.getElementById('btn-call').style.display = (mac==='bc')?'none':'inline-block';
  if(!desktop){
    document.getElementById('view-list').classList.remove('show');
    document.getElementById('view-chat').classList.add('show');
    document.getElementById('view-more').classList.remove('show');
  }
  renderList(); renderThread();
  markRead();
  setTimeout(function(){var el=document.getElementById('msg'); if(el)el.focus();},50);
}
function showMore(){
  var v=document.getElementById('view-more');
  v.classList.add('show');
  if(desktop) v.classList.add('overlay'); else v.classList.remove('overlay');
  refreshPinUi();
}
function hideMore(){
  var v=document.getElementById('view-more');
  v.classList.remove('show'); v.classList.remove('overlay');
}

function renderList(){
  var q=(document.getElementById('q').value||'').toLowerCase();
  var el=document.getElementById('clist');
  var items=[{mac:'bc',name:'Alle',online:true,bc:true}];
  var sorted=peers.slice().sort(function(a,b){
    if(!!a.online!==!!b.online) return a.online?-1:1;
    return String(a.name||'').localeCompare(String(b.name||''));
  });
  sorted.forEach(function(p){items.push(p);});
  items=items.filter(function(p){
    if(!q) return true;
    return (p.name||'').toLowerCase().indexOf(q)>=0 || (p.mac||'').toLowerCase().indexOf(q)>=0;
  });
  if(!items.length){
    el.innerHTML='<div class="empty"><strong>Keine Treffer</strong>Andere Suche versuchen</div>';
    return;
  }
  var onlineN=peers.filter(function(p){return p.online;}).length;
  var head='<div style="padding:4px 8px 10px;color:var(--muted);font-size:12px;font-weight:700">'
    +onlineN+' von '+peers.length+' online</div>';
  el.innerHTML=head+items.map(function(p){
    var key=p.bc?'bc':p.mac;
    var ur=unreadFor(key);
    var active=activePeer && sameMac(activePeer,key);
    var st=p.bc?'<span class="st bc">Broadcast</span>'
      :(p.online?'<span class="st on">Online</span>':'<span class="st off">Offline</span>');
    var tail=p.bc?'':'<span class="macs">'+esc(macTail(p.mac))+'</span>';
    return '<div class="crow'+(active?' active':'')+(p.online||p.bc?'':' offline')+'" onclick="openChat(\''+key+'\',\''+String(p.name||'').replace(/'/g,'')+'\')">'
      +'<div class="av'+(p.bc?' bc':'')+'">'+initials(p.name)
      +(p.bc?'':('<span class="dot '+(p.online?'on':'')+'"></span>'))
      +'</div><div class="cmeta"><div class="n"><span>'+esc(p.name)+'</span>'
      +(ur?'<span class="pill">'+ur+'</span>':'')+'</div>'
      +'<div class="p">'+esc(lastPreview(key))+'</div>'
      +'<div class="subline">'+st+tail+'</div></div></div>';
  }).join('');
}
function esc(s){return String(s||'').replace(/[&<>"']/g,function(c){return ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'})[c];});}

function renderThread(){
  var el=document.getElementById('thread');
  if(!activePeer){
    el.innerHTML='<div class="empty"><strong>Willkommen</strong>Waehle links einen Kontakt oder &bdquo;Alle&ldquo; fuer Broadcast.</div>';
    return;
  }
  var list=threadMsgs(activePeer).slice().reverse();
  if(!list.length){
    el.innerHTML='<div class="empty"><strong>Noch ruhig hier</strong>Schreib die erste Nachricht an '+esc(activeName)+'.</div>';
    return;
  }
  el.innerHTML=list.map(function(m){
    var cls=m.sent?'out':'in';
    var who=(!m.sent && (m.bc||activePeer==='bc')) ? '<div class="fn">'+esc(m.from)+'</div>' : '';
    var id=m.id||0;
    return '<div class="bubble '+cls+'" data-id="'+id+'">'+who+esc(m.text)
      +'<div class="meta"><span>'+ageStr(m.age)+'</span>'
      +(m.bc?'<span>Broadcast</span>':'')
      +(id?'<button class="del" title="Loeschen" onclick="event.stopPropagation();deleteMsg('+id+')">Loeschen</button>':'')
      +'</div></div>';
  }).join('');
  el.scrollTop=el.scrollHeight;
}
function deleteMsg(id){
  if(!id) return;
  if(!confirm('Diese Nachricht loeschen?')) return;
  fetch('/api/delete-msg',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({id:id})})
  .then(function(r){return r.json();}).then(function(d){
    if(d.ok){
      msgs=msgs.filter(function(m){return m.id!==id;});
      renderList(); renderThread(); toast('Nachricht geloescht',true);
    } else toast('Loeschen fehlgeschlagen',false);
  }).catch(function(){toast('Fehler',false);});
}
function clearThread(){
  if(!activePeer){toast('Kein Chat gewaehlt',false);return;}
  var label=activePeer==='bc'?'Broadcast':'diesen Chat';
  if(!confirm(label+' wirklich leeren?')) return;
  fetch('/api/clear-chat',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({peer:activePeer})})
  .then(function(r){return r.json();}).then(function(d){
    if(d.ok){
      msgs=msgs.filter(function(m){ var p=msgPeer(m); return !(p&&sameMac(p,activePeer)); });
      renderList(); renderThread(); toast('Chat geleert',true);
    } else toast('Fehler',false);
  }).catch(function(){toast('Fehler',false);});
}

function sendMsg(){
  if(!activePeer){toast('Erst Kontakt waehlen',false);return;}
  var txt=document.getElementById('msg').value.trim();
  if(!txt){toast('Bitte Text eingeben',false);return;}
  fetch('/api/send',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({to:activePeer,text:txt}),credentials:'same-origin'})
  .then(function(r){return r.json();}).then(function(d){
    if(d.ok){document.getElementById('msg').value='';toast('Gesendet',true);}
    else toast('Fehler: '+(d.error||''),false);
  }).catch(function(){toast('Verbindungsfehler',false);});
}
function toggleEmoji(){
  var bar=document.getElementById('emoji-bar');
  if(bar) bar.classList.toggle('open');
}
function addEmoji(em){
  var ta=document.getElementById('msg');
  if(!ta) return;
  var start=ta.selectionStart||ta.value.length;
  var end=ta.selectionEnd||ta.value.length;
  var v=ta.value;
  var next=v.slice(0,start)+em+v.slice(end);
  if(next.length>199) next=next.slice(0,199);
  ta.value=next;
  var pos=Math.min(start+em.length, next.length);
  ta.focus();
  try{ta.setSelectionRange(pos,pos);}catch(e){}
}
function refreshPinUi(){
  fetch('/api/auth-status',{credentials:'same-origin'}).then(function(r){return r.json();}).then(function(d){
    var on=!!d.pinEnabled;
    var hint=document.getElementById('pin-hint');
    var old=document.getElementById('pin-old');
    var clr=document.getElementById('pin-clear-btn');
    if(hint) hint.textContent=on
      ? 'PIN ist aktiv. Web-UI verlangt Anmeldung. ESP-NOW bleibt geraeteweit.'
      : 'Noch keine PIN. Optional setzen, um die Web-UI zu schuetzen.';
    if(old) old.style.display=on?'':'none';
    if(clr) clr.style.display=on?'':'none';
  }).catch(function(){});
}
function savePin(){
  var pin=(document.getElementById('pin-new').value||'').trim();
  var old=(document.getElementById('pin-old').value||'').trim();
  if(pin.length<4||pin.length>8){toast('PIN 4-8 Ziffern',false);return;}
  fetch('/api/set-pin',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({action:'set',pin:pin,oldPin:old}),credentials:'same-origin'})
  .then(function(r){return r.json();}).then(function(d){
    if(d.ok){
      document.getElementById('pin-new').value='';
      document.getElementById('pin-old').value='';
      toast('PIN gespeichert',true);
      refreshPinUi();
    } else toast(d.error||'Fehler',false);
  }).catch(function(){toast('Fehler',false);});
}
function clearPin(){
  var old=(document.getElementById('pin-old').value||'').trim();
  if(!old){toast('Aktuelle PIN eingeben',false);return;}
  if(!confirm('PIN wirklich entfernen?')) return;
  fetch('/api/set-pin',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({action:'clear',oldPin:old}),credentials:'same-origin'})
  .then(function(r){return r.json();}).then(function(d){
    if(d.ok){
      document.getElementById('pin-new').value='';
      document.getElementById('pin-old').value='';
      toast('PIN entfernt',true);
      refreshPinUi();
    } else toast(d.error||'Fehler',false);
  }).catch(function(){toast('Fehler',false);});
}
function doLogout(){
  fetch('/api/logout',{method:'POST',credentials:'same-origin'})
  .then(function(){ location.href='/'; })
  .catch(function(){ location.href='/'; });
}
function markRead(){
  fetch('/api/mark-read',{method:'POST'}).then(function(){
    msgs.forEach(function(m){m.read=true;});
    renderList(); updateStatusBadge();
  });
}
function setMyName(){
  var name=document.getElementById('my-name').value.trim();
  if(!name||name.length>31){toast('Name 1-31 Zeichen',false);return;}
  fetch('/api/set-name',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({name:name})})
  .then(function(r){return r.json();}).then(function(d){
    if(d.ok){toast('Name gespeichert',true);setTimeout(function(){location.reload();},800);}
    else toast('Fehler',false);
  });
}
function lockBssid(){fetch('/api/bssid-lock',{method:'POST'}).then(function(r){return r.json();}).then(function(d){if(d.ok){toast('AP gesperrt',true);setTimeout(function(){location.reload();},1000);}else toast('Fehler',false);});}
function clearBssid(){fetch('/api/bssid-clear',{method:'POST'}).then(function(){toast('Lock aufgehoben',true);setTimeout(function(){location.reload();},1000);});}
function manualSync(){if(!confirm('Kanal-Sync starten? Ca. 30s.'))return;toast('Suche...',true);fetch('/api/channel-sync',{method:'POST'}).then(function(r){return r.json();}).then(function(d){if(d.peers>0){toast('Peers auf Kanal '+d.channel,true);setTimeout(function(){location.reload();},1500);}else toast('Kein passender AP',false);});}

function updateStatus(d){
  var set=function(id,v){var el=document.getElementById(id); if(el) el.textContent=v;};
  set('s-ch',d.channel||'?'); set('s-peers',d.peers||0); set('s-recv',d.recv||0); set('s-sent',d.sent||0);
  set('s-unread',d.unread||0); set('s-rssi',(d.rssi||0)+' dBm'); set('s-ip',d.ip||'');
  var up=d.uptime||0;
  set('s-uptime', up<60?up+'s':up<3600?Math.floor(up/60)+'min':Math.floor(up/3600)+'h');
  var en=document.getElementById('s-espnow');
  if(en) en.innerHTML=d.espNow?'<span class="ok">Aktiv</span>':'Offline';
  updateStatusBadge();
}
function updateStatusBadge(){}
function renderLogList(evts){
  logs=evts||[];
  var el=document.getElementById('log-list');
  if(!logs.length){el.innerHTML='<div class="muted">Keine Ereignisse</div>';return;}
  el.innerHTML=logs.map(function(e){
    return '<div class="logline"><span>'+esc(e.msg)+'</span><span class="muted">'+ageStr(e.age)+'</span></div>';
  }).join('');
}

// WebRTC
var callMac=null, callName='', callRole=null, callTimer=null, callSeconds=0;
var pc=null, localStream=null, rtcPoll=null, rtcAfter=0, camOn=true, micOn=true;
var pendingIce=[], remoteStream=null, ringAudioCtx=null, ringOsc=null;
function hubUrl(p){return HUB_BASE+p;}
function setRtcStatus(t){ var el=document.getElementById('rtc-status'); if(el) el.textContent=t||''; }
function setRtcWarn(t){
  var el=document.getElementById('rtc-warn'); if(!el) return;
  if(!t){ el.style.display='none'; el.textContent=''; return; }
  el.style.display='block'; el.textContent=t;
}
function showSecureHint(){
  var insecure=!window.isSecureContext && location.hostname!=='localhost' && location.hostname!=='127.0.0.1';
  if(insecure){
    setRtcWarn('Hinweis: http:// im LAN blockiert oft Mikro/Kamera. In Chrome: chrome://flags → „Insecure origins treated as secure“ → http://'+location.host+' hinzufuegen, dann Chrome neu starten.');
  } else setRtcWarn('');
}
function rtcPost(to,type,payload){
  if(!HUB_BASE||!MY_MAC) return Promise.reject(new Error('kein Hub'));
  return fetch(hubUrl('/api/webrtc'),{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({from:MY_MAC,to:normMac(to),type:type,payload:payload||null})});
}
function startRtcPoll(){
  stopRtcPoll(); if(!HUB_BASE||!MY_MAC) return;
  rtcPoll=setInterval(async function(){
    try{
      var r=await fetch(hubUrl('/api/webrtc?mac='+encodeURIComponent(MY_MAC)+'&after='+rtcAfter));
      var d=await r.json(); if(!d.ok||!d.signals) return;
      for(var i=0;i<d.signals.length;i++){ var s=d.signals[i]; if(s.id>rtcAfter) rtcAfter=s.id; await handleRtcSignal(s); }
    }catch(e){}
  },500);
}
function stopRtcPoll(){if(rtcPoll){clearInterval(rtcPoll);rtcPoll=null;}}
function stopRingTone(){
  try{ if(ringOsc){ ringOsc.stop(); ringOsc.disconnect(); } }catch(e){}
  ringOsc=null;
  try{ if(ringAudioCtx) ringAudioCtx.close(); }catch(e){}
  ringAudioCtx=null;
}
function startRingTone(){
  stopRingTone();
  try{
    ringAudioCtx=new (window.AudioContext||window.webkitAudioContext)();
    var o=ringAudioCtx.createOscillator();
    var g=ringAudioCtx.createGain();
    o.type='sine'; o.frequency.value=880;
    g.gain.value=0.04;
    o.connect(g); g.connect(ringAudioCtx.destination);
    o.start();
    ringOsc=o;
    var pulse=setInterval(function(){
      if(!ringOsc||!ringAudioCtx) { clearInterval(pulse); return; }
      g.gain.value = g.gain.value>0.01 ? 0.0 : 0.04;
    },400);
    ringOsc._pulse=pulse;
  }catch(e){}
}
async function ensureMedia(){
  if(localStream) return localStream;
  if(!navigator.mediaDevices||!navigator.mediaDevices.getUserMedia){
    throw new Error('getUserMedia nicht verfuegbar (Secure Context?)');
  }
  try{
    localStream=await navigator.mediaDevices.getUserMedia({
      audio:{echoCancellation:true,noiseSuppression:true,autoGainControl:true},
      video:{facingMode:'user',width:{ideal:320},height:{ideal:240}}
    });
  }catch(e1){
    localStream=await navigator.mediaDevices.getUserMedia({
      audio:{echoCancellation:true,noiseSuppression:true},
      video:false
    });
    toast('Nur Audio — Kamera nicht verfuegbar',true);
  }
  var lv=document.getElementById('rtc-local');
  if(lv){ lv.srcObject=localStream; lv.muted=true; lv.play().catch(function(){}); }
  return localStream;
}
function attachRemote(stream){
  remoteStream=stream;
  var rv=document.getElementById('rtc-remote');
  var ra=document.getElementById('rtc-remote-audio');
  if(rv){
    rv.srcObject=stream;
    rv.muted=false;
    rv.volume=1;
    rv.play().catch(function(){});
  }
  if(ra){
    ra.srcObject=stream;
    ra.muted=false;
    ra.volume=1;
    ra.play().catch(function(){});
  }
  setRtcStatus('Medien empfangen');
}
async function flushIce(){
  if(!pc) return;
  while(pendingIce.length){
    var c=pendingIce.shift();
    try{ await pc.addIceCandidate(c); }catch(e){}
  }
}
function makePc(peerMac){
  if(pc){try{pc.close();}catch(e){}}
  pendingIce=[];
  pc=new RTCPeerConnection({
    iceServers:[
      {urls:'stun:stun.l.google.com:19302'},
      {urls:'stun:stun1.l.google.com:19302'}
    ]
  });
  pc.onicecandidate=function(ev){
    if(ev.candidate) rtcPost(peerMac,'ice',ev.candidate.toJSON?ev.candidate.toJSON():ev.candidate).catch(function(){});
  };
  pc.ontrack=function(ev){
    if(ev.streams&&ev.streams[0]) attachRemote(ev.streams[0]);
    else {
      if(!remoteStream) remoteStream=new MediaStream();
      remoteStream.addTrack(ev.track);
      attachRemote(remoteStream);
    }
  };
  pc.onconnectionstatechange=function(){
    setRtcStatus('Verbindung: '+pc.connectionState);
    if(pc.connectionState==='connected') toast('Anruf verbunden',true);
  };
  pc.oniceconnectionstatechange=function(){
    setRtcStatus('ICE: '+pc.iceConnectionState);
  };
  return pc;
}
async function handleRtcSignal(s){
  if(!callMac||normMac(s.from)!==normMac(callMac)) return;
  if(s.type==='offer'&&callRole==='callee'){
    try{
      setRtcStatus('Offer empfangen...');
      await ensureMedia();
      makePc(callMac);
      localStream.getTracks().forEach(function(t){pc.addTrack(t,localStream);});
      await pc.setRemoteDescription(new RTCSessionDescription(s.payload));
      await flushIce();
      var ans=await pc.createAnswer();
      await pc.setLocalDescription(ans);
      await rtcPost(callMac,'answer',{type:ans.type,sdp:ans.sdp});
      setRtcStatus('Answer gesendet');
    }catch(e){ setRtcStatus('Fehler: '+e.message); toast('WebRTC: '+e.message,false); showSecureHint(); }
  } else if(s.type==='answer'&&callRole==='caller'){
    try{
      if(pc){
        await pc.setRemoteDescription(new RTCSessionDescription(s.payload));
        await flushIce();
        setRtcStatus('Answer empfangen');
      }
    }catch(e){ setRtcStatus('Answer-Fehler: '+e.message); }
  } else if(s.type==='ice'){
    if(!s.payload) return;
    if(!pc || !pc.remoteDescription){ pendingIce.push(s.payload); return; }
    try{ await pc.addIceCandidate(s.payload); }catch(e){}
  } else if(s.type==='bye'){ cleanupRtc(false); toast('Anruf beendet',false); }
}
async function beginRtcAsCaller(){
  callRole='caller'; startRtcPoll(); showSecureHint();
  try{
    setRtcStatus('Mikro/Kamera...');
    await ensureMedia();
    makePc(callMac);
    localStream.getTracks().forEach(function(t){pc.addTrack(t,localStream);});
    setRtcStatus('Erstelle Offer...');
    var offer=await pc.createOffer({offerToReceiveAudio:true,offerToReceiveVideo:true});
    await pc.setLocalDescription(offer);
    await rtcPost(callMac,'offer',{type:offer.type,sdp:offer.sdp});
    setRtcStatus('Warte auf Answer...');
  }catch(e){
    setRtcStatus('Medienfehler');
    toast('Kamera/Mikro: '+e.message,false);
    showSecureHint();
  }
}
async function beginRtcAsCallee(){
  callRole='callee'; startRtcPoll(); showSecureHint();
  try{
    setRtcStatus('Mikro/Kamera...');
    await ensureMedia();
    setRtcStatus('Warte auf Offer...');
  }catch(e){
    setRtcStatus('Medienfehler');
    toast('Kamera/Mikro: '+e.message,false);
    showSecureHint();
  }
}
function cleanupRtc(sendBye){
  stopRingTone();
  document.getElementById('call-modal').classList.remove('show');
  document.getElementById('active-call-modal').classList.remove('show');
  if(callTimer){clearInterval(callTimer);callTimer=null;}
  stopRtcPoll();
  if(sendBye&&callMac) rtcPost(callMac,'bye',null).catch(function(){});
  if(pc){try{pc.close();}catch(e){} pc=null;}
  if(localStream){localStream.getTracks().forEach(function(t){t.stop();});localStream=null;}
  remoteStream=null; pendingIce=[];
  var lv=document.getElementById('rtc-local'), rv=document.getElementById('rtc-remote'), ra=document.getElementById('rtc-remote-audio');
  if(lv) lv.srcObject=null; if(rv) rv.srcObject=null; if(ra) ra.srcObject=null;
  callMac=null; callName=''; callRole=null; callSeconds=0; camOn=true; micOn=true;
  setRtcStatus(''); setRtcWarn('');
}
function callCurrent(){ if(activePeer && activePeer!=='bc') startCall(activePeer, activeName); }
function startCall(mac,name){
  if(callMac){toast('Bereits im Gespraech',false);return;}
  if(!HUB_BASE){toast('Kein Hub konfiguriert',false);return;}
  callMac=mac; callName=name||mac;
  document.getElementById('active-call-name').textContent='Rufe an: '+callName+'...';
  document.getElementById('active-call-duration').textContent='Klingelt...';
  setRtcStatus('Klingelt...'); showSecureHint();
  document.getElementById('active-call-modal').classList.add('show');
  startRtcPoll();
  fetch('/api/call',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({to:mac,action:'ring'})}).catch(function(){toast('Fehler',false);});
}
function acceptCall(){
  stopRingTone();
  document.getElementById('call-modal').classList.remove('show');
  var mac=document.getElementById('call-modal').dataset.mac;
  var name=document.getElementById('call-from').textContent;
  callMac=mac; callName=name; callSeconds=0;
  document.getElementById('active-call-name').textContent='Verbunden: '+name;
  startCallTimer(); document.getElementById('active-call-modal').classList.add('show');
  fetch('/api/call',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({to:mac,action:'accept'})}).catch(function(){});
  beginRtcAsCallee();
}
function rejectCall(){
  stopRingTone();
  var mac=document.getElementById('call-modal').dataset.mac;
  document.getElementById('call-modal').classList.remove('show');
  fetch('/api/call',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({to:mac,action:'reject'})}).catch(function(){});
  callMac=null;
}
function endCall(){
  if(callMac){
    fetch('/api/call',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({to:callMac,action:'end'})}).catch(function(){});
  }
  cleanupRtc(true);
}
function toggleCam(){ if(!localStream)return; camOn=!camOn; localStream.getVideoTracks().forEach(function(t){t.enabled=camOn;}); document.getElementById('rtc-cam-btn').textContent=camOn?'Kamera':'Kamera aus'; }
function toggleMic(){ if(!localStream)return; micOn=!micOn; localStream.getAudioTracks().forEach(function(t){t.enabled=micOn;}); document.getElementById('rtc-mic-btn').textContent=micOn?'Mikro':'Mikro aus'; }
function startCallTimer(){
  callSeconds=0; if(callTimer) clearInterval(callTimer);
  callTimer=setInterval(function(){
    callSeconds++; var m=Math.floor(callSeconds/60), s=callSeconds%60;
    document.getElementById('active-call-duration').textContent=(m<10?'0':'')+m+':'+(s<10?'0':'')+s;
  },1000);
}
function handleCallEvent(data){
  var d=JSON.parse(data);
  if(d.type==='ring'){
    var modal=document.getElementById('call-modal');
    modal.dataset.mac=d.mac;
    document.getElementById('call-title').textContent='Eingehender Anruf';
    document.getElementById('call-from').textContent=d.from;
    modal.classList.add('show');
    startRingTone();
  } else if(d.type==='accept'){
    document.getElementById('active-call-name').textContent='Verbunden mit '+d.from;
    startCallTimer(); beginRtcAsCaller();
  } else if(d.type==='end'||d.type==='timeout'||d.type==='reject'){
    cleanupRtc(false);
    toast(d.type==='timeout'?'Kein Annehmen':(d.type==='reject'?'Abgelehnt':'Anruf beendet'),false);
  }
}

var sse;
function connectSse(){
  sse=new EventSource('/api/sse');
  sse.addEventListener('messages',function(e){ try{msgs=JSON.parse(e.data); renderList(); renderThread();}catch(ex){} fetch('/api/status').then(function(r){return r.json();}).then(updateStatus); });
  sse.addEventListener('peers',function(e){ try{peers=JSON.parse(e.data); renderList();
    if(activePeer&&activePeer!=='bc'){
      var peer=peers.find(function(p){return sameMac(p.mac,activePeer);});
      var on=peer&&peer.online;
      document.getElementById('chat-sub').innerHTML=
        '<span class="st '+(on?'on':'off')+'">'+(on?'Online':'Offline')+'</span> · <span class="macs">'+esc(activePeer)+'</span>';
    }
  }catch(ex){} });
  sse.addEventListener('status',function(e){ try{updateStatus(JSON.parse(e.data));}catch(ex){} });
  sse.addEventListener('log',function(e){ try{renderLogList(JSON.parse(e.data));}catch(ex){} });
  sse.addEventListener('call',function(e){ try{handleCallEvent(e.data);}catch(ex){} });
  sse.onerror=function(){ setTimeout(connectSse,3000); };
}

syncViews();
connectSse();
refreshPinUi();
fetch('/api/messages').then(function(r){return r.json();}).then(function(m){msgs=m||[]; renderList(); renderThread();});
fetch('/api/peers').then(function(r){return r.json();}).then(function(p){peers=p||[]; renderList();});
fetch('/api/events').then(function(r){return r.json();}).then(renderLogList);
fetch('/api/status').then(function(r){return r.json();}).then(updateStatus);
setInterval(function(){
  if(sse&&sse.readyState===1) return;
  fetch('/api/messages').then(function(r){return r.json();}).then(function(m){msgs=m||[]; renderList(); renderThread();});
  fetch('/api/status').then(function(r){return r.json();}).then(updateStatus);
},5000);
setInterval(function(){ fetch('/api/status').then(function(r){return r.json();}).then(updateStatus); },10000);
</script>
</div>
</body></html>
)rawjs");
}

void handleOtaPage() {
    if (pinEnabled() && !sessionValid()) { handleLoginPage(); return; }
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
