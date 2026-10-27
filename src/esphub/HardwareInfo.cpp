/**
 * @file HardwareInfo.cpp
 * @brief Hardware capability probe, S3 eFuse/JEDEC decode, status helpers.
 *
 * Base repository (keep when forking/copying):
 * https://github.com/MPunktBPunkt/ESP32.esp-hub
 */

#include "HardwareInfo.h"

#include <esp_flash.h>
#include <cstdio>
#include <cstring>

#if CONFIG_IDF_TARGET_ESP32S3
#include <soc/soc.h>
#endif

namespace {

#if CONFIG_IDF_TARGET_ESP32S3
uint32_t efuseBlk1Word(uint8_t n) {
    return REG_READ(DR_REG_EFUSE_BASE + 0x44 + 4u * n);
}
#endif

uint32_t psramCapBytes(uint8_t c) {
    switch (c) {
        case 1: return 8u << 20;
        case 2: return 2u << 20;
        case 3: return 16u << 20;
        case 4: return 4u << 20;
        default: return 0;
    }
}

uint32_t flashCapBytes(uint8_t c) {
    switch (c) {
        case 1: return 8u << 20;
        case 2: return 4u << 20;
        default: return 0;
    }
}

const char* psramBusName(uint8_t c) {
    switch (c) {
        case 2: return "quad";
        case 1:
        case 3: return "octal";
        default: return "";
    }
}

const char* psramVendorName(uint8_t v) {
    switch (v) {
        case 1: return "AP_3v3";
        case 2: return "AP_1v8";
        default: return "";
    }
}

const char* flashVendorEfuseName(uint8_t v) {
    switch (v) {
        case 1: return "XMC";
        case 2: return "GD";
        case 3: return "FM";
        case 4: return "TT";
        case 5: return "BY";
        default: return "";
    }
}

const char* jedecVendorName(uint8_t m) {
    switch (m) {
        case 0xC8: return "GigaDevice";
        case 0xEF: return "Winbond";
        case 0x20: return "XMC/Micron";
        case 0x68: return "Boya";
        case 0x5E: return "Zbit";
        case 0xC2: return "Macronix";
        case 0x1C: return "EON";
        case 0x0B: return "XTX";
        case 0x85: return "Puya";
        default: return "";
    }
}

const char* pkgName(uint8_t p) {
    switch (p) {
        case 0: return "QFN56";
        case 1: return "PICO-1 LGA56";
        default: return "";
    }
}

}  // namespace

void HardwareInfo::probeEfuseAndFlash() {
    module_[0] = 0;
    flashBytesReal_ = 0;
    psramBytesEfuse_ = 0;
    jedecId_ = 0;
    jedecOk_ = false;
    flashVendor_[0] = 0;
    psramBus_[0] = 0;
    psramVendor_[0] = 0;
    package_[0] = 0;

#if CONFIG_IDF_TARGET_ESP32S3
    const uint32_t w3 = efuseBlk1Word(3);
    const uint32_t w4 = efuseBlk1Word(4);
    const uint32_t w5 = efuseBlk1Word(5);
    const uint8_t pkg = (w3 >> 21) & 0x07;
    const uint8_t flashCap = (w3 >> 27) & 0x07;
    const uint8_t flashVendor = w4 & 0x07;
    const uint8_t psramCap =
        (uint8_t)((((w5 >> 19) & 0x01) << 2) | ((w4 >> 3) & 0x03));
    const uint8_t psramVendor = (w4 >> 7) & 0x03;

    strncpy(package_, pkgName(pkg), sizeof(package_) - 1);
    strncpy(psramBus_, psramBusName(psramCap), sizeof(psramBus_) - 1);
    strncpy(psramVendor_, psramVendorName(psramVendor), sizeof(psramVendor_) - 1);
    psramBytesEfuse_ = psramCapBytes(psramCap);

    uint32_t id = 0;
    jedecOk_ = esp_flash_read_id(nullptr, &id) == ESP_OK && id != 0 && id != 0xFFFFFF;
    jedecId_ = id;
    const uint8_t cap = id & 0xFF;
    if (jedecOk_ && cap >= 0x10 && cap <= 0x1A) {
        flashBytesReal_ = 1u << cap;
    } else {
        flashBytesReal_ = flashCapBytes(flashCap);
    }

    if (jedecOk_) {
        strncpy(flashVendor_, jedecVendorName((id >> 16) & 0xFF), sizeof(flashVendor_) - 1);
    } else if (flashCap) {
        strncpy(flashVendor_, flashVendorEfuseName(flashVendor), sizeof(flashVendor_) - 1);
    }

    if (flashBytesReal_) {
        if (psramBytesEfuse_) {
            snprintf(module_, sizeof(module_), "N%uR%u",
                     (unsigned)(flashBytesReal_ >> 20),
                     (unsigned)(psramBytesEfuse_ >> 20));
        } else {
            snprintf(module_, sizeof(module_), "N%u",
                     (unsigned)(flashBytesReal_ >> 20));
        }
    }
#else
    // Classic ESP32 / other targets: no S3 BLK1 layout — use configured flash only.
    flashBytesReal_ = flashSizeKb_ * 1024U;
    if (flashBytesReal_) {
        snprintf(module_, sizeof(module_), "N%u", (unsigned)(flashBytesReal_ >> 20));
    }
    uint32_t id = 0;
    jedecOk_ = esp_flash_read_id(nullptr, &id) == ESP_OK && id != 0 && id != 0xFFFFFF;
    jedecId_ = id;
    if (jedecOk_) {
        strncpy(flashVendor_, jedecVendorName((id >> 16) & 0xFF), sizeof(flashVendor_) - 1);
        const uint8_t cap = id & 0xFF;
        if (cap >= 0x10 && cap <= 0x1A) {
            flashBytesReal_ = 1u << cap;
            snprintf(module_, sizeof(module_), "N%u", (unsigned)(flashBytesReal_ >> 20));
        }
    }
#endif
}

void HardwareInfo::probe() {
    chipModel_ = String(ESP.getChipModel());
    chipRevision_ = ESP.getChipRevision();
    chipCores_ = ESP.getChipCores();
    flashSizeKb_ = ESP.getFlashChipSize() / 1024U;
    // Never assume PSRAM — measure only (D-019).
    psramSize_ = ESP.getPsramSize();
    probeEfuseAndFlash();
    probed_ = true;
}

String HardwareInfo::summaryLine() const {
    // ESP32-S3 rev 0 · Modul N8R2 · Flash 8 MiB (XMC/Micron, genutzt 8) ·
    // PSRAM 2 MiB quad AP_3v3 (aktiv) · Heap frei 173 KiB, größter Block 84 KiB
    const unsigned flashRealMb = flashBytesReal_ ? (unsigned)(flashBytesReal_ >> 20) : 0;
    const unsigned flashCfgMb = flashSizeKb_ / 1024U;
    const unsigned psramEfuseMb =
        psramBytesEfuse_ ? (unsigned)(psramBytesEfuse_ >> 20) : 0;
    const unsigned heapKiB = freeHeap() / 1024U;
    const unsigned maxKiB = maxAllocHeap() / 1024U;

    String out = chipModel_;
    out += " rev ";
    out += String(chipRevision_);

    if (module_[0]) {
        out += " · Modul ";
        out += module_;
    }

    out += " · Flash ";
    if (flashRealMb) {
        out += String(flashRealMb);
        out += " MiB";
        if (flashVendor_[0] || flashCfgMb) {
            out += " (";
            if (flashVendor_[0]) out += flashVendor_;
            if (flashVendor_[0] && flashCfgMb) out += ", ";
            if (flashCfgMb) {
                out += "genutzt ";
                out += String(flashCfgMb);
            }
            out += ")";
        }
    } else {
        out += String(flashCfgMb);
        out += " MiB (genutzt)";
    }

    if (psramEfuseMb || hasPsram()) {
        out += " · PSRAM ";
        out += String(psramEfuseMb ? psramEfuseMb : (unsigned)(psramSize_ >> 20));
        out += " MiB";
        if (psramBus_[0]) {
            out += " ";
            out += psramBus_;
        }
        if (psramVendor_[0]) {
            out += " ";
            out += psramVendor_;
        }
        out += hasPsram() ? " (aktiv)" : " (inaktiv)";
    } else {
        out += " · PSRAM 0";
    }

    out += " · Heap frei ";
    out += String(heapKiB);
    out += " KiB, gr\u00f6\u00dfter Block ";
    out += String(maxKiB);
    out += " KiB";
    return out;
}

void HardwareInfo::logToSerial() const {
    Serial.printf("[HW] Chip=%s Rev%d Cores=%d Flash=%u KB\n",
                  chipModel_.c_str(), chipRevision_,
                  chipCores_, flashSizeKb_);
    if (module_[0] || flashBytesReal_ || psramBytesEfuse_) {
        Serial.printf("[HW] module=%s flash_real=%u MiB jedec=0x%06x vendor=%s "
                      "psram_efuse=%u MiB %s %s pkg=%s\n",
                      module_[0] ? module_ : "?",
                      (unsigned)(flashBytesReal_ >> 20),
                      (unsigned)jedecId_,
                      flashVendor_[0] ? flashVendor_ : "-",
                      (unsigned)(psramBytesEfuse_ >> 20),
                      psramBus_[0] ? psramBus_ : "-",
                      psramVendor_[0] ? psramVendor_ : "-",
                      package_[0] ? package_ : "-");
    }
    if (hasPsram()) {
        Serial.printf("[HW] PSRAM=%u KB frei=%u KB\n",
                      (unsigned)(psramSize_ / 1024U),
                      (unsigned)(freePsram() / 1024U));
    } else {
        Serial.println("[HW] PSRAM=0 (Modul fehlt oder Build ohne QSPI PSRAM)");
    }
    Serial.printf("[HW] %s\n", summaryLine().c_str());
}

size_t HardwareInfo::freePsram() const { return ESP.getFreePsram(); }
uint32_t HardwareInfo::freeHeap() const { return ESP.getFreeHeap(); }
uint32_t HardwareInfo::maxAllocHeap() const { return ESP.getMaxAllocHeap(); }
uint32_t HardwareInfo::freeSketchSpace() const { return ESP.getFreeSketchSpace(); }
uint32_t HardwareInfo::sketchSize() const { return ESP.getSketchSize(); }

void HardwareInfo::appendStatusFields(String& json) const {
    // Additive keys only — do not rename baseline fields.
    json += ",\"chipRev\":" + String(chipRevision_);
    json += ",\"cores\":" + String(chipCores_);
    json += ",\"flashKb\":" + String(flashSizeKb_);
    json += ",\"hasPsram\":";
    json += hasPsram() ? "true" : "false";
    json += ",\"heapMax\":" + String(maxAllocHeap());
    json += ",\"sketchUsed\":" + String(sketchSize());
    // OTA free (baseline key `sketch`) is the other app slot — often same size on
    // min_spiffs for 4 MiB and 8 MiB; flashKb/flashRealMb show the real chip size.
    if (module_[0]) {
        json += ",\"module\":\"";
        json += module_;
        json += "\"";
    } else {
        json += ",\"module\":null";
    }
    json += ",\"flashRealMb\":" + String((unsigned)(flashBytesReal_ >> 20));
    json += ",\"flashVendor\":";
    if (flashVendor_[0]) {
        json += "\"";
        json += flashVendor_;
        json += "\"";
    } else {
        json += "null";
    }
    json += ",\"psramEfuseMb\":" + String((unsigned)(psramBytesEfuse_ >> 20));
    json += ",\"psramBus\":";
    if (psramBus_[0]) {
        json += "\"";
        json += psramBus_;
        json += "\"";
    } else {
        json += "null";
    }
    json += ",\"psramVendor\":";
    if (psramVendor_[0]) {
        json += "\"";
        json += psramVendor_;
        json += "\"";
    } else {
        json += "null";
    }
    // Escape is unnecessary for our ASCII summary; keep JSON string simple.
    String sum = summaryLine();
    sum.replace("\\", "\\\\");
    sum.replace("\"", "\\\"");
    json += ",\"hwSummary\":\"";
    json += sum;
    json += "\"";
}
