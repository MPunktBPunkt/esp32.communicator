#pragma once

/**
 * @file HardwareInfo.h
 * @brief Probed board capabilities shared by status and heartbeat.
 *
 * Base repository (keep when forking/copying):
 * https://github.com/MPunktBPunkt/ESP32.esp-hub
 */

#include <Arduino.h>

/**
 * @brief Probed board capabilities (D-017, D-019); one AppCore-owned source of truth.
 *
 * Static traits are captured in probe(); free heap/PSRAM/sketch stay live getters.
 * On ESP32-S3, eFuse BLK1 + flash JEDEC yield the same module guess as hu-probe / esptool.
 */
class HardwareInfo {
public:
    /** @brief Capture chip/flash/PSRAM traits (safe once per boot). */
    void probe();

    void logToSerial() const;

    bool probed() const { return probed_; }

    const String& chipModel() const { return chipModel_; }
    uint8_t chipRevision() const { return chipRevision_; }
    uint8_t chipCores() const { return chipCores_; }
    uint32_t flashSizeKb() const { return flashSizeKb_; }
    uint32_t flashSizeBytes() const { return flashSizeKb_ * 1024U; }

    /** @brief Guessed module tag e.g. "N8R2", or empty if unknown. */
    const char* moduleGuess() const { return module_; }

    /** @brief Physical flash bytes (JEDEC/eFuse); 0 if unknown. */
    uint32_t flashBytesReal() const { return flashBytesReal_; }
    /** @brief In-package PSRAM bytes from eFuse (S3); 0 if none/unknown. */
    uint32_t psramBytesEfuse() const { return psramBytesEfuse_; }

    // Total PSRAM reported by IDF/Arduino (0 if module missing or build without QSPI).
    size_t psramSize() const { return psramSize_; }
    bool hasPsram() const { return psramSize_ > 0; }

    // Live runtime counters — always read from ESP APIs.
    size_t freePsram() const;
    uint32_t freeHeap() const;
    uint32_t maxAllocHeap() const;
    uint32_t freeSketchSpace() const;
    /** @brief Bytes occupied by the running firmware image. */
    uint32_t sketchSize() const;

    /**
     * @brief Human-readable one-liner (hu-probe style), with live heap figures.
     * @return e.g. "ESP32-S3 rev 0 · Modul N8R2 · Flash 8 MiB …"
     */
    String summaryLine() const;

    /** @brief Additive status fields including hwSummary (baseline keys unchanged). */
    void appendStatusFields(String& json) const;

private:
    void probeEfuseAndFlash();

    bool probed_ = false;
    String chipModel_;
    uint8_t chipRevision_ = 0;
    uint8_t chipCores_ = 0;
    uint32_t flashSizeKb_ = 0;
    size_t psramSize_ = 0;

    char module_[16] = {0};
    uint32_t flashBytesReal_ = 0;
    uint32_t psramBytesEfuse_ = 0;
    uint32_t jedecId_ = 0;
    bool jedecOk_ = false;
    char flashVendor_[16] = {0};
    char psramBus_[8] = {0};
    char psramVendor_[12] = {0};
    char package_[20] = {0};
};
