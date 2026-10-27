/**
 * @file CpuLoad.cpp
 * @brief FreeRTOS idle-hook CPU load estimation.
 *
 * Base repository (keep when forking/copying):
 * https://github.com/MPunktBPunkt/ESP32.esp-hub
 */

#include "CpuLoad.h"

#include <esp_freertos_hooks.h>
#include <esp_system.h>

uint8_t CpuLoad::cores_ = 1;
volatile uint32_t CpuLoad::idleCount_[2] = {0, 0};
uint32_t CpuLoad::lastIdle_[2] = {0, 0};
uint32_t CpuLoad::baseline_[2] = {0, 0};
uint32_t CpuLoad::lastSampleMs_ = 0;
int CpuLoad::load_[2] = {0, 0};
bool CpuLoad::ready_ = false;

bool CpuLoad::idleHook() {
    const BaseType_t core = xPortGetCoreID();
    if (core >= 0 && core < 2) {
        idleCount_[core]++;
    }
    return true;
}

void CpuLoad::begin() {
    cores_ = (uint8_t)ESP.getChipCores();
    if (cores_ < 1) cores_ = 1;
    if (cores_ > 2) cores_ = 2;

    esp_register_freertos_idle_hook_for_cpu(idleHook, 0);
    if (cores_ > 1) {
        esp_register_freertos_idle_hook_for_cpu(idleHook, 1);
    }

    // Baseline: idle counts over 100 ms while mostly idle (boot path).
    const uint32_t t0 = millis();
    const uint32_t c0 = idleCount_[0];
    const uint32_t c1 = idleCount_[1];
    delay(100);
    const uint32_t dt = millis() - t0;
    const uint32_t d0 = idleCount_[0] - c0;
    const uint32_t d1 = idleCount_[1] - c1;
    // Scale baseline to 1 second.
    baseline_[0] = dt > 0 ? (d0 * 1000UL) / dt : d0 * 10UL;
    baseline_[1] = dt > 0 ? (d1 * 1000UL) / dt : d1 * 10UL;
    if (baseline_[0] < 100) baseline_[0] = 100;
    if (baseline_[1] < 100) baseline_[1] = 100;

    lastIdle_[0] = idleCount_[0];
    lastIdle_[1] = idleCount_[1];
    lastSampleMs_ = millis();
    ready_ = true;
    Serial.printf("[CPU] idle baseline/s core0=%lu core1=%lu cores=%u\n",
                  (unsigned long)baseline_[0], (unsigned long)baseline_[1], cores_);
}

void CpuLoad::poll() {
    if (!ready_) return;
    const uint32_t now = millis();
    if (now - lastSampleMs_ < 1000UL) return;
    const uint32_t elapsed = now - lastSampleMs_;
    lastSampleMs_ = now;

    for (uint8_t i = 0; i < cores_; i++) {
        const uint32_t idle = idleCount_[i];
        const uint32_t delta = idle - lastIdle_[i];
        lastIdle_[i] = idle;
        // Expected idle ticks for this window at "empty" baseline.
        const uint32_t expect = (baseline_[i] * elapsed) / 1000UL;
        int load = 0;
        if (expect > 0) {
            if (delta >= expect) {
                load = 0;
            } else {
                load = (int)(((expect - delta) * 100UL) / expect);
            }
        }
        if (load < 0) load = 0;
        if (load > 100) load = 100;
        load_[i] = load;
    }
}

void CpuLoad::appendStatusFields(String& json) {
    json += ",\"cpu0\":";
    json += String(load_[0]);
    if (cores_ > 1) {
        json += ",\"cpu1\":";
        json += String(load_[1]);
        json += ",\"cpu\":";
        json += String((load_[0] + load_[1]) / 2);
    } else {
        json += ",\"cpu1\":null";
        json += ",\"cpu\":";
        json += String(load_[0]);
    }
}
