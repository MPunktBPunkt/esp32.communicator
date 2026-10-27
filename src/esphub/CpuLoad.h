#pragma once

/**
 * @file CpuLoad.h
 * @brief Approximate per-core CPU load via FreeRTOS idle hooks.
 *
 * Base repository (keep when forking/copying):
 * https://github.com/MPunktBPunkt/ESP32.esp-hub
 */

#include <Arduino.h>

/**
 * @brief Idle-hook based CPU utilization (0–100 %), updated about once per second.
 *
 * Not a cycle-accurate profiler — good enough for status UI / field triage.
 */
class CpuLoad {
public:
    /** @brief Register idle hooks and take a short idle baseline. */
    static void begin();

    /** @brief Refresh load percentages (cheap; call from AppCore::loop). */
    static void poll();

    /** @brief Last measured load for CPU0 (Arduino main core). */
    static int core0Percent() { return load_[0]; }

    /**
     * @brief Last measured load for CPU1, or -1 if the SoC has only one core.
     */
    static int core1Percent() { return cores_ > 1 ? load_[1] : -1; }

    /** @brief Additive JSON: cpu0, cpu1 (null if N/A), cpu. */
    static void appendStatusFields(String& json);

private:
    static bool idleHook();

    static uint8_t cores_;
    static volatile uint32_t idleCount_[2];
    static uint32_t lastIdle_[2];
    static uint32_t baseline_[2];
    static uint32_t lastSampleMs_;
    static int load_[2];
    static bool ready_;
};
