#pragma once

#include <zephyr/kernel.h>
#include <cstddef>
#include <cstdint>

namespace ThreadConfig {
    constexpr size_t StackSmall = 512;
    constexpr size_t StackLarge = 1024;
    constexpr int PrioProducer  = 6;
    constexpr int PrioProcessor = 5;
    constexpr int PrioLogger    = 4;
}

static_assert(ThreadConfig::PrioProcessor < ThreadConfig::PrioProducer, "Processor must have higher priority than Producer to prevent starvation");

enum class ProducerState : uint8_t {
    ReadBME,
    ReadLPS,
    ReadPAV
};

void producer_thread(void);
void processor_thread(void);
void logger_thread(void);

extern const k_tid_t producer_tid;
extern const k_tid_t processor_tid;
extern const k_tid_t logger_tid;
