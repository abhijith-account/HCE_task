#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
#include "Threads.h"
#include "Thread_Common.h"

LOG_MODULE_DECLARE(COMMANDS, LOG_LEVEL_INF);

#define TRACE_BUFFER_SIZE 4096
#define TRACE_BUFFER_MASK (TRACE_BUFFER_SIZE - 1)

enum class TraceType : uint8_t {
    SWITCH_IN,
    SWITCH_OUT
};

struct TraceEvent {
    uint32_t timestamp;
    struct k_thread* thread;
    TraceType type;
};

static TraceEvent g_trace_buffer[TRACE_BUFFER_SIZE];
static volatile uint32_t g_trace_head = 0;
static uint32_t g_trace_tail = 0;
atomic_t g_switch_hook_hits = ATOMIC_INIT(0);

extern "C" {
    enum class TraceThread : uint8_t {
    PRODUCER = 0,
    PROCESSOR,
    LOGGER,
    INVALID
    };

    static TraceThread get_target_thread(struct k_thread *thread) {
        if (thread == producer_tid)  return TraceThread::PRODUCER;
        if (thread == processor_tid) return TraceThread::PROCESSOR;
        if (thread == logger_tid)    return TraceThread::LOGGER;
        return TraceThread::INVALID;
    }
    static inline void recordTraceEventLocked(struct k_thread* thread, TraceType type) noexcept {
        /* Caller already holds irq_lock() - do not lock again here. */
        if (get_target_thread(thread) == TraceThread::INVALID) {
            return;
        }

        const uint32_t slot = g_trace_head & TRACE_BUFFER_MASK;
        
        g_trace_buffer[slot].timestamp = k_cycle_get_32();
        g_trace_buffer[slot].thread = thread;
        g_trace_buffer[slot].type = type;
        
        g_trace_head = g_trace_head + 1;
    }

    void sys_trace_thread_switched_in_user(void) {
        unsigned int key = irq_lock();
        struct k_thread *thread = k_sched_current_thread_query();
        
        // Replaced get_target_thread_index with get_target_thread
        if (get_target_thread(thread) != TraceThread::INVALID) {
            atomic_inc(&g_switch_hook_hits);
            recordTraceEventLocked(thread, TraceType::SWITCH_IN);
        }
        irq_unlock(key);
    }
    
    void sys_trace_thread_switched_out_user(void) {
        unsigned int key = irq_lock();
        struct k_thread *thread = k_sched_current_thread_query();
        recordTraceEventLocked(thread, TraceType::SWITCH_OUT);
        irq_unlock(key);
    }
}
extern "C" void trace_logger_thread(void *arg1, void *arg2, void *arg3) {
    
    // Array to track the last SWITCH_OUT timestamp for PRODUCER, PROCESSOR, LOGGER
    uint32_t thread_out_times[3] = {0, 0, 0};

    while (true) {
        while (g_powerObserver.isSleeping() || SystemObjects::context().getState() == SystemState::SAFE_HALT) {
            k_msleep(1000);
        }
        
        uint32_t current_head = g_trace_head;

        if (current_head - g_trace_tail > TRACE_BUFFER_SIZE) {
            uint32_t skipped = (current_head - g_trace_tail) - TRACE_BUFFER_SIZE;
            g_trace_tail = current_head - TRACE_BUFFER_SIZE;
            LOG_WRN("Trace buffer overflow: skipped %u stale events", skipped);
        }

        while (g_trace_tail != g_trace_head) {
            uint32_t wrapped_idx = g_trace_tail & TRACE_BUFFER_MASK;
            TraceEvent event;

            unsigned int key = irq_lock();
            if (g_trace_head - g_trace_tail > TRACE_BUFFER_SIZE) {
                irq_unlock(key);
                break; 
            }
            event = g_trace_buffer[wrapped_idx];
            irq_unlock(key);

            TraceThread t_enum = get_target_thread(event.thread);
            const char* t_name = nullptr;
            
            switch (t_enum) {
                case TraceThread::PRODUCER:  t_name = "PRODUCER "; break;
                case TraceThread::PROCESSOR: t_name = "PROCESSOR"; break;
                case TraceThread::LOGGER:    t_name = "LOGGER   "; break;
                default: break;
            }

            if (t_name != nullptr) {
                uint32_t timestamp_us = k_cyc_to_us_floor32(event.timestamp);
                int t_idx = static_cast<int>(t_enum);

                if (event.type == TraceType::SWITCH_OUT) {
                    // Record the time the thread was suspended
                    thread_out_times[t_idx] = timestamp_us;
                } 
                else if (event.type == TraceType::SWITCH_IN) {
                    // Calculate how long the thread was preempted/blocked
                    uint32_t blocked_duration = timestamp_us - thread_out_times[t_idx];
                    
                    if (thread_out_times[t_idx] > 0) {
                        // Define our known, expected thread suspensions
                        bool is_fast_i2c_wait = (blocked_duration < 1000);
                        bool is_normal_500ms_sleep = (blocked_duration >= 490000 && blocked_duration <= 510000);

                        // If the delay is NOT expected, it is a real preemption!
                        if (!is_fast_i2c_wait && !is_normal_500ms_sleep) {
                            LOG_INF("[%s] Preemption delay: %u us", t_name, blocked_duration);
                        }
                    }
                }
            }
            g_trace_tail++;
        }

        k_msleep(50); 
    }
}
extern const k_tid_t trace_tid;
// Define the thread with the lowest priority to avoid starving your compute loops
K_THREAD_DEFINE(trace_tid, 1024, trace_logger_thread, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
