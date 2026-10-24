#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "Threads.h"
#include "Thread_Common.h"
#include "Command_Core.h"

extern "C" void processor_thread(void *arg1, void *arg2, void *arg3) {
    ICommand* incoming_cmd;
    do {
        // Halt processing entirely while in SAFE_HALT
        while (g_powerObserver.isSleeping() || SystemObjects::context().getState() == SystemState::SAFE_HALT) {
            k_msleep(1000);
            atomic_set(&g_processor_alive, 1);
        }

        if (k_msgq_get(PROCESSOR_Q, &incoming_cmd, K_MSEC(500)) == 0) {
            incoming_cmd->execute();
            incoming_cmd->destroy();
        }

        atomic_set(&g_processor_alive, 1);
    } while(THREAD_LOOP_CONDITION);
}

extern "C" void logger_thread(void *arg1, void *arg2, void *arg3) {
    ICommand* incoming_cmd;
    do {
        // Halt processing entirely while in SAFE_HALT
        while (g_powerObserver.isSleeping() || SystemObjects::context().getState() == SystemState::SAFE_HALT) {
            k_msleep(1000);
            atomic_set(&g_logger_alive, 1);
        }

        if (k_msgq_get(LOGGER_Q, &incoming_cmd, K_MSEC(500)) == 0) {
            incoming_cmd->execute();
            incoming_cmd->destroy();
        }

        atomic_set(&g_logger_alive, 1);
    } while(THREAD_LOOP_CONDITION);
}

K_THREAD_DEFINE(processor_tid, ThreadConfig::StackLarge, processor_thread, NULL, NULL, NULL, ThreadConfig::PrioProcessor, 0, 0);
extern const k_tid_t logger_tid;
K_THREAD_DEFINE(logger_tid,    ThreadConfig::StackLarge, logger_thread,    NULL, NULL, NULL, ThreadConfig::PrioLogger,    0, 0);
