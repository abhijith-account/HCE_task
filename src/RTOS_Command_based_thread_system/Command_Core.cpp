#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
#include "Command_Core.h"
#include "Static_Memory+MISRA_Compliance_Layer.h"

/* The one place the COMMANDS log module is registered; every other file
 * of the thread system uses LOG_MODULE_DECLARE(COMMANDS, ...). */
LOG_MODULE_REGISTER(COMMANDS, LOG_LEVEL_INF);

StaticPool<MaxCommandSize, PoolConfig::Elements> g_commandPool;
QueueStats g_queueStats;

K_MSGQ_DEFINE(processor_queue, sizeof(ICommand*), QueueConfig::Depth, QueueConfig::Alignment);
K_MSGQ_DEFINE(logger_queue, sizeof(ICommand*), QueueConfig::Depth, QueueConfig::Alignment);

void* allocateCommandMemory() noexcept {
    void* const mem = g_commandPool.allocate();
    if (!mem) {
        g_queueStats.commandsDropped.fetch_add(1U, std::memory_order_relaxed);
        LOG_ERR("Memory pool exhausted. Dropping command.");
    }
    return mem;
}

bool enqueueCommandRaw(k_msgq* queue, ICommand* cmd) noexcept {
    if (k_msgq_put(queue, &cmd, K_NO_WAIT) != 0) {
        g_queueStats.commandsDropped.fetch_add(1U, std::memory_order_relaxed);
        return false;
    }

    const unsigned int key = irq_lock();
    const uint32_t used = static_cast<uint32_t>(QueueConfig::Depth) - k_msgq_num_free_get(queue);
    if (queue == PROCESSOR_Q && used > g_queueStats.processorPeakDepth.load(std::memory_order_relaxed)) {
        g_queueStats.processorPeakDepth = used;
    } else if (queue == LOGGER_Q && used > g_queueStats.loggerPeakDepth.load(std::memory_order_relaxed)) {
        g_queueStats.loggerPeakDepth = used;
    }
    irq_unlock(key);

    g_queueStats.commandsCreated.fetch_add(1U, std::memory_order_relaxed);
    return true;
}

ICommand::ICommand(){
    static atomic_t cmd_counter = 0;
    command_id = static_cast<uint32_t>(atomic_inc(&cmd_counter));
}

void ICommand::operator delete(void* ptr) noexcept {
    g_commandPool.deallocate(ptr);
}

void ICommand::destroy() noexcept {
    this->~ICommand();
    g_commandPool.deallocate(this);
}
