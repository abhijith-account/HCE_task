#pragma once

#include <zephyr/kernel.h>
#include <cstddef>
#include <cstdint>
#include <new>
#include <utility>

/* Command-pattern infrastructure: base class, memory pool, queues.
 * Nothing in this file knows about sensors. */

namespace QueueConfig {
    constexpr size_t Depth = 10;
    constexpr size_t Alignment = 4;
}

namespace PoolConfig {
    constexpr size_t Elements = 16;
    constexpr size_t Size = 48;
}

static_assert(QueueConfig::Depth > 0, "Queue depth must be greater than 0");
static_assert(PoolConfig::Elements >= QueueConfig::Depth, "Pool must support peak queue depth");

class ICommand {
public:
    uint32_t command_id;

    ICommand();
    virtual ~ICommand() = default;

    ICommand(const ICommand&) = delete;
    ICommand& operator=(const ICommand&) = delete;
    ICommand(ICommand&&) = delete;
    ICommand& operator=(ICommand&&) = delete;

    virtual void execute() noexcept = 0;

    void operator delete(void* ptr) noexcept;
    void destroy() noexcept;
};

struct QueueStats {
    uint32_t commandsCreated = 0;
    uint32_t commandsDropped = 0;
    uint32_t loggerQueueFull = 0;
    uint32_t processorPeakDepth = 0;
    uint32_t loggerPeakDepth = 0;
};
extern QueueStats g_queueStats;

/* Pool slot: every command object must fit in one of these.
 * Each command's .cpp static_asserts its own size and alignment against it. */
struct alignas(8) MaxCommandSize { uint8_t buffer[PoolConfig::Size]{}; };

extern struct k_msgq processor_queue;
extern struct k_msgq logger_queue;

constexpr auto PROCESSOR_Q = &processor_queue;
constexpr auto LOGGER_Q    = &logger_queue;

void* allocateCommandMemory() noexcept;
bool enqueueCommandRaw(k_msgq* queue, ICommand* cmd) noexcept;

template<typename CmdType, typename... Args>
[[nodiscard]] bool enqueueCommand(k_msgq* queue, Args&&... args) noexcept {
    void* mem = allocateCommandMemory();
    if (!mem) return false;
    ICommand* cmd = new(mem) CmdType(std::forward<Args>(args)...);
    if (!enqueueCommandRaw(queue, cmd)) {
        cmd->destroy();
        return false;
    }
    return true;
}
