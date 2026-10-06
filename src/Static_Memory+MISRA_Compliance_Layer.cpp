#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "Static_Memory+MISRA_Compliance_Layer.h"
#include "Device_State_Machine+Watchdog.h"
#include "Power_Management_System.h"
#include <zephyr/logging/log.h>

#ifdef IS_TEST_ENVIRONMENT
extern int run_thread_iterations;
#define THREAD_LOOP_CONDITION \
    (run_thread_iterations > 0 ? (--run_thread_iterations, true) : false)
#else
#define THREAD_LOOP_CONDITION true
#endif

LOG_MODULE_REGISTER(MEM_SYS, LOG_LEVEL_INF);
extern DeviceContext sys_context;

/* Thread Health Monitoring Flag for Memory Monitor */
atomic_t g_mem_mon_alive = ATOMIC_INIT(1);

/* Known application threads, referenced by their K_THREAD_DEFINE-generated
 * k_tid_t symbols. We deliberately do NOT walk the kernel's internal thread
 * list (via thread_analyzer_print() or k_thread_foreach*()) at runtime:
 * that walk was observed to enter an infinite loop after the main() thread
 * terminated and its list node became stale, permanently hanging this
 * thread while flooding the log with a single repeated entry. Querying
 * each statically-known thread directly is bounded and cannot loop. */
extern const k_tid_t processor_tid;
extern const k_tid_t producer_tid;
extern const k_tid_t logger_tid;
extern const k_tid_t battery_tid;
extern const k_tid_t pr_tid;
extern const k_tid_t mem_mon_tid;
extern const k_tid_t hr_prod_tid;
extern const k_tid_t disp_cons_tid;
extern const k_tid_t bms_comm_tid;
extern const k_tid_t shell_tid;
extern const k_tid_t trace_tid;

namespace {
    struct MonitoredThread {
        const k_tid_t *tid;
        const char *label;
    };

    const MonitoredThread kMonitoredThreads[] = {
        { &pr_tid,        "pr_tid" },
        { &mem_mon_tid,   "mem_mon_tid" },
        { &processor_tid, "processor_tid" },
        { &producer_tid,  "producer_tid" },
        { &logger_tid,    "logger_tid" },
        { &battery_tid,   "battery_tid" },
        { &hr_prod_tid,   "hr_prod_tid" },
        { &disp_cons_tid, "disp_cons_tid" },
        { &bms_comm_tid,  "bms_comm_tid" },
        { &shell_tid,     "shell_tid" },
        { &trace_tid,     "trace_tid"}
    };

    /* --- Power Observer for Memory Monitor --- */
    class MemoryPowerObserver final : public IPowerObserver {
    private:
        atomic_t is_sleeping{};
    public:
        MemoryPowerObserver() { atomic_set(&is_sleeping, 0); }

        void beforeSleep() override {
            atomic_set(&is_sleeping, 1);
            LOG_INF("MemoryPowerObserver: Pausing memory monitor for sleep.");
        }

        void afterWakeup() override {
            atomic_set(&is_sleeping, 0);
            LOG_INF("MemoryPowerObserver: Resuming memory monitor.");
        }

        void sleepAborted() override {
            atomic_set(&is_sleeping, 0);
        }

        bool isSleeping() const noexcept { return atomic_get(&is_sleeping) != 0; }
    };

    MemoryPowerObserver g_memoryPowerObserver;
    atomic_t g_memoryObserverRegistered = ATOMIC_INIT(0);

    void ensure_memory_observer_registered() {
        if (atomic_cas(&g_memoryObserverRegistered, 0, 1)) {
            PowerManager::getInstance().registerObserver(&g_memoryPowerObserver);
        }
    }
}

void memory_monitor_thread(void){
    ensure_memory_observer_registered();
    uint32_t seconds_elapsed = 0;

    do {
        /* Explicitly yield CPU cycles if the power observer flags sleep mode */
        while (g_memoryPowerObserver.isSleeping()) {
            k_msleep(200);
            atomic_set(&g_mem_mon_alive, 1); // Keep WDT alive during sleep holding pattern
        }

        /* Strictly enforce RUNNING state instead of just avoiding SAFE_HALT */
        if (sys_context.getState() == SystemState::RUNNING) {

            /* Only trigger the analyzer every 30 seconds */
            if (seconds_elapsed >= 30) {
                LOG_INF("=== [System Health] Thread Stack Watermarks ===");
                for (const auto &mt : kMonitoredThreads) {
                    size_t unused_bytes = 0;
                    int ret = k_thread_stack_space_get(*mt.tid, &unused_bytes);
                    if (ret == 0) {
                        LOG_INF("%-16s: unused %5u B", mt.label, (unsigned)unused_bytes);
                    } else {
                        LOG_INF("%-16s: stack info unavailable (err %d)", mt.label, ret);
                    }
                }
                seconds_elapsed = 0;
            }
        }
        
        /* Sleep for 1 second so we can wake up and feed the Watchdog */
        k_msleep(1000);
        seconds_elapsed++;
        
        /* Flag health for global watchdog */
        atomic_set(&g_mem_mon_alive, 1);
        
    } while(THREAD_LOOP_CONDITION);
}

K_THREAD_DEFINE(mem_mon_tid,1024,memory_monitor_thread,NULL,NULL,NULL,12,0,0);
