#include <zephyr/kernel.h>
#include "Static_Memory+MISRA_Compliance_Layer.h"
#include "Device_State_Machine+Watchdog.h" // For sys_context.getState()
#include <zephyr/logging/log.h>
#include <zephyr/debug/thread_analyzer.h>

#ifdef IS_TEST_ENVIRONMENT
    extern bool run_thread_once;
    #define THREAD_LOOP_CONDITION (run_thread_once ? (run_thread_once = false, true) : false)
#else 
    #define THREAD_LOOP_CONDITION true
#endif
LOG_MODULE_REGISTER(MEM_SYS, LOG_LEVEL_INF);
extern DeviceContext sys_context;

void memory_monitor_thread(void){
    do{
    // Gated on SAFE_HALT only, matching the convention every other
    // background thread in this codebase follows (print_status(),
    // display_consumer_thread, producer_thread, etc.) so routine logging
    // doesn't drown out fault handling during a critical halt.
    //
    // Deliberately NOT gated on the PowerManager sleep state the way I2C/UART
    // consumers are: thread_analyzer_print() reads kernel thread metadata
    // only, touches no peripheral that STOP suspends, and continuing to
    // report stack watermarks during STOP is arguably more useful than less
    // if the device ever hangs while asleep.
    if (sys_context.getState() != SystemState::SAFE_HALT) {
        LOG_INF("=== [System Health] Thread Stack Watermarks ===");
        thread_analyzer_print(0);
    }
    k_msleep(5000);
    }while(THREAD_LOOP_CONDITION);
}

K_THREAD_DEFINE(mem_mon_tid,1536,memory_monitor_thread,NULL,NULL,NULL,12,0,0);
