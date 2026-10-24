#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
#include "Threads.h"
#include "Thread_Common.h"
#include "Command_Core.h"
#include "Sensor_Commands.h"
#include "BME280.h"
#include "LPS22HB.h"
#include "PAV3015.h"

LOG_MODULE_DECLARE(COMMANDS, LOG_LEVEL_INF);

static bool g_bme280_initialized = false;
static bool g_lps22hb_initialized = false;
static bool g_pav3015_initialized = false;

extern "C" void producer_thread(void *arg1, void *arg2, void *arg3) {
    ProducerState state = ProducerState::ReadBME;

    SystemObjects::power().registerObserver(&g_powerObserver);

    do {
       while (g_powerObserver.isSleeping()) {
           k_msleep(200);
           atomic_set(&g_producer_alive, 1);
       }

       const SystemState sys_state = SystemObjects::context().getState();

       if (sys_state == SystemState::FAULT) {
           LOG_WRN("[%s] System in FAULT state. Disabling sensors and entering SAFE_HALT.", LogTags::PRODUCER);
           g_bme280_initialized = false;
           g_lps22hb_initialized = false;
           g_pav3015_initialized = false;
           (void)SystemObjects::context().requestTransition(SystemState::SAFE_HALT);
           k_msleep(100);
           continue;
       }

       if (sys_state == SystemState::SAFE_HALT) {
           /* INDUSTRY STANDARD SAFE HALT: Lock out all autonomous recovery and safely block the thread.
            * This requires a hardware reboot or an explicitly authorized override IRQ to clear. */
           LOG_ERR("[%s] System safely halted. Awaiting manual hardware reset.", LogTags::PRODUCER);
           while (SystemObjects::context().getState() == SystemState::SAFE_HALT) {
               k_msleep(1000);
               atomic_set(&g_producer_alive, 1); // Keep WDT fed so it doesn't bounce immediately
           }
           continue;
       }

       SystemObjects::power().reportActivity();

       if (!g_bme280_initialized) {
           if (!BME280Driver::initDevice(SystemObjects::i2c())) {
               LOG_ERR("[%s] Failed to initialize BME280. Retrying...", LogTags::PRODUCER);
           } else {
               g_bme280_initialized = true;
               SystemObjects::context().requestTransition(SystemState::RUNNING);
               k_msleep(20);
           }
       }

       if (!g_lps22hb_initialized) {
           if (!LPS22HBDriver::initDevice(SystemObjects::i2c())) {
               LOG_ERR("[%s] Failed to initialize LPS22HB. Retrying...", LogTags::PRODUCER);
           } else {
               g_lps22hb_initialized = true;
               LOG_INF("[%s] LPS22HB Initialized Successfully.", LogTags::PRODUCER);
           }
       }

       if (!g_pav3015_initialized) {
           if (!PAV3015Driver::initDevice(SystemObjects::i2c())) {
               LOG_ERR("[%s] Failed to initialize PAV3015. Retrying...", LogTags::PRODUCER);
           } else {
               g_pav3015_initialized = true;
               LOG_INF("[%s] PAV3015 Initialized Successfully.", LogTags::PRODUCER);
           }
       }

       bool enqueued = false;
       if (state == ProducerState::ReadBME) {
           enqueued = enqueueCommand<SensorReadCmd>(PROCESSOR_Q, SensorReg::BME_DESC.id, SensorReg::BME_DESC.reg, SensorReg::BME_DESC.len);
           state = ProducerState::ReadLPS;
       } else if (state == ProducerState::ReadLPS) {
           bool enq1 = enqueueCommand<SensorReadCmd>(PROCESSOR_Q, SensorReg::LPS_P_DESC.id, SensorReg::LPS_P_DESC.reg, SensorReg::LPS_P_DESC.len);
           bool enq2 = enqueueCommand<SensorReadCmd>(PROCESSOR_Q, SensorReg::LPS_T_DESC.id, SensorReg::LPS_T_DESC.reg, SensorReg::LPS_T_DESC.len);
           enqueued = enq1 && enq2;
           state = ProducerState::ReadPAV;
       } else {
           enqueued = enqueueCommand<SensorReadCmd>(PROCESSOR_Q, SensorReg::PAV_DESC.id, SensorReg::PAV_DESC.reg, SensorReg::PAV_DESC.len);
           state = ProducerState::ReadBME;
       }

       (void)enqueued;
       k_msleep(500);
       atomic_set(&g_producer_alive, 1);
    } while(THREAD_LOOP_CONDITION);
}

#ifdef IS_TEST_ENVIRONMENT
void resetRtosCommandTestState() noexcept {
    BME280Driver::resetForTest();
    g_bme280_initialized = false;
    g_lps22hb_initialized = false;
    g_pav3015_initialized = false;
    g_powerObserver.resetForTest();
}
#endif

K_THREAD_DEFINE(producer_tid,  ThreadConfig::StackLarge, producer_thread,  NULL, NULL, NULL, ThreadConfig::PrioProducer,  0, 0);
