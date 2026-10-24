#include "Smart_Battery_System.h"
#include "Power_Management_System.h"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <cstdlib>

#ifdef IS_TEST_ENVIRONMENT
    __attribute__((weak)) int test_iterations_remaining = 0;
    #define THREAD_LOOP_CONDITION (test_iterations_remaining > 0 ? (test_iterations_remaining--, true) : false)
#else
    #define THREAD_LOOP_CONDITION true
#endif

extern DeviceContext sys_context;
#ifndef IS_TEST_ENVIRONMENT
extern I2CManager i2c_manager;
#endif

LOG_MODULE_REGISTER(BATTERY_RUNTIME, LOG_LEVEL_INF);

/* Thread Health Monitoring Flags for Battery System */
atomic_t g_bms_comm_alive = ATOMIC_INIT(1);
atomic_t g_batt_mon_alive = ATOMIC_INIT(1);

extern const k_tid_t bms_comm_tid;
extern const k_tid_t battery_tid;

/* The active battery instance (tests assign this pointer directly). */
SbsBattery* smart_battery = nullptr;

namespace {
    bool isBatteryCharging() {
        return (smart_battery != nullptr) && (smart_battery->getState() == BatteryFSM::CHARGING);
    }

    bool isBatteryInCutoff() {
        return (smart_battery != nullptr) && (smart_battery->getState() == BatteryFSM::CUTOFF);
    }

    class BmsPowerObserver final : public IPowerObserver {
    private:
        atomic_t is_sleeping{};
    public:
        BmsPowerObserver() { atomic_set(&is_sleeping, 0); }
        void beforeSleep() override { atomic_set(&is_sleeping, 1); }
        void afterWakeup() override {
            atomic_set(&is_sleeping, 0);
            if (smart_battery != nullptr) { smart_battery->notifySystemWakeup(); }
        }
        void sleepAborted() override { atomic_set(&is_sleeping, 0); }
        bool isSleeping() const noexcept { return atomic_get(&is_sleeping) != 0; }
    };

    static BmsPowerObserver g_bmsPowerObserver;
    K_SEM_DEFINE(bms_objects_ready_sem, 0, 1);

    bool bms_objects_initialized = false;
#ifndef IS_TEST_ENVIRONMENT
    SbsBattery static_smart_battery(&i2c_manager, &sys_context);
#endif

    void initializeBmsObjects() {
        if (!bms_objects_initialized) {
#ifndef IS_TEST_ENVIRONMENT
            smart_battery = &static_smart_battery;
#endif
            bms_objects_initialized = true;
        }
    }
}

#ifdef IS_TEST_ENVIRONMENT
bool isBmsObserverSleepingForTest() { return g_bmsPowerObserver.isSleeping(); }
#endif

SbsBattery* getSmartBatteryInstance() {
    initializeBmsObjects();
    return smart_battery;
}

#ifndef IS_TEST_ENVIRONMENT
I2CManager* getI2cBusManagerInstance() {
    initializeBmsObjects();
    return &i2c_manager;
}
#endif

void bms_comm_thread(void) {
    initializeBmsObjects();

    if (smart_battery == nullptr) {
        LOG_ERR("Smart battery instance is null.");
        k_sem_give(&bms_objects_ready_sem);
        while(THREAD_LOOP_CONDITION) {
            k_msleep(1000);
            atomic_set(&g_bms_comm_alive, 1);
        }
        return;
    }

    uint32_t init_attempts = 0U;

    while (true) {
        while (g_bmsPowerObserver.isSleeping() || sys_context.getState() == SystemState::SAFE_HALT) {
            k_msleep(1000);
            atomic_set(&g_bms_comm_alive, 1);
        }

        if (smart_battery->init()) {
            break;
        }
        
        ++init_attempts;
        LOG_ERR("BMS sensors (INA226 / thermistor) init failed (attempt %u/%u).", init_attempts, BatteryLimits::MAX_INIT_RETRIES);

        if (init_attempts >= BatteryLimits::MAX_INIT_RETRIES) {
            LOG_ERR("BMS sensor init failed %u times -- escalating fault instead of retrying forever.", init_attempts);
            if (sys_context.getState() != SystemState::SAFE_HALT) {
                sys_context.triggerFault("BMS Sensor Init Failure");
            }
            k_sem_give(&bms_objects_ready_sem);
            
            while (THREAD_LOOP_CONDITION) {
                k_msleep(1000);
                atomic_set(&g_bms_comm_alive, 1);
            }
            return;
        }
        
        k_msleep(5000);
        atomic_set(&g_bms_comm_alive, 1);
    }

    k_sem_give(&bms_objects_ready_sem);
    PowerManager::getInstance().registerObserver(&g_bmsPowerObserver);
    smart_battery->notifySystemWakeup(); 
    k_msleep(250);

    do {
       while ((g_bmsPowerObserver.isSleeping() && !isBatteryCharging()) ||
              (sys_context.getState() == SystemState::SAFE_HALT && !isBatteryInCutoff())) {
           k_msleep(1000);
           atomic_set(&g_bms_comm_alive, 1);
       }

       smart_battery->pollHardwareAndUpdateCache();

       auto v = smart_battery->getVoltage();
       auto i = smart_battery->getCurrent();
       auto soc = smart_battery->getStateOfCharge();
       auto temp = smart_battery->getTemperature();
       auto cap = smart_battery->getCapacity();

       if (v.success && i.success && soc.success && temp.success && cap.success) {
           int32_t temp_c_tenths = static_cast<int32_t>(temp.value.value) - Thermistor::KELVIN_OFFSET_TENTHS;
           LOG_INF("BATTERY STATUS | V: %d mV | I: %d mA | SOC: %u %% | Temp: %d.%d C | Cap: %u mAh",
                   (int)v.value.value, 
                   (int)i.value.value, 
                   (unsigned int)soc.value.value,
                   temp_c_tenths / 10, std::abs(temp_c_tenths % 10), 
                   (unsigned int)cap.value.value);
       } else {
           LOG_WRN("Battery cache invalid | V:%d I:%d SOC:%d T:%d Cap:%d",(int)v.success, (int)i.success, (int)soc.success,(int)temp.success, (int)cap.success);
       }

       k_msleep(1000);
       
       atomic_set(&g_bms_comm_alive, 1);
    } while(THREAD_LOOP_CONDITION);
}

void battery_monitor_thread(void) {
    k_sem_take(&bms_objects_ready_sem, K_FOREVER);
    k_msleep(500);

    do {
        while ((g_bmsPowerObserver.isSleeping() && !isBatteryCharging()) ||
               (sys_context.getState() == SystemState::SAFE_HALT && !isBatteryInCutoff())) {
            k_msleep(1000);
            atomic_set(&g_batt_mon_alive, 1);
        }

        if (smart_battery != nullptr) {
            smart_battery->processFSM();
        }
        
        k_msleep(1000);
        
        atomic_set(&g_batt_mon_alive, 1);
    } while(THREAD_LOOP_CONDITION);
}

K_THREAD_DEFINE(bms_comm_tid, 1024, bms_comm_thread, NULL, NULL, NULL, BatteryLimits::BMS_THREAD_PRIO, 0, 0);
K_THREAD_DEFINE(battery_tid, 1024, battery_monitor_thread, NULL, NULL, NULL, BatteryLimits::MONITOR_THREAD_PRIO, 0, 0);
