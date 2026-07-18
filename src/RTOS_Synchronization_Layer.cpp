#include "RTOS_Synchronization_Layer.h"
#include "Power_Management_System.h"       // Power Management Integration
#include "Device_State_Machine+Watchdog.h" // DeviceContext integration
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(SYNC_LAYER, LOG_LEVEL_WRN);

#ifndef CONTAINER_OF
#define CONTAINER_OF(ptr, type, field) ((type *)(((char *)(ptr)) - offsetof(type, field)))
#endif

#ifndef k_work_delayable_from_work
#define k_work_delayable_from_work(w) CONTAINER_OF(w, struct k_work_delayable, work)
#endif

#ifdef IS_TEST_ENVIRONMENT
    extern bool run_thread_once;
    #define THREAD_LOOP_CONDITION (run_thread_once ? (run_thread_once = false, true) : false)
#else
    #define THREAD_LOOP_CONDITION true
#endif

extern DeviceContext sys_context;

SharedHeartRateBuffer hr_buffer;
ZephyrSemaphore display_sem(0,10);

ZephyrSemaphore::ZephyrSemaphore(unsigned int initial,unsigned int limit){
    k_sem_init(&sem,initial,limit);
}

void ZephyrSemaphore::give(){
    k_sem_give(&sem);
}

int ZephyrSemaphore::take(k_timeout_t timeout){
    return k_sem_take(&sem,timeout);
}

ZephyrMutex::ZephyrMutex(){
    k_mutex_init(&mutex);
}

void ZephyrMutex::lock(){
    k_mutex_lock(&mutex,K_FOREVER);
}

void ZephyrMutex::unlock(){
    k_mutex_unlock(&mutex);
}

void ZephyrWorkQueue::execute_callback(struct k_work *w){
    auto delayable= k_work_delayable_from_work(w);
    auto* self=CONTAINER_OF(delayable,ZephyrWorkQueue,work);
    if (self->callback){
        self->callback();
    }
}

ZephyrWorkQueue::ZephyrWorkQueue(void (*cb)()):callback(cb){
    k_work_init_delayable(&work,execute_callback);
}

void ZephyrWorkQueue::schedule(k_timeout_t delay){
    k_work_schedule(&work,delay);
}

void ZephyrWorkQueue::cancel() {
    k_work_cancel_delayable(&work);
}

extern ZephyrWorkQueue status_work;

namespace {
    // Power Observer to halt periodic workloads during Deep Sleep
    class SyncPowerObserver final : public IPowerObserver {
    private:
        atomic_t is_sleeping;
    public:
        SyncPowerObserver() { atomic_set(&is_sleeping, 0); }
        void beforeSleep() override { 
            atomic_set(&is_sleeping, 1); 
            status_work.cancel(); // Stop the periodic timer from waking the MCU
        }
        void afterWakeup() override { 
            atomic_set(&is_sleeping, 0); 
            status_work.schedule(K_SECONDS(1)); // Restart status reporting
        }
        void sleepAborted() override { 
            atomic_set(&is_sleeping, 0);
            status_work.schedule(K_SECONDS(1)); 
        }
        bool isSleeping() const noexcept { return atomic_get(&is_sleeping) != 0; }
    };

    SyncPowerObserver g_syncPowerObserver;
    atomic_t g_syncObserverRegistered = ATOMIC_INIT(0);

    void ensure_sync_observer_registered() {
        // See identical rationale in Fault_Tolerant_I2C_Communication_Layer.cpp's
        // ensure_power_observer_registered(): registerObserver() was already
        // safe to call redundantly, this just removes the unguarded flag.
        if (atomic_cas(&g_syncObserverRegistered, 0, 1)) {
            PowerManager::getInstance().registerObserver(&g_syncPowerObserver);
        }
    }
}

void print_status(){
    ensure_sync_observer_registered();
    
    // Only print and reschedule if the system is awake and not faulting
    if (!g_syncPowerObserver.isSleeping() && sys_context.getState() != SystemState::SAFE_HALT) {
        LOG_INF("--- [] 1-Second System Statistics Report ---");
        status_work.schedule(K_SECONDS(1));
    }
}

ZephyrWorkQueue status_work(print_status);

void heart_rate_producer_thread(void){
    ensure_sync_observer_registered();
    static uint32_t mock_hr=60;
    
    do{
        // Gate production so we don't wake devices checking memory during Deep Sleep
        if (!g_syncPowerObserver.isSleeping() && sys_context.getState() != SystemState::SAFE_HALT) {
            hr_buffer.mutex.lock();
            
            hr_buffer.data[hr_buffer.head]=mock_hr;
            hr_buffer.head=(hr_buffer.head+1)%hr_buffer.data.size();
            mock_hr=(mock_hr>=120)?60:mock_hr+1;
            
            hr_buffer.mutex.unlock();
            
            display_sem.give();
        }
        k_msleep(500);
    }while(THREAD_LOOP_CONDITION);
}

void display_consumer_thread(void){
    do{
        // We do not gate this on isSleeping() because display_sem.take() 
        // naturally blocks if the producer stops yielding during sleep.
        display_sem.take(K_FOREVER);
        
        hr_buffer.mutex.lock();
        
        uint32_t hr_val=hr_buffer.data[hr_buffer.tail];
        hr_buffer.tail=(hr_buffer.tail+1)%hr_buffer.data.size();
        
        hr_buffer.mutex.unlock();
        
        // Final sanity check before logging
        if (!g_syncPowerObserver.isSleeping() && sys_context.getState() != SystemState::SAFE_HALT) {
            LOG_INF("[Display Consumer] Rendered Heart Rate: %u bpm", hr_val);
        }
    }while(THREAD_LOOP_CONDITION);
}

K_THREAD_DEFINE(hr_prod_tid,256,heart_rate_producer_thread,NULL,NULL,NULL,8,0,0);
K_THREAD_DEFINE(disp_cons_tid,256,display_consumer_thread,NULL,NULL,NULL,9,0,0);
