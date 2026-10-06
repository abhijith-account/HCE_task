#include "RTOS_Synchronization_Layer.h"
#include "Power_Management_System.h"
#include "Device_State_Machine+Watchdog.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(SYNC_LAYER, LOG_LEVEL_INF);

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

atomic_t g_hr_prod_alive = ATOMIC_INIT(1);
atomic_t g_disp_cons_alive = ATOMIC_INIT(1);

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
    class SyncPowerObserver final : public IPowerObserver {
    private:
        atomic_t is_sleeping{};
    public:
        SyncPowerObserver() { atomic_set(&is_sleeping, 0); }
        
        void beforeSleep() override {
            atomic_set(&is_sleeping, 1);
            status_work.cancel();
            LOG_INF("SyncPowerObserver: Pausing synchronization work and heart rate threads for sleep.");
        }
        
        void afterWakeup() override {
            atomic_set(&is_sleeping, 0);
            status_work.schedule(K_SECONDS(1));
            LOG_INF("SyncPowerObserver: Resuming synchronization work and heart rate threads.");
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
        if (atomic_cas(&g_syncObserverRegistered, 0, 1)) {
            PowerManager::getInstance().registerObserver(&g_syncPowerObserver);
        }
    }
}

void print_status(){
    ensure_sync_observer_registered();

    if (!g_syncPowerObserver.isSleeping() && sys_context.getState() == SystemState::RUNNING) {
        LOG_INF("--- [] 1-Second System Statistics Report ---");
        status_work.schedule(K_SECONDS(1));
    }
}

ZephyrWorkQueue status_work(print_status);

void heart_rate_producer_thread(void){
    ensure_sync_observer_registered();

    static uint32_t mock_hr = 70;
    static bool warming_up = true;
    static uint32_t hold_ticks = 15;

    do{
        while (g_syncPowerObserver.isSleeping()) {
            k_msleep(200);
            atomic_set(&g_hr_prod_alive, 1); // Keep WDT alive during sleep holding pattern
        }

        if (sys_context.getState() == SystemState::RUNNING) {
            hr_buffer.mutex.lock();

            hr_buffer.data[hr_buffer.head] = mock_hr;
            hr_buffer.head = (hr_buffer.head + 1) % hr_buffer.data.size();

            if (hold_ticks > 0) {
                hold_ticks--;
                mock_hr += (hold_ticks % 2 == 0) ? 1 : -1;
            } else if (warming_up) {
                mock_hr += 2;
                if (mock_hr >= 145) {
                    warming_up = false;
                    hold_ticks = 20;
                }
            } else {
                mock_hr -= 1;
                if (mock_hr <= 65) {
                    warming_up = true;
                    hold_ticks = 30;
                }
            }

            hr_buffer.mutex.unlock();

            display_sem.give();
        }
        k_msleep(500);

        atomic_set(&g_hr_prod_alive, 1);

    }while(THREAD_LOOP_CONDITION);
}

void display_consumer_thread(void){
    ensure_sync_observer_registered();

    do{
        while (g_syncPowerObserver.isSleeping()) {
            k_msleep(200);
            atomic_set(&g_disp_cons_alive, 1); // Keep WDT alive during sleep holding pattern
        }

        if (display_sem.take(K_MSEC(500)) == 0) {
            
            hr_buffer.mutex.lock();

            uint32_t hr_val=hr_buffer.data[hr_buffer.tail];
            hr_buffer.tail=(hr_buffer.tail+1)%hr_buffer.data.size();

            hr_buffer.mutex.unlock();

            if (sys_context.getState() == SystemState::RUNNING) {
                LOG_INF("[Display Consumer] Rendered Heart Rate: %u bpm", hr_val);
            }
        }

        atomic_set(&g_disp_cons_alive, 1);

    }while(THREAD_LOOP_CONDITION);
}

extern const k_tid_t hr_prod_tid;
extern const k_tid_t disp_cons_tid;
K_THREAD_DEFINE(hr_prod_tid,256,heart_rate_producer_thread,NULL,NULL,NULL,8,0,0);
K_THREAD_DEFINE(disp_cons_tid,1024,display_consumer_thread,NULL,NULL,NULL,9,0,0);
