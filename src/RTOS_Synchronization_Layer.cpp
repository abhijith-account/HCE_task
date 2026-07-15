#include "RTOS_Synchronization_Layer.h"
#include <zephyr/logging/log.h>

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

extern ZephyrWorkQueue status_work;
void print_status(){
    LOG_INF("--- [] 1-Second System Statistics Report ---");
    status_work.schedule(K_SECONDS(1));
}

ZephyrWorkQueue status_work(print_status);

void heart_rate_producer_thread(void){
    static uint32_t mock_hr=60;
    do{
          hr_buffer.mutex.lock();
          
          hr_buffer.data[hr_buffer.head]=mock_hr;
          hr_buffer.head=(hr_buffer.head+1)%hr_buffer.data.size();
          mock_hr=(mock_hr>=120)?60:mock_hr+1;
          
          hr_buffer.mutex.unlock();
          
          display_sem.give();
          k_msleep(500);
    }while(THREAD_LOOP_CONDITION);
}

void display_consumer_thread(void){
    do{
        display_sem.take(K_FOREVER);
        
        hr_buffer.mutex.lock();
        
        uint32_t hr_val=hr_buffer.data[hr_buffer.tail];
        hr_buffer.tail=(hr_buffer.tail+1)%hr_buffer.data.size();
        
        hr_buffer.mutex.unlock();
        
        LOG_INF("[Display Consumer] Rendered Heart Rate: %u bpm", hr_val);
    }while(THREAD_LOOP_CONDITION);
}

K_THREAD_DEFINE(hr_prod_tid,256,heart_rate_producer_thread,NULL,NULL,NULL,8,0,0);
K_THREAD_DEFINE(disp_cons_tid,256,display_consumer_thread,NULL,NULL,NULL,9,0,0);

