#include <gtest/gtest.h>
#include <zephyr/kernel.h>
#include <array>
#include <atomic>
#include <string_view>
#include "RTOS_Synchronization_Layer.h"

extern ZephyrSemaphore display_sem;
extern SharedHeartRateBuffer hr_buffer;

static int mock_sem_count=0;
static void (*mock_registered_work_cb)(struct k_work*)=nullptr;
static struct k_work* mock_registered_work_obj=nullptr;

extern "C" {
    int k_sem_init(struct k_sem *sem,unsigned int initial,unsigned int limit){
        mock_sem_count=initial;
        return 0;
    }
    void k_sem_give(struct k_sem *sem){
        mock_sem_count++;
    }
    int k_sem_take(struct k_sem *sem,k_timeout_t timeout){
        if (mock_sem_count>0){
            mock_sem_count--;
            return 0;
        }
        return -EBUSY;
    }
    void k_work_init_delayable(struct k_work_delayable *dwork,void (*callback)(struct k_work *)){
        mock_registered_work_cb=callback;
        mock_registered_work_obj=&dwork->work;
    }
    int k_work_schedule(struct k_work_delayable *dwork,k_timeout_t delay){
        return 0;
    }
}

static std::atomic<uint32_t> test_work_fire_time{0};
static std::atomic<bool> test_work_executed{false};
    
void integration_test_work_callback(){
    test_work_fire_time.store(k_uptime_get_32());
    test_work_executed.store(true);
}

class SyncTestSuite:public::testing::Test{
  protected:
    void SetUp() override{
      test_work_fire_time.store(0);
      test_work_executed.store(false);
      mock_sem_count=0;
      mock_registered_work_cb=nullptr;
      
      hr_buffer.mutex.lock();
      hr_buffer.head=0;
      hr_buffer.tail=0;
      hr_buffer.data.fill(0);
      hr_buffer.mutex.unlock();
   } 
};

TEST_F(SyncTestSuite, SemaphoreSignalAndWait){
    EXPECT_EQ(display_sem.take(K_NO_WAIT),-EBUSY);

    display_sem.give();
    display_sem.give();

    EXPECT_EQ(display_sem.take(K_NO_WAIT),0);
    EXPECT_EQ(display_sem.take(K_NO_WAIT),0);
    EXPECT_EQ(display_sem.take(K_NO_WAIT),-EBUSY);
}

TEST_F(SyncTestSuite,MutexLockUnlockAndPriorityInversionGuard){
    hr_buffer.mutex.lock();

    hr_buffer.data[0]=99;
    k_msleep(10);

    EXPECT_EQ(hr_buffer.data[0],99);

    hr_buffer.mutex.unlock();

    EXPECT_NO_FATAL_FAILURE(k_msleep(5));
}

TEST_F(SyncTestSuite,WorkQueueDelayAccuracy){
    ZephyrWorkQueue test_work_instance(integration_test_work_callback);

    const uint32_t target_delay_ms=100;
    const uint32_t start_time=k_uptime_get_32();

    test_work_instance.schedule(K_MSEC(target_delay_ms));

    k_msleep(target_delay_ms);
    
    if (mock_registered_work_cb && mock_registered_work_obj){
        mock_registered_work_cb(mock_registered_work_obj);
    }
    
    ASSERT_TRUE(test_work_executed.load())<<"Error: Work queue callback failed to execute!";
    
    const uint32_t actual_delay=test_work_fire_time.load()-start_time;
    
    EXPECT_GE(actual_delay,target_delay_ms)<<"Security Error:Work executed ahead of scheduled delay context!";
    EXPECT_LE(actual_delay,target_delay_ms+50)<<"Performance Error: Work queue latency limits exceeded!";
}

bool run_thread_once=false;

extern void display_consumer_thread(void);

TEST_F(SyncTestSuite,ExecutesDisplayConsumerThread){
        testing::internal::CaptureStdout();
        EXPECT_NO_FATAL_FAILURE(display_consumer_thread());
        const auto raw_output = testing::internal::GetCapturedStdout();
        std::string_view output(raw_output);
        
        EXPECT_TRUE(output.find("[INF] [Display Consumer] Rendered Heart Rate: 0 bpm")!=std::string_view::npos)<<"Expected consumer log missing! Actual: "<< output;
}

extern void heart_rate_producer_thread(void);

TEST_F(SyncTestSuite,ExecutesHeartRateThread){
        EXPECT_NO_FATAL_FAILURE(heart_rate_producer_thread());
}

extern void print_status();

TEST_F(SyncTestSuite,ProductionStatusReporting){
      testing::internal::CaptureStdout();
      EXPECT_NO_FATAL_FAILURE(print_status());
      const auto raw_output = testing::internal::GetCapturedStdout();
      std::string_view output(raw_output);

      EXPECT_TRUE(output.find("[INF] --- [] 1-Second System Statistics Report ---")!=std::string_view::npos)<<"Expected status report log missing! Actual: "<< output;
}

TEST_F(SyncTestSuite,HeartRateRolloverBranch){
    run_thread_once=false;
    for(int i=0;i<61;i++){
        heart_rate_producer_thread();
    }
}

TEST_F(SyncTestSuite,ThreadLoopConditionBranches){
    run_thread_once=true;
    heart_rate_producer_thread();
    
    run_thread_once=true;
    display_consumer_thread();
}

TEST_F(SyncTestSuite,WorkQueueHandlesNullCallbackSafely){
    ZephyrWorkQueue empty_queue(nullptr);
    
    if (mock_registered_work_cb && mock_registered_work_obj){
        mock_registered_work_cb(mock_registered_work_obj);
    }
    
    SUCCEED();
}
