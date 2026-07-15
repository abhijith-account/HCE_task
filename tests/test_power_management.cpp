#include <gtest/gtest.h>
#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include "Power_Management_System.h"

#define PM_STATE_SUSPEND_TO_RAM 1
#define PM_ALL_SUBSTATES 0

extern "C" uint32_t virtual_uptime;

enum class PmAction {NONE,LOCK_ACQUIRED,LOCK_RELEASED,I2C_SUSPENDED,I2C_RESUMED,RTC_SET };
static std::array<PmAction,10> action_history;
static size_t action_idx=0;

static void (*mock_rtc_alarm_cb)(const device*, uint8_t,uint32_t,void*)=nullptr;
static void* mock_rtc_user_data=nullptr;
static bool mock_device_ready_status = true;

extern "C"{
   void pm_policy_state_lock_get(uint8_t state,uint8_t substate_id){
         if (action_idx < action_history.size()){
            action_history[action_idx++]=PmAction::LOCK_ACQUIRED;
         }
   }
   
   void pm_policy_state_lock_put(uint8_t state,uint8_t substate_id){
        if (action_idx<action_history.size()){
            action_history[action_idx++]=PmAction::LOCK_RELEASED;
        }
   }
   
   int pm_device_action_run(const struct device *dev,enum pm_device_action action){
        if (action==PM_DEVICE_ACTION_SUSPEND){
            if (action_idx<action_history.size()){
                action_history[action_idx++]=PmAction::I2C_SUSPENDED;
            }
        }
        else if(action==PM_DEVICE_ACTION_RESUME){
            if (action_idx<action_history.size()){
              action_history[action_idx++]=PmAction::I2C_RESUMED;
            }
        }
        return 0;
   }
   
   bool device_is_ready(const struct device *dev){
        return mock_device_ready_status;
   }
   
   uint32_t counter_us_to_ticks(const struct device *dev,uint64_t us){
        return us/1000;
   }
   
   int counter_set_channel_alarm(const struct device *dev,uint8_t chan_id,const struct counter_alarm_cfg *alarm_cfg){
       if (action_idx < action_history.size()){
          action_history[action_idx++]=PmAction::RTC_SET;
       }
       
       mock_rtc_alarm_cb=alarm_cfg->callback;
       mock_rtc_user_data=alarm_cfg->user_data;
       return 0;
   }
}

class PowerManagementTestSuite:public::testing::Test{
    protected:
          const device* dummy_rtc=reinterpret_cast<const device*>(0x111);
          const device* dummy_i2c=reinterpret_cast<const device*>(0x222);
          
          void SetUp() override {
              virtual_uptime=0;
              action_idx=0;
              action_history.fill(PmAction::NONE);
              mock_rtc_alarm_cb=nullptr;
              mock_device_ready_status = true;
          }
};

TEST_F(PowerManagementTestSuite, InitSuccessLogsInfo){
    PowerManager pwr(dummy_rtc,dummy_i2c);

    testing::internal::CaptureStdout();
    EXPECT_TRUE(pwr.init());
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);

    EXPECT_TRUE(output.find("[INF] Power Manager initialized. Deep Sleep Locked.") != std::string_view::npos)
    << "Expected successful init log missing! Actual: " << output;
}

TEST_F(PowerManagementTestSuite,FastForwardsThroughStateTransitions){
    PowerManager pwr(dummy_rtc,dummy_i2c);
    pwr.init();
    
    EXPECT_EQ(pwr.getMode(),PowerMode::ACTIVE);
    
    virtual_uptime=29999;
    pwr.processFSM();
    EXPECT_EQ(pwr.getMode(),PowerMode::ACTIVE)<<"Premature transition to IDLE!";
    
    virtual_uptime=30000;
    testing::internal::CaptureStdout();
    pwr.processFSM();
    const auto raw_output_idle= testing::internal::GetCapturedStdout();
    std::string_view output_idle(raw_output_idle);
    
    EXPECT_EQ(pwr.getMode(),PowerMode::IDLE)<<"Failed to transition to IDLE at 30s boundary";
    EXPECT_TRUE(output_idle.find("[INF] 30s Inactivity: Transitioning to IDLE mode")!=std::string_view::npos)<<"Expected IDLE transition log missing!";
    
    virtual_uptime=34999;
    pwr.processFSM();
    EXPECT_EQ(pwr.getMode(),PowerMode::IDLE)<<"Premature transition to STOP!";
    
    virtual_uptime=35000;
    testing::internal::CaptureStdout();
    pwr.processFSM();
    const auto  raw_output_stop= testing::internal::GetCapturedStdout();
    std::string_view output_stop(raw_output_stop);
    
    EXPECT_EQ(pwr.getMode(),PowerMode::STOP)<<"Failed to enter Deep Sleep at 35s boundary!";
    EXPECT_TRUE(output_stop.find("[WRN] 35s Inactivity: Transitioning to STOP mode (Deep Sleep).")!=std::string_view::npos)<<"Expected STOP transition log missing!";
}

TEST_F(PowerManagementTestSuite,VerifiesSleepNotifictionOrder){
    PowerManager pwr(dummy_rtc,dummy_i2c);
    pwr.init();
    
    action_idx=0;
    action_history.fill(PmAction::NONE);
    
    virtual_uptime=35000;
    pwr.processFSM();
    pwr.processFSM();
    
    ASSERT_GE(action_idx,3);
    EXPECT_EQ(action_history[0],PmAction::I2C_SUSPENDED);
    EXPECT_EQ(action_history[1],PmAction::RTC_SET);
    EXPECT_EQ(action_history[2],PmAction::LOCK_RELEASED);
}

TEST_F(PowerManagementTestSuite,WakeupInterruptRestoreSystem){
    PowerManager pwr(dummy_rtc,dummy_i2c);
    pwr.init();

    virtual_uptime=35000;
    pwr.processFSM();
    pwr.processFSM();
    ASSERT_EQ(pwr.getMode(),PowerMode::STOP);
    ASSERT_NE(mock_rtc_alarm_cb, nullptr)<<"RTC Alarm callback was never registered!";

    action_idx=0;
    action_history.fill(PmAction::NONE);

    virtual_uptime=95000;
    
    testing::internal::CaptureStdout();
    mock_rtc_alarm_cb(dummy_rtc,0,0,mock_rtc_user_data);
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    
    EXPECT_EQ(pwr.getMode(),PowerMode::ACTIVE)<<"System failed to wake up from RTC interrupt!";
    EXPECT_TRUE(output.find("[WRN] === 60s RTC Wakeup Triggered! Restoring System State ===")!=std::string_view::npos)<<"Expected RTC Wakeup log missing!";
    EXPECT_TRUE(output.find("[INF] Hardware Activity Detected! Waking up I2C Bus...")!=std::string_view::npos)<<"Expected I2C resume log missing!";
    
    ASSERT_GE(action_idx,2);
    EXPECT_EQ(action_history[0],PmAction::I2C_RESUMED);
    EXPECT_EQ(action_history[1],PmAction::LOCK_ACQUIRED);
}

TEST_F(PowerManagementTestSuite,HandlesHardwareInitFailure){
    mock_device_ready_status=false;
    
    PowerManager pwr(dummy_rtc,dummy_i2c);
    
    testing::internal::CaptureStdout();
    EXPECT_FALSE(pwr.init())<<"System failed to catch missing RTC hardware!";
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    
    EXPECT_TRUE(output.find("[ERR] RTC device not ready")!=std::string_view::npos)<<"Expected hardware failure log missing";
}

bool run_thread_once=false;

extern void power_monitor_thread(void);

TEST_F(PowerManagementTestSuite,ExecutesPowerMonitorThread){
        EXPECT_NO_FATAL_FAILURE(power_monitor_thread());
}

TEST_F(PowerManagementTestSuite, ReportActivityWhenNotStopped){
    PowerManager pwr(dummy_rtc,dummy_i2c);
    
    pwr.reportActivity();
    
    EXPECT_EQ(pwr.getMode(),PowerMode::ACTIVE);
}

TEST_F(PowerManagementTestSuite, ThreadLoopConditionBranches){
    run_thread_once=true;

    testing::internal::CaptureStdout();
    EXPECT_NO_FATAL_FAILURE(power_monitor_thread());
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);

    EXPECT_TRUE(output.find("[INF] Power Manager initialized.")!=std::string_view::npos);
}
