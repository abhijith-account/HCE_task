#include "test_power_common.h"

TEST_F(PowerManagementTestSuite, StopExitNormalWakeup) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    virtual_uptime = 30000;
    pm.processFSM();
    virtual_uptime = 35000;
    pm.processFSM();

    virtual_uptime += 70000;
    mocks.setAllResumeRet(0);

    pm.reportActivity();
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_ACTIVE"));
    EXPECT_EQ(pm.consecutive_pm_failures, 0);
}

TEST_F(PowerManagementTestSuite, SimulationModeCoverage) {
    PowerManager& pm = PowerManager::getInstance();
    
    EXPECT_TRUE(pm.init(nullptr, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context));

    virtual_uptime = 30000;
    pm.processFSM(); 

    virtual_uptime = 35000;
    pm.processFSM(); 
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_SUSPEND_TO_RAM"));

    pm.reportActivity(); 
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_ACTIVE"));
}

TEST_F(PowerManagementTestSuite, SuspendResumeIgnoredErrors) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    mocks.setAllSuspendRet(-ENOSYS);
    mocks.setAllResumeRet(-ENOSYS);

    virtual_uptime = 30000;
    pm.processFSM(); 
    virtual_uptime = 35000;
    pm.processFSM(); 
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_SUSPEND_TO_RAM"));

    pm.reportActivity(); 
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_ACTIVE"));

    mocks.setAllSuspendRet(-ENOTSUP);
    mocks.setAllResumeRet(-ENOTSUP);

    virtual_uptime = 70000;
    pm.processFSM(); 
    virtual_uptime = 75000;
    pm.processFSM(); 
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_SUSPEND_TO_RAM"));

    pm.reportActivity();
    pm.processFSM();
    EXPECT_EQ(std::string_view(pm.current_state->getName()), std::string_view("PM_STATE_ACTIVE"));
}

TEST_F(PowerManagementTestSuite, TolerantSuspendResumeCoverage) {
    PowerManager& pm = PowerManager::getInstance();
    
    // 1. Cover nullptr devices returning true in suspend/resume wrappers
    pm.init(dummy_rtc, nullptr, nullptr, nullptr, nullptr, &sys_context);
    virtual_uptime = 30000;
    pm.processFSM(); // Enters Idle. Suspend wrappers called with nullptr.
    
    pm.reportActivity();
    pm.processFSM(); // Exits Idle. Resume wrappers called with nullptr.
    
    // 2. Cover valid devices returning non-ignored errors during resume
    pm.resetForTest();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    virtual_uptime = 30000;
    pm.processFSM(); // Enters Idle
    
    mocks.setAllResumeRet(-EIO); // Return a non-ignored error code
    pm.reportActivity();
    pm.processFSM(); // Exits Idle, triggers resume failure logs and returns false
}
