#include "test_power_common.h"
#include "test_Power_patch.h"

TEST_F(PowerManagementTestSuite, SuspendFailuresIdle) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);

    mocks.pm_uart_suspend_ret = -EIO;
    EXPECT_FALSE(IdleState::getInstance().enter(pm));
    mocks.pm_uart_suspend_ret = 0;

    mocks.pm_adc_suspend_ret = -EIO;
    EXPECT_FALSE(IdleState::getInstance().enter(pm));
    mocks.pm_adc_suspend_ret = 0;

    mocks.pm_i2c_suspend_ret = -EIO;
    EXPECT_FALSE(IdleState::getInstance().enter(pm));
    mocks.pm_i2c_suspend_ret = 0;

}

TEST_F(PowerManagementTestSuite, ActiveExecuteCoverage) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    ActiveState& active = ActiveState::getInstance();
    
    pm.last_activity_time = k_uptime_get_32();
    EXPECT_EQ(std::string_view(active.execute(pm).getName()), "PM_STATE_ACTIVE");

    pm.last_activity_time = k_uptime_get_32() - 40000;
    EXPECT_EQ(std::string_view(active.execute(pm).getName()), "PM_STATE_SUSPEND_TO_IDLE");
}

TEST_F(PowerManagementTestSuite, IdleExecuteCoverage) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    IdleState& idle = IdleState::getInstance();
    
    // Enter the state formally to capture the initial entry time internally
    virtual_uptime = 10000;
    idle.enter(pm);
    
    // Advance time by 3s (less than the 5s IDLE_TIMEOUT_MS)
    virtual_uptime = 13000;
    EXPECT_EQ(std::string_view(idle.execute(pm).getName()), "PM_STATE_SUSPEND_TO_IDLE");

    // Advance time past the 5s timeout
    virtual_uptime = 16000;
    EXPECT_EQ(std::string_view(idle.execute(pm).getName()), "PM_STATE_SUSPEND_TO_RAM");
}

TEST_F(PowerManagementTestSuite, IdleClockRestoreFailureCoverage) {
    PowerManager& pm = PowerManager::getInstance();
    pm.init(dummy_rtc, dummy_i2c, dummy_uart, dummy_usb, dummy_adc, &sys_context);
    IdleState::getInstance().enter(pm);

#ifdef PM_TEST_CAN_PATCH_CODE
    // clock_restore_from_idle() lives in an anonymous namespace and always
    // returns true on the host build, so the `!clock_ok` branch in
    // IdleState::exit() can only be reached by forcing it to return false.
    uint8_t* fn = pm_test_patch::find_own_function("clock_restore_from_idle");
    ASSERT_NE(fn, nullptr) << "clock_restore_from_idle symbol not found "
                              "(build must not strip symbols or inline it; use -O0)";

    uint8_t orig[8];
    ASSERT_TRUE(pm_test_patch::patch_return_false(fn, orig)) << "mprotect failed";

    testing::internal::CaptureStdout();
    IdleState::getInstance().exit(pm);   // clock_ok == false -> LOG_ERR + reportPmFailure()
    const auto raw_output = testing::internal::GetCapturedStdout();

    pm_test_patch::restore(fn, orig);

    EXPECT_TRUE(std::string_view(raw_output).find("Idle clock restore failed") != std::string_view::npos);
#else
    GTEST_SKIP() << "Runtime code patching unsupported on this platform";
#endif
}
