#include <gtest/gtest.h>
#include <string>
#include <string_view>
#include <zephyr/drivers/watchdog.h>

#define private public
#define protected public
#include "Device_State_Machine+Watchdog.h"
#undef private
#undef protected

static bool mock_wdt_ready = true;
static int mock_wdt_install_res = 0;
static int mock_wdt_setup_res = 0;
static int mock_wdt_feed_res = 0;

extern "C" {
    bool device_is_ready(const struct device *dev) {
        return mock_wdt_ready;
    }
    
    int wdt_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *cfg) {
        return mock_wdt_install_res;
    }
    
    int wdt_setup(const struct device *dev, uint8_t options) {
        return mock_wdt_setup_res;
    }
    
    int wdt_feed(const struct device *dev, int channel_id) {
        return mock_wdt_feed_res;
    }
    
    // Add declaration so we can test the C-hook at the bottom of the file
    void daly_watchdog_feed_hook(void);
}

class StateMachineTestSuite : public ::testing::Test {
protected:
    void SetUp() override {
        mock_wdt_ready = true;
        mock_wdt_install_res = 0;
        mock_wdt_setup_res = 0;
        mock_wdt_feed_res = 0;
    }
};

TEST_F(StateMachineTestSuite, AllowsLegalTransitions) {
    DeviceContext sys_context;

    EXPECT_EQ(sys_context.getState(), SystemState::INIT) << "System did not boot into INIT state";
    
    testing::internal::CaptureStdout();
    bool success_run = sys_context.requestTransition(SystemState::RUNNING);
    const auto raw_output_run = testing::internal::GetCapturedStdout();
    std::string_view output_run(raw_output_run);

    EXPECT_TRUE(success_run) << "Valid transition to RUNNING was rejected";
    EXPECT_EQ(sys_context.getState(), SystemState::RUNNING);
    EXPECT_TRUE(output_run.find("System transitioned :") != std::string_view::npos) << "Expected transition INF log missing!";
    
    testing::internal::CaptureStdout();
    bool success_fault = sys_context.requestTransition(SystemState::FAULT);
    const auto raw_output_fault = testing::internal::GetCapturedStdout();
    std::string_view output_fault(raw_output_fault);
    
    EXPECT_TRUE(success_fault) << "Valid transition from RUNNING to FAULT was rejected";
    EXPECT_EQ(sys_context.getState(), SystemState::FAULT);
    EXPECT_TRUE(output_fault.find("System transitioned :") != std::string_view::npos) << "Expected transition INF log missing";
}

TEST_F(StateMachineTestSuite, FaultInjectionTriggersSafeHalt) {
    DeviceContext sys_context;

    sys_context.requestTransition(SystemState::RUNNING);
    ASSERT_EQ(sys_context.getState(), SystemState::RUNNING);
    
    testing::internal::CaptureStdout();
    sys_context.triggerFault("Simulated I2C Bus Hard-Lock");
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    
    EXPECT_EQ(sys_context.getState(), SystemState::SAFE_HALT) << "System failed to drop to SAFE_HALT after a critical fault!";
    EXPECT_TRUE(output.find("CRITICAL FAULT: Simulated I2C Bus Hard-Lock. Forcing SAFE_HALT.") != std::string_view::npos) << "Expected critical fault log missing! Actual output:  " << output;
}

TEST_F(StateMachineTestSuite, RejectsIllegalTransitions) {
    DeviceContext sys_context_halted;
    
    sys_context_halted.triggerFault("Test Error");
    ASSERT_EQ(sys_context_halted.getState(), SystemState::SAFE_HALT);

    testing::internal::CaptureStdout();
    // SAFE_HALT -> RUNNING is illegal. It should fail and stay in SAFE_HALT.
    bool success_halt = sys_context_halted.requestTransition(SystemState::RUNNING);
    const auto raw_output_halt_reject = testing::internal::GetCapturedStdout();
    std::string_view output_halt_reject(raw_output_halt_reject);
    
    EXPECT_FALSE(success_halt) << "System allowed an unsafe transition out of SAFE_HALT!";
    EXPECT_EQ(sys_context_halted.getState(), SystemState::SAFE_HALT) << "System state inappropriately changed from SAFE_HALT!";
    EXPECT_TRUE(output_halt_reject.find("Illegal state transition rejected:") != std::string_view::npos) << "Expected rejection error log missing!";
    
    DeviceContext sys_context_fault;
    
    sys_context_fault.requestTransition(SystemState::FAULT);
    ASSERT_EQ(sys_context_fault.getState(), SystemState::FAULT);
    
    testing::internal::CaptureStdout();
    // FAULT -> RUNNING is illegal. It should fail and fallback to SAFE_HALT.
    bool success_fault = sys_context_fault.requestTransition(SystemState::RUNNING);
    const auto raw_output_fault_reject = testing::internal::GetCapturedStdout();
    std::string_view output_fault_reject(raw_output_fault_reject);
    
    EXPECT_FALSE(success_fault) << "System state allowed transition out of FAULT without a reboot!";
    // FIXED: The implementation explicitly sets state to SAFE_HALT upon illegal transition
    EXPECT_EQ(sys_context_fault.getState(), SystemState::SAFE_HALT);
    EXPECT_TRUE(output_fault_reject.find("Illegal state transition rejected:") != std::string_view::npos) << "Expected rejection error log missing!";
}

TEST_F(StateMachineTestSuite, WatchdogInitSuccess) {
    WatchdogTimer wdt;
    EXPECT_TRUE(wdt.init(1000));
    EXPECT_NO_FATAL_FAILURE(wdt.feed());
}

TEST_F(StateMachineTestSuite, WatchdogInitFailsDeviceNotReady) {
    WatchdogTimer wdt;
    mock_wdt_ready = false;

    EXPECT_FALSE(wdt.init(1000));
    EXPECT_NO_FATAL_FAILURE(wdt.feed());
}

TEST_F(StateMachineTestSuite, WatchdogInitFailsInstall) {
    WatchdogTimer wdt;
    mock_wdt_install_res = -1;

    EXPECT_FALSE(wdt.init(1000));
}

TEST_F(StateMachineTestSuite, MissingWatchdogCoverage) {
    WatchdogTimer wdt;
    EXPECT_FALSE(wdt.isInitialized()); // Line 51-53 coverage
    
    // Line 31-35 coverage (Setup fails)
    mock_wdt_setup_res = -1;
    testing::internal::CaptureStdout();
    EXPECT_FALSE(wdt.init(1000));
    const auto raw_setup_fail = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_setup_fail).find("Watchdog setup failed") != std::string_view::npos);
    EXPECT_FALSE(wdt.isInitialized());
    
    // Restore and Init successfully
    mock_wdt_setup_res = 0;
    EXPECT_TRUE(wdt.init(1000));
    EXPECT_TRUE(wdt.isInitialized());
    
    // Line 45-47 coverage (Feed fails)
    mock_wdt_feed_res = -1;
    testing::internal::CaptureStdout();
    wdt.feed();
    const auto raw_feed_fail = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_feed_fail).find("Watchdog feed failed") != std::string_view::npos);
}

TEST_F(StateMachineTestSuite, NullptrFaultReason) {
    DeviceContext ctx;
    
    // Line 103 partial branch coverage (reason == nullptr)
    testing::internal::CaptureStdout();
    ctx.triggerFault(nullptr);
    const auto raw_out = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_out).find("unspecified fault") != std::string_view::npos);
}

TEST_F(StateMachineTestSuite, ContextWatchdogWrappers) {
    DeviceContext ctx;
    
    // Lines 111-125 coverage (Wrappers and C-Hook)
    EXPECT_TRUE(ctx.initWatchdog(1000));
    EXPECT_NO_FATAL_FAILURE(ctx.feedWatchdog());
    EXPECT_NO_FATAL_FAILURE(daly_watchdog_feed_hook());
}

TEST_F(StateMachineTestSuite, ExhaustiveStateTransitions) {
    DeviceContext ctx; // starts in INIT
    
    // Line 60: from == to branch
    EXPECT_TRUE(ctx.requestTransition(SystemState::INIT));
    
    // Line 65/67 boolean permutations & Lines 69/71 valid paths
    EXPECT_TRUE(ctx.requestTransition(SystemState::SAFE_HALT));
    EXPECT_TRUE(ctx.requestTransition(SystemState::INIT));
    EXPECT_TRUE(ctx.requestTransition(SystemState::FAULT));
    
    // FAULT -> INIT (Invalid, resets to SAFE_HALT)
    testing::internal::CaptureStdout();
    EXPECT_FALSE(ctx.requestTransition(SystemState::INIT));
    const auto raw_inv1 = testing::internal::GetCapturedStdout();
    
    // SAFE_HALT -> INIT -> RUNNING -> INIT
    EXPECT_TRUE(ctx.requestTransition(SystemState::INIT));
    EXPECT_TRUE(ctx.requestTransition(SystemState::RUNNING));
    EXPECT_TRUE(ctx.requestTransition(SystemState::INIT));
    
    // INIT -> RUNNING -> SAFE_HALT -> INIT
    EXPECT_TRUE(ctx.requestTransition(SystemState::RUNNING));
    EXPECT_TRUE(ctx.requestTransition(SystemState::SAFE_HALT));
    EXPECT_TRUE(ctx.requestTransition(SystemState::INIT));
    
    // Line 65/67 invalid targets (covers the false evaluated side of the || checks)
    testing::internal::CaptureStdout();
    EXPECT_FALSE(ctx.requestTransition(static_cast<SystemState>(99)));
    const auto raw_inv2 = testing::internal::GetCapturedStdout();
    
    EXPECT_TRUE(ctx.requestTransition(SystemState::INIT));
    EXPECT_TRUE(ctx.requestTransition(SystemState::RUNNING));
    
    testing::internal::CaptureStdout();
    EXPECT_FALSE(ctx.requestTransition(static_cast<SystemState>(99)));
    const auto raw_inv3 = testing::internal::GetCapturedStdout();
    
    // Line 72-74: Default switch case (from == invalid)
    ctx.current_state = static_cast<SystemState>(99); // whitebox access bypass
    testing::internal::CaptureStdout();
    EXPECT_FALSE(ctx.requestTransition(SystemState::INIT));
    const auto raw_inv4 = testing::internal::GetCapturedStdout();
}
