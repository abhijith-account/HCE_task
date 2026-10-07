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

atomic_t g_processor_alive = ATOMIC_INIT(0);
atomic_t g_producer_alive = ATOMIC_INIT(0);
atomic_t g_logger_alive = ATOMIC_INIT(0);
atomic_t g_hr_prod_alive = ATOMIC_INIT(0);
atomic_t g_disp_cons_alive = ATOMIC_INIT(0);
atomic_t g_bms_comm_alive = ATOMIC_INIT(0);
atomic_t g_batt_mon_alive = ATOMIC_INIT(0);
atomic_t g_mem_mon_alive = ATOMIC_INIT(0);
atomic_t g_shell_alive = ATOMIC_INIT(0);

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

TEST_F(StateMachineTestSuite, FaultInjectionLatchesFault) {
    DeviceContext sys_context;

    sys_context.requestTransition(SystemState::RUNNING);
    ASSERT_EQ(sys_context.getState(), SystemState::RUNNING);

    testing::internal::CaptureStdout();
    sys_context.triggerFault("Simulated I2C Bus Hard-Lock");
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);

    // triggerFault() latches FAULT (the WDT hook stops feeding in FAULT to force a
    // hardware reset); SAFE_HALT is only reached via an explicit FAULT->SAFE_HALT request.
    EXPECT_EQ(sys_context.getState(), SystemState::FAULT) << "triggerFault() did not latch the FAULT state!";
    EXPECT_TRUE(output.find("CRITICAL FAULT: Simulated I2C Bus Hard-Lock. Forcing SAFE_HALT.") != std::string_view::npos) << "Expected critical fault log missing! Actual output:  " << output;
}

TEST_F(StateMachineTestSuite, RejectsIllegalTransitions) {
    DeviceContext sys_context_halted;

    sys_context_halted.triggerFault("Test Error");
    ASSERT_EQ(sys_context_halted.getState(), SystemState::FAULT);

    // FAULT -> SAFE_HALT is the only legal exit from FAULT.
    ASSERT_TRUE(sys_context_halted.requestTransition(SystemState::SAFE_HALT));
    ASSERT_EQ(sys_context_halted.getState(), SystemState::SAFE_HALT);

    testing::internal::CaptureStdout();

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

    bool success_fault = sys_context_fault.requestTransition(SystemState::RUNNING);
    const auto raw_output_fault_reject = testing::internal::GetCapturedStdout();
    std::string_view output_fault_reject(raw_output_fault_reject);

    EXPECT_FALSE(success_fault) << "System state allowed transition out of FAULT without a reboot!";

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
    EXPECT_FALSE(wdt.isInitialized());

    mock_wdt_setup_res = -1;
    testing::internal::CaptureStdout();
    EXPECT_FALSE(wdt.init(1000));
    const auto raw_setup_fail = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_setup_fail).find("Watchdog setup failed") != std::string_view::npos);
    EXPECT_FALSE(wdt.isInitialized());

    mock_wdt_setup_res = 0;
    EXPECT_TRUE(wdt.init(1000));
    EXPECT_TRUE(wdt.isInitialized());

    mock_wdt_feed_res = -1;
    testing::internal::CaptureStdout();
    wdt.feed();
    const auto raw_feed_fail = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_feed_fail).find("Watchdog feed failed") != std::string_view::npos);
}

TEST_F(StateMachineTestSuite, NullptrFaultReason) {
    DeviceContext ctx;

    testing::internal::CaptureStdout();
    ctx.triggerFault(nullptr);
    const auto raw_out = testing::internal::GetCapturedStdout();
    EXPECT_TRUE(std::string_view(raw_out).find("unspecified fault") != std::string_view::npos);
}

TEST_F(StateMachineTestSuite, ContextWatchdogWrappers) {
    DeviceContext ctx;

    EXPECT_TRUE(ctx.initWatchdog(1000));
    EXPECT_NO_FATAL_FAILURE(ctx.feedWatchdog());
    EXPECT_NO_FATAL_FAILURE(daly_watchdog_feed_hook());
}

TEST_F(StateMachineTestSuite, ExhaustiveStateTransitions) {
    DeviceContext ctx;

    EXPECT_TRUE(ctx.requestTransition(SystemState::INIT));

    EXPECT_TRUE(ctx.requestTransition(SystemState::SAFE_HALT));
    EXPECT_TRUE(ctx.requestTransition(SystemState::INIT));
    EXPECT_TRUE(ctx.requestTransition(SystemState::FAULT));

    testing::internal::CaptureStdout();
    EXPECT_FALSE(ctx.requestTransition(SystemState::INIT));
    const auto raw_inv1 = testing::internal::GetCapturedStdout();

    EXPECT_TRUE(ctx.requestTransition(SystemState::INIT));
    EXPECT_TRUE(ctx.requestTransition(SystemState::RUNNING));
    EXPECT_TRUE(ctx.requestTransition(SystemState::INIT));

    EXPECT_TRUE(ctx.requestTransition(SystemState::RUNNING));
    EXPECT_TRUE(ctx.requestTransition(SystemState::SAFE_HALT));
    EXPECT_TRUE(ctx.requestTransition(SystemState::INIT));

    testing::internal::CaptureStdout();
    EXPECT_FALSE(ctx.requestTransition(static_cast<SystemState>(99)));
    const auto raw_inv2 = testing::internal::GetCapturedStdout();

    EXPECT_TRUE(ctx.requestTransition(SystemState::INIT));
    EXPECT_TRUE(ctx.requestTransition(SystemState::RUNNING));

    testing::internal::CaptureStdout();
    EXPECT_FALSE(ctx.requestTransition(static_cast<SystemState>(99)));
    const auto raw_inv3 = testing::internal::GetCapturedStdout();

    ctx.current_state = static_cast<SystemState>(99);
    testing::internal::CaptureStdout();
    EXPECT_FALSE(ctx.requestTransition(SystemState::INIT));
    const auto raw_inv4 = testing::internal::GetCapturedStdout();
}

TEST_F(StateMachineTestSuite, ContextWatchdogInitFails) {
    DeviceContext ctx;

    mock_wdt_ready = false;

    testing::internal::CaptureStdout();
    EXPECT_FALSE(ctx.initWatchdog(1000));
    testing::internal::GetCapturedStdout();
}

TEST_F(StateMachineTestSuite, PowerObserverCallbacks) {
    DeviceContext ctx;

    mock_wdt_ready = true;
    mock_wdt_install_res = 0;
    mock_wdt_setup_res = 0;
    EXPECT_TRUE(ctx.initWatchdog(1000));

    EXPECT_NO_FATAL_FAILURE(ctx.beforeSleep()) << "beforeSleep() crashed";
    EXPECT_NO_FATAL_FAILURE(ctx.afterWakeup()) << "afterWakeup() crashed";
    EXPECT_NO_FATAL_FAILURE(ctx.sleepAborted()) << "sleepAborted() crashed";
}

TEST_F(StateMachineTestSuite, IsLegalTransitionBranches)
{
    DeviceContext ctx;

    EXPECT_TRUE(ctx.isLegalTransition(SystemState::INIT, SystemState::INIT));
    EXPECT_TRUE(ctx.isLegalTransition(SystemState::RUNNING, SystemState::RUNNING));
    EXPECT_TRUE(ctx.isLegalTransition(SystemState::FAULT, SystemState::FAULT));
    EXPECT_TRUE(ctx.isLegalTransition(SystemState::SAFE_HALT, SystemState::SAFE_HALT));

    EXPECT_TRUE(ctx.isLegalTransition(SystemState::INIT, SystemState::RUNNING));
    EXPECT_TRUE(ctx.isLegalTransition(SystemState::INIT, SystemState::FAULT));
    EXPECT_TRUE(ctx.isLegalTransition(SystemState::INIT, SystemState::SAFE_HALT));
    EXPECT_FALSE(ctx.isLegalTransition(SystemState::INIT, static_cast<SystemState>(99)));

    EXPECT_TRUE(ctx.isLegalTransition(SystemState::RUNNING, SystemState::INIT));
    EXPECT_TRUE(ctx.isLegalTransition(SystemState::RUNNING, SystemState::FAULT));
    EXPECT_TRUE(ctx.isLegalTransition(SystemState::RUNNING, SystemState::SAFE_HALT));
    EXPECT_FALSE(ctx.isLegalTransition(SystemState::RUNNING, static_cast<SystemState>(99)));

    EXPECT_TRUE(ctx.isLegalTransition(SystemState::FAULT, SystemState::SAFE_HALT));
    EXPECT_FALSE(ctx.isLegalTransition(SystemState::FAULT, SystemState::RUNNING));

    EXPECT_TRUE(ctx.isLegalTransition(SystemState::SAFE_HALT, SystemState::INIT));
    EXPECT_FALSE(ctx.isLegalTransition(SystemState::SAFE_HALT, SystemState::FAULT));

    EXPECT_FALSE(ctx.isLegalTransition(static_cast<SystemState>(99),
                                       SystemState::INIT));
}

extern uint32_t virtual_uptime;
extern DeviceContext sys_context; // Targets the global instance used by the hook

TEST_F(StateMachineTestSuite, DalyWatchdogFeedHookComprehensive) {
    // 1. Cover the FAULT early-return branch
    sys_context.current_state = SystemState::INIT; // Reset global state
    sys_context.requestTransition(SystemState::RUNNING);
    sys_context.triggerFault("Hook Fault");
    ASSERT_EQ(sys_context.getState(), SystemState::FAULT);
    
    daly_watchdog_feed_hook(); // Exits immediately at FAULT check

    // 2. Cover the SAFE_HALT early-return branch
    sys_context.requestTransition(SystemState::SAFE_HALT);
    ASSERT_EQ(sys_context.getState(), SystemState::SAFE_HALT);
    
    daly_watchdog_feed_hook(); // Feeds and exits at SAFE_HALT check

    // 3. Cover the False branch of thread liveness (time >= 1000, threads dead)
    sys_context.requestTransition(SystemState::INIT); // Restore to normal state
    virtual_uptime += 2000; // Advance time to bypass the < 1000ms early return
    
    g_producer_alive = false; // Ensure at least one thread evaluates to false
    daly_watchdog_feed_hook(); // Reaches the LOG_DBG else-block

    // 4. Cover the True branch of thread liveness (time >= 1000, threads alive)
    virtual_uptime += 2000; // Advance time again for the next check
    
    g_producer_alive = true;
    g_processor_alive = true;
    g_logger_alive = true;
    g_hr_prod_alive = true;
    g_disp_cons_alive = true;
    g_bms_comm_alive = true;
    g_batt_mon_alive = true;
    g_mem_mon_alive = true;
    g_shell_alive = true;
    
    daly_watchdog_feed_hook(); // Reaches sys_context.feedWatchdog() inside the true block
}
