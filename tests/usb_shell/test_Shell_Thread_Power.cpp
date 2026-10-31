// test_Shell_Thread_Power.cpp
// Tests for UsbShell::process() and shell_thread(): the main loop, power-observer sleep gating (worker thread),
// and the SAFE_HALT / FAULT interlock on state-changing commands.

#include "test_usb_shell_common.h"

TEST_F(UsbShellTestSuite,ProcessAndShellThread){
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx,&battery);

    force_device_not_ready=true; testing::internal::CaptureStdout();
    shell.process();
    EXPECT_NE(testing::internal::GetCapturedStdout().find("[ERR] USB initialization failed"),std::string_view::npos);
    force_device_not_ready=false;

    mock_dtr_state=1; run_thread_once=false; mock_tx_index=0; mock_tx_buffer.fill(0);
    shell.process();
    inject_mock_uart_data("status\n");
    run_thread_once=false;
    shell.process();
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("sys_state"),std::string_view::npos);
    EXPECT_NE(out.find("med-device:~$ "),std::string_view::npos);

    mock_dtr_state=1; mock_tx_index=0;

    run_thread_once=true;

    shell.process();
    inject_mock_uart_data("\n");
    run_thread_once=false;
    shell.process();

    mock_dtr_state=0; run_thread_once=false; shell.process(); SUCCEED();

    mock_dtr_state=1; run_thread_once=false; EXPECT_NO_FATAL_FAILURE(shell_thread());
    inject_mock_uart_data("status\n"); run_thread_once=false; EXPECT_NO_FATAL_FAILURE(shell_thread());
}

TEST_F(UsbShellTestSuite, PowerObserverAndSleepGating) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);

    mock_dtr_state = 1;
    extern bool run_thread_once;
    run_thread_once = false;
    shell.process();                       // init USB + register the shell power observer

    // process() blocks inside `while (isSleeping()) k_msleep(200)` until wake-up, so run it
    // on a worker thread. g_shell_alive is only re-armed inside that loop while sleeping.
    PowerManager::getInstance().notifyBeforeSleep();
    atomic_set(&g_shell_alive, 0);
    std::thread worker([&shell]() { shell.process(); });

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (atomic_get(&g_shell_alive) == 0 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::yield();
    }
    const bool entered_sleep_loop = (atomic_get(&g_shell_alive) != 0);

    PowerManager::getInstance().notifyAfterWakeup();   // lets the worker leave the loop
    worker.join();
    EXPECT_TRUE(entered_sleep_loop) << "shell never reached its sleep loop";

    // After waking, queued input is processed normally.
    inject_mock_uart_data("status\n");
    mock_tx_index = 0;
    run_thread_once = false;
    shell.process();
    EXPECT_GT(mock_tx_index, 0u) << "Shell should process input after waking up";

    // Aborted sleep also resumes the shell without blocking.
    PowerManager::getInstance().notifyBeforeSleep();
    PowerManager::getInstance().notifySleepAborted();
    inject_mock_uart_data("log dump\n");
    mock_tx_index = 0;
    run_thread_once = false;
    shell.process();
    EXPECT_GT(mock_tx_index, 0u) << "Shell should resume if sleep is aborted";

    PowerManager::getInstance().notifyAfterWakeup();
}

TEST_F(UsbShellTestSuite, SafeHaltAndFaultInterlock) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    auto& cfg = ConfigStore::getInstance();
    uint8_t slot = 0;
    ASSERT_TRUE(cfg.findSlotByDeviceId(1001, slot));
    ASSERT_TRUE(cfg.setAlarmThreshold(slot, 80));
    ASSERT_TRUE(cfg.setInfusionRate(slot, 10));

    mock_dtr_state = 1;
    extern bool run_thread_once;
    run_thread_once = false;
    shell.process();                       // init USB

    // SAFE_HALT: diagnostics keep working (shell does not freeze) ...
    ctx.current_state = SystemState::SAFE_HALT;
    inject_mock_uart_data("status\n");
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    run_thread_once = false;
    shell.process();
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("sys_state"), std::string_view::npos);

    // ... but state-changing commands are rejected
    inject_mock_uart_data("set_rate 1001 50\n");
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    run_thread_once = false;
    shell.process();
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("SAFE_HALT/FAULT"), std::string_view::npos);

    // FAULT: second operand of the interlock condition
    ctx.current_state = SystemState::FAULT;
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate 1001 50");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("SAFE_HALT/FAULT"), std::string_view::npos);

    uint8_t rate = 0;
    ASSERT_TRUE(cfg.getInfusionRate(slot, rate));
    EXPECT_EQ(rate, 10) << "rejected command must not change the stored rate";

    // Back to RUNNING: command goes through
    ctx.current_state = SystemState::RUNNING;
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate 1001 50");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Success"), std::string_view::npos);
    ASSERT_TRUE(cfg.getInfusionRate(slot, rate));
    EXPECT_EQ(rate, 50);
}
