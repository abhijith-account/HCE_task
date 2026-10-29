// test_Producer_Thread.cpp
// Tests for Producer_Thread.cpp: producer_thread() state cycle, sensor init retry, queue-full handling, SAFE_HALT and sleep gating.

#include "test_rtos_common.h"

TEST_F(RTOSCommandsTestSuite, ProducerHandlesSafeHalt) {
    SystemObjects::context().triggerFault("Coverage Test Halt");
    run_thread_once = true;

    testing::internal::CaptureStdout();
    producer_thread();
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    (void)output;

    EXPECT_EQ(g_queueStats.commandsCreated, 0);

    SystemObjects::context().requestTransition(SystemState::INIT);
}

TEST_F(RTOSCommandsTestSuite, ProducerThreadFullStateCycle) {
    test_iterations_remaining = 3;
    producer_thread();
    EXPECT_GT(g_queueStats.commandsCreated, 0u);
}

TEST_F(RTOSCommandsTestSuite, ProducerThreadBME280InitFailureLogsWarning) {
    g_i2c_force_errno = -110;
    test_iterations_remaining = 1;
    testing::internal::CaptureStdout();
    producer_thread();
    const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);
    g_i2c_force_errno = 0;
    EXPECT_NE(out.find("Failed to initialize BME280"), std::string_view::npos);
}

TEST_F(RTOSCommandsTestSuite, ProducerCyclesThroughAllStates) {
    test_iterations_remaining = 3;
    producer_thread();
    EXPECT_GT(g_queueStats.commandsCreated, 2u);
}

TEST_F(RTOSCommandsTestSuite, ProducerAlwaysSafeHalt) {
    SystemObjects::context().triggerFault("fault");
    SystemObjects::context().requestTransition(SystemState::SAFE_HALT);

    std::thread rescuer([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        SystemObjects::context().requestTransition(SystemState::INIT);
    });

    test_iterations_remaining = 0;
    producer_thread();

    rescuer.join();

    EXPECT_EQ(g_queueStats.commandsCreated, 0u);
}

TEST_F(RTOSCommandsTestSuite, ProducerThreadQueueFullSkipsActivity) {

    for (int i = 0; i < QueueConfig::Depth; i++) {
    EXPECT_TRUE(enqueueCommand<PreemptionTestCmd>(PROCESSOR_Q, i));
    }
    test_iterations_remaining = 1;
    producer_thread();

    EXPECT_EQ(g_queueStats.commandsCreated, QueueConfig::Depth);
}

TEST_F(RTOSCommandsTestSuite, ProducerThreadLPSPartialEnqueue) {

    for (int i = 0; i < QueueConfig::Depth - 2; i++) {
    EXPECT_TRUE(enqueueCommand<PreemptionTestCmd>(PROCESSOR_Q, i));
    }

    test_iterations_remaining = 2;
    producer_thread();

    EXPECT_EQ(k_msgq_num_free_get(PROCESSOR_Q), 0);

    EXPECT_GT(g_queueStats.commandsDropped, 0u);
}

TEST_F(RTOSCommandsTestSuite, ProducerThreadBME280InitPartialFailure) {
    g_i2c_call_counter = 0;
    g_i2c_fail_on_call_n = 2;
    g_i2c_fail_on_call_errno = -110;

    test_iterations_remaining = 1;
    testing::internal::CaptureStdout();
    producer_thread();
    const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);

    EXPECT_NE(out.find("Failed to initialize BME280"), std::string_view::npos);
    g_i2c_fail_on_call_n = 0;
}

TEST_F(RTOSCommandsTestSuite, PowerObserverIntegration) {
    // Initial run to clear any startup states
    test_iterations_remaining = 0;
    producer_thread();

    // --- Phase 1: Test Sleep & Wakeup ---
    SystemObjects::power().notifyBeforeSleep();
    g_queueStats.commandsCreated = 0;
    
    // Set to 0 so the thread exits naturally after breaking out of the sleep loop
    test_iterations_remaining = 0; 

    uint32_t commands_created_while_sleeping = 0;

    // Spawn a background waker thread to break the deadlock
    std::thread waker([&commands_created_while_sleeping]() {
        // Wait 100ms to allow producer_thread to enter its sleep loop
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Capture metric while blocked
        commands_created_while_sleeping = g_queueStats.commandsCreated;
        
        // Trigger wakeup, allowing the producer thread to exit the infinite sleep loop
        SystemObjects::power().notifyAfterWakeup();
    });

    // Run the producer (will block until waker thread signals)
    producer_thread();
    waker.join();

    EXPECT_EQ(commands_created_while_sleeping, 0u) << "Producer should bypass logic while sleeping";
    EXPECT_GT(g_queueStats.commandsCreated, 0u) << "Producer should resume logic after wakeup";

    // --- Phase 2: Test Sleep Aborted ---
    SystemObjects::power().notifyBeforeSleep();
    g_queueStats.commandsCreated = 0;
    test_iterations_remaining = 0;
    
    uint32_t commands_created_while_aborted = 0;

    // Spawn a background aborter thread
    std::thread aborter([&commands_created_while_aborted]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        commands_created_while_aborted = g_queueStats.commandsCreated;
        
        SystemObjects::power().notifySleepAborted();
    });

    // Run the producer (will block until aborter thread signals)
    producer_thread();
    aborter.join();

    EXPECT_EQ(commands_created_while_aborted, 0u) << "Producer should bypass logic while sleeping";
    EXPECT_GT(g_queueStats.commandsCreated, 0u) << "Producer should resume logic after sleep aborted";
}

TEST_F(RTOSCommandsTestSuite, ProducerThreadLPSFirstEnqueueFails) {

    for (int i = 0; i < QueueConfig::Depth - 1; i++) {
        EXPECT_TRUE(enqueueCommand<PreemptionTestCmd>(PROCESSOR_Q, i));
    }

    test_iterations_remaining = 2;
    producer_thread();

    EXPECT_EQ(k_msgq_num_free_get(PROCESSOR_Q), 0);
    EXPECT_GT(g_queueStats.commandsDropped, 0u);
}

TEST_F(RTOSCommandsTestSuite, ProducerThreadBME280InitFirstWriteFailsSecondSucceeds) {
    g_i2c_call_counter = 0;
    g_i2c_fail_on_call_n = 1;
    g_i2c_fail_on_call_errno = -110;

    test_iterations_remaining = 1;
    testing::internal::CaptureStdout();
    producer_thread();
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view out(raw_out);

    EXPECT_NE(out.find("Failed to initialize BME280"), std::string_view::npos);
    g_i2c_fail_on_call_n = 0;
}
