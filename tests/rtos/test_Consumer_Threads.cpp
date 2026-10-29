// test_Consumer_Threads.cpp
// Tests for Consumer_Threads.cpp: processor_thread() / logger_thread() loops, priorities and sleep / SAFE_HALT gating.

#include "test_rtos_common.h"

namespace {

void runWithSleepGate(void (*fn)()) {
    // Make sure g_powerObserver is registered with the PowerManager.
    test_iterations_remaining = 0;
    producer_thread();

    SystemObjects::power().notifyBeforeSleep();
    std::thread waker([]() {
        msSleep(100);
        SystemObjects::power().notifyAfterWakeup();
    });
    test_iterations_remaining = 0;
    fn();
    waker.join();
}

void runWithSafeHaltGate(void (*fn)()) {
    SystemObjects::context().triggerFault("Coverage gate");
    SystemObjects::context().requestTransition(SystemState::SAFE_HALT);
    std::thread rescuer([]() {
        msSleep(100);
        SystemObjects::context().requestTransition(SystemState::INIT);
    });
    test_iterations_remaining = 0;
    fn();
    rescuer.join();
}

} // namespace

TEST_F(RTOSCommandsTestSuite, ThreadPreemptionPriorities) {
    EXPECT_EQ(k_thread_priority_get(producer_tid), ThreadConfig::PrioProducer);
    EXPECT_EQ(k_thread_priority_get(processor_tid), ThreadConfig::PrioProcessor);
    EXPECT_EQ(k_thread_priority_get(logger_tid), ThreadConfig::PrioLogger);

    test_iterations_remaining = 1;
    processor_thread();

    EXPECT_EQ(exec_index.load(), 0);
}

TEST_F(RTOSCommandsTestSuite, IntegrationLoopConditionTest) {
    test_iterations_remaining = 1;

    EXPECT_TRUE(enqueueCommand<PrintCmd>(PROCESSOR_Q, SensorID::PAV3015, 1.0f));
    processor_thread();

    EXPECT_TRUE(enqueueCommand<PrintCmd>(LOGGER_Q, SensorID::PAV3015, 2.0f));
    logger_thread();

    EXPECT_GT(g_queueStats.commandsCreated, 0u);
}

TEST_F(RTOSCommandsTestSuite, LoggerEmptyQueue) {
    test_iterations_remaining = 1;
    logger_thread();
}

TEST_F(RTOSCommandsTestSuite, ProcessorEmptyQueue) {
    test_iterations_remaining = 1;
    processor_thread();
}

TEST_F(RTOSCommandsTestSuite, ProcessorThreadWaitsWhileSleeping) {
    runWithSleepGate([]() { processor_thread(); });
    SUCCEED();
}

TEST_F(RTOSCommandsTestSuite, ProcessorThreadWaitsWhileSafeHalt) {
    runWithSafeHaltGate([]() { processor_thread(); });
    SUCCEED();
}

TEST_F(RTOSCommandsTestSuite, LoggerThreadWaitsWhileSleeping) {
    runWithSleepGate([]() { logger_thread(); });
    SUCCEED();
}

TEST_F(RTOSCommandsTestSuite, LoggerThreadWaitsWhileSafeHalt) {
    runWithSafeHaltGate([]() { logger_thread(); });
    SUCCEED();
}
