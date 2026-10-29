// test_Command_Core.cpp
// Tests for Command_Core.cpp: command pool, enqueue/queue statistics, ICommand id and delete.

#include "test_rtos_common.h"

TEST_F(RTOSCommandsTestSuite, CommandDispatchAndPoolCycle) {
    bool enqueued = enqueueCommand<PreemptionTestCmd>(PROCESSOR_Q, 99);
    ASSERT_TRUE(enqueued);

    ICommand* cmd = nullptr;
    ASSERT_EQ(k_msgq_get(PROCESSOR_Q, &cmd, K_NO_WAIT), 0);

    EXPECT_EQ(exec_index.load(), 0);
    cmd->execute();
    EXPECT_EQ(exec_index.load(), 1);
    EXPECT_EQ(execution_order[0], 99);

    cmd->destroy();
}

TEST_F(RTOSCommandsTestSuite, MessageQueueOverflowSafety) {
    for (int i = 0; i < QueueConfig::Depth; i++) {
        EXPECT_TRUE(enqueueCommand<PreemptionTestCmd>(PROCESSOR_Q, i));
    }

    EXPECT_FALSE(enqueueCommand<PreemptionTestCmd>(PROCESSOR_Q, 999));
    EXPECT_EQ(k_msgq_num_free_get(PROCESSOR_Q), 0);
}

TEST_F(RTOSCommandsTestSuite, ICommandOperatorDelete) {
    ASSERT_TRUE(enqueueCommand<PreemptionTestCmd>(PROCESSOR_Q, 42));

    ICommand* cmd = nullptr;
    ASSERT_EQ(k_msgq_get(PROCESSOR_Q, &cmd, K_NO_WAIT), 0);

    ICommand::operator delete(cmd);
}

TEST_F(RTOSCommandsTestSuite, MemoryPoolExhaustion) {
    std::array<ICommand*, 128> cmds{};
    size_t count = 0;
    while (count < cmds.size()) {
        void *mem = allocateCommandMemory();
        if (!mem) break;
        cmds[count++] = new(mem) PreemptionTestCmd(0);
    }

    EXPECT_GT(g_queueStats.commandsDropped, 0u);

    for(size_t i = 0; i < count; i++) {
        cmds[i]->destroy();
    }
}

TEST_F(RTOSCommandsTestSuite, EnqueueRawFailure) {
    for(int i = 0; i < QueueConfig::Depth; i++) {
        EXPECT_TRUE(enqueueCommand<PreemptionTestCmd>(PROCESSOR_Q, i));
    }

    void *mem = allocateCommandMemory();
    auto *cmd = new(mem) PreemptionTestCmd(100);

    EXPECT_FALSE(enqueueCommandRaw(PROCESSOR_Q, cmd));
    cmd->destroy();
}

TEST_F(RTOSCommandsTestSuite, LoggerPeakDepthUpdated) {
    EXPECT_TRUE(enqueueCommand<PreemptionTestCmd>(LOGGER_Q,1));
    EXPECT_EQ(g_queueStats.loggerPeakDepth,1u);
}

TEST_F(RTOSCommandsTestSuite, CommandIdsIncrease) {
    auto *m1 = allocateCommandMemory();
    auto *m2 = allocateCommandMemory();

    auto *c1 = new(m1) PreemptionTestCmd(1);
    auto *c2 = new(m2) PreemptionTestCmd(2);

    EXPECT_LT(c1->command_id, c2->command_id);

    c1->destroy();
    c2->destroy();
}

TEST_F(RTOSCommandsTestSuite, PoolReuse) {
    auto *m = allocateCommandMemory();
    auto *c = new(m) PreemptionTestCmd(1);
    c->destroy();
    EXPECT_NE(allocateCommandMemory(), nullptr);
}

TEST_F(RTOSCommandsTestSuite, EnqueueRawPeakDepthNotUpdatedIfLower) {
    EXPECT_TRUE(enqueueCommand<PreemptionTestCmd>(PROCESSOR_Q, 1));
    EXPECT_EQ(g_queueStats.processorPeakDepth, 1u);

    ICommand* cmd;
    k_msgq_get(PROCESSOR_Q, &cmd, K_NO_WAIT);
    cmd->destroy();

    EXPECT_TRUE(enqueueCommand<PreemptionTestCmd>(PROCESSOR_Q, 2));
    EXPECT_EQ(g_queueStats.processorPeakDepth, 1u);
}

TEST_F(RTOSCommandsTestSuite, EnqueueRawLoggerPeakDepthNotUpdatedIfLower) {
    EXPECT_TRUE(enqueueCommand<PreemptionTestCmd>(LOGGER_Q, 1));
    EXPECT_EQ(g_queueStats.loggerPeakDepth, 1u);

    ICommand* cmd = nullptr;
    ASSERT_EQ(k_msgq_get(LOGGER_Q, &cmd, K_NO_WAIT), 0);
    cmd->destroy();

    EXPECT_TRUE(enqueueCommand<PreemptionTestCmd>(LOGGER_Q, 2));
    EXPECT_EQ(g_queueStats.loggerPeakDepth, 1u);
}
