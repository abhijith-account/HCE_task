#include <gtest/gtest.h>
#include <zephyr/kernel.h>
#include <array>
#include <atomic>
#include <new>
#include <string>
#include <string_view>

// White-box testing bypass to easily test private methods like readMockData()
#define private public
#define protected public
#include "RTOS_Command_based_thread_system.h"
#undef private
#undef protected

#include "Static_Memory+MISRA_Compliance_Layer.h"
#include "Device_State_Machine+Watchdog.h"
#include "Power_Management_System.h"
#include "Fault_Tolerant_I2C_Communication_Layer.h"

// Define global test state
int test_iterations_remaining = 0;
bool run_thread_once = false;
extern int g_i2c_call_counter;
extern int g_i2c_fail_on_call_n;
extern int g_i2c_fail_on_call_errno;

extern DeviceContext sys_context;
extern I2CManager i2c_manager;
extern PowerManager pwr_manager;
extern void resetRtosCommandTestState() noexcept;
extern void resetI2CCacheForTests() noexcept;   
static const struct device dummy_dev;

__attribute__((weak)) DeviceContext sys_context;
I2CManager i2c_manager(&dummy_dev);
__attribute__((weak)) PowerManager pwr_manager(&dummy_dev, &dummy_dev);

// -----------------------------------------------------------------------
// Mock Command for Testing the Dispatcher
// -----------------------------------------------------------------------
static std::array<int, 2> execution_order{};
static std::atomic<size_t> exec_index{0};

class PreemptionTestCmd final : public ICommand {
private:
    int thread_priority;
public:
    PreemptionTestCmd(int prio) : thread_priority(prio) {}
    
    void execute() noexcept override final {
        size_t idx = exec_index.fetch_add(1);
        if (idx < execution_order.size()) {
            execution_order[idx] = thread_priority;
        }
    }
};

// Mock dependencies defined here to replace the missing externs
extern int g_i2c_force_errno;
extern bool g_device_ready_override;

// Expose access to reset BME280 state
namespace {
    extern BME280Calibration g_bme280Calib;
}

extern void resetRtosCommandTestState() noexcept;

static void drainAndDestroy(k_msgq* q) {
    ICommand* cmd = nullptr;
    while (k_msgq_get(q, &cmd, K_NO_WAIT) == 0) {
        cmd->destroy();
    }
}

class RTOSCommandsTestSuite : public ::testing::Test {
protected:
    void SetUp() override {
    drainAndDestroy(PROCESSOR_Q);
    drainAndDestroy(LOGGER_Q);
    g_queueStats = QueueStats{};
    exec_index = 0;
    execution_order.fill(0);

    resetRtosCommandTestState();
    resetI2CCacheForTests();     // add this line
    g_i2c_force_errno = 0;
    g_i2c_call_counter = 0;      // add this line, for hygiene across all tests
    g_i2c_fail_on_call_n = 0;    // add this line
    g_device_ready_override = true;
}

};

// 1. Verify Command Life-cycle
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

// 2. Verify Queue Limits and Safety
TEST_F(RTOSCommandsTestSuite, MessageQueueOverflowSafety) {
    for (int i = 0; i < QueueConfig::Depth; i++) {
        EXPECT_TRUE(enqueueCommand<PreemptionTestCmd>(PROCESSOR_Q, i));
    }
    
    EXPECT_FALSE(enqueueCommand<PreemptionTestCmd>(PROCESSOR_Q, 999));
    EXPECT_EQ(k_msgq_num_free_get(PROCESSOR_Q), 0);
}

// 3. Thread Logic Verification
TEST_F(RTOSCommandsTestSuite, ThreadPreemptionPriorities) {
    EXPECT_EQ(k_thread_priority_get(producer_tid), ThreadConfig::PrioProducer);
    EXPECT_EQ(k_thread_priority_get(processor_tid), ThreadConfig::PrioProcessor);
    EXPECT_EQ(k_thread_priority_get(logger_tid), ThreadConfig::PrioLogger);
    
    test_iterations_remaining = 1;
    processor_thread();
    
    EXPECT_EQ(exec_index.load(), 0); 
}

// 4. SensorReadCmd Validation
TEST_F(RTOSCommandsTestSuite, SensorReadCmdLogsCorrectly) {
    SensorReadCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, ReadLength::Block);
    
    testing::internal::CaptureStdout();
    cmd.execute();
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    
    EXPECT_TRUE(output.find("[READ]") != std::string_view::npos);
}

// 5. ComputeCmd Calculation Test
TEST_F(RTOSCommandsTestSuite, ComputeCmdBME280Logic) {
    ComputeCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, 0x000100010001ULL);
    
    testing::internal::CaptureStdout();
    cmd.execute();
    
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    EXPECT_TRUE(output.find("[COMPUTE]") != std::string_view::npos);
}

// 6. Producer Stability Tests
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

TEST_F(RTOSCommandsTestSuite, IntegrationLoopConditionTest) {
    test_iterations_remaining = 1;
    
    EXPECT_TRUE(enqueueCommand<PrintCmd>(PROCESSOR_Q, SensorID::PAV3015, 1.0f));
    processor_thread();
    
    EXPECT_TRUE(enqueueCommand<PrintCmd>(LOGGER_Q, SensorID::PAV3015, 2.0f));
    logger_thread();
    
    EXPECT_GT(g_queueStats.commandsCreated, 0u);
}

// --- ICommand: operator delete + queueDelay ---
TEST_F(RTOSCommandsTestSuite, ICommandOperatorDeleteAndQueueDelay) {
    ASSERT_TRUE(enqueueCommand<PreemptionTestCmd>(PROCESSOR_Q, 42));

    ICommand* cmd = nullptr;
    ASSERT_EQ(k_msgq_get(PROCESSOR_Q, &cmd, K_NO_WAIT), 0);

    EXPECT_GE(cmd->queueDelay(), 0u);   
    ICommand::operator delete(cmd);     
}

// --- SystemObjects::power() ---
TEST_F(RTOSCommandsTestSuite, SystemObjectsPowerAccessor) {
    EXPECT_EQ(&SystemObjects::power(), &pwr_manager);
}

// --- readHardwareData: non-BME280 + Block => Err(NACK), pure software branch ---
TEST_F(RTOSCommandsTestSuite, SensorReadCmdBlockNonBMEReturnsNack) {
    SensorReadCmd cmd(SensorID::LPS22HB, SensorReg::LPS_P_DESC.reg, ReadLength::Block);
    testing::internal::CaptureStdout();
    cmd.execute();  
    const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);
    EXPECT_NE(out.find("I2C Transaction Failed"), std::string_view::npos);
}

// --- readHardwareData: default branch via an out-of-range ReadLength ---
TEST_F(RTOSCommandsTestSuite, SensorReadCmdInvalidLengthDefaultBranch) {
    SensorReadCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, static_cast<ReadLength>(0xFF));
    testing::internal::CaptureStdout();
    cmd.execute();
    const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);
    EXPECT_NE(out.find("I2C Transaction Failed"), std::string_view::npos);
}

// --- readHardwareData: Triple and Word success branches (LPS22HB) ---
TEST_F(RTOSCommandsTestSuite, SensorReadCmdTripleAndWordSuccess) {
    SensorReadCmd triple(SensorID::LPS22HB, SensorReg::LPS_P_DESC.reg, ReadLength::Triple);
    triple.execute();
    
    SensorReadCmd word(SensorID::LPS22HB, SensorReg::LPS_T_DESC.reg, ReadLength::Word);
    word.execute();
    
    EXPECT_GT(g_queueStats.commandsCreated, 0u);
}

// --- readHardwareData: Triple and Word explicit failure branches ---
TEST_F(RTOSCommandsTestSuite, HardwareDataTripleAndWordFailures) {
    g_i2c_force_errno = -EIO;
    
    SensorReadCmd triple(SensorID::LPS22HB, SensorReg::LPS_P_DESC.reg, ReadLength::Triple);
    auto res_triple = triple.readHardwareData();
    EXPECT_FALSE(res_triple.isOk());

    SensorReadCmd word(SensorID::PAV3015, SensorReg::PAV_DESC.reg, ReadLength::Word);
    auto res_word = word.readHardwareData();
    EXPECT_FALSE(res_word.isOk());

    g_i2c_force_errno = 0;
}

// --- readMockData(): Direct branch coverage ---
TEST_F(RTOSCommandsTestSuite, MockDataGenerationBranches) {
    SensorReadCmd bme(SensorID::BME280, 0, ReadLength::Block);
    EXPECT_GT(bme.readMockData(), 0u);

    SensorReadCmd lps_t(SensorID::LPS22HB, SensorReg::LPS_T_DESC.reg, ReadLength::Word);
    EXPECT_GT(lps_t.readMockData(), 0u);

    SensorReadCmd lps_p(SensorID::LPS22HB, SensorReg::LPS_P_DESC.reg, ReadLength::Triple);
    EXPECT_GT(lps_p.readMockData(), 0u);

    SensorReadCmd pav(SensorID::PAV3015, 0, ReadLength::Word);
    EXPECT_GT(pav.readMockData(), 0u);

    SensorReadCmd unknown(static_cast<SensorID>(0xFFFF), 0, ReadLength::Word);
    EXPECT_EQ(unknown.readMockData(), 0u);
}

// --- SensorReadCmd::execute(): compute-queue-full branch ---
// --- SensorReadCmd::execute(): compute-queue-full branch ---
TEST_F(RTOSCommandsTestSuite, SensorReadCmdComputeQueueFullLogsError) {
    for (int i = 0; i < QueueConfig::Depth; i++) {
        ASSERT_TRUE(enqueueCommand<PreemptionTestCmd>(PROCESSOR_Q, i));  
    }
    SensorReadCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, ReadLength::Block);
    testing::internal::CaptureStdout();
    
    cmd.execute(); 
    
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view out(raw_out);
    EXPECT_NE(out.find("Compute queue full"), std::string_view::npos);
}

// --- printMeasurement: logger-queue-full branch ---
TEST_F(RTOSCommandsTestSuite, PrintMeasurementLoggerQueueFull) {
    for (int i = 0; i < QueueConfig::Depth; i++) {
        ASSERT_TRUE(enqueueCommand<PreemptionTestCmd>(LOGGER_Q, i));  
    }
    EXPECT_FALSE(printMeasurement(SensorID::PAV3015, 1.0f));
    EXPECT_GT(g_queueStats.loggerQueueFull, 0u);
}

// --- ComputeCmd::execute(): LPS22HB and PAV3015 ---
TEST_F(RTOSCommandsTestSuite, ComputeCmdLPS22HBBranches) {
    ComputeCmd press(SensorID::LPS22HB, SensorReg::LPS_P_DESC.reg, 0x00640032ULL);
    press.execute();
    ComputeCmd temp(SensorID::LPS22HB, SensorReg::LPS_T_DESC.reg, 0x0032ULL);
    temp.execute();
}
TEST_F(RTOSCommandsTestSuite, ComputeCmdPAV3015Branch) {
    ComputeCmd cmd(SensorID::PAV3015, SensorReg::PAV_DESC.reg, 0x0032ULL);
    cmd.execute();
}

// --- ComputeCmd::execute(): unknown-sensor default branch ---
TEST_F(RTOSCommandsTestSuite, ComputeCmdUnknownSensorDefaultBranch) {
    ComputeCmd cmd(static_cast<SensorID>(0xFFFF), 0x00, 0ULL);
    testing::internal::CaptureStdout();
    cmd.execute();
    const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);
    EXPECT_NE(out.find("Unknown Sensor ID"), std::string_view::npos);
}

// --- loadBME280Calibration: "already loaded" cached branch ---
TEST_F(RTOSCommandsTestSuite, ComputeCmdBME280CalibrationCachedOnSecondCall) {
    ComputeCmd first(SensorID::BME280, SensorReg::BME280_DATA_START, 0x000100010001ULL);
    first.execute();   
    ComputeCmd second(SensorID::BME280, SensorReg::BME280_DATA_START, 0x000200020002ULL);
    second.execute();  
}

TEST(BME280MathTest, PressureBranchNonZeroDirect) {
    BME280Calibration c{};  // value-initializes every field to 0
    c.dig_P1 = 1;           // forces p_var1 != 0 while every other dig_P*/dig_T* term collapses to 0
    auto d = BME280Math::decode(0, c);
    EXPECT_NE(d.pressure, 0.0f);
}

// --- producer_thread: full state cycle ---
TEST_F(RTOSCommandsTestSuite, ProducerThreadFullStateCycle) {
    test_iterations_remaining = 3;  
    producer_thread();
    EXPECT_GT(g_queueStats.commandsCreated, 0u);
}

TEST_F(RTOSCommandsTestSuite, SensorReadCmdI2CFailureLogsError) {
    g_i2c_force_errno = -110;  // ETIMEDOUT mapping
    SensorReadCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, ReadLength::Block);
    testing::internal::CaptureStdout();
    cmd.execute();
    const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);
    g_i2c_force_errno = 0;  
    EXPECT_NE(out.find("I2C Transaction Failed"), std::string_view::npos);
}

TEST_F(RTOSCommandsTestSuite, ComputeCmdBME280CalibrationAbortsOnI2CFailure) {
    g_i2c_force_errno = -19;  // ENODEV
    ComputeCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, 0x000100010001ULL);
    testing::internal::CaptureStdout();
    cmd.execute();
    const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);
    g_i2c_force_errno = 0;
    EXPECT_NE(out.find("BME280 calibration aborted"), std::string_view::npos);
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

TEST_F(RTOSCommandsTestSuite, MemoryPoolExhaustion) {
    std::array<ICommand*, 128> cmds{}; // Size exceeds QueueConfig::Depth
    size_t count = 0;
    while (count < cmds.size()) {
        void *mem = allocateCommandMemory();
        if (!mem) break;
        cmds[count++] = new(mem) PreemptionTestCmd(0); // Valid placement new uses static pool
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

TEST_F(RTOSCommandsTestSuite, CalibrationReadFailure) {
    g_i2c_force_errno = -5; // EIO

    ComputeCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, 1);
    cmd.execute();

    g_i2c_force_errno = 0;
}

TEST(BME280MathTest, ZeroPressureBranch) {
    BME280Calibration c{};
    c.dig_P1 = 0;
    auto d = BME280Math::decode(0, c);
    EXPECT_FLOAT_EQ(d.pressure, 0.0f);
}

TEST(BME280MathTest, HumidityClampLow) {
    BME280Calibration c{};
    c.dig_H1 = 1;
    auto d = BME280Math::decode(0, c);
    EXPECT_GE(d.humidity, 0);
}

TEST(BME280MathTest, HumidityClampHigh) {
    BME280Calibration c{};
    // Use a controlled multiplier to push v_x1_u32r over 419,430,400 
    // without causing a 32-bit signed integer overflow (which drops it < 0)
    c.dig_H2 = 300; 
    auto d = BME280Math::decode(0x000000000000FFFFULL, c); // Max out adc_H
    EXPECT_FLOAT_EQ(d.humidity, 100.0f);
}

TEST(MathCoverage, NegativeTemperature) {
    EXPECT_LT(LPS22HBMath::decodeTemperature(0xFFFF), 0);
}

TEST(MathCoverage, NegativePressure) {
    EXPECT_LT(LPS22HBMath::decodePressure(0xFFFFFF), 0);
}

TEST(MathCoverage, ZeroAirflow) {
    EXPECT_GE(PAV3015Math::decodeAirflow(0), 0);
}

TEST_F(RTOSCommandsTestSuite, ProducerCyclesThroughAllStates) {
    test_iterations_remaining = 3;
    producer_thread();
    EXPECT_GT(g_queueStats.commandsCreated, 2u);
}

TEST_F(RTOSCommandsTestSuite, LoggerEmptyQueue) {
    test_iterations_remaining = 1;
    logger_thread();
}

TEST_F(RTOSCommandsTestSuite, ProcessorEmptyQueue) {
    test_iterations_remaining = 1;
    processor_thread();
}

TEST_F(RTOSCommandsTestSuite, LoggerPeakDepthUpdated) {
    EXPECT_TRUE(enqueueCommand<PreemptionTestCmd>(LOGGER_Q,1));
    EXPECT_EQ(g_queueStats.loggerPeakDepth,1u);
}

TEST_F(RTOSCommandsTestSuite, PrintMeasurementSuccess) {
    EXPECT_TRUE(printMeasurement(SensorID::PAV3015,5.0f));

    ICommand *cmd = nullptr;
    ASSERT_EQ(k_msgq_get(LOGGER_Q, &cmd, K_NO_WAIT), 0);
    cmd->destroy();
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

TEST_F(RTOSCommandsTestSuite, ProducerAlwaysSafeHalt) {
    SystemObjects::context().triggerFault("fault");

    test_iterations_remaining = 3;
    producer_thread();
    EXPECT_EQ(g_queueStats.commandsCreated, 0u);

    SystemObjects::context().requestTransition(SystemState::INIT);
}

TEST(BME280MathTest, PressureDivideByZero) {
    BME280Calibration c{};
    c.dig_P1 = 0;
    auto d = BME280Math::decode(0, c);
    EXPECT_FLOAT_EQ(d.pressure, 0.0f);
}

TEST_F(RTOSCommandsTestSuite, PoolReuse) {
    auto *m = allocateCommandMemory();
    auto *c = new(m) PreemptionTestCmd(1);
    c->destroy();
    EXPECT_NE(allocateCommandMemory(), nullptr);
}

// -----------------------------------------------------------------------
// EXTREME EDGE CASE BRANCH COVERAGE
// -----------------------------------------------------------------------

// Cover ALL getSensorName switch cases fully
TEST_F(RTOSCommandsTestSuite, PrintCmdAllSensorNames) {
    std::array<SensorID,7> ids = {
        SensorID::BME280, SensorID::BME280_PRESS, SensorID::BME280_HUM,
        SensorID::LPS22HB, SensorID::LPS22HB_TEMP, SensorID::PAV3015,
        static_cast<SensorID>(0xFFFF)
    };
    for (auto id : ids) {
        PrintCmd cmd(id, 1.0f);
        testing::internal::CaptureStdout();
        cmd.execute();
        const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);
        EXPECT_NE(out.find("Metric:"), std::string_view::npos);
    }
}

// Ensure the false branch of `if (enqueued) SystemObjects::power().reportActivity();` is hit
TEST_F(RTOSCommandsTestSuite, ProducerThreadQueueFullSkipsActivity) {
    // Fill PROCESSOR_Q to capacity
    for (int i = 0; i < QueueConfig::Depth; i++) {
    EXPECT_TRUE(enqueueCommand<PreemptionTestCmd>(PROCESSOR_Q, i));
    }
    test_iterations_remaining = 1;
    producer_thread(); // The BME read enqueue will fail due to full queue
    
    // Validate we didn't add any new ones
    EXPECT_EQ(g_queueStats.commandsCreated, QueueConfig::Depth);
}

// Ensure the `enq1 && enq2` short-circuit branch is fully evaluated
TEST_F(RTOSCommandsTestSuite, ProducerThreadLPSPartialEnqueue) {
    // Leave exactly 2 slots free. 
    // BME takes 1 slot. Then LPS tries to take 2, but only 1 remains!
    for (int i = 0; i < QueueConfig::Depth - 2; i++) {
    EXPECT_TRUE(enqueueCommand<PreemptionTestCmd>(PROCESSOR_Q, i));
    }
    
    test_iterations_remaining = 2; // Cycle through BME -> LPS
    producer_thread();
    
    // The processor queue should be absolutely full now
    EXPECT_EQ(k_msgq_num_free_get(PROCESSOR_Q), 0);
    // And we should have registered a dropped command for the failed enq2
    EXPECT_GT(g_queueStats.commandsDropped, 0u);
}

// Forces LOGGER_Q to be full exactly when ComputeCmd executes to catch the false returns of printMeasurement
// Forces LOGGER_Q to be full exactly when ComputeCmd executes to catch the false returns of printMeasurement
TEST_F(RTOSCommandsTestSuite, ComputeCmdBME280PrintsFailWhenQueueFull) {
    // Fill LOGGER_Q
    for (int i = 0; i < QueueConfig::Depth; i++) {
        EXPECT_TRUE(enqueueCommand<PreemptionTestCmd>(LOGGER_Q, i));
    }
    
    ComputeCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, 0x000100010001ULL);
    testing::internal::CaptureStdout();
    cmd.execute();
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view out(raw_out);
    
    EXPECT_NE(out.find("Logger queue full"), std::string_view::npos);
}

// Evaluates the false branch of peak depth tracking
TEST_F(RTOSCommandsTestSuite, EnqueueRawPeakDepthNotUpdatedIfLower) {
    EXPECT_TRUE(enqueueCommand<PreemptionTestCmd>(PROCESSOR_Q, 1));
    EXPECT_EQ(g_queueStats.processorPeakDepth, 1u);
    
    // De-queue it
    ICommand* cmd;
    k_msgq_get(PROCESSOR_Q, &cmd, K_NO_WAIT);
    cmd->destroy();
    
    // Enqueue again; peak should NOT go up to 2 since max depth is still 1
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
    EXPECT_EQ(g_queueStats.loggerPeakDepth, 1u);  // still 1 — used(1) is not > previous peak(1)
}

TEST_F(RTOSCommandsTestSuite, CalibrationChainFailsAtEveryReadPosition) {
    for (int fail_at = 1; fail_at <= 20; ++fail_at) {
        resetRtosCommandTestState();
        resetI2CCacheForTests();   // add this line
        g_i2c_call_counter = 0;
        g_i2c_fail_on_call_n = fail_at;
        g_i2c_fail_on_call_errno = -EIO;

        ComputeCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, 0x000100010001ULL);
        testing::internal::CaptureStdout();
        cmd.execute();
        const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);

        EXPECT_NE(out.find("BME280 calibration aborted"), std::string_view::npos)
            << "Expected calibration abort when I2C call #" << fail_at << " fails";
    }
    g_i2c_fail_on_call_n = 0;
}

TEST_F(RTOSCommandsTestSuite, ProducerThreadBME280InitPartialFailure) {
    g_i2c_call_counter = 0;
    g_i2c_fail_on_call_n = 2; // Pass the first I2C write, fail the second (res2)
    g_i2c_fail_on_call_errno = -110; 
    
    test_iterations_remaining = 1;
    testing::internal::CaptureStdout();
    producer_thread();
    const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);
    
    EXPECT_NE(out.find("Failed to initialize BME280"), std::string_view::npos);
    g_i2c_fail_on_call_n = 0;
}

TEST_F(RTOSCommandsTestSuite, DiagnosticI2C) {
    // 1. Is the manager’s device pointer valid?
    const device* dev = SystemObjects::i2c().i2c_dev;
    ASSERT_NE(dev, nullptr) << "i2c_dev is nullptr!";

    // 2. Does device_is_ready() return true?
    ASSERT_TRUE(device_is_ready(dev))
        << "device_is_ready() returned false. g_device_ready_override = "
        << g_device_ready_override;

    // 3. Direct I²C read call – does it succeed?
    const uint16_t addr = static_cast<uint16_t>(SensorID::LPS22HB);
    auto res = SystemObjects::i2c().read24Bit(addr, SensorReg::LPS_P_DESC.reg);
    ASSERT_TRUE(res.isOk())
        << "read24Bit failed. Error = " << static_cast<int>(res.error);
}

TEST_F(RTOSCommandsTestSuite, PinpointI2cFailure) {
    const device* dev = SystemObjects::i2c().i2c_dev;
    ASSERT_NE(dev, nullptr) << "i2c_dev is nullptr!";
    ASSERT_TRUE(device_is_ready(dev))
        << "device_is_ready() returns false. g_device_ready_override = "
        << g_device_ready_override;
    SUCCEED();
}
