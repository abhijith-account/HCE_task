#include <gtest/gtest.h>
#include <zephyr/kernel.h>
#include <array>
#include <atomic>
#include <new>
#include <string>
#include <string_view>

#define private public
#define protected public
#include "RTOS_Command_based_thread_system.h"
#undef private
#undef protected

#include "Static_Memory+MISRA_Compliance_Layer.h"
#include "Device_State_Machine+Watchdog.h"
#include "Power_Management_System.h"
#include "Fault_Tolerant_I2C_Communication_Layer.h"

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
__attribute__((weak)) PowerManager pwr_manager;

static std::array<int, 2> execution_order{};
static std::atomic<size_t> exec_index{0};

uint16_t g_i2c_mock_word_val = 0; 
extern int g_i2c_force_errno;
extern "C" {
    int i2c_burst_read(const struct device *dev, uint16_t dev_addr,
                        uint8_t start_addr, uint8_t *buf, uint32_t num_bytes) {
        ++g_i2c_call_counter;
        if (g_i2c_force_errno != 0) return g_i2c_force_errno;
        if (g_i2c_fail_on_call_n != 0 && g_i2c_call_counter == g_i2c_fail_on_call_n) {
            return g_i2c_fail_on_call_errno;
        }
        if (buf && num_bytes == 2) {
            buf[0] = g_i2c_mock_word_val & 0xFF;
            buf[1] = (g_i2c_mock_word_val >> 8) & 0xFF;
        } else if (buf && num_bytes > 0) {
            memset(buf, 0, num_bytes);   // Triple/Block reads unaffected -- same as before
        }
        return 0;
    }
}
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
extern bool g_device_ready_override;

namespace {
    extern BME280Calibration g_bme280Calib;
}

extern void resetRtosCommandTestState() noexcept;
extern void resetSensorReadCmdLastValsForTest() noexcept;

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
    resetI2CCacheForTests();
    resetSensorReadCmdLastValsForTest(); 
    g_i2c_force_errno = 0;
    g_i2c_call_counter = 0;
    g_i2c_fail_on_call_n = 0;
    g_device_ready_override = true;
    g_i2c_mock_word_val = 0; 
}

};

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

TEST_F(RTOSCommandsTestSuite, ThreadPreemptionPriorities) {
    EXPECT_EQ(k_thread_priority_get(producer_tid), ThreadConfig::PrioProducer);
    EXPECT_EQ(k_thread_priority_get(processor_tid), ThreadConfig::PrioProcessor);
    EXPECT_EQ(k_thread_priority_get(logger_tid), ThreadConfig::PrioLogger);

    test_iterations_remaining = 1;
    processor_thread();

    EXPECT_EQ(exec_index.load(), 0);
}

TEST_F(RTOSCommandsTestSuite, SensorReadCmdLogsCorrectly) {
    SensorReadCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, ReadLength::Block);

    testing::internal::CaptureStdout();
    cmd.execute();
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);

    EXPECT_TRUE(output.find("[READ]") != std::string_view::npos);
}

TEST_F(RTOSCommandsTestSuite, ComputeCmdBME280Logic) {
    ComputeCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, 0x000100010001ULL);

    testing::internal::CaptureStdout();
    cmd.execute();

    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    EXPECT_TRUE(output.find("[COMPUTE]") != std::string_view::npos);
}

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

TEST_F(RTOSCommandsTestSuite, ICommandOperatorDeleteAndQueueDelay) {
    ASSERT_TRUE(enqueueCommand<PreemptionTestCmd>(PROCESSOR_Q, 42));

    ICommand* cmd = nullptr;
    ASSERT_EQ(k_msgq_get(PROCESSOR_Q, &cmd, K_NO_WAIT), 0);

    EXPECT_GE(cmd->queueDelay(), 0u);
    ICommand::operator delete(cmd);
}

TEST_F(RTOSCommandsTestSuite, SystemObjectsPowerAccessor) {
    EXPECT_EQ(&SystemObjects::power(), &PowerManager::getInstance());
}

TEST_F(RTOSCommandsTestSuite, SensorReadCmdBlockNonBMEReturnsNack) {
    SensorReadCmd cmd(SensorID::LPS22HB, SensorReg::LPS_P_DESC.reg, ReadLength::Block);
    testing::internal::CaptureStdout();
    cmd.execute();
    const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);
    EXPECT_NE(out.find("I2C Transaction Failed"), std::string_view::npos);
}

TEST_F(RTOSCommandsTestSuite, SensorReadCmdInvalidLengthDefaultBranch) {
    SensorReadCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, static_cast<ReadLength>(0xFF));
    testing::internal::CaptureStdout();
    cmd.execute();
    const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);
    EXPECT_NE(out.find("I2C Transaction Failed"), std::string_view::npos);
}

TEST_F(RTOSCommandsTestSuite, SensorReadCmdTripleAndWordSuccess) {
    SensorReadCmd triple(SensorID::LPS22HB, SensorReg::LPS_P_DESC.reg, ReadLength::Triple);
    triple.execute();

    SensorReadCmd word(SensorID::LPS22HB, SensorReg::LPS_T_DESC.reg, ReadLength::Word);
    word.execute();

    EXPECT_GT(g_queueStats.commandsCreated, 0u);
}

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

TEST_F(RTOSCommandsTestSuite, PrintMeasurementLoggerQueueFull) {
    for (int i = 0; i < QueueConfig::Depth; i++) {
        ASSERT_TRUE(enqueueCommand<PreemptionTestCmd>(LOGGER_Q, i));
    }
    EXPECT_FALSE(printMeasurement(SensorID::PAV3015, 1.0f));
    EXPECT_GT(g_queueStats.loggerQueueFull, 0u);
}

TEST_F(RTOSCommandsTestSuite, ComputeCmdLPS22HBBranches) {

    ComputeCmd temp_early(SensorID::LPS22HB, SensorReg::LPS_T_DESC.reg, 0x0032ULL);
    temp_early.execute();

    ComputeCmd press(SensorID::LPS22HB, SensorReg::LPS_P_DESC.reg, 0x00640032ULL);
    press.execute();

    ComputeCmd temp_late(SensorID::LPS22HB, SensorReg::LPS_T_DESC.reg, 0x0032ULL);
    temp_late.execute();

    ComputeCmd unknown_reg(SensorID::LPS22HB, 0xFF, 0x00ULL);
    unknown_reg.execute();
}
TEST_F(RTOSCommandsTestSuite, ComputeCmdPAV3015Branch) {
    ComputeCmd cmd(SensorID::PAV3015, SensorReg::PAV_DESC.reg, 0x0032ULL);
    cmd.execute();
}

TEST_F(RTOSCommandsTestSuite, ComputeCmdUnknownSensorDefaultBranch) {
    ComputeCmd cmd(static_cast<SensorID>(0xFFFF), 0x00, 0ULL);
    testing::internal::CaptureStdout();
    cmd.execute();
    const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);
    EXPECT_NE(out.find("Unknown Sensor ID"), std::string_view::npos);
}

TEST_F(RTOSCommandsTestSuite, ComputeCmdBME280CalibrationCachedOnSecondCall) {
    ComputeCmd first(SensorID::BME280, SensorReg::BME280_DATA_START, 0x000100010001ULL);
    first.execute();
    ComputeCmd second(SensorID::BME280, SensorReg::BME280_DATA_START, 0x000200020002ULL);
    second.execute();
}

TEST(BME280MathTest, PressureBranchNonZeroDirect) {
    BME280Calibration c{};
    c.dig_P1 = 1;
    auto d = BME280Math::decode(0, c);
    EXPECT_NE(d.pressure, 0.0f);
}

TEST_F(RTOSCommandsTestSuite, ProducerThreadFullStateCycle) {
    test_iterations_remaining = 3;
    producer_thread();
    EXPECT_GT(g_queueStats.commandsCreated, 0u);
}

TEST_F(RTOSCommandsTestSuite, SensorReadCmdI2CFailureLogsError) {
    g_i2c_force_errno = -110;
    SensorReadCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, ReadLength::Block);
    testing::internal::CaptureStdout();
    cmd.execute();
    const auto raw_out = testing::internal::GetCapturedStdout();
std::string_view out(raw_out);
    g_i2c_force_errno = 0;
    EXPECT_NE(out.find("I2C Transaction Failed"), std::string_view::npos);
}

TEST_F(RTOSCommandsTestSuite, ComputeCmdBME280CalibrationAbortsOnI2CFailure) {
    g_i2c_force_errno = -19;
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

TEST_F(RTOSCommandsTestSuite, CalibrationReadFailure) {
    g_i2c_force_errno = -5;

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

    c.dig_H2 = 300;
    auto d = BME280Math::decode(0x000000000000FFFFULL, c);
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

TEST_F(RTOSCommandsTestSuite, ComputeCmdBME280PrintsFailWhenQueueFull) {

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

TEST_F(RTOSCommandsTestSuite, CalibrationChainFailsAtEveryReadPosition) {
    for (int fail_at = 1; fail_at <= 20; ++fail_at) {
        resetRtosCommandTestState();
        resetI2CCacheForTests();
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

TEST_F(RTOSCommandsTestSuite, DiagnosticI2C) {

    const device* dev = SystemObjects::i2c().i2c_dev;
    ASSERT_NE(dev, nullptr) << "i2c_dev is nullptr!";

    ASSERT_TRUE(device_is_ready(dev))
        << "device_is_ready() returned false. g_device_ready_override = "
        << g_device_ready_override;

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

TEST_F(RTOSCommandsTestSuite, PowerObserverIntegration) {

    test_iterations_remaining = 1;
    producer_thread();

    SystemObjects::power().notifyBeforeSleep();

    g_queueStats.commandsCreated = 0;
    test_iterations_remaining = 1;
    producer_thread();
    EXPECT_EQ(g_queueStats.commandsCreated, 0u) << "Producer should bypass logic while sleeping";

    SystemObjects::power().notifyAfterWakeup();

    test_iterations_remaining = 1;
    producer_thread();
    EXPECT_GT(g_queueStats.commandsCreated, 0u) << "Producer should resume logic after wakeup";

    SystemObjects::power().notifyBeforeSleep();
    g_queueStats.commandsCreated = 0;
    test_iterations_remaining = 1;
    producer_thread();
    EXPECT_EQ(g_queueStats.commandsCreated, 0u);

    SystemObjects::power().notifySleepAborted();
    test_iterations_remaining = 1;
    producer_thread();
    EXPECT_GT(g_queueStats.commandsCreated, 0u) << "Producer should resume logic after sleep aborted";
}

extern "C" void sys_trace_thread_switched_in_user(struct k_thread *thread);
extern "C" void sys_trace_thread_switched_out_user(struct k_thread *thread);

TEST_F(RTOSCommandsTestSuite, TraceHooksSwitchInAndOut) {
    testing::internal::CaptureStdout();

    sys_trace_thread_switched_in_user((struct k_thread*)producer_tid);
    sys_trace_thread_switched_in_user((struct k_thread*)processor_tid);
    sys_trace_thread_switched_in_user((struct k_thread*)logger_tid);

    sys_trace_thread_switched_out_user((struct k_thread*)producer_tid);
    sys_trace_thread_switched_out_user((struct k_thread*)processor_tid);
    sys_trace_thread_switched_out_user((struct k_thread*)logger_tid);

    struct k_thread* dummy_thread_ptr = reinterpret_cast<struct k_thread*>(0xDEADBEEF);
    sys_trace_thread_switched_in_user(dummy_thread_ptr);
    sys_trace_thread_switched_out_user(dummy_thread_ptr);

    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);

    EXPECT_NE(output.find("switched IN"), std::string_view::npos);
    EXPECT_NE(output.find("switched OUT"), std::string_view::npos);
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

TEST_F(RTOSCommandsTestSuite, PrintCmdUnknownSensorDirectCall) {
    PrintCmd cmd(static_cast<SensorID>(0xFFFF), 1.0f);
    EXPECT_STREQ(cmd.getSensorName(), "Unknown Sensor");
}

TEST_F(RTOSCommandsTestSuite, PrintBME280AndLPS22HBExecuteDirectly) {
    BME280Data bme_data{25.0f, 1013.25f, 50.0f};
    PrintBME280Cmd bme_cmd(bme_data);

    testing::internal::CaptureStdout();
    bme_cmd.execute();
    std::string_view out_bme(testing::internal::GetCapturedStdout());
    EXPECT_NE(out_bme.find("BME280 Temp ="), std::string_view::npos);

    LPS22HBData lps_data{25.0f, 1013.25f};
    PrintLPS22HBCmd lps_cmd(lps_data);

    testing::internal::CaptureStdout();
    lps_cmd.execute();
    std::string_view out_lps(testing::internal::GetCapturedStdout());
    EXPECT_NE(out_lps.find("LPS22HB Temp ="), std::string_view::npos);
}

TEST_F(RTOSCommandsTestSuite, PrintLPS22HBMeasurementLoggerQueueFull) {

    for (int i = 0; i < QueueConfig::Depth; i++) {
        ASSERT_TRUE(enqueueCommand<PreemptionTestCmd>(LOGGER_Q, i));
    }

    LPS22HBData mock_data{25.0f, 1013.0f};
    testing::internal::CaptureStdout();

    EXPECT_FALSE(printLPS22HBMeasurement(mock_data));

    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view out(raw_out);
    EXPECT_NE(out.find("Logger queue full"), std::string_view::npos);
    EXPECT_GT(g_queueStats.loggerQueueFull, 0u);
}

TEST_F(RTOSCommandsTestSuite, SensorReadCmdExecuteBranchCoverage) {
    SensorReadCmd pav_cmd(SensorID::PAV3015, SensorReg::PAV_DESC.reg, ReadLength::Word);
    pav_cmd.execute();

    SensorReadCmd unknown_reg_cmd(SensorID::LPS22HB, 0xFF, ReadLength::Word);
    unknown_reg_cmd.execute();

    SensorReadCmd lps_t_cmd(SensorID::LPS22HB, SensorReg::LPS_T_DESC.reg, ReadLength::Word);

    // A: last_raw_lps_t == 0 after SetUp's reset -> covers line 464's
    //    "*last_val_ptr == 0" branch (right side of ||), regardless of diff.
    g_i2c_mock_word_val = 0x0010;
    lps_t_cmd.execute();               // last_raw_lps_t becomes 0x0010

    // B: raw(0x0100) > *last_val_ptr(0x0010) AND diff(0xF0) > threshold(0x10)
    //    -> covers ternary TRUE side (line 462) and OR left-branch true (line 464)
    g_i2c_mock_word_val = 0x0100;
    lps_t_cmd.execute();               // last_raw_lps_t becomes 0x0100

    // C: raw(0x00F8) < *last_val_ptr(0x0100) AND diff(0x08) <= threshold(0x10),
    //    *last_val_ptr != 0 -> covers ternary FALSE side (line 462) and
    //    OR both-false (line 464); *last_val_ptr stays 0x0100 (not updated)
    g_i2c_mock_word_val = 0x00F8;
    lps_t_cmd.execute();
}
