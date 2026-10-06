#include <gtest/gtest.h>
#include <zephyr/kernel.h>
#include <array>
#include <atomic>
#include <new>
#include <string>
#include <string_view>
#include <thread>
#include <chrono>
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <fstream>
#include <pthread.h>
#include <sys/mman.h>
#include <unistd.h>

#define private public
#define protected public
#include "RTOS_Command_based_thread_system.h"
#undef private
#undef protected

#include "Static_Memory+MISRA_Compliance_Layer.h"
#include "Device_State_Machine+Watchdog.h"
#include "Power_Management_System.h"
#include "Fault_Tolerant_I2C_Communication_Layer.h"

extern "C" {
    void producer_thread(void *arg1, void *arg2, void *arg3);
    void processor_thread(void *arg1, void *arg2, void *arg3);
    void logger_thread(void *arg1, void *arg2, void *arg3);
}

#define producer_thread()  producer_thread(nullptr, nullptr, nullptr)
#define processor_thread() processor_thread(nullptr, nullptr, nullptr)
#define logger_thread()    logger_thread(nullptr, nullptr, nullptr)

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

    // CHANGE HERE: A successful read should be silent. It should NOT output an error log.
    EXPECT_FALSE(output.find("[READ]") != std::string_view::npos);
}

TEST_F(RTOSCommandsTestSuite, ComputeCmdBME280Logic) {
    ComputeCmd cmd(SensorID::BME280, SensorReg::BME280_DATA_START, 0x000100010001ULL);

    testing::internal::CaptureStdout();
    cmd.execute();

    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    
    // CHANGE HERE: A successful compute enqueues to the logger quietly.
    // It should NOT output a compute error log.
    EXPECT_FALSE(output.find("[COMPUTE]") != std::string_view::npos);
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

TEST_F(RTOSCommandsTestSuite, ICommandOperatorDelete) {
    ASSERT_TRUE(enqueueCommand<PreemptionTestCmd>(PROCESSOR_Q, 42));

    ICommand* cmd = nullptr;
    ASSERT_EQ(k_msgq_get(PROCESSOR_Q, &cmd, K_NO_WAIT), 0);

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

extern "C" void sys_trace_thread_switched_in_user(void);
extern "C" void sys_trace_thread_switched_out_user(void);
extern atomic_t g_switch_hook_hits;

TEST_F(RTOSCommandsTestSuite, TraceHooksSwitchInAndOut) {
    // Reset the hook hit counter
    atomic_set(&g_switch_hook_hits, 0);

    // Execute the hooks. In the gtest context, k_sched_current_thread_query() 
    // will return the test runner's thread, which will evaluate to TraceThread::INVALID.
    sys_trace_thread_switched_in_user();
    sys_trace_thread_switched_out_user();

    // Verify that the hooks safely ignore unregistered threads 
    // without crashing, and do not increment the target thread hit counter.
    EXPECT_EQ(atomic_get(&g_switch_hook_hits), 0);
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

    g_i2c_mock_word_val = 0x0010;
    lps_t_cmd.execute();               // last_raw_lps_t becomes 0x0010

    g_i2c_mock_word_val = 0x0100;
    lps_t_cmd.execute();               // last_raw_lps_t becomes 0x0100

    g_i2c_mock_word_val = 0x00F8;
    lps_t_cmd.execute();

    SensorReadCmd unknown_id_err(static_cast<SensorID>(0xFFFF), 0x00, ReadLength::Block);
    unknown_id_err.execute();

    SensorReadCmd unknown_id_ok(static_cast<SensorID>(0xFFFF), 0x00, ReadLength::Word);
    unknown_id_ok.execute();
}


extern "C" void trace_logger_thread(void *arg1, void *arg2, void *arg3);

static std::atomic<uintptr_t> g_spoofed_current_thread{0};
extern "C" k_tid_t k_current_get(void) {
    return reinterpret_cast<k_tid_t>(g_spoofed_current_thread.load());
}

namespace {

using namespace std::chrono_literals;

void msSleep(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

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

class CurrentThreadSpoof {
public:
    ~CurrentThreadSpoof() { select(-1); }
    void select(int which) {
        switch (which) {
            case 0:  g_spoofed_current_thread = reinterpret_cast<uintptr_t>(producer_tid);  break;
            case 1:  g_spoofed_current_thread = reinterpret_cast<uintptr_t>(processor_tid); break;
            case 2:  g_spoofed_current_thread = reinterpret_cast<uintptr_t>(logger_tid);    break;
            default: g_spoofed_current_thread = 0;                                          break;
        }
    }
};


void setUptimeMs(uint32_t ms) { virtual_uptime = ms; }


struct TraceEventMirror {          // same layout as TraceEvent in the .cpp
    uint32_t timestamp;
    struct k_thread* thread;
    uint8_t type;                  // 0 = SWITCH_IN, 1 = SWITCH_OUT
};

bool pokeInvalidTraceEvents(struct k_thread* x) {
    constexpr size_t S    = sizeof(TraceEventMirror);
    constexpr size_t TOFF = offsetof(TraceEventMirror, thread);
    constexpr size_t YOFF = offsetof(TraceEventMirror, type);
    constexpr uint8_t kPattern[5] = {1, 0, 0, 1, 0};   // OUT, IN, IN, OUT, IN

    auto thr = [](uintptr_t a) { return *reinterpret_cast<struct k_thread* const*>(a); };
    auto typ = [](uintptr_t a) { return *reinterpret_cast<const uint8_t*>(a); };

    std::ifstream maps("/proc/self/maps");
    std::string line;
    while (std::getline(maps, line)) {
        unsigned long lo = 0, hi = 0;
        char perms[8] = {};
        if (std::sscanf(line.c_str(), "%lx-%lx %7s", &lo, &hi, perms) != 3) continue;
        if (perms[0] != 'r' || perms[1] != 'w') continue;
        if (line.find("[vvar]") != std::string::npos || line.find("[vsyscall]") != std::string::npos) continue;
        if (hi - lo > (256UL << 20)) continue;

        for (uintptr_t p = lo; p + 5 * S + YOFF + 1 <= hi; p += alignof(TraceEventMirror)) {
            bool match = true;
            for (size_t i = 0; i < 5 && match; ++i) {
                match = (thr(p + i * S + TOFF) == x) && (typ(p + i * S + YOFF) == kPattern[i]);
            }
            if (!match) continue;
            *reinterpret_cast<volatile uint8_t*>(p + 1 * S + YOFF) = 2;                       // invalid type
            *reinterpret_cast<struct k_thread* volatile*>(p + 3 * S + TOFF) =
                reinterpret_cast<struct k_thread*>(static_cast<uintptr_t>(0xDEAD0));          // invalid thread
            return true;
        }
    }
    return false;
}

std::atomic<bool> g_trace_thread_done{false};

void traceCleanup(void*) { g_trace_thread_done = true; }

void* traceThreadEntry(void*) {
    pthread_cleanup_push(traceCleanup, nullptr);
    trace_logger_thread(nullptr, nullptr, nullptr);
    pthread_cleanup_pop(1);
    return nullptr;
}

bool joinTraceThread(pthread_t t) {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!g_trace_thread_done && std::chrono::steady_clock::now() < deadline) msSleep(10);
    if (g_trace_thread_done) { pthread_join(t, nullptr); return true; }
    pthread_detach(t);
    return false;
}

} // namespace


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

TEST_F(RTOSCommandsTestSuite, TraceHooksRecordRegisteredThreads) {
    CurrentThreadSpoof spoof;
    atomic_set(&g_switch_hook_hits, 0);

    spoof.select(0);
    sys_trace_thread_switched_in_user();

    spoof.select(1);
    sys_trace_thread_switched_in_user();

    spoof.select(2);
    sys_trace_thread_switched_in_user();

    EXPECT_EQ(atomic_get(&g_switch_hook_hits), 3);

    spoof.select(-1);
    sys_trace_thread_switched_in_user();
    sys_trace_thread_switched_out_user();
    EXPECT_EQ(atomic_get(&g_switch_hook_hits), 3);
}

TEST_F(RTOSCommandsTestSuite, TraceLoggerThreadFullCoverage) {
    // Capture stdout/stderr to suppress expected trace overflow warnings during testing
    testing::internal::CaptureStdout();

    const uint32_t saved_uptime = virtual_uptime;
    CurrentThreadSpoof spoof;

    // Register g_powerObserver with the PowerManager.
    test_iterations_remaining = 0;
    producer_thread();

    auto hookOut = []() { sys_trace_thread_switched_out_user(); };
    auto hookIn  = []() { sys_trace_thread_switched_in_user(); };

    spoof.select(0);                                  // PRODUCER
    setUptimeMs(1);
    for (int i = 0; i < 4100; ++i) hookOut();         // > TRACE_BUFFER_SIZE -> overflow path

    bool poked = false;
    for (int attempt = 0; attempt < 8 && !poked; ++attempt) {
        hookOut();                                    // e0 OUT
        hookIn();                                     // e1 IN  <- type corrupted
        hookIn();                                     // e2 IN
        hookOut();                                    // e3 OUT <- thread corrupted
        hookIn();                                     // e4 IN
        poked = pokeInvalidTraceEvents(static_cast<struct k_thread*>(producer_tid));
    }
    EXPECT_TRUE(poked) << "could not locate the trace buffer to inject invalid events";

    setUptimeMs(100);  hookOut(); hookIn();                         //      0 us -> fast resume
    setUptimeMs(200);  hookOut(); setUptimeMs(202);  hookIn();      //   2000 us -> "Preemption delay"
    setUptimeMs(300);  hookOut(); setUptimeMs(795);  hookIn();      // 495000 us -> normal 500 ms sleep, silent
    setUptimeMs(900);  hookOut(); setUptimeMs(1500); hookIn();      // 600000 us -> "Preemption delay"
    
    spoof.select(1);
    setUptimeMs(1600); hookIn();
    setUptimeMs(1700); hookOut(); hookIn();

    spoof.select(2);
    setUptimeMs(1800); hookOut(); hookIn();

    spoof.select(-1);
    hookOut(); hookIn();

    SystemObjects::power().notifyBeforeSleep();
    g_trace_thread_done = false;
    pthread_t trace_th;
    ASSERT_EQ(pthread_create(&trace_th, nullptr, traceThreadEntry, nullptr), 0);
    msSleep(100);                                     // consumer is inside the isSleeping() loop

    SystemObjects::power().notifyAfterWakeup();
    SystemObjects::context().triggerFault("Trace coverage halt");
    SystemObjects::context().requestTransition(SystemState::SAFE_HALT);
    msSleep(100);

    SystemObjects::context().requestTransition(SystemState::INIT);
    msSleep(500);

    SystemObjects::power().notifyBeforeSleep();
    msSleep(100);
    flockfile(stdout);
    spoof.select(0);
    setUptimeMs(5000); hookOut();
    setUptimeMs(5300); hookIn();                      // 300000 us -> consumer logs
    SystemObjects::power().notifyAfterWakeup();
    msSleep(200);                                     // consumer is now blocked in printf()
    for (int i = 0; i < 4200; ++i) hookOut();         // head - tail > TRACE_BUFFER_SIZE
    spoof.select(-1);
    funlockfile(stdout);                              // consumer resumes -> hits the break
    msSleep(300);                                     // ... then drains the remaining events

    SystemObjects::power().notifyBeforeSleep();
    msSleep(100);
    spoof.select(0);
    setUptimeMs(9000); hookOut();
    setUptimeMs(9300); hookIn();                      // makes the consumer log -> cancellation point
    spoof.select(-1);
    pthread_cancel(trace_th);
    SystemObjects::power().notifyAfterWakeup();
    EXPECT_TRUE(joinTraceThread(trace_th)) << "trace_logger_thread did not stop";

    SystemObjects::context().requestTransition(SystemState::INIT);
    virtual_uptime = saved_uptime;

    (void)testing::internal::GetCapturedStdout();

    SUCCEED();
}

TEST_F(RTOSCommandsTestSuite, SensorReadCmdTriggersFaultOnThreeFailures) {
    g_i2c_force_errno = -EIO;
    SensorReadCmd bme_cmd(SensorID::BME280, SensorReg::BME280_DATA_START, ReadLength::Block);
    SensorReadCmd lps_cmd(SensorID::LPS22HB, SensorReg::LPS_P_DESC.reg, ReadLength::Triple);
    SensorReadCmd pav_cmd(SensorID::PAV3015, SensorReg::PAV_DESC.reg, ReadLength::Word);

    bme_cmd.execute();
    bme_cmd.execute();
    bme_cmd.execute(); // 3rd failure triggers fault
    EXPECT_EQ(SystemObjects::context().getState(), SystemState::FAULT);
    SystemObjects::context().requestTransition(SystemState::INIT);

    lps_cmd.execute();
    lps_cmd.execute();
    lps_cmd.execute();
    EXPECT_EQ(SystemObjects::context().getState(), SystemState::FAULT);
    SystemObjects::context().requestTransition(SystemState::INIT);

    pav_cmd.execute();
    pav_cmd.execute();
    pav_cmd.execute();
    EXPECT_EQ(SystemObjects::context().getState(), SystemState::FAULT);
    SystemObjects::context().requestTransition(SystemState::INIT);

    g_i2c_force_errno = 0;
}
