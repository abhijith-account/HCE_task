#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include "RTOS_Command_based_thread_system.h"
#include <zephyr/logging/log.h>
#include "Static_Memory+MISRA_Compliance_Layer.h"
#include "Fault_Tolerant_I2C_Communication_Layer.h"
#include "Device_State_Machine+Watchdog.h"
#include <new>

#ifdef IS_TEST_ENVIRONMENT
    extern int test_iterations_remaining;
    #define THREAD_LOOP_CONDITION (test_iterations_remaining > 0 ? (test_iterations_remaining--, true) : false)
#else
    #define THREAD_LOOP_CONDITION true
#endif

LOG_MODULE_REGISTER(COMMANDS, LOG_LEVEL_INF);

#define TRACE_BUFFER_SIZE 4096
#define TRACE_BUFFER_MASK (TRACE_BUFFER_SIZE - 1)

enum class TraceType : uint8_t {
    SWITCH_IN,
    SWITCH_OUT
};

struct TraceEvent {
    uint32_t timestamp;
    struct k_thread* thread;
    TraceType type;
};

static TraceEvent g_trace_buffer[TRACE_BUFFER_SIZE];
static volatile uint32_t g_trace_head = 0;
static uint32_t g_trace_tail = 0;
atomic_t g_producer_alive = ATOMIC_INIT(1);
atomic_t g_processor_alive = ATOMIC_INIT(1);
atomic_t g_logger_alive = ATOMIC_INIT(1);

atomic_t g_switch_hook_hits = ATOMIC_INIT(0);

inline void logCommandError(const char* tag, uint32_t id, const char* msg) noexcept {
    LOG_ERR("[%s] #%u: %s", tag, id, msg);
}

struct alignas(8) MaxCommandSize { uint8_t buffer[PoolConfig::Size]{}; };
StaticPool<MaxCommandSize, PoolConfig::Elements> g_commandPool;
QueueStats g_queueStats;
static bool g_bme280_initialized = false;
static bool g_lps22hb_initialized = false; 
static bool g_pav3015_initialized = false;
extern bool g_usb_connected;

static_assert(sizeof(SensorReadCmd) <= sizeof(MaxCommandSize), "SensorReadCmd exceeds memory pool size");
static_assert(sizeof(ComputeCmd) <= sizeof(MaxCommandSize), "ComputeCmd exceeds memory pool size");
static_assert(sizeof(PrintCmd) <= sizeof(MaxCommandSize), "PrintCmd exceeds memory pool size");
static_assert(sizeof(PrintBME280Cmd) <= sizeof(MaxCommandSize), "PrintBME280Cmd exceeds memory pool size");
static_assert(sizeof(PrintLPS22HBCmd) <= sizeof(MaxCommandSize), "PrintLPS22HBCmd exceeds memory pool size");

static_assert(alignof(MaxCommandSize) >= alignof(PrintLPS22HBCmd), "Alignment mismatch on PrintLPS22HBCmd");
static_assert(alignof(MaxCommandSize) >= alignof(SensorReadCmd), "Alignment mismatch on SensorReadCmd");
static_assert(alignof(MaxCommandSize) >= alignof(ComputeCmd), "Alignment mismatch on ComputeCmd");
static_assert(alignof(MaxCommandSize) >= alignof(PrintCmd), "Alignment mismatch on PrintCmd");
static_assert(alignof(MaxCommandSize) >= alignof(PrintBME280Cmd), "Alignment mismatch on PrintBME280Cmd");

K_MSGQ_DEFINE(processor_queue, sizeof(ICommand*), QueueConfig::Depth, QueueConfig::Alignment);
K_MSGQ_DEFINE(logger_queue, sizeof(ICommand*), QueueConfig::Depth, QueueConfig::Alignment);

void* allocateCommandMemory() noexcept {
    void* mem = g_commandPool.allocate();
    if (!mem) {
        g_queueStats.commandsDropped++;
        LOG_ERR("Memory pool exhausted. Dropping command.");
    }
    return mem;
}

bool enqueueCommandRaw(k_msgq* queue, ICommand* cmd) noexcept {
    if (k_msgq_put(queue, &cmd, K_NO_WAIT) != 0) {
        g_queueStats.commandsDropped++;
        return false;
    }

    uint32_t used = QueueConfig::Depth - k_msgq_num_free_get(queue);
    if (queue == PROCESSOR_Q && used > g_queueStats.processorPeakDepth) {
        g_queueStats.processorPeakDepth = used;
    } else if (queue == LOGGER_Q && used > g_queueStats.loggerPeakDepth) {
        g_queueStats.loggerPeakDepth = used;
    }

    g_queueStats.commandsCreated++;
    return true;
}

bool printMeasurement(SensorID id, float value) noexcept {
    if (!enqueueCommand<PrintCmd>(LOGGER_Q, id, value)) {
        g_queueStats.loggerQueueFull++;
        LOG_ERR("[%s] Logger queue full. Metric dropped.", LogTags::PRINT);
        return false;
    }
    return true;
}

ICommand::ICommand(){
    static atomic_t cmd_counter = 0;
    command_id = static_cast<uint32_t>(atomic_inc(&cmd_counter));
}

void ICommand::operator delete(void* ptr) noexcept {
    g_commandPool.deallocate(ptr);
}

void ICommand::destroy() noexcept {
    this->~ICommand();
    g_commandPool.deallocate(this);
}

extern DeviceContext sys_context;
extern I2CManager i2c_manager;

namespace SystemObjects {
    DeviceContext& context() { return sys_context; }
    I2CManager& i2c() { return i2c_manager; }
    PowerManager& power() { return PowerManager::getInstance(); }
}

class ThreadSystemPowerObserver final : public IPowerObserver {
private:
    atomic_t is_sleeping{};
public:
    ThreadSystemPowerObserver() { atomic_set(&is_sleeping, 0); }
    void beforeSleep() override { atomic_set(&is_sleeping, 1); }
    void afterWakeup() override { atomic_set(&is_sleeping, 0); }
    void sleepAborted() override { atomic_set(&is_sleeping, 0); }
    bool isSleeping() const noexcept { return atomic_get(&is_sleeping) != 0; }
    void resetForTest() noexcept { atomic_set(&is_sleeping, 0); }
};

static ThreadSystemPowerObserver g_powerObserver;

namespace BMEConstants {
    constexpr int32_t T_FINE_OFFSET = 76800;
    constexpr int64_t P_OFFSET      = 128000;
    constexpr int64_t P_1048576     = 1048576;
    constexpr int64_t P_3125        = 3125;
    constexpr int32_t H_OFFSET      = 16384;
    constexpr int32_t H_32768       = 32768;
    constexpr int32_t H_2097152     = 2097152;
    constexpr int32_t H_8192        = 8192;
    constexpr int32_t H_MAX         = 419430400;

    constexpr float TEMP_DIV   = 100.0F;
    constexpr float PRESS_DIV1 = 256.0F;
    constexpr float PRESS_DIV2 = 100.0F;
    constexpr float HUM_DIV    = 1024.0F;
}

namespace LPS22HBConst {
    constexpr float TEMP_DIV  = 100.0F;
    constexpr float PRESS_DIV = 4096.0F;
}

namespace PAV3015Const {
    constexpr float OFFSET = 0.1F;
    constexpr float LINEAR = 0.0000506F;
    constexpr float QUADRATIC = 0.000001F;
}

namespace {
    BME280Calibration g_bme280Calib;

    static struct k_mutex calib_mutex;
    static bool calib_mutex_initialized = false;

    static void ensureCalibMutex() {
        if (!calib_mutex_initialized) {
            k_mutex_init(&calib_mutex);
            calib_mutex_initialized = true;
        }
    }

    [[nodiscard]] bool readTempCalibration(uint16_t addr) noexcept {
        auto t1 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_T1); if (!t1.isOk()) { return false; }
        auto t2 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_T2); if (!t2.isOk()) { return false; }
        auto t3 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_T3); if (!t3.isOk()) { return false; }

        g_bme280Calib.dig_T1 = t1.unwrap();
        g_bme280Calib.dig_T2 = static_cast<int16_t>(t2.unwrap());
        g_bme280Calib.dig_T3 = static_cast<int16_t>(t3.unwrap());
        return true;
    }

    [[nodiscard]] bool readPressureCalibration(uint16_t addr) noexcept {
        auto p1 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_P1); if (!p1.isOk()) { return false; }
        auto p2 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_P2); if (!p2.isOk()) { return false; }
        auto p3 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_P3); if (!p3.isOk()) { return false; }
        auto p4 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_P4); if (!p4.isOk()) { return false; }
        auto p5 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_P5); if (!p5.isOk()) { return false; }
        auto p6 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_P6); if (!p6.isOk()) { return false; }
        auto p7 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_P7); if (!p7.isOk()) { return false; }
        auto p8 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_P8); if (!p8.isOk()) { return false; }
        auto p9 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_P9); if (!p9.isOk()) { return false; }

        g_bme280Calib.dig_P1 = p1.unwrap();
        g_bme280Calib.dig_P2 = static_cast<int16_t>(p2.unwrap());
        g_bme280Calib.dig_P3 = static_cast<int16_t>(p3.unwrap());
        g_bme280Calib.dig_P4 = static_cast<int16_t>(p4.unwrap());
        g_bme280Calib.dig_P5 = static_cast<int16_t>(p5.unwrap());
        g_bme280Calib.dig_P6 = static_cast<int16_t>(p6.unwrap());
        g_bme280Calib.dig_P7 = static_cast<int16_t>(p7.unwrap());
        g_bme280Calib.dig_P8 = static_cast<int16_t>(p8.unwrap());
        g_bme280Calib.dig_P9 = static_cast<int16_t>(p9.unwrap());
        return true;
    }

    [[nodiscard]] bool readHumidityCalibration(uint16_t addr) noexcept {
        auto h1 = SystemObjects::i2c().readRegister(addr, BME280CalibReg::DIG_H1); if (!h1.isOk()) { return false;}
        auto h2_l = SystemObjects::i2c().readRegister(addr, BME280CalibReg::DIG_H2_L); if (!h2_l.isOk()) { return false;}
        auto h2_m = SystemObjects::i2c().readRegister(addr, BME280CalibReg::DIG_H2_M); if (!h2_m.isOk()) { return false;}
        auto h3 = SystemObjects::i2c().readRegister(addr, BME280CalibReg::DIG_H3); if (!h3.isOk()) { return false;}
        auto h4_m = SystemObjects::i2c().readRegister(addr, BME280CalibReg::DIG_H4_M); if (!h4_m.isOk()) { return false;}
        auto h_shared = SystemObjects::i2c().readRegister(addr, BME280CalibReg::DIG_H_SHARED); if (!h_shared.isOk()) { return false;}
        auto h5_m = SystemObjects::i2c().readRegister(addr, BME280CalibReg::DIG_H5_M); if (!h5_m.isOk()) { return false;}
        auto h6 = SystemObjects::i2c().readRegister(addr, BME280CalibReg::DIG_H6); if (!h6.isOk()) { return false;}

        const auto shared_val = h_shared.unwrap();
        g_bme280Calib.dig_H1 = h1.unwrap();
        g_bme280Calib.dig_H2 = static_cast<int16_t>((h2_m.unwrap() << 8) | h2_l.unwrap());
        g_bme280Calib.dig_H3 = h3.unwrap();
        g_bme280Calib.dig_H4 = static_cast<int16_t>((static_cast<int8_t>(h4_m.unwrap()) * 16) | (shared_val & 0x0F));
        g_bme280Calib.dig_H5 = static_cast<int16_t>((static_cast<int8_t>(h5_m.unwrap()) * 16) | (shared_val >> 4));
        g_bme280Calib.dig_H6 = static_cast<int8_t>(h6.unwrap());
        return true;
    }

    [[nodiscard]] bool loadBME280Calibration(uint16_t addr) noexcept {
        ensureCalibMutex();
        k_mutex_lock(&calib_mutex, K_FOREVER);
        if (g_bme280Calib.is_loaded) {
            k_mutex_unlock(&calib_mutex);
            return true;
        }

        if (!readTempCalibration(addr) || !readPressureCalibration(addr) || !readHumidityCalibration(addr)) {
            k_mutex_unlock(&calib_mutex);
            return false;
        }
        g_bme280Calib.is_loaded = true;
        k_mutex_unlock(&calib_mutex);
        LOG_INF("[%s] BME280 ROM Calibration Loaded Successfully.", LogTags::PRODUCER);
        return true;
    }
}

namespace BitExtractor {
    constexpr int32_t adcTemp(uint64_t raw) noexcept { return static_cast<int32_t>((raw >> 16) & 0xFFFFFF) >> 4; }
    constexpr int32_t adcPress(uint64_t raw) noexcept { return static_cast<int32_t>((raw >> 40) & 0xFFFFFF) >> 4; }
    constexpr int32_t adcHum(uint64_t raw) noexcept { return static_cast<int32_t>(raw & 0xFFFF); }
}

BME280Data BME280Math::decode(uint64_t raw_data, const BME280Calibration& c) noexcept {
    const int32_t adc_T = BitExtractor::adcTemp(raw_data);
    const int32_t adc_P = BitExtractor::adcPress(raw_data);
    const int32_t adc_H = BitExtractor::adcHum(raw_data);

    const int32_t var1 = ((((adc_T >> 3) - (static_cast<int32_t>(c.dig_T1) << 1))) * static_cast<int32_t>(c.dig_T2)) >> 11;
    const int32_t var2 = (((((adc_T >> 4) - static_cast<int32_t>(c.dig_T1)) * ((adc_T >> 4) - static_cast<int32_t>(c.dig_T1))) >> 12) * static_cast<int32_t>(c.dig_T3)) >> 14;
    const int32_t t_fine = var1 + var2;

    BME280Data result{};
    result.temperature = static_cast<float>((t_fine * 5 + 128) >> 8) / BMEConstants::TEMP_DIV;

    int64_t p_var1 = static_cast<int64_t>(t_fine) - BMEConstants::P_OFFSET;
    int64_t p_var2 = p_var1 * p_var1 * static_cast<int64_t>(c.dig_P6);
    p_var2 = p_var2 + ((p_var1 * static_cast<int64_t>(c.dig_P5)) << 17);
    p_var2 = p_var2 + (static_cast<int64_t>(c.dig_P4) << 35);
    p_var1 = ((p_var1 * p_var1 * static_cast<int64_t>(c.dig_P3)) >> 8) + ((p_var1 * static_cast<int64_t>(c.dig_P2)) << 12);
    p_var1 = (((static_cast<int64_t>(1) << 47) + p_var1)) * static_cast<int64_t>(c.dig_P1) >> 33;

    if (p_var1 != 0) {
        int64_t p = BMEConstants::P_1048576 - adc_P;
        p = (((p << 31) - p_var2) * BMEConstants::P_3125) / p_var1;
        p_var1 = (static_cast<int64_t>(c.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        p_var2 = (static_cast<int64_t>(c.dig_P8) * p) >> 19;
        p = ((p + p_var1 + p_var2) >> 8) + (static_cast<int64_t>(c.dig_P7) << 4);
        result.pressure = static_cast<float>(p) / BMEConstants::PRESS_DIV1 / BMEConstants::PRESS_DIV2;
    }

    int32_t v_x1_u32r = t_fine - BMEConstants::T_FINE_OFFSET;
    const int32_t hum_h4 = static_cast<int32_t>(c.dig_H4) << 20;
    const int32_t hum_h5 = static_cast<int32_t>(c.dig_H5) * v_x1_u32r;
    const int32_t h_term1 = (((adc_H << 14) - hum_h4 - hum_h5) + BMEConstants::H_OFFSET) >> 15;

    const int32_t hum_h6 = (v_x1_u32r * static_cast<int32_t>(c.dig_H6)) >> 10;
    const int32_t hum_h3 = (v_x1_u32r * static_cast<int32_t>(c.dig_H3)) >> 11;
    const int32_t h_term2 = (((hum_h6 * (hum_h3 + BMEConstants::H_32768)) >> 10) + BMEConstants::H_2097152);
    const int32_t h_term3 = (h_term2 * static_cast<int32_t>(c.dig_H2) + BMEConstants::H_8192) >> 14;

    v_x1_u32r = h_term1 * h_term3;
    const int32_t hum_h1 = static_cast<int32_t>(c.dig_H1);
    const int32_t h_term4 = ((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7;

    v_x1_u32r = v_x1_u32r - ((h_term4 * hum_h1) >> 4);
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > BMEConstants::H_MAX ? BMEConstants::H_MAX : v_x1_u32r);
    result.humidity = static_cast<float>(v_x1_u32r >> 12) / BMEConstants::HUM_DIV;

    return result;
}

float LPS22HBMath::decodeTemperature(uint64_t raw_data) noexcept {
    const uint8_t byte2 = (raw_data >> 8) & 0xFF;
    const uint8_t byte3 = raw_data & 0xFF;
    return static_cast<float>(static_cast<int16_t>((byte2 << 8) | byte3)) / LPS22HBConst::TEMP_DIV;
}

float LPS22HBMath::decodePressure(uint64_t raw_data) noexcept {
    const uint8_t byte1 = (raw_data >> 16) & 0xFF;
    const uint8_t byte2 = (raw_data >> 8) & 0xFF;
    const uint8_t byte3 = raw_data & 0xFF;
    int32_t press_raw = static_cast<int32_t>((byte3 << 16) | (byte2 << 8) | byte1);
    if (press_raw & 0x00800000) press_raw |= 0xFF000000;
    return static_cast<float>(press_raw) / LPS22HBConst::PRESS_DIV;
}

float PAV3015Math::decodeAirflow(uint64_t raw_data) noexcept {
    const uint8_t byte2 = (raw_data >> 8) & 0xFF;
    const uint8_t byte3 = raw_data & 0xFF;
    const float x = static_cast<float>(static_cast<uint16_t>((byte2 << 8) | byte3));
    float flow = PAV3015Const::OFFSET + (PAV3015Const::LINEAR * x) + (PAV3015Const::QUADRATIC * x * x);
    return flow ;
}

SensorReadCmd::SensorReadCmd(SensorID s_id, uint8_t r_addr, ReadLength len) noexcept
    : sensor_id(s_id), reg_addr(r_addr), length(len) {}

#ifdef CONFIG_BOARD_MPS2_AN386
uint64_t SensorReadCmd::readMockPAVData() const noexcept {
    static int32_t mock_pav = MockValues::PAV_BASE;
    static bool increasing = true;

    if (increasing) {
        mock_pav += 1;
        if (mock_pav > static_cast<int32_t>(MockValues::PAV_BASE) + 50) { 
            increasing = false;
        }
    } else {
        mock_pav -= 1;
        if (mock_pav < static_cast<int32_t>(MockValues::PAV_BASE) - 50) {
            increasing = true;
        }
    }
    return mock_pav;
}
#endif

Result<uint64_t> SensorReadCmd::readHardwareData() const noexcept {
    const uint16_t s_addr = static_cast<uint16_t>(sensor_id);
    switch (length) {
        case ReadLength::Block:
            if (sensor_id == SensorID::BME280) return SystemObjects::i2c().read64Bit(s_addr, reg_addr);
            return Result<uint64_t>::Err(I2CFault::NACK);

        case ReadLength::Triple: {
            auto res = SystemObjects::i2c().read24Bit(s_addr, reg_addr);
            return res.isOk() ? Result<uint64_t>::Ok(res.unwrap()) : Result<uint64_t>::Err(res.error);
        }

        case ReadLength::Word: {
            auto res = SystemObjects::i2c().readWord(s_addr, reg_addr);
            return res.isOk() ? Result<uint64_t>::Ok(res.unwrap()) : Result<uint64_t>::Err(res.error);
        }

        default:
            return Result<uint64_t>::Err(I2CFault::NACK);
    }
}

static uint64_t last_raw_bme = 0;
static uint64_t last_raw_lps_p = 0;
static uint64_t last_raw_lps_t = 0;
static uint64_t last_raw_pav = 0;

void SensorReadCmd::execute() noexcept {
    uint64_t raw = 0;

    static uint8_t bme_failures = 0;
    static uint8_t lps_failures = 0;
    static uint8_t pav_failures = 0;

    uint8_t* fail_counter = nullptr;
    if (sensor_id == SensorID::BME280) fail_counter = &bme_failures;
    else if (sensor_id == SensorID::LPS22HB) fail_counter = &lps_failures;
    else if (sensor_id == SensorID::PAV3015) fail_counter = &pav_failures;

#ifdef CONFIG_BOARD_MPS2_AN386
    if (sensor_id == SensorID::PAV3015) {
        raw = readMockPAVData();
        if (fail_counter) *fail_counter = 0;
    } else {
        auto res = readHardwareData();
        if (!res.isOk()) {
            logCommandError(LogTags::READ, command_id, "I2C Transaction Failed.");
            if (fail_counter && ++(*fail_counter) >= 3) {
                SystemObjects::context().triggerFault("I2C Sensor Disconnected");
            }
            return;
        }
        raw = res.unwrap();
        if (fail_counter) *fail_counter = 0;
    } 
#else
    auto res = readHardwareData();
    if (!res.isOk()) {
        logCommandError(LogTags::READ, command_id, "I2C Transaction Failed.");
        if (fail_counter && ++(*fail_counter) >= 3) {
            SystemObjects::context().triggerFault("I2C Sensor Disconnected");
        }
        return;
    }
    raw = res.unwrap();
    if (fail_counter) *fail_counter = 0;
#endif

    uint64_t* last_val_ptr = nullptr;
    uint64_t threshold = 0;

    if (sensor_id == SensorID::BME280) {
        last_val_ptr = &last_raw_bme;
        threshold = 0x1000; 
    } else if (sensor_id == SensorID::LPS22HB && reg_addr == SensorReg::LPS_P_DESC.reg) {
        last_val_ptr = &last_raw_lps_p;
        threshold = 0x50;   
    } else if (sensor_id == SensorID::LPS22HB && reg_addr == SensorReg::LPS_T_DESC.reg) {
        last_val_ptr = &last_raw_lps_t;
        threshold = 0x10;  
    } else if (sensor_id == SensorID::PAV3015) {
        last_val_ptr = &last_raw_pav;
        threshold = 0x05;   
    }

    if (last_val_ptr != nullptr) {
        uint64_t diff = (raw > *last_val_ptr) ? (raw - *last_val_ptr) : (*last_val_ptr - raw);
        if (diff > threshold || *last_val_ptr == 0) {
            *last_val_ptr = raw;
            SystemObjects::power().reportActivity();
        }
    }

    if (!enqueueCommand<ComputeCmd>(PROCESSOR_Q, sensor_id, reg_addr, raw)) {
        logCommandError(LogTags::READ, command_id, "Compute queue full. Discarding raw data.");
    }
}

#ifdef IS_TEST_ENVIRONMENT
void resetSensorReadCmdLastValsForTest() noexcept {
    last_raw_bme = 0;
    last_raw_lps_p = 0;
    last_raw_lps_t = 0;
    last_raw_pav = 0;
}
#endif

ComputeCmd::ComputeCmd(SensorID s_id, uint8_t r_addr, uint64_t data) noexcept
    : sensor_id(s_id), reg_addr(r_addr), raw_data(data) {}

void ComputeCmd::execute() noexcept {

    switch (sensor_id) {
        case SensorID::BME280: {
            if (!loadBME280Calibration(static_cast<uint16_t>(sensor_id))) {
                logCommandError(LogTags::COMPUTE, command_id, "BME280 calibration aborted. Math halting.");
                return;
            }
            BME280Data result = BME280Math::decode(raw_data, g_bme280Calib);
            (void)printBME280Measurement(result);
            break;
        }
        case SensorID::LPS22HB: {
            static LPS22HBData lps_data{};
            static bool has_pressure = false;

            if (reg_addr == SensorReg::LPS_P_DESC.reg) {
                lps_data.pressure = LPS22HBMath::decodePressure(raw_data);
                has_pressure = true;
            } else if (reg_addr == SensorReg::LPS_T_DESC.reg) {
                lps_data.temperature = LPS22HBMath::decodeTemperature(raw_data);
                if (has_pressure) {
                    (void)printLPS22HBMeasurement(lps_data);
                    has_pressure = false;
                }
            }
            break;
        }
        case SensorID::PAV3015: {
            (void)printMeasurement(sensor_id, PAV3015Math::decodeAirflow(raw_data));
            break;
        }
        default:
            logCommandError(LogTags::COMPUTE, command_id, "Unknown Sensor ID.");
            break;
    }
}

PrintCmd::PrintCmd(SensorID s_id, float val) noexcept : sensor_id(s_id), final_value(val) {}

struct SensorInfo {
    SensorID id{};
    const char* name{};
};
constexpr SensorInfo sensorTable[] = {
    {SensorID::BME280,       "BME280(Temp C)"},
    {SensorID::BME280_PRESS, "BME280(Press hPa)"},
    {SensorID::BME280_HUM,   "BME280(Humidity %RH)"},
    {SensorID::LPS22HB,      "LPS22HB(Press hPa)"},
    {SensorID::LPS22HB_TEMP, "LPS22HB(Temp C)"},
    {SensorID::PAV3015,      "PAV3015(Airflow m/s)"}
};

const char* PrintCmd::getSensorName() const noexcept {
    for (const auto& info : sensorTable) {
        if (info.id == sensor_id) return info.name;
    }
    return "Unknown Sensor";
}

void PrintCmd::execute() noexcept {
    LOG_INF("[%s] #%u: Metric: %s = %.2f", LogTags::PRINT, command_id, getSensorName(), (double)final_value);
}

PrintBME280Cmd::PrintBME280Cmd(const BME280Data& d) noexcept : data(d) {}

void PrintBME280Cmd::execute() noexcept {
    LOG_INF("[%s] #%u: Metric: BME280 Temp = %.2f C | Press = %.2f hPa | Hum = %.2f %%RH",
            LogTags::PRINT, command_id,
            (double)data.temperature, (double)data.pressure, (double)data.humidity);
}

bool printBME280Measurement(const BME280Data& data) noexcept {
    if (!enqueueCommand<PrintBME280Cmd>(LOGGER_Q, data)) {
        g_queueStats.loggerQueueFull++;
        LOG_ERR("[%s] Logger queue full. BME280 metrics dropped.", LogTags::PRINT);
        return false;
    }
    return true;
}

PrintLPS22HBCmd::PrintLPS22HBCmd(const LPS22HBData& d) noexcept : data(d) {}

void PrintLPS22HBCmd::execute() noexcept {
    LOG_INF("[%s] #%u: Metric: LPS22HB Temp = %.2f C | Press = %.2f hPa",
            LogTags::PRINT, command_id,
            (double)data.temperature, (double)data.pressure);
}

bool printLPS22HBMeasurement(const LPS22HBData& data) noexcept {
    if (!enqueueCommand<PrintLPS22HBCmd>(LOGGER_Q, data)) {
        g_queueStats.loggerQueueFull++;
        LOG_ERR("[%s] Logger queue full. LPS22HB metrics dropped.", LogTags::PRINT);
        return false;
    }
    return true;
}

extern "C" void producer_thread(void *arg1, void *arg2, void *arg3) {
    ProducerState state = ProducerState::ReadBME;

    SystemObjects::power().registerObserver(&g_powerObserver);

    do {
       while (g_powerObserver.isSleeping()) {
           k_msleep(200);
           atomic_set(&g_producer_alive, 1);
       }

       const SystemState sys_state = SystemObjects::context().getState();

       if (sys_state == SystemState::FAULT) {
           LOG_WRN("[%s] System in FAULT state. Disabling sensors and entering SAFE_HALT.", LogTags::PRODUCER);
           g_bme280_initialized = false;
           g_lps22hb_initialized = false;
           g_pav3015_initialized = false;
           (void)SystemObjects::context().requestTransition(SystemState::SAFE_HALT);
           k_msleep(100);
           continue;
       }

       if (sys_state == SystemState::SAFE_HALT) {
           /* INDUSTRY STANDARD SAFE HALT: Lock out all autonomous recovery and safely block the thread.
            * This requires a hardware reboot or an explicitly authorized override IRQ to clear. */
           LOG_ERR("[%s] System safely halted. Awaiting manual hardware reset.", LogTags::PRODUCER);
           while (SystemObjects::context().getState() == SystemState::SAFE_HALT) {
               k_msleep(1000);
               atomic_set(&g_producer_alive, 1); // Keep WDT fed so it doesn't bounce immediately
           }
           continue;
       }

       SystemObjects::power().reportActivity();

       if (!g_bme280_initialized) {
           const uint16_t bme_addr = static_cast<uint16_t>(SensorID::BME280);
           auto res1 = SystemObjects::i2c().writeRegister(bme_addr, SensorReg::BME280_CTRL_HUM, BME280Config::CTRL_HUM);
           auto res2 = SystemObjects::i2c().writeRegister(bme_addr, SensorReg::BME280_CTRL_MEAS, BME280Config::CTRL_MEAS);
           if (!res1.isOk() || !res2.isOk()) {
               LOG_ERR("[%s] Failed to initialize BME280. Retrying...", LogTags::PRODUCER);
           } else {
               g_bme280_initialized = true;
               SystemObjects::context().requestTransition(SystemState::RUNNING);
               k_msleep(20);
           }
       }

       if (!g_lps22hb_initialized) {
           const uint16_t lps_addr = static_cast<uint16_t>(SensorID::LPS22HB);
           auto res = SystemObjects::i2c().writeRegister(lps_addr, 0x10, 0x10); 
           if (!res.isOk()) {
               LOG_ERR("[%s] Failed to initialize LPS22HB. Retrying...", LogTags::PRODUCER);
           } else {
               g_lps22hb_initialized = true;
               LOG_INF("[%s] LPS22HB Initialized Successfully.", LogTags::PRODUCER);
           }
       }

       if (!g_pav3015_initialized) {
           const uint16_t pav_addr = static_cast<uint16_t>(SensorID::PAV3015);
           auto res = SystemObjects::i2c().readWord(pav_addr, 0x00);
           if (!res.isOk()) {
               LOG_ERR("[%s] Failed to initialize PAV3015. Retrying...", LogTags::PRODUCER);
           } else {
               g_pav3015_initialized = true;
               LOG_INF("[%s] PAV3015 Initialized Successfully.", LogTags::PRODUCER);
           }
       }

       bool enqueued = false;
       if (state == ProducerState::ReadBME) {
           enqueued = enqueueCommand<SensorReadCmd>(PROCESSOR_Q, SensorReg::BME_DESC.id, SensorReg::BME_DESC.reg, SensorReg::BME_DESC.len);
           state = ProducerState::ReadLPS;
       } else if (state == ProducerState::ReadLPS) {
           bool enq1 = enqueueCommand<SensorReadCmd>(PROCESSOR_Q, SensorReg::LPS_P_DESC.id, SensorReg::LPS_P_DESC.reg, SensorReg::LPS_P_DESC.len);
           bool enq2 = enqueueCommand<SensorReadCmd>(PROCESSOR_Q, SensorReg::LPS_T_DESC.id, SensorReg::LPS_T_DESC.reg, SensorReg::LPS_T_DESC.len);
           enqueued = enq1 && enq2;
           state = ProducerState::ReadPAV;
       } else {
           enqueued = enqueueCommand<SensorReadCmd>(PROCESSOR_Q, SensorReg::PAV_DESC.id, SensorReg::PAV_DESC.reg, SensorReg::PAV_DESC.len);
           state = ProducerState::ReadBME;
       }

       (void)enqueued;
       k_msleep(500);
       atomic_set(&g_producer_alive, 1);
    } while(THREAD_LOOP_CONDITION);
}

#ifdef IS_TEST_ENVIRONMENT
void resetRtosCommandTestState() noexcept {
    g_bme280Calib = BME280Calibration{};
    g_bme280_initialized = false;
    g_lps22hb_initialized = false;
    g_pav3015_initialized = false;
    g_powerObserver.resetForTest();
}
void resetSensorReadCmdLastValsForTest() noexcept;
#endif

extern "C" void processor_thread(void *arg1, void *arg2, void *arg3) {
    ICommand* incoming_cmd;
    do {
        // Halt processing entirely while in SAFE_HALT
        while (g_powerObserver.isSleeping() || sys_context.getState() == SystemState::SAFE_HALT) {
            k_msleep(1000);
            atomic_set(&g_processor_alive, 1);
        }

        if (k_msgq_get(PROCESSOR_Q, &incoming_cmd, K_MSEC(500)) == 0) {
            incoming_cmd->execute();
            incoming_cmd->destroy();
        }

        atomic_set(&g_processor_alive, 1);
    } while(THREAD_LOOP_CONDITION);
}

extern "C" void logger_thread(void *arg1, void *arg2, void *arg3) {
    ICommand* incoming_cmd;
    do {
        // Halt processing entirely while in SAFE_HALT
        while (g_powerObserver.isSleeping() || sys_context.getState() == SystemState::SAFE_HALT) {
            k_msleep(1000);
            atomic_set(&g_logger_alive, 1);
        }

        if (k_msgq_get(LOGGER_Q, &incoming_cmd, K_MSEC(500)) == 0) {
            incoming_cmd->execute();
            incoming_cmd->destroy();
        }

        atomic_set(&g_logger_alive, 1);
    } while(THREAD_LOOP_CONDITION);
}

K_THREAD_DEFINE(producer_tid,  ThreadConfig::StackLarge, producer_thread,  NULL, NULL, NULL, ThreadConfig::PrioProducer,  0, 0);
K_THREAD_DEFINE(processor_tid, ThreadConfig::StackLarge, processor_thread, NULL, NULL, NULL, ThreadConfig::PrioProcessor, 0, 0);
extern const k_tid_t logger_tid;
K_THREAD_DEFINE(logger_tid,    ThreadConfig::StackLarge, logger_thread,    NULL, NULL, NULL, ThreadConfig::PrioLogger,    0, 0);

extern "C" {
    enum class TraceThread : uint8_t {
    PRODUCER = 0,
    PROCESSOR,
    LOGGER,
    INVALID
    };

    static TraceThread get_target_thread(struct k_thread *thread) {
        if (thread == producer_tid)  return TraceThread::PRODUCER;
        if (thread == processor_tid) return TraceThread::PROCESSOR;
        if (thread == logger_tid)    return TraceThread::LOGGER;
        return TraceThread::INVALID;
    }
    static inline void recordTraceEventLocked(struct k_thread* thread, TraceType type) noexcept {
        /* Caller already holds irq_lock() - do not lock again here. */
        if (get_target_thread(thread) == TraceThread::INVALID) {
            return;
        }

        const uint32_t slot = g_trace_head & TRACE_BUFFER_MASK;
        
        g_trace_buffer[slot].timestamp = k_cycle_get_32();
        g_trace_buffer[slot].thread = thread;
        g_trace_buffer[slot].type = type;
        
        g_trace_head = g_trace_head + 1;
    }

    void sys_trace_thread_switched_in_user(void) {
        unsigned int key = irq_lock();
        struct k_thread *thread = k_sched_current_thread_query();
        
        // Replaced get_target_thread_index with get_target_thread
        if (get_target_thread(thread) != TraceThread::INVALID) {
            atomic_inc(&g_switch_hook_hits);
            recordTraceEventLocked(thread, TraceType::SWITCH_IN);
        }
        irq_unlock(key);
    }
    
    void sys_trace_thread_switched_out_user(void) {
        unsigned int key = irq_lock();
        struct k_thread *thread = k_sched_current_thread_query();
        recordTraceEventLocked(thread, TraceType::SWITCH_OUT);
        irq_unlock(key);
    }
}
extern "C" void trace_logger_thread(void *arg1, void *arg2, void *arg3) {
    
    // Array to track the last SWITCH_OUT timestamp for PRODUCER, PROCESSOR, LOGGER
    uint32_t thread_out_times[3] = {0, 0, 0};

    while (true) {
        while (g_powerObserver.isSleeping() || sys_context.getState() == SystemState::SAFE_HALT) {
            k_msleep(1000);
        }
        
        uint32_t current_head = g_trace_head;

        if (current_head - g_trace_tail > TRACE_BUFFER_SIZE) {
            uint32_t skipped = (current_head - g_trace_tail) - TRACE_BUFFER_SIZE;
            g_trace_tail = current_head - TRACE_BUFFER_SIZE;
            LOG_WRN("Trace buffer overflow: skipped %u stale events", skipped);
        }

        while (g_trace_tail != g_trace_head) {
            uint32_t wrapped_idx = g_trace_tail & TRACE_BUFFER_MASK;
            TraceEvent event;

            unsigned int key = irq_lock();
            if (g_trace_head - g_trace_tail > TRACE_BUFFER_SIZE) {
                irq_unlock(key);
                break; 
            }
            event = g_trace_buffer[wrapped_idx];
            irq_unlock(key);

            TraceThread t_enum = get_target_thread(event.thread);
            const char* t_name = nullptr;
            
            switch (t_enum) {
                case TraceThread::PRODUCER:  t_name = "PRODUCER "; break;
                case TraceThread::PROCESSOR: t_name = "PROCESSOR"; break;
                case TraceThread::LOGGER:    t_name = "LOGGER   "; break;
                default: break;
            }

            if (t_name != nullptr) {
                uint32_t timestamp_us = k_cyc_to_us_floor32(event.timestamp);
                int t_idx = static_cast<int>(t_enum);

                if (event.type == TraceType::SWITCH_OUT) {
                    // Record the time the thread was suspended
                    thread_out_times[t_idx] = timestamp_us;
                } 
                else if (event.type == TraceType::SWITCH_IN) {
                    // Calculate how long the thread was preempted/blocked
                    uint32_t blocked_duration = timestamp_us - thread_out_times[t_idx];
                    
                    if (thread_out_times[t_idx] > 0) {
                        // Define our known, expected thread suspensions
                        bool is_fast_i2c_wait = (blocked_duration < 1000);
                        bool is_normal_500ms_sleep = (blocked_duration >= 490000 && blocked_duration <= 510000);

                        // If the delay is NOT expected, it is a real preemption!
                        if (!is_fast_i2c_wait && !is_normal_500ms_sleep) {
                            LOG_INF("[%s] Preemption delay: %u us", t_name, blocked_duration);
                        }
                    }
                }
            }
            g_trace_tail++;
        }

        k_msleep(50); 
    }
}
extern const k_tid_t trace_tid;
// Define the thread with the lowest priority to avoid starving your compute loops
K_THREAD_DEFINE(trace_tid, 1024, trace_logger_thread, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
