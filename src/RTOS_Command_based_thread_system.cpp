#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
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

inline void logCommandError(const char* tag, uint32_t id, const char* msg) noexcept {
    LOG_ERR("[%s] #%u: %s", tag, id, msg);
}

struct alignas(8) MaxCommandSize { uint8_t buffer[PoolConfig::Size]; };
StaticPool<MaxCommandSize, PoolConfig::Elements> g_commandPool;
QueueStats g_queueStats;
static bool g_bme280_initialized = false;

static_assert(sizeof(SensorReadCmd) <= sizeof(MaxCommandSize), "SensorReadCmd exceeds memory pool size");
static_assert(sizeof(ComputeCmd) <= sizeof(MaxCommandSize), "ComputeCmd exceeds memory pool size");
static_assert(sizeof(PrintCmd) <= sizeof(MaxCommandSize), "PrintCmd exceeds memory pool size");
static_assert(alignof(MaxCommandSize) >= alignof(SensorReadCmd), "Alignment mismatch on SensorReadCmd");
static_assert(alignof(MaxCommandSize) >= alignof(ComputeCmd), "Alignment mismatch on ComputeCmd");
static_assert(alignof(MaxCommandSize) >= alignof(PrintCmd), "Alignment mismatch on PrintCmd");

K_MSGQ_DEFINE(processor_queue, sizeof(ICommand*), QueueConfig::Depth, QueueConfig::Alignment);
K_MSGQ_DEFINE(logger_queue, sizeof(ICommand*), QueueConfig::Depth, QueueConfig::Alignment);

class CycleProfiler {
private:
    const char* tag;
    uint32_t id;
    uint32_t start;
public:
    CycleProfiler(const char* t, uint32_t cmd_id) noexcept : tag(t), id(cmd_id), start(k_cycle_get_32()) {}
    ~CycleProfiler() {
        LOG_INF("[%s] #%u: Execution took %u cycles", tag, id, k_cycle_get_32() - start);
    }
};

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

ICommand::ICommand() : timestamp_queued(k_cycle_get_32()) {
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

uint32_t ICommand::queueDelay() const noexcept {
    return k_cycle_get_32() - timestamp_queued;
}

// ---------------------------------------------------------
// Global Object References for SystemObjects accessor
// ---------------------------------------------------------
extern DeviceContext sys_context;
extern I2CManager i2c_manager;

namespace SystemObjects {
    DeviceContext& context() { return sys_context; }
    I2CManager& i2c() { return i2c_manager; }
    PowerManager& power() { return PowerManager::getInstance(); } // Fixed: Use Meyer's Singleton
}

// ---------------------------------------------------------
// Power Observer implementation to gracefully suspend tasks 
// ---------------------------------------------------------
class ThreadSystemPowerObserver final : public IPowerObserver {
private:
    atomic_t is_sleeping;
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
    
    constexpr float TEMP_DIV   = 100.0f;
    constexpr float PRESS_DIV1 = 256.0f;
    constexpr float PRESS_DIV2 = 100.0f;
    constexpr float HUM_DIV    = 1024.0f;
}

namespace LPS22HBConst {
    constexpr float TEMP_DIV  = 100.0f;
    constexpr float PRESS_DIV = 4096.0f;
}

namespace PAV3015Const {
    constexpr float OFFSET = 0.1f;
    constexpr float LINEAR = 0.0000506f;
    constexpr float QUADRATIC = 0.000001f;
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
        auto t1 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_T1); if (!t1.isOk()) return false; 
        auto t2 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_T2); if (!t2.isOk()) return false; 
        auto t3 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_T3); if (!t3.isOk()) return false;
    
        g_bme280Calib.dig_T1 = t1.unwrap();
        g_bme280Calib.dig_T2 = static_cast<int16_t>(t2.unwrap());
        g_bme280Calib.dig_T3 = static_cast<int16_t>(t3.unwrap());
        return true;
    }

    [[nodiscard]] bool readPressureCalibration(uint16_t addr) noexcept {
        auto p1 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_P1); if (!p1.isOk()) return false; 
        auto p2 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_P2); if (!p2.isOk()) return false; 
        auto p3 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_P3); if (!p3.isOk()) return false; 
        auto p4 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_P4); if (!p4.isOk()) return false; 
        auto p5 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_P5); if (!p5.isOk()) return false; 
        auto p6 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_P6); if (!p6.isOk()) return false; 
        auto p7 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_P7); if (!p7.isOk()) return false; 
        auto p8 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_P8); if (!p8.isOk()) return false; 
        auto p9 = SystemObjects::i2c().readWord(addr, BME280CalibReg::DIG_P9); if (!p9.isOk()) return false; 
        
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
        auto h1 = SystemObjects::i2c().readRegister(addr, BME280CalibReg::DIG_H1); if (!h1.isOk()) return false; 
        auto h2_l = SystemObjects::i2c().readRegister(addr, BME280CalibReg::DIG_H2_L); if (!h2_l.isOk()) return false; 
        auto h2_m = SystemObjects::i2c().readRegister(addr, BME280CalibReg::DIG_H2_M); if (!h2_m.isOk()) return false; 
        auto h3 = SystemObjects::i2c().readRegister(addr, BME280CalibReg::DIG_H3); if (!h3.isOk()) return false; 
        auto h4_m = SystemObjects::i2c().readRegister(addr, BME280CalibReg::DIG_H4_M); if (!h4_m.isOk()) return false; 
        auto h_shared = SystemObjects::i2c().readRegister(addr, BME280CalibReg::DIG_H_SHARED); if (!h_shared.isOk()) return false;
        auto h5_m = SystemObjects::i2c().readRegister(addr, BME280CalibReg::DIG_H5_M); if (!h5_m.isOk()) return false;
        auto h6 = SystemObjects::i2c().readRegister(addr, BME280CalibReg::DIG_H6); if (!h6.isOk()) return false;
        
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

uint64_t SensorReadCmd::readMockData() const noexcept {
    static uint16_t counter = 0; 
    counter += MockValues::STEP; 
    const uint16_t offset = counter % 1000;

    switch (sensor_id) {
        case SensorID::BME280:
            return (((MockValues::BME_P_BASE + offset) << 4) << 40) | 
                   (((MockValues::BME_T_BASE + offset) << 4) << 16) | 
                   (MockValues::BME_H_BASE + offset);
        case SensorID::LPS22HB:
            if (reg_addr == SensorReg::LPS_T_DESC.reg) return MockValues::LPS_T_BASE + (offset % 500); 
            return MockValues::LPS_P_BASE + offset;
        case SensorID::PAV3015:
            return MockValues::PAV_BASE + (offset % 3426);
        default: return 0;
    }
}

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

void SensorReadCmd::execute() noexcept {
    CycleProfiler profiler(LogTags::READ, command_id);
    uint64_t raw = 0;

#ifdef CONFIG_BOARD_QEMU_CORTEX_M3
    k_msleep(50);
    raw = readMockData();
#else
    auto res = readHardwareData();
    if (!res.isOk()) { 
        logCommandError(LogTags::READ, command_id, "I2C Transaction Failed.");
        return;
    }
    raw = res.unwrap();
#endif 

    if (!enqueueCommand<ComputeCmd>(PROCESSOR_Q, sensor_id, reg_addr, raw)) {
        logCommandError(LogTags::READ, command_id, "Compute queue full. Discarding raw data.");
    }
}

ComputeCmd::ComputeCmd(SensorID s_id, uint8_t r_addr, uint64_t data) noexcept
    : sensor_id(s_id), reg_addr(r_addr), raw_data(data) {}

void ComputeCmd::execute() noexcept {
    CycleProfiler profiler(LogTags::COMPUTE, command_id);
    
    switch (sensor_id) {
        case SensorID::BME280: {
            if (!loadBME280Calibration(static_cast<uint16_t>(sensor_id))) { 
                logCommandError(LogTags::COMPUTE, command_id, "BME280 calibration aborted. Math halting.");
                return;
            }
            BME280Data result = BME280Math::decode(raw_data, g_bme280Calib);
            (void)printMeasurement(SensorID::BME280, result.temperature);
            (void)printMeasurement(SensorID::BME280_PRESS, result.pressure);
            (void)printMeasurement(SensorID::BME280_HUM, result.humidity);
            break;
        }
        case SensorID::LPS22HB: {
            float val = (reg_addr == SensorReg::LPS_T_DESC.reg) ? 
                        LPS22HBMath::decodeTemperature(raw_data) : 
                        LPS22HBMath::decodePressure(raw_data);
            SensorID v_id = (reg_addr == SensorReg::LPS_T_DESC.reg) ? SensorID::LPS22HB_TEMP : SensorID::LPS22HB;
            (void)printMeasurement(v_id, val);
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
    SensorID id;
    const char* name;
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


void producer_thread(void) {
    ProducerState state = ProducerState::ReadBME; 
    
    // Register the thread system with the power manager to respect Deep Sleep states
    SystemObjects::power().registerObserver(&g_powerObserver);
    
    do {
       // Guard I2C interactions so we don't access hardware while the bus is suspended
       if (SystemObjects::context().getState() != SystemState::SAFE_HALT && !g_powerObserver.isSleeping()) {
           if (!g_bme280_initialized) {
               const uint16_t bme_addr = static_cast<uint16_t>(SensorID::BME280);
               auto res1 = SystemObjects::i2c().writeRegister(bme_addr, SensorReg::BME280_CTRL_HUM, BME280Config::CTRL_HUM);
               auto res2 = SystemObjects::i2c().writeRegister(bme_addr, SensorReg::BME280_CTRL_MEAS, BME280Config::CTRL_MEAS);
               if (!res1.isOk() || !res2.isOk()) {
                   LOG_ERR("[%s] Failed to initialize BME280. Retrying...", LogTags::PRODUCER);
               } else {
                   g_bme280_initialized = true;
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
                // Implicitly ProducerState::ReadPAV
                enqueued = enqueueCommand<SensorReadCmd>(PROCESSOR_Q, SensorReg::PAV_DESC.id, SensorReg::PAV_DESC.reg, SensorReg::PAV_DESC.len);
                state = ProducerState::ReadBME;
            }
              
           // NOTE (behavior change): routine sensor sampling no longer counts
           // as PowerManager "activity". Previously every successful enqueue
           // called reportActivity(), which resets the 30s ACTIVE_TIMEOUT_MS
           // clock every 500ms -- meaning it could never elapse while this
           // thread kept sampling, making IDLE/STOP structurally unreachable
           // no matter how correct PowerManager's own implementation is.
           // This now matches heart_rate_producer_thread's existing pattern
           // in RTOS_Synchronization_Layer.cpp, which never called
           // reportActivity() for its own periodic reads either -- so this
           // brings the two producer threads into agreement rather than
           // introducing a new convention.
           //
           // ASSUMPTION THIS RELIES ON: sensor data going stale for up to
           // STOP_WAKEUP_US (60s) between wakes is acceptable for this
           // device's profile. The system now settles into STOP after ~35s
           // with no operator/fault activity (CLI input, illegal state
           // transitions, and triggerFault() all still call reportActivity()
           // elsewhere), and this thread naturally pauses via the
           // isSleeping() guard above until the next RTC wake or external
           // event. If continuous, uninterrupted sampling is actually a hard
           // requirement for this device, this is the wrong fix -- STOP would
           // need a different reachability condition entirely (e.g. a
           // duty-cycled sampling contract), not just restoring this call.
           (void)enqueued;
       }  
       k_msleep(500); 
    } while(THREAD_LOOP_CONDITION);
}

#ifdef IS_TEST_ENVIRONMENT
void resetRtosCommandTestState() noexcept {
    g_bme280Calib = BME280Calibration{};
    g_bme280_initialized = false;
    g_powerObserver.resetForTest();
}
#endif

void processor_thread(void) {
    ICommand* incoming_cmd;
    do {
        if (k_msgq_get(PROCESSOR_Q, &incoming_cmd, K_SECONDS(2)) == 0) {
            incoming_cmd->execute();
            incoming_cmd->destroy();
        } else {
            LOG_WRN("PROCESSOR: Queue timeout (no commands received)");
        }
    } while(THREAD_LOOP_CONDITION);
}

void logger_thread(void) {
    ICommand* incoming_cmd;
    do {
        if (k_msgq_get(LOGGER_Q, &incoming_cmd, K_SECONDS(2)) == 0) {
            incoming_cmd->execute();
            incoming_cmd->destroy();
        }
    } while(THREAD_LOOP_CONDITION);
}

K_THREAD_DEFINE(producer_tid,  ThreadConfig::StackSmall, producer_thread,  NULL, NULL, NULL, ThreadConfig::PrioProducer,  0, 0);
K_THREAD_DEFINE(processor_tid, ThreadConfig::StackLarge, processor_thread, NULL, NULL, NULL, ThreadConfig::PrioProcessor, 0, 0);
K_THREAD_DEFINE(logger_tid,    ThreadConfig::StackSmall, logger_thread,    NULL, NULL, NULL, ThreadConfig::PrioLogger,    0, 0);
