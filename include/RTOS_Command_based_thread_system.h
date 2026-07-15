#pragma once

#include <zephyr/kernel.h>
#include <cstdint>
#include <utility>
#include "Fault_Tolerant_I2C_Communication_Layer.h"
#include "Device_State_Machine+Watchdog.h"   // for DeviceContext
#include "Power_Management_System.h"         // for PowerManager

namespace QueueConfig {
    constexpr size_t Depth = 10;
    constexpr size_t Alignment = 4;
}

namespace PoolConfig {
    constexpr size_t Elements = 16;
    constexpr size_t Size = 48;
}

namespace ThreadConfig {
    constexpr size_t StackSmall = 512;
    constexpr size_t StackLarge = 1024;
    constexpr int PrioProducer  = 5;
    constexpr int PrioProcessor = 6;
    constexpr int PrioLogger    = 7;
}

enum class SensorID : uint16_t {
    BME280       = 0x76,
    BME280_PRESS = 0x176,
    BME280_HUM   = 0x276,
    LPS22HB      = 0x5C,
    LPS22HB_TEMP = 0x15C, 
    PAV3015      = 0x28
};

enum class ReadLength : uint8_t {
    Word   = 2,
    Triple = 3,
    Block  = 8
};

enum class ProducerState : uint8_t {
    ReadBME,
    ReadLPS,
    ReadPAV
};

static_assert(sizeof(SensorID) == 2, "SensorID must be 16-bit");
static_assert(sizeof(ReadLength) == 1, "ReadLength must be 8-bit");
static_assert(QueueConfig::Depth > 0, "Queue depth must be greater than 0");
static_assert(PoolConfig::Elements >= QueueConfig::Depth, "Pool must support peak queue depth");
static_assert(ThreadConfig::PrioProducer < ThreadConfig::PrioProcessor, "Processor must have higher priority than Producer to prevent starvation");

namespace LogTags {
    constexpr char READ[]     = "READ";
    constexpr char COMPUTE[]  = "COMPUTE";
    constexpr char PRINT[]    = "PRINT";
    constexpr char PRODUCER[] = "PRODUCER";
}

struct SensorDescriptor {
    SensorID id;
    uint8_t reg;
    ReadLength len;
};

namespace SensorReg {
    constexpr uint8_t BME280_CTRL_HUM   = 0xF2;
    constexpr uint8_t BME280_CTRL_MEAS  = 0xF4;
    constexpr uint8_t BME280_DATA_START = 0xF7;
    constexpr SensorDescriptor BME_DESC = { SensorID::BME280, 0xF7, ReadLength::Block };
    
    constexpr SensorDescriptor LPS_P_DESC = { SensorID::LPS22HB, 0x28, ReadLength::Triple };
    constexpr SensorDescriptor LPS_T_DESC = { SensorID::LPS22HB, 0x2B, ReadLength::Word };
    
    constexpr SensorDescriptor PAV_DESC = { SensorID::PAV3015, 0x00, ReadLength::Word };
}

namespace BME280CalibReg {
    constexpr uint8_t DIG_T1 = 0x88; constexpr uint8_t DIG_T2 = 0x8A; constexpr uint8_t DIG_T3 = 0x8C;
    constexpr uint8_t DIG_P1 = 0x8E; constexpr uint8_t DIG_P2 = 0x90; constexpr uint8_t DIG_P3 = 0x92;
    constexpr uint8_t DIG_P4 = 0x94; constexpr uint8_t DIG_P5 = 0x96; constexpr uint8_t DIG_P6 = 0x98;
    constexpr uint8_t DIG_P7 = 0x9A; constexpr uint8_t DIG_P8 = 0x9C; constexpr uint8_t DIG_P9 = 0x9E;
    constexpr uint8_t DIG_H1 = 0xA1; constexpr uint8_t DIG_H2_L = 0xE1; constexpr uint8_t DIG_H2_M = 0xE2;
    constexpr uint8_t DIG_H3 = 0xE3; constexpr uint8_t DIG_H4_M = 0xE4; constexpr uint8_t DIG_H_SHARED = 0xE5;
    constexpr uint8_t DIG_H5_M = 0xE6; constexpr uint8_t DIG_H6 = 0xE7;
}

namespace BME280Config {
    constexpr uint8_t CTRL_HUM = 0x01;
    constexpr uint8_t CTRL_MEAS = 0x27;
}

namespace MockValues {
    constexpr uint16_t STEP = 17; 
    constexpr uint64_t BME_P_BASE = 512000ULL;
    constexpr uint64_t BME_T_BASE = 512000ULL;
    constexpr uint64_t BME_H_BASE = 25000ULL;
    constexpr uint64_t LPS_P_BASE = 4000000ULL;
    constexpr uint64_t LPS_T_BASE = 2500ULL;
    constexpr uint64_t PAV_BASE   = 409ULL;
}

struct alignas(4) BME280Calibration {
    bool is_loaded = false;
    uint16_t dig_T1; int16_t dig_T2; int16_t dig_T3;
    uint16_t dig_P1; int16_t dig_P2; int16_t dig_P3;
    int16_t dig_P4;  int16_t dig_P5; int16_t dig_P6;
    int16_t dig_P7;  int16_t dig_P8; int16_t dig_P9;
    uint8_t  dig_H1; int16_t dig_H2; uint8_t  dig_H3;
    int16_t  dig_H4; int16_t dig_H5; int8_t   dig_H6;
};

struct BME280Data {
    float temperature;
    float pressure;
    float humidity;
};

namespace BME280Math {
    [[nodiscard]] BME280Data decode(uint64_t raw_data, const BME280Calibration& calib) noexcept;
}

namespace LPS22HBMath {
    [[nodiscard]] float decodeTemperature(uint64_t raw) noexcept;
    [[nodiscard]] float decodePressure(uint64_t raw) noexcept;
}

namespace PAV3015Math {
    [[nodiscard]] float decodeAirflow(uint64_t raw) noexcept;
}

class ICommand {
public:
    uint32_t timestamp_queued;
    uint32_t command_id;
    
    ICommand();
    virtual ~ICommand() = default;
    
    ICommand(const ICommand&) = delete;
    ICommand& operator=(const ICommand&) = delete;
    ICommand(ICommand&&) = delete;
    ICommand& operator=(ICommand&&) = delete;
    
    virtual void execute() noexcept = 0; 
    
    void operator delete(void* ptr) noexcept;
    void destroy() noexcept;
    
    [[nodiscard]] uint32_t queueDelay() const noexcept; 
};

class SensorReadCmd final : public ICommand {
private:
    SensorID sensor_id;
    uint8_t reg_addr;
    ReadLength length;
    
    [[nodiscard]] Result<uint64_t> readHardwareData() const noexcept; 
    [[nodiscard]] uint64_t readMockData() const noexcept;

public:
    SensorReadCmd(SensorID s_id, uint8_t r_addr, ReadLength len) noexcept;
    void execute() noexcept override final; 
};

class ComputeCmd final : public ICommand {
private:
    SensorID sensor_id;
    uint8_t reg_addr;
    uint64_t raw_data; 
    
public:
    ComputeCmd(SensorID s_id, uint8_t r_addr, uint64_t data) noexcept;
    void execute() noexcept override final;
};

class PrintCmd final : public ICommand {
private:
    SensorID sensor_id;
    float final_value;

    [[nodiscard]] const char* getSensorName() const noexcept; 

public:
    PrintCmd(SensorID s_id, float val) noexcept;
    void execute() noexcept override final;
};

struct QueueStats {
    uint32_t commandsCreated = 0;
    uint32_t commandsDropped = 0;
    uint32_t loggerQueueFull = 0;
    uint32_t processorPeakDepth = 0;
    uint32_t loggerPeakDepth = 0;
};
extern QueueStats g_queueStats;

namespace SystemObjects {
    DeviceContext& context();
    I2CManager& i2c();
    PowerManager& power();
}

extern struct k_msgq processor_queue;
extern struct k_msgq logger_queue;   

constexpr auto PROCESSOR_Q = &processor_queue;
constexpr auto LOGGER_Q    = &logger_queue;

// FIXED: Moved the template implementation into the header to allow test instantiations
void* allocateCommandMemory() noexcept;
bool enqueueCommandRaw(k_msgq* queue, ICommand* cmd) noexcept;

template<typename CmdType, typename... Args>
[[nodiscard]] bool enqueueCommand(k_msgq* queue, Args&&... args) noexcept {
    void* mem = allocateCommandMemory();
    if (!mem) return false;
    ICommand* cmd = new(mem) CmdType(std::forward<Args>(args)...);
    if (!enqueueCommandRaw(queue, cmd)) {
        cmd->destroy();
        return false;
    }
    return true;
}

[[nodiscard]] bool printMeasurement(SensorID id, float value) noexcept;

void producer_thread(void);
void processor_thread(void);
void logger_thread(void);

extern const k_tid_t producer_tid;
extern const k_tid_t processor_tid;
extern const k_tid_t logger_tid;
