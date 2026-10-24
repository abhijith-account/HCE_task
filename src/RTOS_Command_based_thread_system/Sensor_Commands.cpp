#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "Sensor_Commands.h"
#include "Print_Commands.h"
#include "System_Objects.h"
#include "BME280.h"
#include "LPS22HB.h"
#include "PAV3015.h"

LOG_MODULE_DECLARE(COMMANDS, LOG_LEVEL_INF);

static_assert(sizeof(SensorReadCmd) <= sizeof(MaxCommandSize), "SensorReadCmd exceeds memory pool size");
static_assert(sizeof(ComputeCmd) <= sizeof(MaxCommandSize), "ComputeCmd exceeds memory pool size");
static_assert(alignof(MaxCommandSize) >= alignof(SensorReadCmd), "Alignment mismatch on SensorReadCmd");
static_assert(alignof(MaxCommandSize) >= alignof(ComputeCmd), "Alignment mismatch on ComputeCmd");

inline void logCommandError(const char* tag, uint32_t id, const char* msg) noexcept {
    LOG_ERR("[%s] #%u: %s", tag, id, msg);
}

SensorReadCmd::SensorReadCmd(SensorID s_id, uint8_t r_addr, ReadLength len) noexcept
    : sensor_id(s_id), reg_addr(r_addr), length(len) {}

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
        raw = PAV3015Driver::readMock();
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
        threshold = BME280Driver::ActivityThreshold;
    } else if (sensor_id == SensorID::LPS22HB && reg_addr == SensorReg::LPS_P_DESC.reg) {
        last_val_ptr = &last_raw_lps_p;
        threshold = LPS22HBDriver::ActivityThresholdPressure;
    } else if (sensor_id == SensorID::LPS22HB && reg_addr == SensorReg::LPS_T_DESC.reg) {
        last_val_ptr = &last_raw_lps_t;
        threshold = LPS22HBDriver::ActivityThresholdTemperature;
    } else if (sensor_id == SensorID::PAV3015) {
        last_val_ptr = &last_raw_pav;
        threshold = PAV3015Driver::ActivityThreshold;
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
            BME280Data result{};
            if (!BME280Driver::decodeRaw(SystemObjects::i2c(), raw_data, result)) {
                logCommandError(LogTags::COMPUTE, command_id, "BME280 calibration aborted. Math halting.");
                return;
            }
            (void)printBME280Measurement(result);
            break;
        }
        case SensorID::LPS22HB: {
            LPS22HBData result{};
            if (LPS22HBDriver::accumulate(reg_addr, raw_data, result)) {
                (void)printLPS22HBMeasurement(result);
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
