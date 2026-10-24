#pragma once

#include "Command_Core.h"
#include "Sensor_Common.h"
#include "Fault_Tolerant_I2C_Communication_Layer.h"

/* Commands that read a sensor and turn the raw value into a measurement. */

class SensorReadCmd final : public ICommand {
private:
    SensorID sensor_id;
    uint8_t reg_addr;
    ReadLength length;

    [[nodiscard]] Result<uint64_t> readHardwareData() const noexcept;

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

#ifdef IS_TEST_ENVIRONMENT
void resetSensorReadCmdLastValsForTest() noexcept;
#endif
