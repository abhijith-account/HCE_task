#pragma once

#include "Command_Core.h"
#include "Sensor_Common.h"
#include "BME280.h"
#include "LPS22HB.h"

/* Commands that format a measurement and send it to the log. */

class PrintCmd final : public ICommand {
private:
    SensorID sensor_id;
    float final_value;

    [[nodiscard]] const char* getSensorName() const noexcept;

public:
    PrintCmd(SensorID s_id, float val) noexcept;
    void execute() noexcept override final;
};

class PrintBME280Cmd final : public ICommand {
private:
    BME280Data data;

public:
    explicit PrintBME280Cmd(const BME280Data& d) noexcept;
    void execute() noexcept override final;
};

class PrintLPS22HBCmd final : public ICommand {
private:
    LPS22HBData data;

public:
    explicit PrintLPS22HBCmd(const LPS22HBData& d) noexcept;
    void execute() noexcept override final;
};

[[nodiscard]] bool printBME280Measurement(const BME280Data& data) noexcept;

[[nodiscard]] bool printLPS22HBMeasurement(const LPS22HBData& data) noexcept;

[[nodiscard]] bool printMeasurement(SensorID id, float value) noexcept;
