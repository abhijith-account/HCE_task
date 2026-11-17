#pragma once

#include "Command_Core.h"
#include "Sensor_Common.h"
#include "BME280.h"
#include "LPS22HB.h"

/* Commands that format a measurement and send it to the log. */

class PrintCmd final : public ICommand {
private:
    SensorID sensor_id;
    bool is_degraded{false};
    uint8_t reliability_score{100U};
    float final_value;
    int64_t timestamp_ms{0};

    [[nodiscard]] const char* getSensorName() const noexcept;

public:
    PrintCmd(SensorID s_id, float val, bool degraded = false,
             uint8_t reliability = 100U, int64_t ts_ms = 0) noexcept;
    void execute() noexcept override;
};

class PrintBME280Cmd final : public ICommand {
private:
    BME280Data data;

public:
    explicit PrintBME280Cmd(const BME280Data& d) noexcept;
    void execute() noexcept override;
};

class PrintLPS22HBCmd final : public ICommand {
private:
    LPS22HBData data;

public:
    explicit PrintLPS22HBCmd(const LPS22HBData& d) noexcept;
    void execute() noexcept override;
};

[[nodiscard]] bool printBME280Measurement(const BME280Data& data) noexcept;

[[nodiscard]] bool printLPS22HBMeasurement(const LPS22HBData& data) noexcept;

[[nodiscard]] bool printMeasurement(SensorID id, float value, bool degraded = false,
                                    uint8_t reliability = 100U, int64_t ts_ms = 0) noexcept;
