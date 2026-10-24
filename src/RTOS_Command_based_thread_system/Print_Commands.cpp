#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "Print_Commands.h"

LOG_MODULE_DECLARE(COMMANDS, LOG_LEVEL_INF);

static_assert(sizeof(PrintCmd) <= sizeof(MaxCommandSize), "PrintCmd exceeds memory pool size");
static_assert(sizeof(PrintBME280Cmd) <= sizeof(MaxCommandSize), "PrintBME280Cmd exceeds memory pool size");
static_assert(sizeof(PrintLPS22HBCmd) <= sizeof(MaxCommandSize), "PrintLPS22HBCmd exceeds memory pool size");
static_assert(alignof(MaxCommandSize) >= alignof(PrintCmd), "Alignment mismatch on PrintCmd");
static_assert(alignof(MaxCommandSize) >= alignof(PrintBME280Cmd), "Alignment mismatch on PrintBME280Cmd");
static_assert(alignof(MaxCommandSize) >= alignof(PrintLPS22HBCmd), "Alignment mismatch on PrintLPS22HBCmd");

bool printMeasurement(SensorID id, float value) noexcept {
    if (!enqueueCommand<PrintCmd>(LOGGER_Q, id, value)) {
        g_queueStats.loggerQueueFull++;
        LOG_ERR("[%s] Logger queue full. Metric dropped.", LogTags::PRINT);
        return false;
    }
    return true;
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
