#pragma once
struct device {};
#define DEVICE_DT_GET(...) nullptr
#define DEVICE_DT_GET_ONE(...) nullptr
#define DEVICE_DT_GET_OR_NULL(...) nullptr
#define DT_NODELABEL(...) 0
#define DT_REG_ADDR(...) 0
#define DT_MTD_FROM_FIXED_PARTITION(...) 0
extern "C" bool device_is_ready(const struct device *dev);
