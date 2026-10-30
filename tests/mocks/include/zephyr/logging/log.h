#pragma once
#include <stdio.h>
#define LOG_MODULE_REGISTER(...)
#define LOG_MODULE_DECLARE(...)
#define LOG_INF(fmt, ...) { printf("[INF] " fmt "\n", ##__VA_ARGS__); fflush(stdout); }
#define LOG_WRN(fmt, ...) { printf("[WRN] " fmt "\n", ##__VA_ARGS__); fflush(stdout); }
#define LOG_ERR(fmt, ...) { printf("[ERR] " fmt "\n", ##__VA_ARGS__); fflush(stdout); }
#define LOG_DBG(fmt, ...) { printf("[DBG] " fmt "\n", ##__VA_ARGS__); fflush(stdout); }
