#pragma once

#include <stdint.h>

#ifdef __cplusplus

extern "C" {

#endif

static inline uint16_t sys_cpu_to_be16(uint16_t val) { return __builtin_bswap16(val); }

static inline uint16_t sys_be16_to_cpu(uint16_t val) { return __builtin_bswap16(val); }

#ifdef __cplusplus

}

#endif

