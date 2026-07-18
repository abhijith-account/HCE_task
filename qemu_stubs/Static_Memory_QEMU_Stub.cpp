#include "Static_Memory+MISRA_Compliance_Layer.h"

/*
 * QEMU stub:
 * StaticPool is template/header-based, so RTOS_Command_based_thread_system.cpp
 * can still use StaticPool<SensorReadCmd,16>.
 *
 * We only avoid memory_monitor_thread(), because the real file calls
 * thread_analyzer_print(), which requires CONFIG_THREAD_ANALYZER=y.
 */
