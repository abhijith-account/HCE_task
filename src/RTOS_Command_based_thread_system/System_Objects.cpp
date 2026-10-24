#include "System_Objects.h"

extern DeviceContext sys_context;
extern I2CManager i2c_manager;
extern bool g_usb_connected;

namespace SystemObjects {
    DeviceContext& context() { return sys_context; }
    I2CManager& i2c() { return i2c_manager; }
    PowerManager& power() { return PowerManager::getInstance(); }
}

ThreadSystemPowerObserver g_powerObserver;

atomic_t g_producer_alive = ATOMIC_INIT(1);
atomic_t g_processor_alive = ATOMIC_INIT(1);
atomic_t g_logger_alive = ATOMIC_INIT(1);
