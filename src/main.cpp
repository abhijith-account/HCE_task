#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#ifdef CONFIG_USB_DEVICE_STACK
#include <zephyr/usb/usb_device.h>
#endif
#include <zephyr/drivers/uart.h>
#include <cerrno>

#include "RTOS_Command_based_thread_system.h"
#include "RTOS_Synchronization_Layer.h"
#include "Device_State_Machine+Watchdog.h"
#include "Smart_Battery_System.h"
#include "Persistent_Configuration_System.h"
#include "Static_Memory+MISRA_Compliance_Layer.h"
#include "Power_Management_System.h"

#include <zephyr/debug/thread_analyzer.h>
#include <new>

LOG_MODULE_REGISTER(MAIN_OS, LOG_LEVEL_INF);

extern DeviceContext sys_context;
extern ZephyrWorkQueue status_work;

int main(void)
{
    const struct device *console_dev  = nullptr;
    console_dev=DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (device_is_ready(console_dev)) {
        
#ifdef CONFIG_USB_DEVICE_STACK
        /* Initialize USB Subsystem ONCE globally */
        int usb_err = usb_enable(nullptr);
        
        /* Accept 0 (Success) or -EALREADY (-120, already initialized) */
        if (usb_err == 0 || usb_err == -EALREADY) {
            uint32_t dtr = 0;
            int timeout = 30; // 3 seconds max wait for early logs (100ms * 30)

            while (!dtr && timeout > 0) {
                int ret = 0 ;
                ret = uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr);
                if (ret != 0) {
                    /* If line control isn't supported, break immediately */
                    break; 
                }
                k_msleep(100);
                timeout--;
            }
            
            /* Brief delay to allow desktop serial terminal software to render */
            if (dtr) {
                k_msleep(250); 
            }
        } else {
            LOG_ERR("Failed to initialize USB subsystem (err %d)", usb_err);
        }
#endif
    }

    /* 2. Resume boot sequence - logs will now safely hit the terminal */
    LOG_INF("Command-Based RTOS Booting");

    ConfigStore& config = ConfigStore::getInstance();

    if (config.init()) {
        config.validateEndurance(ConfigKey::ALARM_THRESHOLD_BASE);

        uint16_t infusion_rate = 0;

        if (!config.get(ConfigKey::INFUSION_RATE_BASE, infusion_rate)) {
            LOG_WRN("First boot detected. Setting default infusion rate.");
            config.set(ConfigKey::INFUSION_RATE_BASE, static_cast<uint16_t>(50));
        } else {
            LOG_INF("Loaded Infusion Rate from NVS: %u mL/hr", infusion_rate);
        }
    }

    sys_context.requestTransition(SystemState::RUNNING);

    status_work.schedule(K_SECONDS(1));

    return 0;
}
