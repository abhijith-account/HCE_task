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
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(MAIN_OS, LOG_LEVEL_INF);

// Add the external flag at the top with your other externs
extern bool g_usb_connected;
extern DeviceContext sys_context;
extern ZephyrWorkQueue status_work;

#ifdef CONFIG_USB_DEVICE_STACK
/* Callback to monitor physical USB hardware events */
static void usb_status_cb(enum usb_dc_status_code cb_status, const uint8_t *param) {
    switch (cb_status) {
        case USB_DC_DISCONNECTED:
        case USB_DC_SUSPEND:
            if (g_usb_connected) {
                LOG_WRN("USB Hardware Event: Cable Disconnected");
                g_usb_connected = false;
                
                // Immediately trigger the fault when the physical cable is pulled
                sys_context.triggerFault("USB CDC Cable Disconnected");
            }
            break;
        case USB_DC_CONFIGURED:
            if (!g_usb_connected) {
                LOG_INF("USB Hardware Event: Cable Connected & Configured");
                g_usb_connected = true;
            }
            break;
        default:
            break;
    }
}
#endif

int main(void)
{
    const struct device *console_dev  = nullptr;
    console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    
    if (device_is_ready(console_dev)) {
#ifdef CONFIG_USB_DEVICE_STACK
        /* Initialize USB Subsystem ONCE globally, attaching our new callback */
        int usb_err = usb_enable(usb_status_cb); // Changed from nullptr
        
        if (usb_err == 0 || usb_err == -EALREADY) {
            uint32_t dtr = 0;
            int timeout = 30; // 3 seconds max wait

            while (!dtr && timeout > 0) {
                // If the hardware callback fired and marked us unplugged, skip the wait
                if (!g_usb_connected) break; 
                
                int ret = uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr);
                if (ret == -ENOTSUP || ret == -ENOSYS) break; 
                
                k_msleep(100);
                timeout--;
            }
            
            if (dtr) k_msleep(250); 
        } else {
            LOG_ERR("Failed to initialize USB subsystem (err %d)", usb_err);
        }
#endif
    }

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
    
    status_work.schedule(K_SECONDS(1));
    return 0;
}
