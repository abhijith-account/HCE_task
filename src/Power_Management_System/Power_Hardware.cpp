// Power_Hardware.cpp
// What is left of the hardware glue after the split:
//   - suspend_background_threads() / resume_background_threads()  (all builds)
//   - usb_reconnect_work: delayed work that re-attaches USB after a STOP wake   (real hardware body; global name, the gtest file declares it extern)
//   - pm_device_suspend_tolerant() / pm_device_resume_tolerant()                (MPS2 build only)
// The register-level code lives in Power_Hardware_GpioPeripherals.cpp, Power_Hardware_StopPwr.cpp and Power_Hardware_Clocks.cpp.

#include "Power_Internal.h"

LOG_MODULE_DECLARE(PWR_SYS, LOG_LEVEL_INF);

namespace pm_internal {

void suspend_background_threads() {
    k_thread_suspend(processor_tid);
    k_thread_suspend(producer_tid);
    k_thread_suspend(logger_tid);
    k_thread_suspend(battery_tid);
    k_thread_suspend(shell_tid);
    k_thread_suspend(bms_comm_tid);
    k_thread_suspend(hr_prod_tid);
    k_thread_suspend(disp_cons_tid);
    k_thread_suspend(mem_mon_tid);
    k_thread_suspend(trace_tid);
}

void resume_background_threads() {
    k_thread_resume(processor_tid);
    k_thread_resume(producer_tid);
    k_thread_resume(logger_tid);
    k_thread_resume(battery_tid);
    k_thread_resume(shell_tid);
    k_thread_resume(bms_comm_tid);
    k_thread_resume(hr_prod_tid);
    k_thread_resume(disp_cons_tid);
    k_thread_resume(mem_mon_tid);
    k_thread_resume(trace_tid);
}

} // namespace pm_internal

using namespace pm_internal;

namespace {
void usb_reconnect_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

#ifndef CONFIG_BOARD_MPS2_AN386
    usb_restore_pins_and_reenable();
#ifdef CONFIG_USB_DEVICE_STACK
    int err = usb_dc_attach();
    if (err == 0) {
        LOG_INF("USB re-attached after STOP wake; awaiting host re-enumeration");
    } else {
        LOG_ERR("Failed to re-attach USB after STOP wake (err %d)", err);
    }
#endif
#endif
}
}

K_WORK_DELAYABLE_DEFINE(usb_reconnect_work, usb_reconnect_work_handler);

#ifdef CONFIG_BOARD_MPS2_AN386

bool pm_device_suspend_tolerant(const struct device* dev, const char* name) {
    if (!dev) {
        return true;
    }

    int rc = pm_device_action_run(dev, PM_DEVICE_ACTION_SUSPEND);
    if (rc == 0 || rc == -EALREADY || rc == -ENOSYS || rc == -ENOTSUP) {
        return true;
    }

    LOG_ERR("%s suspend failed: %d", name, rc);
    return false;
}

bool pm_device_resume_tolerant(const struct device* dev, const char* name) {
    if (!dev) {
        return true;
    }

    int rc = pm_device_action_run(dev, PM_DEVICE_ACTION_RESUME);
    if (rc == 0 || rc == -EALREADY || rc == -ENOSYS || rc == -ENOTSUP) {
        return true;
    }

    LOG_ERR("%s resume failed: %d", name, rc);
    return false;
}
#endif
