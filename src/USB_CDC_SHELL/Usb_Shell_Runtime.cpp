// Usb_Shell_Runtime.cpp
// Shell runtime: the power observer that pauses the shell while the system sleeps, the diag_shell instance, the UsbShell::process()
// main loop (observer registration, sleep gating, line reading, dispatch, prompt, watchdog heartbeat) and shell_thread().
// process() lives here, not with the commands, because it uses the observer and g_shell_alive, which are private to this file.

#include "USB_CDC_Virtual_COM_Shell_Interface.h"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "Device_State_Machine+Watchdog.h"
#include "Power_Management_System.h" 

// Loop condition: always true on target; in the host tests it runs the loop body exactly once per run_thread_once.
#ifdef IS_TEST_ENVIRONMENT
    extern bool run_thread_once;
    static inline bool evaluate_and_clear_loop() noexcept {
        bool current = run_thread_once;
        run_thread_once = false;
        return current;
    }
    #define THREAD_LOOP_CONDITION evaluate_and_clear_loop()
#else
    #define THREAD_LOOP_CONDITION true
#endif

LOG_MODULE_DECLARE(USB_CLI, LOG_LEVEL_INF);

atomic_t g_shell_alive = ATOMIC_INIT(1);

namespace {

    class ShellPowerObserver final : public IPowerObserver {
    private:
        atomic_t is_sleeping{};
    public:
        ShellPowerObserver() { atomic_set(&is_sleeping, 0); }
        void beforeSleep() override { atomic_set(&is_sleeping, 1); }
        void afterWakeup() override { atomic_set(&is_sleeping, 0); }
        void sleepAborted() override { atomic_set(&is_sleeping, 0); }
        bool isSleeping() const noexcept { return atomic_get(&is_sleeping) != 0; }
    };
    
    ShellPowerObserver g_shellPowerObserver;
    atomic_t g_shellObserverRegistered = ATOMIC_INIT(0);

    void ensure_shell_observer_registered() {
        if (atomic_cas(&g_shellObserverRegistered, 0, 1)) {
            PowerManager::getInstance().registerObserver(&g_shellPowerObserver);
        }
    }
}

extern DeviceContext sys_context;
UsbShell diag_shell(&sys_context, getSmartBatteryInstance());

void UsbShell::process() {
    ensure_shell_observer_registered();

    if (!usb.init()) {
        LOG_ERR("USB initialization failed");
        return;
    }

    CommandBuffer cmd_buf;
    do {
        while (g_shellPowerObserver.isSleeping()) {
            k_msleep(200);
            atomic_set(&g_shell_alive, 1);
        }

        if (usb.isConnected() && usb.readLine(cmd_buf)) {
            PowerManager::getInstance().reportActivity();
            if (cmd_buf[0] != '\0') {
                dispatchCommand(std::string_view(cmd_buf.data()));
            }

            usb.transmit(PromptStr);
        }
        
        k_msleep(ShellPollMs);

        atomic_set(&g_shell_alive, 1);

    } while (THREAD_LOOP_CONDITION);
}

void shell_thread(void) {
    diag_shell.process();
}
extern const k_tid_t shell_tid;
K_THREAD_DEFINE(shell_tid, ShellStackSize, shell_thread, nullptr, nullptr, nullptr, ShellPriority, 0, 0);
