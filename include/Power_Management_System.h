#pragma once

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <cstdint>
#include <atomic>
#include <array>
#include <zephyr/sys/atomic.h>

constexpr size_t MAX_OBSERVERS = 8;
constexpr uint8_t PM_FAILURE_FAULT_THRESHOLD = 3;
class PowerManager;
class DeviceContext;
extern bool s_real_hw_sleep_active;
extern uint32_t s_elapsed_sleep_time_us;
extern uint32_t s_current_sleep_interval_us;

enum class StopWakeReason : uint8_t {
    WAKE_NONE,
    WAKE_RTC,
    WAKE_ACTIVITY,
    WAKE_UNKNOWN,
    WAKE_ABORTED
};

extern atomic_t s_activity_pending;
extern volatile StopWakeReason s_stop_wake_reason;
extern struct k_sem pm_wake_sem;
extern struct k_sem stop_wake_sem;
void advance_rtc_time(struct rtc_time& time, uint32_t seconds_to_add);
bool clear_rtc_alarm_a_pending();
int set_rtc_alarm(const struct device* rtc_dev, uint32_t interval_us, void* user_data);
void clear_rtc_alarm(const struct device* rtc_dev);
extern const k_tid_t processor_tid;
extern const k_tid_t producer_tid;
extern const k_tid_t logger_tid;
extern const k_tid_t battery_tid;
extern const k_tid_t shell_tid;
extern const k_tid_t bms_comm_tid;
extern const k_tid_t hr_prod_tid;
extern const k_tid_t disp_cons_tid;
extern const k_tid_t mem_mon_tid;
extern const k_tid_t trace_tid;

class IPowerObserver {
public:
    virtual ~IPowerObserver() = default;
    virtual void beforeSleep() = 0;
    virtual void afterWakeup() = 0;
    virtual void sleepAborted() = 0;
};


class IPowerState {
public:
    virtual ~IPowerState() = default;
    
    virtual bool enter(PowerManager& pm) = 0;
    virtual IPowerState& execute(PowerManager& pm) = 0;
    virtual void exit(PowerManager& pm) = 0;
    
    virtual const char* getName() const = 0;
};

class PowerManager {
public:
    static PowerManager& getInstance();

    bool init(const struct device* rtc, const struct device* i2c, 
              const struct device* uart, const struct device* usb, 
              const struct device* adc, DeviceContext* fault_ctx);

    bool registerObserver(IPowerObserver* obs);
    void reportActivity();
    void reportPmFailure();
    void processFSM();

    // Notifiers for observers
    void notifyBeforeSleep();
    void notifyAfterWakeup();
    void notifySleepAborted();

    // Getters and Setters used by concrete states
    uint32_t getLastActivityTime() const { return last_activity_time.load(); }
    void clearWakePending() { atomic_set(&wake_pending, 0); }
    
    // Consumes the wake flag atomically to prevent race conditions during STOP mode wake
    bool consumeWakePending(); 
    
    const struct device* getRtcDev() const { return rtc_dev; }
    const struct device* getI2cDev() const { return i2c_dev; }
    const struct device* getUartDev() const { return uart_dev; }
    const struct device* getUsbDev() const { return usb_dev; }
    const struct device* getAdcDev() const { return adc_dev; }

    void recordSleepTime() { last_sleep_time = k_uptime_get_32(); }
    void setExpectedWakeTime(uint32_t time) { expected_wake_time = time; }
    uint32_t getSleepTime() const { return last_sleep_time; }
    uint32_t getExpectedWakeTime() const { return expected_wake_time; }
    
    void resetPmFailures() { consecutive_pm_failures = 0; }

    // Static callback for RTC API
    static void rtc_alarm_handler(const struct device* dev, uint16_t id, void* user_data);

#ifdef IS_TEST_ENVIRONMENT
    void resetForTest();
#endif

    // Exposed for direct alarm manipulation in states
    atomic_t wake_pending;

private:
    PowerManager();
    ~PowerManager() = default;
    PowerManager(const PowerManager&) = delete;
    PowerManager& operator=(const PowerManager&) = delete;

    void transitionTo(IPowerState& next_state);
    size_t captureObservers(std::array<IPowerObserver*, MAX_OBSERVERS>& out);

    IPowerState* current_state;
    
    uint32_t last_sleep_time;
    uint32_t expected_wake_time;

    const struct device* rtc_dev;
    const struct device* i2c_dev;
    const struct device* uart_dev;
    const struct device* usb_dev;
    const struct device* adc_dev;

    size_t observer_count;
    std::array<IPowerObserver*, MAX_OBSERVERS> observers;
    
    DeviceContext* fault_context;
    uint32_t consecutive_pm_failures;

    std::atomic<uint32_t> last_activity_time;
};


class ActiveState : public IPowerState {
public:
    static ActiveState& getInstance();
    bool enter(PowerManager& pm) override;
    IPowerState& execute(PowerManager& pm) override;
    void exit(PowerManager& pm) override;
    const char* getName() const override { return "PM_STATE_ACTIVE"; }

private:
    ActiveState() = default;
};

class IdleState : public IPowerState {
public:
    static IdleState& getInstance();
    bool enter(PowerManager& pm) override;
    IPowerState& execute(PowerManager& pm) override;
    void exit(PowerManager& pm) override;
    const char* getName() const override { return "PM_STATE_SUSPEND_TO_IDLE"; }

private:
    IdleState() = default;
};

class StopState : public IPowerState {
public:
    static StopState& getInstance();
    bool enter(PowerManager& pm) override;
    IPowerState& execute(PowerManager& pm) override;
    void exit(PowerManager& pm) override;
    const char* getName() const override { return "PM_STATE_SUSPEND_TO_RAM"; }

#ifdef IS_TEST_ENVIRONMENT
    void resetForTest() { sleep_prepared = false; }
#endif

private:
    StopState() = default;
    bool sleep_prepared = false;
};
