#pragma once
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/device.h>
#include <array>
#include <atomic>
#include <cstdint>

class DeviceContext;

class PowerManager;

class IPowerState {
public:
    virtual bool enter(PowerManager& pm) = 0;
    virtual IPowerState& execute(PowerManager& pm) = 0;
    virtual void exit(PowerManager& pm) = 0;
    virtual const char* getName() const = 0;
protected:
    ~IPowerState() = default;
};

class IPowerObserver {
public:
    virtual void beforeSleep() = 0;
    virtual void afterWakeup() = 0;
    virtual void sleepAborted() = 0;
    virtual ~IPowerObserver() = default;
};

class PowerManager {
private:
    IPowerState* current_state;

    std::atomic<uint32_t> last_activity_time{0};
    uint32_t last_sleep_time;
    uint32_t expected_wake_time;

    struct k_mutex state_mutex;
    struct k_mutex observer_mutex;
    atomic_t wake_pending;

    const struct device* rtc_dev;
    const struct device* i2c_dev;
    const struct device* uart_dev;
    const struct device* usb_dev;

    static constexpr size_t MAX_OBSERVERS = 8;
    std::array<IPowerObserver*, MAX_OBSERVERS> observers{};
    size_t observer_count;

    DeviceContext* fault_context;
    uint32_t consecutive_pm_failures;
    static constexpr uint32_t PM_FAILURE_FAULT_THRESHOLD = 3;

    void transitionTo(IPowerState& next_state);

    size_t captureObservers(std::array<IPowerObserver*, MAX_OBSERVERS>& out);

public:
    PowerManager();

    PowerManager(const PowerManager&) = delete;
    PowerManager& operator=(const PowerManager&) = delete;
    static PowerManager& getInstance();

    bool init(const struct device* rtc, const struct device* i2c,
              const struct device* uart, const struct device* usb,
              DeviceContext* fault_ctx = nullptr);

    void reportActivity();
    void processFSM();

    bool registerObserver(IPowerObserver* obs);
    void notifyBeforeSleep();
    void notifyAfterWakeup();
    void notifySleepAborted();

    void reportPmFailure();
    void resetPmFailures() { consecutive_pm_failures = 0; }

    uint32_t getLastActivityTime() const { return last_activity_time.load(); }
    const struct device* getRtcDev() const { return rtc_dev; }
    const struct device* getI2cDev() const { return i2c_dev; }
    const struct device* getUartDev() const { return uart_dev; }
    const struct device* getUsbDev() const { return usb_dev; }

    void recordSleepTime() { last_sleep_time = k_uptime_get_32(); }
    uint32_t getSleepTime() const { return last_sleep_time; }

    void setExpectedWakeTime(uint32_t time_ms) { expected_wake_time = time_ms; }
    uint32_t getExpectedWakeTime() const { return expected_wake_time; }

    static void rtc_alarm_handler(const struct device* dev, uint8_t chan_id,
                                   uint32_t ticks, void* user_data);

#ifdef IS_TEST_ENVIRONMENT
    void resetForTest();
#endif
};

class ActiveState : public IPowerState {
public:
    constexpr ActiveState() = default;
    bool enter(PowerManager& pm) override;
    IPowerState& execute(PowerManager& pm) override;
    void exit(PowerManager& pm) override;
    const char* getName() const override { return "ACTIVE"; }
    static ActiveState& getInstance();
};

class IdleState : public IPowerState {
public:
    constexpr IdleState() = default;
    bool enter(PowerManager& pm) override;
    IPowerState& execute(PowerManager& pm) override;
    void exit(PowerManager& pm) override;
    const char* getName() const override { return "IDLE"; }
    static IdleState& getInstance();
};

class StopState : public IPowerState {
private:
    bool sleep_prepared = false;

public:
    constexpr StopState() : sleep_prepared(false) {}
    bool enter(PowerManager& pm) override;
    IPowerState& execute(PowerManager& pm) override;
    void exit(PowerManager& pm) override;
    const char* getName() const override { return "STOP"; }
    static StopState& getInstance();

#ifdef IS_TEST_ENVIRONMENT
    void resetForTest() { sleep_prepared = false; }
#endif
};

