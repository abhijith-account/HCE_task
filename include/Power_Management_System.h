#pragma once
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/device.h>
#include <array>
#include <atomic>
#include <cstdint>

class DeviceContext; // fwd decl only — full definition not needed here, keeps this header light

class PowerManager;

// ---------------------------------------------------------
// Task 33: State Pattern Interfaces
// ---------------------------------------------------------
class IPowerState {
public:
    virtual bool enter(PowerManager& pm) = 0;
    virtual IPowerState& execute(PowerManager& pm) = 0;
    virtual void exit(PowerManager& pm) = 0;
    virtual const char* getName() const = 0;
protected:
    ~IPowerState() = default;
};

// ---------------------------------------------------------
// Task 34: Observer Pattern Interfaces
// ---------------------------------------------------------
class IPowerObserver {
public:
    virtual void beforeSleep() = 0;
    virtual void afterWakeup() = 0;
    virtual void sleepAborted() = 0;
    virtual ~IPowerObserver() = default;
};

// ---------------------------------------------------------
// Core Manager
// ---------------------------------------------------------
class PowerManager {
private:
    IPowerState* current_state;

    std::atomic<uint32_t> last_activity_time{0};
    uint32_t last_sleep_time;
    uint32_t expected_wake_time;

    struct k_mutex state_mutex;
    struct k_mutex observer_mutex;
    atomic_t wake_pending; // Kept as atomic_t for Zephyr ISR compatibility

    const struct device* rtc_dev;
    const struct device* i2c_dev;
    const struct device* uart_dev;
    const struct device* usb_dev;

    static constexpr size_t MAX_OBSERVERS = 8;
    std::array<IPowerObserver*, MAX_OBSERVERS> observers{};
    size_t observer_count;

    // Optional escalation path. nullptr is a valid "no fault reporting
    // configured" state — every call site checks it before dereferencing.
    //
    // REENTRANCY NOTE: transitionTo() calls fault_context->triggerFault()
    // while state_mutex is held. If the fault_context implementation calls
    // back into PowerManager (DeviceContext::triggerFault() does, via
    // reportActivity()), that relies on the underlying mutex supporting
    // recursive locking by its own owning thread — see the matching note in
    // DeviceContext::triggerFault()'s implementation. A fault_context that
    // isn't safe to have call back in this way must not be registered here.
    DeviceContext* fault_context;
    uint32_t consecutive_pm_failures;
    static constexpr uint32_t PM_FAILURE_FAULT_THRESHOLD = 3;

    // NOTE ON LOCKING: transitionTo() runs the target state's enter()/exit()
    // — which can briefly block on device PM calls and observer callbacks —
    // while holding state_mutex. This keeps a transition atomic w.r.t.
    // concurrent reportActivity()/processFSM() callers, at the cost of those
    // callers blocking for the (expected-short) duration of a transition.
    //
    // NOTE ON FAILURE SEMANTICS: the old state's exit() always runs before
    // the new state's enter() is attempted. If enter() then fails, this is a
    // fail-FORWARD design (falls to IdleState), not a rollback — the old
    // state is never re-entered. Any future exit() implementation must
    // therefore be safe to run even when nothing afterward is guaranteed to
    // successfully enter; do not give exit() side effects that depend on the
    // next state's enter() having succeeded (or assume exit() can be undone
    // by calling enter() on the same state again).
    //
    // CONTRACT FOR IPowerObserver IMPLEMENTATIONS: beforeSleep()/afterWakeup()/
    // sleepAborted() run synchronously from inside a transition. They must
    // return promptly and must NEVER call back into PowerManager (including
    // reportActivity()) — that call would block on state_mutex, which this
    // same thread already holds via transitionTo(), stalling every other
    // thread waiting on PowerManager for as long as the observer runs.
    void transitionTo(IPowerState& next_state); // Caller MUST hold state_mutex.

    // Snapshots up to MAX_OBSERVERS registered observers into out under
    // observer_mutex and returns the count. Clamps defensively even though
    // observer_count is only ever set by registerObserver() (which already
    // enforces the bound) — cheap insurance against future refactors.
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

    // Called by state implementations when a STOP entry/exit step fails.
    // After PM_FAILURE_FAULT_THRESHOLD consecutive failures, escalates to
    // the registered DeviceContext (if any) instead of failing silently
    // forever.
    //
    // NOT independently synchronized: both are only ever called from within
    // an IPowerState::enter()/exit() implementation, which only ever runs
    // inside transitionTo() while the caller holds state_mutex. That's the
    // entire synchronization story for consecutive_pm_failures — do not call
    // either of these from anywhere else without adding real protection.
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
    // Meyer's singletons (this class + the state singletons below) persist
    // for the life of the test binary. Host-side GTest suites need an
    // explicit way to reset that shared state between cases — mirrors
    // resetRtosCommandTestState() used elsewhere in this codebase for the
    // same reason.
    void resetForTest();
#endif
};

// ---------------------------------------------------------
// Concrete States
// ---------------------------------------------------------
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
    constexpr IdleState() = default;   // ADD THIS LINE
    bool enter(PowerManager& pm) override;
    IPowerState& execute(PowerManager& pm) override;
    void exit(PowerManager& pm) override;
    const char* getName() const override { return "IDLE"; }
    static IdleState& getInstance();
};

class StopState : public IPowerState {
private:
    // Global to this singleton: tracks if hardware was actually suspended
    // by *this* enter() call, so exit() knows whether there's anything to
    // undo. Safe because all transitions are serialized via state_mutex.
    bool sleep_prepared = false;

public:
    constexpr StopState() : sleep_prepared(false) {}   // ADD THIS LINE
    bool enter(PowerManager& pm) override;
    IPowerState& execute(PowerManager& pm) override;
    void exit(PowerManager& pm) override;
    const char* getName() const override { return "STOP"; }
    static StopState& getInstance();

#ifdef IS_TEST_ENVIRONMENT
    void resetForTest() { sleep_prepared = false; }
#endif
};
