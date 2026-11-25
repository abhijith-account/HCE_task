// test_Thread_Trace.cpp
// Tests for Thread_Trace.cpp: the scheduler trace hooks and trace_logger_thread().

#include "test_rtos_common.h"

extern "C" void sys_trace_thread_switched_in_user(void);
extern "C" void sys_trace_thread_switched_out_user(void);
extern atomic_t g_switch_hook_hits;
extern "C" void trace_logger_thread(void *arg1, void *arg2, void *arg3);
extern void test_poke_recent_invalid_trace_events(struct k_thread* invalid_thread) noexcept;

namespace {

using namespace std::chrono_literals;

class CurrentThreadSpoof {
public:
    ~CurrentThreadSpoof() { select(-1); }
    void select(int which) {
        switch (which) {
            case 0:  g_spoofed_current_thread = reinterpret_cast<uintptr_t>(producer_tid);  break;
            case 1:  g_spoofed_current_thread = reinterpret_cast<uintptr_t>(processor_tid); break;
            case 2:  g_spoofed_current_thread = reinterpret_cast<uintptr_t>(logger_tid);    break;
            default: g_spoofed_current_thread = 0;                                          break;
        }
    }
};

void setUptimeMs(uint32_t ms) { __atomic_store_n(&virtual_uptime, ms, __ATOMIC_RELAXED); }

struct TraceEventMirror {          // same layout as TraceEvent in the .cpp
    uint32_t timestamp;
    struct k_thread* thread;
    uint8_t type;                  // 0 = SWITCH_IN, 1 = SWITCH_OUT
};

bool pokeInvalidTraceEvents(struct k_thread* x) {
    (void)x;
    test_poke_recent_invalid_trace_events(
        reinterpret_cast<struct k_thread*>(static_cast<uintptr_t>(0xDEAD0)));
    return true;
}

std::atomic<bool> g_trace_thread_done{false};

void traceCleanup(void*) { g_trace_thread_done = true; }

void* traceThreadEntry(void*) {
    pthread_cleanup_push(traceCleanup, nullptr);
    trace_logger_thread(nullptr, nullptr, nullptr);
    pthread_cleanup_pop(1);
    return nullptr;
}

bool joinTraceThread(pthread_t t) {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!g_trace_thread_done && std::chrono::steady_clock::now() < deadline) msSleep(10);
    if (g_trace_thread_done) { pthread_join(t, nullptr); return true; }
    pthread_detach(t);
    return false;
}

} // namespace

TEST_F(RTOSCommandsTestSuite, TraceHooksSwitchInAndOut) {
    // Reset the hook hit counter
    atomic_set(&g_switch_hook_hits, 0);

    // Execute the hooks. In the gtest context, k_sched_current_thread_query() 
    // will return the test runner's thread, which will evaluate to TraceThread::INVALID.
    sys_trace_thread_switched_in_user();
    sys_trace_thread_switched_out_user();

    // Verify that the hooks safely ignore unregistered threads 
    // without crashing, and do not increment the target thread hit counter.
    EXPECT_EQ(atomic_get(&g_switch_hook_hits), 0);
}

TEST_F(RTOSCommandsTestSuite, TraceHooksRecordRegisteredThreads) {
    CurrentThreadSpoof spoof;
    atomic_set(&g_switch_hook_hits, 0);

    spoof.select(0);
    sys_trace_thread_switched_in_user();

    spoof.select(1);
    sys_trace_thread_switched_in_user();

    spoof.select(2);
    sys_trace_thread_switched_in_user();

    EXPECT_EQ(atomic_get(&g_switch_hook_hits), 3);

    spoof.select(-1);
    sys_trace_thread_switched_in_user();
    sys_trace_thread_switched_out_user();
    EXPECT_EQ(atomic_get(&g_switch_hook_hits), 3);
}

TEST_F(RTOSCommandsTestSuite, TraceLoggerThreadFullCoverage) {
    // Capture stdout/stderr to suppress expected trace overflow warnings during testing
    testing::internal::CaptureStdout();

    const uint32_t saved_uptime = virtual_uptime;
    CurrentThreadSpoof spoof;

    // Register g_powerObserver with the PowerManager.
    test_iterations_remaining = 0;
    producer_thread();

    auto hookOut = []() { sys_trace_thread_switched_out_user(); };
    auto hookIn  = []() { sys_trace_thread_switched_in_user(); };

    spoof.select(0);                                  // PRODUCER
    setUptimeMs(1);
    for (int i = 0; i < 4100; ++i) hookOut();         // > TRACE_BUFFER_SIZE -> overflow path

    bool poked = false;
    for (int attempt = 0; attempt < 8 && !poked; ++attempt) {
        hookOut();                                    // e0 OUT
        hookIn();                                     // e1 IN  <- type corrupted
        hookIn();                                     // e2 IN
        hookOut();                                    // e3 OUT <- thread corrupted
        hookIn();                                     // e4 IN
        poked = pokeInvalidTraceEvents(static_cast<struct k_thread*>(producer_tid));
    }
    EXPECT_TRUE(poked) << "could not locate the trace buffer to inject invalid events";

    setUptimeMs(100);  hookOut(); hookIn();                         //      0 us -> fast resume
    setUptimeMs(200);  hookOut(); setUptimeMs(202);  hookIn();      //   2000 us -> "Preemption delay"
    setUptimeMs(300);  hookOut(); setUptimeMs(795);  hookIn();      // 495000 us -> normal 500 ms sleep, silent
    setUptimeMs(900);  hookOut(); setUptimeMs(1500); hookIn();      // 600000 us -> "Preemption delay"
    
    spoof.select(1);
    setUptimeMs(1600); hookIn();
    setUptimeMs(1700); hookOut(); hookIn();

    spoof.select(2);
    setUptimeMs(1800); hookOut(); hookIn();

    spoof.select(-1);
    hookOut(); hookIn();

    SystemObjects::power().notifyBeforeSleep();
    g_trace_thread_done = false;
    pthread_t trace_th;
    ASSERT_EQ(pthread_create(&trace_th, nullptr, traceThreadEntry, nullptr), 0);
    msSleep(100);                                     // consumer is inside the isSleeping() loop

    SystemObjects::power().notifyAfterWakeup();
    SystemObjects::context().triggerFault("Trace coverage halt");
    SystemObjects::context().requestTransition(SystemState::SAFE_HALT);
    msSleep(100);

    SystemObjects::context().requestTransition(SystemState::INIT);
    msSleep(500);

    SystemObjects::power().notifyBeforeSleep();
    msSleep(100);
    flockfile(stdout);
    spoof.select(0);
    setUptimeMs(5000); hookOut();
    setUptimeMs(5300); hookIn();                      // 300000 us -> consumer logs
    SystemObjects::power().notifyAfterWakeup();
    msSleep(200);                                     // consumer is now blocked in printf()
    for (int i = 0; i < 4200; ++i) hookOut();         // head - tail > TRACE_BUFFER_SIZE
    spoof.select(-1);
    funlockfile(stdout);                              // consumer resumes -> hits the break
    msSleep(300);                                     // ... then drains the remaining events

    SystemObjects::power().notifyBeforeSleep();
    msSleep(100);
    spoof.select(0);
    setUptimeMs(9000); hookOut();
    setUptimeMs(9300); hookIn();                      // makes the consumer log -> cancellation point
    spoof.select(-1);
    pthread_cancel(trace_th);
    SystemObjects::power().notifyAfterWakeup();
    EXPECT_TRUE(joinTraceThread(trace_th)) << "trace_logger_thread did not stop";

    SystemObjects::context().requestTransition(SystemState::INIT);
    virtual_uptime = saved_uptime;

    (void)testing::internal::GetCapturedStdout();

    SUCCEED();
}
