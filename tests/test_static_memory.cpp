#include <gtest/gtest.h>
#include <cstddef>
#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <thread>
#include <chrono>
#include "Static_Memory+MISRA_Compliance_Layer.h"

struct DummyPayload{
    uint32_t sensor_id;
    uint32_t timestamp;
    float data_value;
};

#define private public
#define protected public
#include "Device_State_Machine+Watchdog.h"
#undef private
#undef protected

int run_thread_iterations;

extern DeviceContext sys_context;
extern const k_tid_t trace_tid = (k_tid_t)0x19;

/* ---------------------------------------------------------------------------
 * k_thread_stack_space_get() interposer.
 *
 * Needed to reach the "stack info unavailable" branch of memory_monitor_thread().
 * It relies on the linker option
 *     target_link_options(run_static_memory_tests PRIVATE -Wl,--wrap=k_thread_stack_space_get)
 * so that calls from Static_Memory+MISRA_Compliance_Layer.cpp land here. The real
 * function is never called: every thread reports 128 free bytes except the one
 * selected through g_stack_fail_tid, which reports -EINVAL.
 * ------------------------------------------------------------------------- */
static const void* g_stack_fail_tid = nullptr;

extern "C" int __wrap_k_thread_stack_space_get(const void* thread, size_t* unused_ptr) {
    if ((g_stack_fail_tid != nullptr) && (thread == g_stack_fail_tid)) {
        return -EINVAL;
    }
    if (unused_ptr != nullptr) {
        *unused_ptr = 128U;
    }
    return 0;
}

class StaticMemoryTestSuite:public::testing::Test{
    protected:
        SystemState saved_state_{SystemState::INIT};

        void SetUp() override{
            saved_state_ = sys_context.getState();
            run_thread_iterations = 0;
            g_stack_fail_tid = nullptr;
        }

        void TearDown() override{
            run_thread_iterations = 0;
            g_stack_fail_tid = nullptr;
            sys_context.current_state = saved_state_;
        }
};

TEST_F(StaticMemoryTestSuite, AllocationAndFreeRoundTrip){
    StaticPool<DummyPayload,4> pool;

    void* ptr1= pool.allocate();
    EXPECT_NE(ptr1,nullptr)<<"Pool failed to allocate from an empty state!";

    pool.deallocate(ptr1);

    void* ptr2=pool.allocate();
    EXPECT_EQ(ptr1,ptr2)<<"Pool  did not reuse the recently freed memory block";
}

TEST_F(StaticMemoryTestSuite,HandlesOverflowSafely){
    constexpr size_t POOL_SIZE=3;
    StaticPool<DummyPayload,POOL_SIZE> pool;
    std::array<void*,POOL_SIZE> allocated_ptrs;

    for (size_t i=0;i<POOL_SIZE;i++){
        allocated_ptrs[i]=pool.allocate();
        EXPECT_NE(allocated_ptrs[i],nullptr)<<"Failed to allocate block"<<i;
    }

    testing::internal::CaptureStdout();
    void* overflow_ptr=pool.allocate();
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);

    EXPECT_TRUE(output.find("[ERR] StaticPool Out of Memory! Pool Size: 3")!= std::string_view::npos)<<"Expected OOM error not printed! Actual output: "<<output;

    EXPECT_EQ(overflow_ptr,nullptr)<<"Pool overflowed its static boundaries!";

    pool.deallocate(allocated_ptrs[1]);

    void* recovered_ptr=pool.allocate();
    EXPECT_EQ(recovered_ptr,allocated_ptrs[1])<<"Pool failed to recover after freeing a block!";
}

TEST_F(StaticMemoryTestSuite, RejectsInavlidAndNullPointers){
    StaticPool<DummyPayload,2> pool;

    void* valid_ptr=pool.allocate();
    EXPECT_NE(valid_ptr,nullptr);

    EXPECT_NO_FATAL_FAILURE(pool.deallocate(nullptr))<<"Deallocating nullptr caused a system crash!";

    uint32_t rogue_variable=0xDEADBEEF;
    void* rogue_ptr=&rogue_variable;

    testing::internal::CaptureStdout();
    EXPECT_NO_FATAL_FAILURE(pool.deallocate(rogue_ptr))<<"Deallocating an out-of-bounds pointer caused a system crash!";
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);

    EXPECT_TRUE(output.find("[ERR] Invalid pointer passed to deallocate")!= std::string_view::npos)<<"Expected invalid pointer error not printed! Actual output: "<<output;

    void* second_ptr=pool.allocate();
    EXPECT_NE(second_ptr,nullptr);

    void* third_ptr=pool.allocate();
    EXPECT_EQ(third_ptr,nullptr)<<"Pool state corrupted! Rogue deallocate mistakenly freed";
}

TEST_F(StaticMemoryTestSuite,MemoryIsProperlyAligned){
    StaticPool<DummyPayload,4> pool;

    void* ptr=pool.allocate();
    ASSERT_NE(ptr,nullptr);

    uintptr_t numeric_address=reinterpret_cast<uintptr_t>(ptr);
    EXPECT_EQ(numeric_address%alignof(DummyPayload),0)<<"Memory returned by StaticPool is not correctly aligned for the payload type";
}

bool run_thread_once=false;

extern void memory_monitor_thread(void);

TEST_F(StaticMemoryTestSuite,ExecutesMemoryMonitorThread){
    EXPECT_NO_FATAL_FAILURE(memory_monitor_thread());
}

TEST_F(StaticMemoryTestSuite, ThreadLoopConditionBranches)
{
    sys_context.current_state = SystemState::RUNNING;

    run_thread_iterations = 31;

    testing::internal::CaptureStdout();
    EXPECT_NO_FATAL_FAILURE(memory_monitor_thread());

    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);

    EXPECT_TRUE(
        output.find("=== [System Health] Thread Stack Watermarks ===")
        != std::string_view::npos
    );

    sys_context.current_state = SystemState::INIT;
}

TEST_F(StaticMemoryTestSuite, ThreadSkipsLoggingOnSafeHalt) {

    sys_context.current_state = SystemState::SAFE_HALT;

    run_thread_once = false;

    testing::internal::CaptureStdout();
    EXPECT_NO_FATAL_FAILURE(memory_monitor_thread());
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);

    EXPECT_TRUE(output.find("=== [System Health] Thread Stack Watermarks ===") == std::string_view::npos)
        << "Thread failed to suppress logging during SAFE_HALT!";

    sys_context.current_state = SystemState::INIT;
}


/* Covers the "stack info unavailable" else-branch (line 118). */
TEST_F(StaticMemoryTestSuite, StackInfoUnavailableIsLogged)
{
    sys_context.current_state = SystemState::RUNNING;
    g_stack_fail_tid = reinterpret_cast<const void*>(trace_tid);

    run_thread_iterations = 30;   // 31 passes -> the 30 s watermark report runs once

    testing::internal::CaptureStdout();
    EXPECT_NO_FATAL_FAILURE(memory_monitor_thread());
    const std::string raw_output = testing::internal::GetCapturedStdout();

    EXPECT_NE(raw_output.find("stack info unavailable (err -22)"), std::string::npos)
        << "Failure branch not reached. Add -Wl,--wrap=k_thread_stack_space_get to the "
           "link options of run_static_memory_tests. Actual output: " << raw_output;
    EXPECT_NE(raw_output.find("unused"), std::string::npos)
        << "Success branch for the other threads was not logged";
}

/* Covers MemoryPowerObserver::{beforeSleep,afterWakeup,sleepAborted} and the
 * sleep "holding pattern" loop body (lines 68-80, 101-103). */
TEST_F(StaticMemoryTestSuite, SleepHoldingLoopAndObserverCallbacks)
{
    using namespace std::chrono_literals;

    sys_context.current_state = SystemState::INIT;

    // Registers the observer; the sleeping loop condition is false here.
    run_thread_iterations = 0;
    memory_monitor_thread();

    // beforeSleep(): the thread must park in the holding loop until woken.
    PowerManager::getInstance().notifyBeforeSleep();
    atomic_set(&g_mem_mon_alive, 0);   // only the holding loop (or end of pass) sets it again

    run_thread_iterations = 0;
    std::thread monitor(memory_monitor_thread);

    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while ((atomic_get(&g_mem_mon_alive) == 0) && (std::chrono::steady_clock::now() < deadline)) {
        std::this_thread::sleep_for(1ms);
    }
    EXPECT_NE(atomic_get(&g_mem_mon_alive), 0) << "Monitor never entered the sleep holding loop";

    // afterWakeup(): releases the thread, which then finishes its single pass.
    PowerManager::getInstance().notifyAfterWakeup();
    monitor.join();

    // sleepAborted()
    PowerManager::getInstance().notifyBeforeSleep();
    PowerManager::getInstance().notifySleepAborted();

    // Observer is awake again: a normal pass must complete without parking.
    run_thread_iterations = 0;
    EXPECT_NO_FATAL_FAILURE(memory_monitor_thread());
}
