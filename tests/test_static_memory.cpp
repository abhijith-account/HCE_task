#include <gtest/gtest.h>
#include <cstddef>
#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include "Static_Memory+MISRA_Compliance_Layer.h"

struct DummyPayload{
    uint32_t sensor_id;
    uint32_t timestamp;
    float data_value;
};

class StaticMemoryTestSuite:public::testing::Test{
    protected:
        void SetUp() override{
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

TEST_F(StaticMemoryTestSuite, ThreadLoopConditionBranches){
    run_thread_once=true;
    
    testing::internal::CaptureStdout();
    EXPECT_NO_FATAL_FAILURE(memory_monitor_thread());
    const auto raw_output = testing::internal::GetCapturedStdout();
    std::string_view output(raw_output);
    
    EXPECT_TRUE(output.find("=== [System Health] Thread Stack Watermarks ===")!=std::string_view::npos);
}
