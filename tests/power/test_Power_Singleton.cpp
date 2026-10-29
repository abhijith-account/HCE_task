// test_Power_Singleton.cpp
// Global gtest environment that exercises the thread-safe first-use initialisation of the
// PowerManager / ActiveState / IdleState / StopState singletons (race + guard-contention path).

#include "test_power_common.h"
#include "test_Power_patch.h"

namespace {
template <typename Fn>
void RaceForFirstInit(Fn get_instance, int num_threads = 64) {
    std::atomic<bool> start{false};
    std::vector<std::thread> threads;
    threads.reserve(num_threads);
    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back([&] {
            while (!start.load(std::memory_order_acquire)) {
                std::this_thread::yield();
            }
            get_instance();
        });
    }
    start.store(true, std::memory_order_release);
    for (auto& t : threads) t.join();
}

#ifdef PM_TEST_CAN_PATCH_CODE
// A function-local static's thread-safe-init guard has a branch that is only
// taken when __cxa_guard_acquire() returns 0, i.e. "another thread finished
// constructing the object while I was waiting".  A multi-thread first-use race
// hits that only by luck (the winner usually finishes before anyone else gets
// there), which made line `static PowerManager instance;` flip between 4/4 and
// 3/4 branch coverage from run to run.
//
// Deterministic version, single-threaded:
//   1. make sure the singleton exists,
//   2. clear its init guard so the next call takes the slow path,
//   3. temporarily make __cxa_guard_acquire() return 0,
//   4. call getInstance()  -> constructor is skipped, existing instance returned,
//   5. restore the code and mark the guard "initialised" again.
template <typename Fn>
void CoverGuardContendedPath(const char* guard_symbol_part, Fn get_instance) {
    uint8_t* guard = pm_test_patch::find_own_symbol(guard_symbol_part, STT_OBJECT, 8);
    void* acquire = dlsym(RTLD_DEFAULT, "__cxa_guard_acquire");
    if (guard == nullptr || acquire == nullptr) {
        std::fprintf(stderr, "[ WARN ] cannot locate guard '%s' / __cxa_guard_acquire; "
                             "singleton init-guard branch left to chance\n", guard_symbol_part);
        return;
    }

    get_instance();                                           // 1. instance exists
    *reinterpret_cast<volatile uint64_t*>(guard) = 0;         // 2. force slow path

    uint8_t orig[8];
    if (!pm_test_patch::patch_return_false(static_cast<uint8_t*>(acquire), orig)) {
        *reinterpret_cast<volatile uint64_t*>(guard) = 1;
        std::fprintf(stderr, "[ WARN ] mprotect failed; singleton init-guard branch left to chance\n");
        return;
    }
    get_instance();                                           // 3+4. acquire() == 0
    pm_test_patch::restore(static_cast<uint8_t*>(acquire), orig);  // 5.
    *reinterpret_cast<volatile uint64_t*>(guard) = 1;
}
#endif

class SingletonRaceEnvironment : public ::testing::Environment {
public:
    void SetUp() override {
        RaceForFirstInit([] { return &PowerManager::getInstance(); });
        RaceForFirstInit([] { return &ActiveState::getInstance(); });
        RaceForFirstInit([] { return &IdleState::getInstance(); });
        RaceForFirstInit([] { return &StopState::getInstance(); });
#ifdef PM_TEST_CAN_PATCH_CODE
        CoverGuardContendedPath("GVZN12PowerManager11getInstance", [] { return &PowerManager::getInstance(); });
        CoverGuardContendedPath("GVZN11ActiveState11getInstance",  [] { return &ActiveState::getInstance(); });
        CoverGuardContendedPath("GVZN9IdleState11getInstance",     [] { return &IdleState::getInstance(); });
        CoverGuardContendedPath("GVZN9StopState11getInstance",     [] { return &StopState::getInstance(); });
#endif
    }
};

::testing::Environment* const race_env =
    ::testing::AddGlobalTestEnvironment(new SingletonRaceEnvironment);
}
