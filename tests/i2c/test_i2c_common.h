// test_i2c_common.h
// Shared includes, extern declarations, state helpers and the I2CFaultRecoveryTestSuite fixture for the I2C gtest files.
// The mock definitions live in test_i2c_mocks.cpp (link it into the test binary exactly once).
#pragma once

#include <gtest/gtest.h>
#include "Fault_Tolerant_I2C_Communication_Layer.h"
#include "Static_Memory+MISRA_Compliance_Layer.h"
#include "Power_Management_System.h"
#include "Device_State_Machine+Watchdog.h"
#include <new>
#include <string>
#include <string_view>

extern int mock_i2c_err_code;
extern uint8_t mock_read_data[8];
extern const device* mock_dev;
extern bool g_device_ready_override;
extern uint32_t virtual_uptime;
extern int g_i2c_force_errno;
extern void test_update_cache(uint32_t key, uint64_t value, bool calibrated);
extern bool test_get_cached_value(uint32_t key, uint64_t* out_val, uint32_t max_age_ms);
extern void test_set_reliability(uint32_t key, uint8_t score);
extern void resetI2CCacheForTests() noexcept;
extern void exhaustI2CCachePool();
extern void resetI2CCachePool();
extern void ensure_mutex_initialized();
extern DeviceContext sys_context;   // defined by the firmware / test shim
extern void test_set_sample_count(uint32_t,uint32_t);

/* get_cached_value() only emits its "Fallback Rejected" logs while the system
 * is RUNNING. DeviceContext has no setState(), so the helpers below probe for
 * whichever state-changing method your DeviceContext actually provides
 * (requestTransition() is the one the firmware itself uses)
 * (checked at compile time, so this always builds).                         */
namespace state_probe {
    template <int N> struct Rank : Rank<N - 1> {};
    template <> struct Rank<0> {};

    template <class C> auto set(C& c, SystemState s, Rank<9>) -> decltype((void)c.requestTransition(s), bool{}) { (void)c.requestTransition(s); return true; }
    template <class C> auto set(C& c, SystemState s, Rank<8>) -> decltype((void)c.setState(s), bool{})       { c.setState(s);       return true; }
    template <class C> auto set(C& c, SystemState s, Rank<7>) -> decltype((void)c.transitionTo(s), bool{})   { c.transitionTo(s);   return true; }
    template <class C> auto set(C& c, SystemState s, Rank<6>) -> decltype((void)c.changeState(s), bool{})    { c.changeState(s);    return true; }
    template <class C> auto set(C& c, SystemState s, Rank<5>) -> decltype((void)c.setSystemState(s), bool{}) { c.setSystemState(s); return true; }
    template <class C> auto set(C& c, SystemState s, Rank<4>) -> decltype((void)c.requestState(s), bool{})   { c.requestState(s);   return true; }
    template <class C> auto set(C& c, SystemState s, Rank<3>) -> decltype((void)c.forceState(s), bool{})     { c.forceState(s);     return true; }
    template <class C> auto set(C& c, SystemState s, Rank<2>) -> decltype((void)c.transition(s), bool{})     { c.transition(s);     return true; }
    template <class C> auto set(C& c, SystemState s, Rank<1>) -> decltype((void)c.enterState(s), bool{})     { c.enterState(s);     return true; }
    template <class C> bool set(C&, SystemState, Rank<0>) { return false; }   /* no setter found */
}

inline bool trySetSystemState(SystemState s) {
    return state_probe::set(sys_context, s, state_probe::Rank<9>{});
}

inline void forceSystemRunning() {
    (void)trySetSystemState(SystemState::RUNNING);
}

/* Used by the three tests that assert on "Fallback Rejected" log lines. */
#define REQUIRE_SYSTEM_RUNNING()                                                        \
    ASSERT_TRUE(sys_context.getState() == SystemState::RUNNING)                         \
        << "sys_context is not RUNNING, so get_cached_value() suppresses its logs. "    \
           "requestTransition(RUNNING) was not accepted from the current state; drive the state machine to RUNNING in forceSystemRunning()."

inline void walkBackToRunning() {
    if (sys_context.getState() == SystemState::FAULT)     (void)sys_context.requestTransition(SystemState::SAFE_HALT);
    if (sys_context.getState() == SystemState::SAFE_HALT) (void)sys_context.requestTransition(SystemState::INIT);
    if (sys_context.getState() == SystemState::INIT)      (void)sys_context.requestTransition(SystemState::RUNNING);
}

class I2CFaultRecoveryTestSuite : public ::testing::Test {
  protected:
      SystemState saved_state_{};

      void SetUp() override {
          saved_state_ = sys_context.getState();
          forceSystemRunning();
          g_i2c_force_errno = 0;
          mock_i2c_err_code = 0;
          for(int i=0; i<8; i++) mock_read_data[i] = 0;
          resetI2CCacheForTests();
          ensure_mutex_initialized();
      }

      void TearDown() override {
          g_i2c_force_errno = 0;
          mock_i2c_err_code = 0;
          g_device_ready_override = true;
          (void)trySetSystemState(saved_state_);
      }
};
