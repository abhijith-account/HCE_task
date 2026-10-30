#pragma once
#include <stdint.h>
#include <zephyr/pm/pm.h>
extern "C" void pm_policy_state_lock_get(uint8_t state, uint8_t substate_id);
extern "C" void pm_policy_state_lock_put(uint8_t state, uint8_t substate_id);
