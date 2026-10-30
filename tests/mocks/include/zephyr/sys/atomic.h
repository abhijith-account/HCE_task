#pragma once

#include <atomic>

#define ATOMIC_INIT(i) (i)

using atomic_t = std::atomic<int>;

using atomic_val_t = int;

static inline int atomic_inc(atomic_t* ptr) { return ptr->fetch_add(1); }

static inline void atomic_set(atomic_t* ptr, int val) { ptr->store(val); }

static inline int atomic_get(const atomic_t* ptr) { return ptr->load(); }

bool atomic_cas(atomic_t *target, atomic_val_t old_value, atomic_val_t new_value);

