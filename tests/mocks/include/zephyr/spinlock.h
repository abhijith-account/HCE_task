#pragma once
#include <sched.h>

struct k_spinlock {
    int locked;
};
typedef int k_spinlock_key_t;

static inline k_spinlock_key_t k_spin_lock(struct k_spinlock *l) {
    if (l != nullptr) {
        while (__atomic_exchange_n(&l->locked, 1, __ATOMIC_ACQUIRE) != 0) {
            sched_yield();
        }
    }
    return 0;
}

static inline void k_spin_unlock(struct k_spinlock *l, k_spinlock_key_t key) {
    (void)key;
    if (l != nullptr) {
        __atomic_store_n(&l->locked, 0, __ATOMIC_RELEASE);
    }
}
