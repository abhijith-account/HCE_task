#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#define printk printf

typedef int32_t k_timeout_t;
#define K_NO_WAIT 0
#define K_MSEC(ms) (ms)
#define K_SECONDS(s) ((s)*1000)
#define K_USEC(us) ((us)/1000)
#ifndef EBUSY
#define EBUSY 16
#endif
#ifndef ENOMSG
#define ENOMSG 42
#endif
#define K_LOWEST_APPLICATION_THREAD_PRIO 15

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t virtual_uptime;

static inline int64_t k_uptime_get(void) { return static_cast<int64_t>(virtual_uptime); }
static inline uint32_t k_uptime_get_32(void) { return virtual_uptime; }
static inline uint32_t k_cycle_get_32(void) { return virtual_uptime * 1000; }
static inline uint32_t k_cyc_to_us_floor32(uint32_t t) { return t; }
static inline int k_msleep(int ms) { virtual_uptime += ms; return 0; }
bool k_is_in_isr(void);
static inline unsigned int irq_lock(void) { return 0; }
static inline void irq_unlock(unsigned int key) { (void)key; }

struct k_mutex { int locked; };
struct k_sem { unsigned int count; unsigned int limit; };
struct k_work { void (*handler)(struct k_work *); };
struct k_work_delayable { struct k_work work; };
struct k_timer { void (*expiry_fn)(struct k_timer*); void* dummy; };
struct k_thread;

#define K_FOREVER -1

static inline int k_mutex_init(struct k_mutex *mutex) { return 0; }
extern int g_mutex_lock_force_errno;
extern int g_mutex_lock_call_counter;
extern int g_mutex_lock_fail_on_call_n;
extern void* g_mutex_lock_target_ptr;
extern int g_mutex_lock_target_call_counter;
extern int g_mutex_lock_target_fail_on_call_n;
static inline int k_mutex_lock(struct k_mutex *mutex, int timeout) { ++g_mutex_lock_call_counter; if (g_mutex_lock_force_errno != 0) return g_mutex_lock_force_errno; if (g_mutex_lock_fail_on_call_n != 0 && g_mutex_lock_call_counter == g_mutex_lock_fail_on_call_n) return -EAGAIN; if (g_mutex_lock_target_ptr != nullptr && (void*)mutex == g_mutex_lock_target_ptr) { ++g_mutex_lock_target_call_counter; if (g_mutex_lock_target_fail_on_call_n != 0 && g_mutex_lock_target_call_counter == g_mutex_lock_target_fail_on_call_n) return -EAGAIN; } return 0; }
static inline int k_mutex_unlock(struct k_mutex *mutex) { return 0; }

int k_sem_init(struct k_sem *sem, unsigned int initial, unsigned int limit);
void k_sem_give(struct k_sem *sem);
int k_sem_take(struct k_sem *sem, k_timeout_t timeout);
void k_work_init_delayable(struct k_work_delayable *dwork, void (*callback)(struct k_work *));
int k_work_schedule(struct k_work_delayable *dwork, k_timeout_t delay);
int k_work_cancel_delayable(struct k_work_delayable *dwork);
int k_work_submit(struct k_work *work);

extern void (*g_sim_timer_cb)(struct k_timer*);
static inline void k_timer_init(struct k_timer *timer, void (*expiry_fn)(struct k_timer *), void (*stop_fn)(struct k_timer *)) { g_sim_timer_cb = expiry_fn; }

static inline void k_timer_start(struct k_timer *timer, k_timeout_t duration, k_timeout_t period) {}
static inline void k_timer_stop(struct k_timer *timer) {}

typedef void* k_tid_t;
static inline void k_yield(void) {}
static inline int k_thread_priority_get(k_tid_t thread) { return (int)(uintptr_t)thread; }
static inline void k_thread_priority_set(k_tid_t thread, int prio) {}
static inline void k_thread_suspend(k_tid_t thread) {}
static inline void k_thread_resume(k_tid_t thread) {}
static inline void k_sched_lock(void) {}
static inline void k_sched_unlock(void) {}
extern k_tid_t k_current_get(void);
static inline struct k_thread *k_sched_current_thread_query(void) { return (struct k_thread*)k_current_get(); }
int k_thread_stack_space_get(const void *thread, size_t *unused_ptr);

struct k_msgq { size_t msg_size; uint32_t max_msgs; uint32_t num_msgs; void* buffer; };
#define K_MSGQ_DEFINE(name, q_msg_size, q_max_msgs, q_align) static char __msgq_buf_##name[(q_msg_size) * (q_max_msgs)]; struct k_msgq name = { .msg_size = (q_msg_size), .max_msgs = (q_max_msgs), .num_msgs = 0, .buffer = __msgq_buf_##name };
#define K_SEM_DEFINE(name, initial_count, count_limit) struct k_sem name = { .count = (initial_count), .limit = (count_limit) };
#define K_MUTEX_DEFINE(name) struct k_mutex name = {0};
#define K_TIMER_DEFINE(name, expiry_fn, stop_fn) struct k_timer name = {expiry_fn, nullptr};
#define K_WORK_DELAYABLE_DEFINE(name, fn) struct k_work_delayable name = { { fn } };
#define K_WORK_DEFINE(name, fn) struct k_work name = { fn };
#define ARG_UNUSED(x) (void)(x);
static inline void k_msgq_init(struct k_msgq *q, void *buffer, size_t msg_size, uint32_t max_msgs) { q->msg_size = msg_size; q->max_msgs = max_msgs; q->num_msgs = 0; q->buffer = buffer; }
static inline int k_msgq_put(struct k_msgq *q, const void *data, k_timeout_t timeout) { if(q->num_msgs >= q->max_msgs) return -ENOMSG; memcpy((char*)q->buffer + (q->num_msgs * q->msg_size), data, q->msg_size); q->num_msgs++; return 0; }
static inline int k_msgq_get(struct k_msgq *q, void *data, k_timeout_t timeout) { if(q->num_msgs == 0) return -ENOMSG; memcpy(data, q->buffer, q->msg_size); q->num_msgs--; memmove(q->buffer, (char*)q->buffer + q->msg_size, q->num_msgs * q->msg_size); return 0; }
static inline void k_msgq_purge(struct k_msgq *q) { q->num_msgs = 0; }
static inline uint32_t k_msgq_num_free_get(struct k_msgq *q) { return q->max_msgs - q->num_msgs; }

#ifdef __cplusplus
}
#endif

#ifndef CONTAINER_OF
#define CONTAINER_OF(ptr, type, field) ((type *)(((char *)(ptr)) - offsetof(type, field)))
#endif

#ifndef k_work_delayable_from_work
#define k_work_delayable_from_work(w) CONTAINER_OF(w, struct k_work_delayable, work)
#endif

#ifndef K_THREAD_DEFINE
#define K_THREAD_DEFINE(name, stack_size, entry, p1, p2, p3, prio, options, delay) extern const k_tid_t name; const k_tid_t name = (k_tid_t)(uintptr_t)prio;
#endif
