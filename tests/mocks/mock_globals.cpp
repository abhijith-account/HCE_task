#include <zephyr/device.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/fcb.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/rtc.h>

__attribute__((weak)) bool run_thread_once = false;

__attribute__((weak)) const device* i2c_hardware = nullptr;

__attribute__((weak)) bool atomic_cas(atomic_t *target, atomic_val_t old_value, atomic_val_t new_value) { return target->compare_exchange_strong(old_value, new_value); }

uint32_t virtual_uptime = 0;

void (*g_sim_timer_cb)(struct k_timer*) = nullptr;

int g_mutex_lock_force_errno = 0;
int g_mutex_lock_call_counter = 0;
int g_mutex_lock_fail_on_call_n = 0;
void* g_mutex_lock_target_ptr = nullptr;
int g_mutex_lock_target_call_counter = 0;
int g_mutex_lock_target_fail_on_call_n = 0;

bool g_adc_ready_mock = true;

int g_adc_read_force_errno = 0;

// Fault-injection controls for host-side I2C tests. Default state matches the
// original always-succeed mock behavior exactly.
int g_i2c_force_errno = 0;            // 0 = success; set to a negative errno (e.g. -ETIMEDOUT, -EBUSY, -EAGAIN, -ENODEV) to force EVERY I2C call to fail
bool g_device_ready_override = true;  // controls device_is_ready() return value

// Per-call-index fault injection, for covering long straight-line chains of
// sequential I2C reads (e.g. calibration loaders) where each individual read
// needs its own isolated failure to exercise every branch. g_i2c_call_counter
// increments on every mocked I2C transaction (read byte / burst read / write
// byte); when it reaches g_i2c_fail_on_call_n, that single call fails with
// g_i2c_fail_on_call_errno and every other call around it still succeeds.
// Disabled by default (0), and independent of g_i2c_force_errno -- g_i2c_force_errno
// still takes priority (blanket failure) if both are set.
int g_i2c_call_counter = 0;
int g_i2c_fail_on_call_n = 0;
int g_adc_sequence_init_dt_errno = 0;
int g_i2c_fail_on_call_errno = -5;    // -EIO by default
extern __attribute__((weak)) const k_tid_t processor_tid = (k_tid_t)0x10;
extern __attribute__((weak)) const k_tid_t producer_tid  = (k_tid_t)0x11;
extern __attribute__((weak)) const k_tid_t logger_tid    = (k_tid_t)0x12;
extern __attribute__((weak)) const k_tid_t battery_tid   = (k_tid_t)0x13;
extern __attribute__((weak)) const k_tid_t shell_tid     = (k_tid_t)0x14;
extern __attribute__((weak)) const k_tid_t bms_comm_tid  = (k_tid_t)0x15;
extern __attribute__((weak)) const k_tid_t hr_prod_tid   = (k_tid_t)0x16;
extern __attribute__((weak)) const k_tid_t disp_cons_tid = (k_tid_t)0x17;
extern  __attribute__((weak)) const k_tid_t mem_mon_tid   = (k_tid_t)0x18;
extern __attribute__((weak)) const k_tid_t trace_tid   = (k_tid_t)0x19;
__attribute__((weak)) atomic_t g_producer_alive = ATOMIC_INIT(1);
__attribute__((weak)) atomic_t g_processor_alive = ATOMIC_INIT(1);
__attribute__((weak)) atomic_t g_logger_alive = ATOMIC_INIT(1);
__attribute__((weak)) atomic_t g_hr_prod_alive = ATOMIC_INIT(1);
__attribute__((weak)) atomic_t g_disp_cons_alive = ATOMIC_INIT(1);
__attribute__((weak)) atomic_t g_bms_comm_alive = ATOMIC_INIT(1);
__attribute__((weak)) atomic_t g_batt_mon_alive = ATOMIC_INIT(1);
__attribute__((weak)) atomic_t g_mem_mon_alive = ATOMIC_INIT(1);
__attribute__((weak)) atomic_t g_shell_alive = ATOMIC_INIT(1);
extern "C" {
__attribute__((weak)) bool k_is_in_isr(void) { return false; }
__attribute__((weak)) int k_work_submit(struct k_work *work) { return 0; }
__attribute__((weak)) bool device_is_ready(const struct device *dev) { return g_device_ready_override; }
__attribute__((weak)) int wdt_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *cfg) { return 0; }
__attribute__((weak)) int wdt_setup(const struct device *dev, uint8_t options) { return 0; }
__attribute__((weak)) int wdt_feed(const struct device *dev, int channel_id) { return 0; }
__attribute__((weak)) void pm_policy_state_lock_get(uint8_t state, uint8_t substate_id) {}
__attribute__((weak)) void pm_policy_state_lock_put(uint8_t state, uint8_t substate_id) {}
__attribute__((weak)) int pm_device_action_run(const struct device *dev, enum pm_device_action action) { return 0; }
__attribute__((weak)) uint32_t counter_us_to_ticks(const struct device *dev, uint64_t us) { return 0; }
__attribute__((weak)) int counter_set_channel_alarm(const struct device *dev, uint8_t chan_id, const struct counter_alarm_cfg *alarm_cfg) { return 0; }
__attribute__((weak)) int counter_start(const struct device *dev) { return 0; }
__attribute__((weak)) int counter_cancel_channel_alarm(const struct device *dev, uint8_t chan_id) { return 0; }
__attribute__((weak)) int k_thread_stack_space_get(const void *thread, size_t *unused_ptr) { if (unused_ptr) { *unused_ptr = 0; } return 0; }
__attribute__((weak)) int k_work_cancel_delayable(struct k_work_delayable *dwork) { return 0; }
__attribute__((weak)) void k_work_init_delayable(struct k_work_delayable *dwork, void (*callback)(struct k_work *)) { dwork->work.handler = callback; }
__attribute__((weak)) int k_work_schedule(struct k_work_delayable *dwork, k_timeout_t delay) { return 0; }
__attribute__((weak)) k_tid_t k_current_get(void) { return 0; }
__attribute__((weak)) int uart_irq_update(const struct device *dev) { return 1; }
__attribute__((weak)) int uart_err_check(const struct device *dev) { return 0; }
__attribute__((weak)) int uart_irq_tx_ready(const struct device *dev) { return 1; }
__attribute__((weak)) int uart_fifo_fill(const struct device *dev, const uint8_t *tx_data, const int size) { return size; }
__attribute__((weak)) void uart_irq_tx_enable(const struct device *dev) {}
__attribute__((weak)) void uart_irq_tx_disable(const struct device *dev) {}
__attribute__((weak)) void uart_irq_rx_disable(const struct device *dev) {}
__attribute__((weak)) int i2c_read(const struct device *dev, uint8_t *buf, uint32_t num_bytes, uint16_t addr) { return 0; }
__attribute__((weak)) int i2c_write(const struct device *dev, const uint8_t *buf, uint32_t num_bytes, uint16_t addr) { return 0; }
__attribute__((weak)) int i2c_write_read(const struct device *dev, uint16_t addr, const void *write_buf, size_t num_write, void *read_buf, size_t num_read) { return 0; }
__attribute__((weak)) int i2c_reg_read_byte(const struct device *dev, uint16_t dev_addr, uint8_t reg_addr, uint8_t *value) { ++g_i2c_call_counter; if (g_i2c_force_errno != 0) { return g_i2c_force_errno; } if (g_i2c_fail_on_call_n != 0 && g_i2c_call_counter == g_i2c_fail_on_call_n) { return g_i2c_fail_on_call_errno; } if (value) { *value = 0; } return 0; }
__attribute__((weak)) int i2c_reg_write_byte(const struct device *dev, uint16_t dev_addr, uint8_t reg_addr, uint8_t value) { ++g_i2c_call_counter; if (g_i2c_force_errno != 0) { return g_i2c_force_errno; } if (g_i2c_fail_on_call_n != 0 && g_i2c_call_counter == g_i2c_fail_on_call_n) { return g_i2c_fail_on_call_errno; } return 0; }
__attribute__((weak)) int i2c_burst_read(const struct device *dev, uint16_t dev_addr, uint8_t start_addr, uint8_t *buf, uint32_t num_bytes) { ++g_i2c_call_counter; if (g_i2c_force_errno != 0) { return g_i2c_force_errno; } if (g_i2c_fail_on_call_n != 0 && g_i2c_call_counter == g_i2c_fail_on_call_n) { return g_i2c_fail_on_call_errno; } if (buf && num_bytes > 0) { memset(buf, 0, num_bytes); } return 0; }
__attribute__((weak)) int k_sem_init(struct k_sem *sem, unsigned int initial, unsigned int limit) { sem->count = initial; sem->limit = limit; return 0; }
__attribute__((weak)) void k_sem_give(struct k_sem *sem) { if (sem->count < sem->limit) { sem->count++; } }
__attribute__((weak)) int k_sem_take(struct k_sem *sem, k_timeout_t timeout) { if (sem->count > 0) { sem->count--; return 0; } return -EBUSY; }
__attribute__((weak)) int adc_read(const struct device *dev, const struct adc_sequence *sequence) { return g_adc_read_force_errno; }
__attribute__((weak)) int adc_raw_to_millivolts_dt(const struct adc_dt_spec *spec, int32_t *val) { if (val) *val = 2500; return 0; }
__attribute__((weak)) int fcb_init(int f_area_id, struct fcb *fcb) { return 0; }
__attribute__((weak)) int fcb_getnext(struct fcb *fcb, struct fcb_entry *loc) { return -1; }
__attribute__((weak)) int fcb_append(struct fcb *fcb, uint16_t len, struct fcb_entry *loc) { return 0; }
__attribute__((weak)) int fcb_append_finish(struct fcb *fcb, struct fcb_entry *append_loc) { return 0; }
__attribute__((weak)) int flash_area_open(uint8_t id, const struct flash_area **fa) { static struct flash_area dummy= {0, 4096}; *fa = &dummy; return 0; }
__attribute__((weak)) int rtc_set_time(const struct device *dev, const struct rtc_time *timeptr) { return 0; }
__attribute__((weak)) void flash_area_close(const struct flash_area *fa) {}
__attribute__((weak)) int flash_area_read(const struct flash_area *fa, off_t off, void *dst, size_t len) { return 0; }
__attribute__((weak)) int flash_area_write(const struct flash_area *fa, off_t off, const void *src, size_t len) { return 0; }
__attribute__((weak)) int rtc_get_time(const struct device *dev, struct rtc_time *timeptr) { return 0; }
__attribute__((weak)) int rtc_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask, const struct rtc_time *timeptr) { return 0; }
__attribute__((weak)) int rtc_alarm_set_callback(const struct device *dev, uint16_t id, void (*callback)(const struct device *, uint16_t, void *), void *user_data) { return 0; }
__attribute__((weak)) int flash_area_erase(const struct flash_area *fa, off_t off, size_t len) { return 0; }
__attribute__((weak)) int flash_area_get_sectors(int fa_id, uint32_t *count, struct flash_sector *sectors) { *count = 2; return 0; }
__attribute__((weak)) int fcb_rotate(struct fcb *fcb) { return 0; }
}
