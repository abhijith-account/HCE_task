// test_usb_shell_mocks.cpp
// Definitions of every global and Zephyr-API mock (UART, FCB/flash, watchdog, __wrap_snprintf) shared by the USB shell
// gtest files. Link this file into the test binary exactly once (run_usb_shell_tests also passes -Wl,--wrap=snprintf).

#include "test_usb_shell_common.h"

std::array<char,4096> mock_tx_buffer;
size_t mock_tx_index=0;
std::array<char,512> mock_rx_queue;
size_t mock_rx_head=0;
size_t mock_rx_tail=0;
uint32_t mock_dtr_state=0;
void (*mock_uart_irq_cb)(const device*, void*)=nullptr;
void* mock_uart_cb_data=nullptr;
bool mock_uart_line_ctrl_fail = false;
int mock_uart_line_ctrl_ret = 0;
const device* dummy_uart_dev=reinterpret_cast<const device*>(0xCDC);
bool mock_uart_callback_fail = false;
bool force_device_not_ready = false;
bool mock_nvs_write_fail = false;
bool mock_uart_irq_update_fail = false;
bool enable_snprintf_mock = false;
int mock_snprintf_call_count = 0;
int mock_snprintf_fail_on_call = -1;
int mock_snprintf_truncate_on_call = -1;
int mock_snprintf_exact_return_on_call = -1;
int mock_snprintf_exact_return_value = 0;

int mock_flash_read_call_count = 0;
int mock_flash_read_fail_on_call = -1;
std::array<uint8_t, 4096> mock_flash_area;
uint32_t mock_flash_offset = 0;
std::vector<MockFcbEntry> mock_fcb_entries;
size_t mock_fcb_iterator = 0;

extern "C" {
    int __wrap_snprintf(char *str, size_t size, const char *format, ...) {
        va_list args;
        if (enable_snprintf_mock) {
            mock_snprintf_call_count++;
            if (mock_snprintf_call_count == mock_snprintf_fail_on_call) return -1;
            if (mock_snprintf_call_count == mock_snprintf_truncate_on_call) return static_cast<int>(size + 10);
            if (mock_snprintf_call_count == mock_snprintf_exact_return_on_call) return mock_snprintf_exact_return_value;
        }
        va_start(args, format);
        int ret = vsnprintf(str, size, format, args);
        va_end(args);
        return ret;
    }
    void sys_reboot(int type){}
    int wdt_install_timeout(const struct device *dev,const struct wdt_timeout_cfg *cfg){ return 0; }
    int wdt_setup(const struct device *dev,uint8_t options){ return 0; }
    int wdt_feed(const struct device *dev,int channel_id){ return 0; }
    
    int fcb_init(int f_area_id, struct fcb *fcb) { return 0; }
    int fcb_rotate(struct fcb *fcb) { return 0; }
    int fcb_append(struct fcb *fcb, uint16_t len, struct fcb_entry *loc) {
        if (mock_nvs_write_fail) return -1;
        loc->fe_elem_off = mock_flash_offset;
        mock_fcb_entries.push_back({mock_flash_offset, len});
        mock_flash_offset += len;
        return 0;
    }
    int fcb_append_finish(struct fcb *fcb, struct fcb_entry *append_loc) { return 0; }
    int fcb_getnext(struct fcb *fcb, struct fcb_entry *loc) {
        if (loc->fe_sector == nullptr) {
            mock_fcb_iterator = 0;
            loc->fe_sector = (void*)1; 
        }
        if (mock_fcb_iterator < mock_fcb_entries.size()) {
            loc->fe_elem_off = mock_fcb_entries[mock_fcb_iterator].offset;
            mock_fcb_iterator++;
            return 0;
        }
        return -1;
    }
    int flash_area_write(const struct flash_area *fa, off_t off, const void *src, size_t len) {
        if (mock_nvs_write_fail) return -1;
        memcpy(&mock_flash_area[off], src, len);
        return 0;
    }
    int flash_area_read(const struct flash_area *fa, off_t off, void *dst, size_t len) {
        mock_flash_read_call_count++;
        if (mock_flash_read_call_count == mock_flash_read_fail_on_call) return -1;
        memcpy(dst, &mock_flash_area[off], len);
        return 0;
    }
    
    bool device_is_ready(const struct device *dev) { return !force_device_not_ready; }
    int uart_line_ctrl_get(const struct device *dev,uint32_t ctrl,uint32_t *val){
        if (mock_uart_line_ctrl_fail) return -1;
        if (mock_uart_line_ctrl_ret != 0) return mock_uart_line_ctrl_ret;
        (void)ctrl;  // the shell only ever queries DTR
        *val = mock_dtr_state;
        return 0;
    }
    void uart_poll_out(const struct device *dev,unsigned char c){
        if (mock_tx_index<mock_tx_buffer.size()) mock_tx_buffer[mock_tx_index++]=c;
    }
    int uart_irq_callback_user_data_set(const struct device *dev,void (*cb)(const struct device *,void *),void *user_data){
        if (mock_uart_callback_fail) return -1;
        mock_uart_irq_cb=cb;
        mock_uart_cb_data=user_data;
        return 0;
    }
    void uart_irq_rx_enable(const struct device *dev){}
    int uart_irq_update(const struct device *dev) { return mock_uart_irq_update_fail ? 0 : 1; }
    int uart_irq_rx_ready(const struct device *dev){ return (mock_rx_head!=mock_rx_tail)? 1:0; }
    int uart_fifo_read(const struct device *dev,uint8_t *tx_data,const int size){
        if (mock_rx_head!=mock_rx_tail){
            *tx_data=mock_rx_queue[mock_rx_tail%512];
            mock_rx_tail++;
            return 1;
        }
        return 0;
    }
}

void inject_mock_uart_data(std::string_view data){
    for (char c: data) mock_rx_queue[mock_rx_head++%512]=c;
    if (mock_uart_irq_cb) mock_uart_irq_cb(dummy_uart_dev,mock_uart_cb_data);
}
