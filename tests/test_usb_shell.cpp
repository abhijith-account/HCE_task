#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <string_view>
#include <cstdint>
#include <string>
#include <cstdarg>
#include <algorithm>
#include <vector>

#define private public
#define protected public
#include "USB_CDC_Virtual_COM_Shell_Interface.h"
#undef private
#undef protected
#include "Device_State_Machine+Watchdog.h"
#include "Smart_Battery_System.h"
#include "Persistent_Configuration_System.h"
#include "Power_Management_System.h"

#define MOCK_UART_DTR_CTRL 1
#ifndef ENOSYS
#define ENOSYS 38
#endif

static std::array<char,512> mock_tx_buffer;
static size_t mock_tx_index=0;
extern UsbShell diag_shell;
static std::array<char,512> mock_rx_queue;
static size_t mock_rx_head=0;
static size_t mock_rx_tail=0;
static uint32_t mock_dtr_state=0;
static void (*mock_uart_irq_cb)(const device*, void*)=nullptr;
static void* mock_uart_cb_data=nullptr;
bool mock_uart_line_ctrl_fail = false;
int mock_uart_line_ctrl_ret = 0;
const device* dummy_uart_dev=reinterpret_cast<const device*>(0xCDC);
bool mock_uart_callback_fail = false;
extern DeviceContext sys_context;
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
extern uint32_t virtual_uptime;

struct MockFcbEntry {
    uint32_t offset;
    uint16_t len;
};
static std::array<uint8_t, 4096> mock_flash_area;
static uint32_t mock_flash_offset = 0;
static std::vector<MockFcbEntry> mock_fcb_entries;
static size_t mock_fcb_iterator = 0;

extern bool test_parseIntToken(std::string_view token, int& out_value) noexcept;
extern std::string_view test_trim(std::string_view s) noexcept;

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
        if (ctrl == MOCK_UART_DTR_CTRL) *val = mock_dtr_state;
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

class UsbShellTestSuite : public ::testing::Test {
  protected:
      UsbCdcFacade facade;
      DeviceContext global_ctx;
      I2CManager global_i2c{dummy_uart_dev};
      SbsBattery global_battery{&global_i2c, &global_ctx};

      void SetUp() override {
          mock_dtr_state = 0;
          mock_tx_index = 0;
          mock_tx_buffer.fill(0);
          mock_rx_head = 0;
          mock_rx_tail = 0;
          mock_nvs_write_fail = false;
          
          static bool first_run = true;
          static std::vector<MockFcbEntry> backup_fcb_entries;
          static std::array<uint8_t, 4096> backup_flash_area;
          static uint32_t backup_flash_offset = 0;

          if (first_run) {
              mock_fcb_entries.clear();
              mock_flash_offset = 0;
              mock_flash_area.fill(0);
          } else {
              mock_fcb_entries = backup_fcb_entries;
              mock_flash_offset = backup_flash_offset;
              mock_flash_area = backup_flash_area;
          }
          
          enable_snprintf_mock = false;
          mock_snprintf_fail_on_call = -1;
          mock_snprintf_truncate_on_call = -1;
          mock_snprintf_exact_return_on_call = -1;
          mock_snprintf_call_count = 0;
          virtual_uptime = 0;

          mock_flash_read_call_count = 0;
          mock_flash_read_fail_on_call = -1;

          mock_uart_line_ctrl_fail = false;
          mock_uart_line_ctrl_ret = 0;
          mock_uart_callback_fail = false;
          force_device_not_ready = false;
          mock_uart_irq_update_fail = false;
          extern bool run_thread_once;
          run_thread_once = false;

          auto* mut_sys_ctx = const_cast<DeviceContext**>(&diag_shell.sys_ctx);
          *mut_sys_ctx = &global_ctx;
          auto* mut_battery = const_cast<SbsBattery**>(&diag_shell.battery);
          *mut_battery = &global_battery;

          EXPECT_TRUE(facade.init());
          ConfigStore::getInstance().init();
          
          if (first_run) {
              backup_fcb_entries = mock_fcb_entries;
              backup_flash_offset = mock_flash_offset;
              backup_flash_area = mock_flash_area;
              first_run = false;
          }
      }
      void TearDown() override {
          PowerManager::getInstance().notifyAfterWakeup();
      }
};

TEST_F(UsbShellTestSuite,InitSuccess){
    UsbCdcFacade fresh; testing::internal::CaptureStdout();
    EXPECT_TRUE(fresh.init());
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view out(raw_out);
    EXPECT_NE(out.find("[INF] USB CDC Facade Initialized"),std::string_view::npos);
}
TEST_F(UsbShellTestSuite,InitHardwareNotReady){
    force_device_not_ready=true; UsbCdcFacade fresh; testing::internal::CaptureStdout();
    EXPECT_FALSE(fresh.init());
    EXPECT_NE(testing::internal::GetCapturedStdout().find("[ERR] Failed to enable USB"),std::string_view::npos);
    force_device_not_ready=false;
}
TEST_F(UsbShellTestSuite,InitCallbackFails){
    mock_uart_callback_fail=true; UsbCdcFacade fresh;
    EXPECT_FALSE(fresh.init());
    mock_uart_callback_fail=false;
}
TEST_F(UsbShellTestSuite,InitTwice){
    ASSERT_TRUE(facade.init());
    mock_uart_callback_fail=true; EXPECT_TRUE(facade.init());
    mock_uart_callback_fail=false;
}

TEST_F(UsbShellTestSuite,DtrConnectDisconnect){
    mock_dtr_state=0; EXPECT_FALSE(facade.isConnected());
    mock_dtr_state=1; testing::internal::CaptureStdout();
    EXPECT_TRUE(facade.isConnected());
    EXPECT_NE(testing::internal::GetCapturedStdout().find("[INF] USB Terminal Connected (DTR High)"),std::string_view::npos);
    mock_dtr_state=0; testing::internal::CaptureStdout();
    EXPECT_FALSE(facade.isConnected());
    EXPECT_NE(testing::internal::GetCapturedStdout().find("[WRN] USB Terminal Disconnected (DTR Low)"),std::string_view::npos);
}
TEST_F(UsbShellTestSuite,LineCtrlFailureLogsOnce){
    mock_uart_line_ctrl_fail=true; testing::internal::CaptureStdout();
    EXPECT_FALSE(facade.isConnected()); EXPECT_FALSE(facade.isConnected());
    const auto raw_out = testing::internal::GetCapturedStdout();
    std::string_view out(raw_out);
    size_t first=out.find("uart_line_ctrl_get failed");
    ASSERT_NE(first,std::string_view::npos);
    EXPECT_EQ(out.find("uart_line_ctrl_get failed",first+1),std::string_view::npos);
    mock_uart_line_ctrl_fail=false;
}

TEST_F(UsbShellTestSuite,Transmit){
    mock_dtr_state=1; mock_tx_index=0; facade.transmit("AB");
    ASSERT_EQ(mock_tx_index,2u); EXPECT_EQ(mock_tx_buffer[0],'A'); EXPECT_EQ(mock_tx_buffer[1],'B');
    mock_dtr_state=0; mock_tx_index=0; facade.transmit("Data"); EXPECT_EQ(mock_tx_index,0u);
    mock_dtr_state=1; mock_tx_index=0; facade.transmit(""); EXPECT_EQ(mock_tx_index,0u);
}

TEST_F(UsbShellTestSuite,IrqUpdateFails){
    mock_uart_irq_update_fail=true; ASSERT_TRUE(facade.init());
    if(mock_uart_irq_cb) mock_uart_irq_cb(dummy_uart_dev,mock_uart_cb_data);
    mock_uart_irq_update_fail=false;
}
TEST_F(UsbShellTestSuite,IrqNoRxReady){
    UsbCdcFacade local; ASSERT_TRUE(local.init());
    mock_rx_head=mock_rx_tail=0;
    mock_uart_irq_cb(dummy_uart_dev,mock_uart_cb_data);
}
TEST_F(UsbShellTestSuite,RingBufferAndOverflow){
    UsbCdcFacade local; ASSERT_TRUE(local.init());

    inject_mock_uart_data("hello\n");
    std::array<char,MAX_CMD_LEN> cmd;
    ASSERT_TRUE(local.readLine(cmd)); EXPECT_STREQ(cmd.data(),"hello");

    std::array<char, 500> arr_X; arr_X.fill('X');
    inject_mock_uart_data(std::string_view(arr_X.data(), arr_X.size()));
    EXPECT_GT(local.overflow_count.load(),0u);
    EXPECT_GT(local.dropped_bytes.load(),0u);

    inject_mock_uart_data("B");

    local.overflow_logged.store(false);
    testing::internal::CaptureStdout();
    std::array<char, 300> arr_Y; arr_Y.fill('Y');
    inject_mock_uart_data(std::string_view(arr_Y.data(), arr_Y.size()));
    EXPECT_NE(testing::internal::GetCapturedStdout().find("USB RX overflow"),std::string_view::npos);

    local.overflow_logged.store(true);
    local.rx_tail.store(100); local.rx_head.store(99);
    testing::internal::CaptureStdout();
    inject_mock_uart_data("Z");
    EXPECT_EQ(testing::internal::GetCapturedStdout().find("USB RX overflow"),std::string_view::npos);
}

TEST_F(UsbShellTestSuite, BackspaceHandling) {
    UsbCdcFacade local;
    ASSERT_TRUE(local.init());
    mock_rx_head = mock_rx_tail = 0;
    
    inject_mock_uart_data("a\b");
    inject_mock_uart_data("b\n");
    
    std::array<char, MAX_CMD_LEN> cmd;
    ASSERT_TRUE(local.readLine(cmd));
    EXPECT_STREQ(cmd.data(), "b");

    inject_mock_uart_data("\b\n");
    ASSERT_TRUE(local.readLine(cmd));
    EXPECT_STREQ(cmd.data(), "");

    inject_mock_uart_data("x\x7Fy\n");
    ASSERT_TRUE(local.readLine(cmd));
    EXPECT_STREQ(cmd.data(), "y");
}

TEST_F(UsbShellTestSuite, ReadLineVariants) {
    std::array<char, MAX_CMD_LEN> cmd;

    EXPECT_FALSE(facade.readLine(cmd));

    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        mock_rx_head = mock_rx_tail = 0;
        inject_mock_uart_data("partial");
        EXPECT_FALSE(f.readLine(cmd));
        inject_mock_uart_data("\n");
        ASSERT_TRUE(f.readLine(cmd));
        EXPECT_STREQ(cmd.data(), "partial");
    }

    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        mock_rx_head = mock_rx_tail = 0;
        std::array<char, MAX_CMD_LEN - 1> arr_A; arr_A.fill('A');
        inject_mock_uart_data(std::string_view(arr_A.data(), arr_A.size()));
        EXPECT_FALSE(f.readLine(cmd));
    }

    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        std::array<char, 150> arr_B; arr_B.fill('B');
        inject_mock_uart_data(std::string_view(arr_B.data(), arr_B.size()));
        EXPECT_FALSE(f.readLine(cmd));
    }

    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        mock_rx_head = mock_rx_tail = 0;
        inject_mock_uart_data("\r\nstatus\r\n");
        EXPECT_TRUE(f.readLine(cmd)); EXPECT_STREQ(cmd.data(), "");
        EXPECT_TRUE(f.readLine(cmd)); EXPECT_STREQ(cmd.data(), "");
        EXPECT_TRUE(f.readLine(cmd)); EXPECT_STREQ(cmd.data(), "status");
    }

    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        mock_rx_head = mock_rx_tail = 0;
        inject_mock_uart_data("reboot\r");
        ASSERT_TRUE(f.readLine(cmd));
        EXPECT_STREQ(cmd.data(), "reboot");
    }

    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        mock_rx_head = mock_rx_tail = 0;
        inject_mock_uart_data("a\rb\n");
        ASSERT_TRUE(f.readLine(cmd)); EXPECT_STREQ(cmd.data(), "a");
        ASSERT_TRUE(f.readLine(cmd)); EXPECT_STREQ(cmd.data(), "b");
    }

    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        mock_rx_head = mock_rx_tail = 0;
        std::array<char, MAX_CMD_LEN> arr_C; arr_C.fill('C'); arr_C.back() = '\n';
        inject_mock_uart_data(std::string_view(arr_C.data(), arr_C.size()));
        EXPECT_TRUE(f.readLine(cmd));
        EXPECT_EQ(strlen(cmd.data()), MAX_CMD_LEN - 1);
    }

    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        mock_rx_head = mock_rx_tail = 0;
        std::array<char, MAX_CMD_LEN> wcmd;

        std::array<char, 101> a; a.fill('D'); a.back() = '\n';
        std::array<char, 101> b; b.fill('E'); b.back() = '\n';
        std::array<char, 101> c; c.fill('F'); c.back() = '\n';

        std::array<char, 101> a_cmp; a_cmp.fill('D'); a_cmp.back() = '\0';
        std::array<char, 101> b_cmp; b_cmp.fill('E'); b_cmp.back() = '\0';
        std::array<char, 101> c_cmp; c_cmp.fill('F'); c_cmp.back() = '\0';

        inject_mock_uart_data(std::string_view(a.data(), a.size()));
        EXPECT_TRUE(f.readLine(wcmd));
        EXPECT_STREQ(wcmd.data(), a_cmp.data());

        inject_mock_uart_data(std::string_view(b.data(), b.size()));
        EXPECT_TRUE(f.readLine(wcmd));
        EXPECT_STREQ(wcmd.data(), b_cmp.data());

        inject_mock_uart_data(std::string_view(c.data(), c.size()));
        EXPECT_TRUE(f.readLine(wcmd));
        EXPECT_STREQ(wcmd.data(), c_cmp.data());
    }
}

TEST_F(UsbShellTestSuite, ParseIntTokenAndTrimDirectEdgeCases) {
    int val = 0;
    EXPECT_FALSE(test_parseIntToken("", val));
    EXPECT_TRUE(test_parseIntToken("42", val));
    EXPECT_EQ(val, 42);
    EXPECT_FALSE(test_parseIntToken("12x", val));

    EXPECT_EQ(test_trim(""), "");
    EXPECT_EQ(test_trim(" "), "");
    EXPECT_EQ(test_trim("\t"), "");
    EXPECT_EQ(test_trim(" a "), "a");
    EXPECT_EQ(test_trim("\ta\t"), "a");
}

extern bool run_thread_once;
extern void shell_thread(void);

TEST_F(UsbShellTestSuite,ProcessAndShellThread){
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx,&battery);

    force_device_not_ready=true; testing::internal::CaptureStdout();
    shell.process();
    EXPECT_NE(testing::internal::GetCapturedStdout().find("[ERR] USB initialization failed"),std::string_view::npos);
    force_device_not_ready=false;

    mock_dtr_state=1; run_thread_once=false; mock_tx_index=0; mock_tx_buffer.fill(0);
    shell.process();
    inject_mock_uart_data("status\n");
    run_thread_once=false;
    shell.process();
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("sys_state"),std::string_view::npos);
    EXPECT_NE(out.find("med-device:~$ "),std::string_view::npos);

    mock_dtr_state=1; mock_tx_index=0;

    run_thread_once=true;

    shell.process();
    inject_mock_uart_data("\n");
    run_thread_once=false;
    shell.process();

    mock_dtr_state=0; run_thread_once=false; shell.process(); SUCCEED();

    mock_dtr_state=1; run_thread_once=false; EXPECT_NO_FATAL_FAILURE(shell_thread());
    inject_mock_uart_data("status\n"); run_thread_once=false; EXPECT_NO_FATAL_FAILURE(shell_thread());
}

TEST_F(UsbShellTestSuite, DispatchCommands) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1; mock_tx_index = 0; mock_tx_buffer.fill(0);

    shell.dispatchCommand("status");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("sys_state"), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("log dump");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("NVS System Log Dump"), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("reboot");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Rebooting system..."), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("bogus");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Unknown command"), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("STATUS");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Unknown"), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("status ");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("sys_state"), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("status\t");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("sys_state"), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("status x");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Unknown command"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite,SetRateVariants){
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c,&ctx);
    UsbShell shell(&ctx,&battery);
    mock_dtr_state=1; mock_tx_index=0; mock_tx_buffer.fill(0);

    shell.dispatchCommand("set_rate 1001 50");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index).find("Success"),std::string_view::npos);
    uint8_t rate=0; EXPECT_TRUE(ConfigStore::getInstance().getInfusionRate(1,rate)); EXPECT_EQ(rate,50);

    mock_tx_index=0; shell.dispatchCommand("set_rate 1001");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate ABC 50");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 XYZ");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 150");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Rate must be between 1 and 100"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 -1");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Rate must be between 1 and 100"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 50abc");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 +50");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate 9999 50");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("not provisioned"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 85");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("exceeds alarm threshold"),std::string_view::npos);

    mock_tx_index=0; mock_nvs_write_fail=true; shell.dispatchCommand("set_rate 1001 50");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Failed to persist infusion rate"),std::string_view::npos);
    mock_nvs_write_fail=false;

    mock_tx_index=0;
    for (auto it = mock_fcb_entries.begin(); it != mock_fcb_entries.end(); ) {
        FcbRecordHeader header;
        memcpy(&header, &mock_flash_area[it->offset], sizeof(header));
        if (static_cast<uint16_t>(header.key) == static_cast<uint16_t>(ConfigKey::ALARM_THRESHOLD_BASE)) {
            it = mock_fcb_entries.erase(it);
        } else {
            ++it;
        }
    }
    
    ConfigStore::getInstance().init(); shell.dispatchCommand("set_rate 1001 75");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index).find("Success"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate      1001     50");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index).find("Success"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 0");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Rate must be between 1 and 100"),std::string_view::npos);

    mock_tx_index=0; shell.dispatchCommand("set_rate 2147483648 50");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);
    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 2147483648");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);

    ASSERT_TRUE(ConfigStore::getInstance().init());    

    auto &cfg = ConfigStore::getInstance();
    uint8_t slot=0;

    ASSERT_TRUE(cfg.findSlotByDeviceId(1001,slot));
    ASSERT_TRUE(cfg.setAlarmThreshold(slot,80));
    ASSERT_TRUE(cfg.setInfusionRate(slot,10));

    mock_tx_index=0;
    mock_tx_buffer.fill(0);

    shell.dispatchCommand("set_rate 1001 80");

    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index).find("Success"),std::string_view::npos);

    enable_snprintf_mock=true; mock_snprintf_call_count=0; mock_snprintf_fail_on_call=1;
    mock_tx_index=0; shell.dispatchCommand("set_rate 9999 50");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Device ID not provisioned"),std::string_view::npos);
    mock_snprintf_fail_on_call=-1; mock_snprintf_call_count=0; mock_snprintf_truncate_on_call=1;
    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 50");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index).find("Success"),std::string_view::npos);
    enable_snprintf_mock=false;

    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 ");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);
}

TEST_F(UsbShellTestSuite,StatusFormatting){
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c,&ctx);
    UsbShell shell(&ctx,&battery);

    mock_dtr_state=1; mock_fcb_entries.clear(); mock_tx_index=0;
    shell.dispatchCommand("status");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index).find("\"device_id\":0"),std::string_view::npos);

    UsbShell::StatusSnapshot snap{}; StatusBuffer buf{};
    size_t written=UsbShell::formatStatus(snap,buf);
    EXPECT_NE(std::string_view(buf.data(),written).find("\"devices\":[]"),std::string_view::npos);
    EXPECT_TRUE(std::string_view(buf.data(),written).ends_with("]}\r\n"));

    enable_snprintf_mock=true; mock_snprintf_call_count=0; mock_snprintf_truncate_on_call=1;
    mock_dtr_state=1; mock_tx_index=0; testing::internal::CaptureStdout();
    shell.dispatchCommand("status");
    EXPECT_NE(testing::internal::GetCapturedStdout().find("Status JSON truncated at header"),std::string_view::npos);

    mock_snprintf_call_count=0; mock_snprintf_truncate_on_call=3;
    mock_tx_index=0; testing::internal::CaptureStdout(); shell.dispatchCommand("status");
    EXPECT_NE(testing::internal::GetCapturedStdout().find("Status JSON truncated after"),std::string_view::npos);

    snap.slot_count=static_cast<uint8_t>(snap.slots.size());
    for(uint8_t i=0;i<snap.slot_count;++i) snap.slots[i]={static_cast<uint32_t>(1000+i),50,80};
    written=UsbShell::formatStatus(snap,buf);
    EXPECT_LT(written,buf.size());

    mock_snprintf_call_count=0; mock_snprintf_fail_on_call=2; mock_snprintf_truncate_on_call=-1;
    written=UsbShell::formatStatus(UsbShell::StatusSnapshot{},buf); EXPECT_LT(written,buf.size());
    mock_snprintf_call_count=0; mock_snprintf_truncate_on_call=2; mock_snprintf_fail_on_call=-1;
    written=UsbShell::formatStatus(UsbShell::StatusSnapshot{},buf); EXPECT_LT(written,buf.size());
    enable_snprintf_mock=false;

    mock_dtr_state=1; mock_tx_index=0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("status");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("\"battery_soc\":0"),std::string_view::npos);
    virtual_uptime=5000; BmsCache mc{}; mc.valid=true; mc.last_error=CommFault::NONE;
    mc.timestamp_ms=5000; mc.soc.value=85; battery.cache=mc;
    mock_tx_index=0; mock_tx_buffer.fill(0); shell.dispatchCommand("status");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("\"battery_soc\":85"),std::string_view::npos);

    mock_fcb_entries.clear();
    ConfigStore::getInstance().init();
    auto snap2 = shell.collectStatus();
    EXPECT_EQ(snap2.slots[0].rate,0);

    enable_snprintf_mock=true;
    mock_snprintf_call_count=0;
    mock_snprintf_fail_on_call=1;
    EXPECT_EQ(UsbShell::formatStatus(UsbShell::StatusSnapshot{},buf),0u);
    EXPECT_EQ(mock_snprintf_call_count,1);
    enable_snprintf_mock=false;
    mock_snprintf_fail_on_call=-1;

    UsbShell::StatusSnapshot snap3{};
    snap3.slot_count=2;
    snap3.slots[0]={1001,50,80};
    snap3.slots[1]={1002,60,90};
    enable_snprintf_mock=true;
    mock_snprintf_call_count=0;
    mock_snprintf_fail_on_call=2;
    const auto written2 = UsbShell::formatStatus(snap3, buf);
    EXPECT_GT(written2, 0u);
    enable_snprintf_mock=false;
    mock_snprintf_fail_on_call=-1;
}

TEST_F(UsbShellTestSuite, FormatStatusFooterExactlyOneByteRemaining) {
    UsbShell::StatusSnapshot snap{};
    snap.slot_count = 0;
    StatusBuffer buf{};

    enable_snprintf_mock = true;
    mock_snprintf_call_count = 0;
    mock_snprintf_fail_on_call = -1;
    mock_snprintf_truncate_on_call = -1;
    mock_snprintf_exact_return_on_call = 1;
    mock_snprintf_exact_return_value = static_cast<int>(buf.size() - 1);

    const size_t written = UsbShell::formatStatus(snap, buf);
    EXPECT_EQ(written, buf.size() - 1);

    enable_snprintf_mock = false;
    mock_snprintf_exact_return_on_call = -1;
}

TEST_F(UsbShellTestSuite, OverlongCommandWithEol) {
    UsbCdcFacade local;
    ASSERT_TRUE(local.init());
    mock_rx_head = mock_rx_tail = 0;
    std::array<char, MAX_CMD_LEN> cmd;
    std::array<char, 151> arr_X; arr_X.fill('X'); arr_X.back() = '\n';
    inject_mock_uart_data(std::string_view(arr_X.data(), arr_X.size()));
    EXPECT_FALSE(local.readLine(cmd));
}

TEST_F(UsbShellTestSuite, DispatchBlankCommand) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1; mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("  \t  ");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Unknown"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, SetRateWithTabsInArgs) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1; mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate \t1001\t 50");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Success"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchSetRateWithOnlySpacesAfterCommand) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1; mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate ");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Usage"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchNoArgumentHandlersDirectly)
{
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1;

    mock_tx_index = 0;
    mock_tx_buffer.fill(0);
    shell.dispatchCommand("log dump");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index)
                  .find("NVS System Log Dump"),
              std::string_view::npos);

    mock_tx_index = 0;
    mock_tx_buffer.fill(0);
    shell.dispatchCommand("reboot");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index)
                  .find("Rebooting system"),
              std::string_view::npos);
}

TEST_F(UsbShellTestSuite, CollectStatusAllSlotsHaveRates)
{
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    auto &cfg = ConfigStore::getInstance();

    for (uint8_t slot = InfusionDeviceConfig::MinSlot;
         slot <= InfusionDeviceConfig::MaxSlot;
         ++slot)
    {
        ASSERT_TRUE(cfg.setInfusionRate(slot, 42));
    }

    auto snap = shell.collectStatus();
    for (uint8_t i = 0; i < snap.slot_count; ++i)
    {
        EXPECT_EQ(snap.slots[i].rate, 42);
    }
}

TEST_F(UsbShellTestSuite, SetRateEqualAndLessThanThreshold)
{
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1;

    auto &cfg = ConfigStore::getInstance();
    uint8_t slot = 0;

    ASSERT_TRUE(cfg.findSlotByDeviceId(1001, slot));
    ASSERT_TRUE(cfg.setAlarmThreshold(slot, 80));

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate 1001 79");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Success"), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate 1001 80");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Success"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchSetRatePrefixWithoutSpace)
{
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx,&battery);
    mock_dtr_state = 1;
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rateXYZ");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index).find("Unknown"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchSetRateFollowedByTab)
{
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1;
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate\t1001 50");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Unknown"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchExactSetRateCommand)
{
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx,&battery);
    mock_dtr_state = 1;
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Usage"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchSetRatePrefixButInvalidSeparator)
{
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1;
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate:");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Unknown"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchSetRateWhitespaceOnlyArgument)
{
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1;
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate          ");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Usage"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, SetRateWithoutAlarmThreshold)
{
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    auto &cfg = ConfigStore::getInstance();
    mock_dtr_state = 1;

    for (uint16_t slot = InfusionDeviceConfig::MinSlot;
         slot <= InfusionDeviceConfig::MaxSlot;
         ++slot)
    {
        for (auto it = mock_fcb_entries.begin(); it != mock_fcb_entries.end(); ) {
            FcbRecordHeader header;
            memcpy(&header, &mock_flash_area[it->offset], sizeof(header));
            if (static_cast<uint16_t>(header.key) == static_cast<uint16_t>(ConfigKey::ALARM_THRESHOLD_BASE) + slot) {
                it = mock_fcb_entries.erase(it);
            } else {
                ++it;
            }
        }
    }
    cfg.init();

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate 1001 50");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Success"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, SetRateThresholdExhaustiveBranches) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1; mock_tx_index = 0; mock_tx_buffer.fill(0);
    auto &cfg = ConfigStore::getInstance();
    uint8_t slot = 0;
    ASSERT_TRUE(cfg.findSlotByDeviceId(1001, slot));

    for (auto it = mock_fcb_entries.begin(); it != mock_fcb_entries.end(); ) {
        FcbRecordHeader header;
        memcpy(&header, &mock_flash_area[it->offset], sizeof(header));
        if (static_cast<uint16_t>(header.key) == static_cast<uint16_t>(ConfigKey::ALARM_THRESHOLD_BASE) + slot) {
            it = mock_fcb_entries.erase(it);
        } else {
            ++it;
        }
    }

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate 1001 50");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Success"), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    ASSERT_TRUE(cfg.setAlarmThreshold(slot, 80));
    shell.dispatchCommand("set_rate 1001 80");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Success"), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate 1001 81");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("exceeds alarm threshold"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, PowerObserverAndSafeHaltGating) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);

    mock_dtr_state = 1;
    mock_tx_index = 0;

    extern bool run_thread_once;
    run_thread_once = false;
    shell.process();

    PowerManager::getInstance().notifyBeforeSleep();
    inject_mock_uart_data("status\n");
    mock_tx_index = 0;
    run_thread_once = false;
    shell.process();
    EXPECT_EQ(mock_tx_index, 0u) << "Shell should ignore input while sleeping";

    PowerManager::getInstance().notifyAfterWakeup();
    run_thread_once = false;
    shell.process();
    EXPECT_GT(mock_tx_index, 0u) << "Shell should process queued input after waking up";

    PowerManager::getInstance().notifyBeforeSleep();
    PowerManager::getInstance().notifySleepAborted();
    mock_tx_index = 0;
    inject_mock_uart_data("log dump\n");
    mock_tx_index = 0;
    run_thread_once = false;
    shell.process();
    EXPECT_GT(mock_tx_index, 0u) << "Shell should resume if sleep is aborted";

    ctx.current_state = SystemState::SAFE_HALT;
    mock_tx_index = 0;
    inject_mock_uart_data("status\n");
    mock_tx_index = 0;
    run_thread_once = false;
    shell.process();
    EXPECT_EQ(mock_tx_index, 0u) << "Shell should halt processing in SAFE_HALT state";

    ctx.current_state = SystemState::RUNNING;
    PowerManager::getInstance().notifyAfterWakeup();
}

TEST_F(UsbShellTestSuite, TransmitFormattedFallbackOnSnprintfFailure) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1;
    mock_tx_index = 0;
    mock_tx_buffer.fill(0);

    enable_snprintf_mock = true;
    mock_snprintf_fail_on_call = 1;
    shell.dispatchCommand("set_rate 1001 50");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Success: infusion rate updated"), std::string_view::npos);

    enable_snprintf_mock = false;
}

TEST_F(UsbShellTestSuite, BatteryCacheAndStateOfCharge) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);

    virtual_uptime = 10000;

    BmsCache valid_cache{};
    valid_cache.valid = true;
    valid_cache.last_error = CommFault::NONE;
    valid_cache.soc.value = 75;
    valid_cache.timestamp_ms = k_uptime_get_32();
    battery.cache = valid_cache;

    auto soc = battery.getStateOfCharge();
    ASSERT_TRUE(soc.success);
    EXPECT_EQ(soc.value.value, 75);

    valid_cache.timestamp_ms = 0;
    battery.cache = valid_cache;
    soc = battery.getStateOfCharge();
    EXPECT_FALSE(soc.success);
    EXPECT_EQ(soc.error, CommFault::CACHE_INVALID);
}

TEST_F(UsbShellTestSuite, LogDumpWithEntries) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    
    mock_fcb_entries.clear();
    
    auto& cfg = ConfigStore::getInstance();
    
    // Add multiple entries to fully cover the (i % 10) == 0 alternating condition
    ConfigStore::LogEntry log_entry1{12345, 0x01, 100};
    ConfigStore::LogEntry log_entry2{12346, 0x02, 200};
    ASSERT_TRUE(cfg.set(ConfigKey::FULL_CHARGE_LOG, log_entry1));
    ASSERT_TRUE(cfg.set(ConfigKey::FULL_CHARGE_LOG, log_entry2));
    
    mock_dtr_state = 1;
    mock_tx_index = 0;
    mock_tx_buffer.fill(0);
    shell.dispatchCommand("log dump");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Event: 1"), std::string_view::npos);
    EXPECT_NE(out.find("Event: 2"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, LogDumpReadFailure) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    
    mock_fcb_entries.clear();
    
    auto& cfg = ConfigStore::getInstance();
    ConfigStore::LogEntry log_entry{12345, 0x01, 100};
    ASSERT_TRUE(cfg.set(ConfigKey::FULL_CHARGE_LOG, log_entry));
    
    mock_dtr_state = 1;
    mock_tx_index = 0;
    mock_tx_buffer.fill(0);
    
    mock_flash_read_call_count = 0;
    // Call 1: getStoredLogCount reads header (returns 0)
    // Call 2: getLogEntry reads header (returns 0)
    // Call 3: getLogEntry reads payload (returns -1 due to mock instruction)
    mock_flash_read_fail_on_call = 3; 
    
    shell.dispatchCommand("log dump");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Failed to read log entry"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, LogDumpSnprintfFailure) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    
    mock_fcb_entries.clear();
    
    auto& cfg = ConfigStore::getInstance();
    ConfigStore::LogEntry log_entry{12345, 0x01, 100};
    ASSERT_TRUE(cfg.set(ConfigKey::FULL_CHARGE_LOG, log_entry));
    
    mock_dtr_state = 1;
    mock_tx_index = 0;
    mock_tx_buffer.fill(0);
    
    enable_snprintf_mock = true;
    mock_snprintf_call_count = 0;
    mock_snprintf_fail_on_call = 1; 
    
    testing::internal::CaptureStdout();
    shell.dispatchCommand("log dump");
    std::string stdout_out = testing::internal::GetCapturedStdout();
    
    enable_snprintf_mock = false;
    mock_snprintf_fail_on_call = -1;
    
    EXPECT_NE(stdout_out.find("formatting truncated or failed"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, LogDumpSnprintfTruncation) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    
    mock_fcb_entries.clear();
    
    auto& cfg = ConfigStore::getInstance();
    ConfigStore::LogEntry log_entry{12345, 0x01, 100};
    ASSERT_TRUE(cfg.set(ConfigKey::FULL_CHARGE_LOG, log_entry));
    
    mock_dtr_state = 1;
    mock_tx_index = 0;
    mock_tx_buffer.fill(0);
    
    enable_snprintf_mock = true;
    mock_snprintf_call_count = 0;
    mock_snprintf_truncate_on_call = 1; 
    
    testing::internal::CaptureStdout();
    shell.dispatchCommand("log dump");
    std::string stdout_out = testing::internal::GetCapturedStdout();
    
    enable_snprintf_mock = false;
    mock_snprintf_truncate_on_call = -1;
    
    EXPECT_NE(stdout_out.find("formatting truncated or failed"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, SetRateEmptyArgs) {
    DeviceContext ctx;
    I2CManager i2c(dummy_uart_dev);
    SbsBattery battery(&i2c, &ctx);
    UsbShell shell(&ctx, &battery);
    
    mock_dtr_state = 1;
    mock_tx_index = 0;
    mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate  \t  ");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Usage"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, IsConnectedMockUartFallback) {
    // 1. Cover ret == -ENOTSUP (short-circuits ||) and !dtr_ready == true
    mock_uart_line_ctrl_ret = -ENOTSUP;
    facade.dtr_ready = false; 
    testing::internal::CaptureStdout();
    EXPECT_TRUE(facade.isConnected());
    EXPECT_NE(testing::internal::GetCapturedStdout().find("Virtual USB Terminal Connected (Mock UART mode)"), std::string_view::npos);

    // 2. Cover ret == -ENOTSUP and !dtr_ready == false
    EXPECT_TRUE(facade.isConnected());

    // 3. Cover ret == -ENOSYS (evaluates both sides of ||)
    mock_uart_line_ctrl_ret = -ENOSYS;
    facade.dtr_ready = false;
    EXPECT_TRUE(facade.isConnected());
}
