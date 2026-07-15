#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <string_view>
#include <cstdint>
#include <string>
#include <cstdarg>
// White-Box Access
#define private public
#define protected public
#include "USB_CDC_Virtual_COM_Shell_Interface.h"
#undef private
#undef protected
#include "Device_State_Machine+Watchdog.h"
#include "Smart_Battery_System.h"
#include "Persistent_Configuration_System.h"
#define MOCK_UART_DTR_CTRL 1
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
// Consumed by testable_snprintf() in USB_CDC_Virtual_COM_Shell_Interface.cpp
// as of the fix below -- previously declared here but never read by the
// production/test-harness code, so tests relying on it silently no-op'd.
int mock_snprintf_exact_return_on_call = -1;
int mock_snprintf_exact_return_value = 0;
extern uint32_t virtual_uptime;
struct MockNvsEntry {
    uint16_t id;
    std::array<uint8_t, 64> data;
    size_t len;
    bool active;
};
static std::array<MockNvsEntry, 32> mock_nvs_map;

// White-box shims defined in USB_CDC_Virtual_COM_Shell_Interface.cpp under
// IS_TEST_ENVIRONMENT. They give external-linkage access to the
// anonymous-namespace parseIntToken()/trim() helpers so their branches
// that are unreachable through the public dispatch API (see cmdSetRate)
// can be exercised directly.
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
    int nvs_mount(struct nvs_fs *fs){ return 0; }
    ssize_t nvs_read(struct nvs_fs *fs,uint16_t id,void *data,size_t len){
        for (const auto& entry : mock_nvs_map) {
            if (entry.active && entry.id == id) {
                size_t copy_len = std::min(len, entry.len);
                std::memcpy(data, entry.data.data(), copy_len);
                return static_cast<ssize_t>(copy_len);
            }
        }
        return -2;
    }
    ssize_t nvs_write(struct nvs_fs *fs,uint16_t id,const void *data,size_t len){
        if (mock_nvs_write_fail) return -1;
        const uint8_t* bytes = static_cast<const uint8_t*>(data);
        MockNvsEntry* empty_slot = nullptr;
        for (auto& entry : mock_nvs_map) {
            if (entry.active && entry.id == id) {
                size_t copy_len = std::min(len, entry.data.size());
                std::memcpy(entry.data.data(), bytes, copy_len);
                entry.len = copy_len;
                return static_cast<ssize_t>(copy_len);
            }
            if (!entry.active && empty_slot == nullptr) empty_slot = &entry;
        }
        if (empty_slot) {
            empty_slot->active = true;
            empty_slot->id = id;
            size_t copy_len = std::min(len, empty_slot->data.size());
            std::memcpy(empty_slot->data.data(), bytes, copy_len);
            empty_slot->len = copy_len;
            return static_cast<ssize_t>(copy_len);
        }
        return -1;
    }
    bool device_is_ready(const struct device *dev) { return !force_device_not_ready; }
    int uart_line_ctrl_get(const struct device *dev,uint32_t ctrl,uint32_t *val){
        if (mock_uart_line_ctrl_fail) return -1;
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
class UsbShellTestSuite:public::testing::Test{
  protected:
      UsbCdcFacade facade;
      void SetUp() override{
          mock_dtr_state=0; mock_tx_index=0; mock_tx_buffer.fill(0);
          mock_rx_head=0; mock_rx_tail=0;
          mock_nvs_write_fail = false;
          for (auto& entry : mock_nvs_map) entry.active = false;
          enable_snprintf_mock = false;
          mock_snprintf_fail_on_call = -1;
          mock_snprintf_truncate_on_call = -1;
          mock_snprintf_exact_return_on_call = -1;
          mock_snprintf_call_count = 0;
          virtual_uptime = 0;
          EXPECT_TRUE(facade.init());
          ConfigStore::getInstance().init();
      }
};
// ---- Init ----
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
// ---- Connection ----
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
// ---- Transmit ----
TEST_F(UsbShellTestSuite,Transmit){
    mock_dtr_state=1; mock_tx_index=0; facade.transmit("AB");
    ASSERT_EQ(mock_tx_index,2u); EXPECT_EQ(mock_tx_buffer[0],'A'); EXPECT_EQ(mock_tx_buffer[1],'B');
    mock_dtr_state=0; mock_tx_index=0; facade.transmit("Data"); EXPECT_EQ(mock_tx_index,0u);
    mock_dtr_state=1; mock_tx_index=0; facade.transmit(""); EXPECT_EQ(mock_tx_index,0u);
}
// ---- IRQ & Ring buffer ----
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
    // normal reception
    inject_mock_uart_data("hello\n");
    std::array<char,MAX_CMD_LEN> cmd;
    ASSERT_TRUE(local.readLine(cmd)); EXPECT_STREQ(cmd.data(),"hello");
    // overflow
    std::array<char, 500> arr_X; arr_X.fill('X');
    inject_mock_uart_data(std::string_view(arr_X.data(), arr_X.size()));
    EXPECT_GT(local.overflow_count.load(),0u);
    EXPECT_GT(local.dropped_bytes.load(),0u);
    // reset on normal byte
    inject_mock_uart_data("B");
    // overflow warning repeat after flag reset
    local.overflow_logged.store(false);
    testing::internal::CaptureStdout();
    std::array<char, 300> arr_Y; arr_Y.fill('Y');
    inject_mock_uart_data(std::string_view(arr_Y.data(), arr_Y.size()));
    EXPECT_NE(testing::internal::GetCapturedStdout().find("USB RX overflow"),std::string_view::npos);
    // suppressed when latch already set
    local.overflow_logged.store(true);
    local.rx_tail.store(100); local.rx_head.store(99);
    testing::internal::CaptureStdout();
    inject_mock_uart_data("Z");
    EXPECT_EQ(testing::internal::GetCapturedStdout().find("USB RX overflow"),std::string_view::npos);
}
// ---- ReadLine ----
TEST_F(UsbShellTestSuite, ReadLineVariants) {
    std::array<char, MAX_CMD_LEN> cmd;

    // --- empty buffer ---
    EXPECT_FALSE(facade.readLine(cmd));

    // --- incomplete then complete ---
    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        mock_rx_head = mock_rx_tail = 0;
        inject_mock_uart_data("partial");
        EXPECT_FALSE(f.readLine(cmd));
        inject_mock_uart_data("\n");
        ASSERT_TRUE(f.readLine(cmd));
        EXPECT_STREQ(cmd.data(), "partial");
    }

    // --- max length without EOL ---
    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        mock_rx_head = mock_rx_tail = 0;
        std::array<char, MAX_CMD_LEN - 1> arr_A; arr_A.fill('A');
        inject_mock_uart_data(std::string_view(arr_A.data(), arr_A.size()));
        EXPECT_FALSE(f.readLine(cmd));
    }

    // --- overlong ---
    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        std::array<char, 150> arr_B; arr_B.fill('B');
        inject_mock_uart_data(std::string_view(arr_B.data(), arr_B.size()));
        EXPECT_FALSE(f.readLine(cmd));
    }

    // --- CR/LF handling ---
    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        mock_rx_head = mock_rx_tail = 0;
        inject_mock_uart_data("\r\nstatus\r\n");
        EXPECT_TRUE(f.readLine(cmd)); EXPECT_STREQ(cmd.data(), "");
        EXPECT_TRUE(f.readLine(cmd)); EXPECT_STREQ(cmd.data(), "");
        EXPECT_TRUE(f.readLine(cmd)); EXPECT_STREQ(cmd.data(), "status");
    }

    // --- CR only ---
    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        mock_rx_head = mock_rx_tail = 0;
        inject_mock_uart_data("reboot\r");
        ASSERT_TRUE(f.readLine(cmd));
        EXPECT_STREQ(cmd.data(), "reboot");
    }

    // --- mixed terminators ---
    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        mock_rx_head = mock_rx_tail = 0;
        inject_mock_uart_data("a\rb\n");
        ASSERT_TRUE(f.readLine(cmd)); EXPECT_STREQ(cmd.data(), "a");
        ASSERT_TRUE(f.readLine(cmd)); EXPECT_STREQ(cmd.data(), "b");
    }

    // --- exact max length ---
    {
        UsbCdcFacade f; ASSERT_TRUE(f.init());
        mock_rx_head = mock_rx_tail = 0;
        std::array<char, MAX_CMD_LEN> arr_C; arr_C.fill('C'); arr_C.back() = '\n';
        inject_mock_uart_data(std::string_view(arr_C.data(), arr_C.size()));
        EXPECT_TRUE(f.readLine(cmd));
        EXPECT_EQ(strlen(cmd.data()), MAX_CMD_LEN - 1);
    }

    // --- ring-buffer wrap-around (100-char batches) ---
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

// Covers: parseIntToken's `if (token.empty()) return false;` branch, plus
// trim()'s empty/all-whitespace loop-entry cases. Neither is reachable
// through cmdSetRate (it always trims args upstream, so parseIntToken
// never actually sees an empty token) -- exercised directly via the
// IS_TEST_ENVIRONMENT-only shims.
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

// ---- Process / Thread ----
bool run_thread_once=false;
extern void shell_thread(void);
TEST_F(UsbShellTestSuite,ProcessAndShellThread){
    DeviceContext ctx; UARTManager uart(dummy_uart_dev); SbsBattery battery(&uart,&ctx);
    UsbShell shell(&ctx,&battery);
    // init fails
    force_device_not_ready=true; testing::internal::CaptureStdout();
    shell.process();
    EXPECT_NE(testing::internal::GetCapturedStdout().find("[ERR] USB initialization failed"),std::string_view::npos);
    force_device_not_ready=false;
    // connected command
    mock_dtr_state=1; run_thread_once=false; mock_tx_index=0; mock_tx_buffer.fill(0);
    shell.process(); inject_mock_uart_data("status\n"); run_thread_once=true; shell.process();
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("sys_state"),std::string_view::npos);
    EXPECT_NE(out.find("med-device:~$ "),std::string_view::npos);
    // empty line
    mock_dtr_state=1; mock_tx_index=0; run_thread_once=false;
    shell.process(); inject_mock_uart_data("\n"); shell.process();
    EXPECT_EQ(mock_tx_index,0u);
    // disconnected
    mock_dtr_state=0; run_thread_once=true; shell.process(); SUCCEED();
    // shell thread
    mock_dtr_state=1; EXPECT_NO_FATAL_FAILURE(shell_thread());
    inject_mock_uart_data("status\n"); EXPECT_NO_FATAL_FAILURE(shell_thread());
}
// ---- Dispatch / Commands ----
TEST_F(UsbShellTestSuite, DispatchCommands) {
    DeviceContext ctx; UARTManager uart(dummy_uart_dev); SbsBattery battery(&uart, &ctx);
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

    // trailing space/tab are trimmed -> should be valid status
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("status ");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("sys_state"), std::string_view::npos);

    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("status\t");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("sys_state"), std::string_view::npos);

    // extra argument -> unknown
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("status x");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Unknown command"), std::string_view::npos);
}
// ---- set_rate ----
TEST_F(UsbShellTestSuite,SetRateVariants){
    DeviceContext ctx; UARTManager uart(dummy_uart_dev); SbsBattery battery(&uart,&ctx);
    UsbShell shell(&ctx,&battery);
    mock_dtr_state=1; mock_tx_index=0; mock_tx_buffer.fill(0);
    // success
    shell.dispatchCommand("set_rate 1001 50");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index).find("Success"),std::string_view::npos);
    uint8_t rate=0; EXPECT_TRUE(ConfigStore::getInstance().getInfusionRate(1,rate)); EXPECT_EQ(rate,50);
    // missing args
    mock_tx_index=0; shell.dispatchCommand("set_rate 1001");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);
    // invalid device id
    mock_tx_index=0; shell.dispatchCommand("set_rate ABC 50");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);
    // invalid rate
    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 XYZ");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);
    // rate out of range
    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 150");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Rate must be between 1 and 100"),std::string_view::npos);
    // negative rate
    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 -1");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Rate must be between 1 and 100"),std::string_view::npos);
    // partial integer
    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 50abc");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);
    // plus sign
    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 +50");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);
    // not provisioned
    mock_tx_index=0; shell.dispatchCommand("set_rate 9999 50");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("not provisioned"),std::string_view::npos);
    // exceeds threshold
    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 85");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("exceeds alarm threshold"),std::string_view::npos);
    // persist fail
    mock_tx_index=0; mock_nvs_write_fail=true; shell.dispatchCommand("set_rate 1001 50");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Failed to persist infusion rate"),std::string_view::npos);
    mock_nvs_write_fail=false;
    // without threshold (haveThreshold false)
    mock_tx_index=0; 
    for(auto& entry : mock_nvs_map) {
        if(entry.id == static_cast<uint16_t>(ConfigKey::ALARM_THRESHOLD_BASE)) entry.active = false;
    }
    ConfigStore::getInstance().init(); shell.dispatchCommand("set_rate 1001 75");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index).find("Success"),std::string_view::npos);
    // multiple spaces
    mock_tx_index=0; shell.dispatchCommand("set_rate      1001     50");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index).find("Success"),std::string_view::npos);
    // zero rate
    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 0");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Rate must be between 1 and 100"),std::string_view::npos);
    // overflow device id and rate
    mock_tx_index=0; shell.dispatchCommand("set_rate 2147483648 50");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);
    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 2147483648");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);
    // threshold equal (covers haveThreshold=true && rate<=threshold)
    auto &cfg = ConfigStore::getInstance();
    uint8_t slot=0;

    ASSERT_TRUE(cfg.findSlotByDeviceId(1001,slot));
    ASSERT_TRUE(cfg.setAlarmThreshold(slot,80));
    ASSERT_TRUE(cfg.setInfusionRate(slot,10));

    mock_tx_index=0;
    mock_tx_buffer.fill(0);

    shell.dispatchCommand("set_rate 1001 80");

    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index).find("Success"),std::string_view::npos);
    // snprintf fallback for not provisioned and success
    enable_snprintf_mock=true; mock_snprintf_call_count=0; mock_snprintf_fail_on_call=1;
    mock_tx_index=0; shell.dispatchCommand("set_rate 9999 50");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Device ID not provisioned"),std::string_view::npos);
    mock_snprintf_fail_on_call=-1; mock_snprintf_call_count=0; mock_snprintf_truncate_on_call=1;
    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 50");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index).find("Success"),std::string_view::npos);
    enable_snprintf_mock=false;
    // empty rate string due to trailing space
    mock_tx_index=0; shell.dispatchCommand("set_rate 1001 ");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("Usage"),std::string_view::npos);
}
// ---- Status formatting ----
TEST_F(UsbShellTestSuite,StatusFormatting){
    DeviceContext ctx; UARTManager uart(dummy_uart_dev); SbsBattery battery(&uart,&ctx);
    UsbShell shell(&ctx,&battery);
    // missing config
    mock_dtr_state=1; for(auto& entry : mock_nvs_map) entry.active = false; mock_tx_index=0;
    shell.dispatchCommand("status");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index).find("\"device_id\":0"),std::string_view::npos);
    // empty snapshot
    UsbShell::StatusSnapshot snap{}; StatusBuffer buf{};
    size_t written=UsbShell::formatStatus(snap,buf);
    EXPECT_NE(std::string_view(buf.data(),written).find("\"devices\":[]"),std::string_view::npos);
    EXPECT_TRUE(std::string_view(buf.data(),written).ends_with("]}\r\n"));
    // header truncation
    enable_snprintf_mock=true; mock_snprintf_call_count=0; mock_snprintf_truncate_on_call=1;
    mock_dtr_state=1; mock_tx_index=0; testing::internal::CaptureStdout();
    shell.dispatchCommand("status");
    EXPECT_NE(testing::internal::GetCapturedStdout().find("Status JSON truncated at header"),std::string_view::npos);
    // device entry truncation
    mock_snprintf_call_count=0; mock_snprintf_truncate_on_call=3;
    mock_tx_index=0; testing::internal::CaptureStdout(); shell.dispatchCommand("status");
    EXPECT_NE(testing::internal::GetCapturedStdout().find("Status JSON truncated after"),std::string_view::npos);
    // many slots truncation
    snap.slot_count=static_cast<uint8_t>(snap.slots.size());
    for(uint8_t i=0;i<snap.slot_count;++i) snap.slots[i]={static_cast<uint32_t>(1000+i),50,80};
    written=UsbShell::formatStatus(snap,buf);
    EXPECT_LT(written,buf.size());
    // closing failures
    mock_snprintf_call_count=0; mock_snprintf_fail_on_call=2; mock_snprintf_truncate_on_call=-1;
    written=UsbShell::formatStatus(UsbShell::StatusSnapshot{},buf); EXPECT_LT(written,buf.size());
    mock_snprintf_call_count=0; mock_snprintf_truncate_on_call=2; mock_snprintf_fail_on_call=-1;
    written=UsbShell::formatStatus(UsbShell::StatusSnapshot{},buf); EXPECT_LT(written,buf.size());
    enable_snprintf_mock=false;
    // SoC failure and success
    mock_dtr_state=1; mock_tx_index=0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("status");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("\"battery_soc\":0"),std::string_view::npos);
    virtual_uptime=5000; BmsCache mc{}; mc.valid=true; mc.last_error=CommFault::NONE;
    mc.timestamp_ms=5000; mc.soc.value=85; battery.cache=mc;
    mock_tx_index=0; mock_tx_buffer.fill(0); shell.dispatchCommand("status");
    EXPECT_NE(std::string(mock_tx_buffer.data(),mock_tx_index).find("\"battery_soc\":85"),std::string_view::npos);
    // collectStatus without infusion rate
    for(auto& entry : mock_nvs_map) entry.active = false;
    ConfigStore::getInstance().init();

    auto snap2 = shell.collectStatus();

    EXPECT_EQ(snap2.slots[0].rate,0);

    // header snprintf failure
    enable_snprintf_mock=true;
    mock_snprintf_call_count=0;
    mock_snprintf_fail_on_call=1;

    EXPECT_EQ(UsbShell::formatStatus(UsbShell::StatusSnapshot{},buf),0u);
    EXPECT_EQ(mock_snprintf_call_count,1);

    enable_snprintf_mock=false;
    mock_snprintf_fail_on_call=-1;

    // device-entry snprintf failure
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

// Covers the false side of formatStatus's footer-write branch:
//   if (written > 0) {
//     const std::size_t remaining = out_buf.size() - offset;
//     if (remaining > 1) { offset += ...; }   <-- this
//   }
// i.e. exactly 1 byte of room is left when the closing "]}\r\n" write
// happens, so its return value must be discarded rather than folded
// into offset. Forced by making the *header* write consume all but one
// byte of the buffer (via mock_snprintf_exact_return_on_call, now wired
// up in testable_snprintf), which also causes the device-entry loop to
// break immediately on the JsonSafetyMargin check, leaving the footer as
// the very next snprintf call.
TEST_F(UsbShellTestSuite, FormatStatusFooterExactlyOneByteRemaining) {
    UsbShell::StatusSnapshot snap{};
    snap.slot_count = 0;
    StatusBuffer buf{};

    enable_snprintf_mock = true;
    mock_snprintf_call_count = 0;
    mock_snprintf_fail_on_call = -1;
    mock_snprintf_truncate_on_call = -1;
    mock_snprintf_exact_return_on_call = 1;  // the header call
    mock_snprintf_exact_return_value = static_cast<int>(buf.size() - 1);

    const size_t written = UsbShell::formatStatus(snap, buf);

    // offset must stay pinned at buf.size()-1: the footer's own
    // (unbounded) intended length was discarded because only 1 byte of
    // room remained.
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
    DeviceContext ctx; UARTManager uart(dummy_uart_dev); SbsBattery battery(&uart, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1; mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("  \t  ");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Unknown"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, SetRateWithTabsInArgs) {
    DeviceContext ctx; UARTManager uart(dummy_uart_dev); SbsBattery battery(&uart, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1; mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate \t1001\t 50");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Success"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchSetRateWithOnlySpacesAfterCommand) {
    DeviceContext ctx; UARTManager uart(dummy_uart_dev); SbsBattery battery(&uart, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1; mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate ");
    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Usage"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchNoArgumentHandlersDirectly)
{
    DeviceContext ctx;
    UARTManager uart(dummy_uart_dev);
    SbsBattery battery(&uart, &ctx);
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
    UARTManager uart(dummy_uart_dev);
    SbsBattery battery(&uart, &ctx);
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
    UARTManager uart(dummy_uart_dev);
    SbsBattery battery(&uart, &ctx);
    UsbShell shell(&ctx, &battery);

    mock_dtr_state = 1;

    auto &cfg = ConfigStore::getInstance();
    uint8_t slot = 0;

    ASSERT_TRUE(cfg.findSlotByDeviceId(1001, slot));
    ASSERT_TRUE(cfg.setAlarmThreshold(slot, 80));

    mock_tx_index = 0;
    mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate 1001 79");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index)
                  .find("Success"),
              std::string_view::npos);

    mock_tx_index = 0;
    mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate 1001 80");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index)
                  .find("Success"),
              std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchSetRatePrefixWithoutSpace)
{
    DeviceContext ctx;
    UARTManager uart(dummy_uart_dev);
    SbsBattery battery(&uart,&ctx);
    UsbShell shell(&ctx,&battery);

    mock_dtr_state = 1;
    mock_tx_index = 0;
    mock_tx_buffer.fill(0);

    shell.dispatchCommand("set_rateXYZ");

    EXPECT_NE(std::string_view(mock_tx_buffer.data(),mock_tx_index)
                  .find("Unknown"),
              std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchSetRateFollowedByTab)
{
    DeviceContext ctx;
    UARTManager uart(dummy_uart_dev);
    SbsBattery battery(&uart, &ctx);
    UsbShell shell(&ctx, &battery);

    mock_dtr_state = 1;
    mock_tx_index = 0;
    mock_tx_buffer.fill(0);

    shell.dispatchCommand("set_rate\t1001 50");

    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index)
                  .find("Unknown"),
              std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchExactSetRateCommand)
{
    DeviceContext ctx;
    UARTManager uart(dummy_uart_dev);
    SbsBattery battery(&uart,&ctx);
    UsbShell shell(&ctx,&battery);

    mock_dtr_state = 1;
    mock_tx_index = 0;
    mock_tx_buffer.fill(0);

    shell.dispatchCommand("set_rate");

    std::string_view out(mock_tx_buffer.data(), mock_tx_index);
    EXPECT_NE(out.find("Usage"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchSetRatePrefixButInvalidSeparator)
{
    DeviceContext ctx;
    UARTManager uart(dummy_uart_dev);
    SbsBattery battery(&uart, &ctx);
    UsbShell shell(&ctx, &battery);

    mock_dtr_state = 1;
    mock_tx_index = 0;
    mock_tx_buffer.fill(0);

    shell.dispatchCommand("set_rate:");

    std::string_view out(mock_tx_buffer.data(), mock_tx_index);

    EXPECT_NE(out.find("Unknown"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, DispatchSetRateWhitespaceOnlyArgument)
{
    DeviceContext ctx;
    UARTManager uart(dummy_uart_dev);
    SbsBattery battery(&uart, &ctx);
    UsbShell shell(&ctx, &battery);

    mock_dtr_state = 1;

    mock_tx_index = 0;
    mock_tx_buffer.fill(0);

    shell.dispatchCommand("set_rate          ");

    std::string_view out(mock_tx_buffer.data(), mock_tx_index);

    EXPECT_NE(out.find("Usage"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, SetRateWithoutAlarmThreshold)
{
    DeviceContext ctx;
    UARTManager uart(dummy_uart_dev);
    SbsBattery battery(&uart, &ctx);
    UsbShell shell(&ctx, &battery);

    auto &cfg = ConfigStore::getInstance();

    mock_dtr_state = 1;

    for (uint16_t slot = InfusionDeviceConfig::MinSlot;
         slot <= InfusionDeviceConfig::MaxSlot;
         ++slot)
    {
        for (uint16_t slot = InfusionDeviceConfig::MinSlot; slot <= InfusionDeviceConfig::MaxSlot; ++slot) {
            for(auto& entry : mock_nvs_map) {
                if(entry.id == static_cast<uint16_t>(static_cast<uint16_t>(ConfigKey::ALARM_THRESHOLD_BASE) + slot)) {
                    entry.active = false;
                }
            }
        }
    }

    cfg.init();

    mock_tx_index = 0;
    mock_tx_buffer.fill(0);

    shell.dispatchCommand("set_rate 1001 50");

    std::string_view out(mock_tx_buffer.data(), mock_tx_index);

    EXPECT_NE(out.find("Success"), std::string_view::npos);
}

TEST_F(UsbShellTestSuite, SetRateThresholdExhaustiveBranches) {
    DeviceContext ctx; UARTManager uart(dummy_uart_dev); SbsBattery battery(&uart, &ctx);
    UsbShell shell(&ctx, &battery);
    mock_dtr_state = 1; mock_tx_index = 0; mock_tx_buffer.fill(0);
    auto &cfg = ConfigStore::getInstance();
    uint8_t slot = 0;
    ASSERT_TRUE(cfg.findSlotByDeviceId(1001, slot));
    
    // Path 1: haveThreshold = false
    // Clear it AFTER initialization to ensure default seeding doesn't restore it
    for(auto& entry : mock_nvs_map) {
        if(entry.id == static_cast<uint16_t>(static_cast<uint16_t>(ConfigKey::ALARM_THRESHOLD_BASE) + slot)) {
            entry.active = false;
        }
    }
    
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate 1001 50");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Success"), std::string_view::npos);

    // Path 2: haveThreshold = true, rate == threshold
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    ASSERT_TRUE(cfg.setAlarmThreshold(slot, 80));
    shell.dispatchCommand("set_rate 1001 80");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("Success"), std::string_view::npos);

    // Path 3: haveThreshold = true, rate > threshold
    mock_tx_index = 0; mock_tx_buffer.fill(0);
    shell.dispatchCommand("set_rate 1001 81");
    EXPECT_NE(std::string_view(mock_tx_buffer.data(), mock_tx_index).find("exceeds alarm threshold"), std::string_view::npos);
}
