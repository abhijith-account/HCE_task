// test_usb_shell_common.h
// Shared includes, extern declarations and the UsbShellTestSuite fixture for the USB shell gtest files.
// The mock definitions live in test_usb_shell_mocks.cpp (link it into the test binary exactly once).
#pragma once

#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <string_view>
#include <cstdint>
#include <string>
#include <cstdarg>
#include <algorithm>
#include <vector>
#include <cerrno>
#include <thread>
#include <chrono>

#define private public
#define protected public
#include "USB_CDC_Virtual_COM_Shell_Interface.h"
#undef private
#undef protected
#include "Device_State_Machine+Watchdog.h"
#include "Smart_Battery_System.h"
#include "Persistent_Configuration_System.h"
#include "Power_Management_System.h"
#include <zephyr/sys/atomic.h>

#define MOCK_UART_DTR_CTRL 1
#ifndef ENOSYS
#define ENOSYS 38
#endif

struct MockFcbEntry {
    uint32_t offset;
    uint16_t len;
};

extern std::array<char,4096> mock_tx_buffer;
extern size_t mock_tx_index;
extern std::array<char,512> mock_rx_queue;
extern size_t mock_rx_head;
extern size_t mock_rx_tail;
extern uint32_t mock_dtr_state;
extern void (*mock_uart_irq_cb)(const device*, void*);
extern void* mock_uart_cb_data;
extern bool mock_uart_line_ctrl_fail;
extern int mock_uart_line_ctrl_ret;
extern const device* dummy_uart_dev;
extern bool mock_uart_callback_fail;
extern bool force_device_not_ready;
extern bool mock_nvs_write_fail;
extern bool mock_uart_irq_update_fail;
extern bool enable_snprintf_mock;
extern int mock_snprintf_call_count;
extern int mock_snprintf_fail_on_call;
extern int mock_snprintf_truncate_on_call;
extern int mock_snprintf_exact_return_on_call;
extern int mock_snprintf_exact_return_value;
extern int mock_flash_read_call_count;
extern int mock_flash_read_fail_on_call;
extern std::array<uint8_t, 4096> mock_flash_area;
extern uint32_t mock_flash_offset;
extern std::vector<MockFcbEntry> mock_fcb_entries;
extern size_t mock_fcb_iterator;

extern UsbShell diag_shell;
extern bool g_usb_connected;      // physical USB state, owned by the shell TU
extern atomic_t g_shell_alive;    // shell watchdog heartbeat, owned by the shell TU
extern DeviceContext sys_context;
extern uint32_t virtual_uptime;
extern bool test_parseIntToken(std::string_view token, int& out_value) noexcept;
extern std::string_view test_trim(std::string_view s) noexcept;
extern bool run_thread_once;
extern void shell_thread(void);

void inject_mock_uart_data(std::string_view data);

class UsbShellTestSuite : public ::testing::Test {
  protected:
      UsbCdcFacade facade;
      DeviceContext global_ctx;
      I2CManager global_i2c{dummy_uart_dev};
      SbsBattery global_battery{&global_i2c, &global_ctx};

      void SetUp() override {
          mock_dtr_state = 0;
          g_usb_connected = true;   // USB cable attached unless a test says otherwise
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
          g_usb_connected = false;
          PowerManager::getInstance().notifyAfterWakeup();
      }
};
