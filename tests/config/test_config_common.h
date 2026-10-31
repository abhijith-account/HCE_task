// test_config_common.h
// Shared includes, extern declarations and the ConfigStoreTestSuite fixture for the ConfigStore gtest files.
// The mock definitions live in test_config_mocks.cpp (link it into the test binary exactly once).
#pragma once

#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <cstdint>
#include <cerrno>
#include <vector>
#include <map>
#include <string>
#include <string_view>
#include <cstdio>

#define private public
#define protected public
#include "Persistent_Configuration_System.h"
#include "Device_State_Machine+Watchdog.h"
#undef private
#undef protected

extern DeviceContext sys_context;

extern int mock_flash_area_open_err;
extern int mock_flash_area_get_sectors_err;
extern int mock_fcb_init_err_first;
extern int mock_fcb_init_err_second;
extern int fcb_init_call_count;
extern int mock_flash_area_erase_err;
extern bool mock_flash_read_fail;
extern bool mock_flash_write_fail;
extern bool mock_fcb_getnext_fail;
extern bool mock_nvs_corrupt_data;
extern bool mock_trip_fault_on_getnext;   // flips sys_context to FAULT mid-traversal

extern std::map<off_t, uint8_t> mock_flash_memory;
extern std::vector<fcb_entry> mock_fcb_entries;
extern size_t mock_fcb_read_idx;
extern off_t current_append_offset;

// --- Test Suite ---
class ConfigStoreTestSuite : public ::testing::Test {
protected:
    void SetUp() override {
        mock_flash_memory.clear();
        mock_fcb_entries.clear();
        mock_fcb_read_idx = 0;
        current_append_offset = 0;

        mock_flash_area_open_err = 0;
        mock_flash_area_get_sectors_err = 0;
        mock_fcb_init_err_first = 0;
        mock_fcb_init_err_second = 0;
        fcb_init_call_count = 0;
        mock_flash_area_erase_err = 0;
        
        mock_flash_read_fail = false;
        mock_flash_write_fail = false;
        mock_fcb_getnext_fail = false;
        mock_nvs_corrupt_data = false;
        mock_trip_fault_on_getnext = false;

        sys_context.current_state = SystemState::INIT;
        ConfigStore::getInstance().initialized = false;
    }

    void TearDown() override {
        mock_trip_fault_on_getnext = false;
        sys_context.current_state = SystemState::INIT;
    }
};
