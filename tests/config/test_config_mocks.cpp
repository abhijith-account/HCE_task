// test_config_mocks.cpp
// Definitions of the mocked Zephyr flash-area / FCB driver calls (strong overrides of the weak ones in mock_globals.cpp)
// and the shared mock state used by the ConfigStore gtest files. Link this file into the test binary exactly once.

#include "test_config_common.h"

// --- FCB / Flash Mock Environment ---
int mock_flash_area_open_err = 0;
int mock_flash_area_get_sectors_err = 0;
int mock_fcb_init_err_first = 0;
int mock_fcb_init_err_second = 0;
int fcb_init_call_count = 0;
int mock_flash_area_erase_err = 0;
bool mock_flash_read_fail = false;
bool mock_flash_write_fail = false;
bool mock_fcb_getnext_fail = false;
bool mock_nvs_corrupt_data = false;
bool mock_trip_fault_on_getnext = false;   // flips sys_context to FAULT mid-traversal

std::map<off_t, uint8_t> mock_flash_memory;
std::vector<fcb_entry> mock_fcb_entries;
size_t mock_fcb_read_idx = 0;
off_t current_append_offset = 0;

extern "C" {
    int flash_area_open(uint8_t id, const struct flash_area **fa) {
        static struct flash_area dummy = {0, 4096};
        *fa = &dummy;
        return mock_flash_area_open_err;
    }

    void flash_area_close(const struct flash_area *fa) {}

    int flash_area_get_sectors(int fa_id, uint32_t *count, struct flash_sector *sectors) {
        *count = 2;
        return mock_flash_area_get_sectors_err;
    }

    int fcb_init(int f_area_id, struct fcb *fcb) {
        fcb_init_call_count++;
        if (fcb_init_call_count == 1 && mock_fcb_init_err_first != 0) return mock_fcb_init_err_first;
        if (fcb_init_call_count == 2 && mock_fcb_init_err_second != 0) return mock_fcb_init_err_second;
        return 0;
    }

    int flash_area_erase(const struct flash_area *fa, off_t off, size_t len) {
        return mock_flash_area_erase_err;
    }

    int fcb_append(struct fcb *fcb, uint16_t len, struct fcb_entry *loc) {
        if (mock_flash_write_fail) return -1;
        loc->fe_elem_off = current_append_offset;
        current_append_offset += len;
        mock_fcb_entries.push_back(*loc);
        return 0;
    }

    int fcb_append_finish(struct fcb *fcb, struct fcb_entry *append_loc) {
        return 0;
    }

    int flash_area_write(const struct flash_area *fa, off_t off, const void *src, size_t len) {
        if (mock_flash_write_fail) return -1;
        const uint8_t* ptr = static_cast<const uint8_t*>(src);
        for (size_t i = 0; i < len; ++i) {
            mock_flash_memory[off + i] = ptr[i];
        }
        return 0;
    }

    int flash_area_read(const struct flash_area *fa, off_t off, void *dst, size_t len) {
        if (mock_flash_read_fail) return -1;
        uint8_t* ptr = static_cast<uint8_t*>(dst);
        for (size_t i = 0; i < len; ++i) {
            ptr[i] = mock_flash_memory[off + i];
        }
        
        // Corrupt only the payload (uint16_t is 2 bytes), leaving headers intact
        if (mock_nvs_corrupt_data && len == sizeof(uint16_t)) {
            ptr[0] ^= 0xFF;
        }
        return 0;
    }

    int fcb_getnext(struct fcb *fcb, struct fcb_entry *loc) {
        if (mock_fcb_getnext_fail) return -1;
        if (loc->fe_sector == nullptr) {
            mock_fcb_read_idx = 0;
        }
        if (mock_fcb_read_idx < mock_fcb_entries.size()) {
            *loc = mock_fcb_entries[mock_fcb_read_idx++];
            loc->fe_sector = (void*)0x1234; // Prevent infinite re-init
            if (mock_trip_fault_on_getnext) {
                sys_context.current_state = SystemState::FAULT;
            }
            return 0;
        }
        return -1;
    }
}
