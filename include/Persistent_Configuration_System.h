#pragma once

#include <zephyr/fs/fcb.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/policy.h>
#include "Power_Management_System.h"
#include <type_traits>
#include <cstdint>
#include <array>

enum class ConfigKey : uint16_t {
    FULL_CHARGE_LOG        = 4,
    DEVICE_ID_BASE         = 100,
    INFUSION_RATE_BASE     = 200,
    ALARM_THRESHOLD_BASE   = 300
};

namespace InfusionDeviceConfig {
    constexpr uint8_t MinSlot = 1;
    constexpr uint8_t MaxSlot = 5;
    constexpr uint8_t NumSlots = 5;

    constexpr uint32_t UnprovisionedId = 0;

    constexpr std::array<uint32_t, NumSlots> DefaultDeviceIds        = {1001, 1002, 1003, 1004, 1005};
    constexpr std::array<uint8_t,  NumSlots> DefaultAlarmThresholds  = {80, 85, 90, 95, 100};

    [[nodiscard]] constexpr bool isValidSlot(uint8_t slot) noexcept {
        return slot >= MinSlot && slot <= MaxSlot;
    }
}

class FlashPowerGuard {
public:
    FlashPowerGuard() {
        pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
    }
    ~FlashPowerGuard() {
        pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
    }
};

struct FcbRecordHeader {
    ConfigKey key;
    uint16_t length;
};

class ConfigStore {
    private:
        struct fcb fcb_instance;
        struct flash_sector fcb_sectors[2]; // Sectors 6 and 7
        bool initialized;

        static ConfigStore instance;

        ConfigStore();
        ~ConfigStore()=default;

        ConfigStore(const ConfigStore&)=delete;
        ConfigStore& operator=(const ConfigStore&)=delete;

        void seedDefaultDeviceIds();
        void seedDefaultAlarmThresholds();

    public:
        static ConfigStore& getInstance();

        bool init();

        template <typename T>
        bool set(ConfigKey key, const T& value) {
            static_assert(std::is_trivially_copyable<T>::value, "Config data must be trivially copyable");

            if (!initialized){
                return false;
            }

            FlashPowerGuard pm_guard;

            struct fcb_entry loc;
            int rc = fcb_append(&fcb_instance, sizeof(FcbRecordHeader) + sizeof(T), &loc);
            if (rc == -ENOSPC) {
                // Flash area full: rotate (erase oldest sector) and retry once.
                rc = fcb_rotate(&fcb_instance);
                if (rc != 0) {
                    return false;
                }
                rc = fcb_append(&fcb_instance, sizeof(FcbRecordHeader) + sizeof(T), &loc);
            }
            if (rc != 0) {
                return false;
            }

            FcbRecordHeader header = { key, sizeof(T) };
            rc = flash_area_write(fcb_instance.fap, FCB_ENTRY_FA_DATA_OFF(loc), &header, sizeof(header));
            if (rc != 0) return false;

            rc = flash_area_write(fcb_instance.fap, FCB_ENTRY_FA_DATA_OFF(loc) + sizeof(header), &value, sizeof(T));
            if (rc != 0) return false;

            fcb_append_finish(&fcb_instance, &loc);
            PowerManager::getInstance().reportActivity();
            return true;
        }

        template <typename T>
        bool get(ConfigKey key, T& out_value) {
            static_assert(std::is_trivially_copyable<T>::value, "Config data must be trivially copyable.");

            if (!initialized){
                return false;
            }

            struct fcb_entry loc;
            loc.fe_sector = nullptr;
            loc.fe_elem_off = 0;

            bool found = false;
            FcbRecordHeader header;
            T temp_val;

            // Walk through all records to find the most recent one for this key
            while (fcb_getnext(&fcb_instance, &loc) == 0) {
                flash_area_read(fcb_instance.fap, FCB_ENTRY_FA_DATA_OFF(loc), &header, sizeof(header));
                
                if (header.key == key && header.length == sizeof(T)) {
                    flash_area_read(fcb_instance.fap, FCB_ENTRY_FA_DATA_OFF(loc) + sizeof(header), &temp_val, sizeof(T));
                    found = true;
                }
            }

            if (found) {
                out_value = temp_val;
            }
            return found;
        }

        bool validateEndurance(ConfigKey key);

        [[nodiscard]] bool setDeviceId(uint8_t slot, uint32_t deviceId){
            if (!InfusionDeviceConfig::isValidSlot(slot)){
                return false;
            }
            return set(static_cast<ConfigKey>(static_cast<uint16_t>(ConfigKey::DEVICE_ID_BASE) + slot), deviceId);
        }

        [[nodiscard]] bool getDeviceId(uint8_t slot, uint32_t& out){
            if (!InfusionDeviceConfig::isValidSlot(slot)){
                return false;
            }
            return get(static_cast<ConfigKey>(static_cast<uint16_t>(ConfigKey::DEVICE_ID_BASE) + slot), out);
        }

        [[nodiscard]] bool findSlotByDeviceId(uint32_t deviceId, uint8_t& out_slot){
            for (uint8_t slot = InfusionDeviceConfig::MinSlot; slot <= InfusionDeviceConfig::MaxSlot; ++slot){
                uint32_t stored = InfusionDeviceConfig::UnprovisionedId;
                if (getDeviceId(slot, stored) && stored == deviceId){
                    out_slot = slot;
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] bool setInfusionRate(uint8_t slot, uint8_t rate){
            if (!InfusionDeviceConfig::isValidSlot(slot) || rate < 1 || rate > 100){
                return false;
            }
            return set(static_cast<ConfigKey>(static_cast<uint16_t>(ConfigKey::INFUSION_RATE_BASE) + slot), rate);
        }

        [[nodiscard]] bool getInfusionRate(uint8_t slot, uint8_t& out){
            if (!InfusionDeviceConfig::isValidSlot(slot)){
                return false;
            }
            return get(static_cast<ConfigKey>(static_cast<uint16_t>(ConfigKey::INFUSION_RATE_BASE) + slot), out);
        }

        [[nodiscard]] bool setAlarmThreshold(uint8_t slot, uint8_t threshold){
            if (!InfusionDeviceConfig::isValidSlot(slot) || threshold < 80 || threshold > 100){
                return false;
            }
            return set(static_cast<ConfigKey>(static_cast<uint16_t>(ConfigKey::ALARM_THRESHOLD_BASE) + slot), threshold);
        }

        [[nodiscard]] bool getAlarmThreshold(uint8_t slot, uint8_t& out){
            if (!InfusionDeviceConfig::isValidSlot(slot)){
                return false;
            }
            return get(static_cast<ConfigKey>(static_cast<uint16_t>(ConfigKey::ALARM_THRESHOLD_BASE) + slot), out);
        }
        
        struct LogEntry {
        uint32_t timestamp_ms;
        uint8_t event_id; // e.g., 0x01 = Battery Full, 0x02 = Fault, 0x03 = State Change
        int32_t event_data;
    };

    // New methods required for the USB Shell
    [[nodiscard]] uint32_t getStoredLogCount() const noexcept;
    [[nodiscard]] bool getLogEntry(uint32_t index, LogEntry& out_entry) const noexcept;
};

