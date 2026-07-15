#include <zephyr/kvss/nvs.h>
#include <zephyr/logging/log.h>
#include <type_traits>
#include <cstdint>
#include <array>
#pragma once

// 5 mocked infusion motor devices occupy 5 fixed physical NVS "slots".
// Each slot holds three persisted fields:
//   - a Device ID (serial number), set once at provisioning/manufacturing
//   - an infusion rate (1-100), settable at runtime
//   - an alarm threshold (80-100), seeded with a default at provisioning
//
// Slot numbers (1-5) are an internal storage detail only. Callers address
// devices by Device ID (serial), not by slot -- use findSlotByDeviceId().
enum class ConfigKey : uint16_t {
    FULL_CHARGE_LOG        = 4,
    DEVICE_ID_BASE          = 100,  // + slot -> uint32_t serial number
    INFUSION_RATE_BASE     = 200,  // + slot -> uint8_t rate (1-100)
    ALARM_THRESHOLD_BASE   = 300   // + slot -> uint8_t threshold (80-100)
};

namespace InfusionDeviceConfig {
    constexpr uint8_t MinSlot = 1;
    constexpr uint8_t MaxSlot = 5;
    constexpr uint8_t NumSlots = 5;

    // Sentinel meaning "this slot has not been provisioned yet".
    constexpr uint32_t UnprovisionedId = 0;

    // Seeded once at first boot if a slot has no Device ID yet.
    // Replace with real serials at manufacturing time if desired.
    constexpr std::array<uint32_t, NumSlots> DefaultDeviceIds        = {1001, 1002, 1003, 1004, 1005};
    constexpr std::array<uint8_t,  NumSlots> DefaultAlarmThresholds  = {80, 85, 90, 95, 100};

    [[nodiscard]] constexpr bool isValidSlot(uint8_t slot) noexcept {
        return slot >= MinSlot && slot <= MaxSlot;
    }
}

class ConfigStore {
    private:
        struct nvs_fs fs;
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
        bool set(ConfigKey key,const T& value){
            static_assert(std::is_trivially_copyable<T>::value,"Config data must be trivially copyable");

            if (!initialized){
                return false;
            }

#if defined(CONFIG_NVS) || defined(IS_TEST_ENVIRONMENT)
            uint16_t nvs_id=static_cast<uint16_t>(key);
            ssize_t written=nvs_write(&fs,nvs_id,&value,sizeof(T));

            return (written>=0);
#else
            return true;
#endif
          }

          template <typename T>
          bool get(ConfigKey key,T& out_value){
              static_assert(std::is_trivially_copyable<T>::value,"Config data must be trivially copyable.");

              if (!initialized){
                  return false;
              }

#if defined(CONFIG_NVS) || defined(IS_TEST_ENVIRONMENT)
              uint16_t nvs_id=static_cast<uint16_t>(key);
              ssize_t bytes_read=nvs_read(&fs,nvs_id,&out_value,sizeof(T));

              return (bytes_read==sizeof(T));
#else
              return true;
#endif
           }

           bool validateEndurance(ConfigKey key);

           // --- Per-slot Device ID (serial number), set at provisioning ---
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

           // Scans all provisioned slots for a matching Device ID.
           // Returns true and sets out_slot if found.
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

           // --- Per-slot infusion rate (1-100) ---
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

           // --- Per-slot alarm threshold (80-100), seeded with defaults ---
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
};
