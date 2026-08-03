#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/pm/policy.h>
#include "Persistent_Configuration_System.h"
#include "Device_State_Machine+Watchdog.h"
#include "Power_Management_System.h"

LOG_MODULE_REGISTER(CONFIG_SYS, LOG_LEVEL_INF);

#define NVS_PARTITION storage_partition
#define NVS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(NVS_PARTITION)

extern DeviceContext sys_context;
ConfigStore ConfigStore::instance;
ConfigStore::ConfigStore() : initialized(false) {}
ConfigStore& ConfigStore::getInstance() {
    return instance;
}

bool ConfigStore::init() {
    const struct flash_area *fap;
    
    int rc = flash_area_open(DT_FIXED_PARTITION_ID(DT_NODELABEL(storage_partition)), &fap);
    if (rc != 0) {
        LOG_ERR("Failed to open flash area: %d", rc);
        return false;
    }

    uint32_t sector_cnt = 2;
    rc = flash_area_get_sectors(DT_FIXED_PARTITION_ID(DT_NODELABEL(storage_partition)), &sector_cnt, fcb_sectors);
    if (rc != 0) {
        LOG_ERR("Failed to get flash sectors: %d", rc);
        return false;
    }

    fcb_instance.fap = fap;
    fcb_instance.f_magic = 0x12345678; 
    fcb_instance.f_erase_value = 0xff;
    fcb_instance.f_sectors = fcb_sectors;
    fcb_instance.f_sector_cnt = sector_cnt; 

    // Initial mount attempt
    rc = fcb_init(DT_FIXED_PARTITION_ID(DT_NODELABEL(storage_partition)), &fcb_instance);
    
    if (rc == -ENOMSG) {
        LOG_WRN("Flash storage is empty. Erasing and formatting FCB sectors...");

        // Feed watchdog before starting the erase sequence
        sys_context.feedWatchdog();

        // Erase the entire storage partition (Sectors 6 & 7) safely
        rc = flash_area_erase(fap, 0, fap->fa_size);
        if (rc != 0) {
            LOG_ERR("Failed to erase storage partition: %d", rc);
            return false;
        }

        // Feed watchdog again after the erase completes
        sys_context.feedWatchdog();

        // Re-initialize FCB on the freshly erased partition
        rc = fcb_init(DT_FIXED_PARTITION_ID(DT_NODELABEL(storage_partition)), &fcb_instance);
        if (rc != 0) {
            LOG_ERR("FCB re-init failed after formatting: %d", rc);
            return false;
        }
    } else if (rc != 0) {
        LOG_ERR("FCB Mount failed: %d", rc);
        return false;
    }

    initialized = true;
    LOG_INF("FCB Configuration Store Mounted Successfully.");

    seedDefaultDeviceIds();
    seedDefaultAlarmThresholds();

    return true;
}

void ConfigStore::seedDefaultDeviceIds() {
    for (uint8_t slot = InfusionDeviceConfig::MinSlot; slot <= InfusionDeviceConfig::MaxSlot; ++slot){
        sys_context.feedWatchdog();
        uint32_t existing = InfusionDeviceConfig::UnprovisionedId;
        if (!getDeviceId(slot, existing) || existing == InfusionDeviceConfig::UnprovisionedId){
            uint32_t defaultId = InfusionDeviceConfig::DefaultDeviceIds[slot - InfusionDeviceConfig::MinSlot];
            if (!setDeviceId(slot, defaultId)){
                LOG_WRN("Failed to provision default Device ID for slot %u", slot);
            } else {
                LOG_INF("Provisioned slot %u with Device ID %u", slot, defaultId);
            }
        }
    }
}

void ConfigStore::seedDefaultAlarmThresholds() {
    for (uint8_t slot = InfusionDeviceConfig::MinSlot; slot <= InfusionDeviceConfig::MaxSlot; ++slot){
        sys_context.feedWatchdog();
        uint8_t existing = 0;
        if (!getAlarmThreshold(slot, existing)){
            uint8_t defaultVal = InfusionDeviceConfig::DefaultAlarmThresholds[slot - InfusionDeviceConfig::MinSlot];
            if (!setAlarmThreshold(slot, defaultVal)){
                LOG_WRN("Failed to seed default alarm threshold for slot %u", slot);
            } else {
                LOG_INF("Seeded slot %u alarm threshold = %u", slot, defaultVal);
            }
        }
    }
}

bool ConfigStore::validateEndurance(ConfigKey key) {
    if (!initialized){
        return false;
    }
    
    uint16_t read_val = 0;
    if (get(key, read_val) && read_val == 999) {
        LOG_INF("Endurance test already completed on a previous boot. Skipping to save flash wear.");
        return true;
    }
    
    LOG_WRN("Starting 1,000 Write-Cycle Endurance Test on Key:%d...", static_cast<int>(key));
    pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
    uint16_t test_val = 0;
    bool success = true;
    for (int i = 0; i < 1000; i++){

        sys_context.feedWatchdog();
        PowerManager::getInstance().reportActivity();
        test_val = i;

        if (!set(key, test_val)){
            LOG_ERR("Endurance Test failed to write at cycle %d", i);
            success = false;
            break;
        }

        if (!get(key, read_val) || read_val != test_val){
            LOG_ERR("Endurance Test failed to verify at cycle %d", i);
            success = false;
            break;
        }
        
        if (i > 0 && i % 100 == 0) {
            LOG_INF("Endurance Test Progress: %d/1000...", i);
        }
    }

    pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
    PowerManager::getInstance().reportActivity();

    if (success) {
        LOG_INF("1,000 Write-Cycle Endurance Test passed. Flash is stable");
    }

    return success;
}

uint32_t ConfigStore::getStoredLogCount() const noexcept {
    if (!initialized) {
        return 0;
    }
    
    struct fcb_entry loc;
    loc.fe_sector = nullptr;
    loc.fe_elem_off = 0;
    
    uint32_t count = 0;
    FcbRecordHeader header;
    
    // fcb_getnext requires a non-const fcb_instance pointer
    auto* non_const_fcb = const_cast<struct fcb*>(&fcb_instance);
    
    while (fcb_getnext(non_const_fcb, &loc) == 0) {
        if (flash_area_read(fcb_instance.fap, FCB_ENTRY_FA_DATA_OFF(loc), &header, sizeof(header)) == 0) {
            // Count entries that are specifically logs
            if (header.key == ConfigKey::FULL_CHARGE_LOG && header.length == sizeof(LogEntry)) {
                count++;
            }
        }
    }
    return count;
}

bool ConfigStore::getLogEntry(uint32_t index, LogEntry& out_entry) const noexcept {
    if (!initialized) {
        return false;
    }
    
    struct fcb_entry loc;
    loc.fe_sector = nullptr;
    loc.fe_elem_off = 0;
    
    uint32_t current_index = 0;
    FcbRecordHeader header;
    auto* non_const_fcb = const_cast<struct fcb*>(&fcb_instance);
    
    while (fcb_getnext(non_const_fcb, &loc) == 0) {
        if (flash_area_read(fcb_instance.fap, FCB_ENTRY_FA_DATA_OFF(loc), &header, sizeof(header)) == 0) {
            if (header.key == ConfigKey::FULL_CHARGE_LOG && header.length == sizeof(LogEntry)) {
                if (current_index == index) {
                    // Target index reached; read the payload
                    int rc = flash_area_read(fcb_instance.fap, 
                                             FCB_ENTRY_FA_DATA_OFF(loc) + sizeof(header), 
                                             &out_entry, sizeof(LogEntry));
                    return (rc == 0);
                }
                current_index++;
            }
        }
    }
    return false;
}
