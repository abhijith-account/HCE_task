#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/pm/policy.h>
#include "Persistent_Configuration_System.h"
#include "Device_State_Machine+Watchdog.h" // For sys_context Watchdog feeding
#include "Power_Management_System.h"

LOG_MODULE_REGISTER(CONFIG_SYS, LOG_LEVEL_INF);

#define NVS_PARTITION storage_partition
#define NVS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(NVS_PARTITION)

extern DeviceContext sys_context;

ConfigStore ConfigStore::instance;

ConfigStore::ConfigStore() : initialized(false) {
    fs.flash_device = DEVICE_DT_GET(DT_MTD_FROM_FIXED_PARTITION(DT_NODELABEL(storage_partition)));
    fs.offset = DT_REG_ADDR(DT_NODELABEL(storage_partition));
    fs.sector_size = 2048;
    fs.sector_count = 4;
}

ConfigStore& ConfigStore::getInstance() {
    return instance;
}

bool ConfigStore::init() {
    if (!device_is_ready(fs.flash_device)){
        LOG_ERR("Flash device not ready");
        return false;
    }

    int rc = nvs_mount(&fs);
    if (rc){
        LOG_ERR("NVS Mount failed: %d", rc);
        return false;
    }

    initialized = true;
    LOG_INF("NVS Configuration Store Mounted Successfully.");

    seedDefaultDeviceIds();
    seedDefaultAlarmThresholds();

    return true;
}

void ConfigStore::seedDefaultDeviceIds() {
    for (uint8_t slot = InfusionDeviceConfig::MinSlot; slot <= InfusionDeviceConfig::MaxSlot; ++slot){
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

    LOG_WRN("Starting 1,000 Write-Cycle Endurance Test on Key:%d...", static_cast<int>(key));

    // Manually lock Deep Sleep for the entire duration of this extended test
    // to prevent the OS from attempting to sleep between individual writes.
    pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);

    uint16_t test_val = 0;
    uint16_t read_val = 0;
    bool success = true;

    for (int i = 0; i < 1000; i++){
        // Critical: Massive consecutive flash writes take enough time to trigger a hardware watchdog reset.
        sys_context.feedWatchdog();

        // Also critical: without this, PowerManager's own inactivity timer
        // (last_activity_time) goes stale during a long-running test, and its
        // FSM can independently decide to enter STOP mid-loop -- suspending
        // I2C and notifying every registered observer (halting the sensor
        // pipeline, shell, sync layer, BMS polling) even though this test's
        // own pm_policy_state_lock_get() above already prevents the hardware
        // from actually sleeping. feedWatchdog() only protects against a
        // *hardware* watchdog reset; this protects against a *software* STOP
        // transition -- they are not the same thing and one does not imply
        // the other.
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
    }

    pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
    
    PowerManager::getInstance().reportActivity();

    if (success) {
        LOG_INF("1,000 Write-Cycle Endurance Test passed. Flash is stable");
    }
    
    return success;
}
